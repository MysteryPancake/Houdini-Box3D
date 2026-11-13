// Box3DPointSync.cpp
//
// One body per point: reading points, the change fingerprint, body settings,
// shape plans and syncObject(), which adds, updates and removes an object's
// bodies.
#include "Box3DSolverInternal.h"

namespace box3dsolver {

// ---------------------------------------------------------------------------
// Point state
// ---------------------------------------------------------------------------

// P / rotation / v / w of one point (a packed point's rotation from its
// primitive). False if P is not finite. A non-finite v or w is read as zero
// (Box3D asserts on it, and a NaN velocity would spread to the body's
// position); *badVelocity says so.
bool readPointState(const GU_Detail *gdp, const PointAttrs &a, GA_Offset off,
                    const PackedInfo &pk, Box3DBodyPose &out,
                    bool *badVelocity) {
  if (badVelocity)
    *badVelocity = false;
  const UT_Vector3 p = gdp->getPos3(off);
  out.pos[0] = p.x();
  out.pos[1] = p.y();
  out.pos[2] = p.z();
  if (!std::isfinite(out.pos[0]) || !std::isfinite(out.pos[1]) ||
      !std::isfinite(out.pos[2]))
    return false;

  out.rot[0] = out.rot[1] = out.rot[2] = 0.0f;
  out.rot[3] = 1.0f;
  if (pk.prim) {
    b3dop::copy4(out.rot, pk.rot);
    b3dop::sanitizeQuat(out.rot);
  } else if (a.orient.isValid()) {
    const UT_QuaternionF q = a.orient.get(off);
    out.rot[0] = q.x();
    out.rot[1] = q.y();
    out.rot[2] = q.z();
    out.rot[3] = q.w();
    b3dop::sanitizeQuat(out.rot);
  }

  out.lin[0] = out.lin[1] = out.lin[2] = 0.0f;
  out.ang[0] = out.ang[1] = out.ang[2] = 0.0f;
  if (a.v.isValid()) {
    const UT_Vector3 v = a.v.get(off);
    if (b3dop::finite3(v.data())) {
      out.lin[0] = v.x();
      out.lin[1] = v.y();
      out.lin[2] = v.z();
    } else if (badVelocity)
      *badVelocity = true;
  }
  if (a.w.isValid()) {
    const UT_Vector3 w = a.w.get(off);
    if (b3dop::finite3(w.data())) {
      out.ang[0] = w.x();
      out.ang[1] = w.y();
      out.ang[2] = w.z();
    } else if (badVelocity)
      *badVelocity = true;
  }
  return true;
}

// Position, orientation or velocity differ beyond float noise (q and -q are the
// same rotation). The quaternions are compared component-wise after aligning
// signs: a dot-product test would only see rotations above ~0.16 degrees (dot =
// cos(theta / 2)).
bool poseDiffers(const Box3DBodyPose &a, const Box3DBodyPose &b) {
  const double eps = 1e-5;
  if (b3dop::quatsDiffer(a.rot, b.rot, eps))
    return true;
  for (int i = 0; i < 3; ++i)
    if (std::fabs(a.pos[i] - b.pos[i]) > eps ||
        std::fabs(a.lin[i] - b.lin[i]) > eps ||
        std::fabs(a.ang[i] - b.ang[i]) > eps)
      return true;
  return false;
}

// The primitive list's data id: the marker for packed transforms (intrinsics
// have no attribute id). Bumping it is up to whoever edits a primitive; an edit
// that does not is picked up on the next scan instead.
GA_DataId primitiveListDataId(const GU_Detail *gdp) {
  return gdp->getNumPrimitives() > 0 ? gdp->getPrimitiveList().getDataId()
                                     : Box3DSyncFingerprint::kNoId;
}

// The object's change markers (Box3DSyncFingerprint). `a` must be bound on
// `gdp`.
Box3DSyncFingerprint makeFingerprint(const GU_Detail *gdp, const PointAttrs &a,
                                     int nodeType,
                                     const b3dop::BodyProps &nodeProps,
                                     bool addImpact) {
  // Packed geometry is assumed to keep its topology (a new instance gets a new
  // `id`, which the id / P markers show); plain points are checked for count
  // changes.
  const bool trustTopology = gdp->getNumPrimitives() > 0;
  auto idOf = [](const auto &h) {
    return h.isValid() ? h.getAttribute()->getDataId()
                       : Box3DSyncFingerprint::kNoId;
  };
  Box3DSyncFingerprint fp;
  fp.numPoints = trustTopology ? -1 : static_cast<int64_t>(gdp->getNumPoints());
  fp.numOffsets =
      trustTopology ? -1 : static_cast<int64_t>(gdp->getNumPointOffsets());
  const GA_Attribute *P = gdp->getP();
  fp.pId = P ? P->getDataId() : Box3DSyncFingerprint::kNoId;
  fp.orientId = idOf(a.orient);
  fp.vId = idOf(a.v);
  fp.wId = idOf(a.w);
  fp.idId = idOf(a.id);
  fp.activeId = idOf(a.active);
  fp.nodeType = nodeType;
  // By name: `a` has only the sync-critical handles bound at this point.
  for (int i = 0; i < Box3DSyncFingerprint::kNumPropIds; ++i) {
    const GA_Attribute *attr = gdp->findPointAttribute(kPropAttribNames[i]);
    fp.propIds[i] = attr ? attr->getDataId() : Box3DSyncFingerprint::kNoId;
  }
  fp.primsId = primitiveListDataId(gdp);
  fp.nodeProps = nodeProps;
  fp.addImpact = addImpact;
  return fp;
}

#ifdef BOX3DDOP_DEBUG
// Markers that differ, old -> new, for the debug log.
std::string describeFingerprintDifference(const Box3DSyncFingerprint &was,
                                          const Box3DSyncFingerprint &now) {
  std::string out;
  auto add = [&](const char *name, long long a, long long b) {
    if (a == b)
      return;
    char buf[96];
    std::snprintf(buf, sizeof(buf), "%s%s %lld->%lld", out.empty() ? "" : ", ",
                  name, a, b);
    out += buf;
  };
  add("point count", was.numPoints, now.numPoints);
  add("offset range", was.numOffsets, now.numOffsets);
  add("P", (long long)was.pId, (long long)now.pId);
  add("orient", (long long)was.orientId, (long long)now.orientId);
  add("v", (long long)was.vId, (long long)now.vId);
  add("w", (long long)was.wId, (long long)now.wId);
  add("id", (long long)was.idId, (long long)now.idId);
  add("active", (long long)was.activeId, (long long)now.activeId);
  add("body type", was.nodeType, now.nodeType);
  for (int i = 0; i < Box3DSyncFingerprint::kNumPropIds; ++i)
    add(kPropAttribNames[i], (long long)was.propIds[i],
        (long long)now.propIds[i]);
  add("primitives", (long long)was.primsId, (long long)now.primsId);
  if (was.nodeProps != now.nodeProps || was.addImpact != now.addImpact)
    out += out.empty() ? "Box3D Data values" : ", Box3D Data values";
  return out;
}
#endif

// ---------------------------------------------------------------------------
// Body settings, shape planning, and mid-simulation updates
// ---------------------------------------------------------------------------

// Box3D asserts on negatives: a negative value is clamped to 0, a non-finite
// one becomes `fallback`.
float clampNonNeg(float v, float fallback) {
  return std::isfinite(v) ? std::max(v, 0.0f) : fallback;
}

// A point's changeable settings: Box3D Data's, with the point's attributes on
// top. Building and comparing both read through here, so an untouched point
// compares equal. (`awake` is not a setting: see startAwakeFor.)
b3dop::BodyProps readDynamicProps(const b3dop::BodySettings &node,
                                  const PointAttrs &a, GA_Offset off) {
  b3dop::BodyProps p = node.props;
  if (a.mass.isValid()) {
    const float m = static_cast<float>(a.mass.get(off));
    if (m > 0.0f && std::isfinite(m))
      p.mass = m;
  }
  if (a.friction.isValid())
    p.friction = clampNonNeg(a.friction.get(off), p.friction);
  if (a.bounce.isValid())
    p.restitution = clampNonNeg(a.bounce.get(off), p.restitution);
  if (a.rollingResistance.isValid())
    p.rollingResistance =
        clampNonNeg(a.rollingResistance.get(off), p.rollingResistance);
  if (a.tangentVelocity.isValid()) {
    const UT_Vector3 t = a.tangentVelocity.get(off);
    if (std::isfinite(t.x()) && std::isfinite(t.y()) && std::isfinite(t.z()))
      for (int i = 0; i < 3; ++i)
        p.tangentVelocity[i] = static_cast<float>(t(i));
  }
  if (a.lineardamping.isValid())
    p.linearDamping = clampNonNeg(a.lineardamping.get(off), p.linearDamping);
  if (a.angulardamping.isValid())
    p.angularDamping = clampNonNeg(a.angulardamping.get(off), p.angularDamping);
  if (a.gravityscale.isValid()) {
    const float g = a.gravityscale.get(off);
    if (std::isfinite(g))
      p.gravityScale = g; // may be negative
  }
  if (a.enablesleep.isValid())
    p.enableSleep = a.enablesleep.get(off) != 0;
  if (a.enabled.isValid())
    p.enabled = a.enabled.get(off) != 0;

  if (a.computeMass.isValid())
    p.computeMass = a.computeMass.get(off) != 0;
  if (a.density.isValid()) {
    const float d = a.density.get(off);
    if (d > 0.0f && std::isfinite(d))
      p.density = d;
  }
  if (a.inertiaStiffness.isValid()) {
    const float k = a.inertiaStiffness.get(off);
    if (k > 0.0f && std::isfinite(k))
      p.inertiaScale = k;
  }
  auto threshold = [&](const GA_ROHandleF &h) {
    if (!h.isValid())
      return -1.0f;
    const float v = h.get(off);
    return (v >= 0.0f && std::isfinite(v)) ? v : -1.0f;
  };
  if (a.sleepThreshold.isValid()) {
    const float t = threshold(a.sleepThreshold);
    if (t >= 0.0f)
      p.sleepThreshold = t;
  }
  if (a.ccdThreshold.isValid()) {
    const float t = threshold(a.ccdThreshold);
    if (t >= 0.0f)
      p.ccdThreshold = t;
  }
  if (a.fastObject.isValid())
    p.fastObject = a.fastObject.get(off) != 0;
  if (a.fastRotation.isValid())
    p.fastRotation = a.fastRotation.get(off) != 0;
  if (a.contactRecycling.isValid())
    p.contactRecycling = a.contactRecycling.get(off) != 0;
  if (a.collisionCategory.isValid())
    p.filter.category = static_cast<uint64_t>(a.collisionCategory.get(off));
  if (a.collisionMask.isValid())
    p.filter.mask = static_cast<uint64_t>(a.collisionMask.get(off));
  if (a.collisionGroup.isValid())
    p.filter.group = a.collisionGroup.get(off);
  if (a.lockLinear.isValid()) {
    const UT_Vector3 l = a.lockLinear.get(off);
    for (int i = 0; i < 3; ++i)
      p.locks.linear[i] = l(i) != 0.0f;
  }
  if (a.lockAngular.isValid()) {
    const UT_Vector3 l = a.lockAngular.get(off);
    for (int i = 0; i < 3; ++i)
      p.locks.angular[i] = l(i) != 0.0f;
  }
  return p;
}

// A string attribute read through its shared-string handles where it has them,
// so per-point work can be memoised by handle (an int) rather than by string.
struct StringAttr {
  const GA_ROHandleS *handle = nullptr;
  const GA_Attribute *attr = nullptr;
  const GA_AIFSharedStringTuple *aif = nullptr;

  void bind(const GA_ROHandleS &h) {
    handle = &h;
    attr = h.isValid() ? h.getAttribute() : nullptr;
    aif = attr ? attr->getAIFSharedStringTuple() : nullptr;
  }
  bool valid() const { return attr != nullptr; }
  bool memoisable() const { return !attr || aif; }

  // The point's string handle; -1 for none (empty, or no attribute). Only with
  // memoisable().
  int64_t id(GA_Offset off) const {
    if (!attr)
      return -1;
    const GA_StringIndexType h = aif->getHandle(attr, off, 0);
    return h == GA_INVALID_STRING_INDEX ? -1 : static_cast<int64_t>(h);
  }
  std::string str(GA_Offset off) const {
    if (!attr)
      return std::string();
    const char *v = handle->get(off);
    return v ? std::string(v) : std::string();
  }
};

// Box3D density: computemass -> density; else mass / volume (density if
// unusable).
float boxDensity(const b3dop::BodyProps &p, float volume) {
  if (p.computeMass)
    return p.density;
  return (p.mass > 0.0f && std::isfinite(p.mass) && volume > 0.0f)
             ? p.mass / volume
             : p.density;
}

// The mass Houdini reports (written to `mass` with computemass).
float effectiveMass(const b3dop::BodyProps &p, float volume) {
  return p.computeMass ? p.density * volume : p.mass;
}

// active -> the node's type; inactive -> static, or kinematic if animated;
// shapeless -> static.
b3dop::BodySettings::Type wantedType(const b3dop::BodySettings &node,
                                     bool active, bool animated,
                                     bool shapeless) {
  if (shapeless)
    return b3dop::BodySettings::Type::Static;
  if (active)
    return node.type;
  return animated ? b3dop::BodySettings::Type::Kinematic
                  : b3dop::BodySettings::Type::Static;
}

// The body's mass after shapes are added / replaced or its type changes. As in
// Houdini RBD the point (pivot) is the centre of mass, so the shapes' centroid
// is moved there; inertia is scaled by inertialtensorstiffness. Static /
// kinematic bodies only get shape densities. setShapeDensity = false for shapes
// just created with boxDensity(p, volume).
void applyBodyMass(b3dop::Box3DWorld &world, b3dop::BodyHandle h,
                   const b3dop::BodyProps &p, float volume,
                   bool setShapeDensity) {
  world.applyMassByHandle(h, boxDensity(p, volume), true, p.inertiaScale,
                          setShapeDensity);
}

// `awake` for a new body: the point's value, else the Box3D Data default.
// (Box3D starts a body that may not sleep awake whatever this says.)
bool startAwakeFor(const b3dop::BodySettings &node, const PointAttrs &a,
                   GA_Offset off) {
  return a.awake.isValid() ? a.awake.get(off) != 0 : node.startAwake;
}

// Point attributes that replace a Box3D Data value on an object with a Box3D
// Data node. RBD Packed Object writes several of them onto every point from its
// own parameters (mass, density, computemass, friction, bounce in Houdini
// 22.0), so its Box3D Data values for those do nothing; this makes that
// visible. Reported once per object: on its first full scan, and again only if
// the set changes. (`active` is left out: it only takes over Body Type when 0.)
void noteOverriddenData(const GU_Detail *gdp, Box3DObjectSync &os,
                        SyncCounts &c) {
  static const char *const kOverriding[] = {"mass",
                                            "density",
                                            "computemass",
                                            "friction",
                                            "bounce",
                                            "rollingresistance",
                                            "tangentvelocity",
                                            "lineardamping",
                                            "angulardamping",
                                            "gravityscale",
                                            "enablesleep",
                                            "awake",
                                            "sleepthreshold",
                                            "ccdthreshold",
                                            "fastobject",
                                            "contactrecycling",
                                            "bullet_georep",
                                            "bullet_autofit"};
  std::string found;
  for (const char *name : kOverriding)
    if (gdp->findPointAttribute(name)) {
      found += found.empty() ? "" : " ";
      found += name;
    }
  if (found == os.overridesReported)
    return;
  os.overridesReported = found;
  if (found.empty())
    return;
  ++c.overriddenObjects;
  for (const char *name : kOverriding)
    if (gdp->findPointAttribute(name))
      ++c.overriddenData[name];
}

// pscale / scale (a packed point's size is its transform; these only size its
// fallback box).
void readPscaleScale(const PointAttrs &a, GA_Offset off, float &ps,
                     float sc[3]) {
  ps = a.pscale.isValid() ? static_cast<float>(a.pscale.get(off)) : 1.0f;
  sc[0] = sc[1] = sc[2] = 1.0f;
  if (a.scale.isValid()) {
    const UT_Vector3 s = a.scale.get(off);
    sc[0] = s.x();
    sc[1] = s.y();
    sc[2] = s.z();
  }
}

// Within 1e-4 relative (our write-back recomposes packed transforms with float
// noise).
bool sameSizeKey(const double a[3], const double b[3]) {
  for (int i = 0; i < 3; ++i) {
    const double tol = 1e-4 * std::max(std::fabs(a[i]), std::fabs(b[i]));
    if (!(std::fabs(a[i] - b[i]) <= tol))
      return false;
  }
  return true;
}

// The scale convex / concave shapes depend on: packed row lengths, else pscale
// * scale.
bool sizeKey(bool packed, const double m3[9], float ps, const float sc[3],
             double out[3]) {
  if (packed)
    return b3dop::matrixRowLengths(m3, out);
  for (int i = 0; i < 3; ++i)
    out[i] = static_cast<double>(ps) * static_cast<double>(sc[i]);
  return true;
}

// Equal signatures mean identical shapes (hull keys, primitive numbers): skip
// the rebuild.
uint64_t shapeSignature(b3dop::ShapeType type, const BuiltShapes &s) {
  uint64_t h = b3dop::mixHash(
      0x9e3779b97f4a7c15ull, static_cast<uint64_t>(static_cast<int>(type)) + 1);
  for (const Piece &pc : s.pieces) {
    if (pc.isHull)
      h = b3dop::mixHash(b3dop::mixHash(h, 1), pc.hull);
    else {
      const b3dop::ShapeSpec &sp = pc.spec;
      const float f[] = {static_cast<float>(static_cast<int>(sp.type)),
                         sp.halfExtents[0],
                         sp.halfExtents[1],
                         sp.halfExtents[2],
                         sp.radius,
                         sp.halfLength,
                         sp.center[0],
                         sp.center[1],
                         sp.center[2],
                         sp.rot[0],
                         sp.rot[1],
                         sp.rot[2],
                         sp.rot[3]};
      h = b3dop::mixHash(b3dop::mixHash(h, 2),
                         b3dop::hashFloats(f, sizeof(f) / sizeof(f[0])));
    }
  }
  return h ? h : 1;
}

// The explicit primitive attributes (used when there is no packed geometry to
// fit).
b3dop::PrimOverrides readPrimOverrides(const PointAttrs &a, GA_Offset off) {
  b3dop::PrimOverrides ov;
  if (a.primS.isValid()) {
    const UT_Vector3 v = a.primS.get(off);
    ov.hasSize = true;
    ov.size[0] = v.x();
    ov.size[1] = v.y();
    ov.size[2] = v.z();
  }
  if (a.primR.isValid()) {
    const UT_Vector3 v = a.primR.get(off);
    // Euler degrees, Houdini's XYZ rotate order. Non-finite: no override.
    if (SYSisFinite(v.x()) && SYSisFinite(v.y()) && SYSisFinite(v.z())) {
      const float deg[3] = {v.x(), v.y(), v.z()};
      box3dEulerToQuat(deg, ov.rot);
      ov.hasRot = true;
    }
  }
  if (a.primT.isValid()) {
    const UT_Vector3 v = a.primT.get(off);
    ov.hasPos = true;
    ov.pos[0] = v.x();
    ov.pos[1] = v.y();
    ov.pos[2] = v.z();
  }
  if (a.radius.isValid()) {
    ov.hasRadius = true;
    ov.radius = a.radius.get(off);
  }
  if (a.length.isValid()) {
    ov.hasLength = true;
    ov.length = a.length.get(off);
  }
  return ov;
}

// A point's shapes from its current attributes and packed geometry. Only the
// hull cache is touched, no body.
ShapePlan planShapes(ScanContext &ctx, GA_Offset off, int64_t bodyId,
                     const PackedInfo &pk) {
  const PointAttrs &a = ctx.a;
  ShapePlan plan;

  plan.type = ctx.node.defaultShape;
  ctx.georep.get(a, off, plan.type); // else the node's default

  // `none`: a static, shapeless body (like Bullet, it still exists for
  // constraints).
  if (plan.type == b3dop::ShapeType::None) {
    plan.sig = shapeSignature(plan.type, plan.shapes);
    return plan;
  }

  float ps, sc[3];
  readPscaleScale(a, off, ps, sc);
  const bool autofit =
      a.autofit.isValid() ? (a.autofit.get(off) != 0) : ctx.node.autoFit;
  plan.shapes = buildShapes(ctx.world, ctx.sources, plan.type,
                            ctx.node.hullPerConnectedSet, ps, sc, pk, autofit,
                            readPrimOverrides(a, off));
  if (plan.shapes.note)
    B3D_LOG("  body id=%lld: %s", (long long)bodyId, plan.shapes.note);
  plan.sig = shapeSignature(plan.type, plan.shapes);

  // Convex / concave follow scale changes (as in Houdini); primitives do not.
  plan.autoScale = (plan.type == b3dop::ShapeType::Convex ||
                    plan.type == b3dop::ShapeType::Concave) &&
                   sizeKey(pk.prim != nullptr, pk.m3, ps, sc, plan.sizeKey);
  return plan;
}

// Adds the pieces without mass updates (the caller calls applyBodyMass).
// Returns the count.
int addShapes(b3dop::Box3DWorld &world, b3dop::BodyHandle h,
              const b3dop::BodySettings &b, const BuiltShapes &shapes) {
  int added = 0;
  for (const Piece &pc : shapes.pieces)
    added += (pc.isHull ? world.createHullShape(h, b, pc.hull, false)
                        : world.createShape(h, b, pc.spec, false))
                 ? 1
                 : 0;
  return added;
}

// A body with settings `b` at `pose` (its user data `entry`), its `shapes` and
// mass, then its initial velocity when `giveVelocity`: after the mass is final
// (a moving centre of mass shifts the velocity), and not for a static body or
// one that starts asleep or disabled (a velocity would wake it, so `v` only
// takes effect once something wakes it). A shapeless body (`none`) gets no
// shapes. 0 on failure, leaving no body behind.
b3dop::BodyHandle
createBodyWithShapes(b3dop::Box3DWorld &world, const b3dop::BodySettings &b,
                     const Box3DBodyPose &pose, const BuiltShapes &shapes,
                     bool shapeless, const b3dop::BodyProps &p,
                     bool giveVelocity, Box3DBodyMirror &entry) {
  const b3dop::BodyHandle h = world.createBody(b, pose.pos, pose.rot, &entry);
  if (!h || shapeless)
    return h;
  if (addShapes(world, h, b, shapes) == 0) {
    world.destroyBody(h); // never a shapeless body, except `none`
    return 0;
  }
  applyBodyMass(world, h, p, shapes.totalVolume, false);
  const bool hasVel =
      !std::all_of(pose.lin, pose.lin + 3, [](float v) { return v == 0.0f; }) ||
      !std::all_of(pose.ang, pose.ang + 3, [](float v) { return v == 0.0f; });
  if (giveVelocity && hasVel && b.type != b3dop::BodySettings::Type::Static &&
      world.isAwakeByHandle(h))
    world.setVelocityByHandle(h, pose.lin, pose.ang);
  return h;
}

// What the body was built from, into its mirror entry.
void storePlan(Box3DBodyMirror &m, const ShapePlan &plan) {
  m.shapeSig = plan.sig;
  m.volume = plan.shapes.totalVolume;
  m.autoScale = plan.autoScale;
  b3dop::copy3(m.sizeKey, plan.sizeKey);
}

// The slot's plan from the parallel pre-pass (moved out), else made now.
ShapePlan takePlan(ScanContext &ctx, const Box3DPointSlot &s,
                   std::vector<ShapePlan> &plans) {
  if (s.plan >= 0)
    return std::move(plans[static_cast<size_t>(s.plan)]);
  return planShapes(ctx, s.off, s.bid, ctx.packedInfo(s.off));
}

// Create the body for slot `s` (settings `p`), its shapes and initial velocity.
// `entry` becomes the body's user data and records what it was built from.
// Returns 0 on failure.
b3dop::BodyHandle addPointBody(ScanContext &ctx, const Box3DPointSlot &s,
                               const ShapePlan &plan, bool animated,
                               const b3dop::BodyProps &p,
                               Box3DBodyMirror &entry,
                               std::string &fallbackNote) {
  b3dop::Box3DWorld &world = ctx.world;
  const Box3DBodyPose &st = s.cur;
  if (plan.shapes.note)
    fallbackNote = plan.shapes.note;
  entry.props = p;
  entry.shapeless = (plan.type == b3dop::ShapeType::None);
  entry.animated = animated;
  storePlan(entry, plan);

  b3dop::BodySettings b = ctx.node;
  b.type = wantedType(ctx.node, s.active, animated, entry.shapeless);
  b.props = p;
  b.props.density = boxDensity(p, plan.shapes.totalVolume);
  b.startAwake = startAwakeFor(ctx.node, ctx.a, s.off);
  entry.awake = b.startAwake;

  // Animated bodies follow their point instead of starting with its velocity.
  return createBodyWithShapes(world, b, st, plan.shapes, entry.shapeless, p,
                              !animated, entry);
}

// Rebuild a body's shapes from current attributes (deforming,
// bullet_autofit_valid = 0, or a convex / concave scale change), keeping the
// body. Identical shapes: nothing happens. Returns true if replaced; `noShape`
// if none could be added (the body is then static).
bool rebuildPointShapes(ScanContext &ctx, const Box3DPointSlot &s,
                        const ShapePlan &plan, const b3dop::BodyProps &p,
                        Box3DBodyMirror &entry, std::string &note,
                        bool &noShape) {
  b3dop::Box3DWorld &world = ctx.world;
  noShape = false;
  if (plan.shapes.note)
    note = plan.shapes.note;
  if (plan.sig == entry.shapeSig) {
    storePlan(entry, plan); // same shapes; refresh the size key
    return false;
  }

  const b3dop::BodyHandle h = entry.handle;
  world.destroyShapesByHandle(h);
  storePlan(entry, plan);
  entry.shapeless = true;
  if (plan.type != b3dop::ShapeType::None) {
    b3dop::BodySettings b = ctx.node;
    b.type = entry.type;
    b.props = p;
    b.props.density = boxDensity(p, plan.shapes.totalVolume);
    if (addShapes(world, h, b, plan.shapes) > 0)
      entry.shapeless = false;
    else
      noShape = true;
  }
  applyBodyMass(world, h, p, plan.shapes.totalVolume,
                false);  // even with no shapes: clears Box3D's dirty mass
  world.wakeByHandle(h); // let it re-settle
  B3D_LOG_VERBOSE("  body id=%lld: collision shape rebuilt", (long long)s.bid);
  return true;
}

// Push the settings that differ. After a rebuild the new shapes already carry
// material, mass and filter profile. Returns kPropsChanged / kMassChanged.
enum : unsigned { kPropsChanged = 1, kMassChanged = 2 };

unsigned applyPropChanges(b3dop::Box3DWorld &world, Box3DBodyMirror &e,
                          const b3dop::BodyProps &p, bool shapesRebuilt) {
  const b3dop::BodyProps &o = e.props;
  if (!shapesRebuilt && p == o)
    // nothing differs (the field compares below would call setters for a NaN)
    return 0;
  const b3dop::BodyHandle h = e.handle;
  unsigned changed = shapesRebuilt ? static_cast<unsigned>(kPropsChanged) : 0u;
  // Runs `set` and records a props change when `differs`.
  auto apply = [&](bool differs, auto &&set) {
    if (differs) {
      set();
      changed |= kPropsChanged;
    }
  };
  const bool massChanged = p.mass != o.mass || p.computeMass != o.computeMass ||
                           p.density != o.density ||
                           p.inertiaScale != o.inertiaScale;
  if (!shapesRebuilt) {
    apply(p.friction != o.friction || p.restitution != o.restitution,
          [&] { world.setMaterialByHandle(h, p.friction, p.restitution); });
    if (massChanged) {
      applyBodyMass(world, h, p, e.volume);
      changed |= kPropsChanged | kMassChanged;
    }
    apply(p.filter != o.filter, [&] { world.setFilterByHandle(h, p.filter); });
  } else if (massChanged)
    changed |= kMassChanged;
  apply(
      p.linearDamping != o.linearDamping ||
          p.angularDamping != o.angularDamping,
      [&] { world.setDampingByHandle(h, p.linearDamping, p.angularDamping); });
  apply(p.gravityScale != o.gravityScale,
        [&] { world.setGravityScaleByHandle(h, p.gravityScale); });
  apply(p.ccdThreshold != o.ccdThreshold,
        [&] { world.setCcdThresholdByHandle(h, p.ccdThreshold); });
  apply(p.sleepThreshold != o.sleepThreshold,
        [&] { world.setSleepThresholdByHandle(h, p.sleepThreshold); });
  apply(p.fastRotation != o.fastRotation,
        [&] { world.setFastRotationByHandle(h, p.fastRotation); });
  apply(p.fastObject != o.fastObject,
        [&] { world.setBulletByHandle(h, p.fastObject); });
  apply(p.rollingResistance != o.rollingResistance ||
            !std::equal(p.tangentVelocity, p.tangentVelocity + 3,
                        o.tangentVelocity),
        [&] {
          world.setSurfaceByHandle(h, p.rollingResistance, p.tangentVelocity);
        });
  apply(p.locks != o.locks, [&] { world.setMotionLocksByHandle(h, p.locks); });
  apply(p.contactRecycling != o.contactRecycling,
        [&] { world.setContactRecyclingByHandle(h, p.contactRecycling); });
  apply(p.enableSleep != o.enableSleep,
        [&] { world.setSleepEnabledByHandle(h, p.enableSleep); });
  apply(p.enabled != o.enabled, [&] {
    world.setEnabledByHandle(h, p.enabled);
    if (p.enabled)
      // Box3D enables a body awake; the awake check after this applies a 0
      e.awake = true;
  });
  e.props = p;
  return changed;
}

// Queue a point attribute write for after the step.
void queuePointWrite(Box3DSharedState &st, Box3DBodyMirror &m, unsigned flags,
                     float mass) {
  st.pointWrites.emplace_back();
  Box3DPointWrite &w = st.pointWrites.back();
  w.entry = &m;
  w.objectId = m.objectId;
  w.flags = flags;
  w.mass = mass;
}

// The point's packed primitive, as pass 1 read it, for the write-back.
void storePacked(Box3DBodyMirror &m, const Box3DPointSlot &s) {
  m.packed = s.packed;
  m.primOff = s.primOff;
  if (s.packed) {
    std::copy(s.m3, s.m3 + 9, m.m3);
    std::copy(s.rowScale, s.rowScale + 3, m.rowScale);
  }
}

// The mid-simulation changes of an existing body (docs/agents/bodies.md,
// "Mid-simulation updates"): shape rebuild and settings, type, then the
// animated target or an external edit of its pose. Adds to `writes` the point
// attributes to write after the step.
void updateExistingBody(ScanContext &ctx, Box3DSharedState &st,
                        Box3DPointSlot &s, Box3DBodyMirror &entry,
                        bool animated, bool checkProps,
                        std::vector<ShapePlan> &plans, SyncCounts &c,
                        unsigned &writes, bool &anyDriven) {
  b3dop::Box3DWorld &world = ctx.world;
  const PointAttrs &a = ctx.a;
  const GA_Offset off = s.off;

  // Shape first (a shapeless body is static); pass 1 decided whether to
  // rebuild.
  const bool rebuild = s.rebuild;
  if (rebuild || checkProps) {
    b3dop::BodyProps p = ctx.props(off);
    bool shapesRebuilt = false;
    if (rebuild) {
      std::string note;
      bool noShape = false;
      shapesRebuilt = rebuildPointShapes(ctx, s, takePlan(ctx, s, plans), p,
                                         entry, note, noShape);
      if (shapesRebuilt) {
        ++c.rebuilt;
        if (!note.empty())
          ++c.fallbacks[note];
        if (noShape)
          ++c.fallbacks["the rebuilt collision shape could not be created; the "
                        "body is static until it gets one"];
      }
    }
    const unsigned changed = applyPropChanges(world, entry, p, shapesRebuilt);
    if (changed)
      ++c.updated;
    if (p.computeMass && (shapesRebuilt || (changed & kMassChanged)))
      writes |= Box3DPointWrite::kMass;
    // `awake` written since the solver last read or wrote it: 1 wakes the body,
    // 0 puts it to sleep (with whatever touches it). A disabled body is skipped
    // until it is enabled.
    if (a.awake.isValid() && p.enabled) {
      const bool want = a.awake.get(off) != 0;
      if (want != entry.awake) {
        if (want)
          world.wakeByHandle(entry.handle);
        else
          world.sleepByHandle(entry.handle);
        entry.awake = want;
      }
    }
  }

  // Did `active` / `animated` flip?
  const b3dop::BodySettings::Type wantType =
      wantedType(ctx.node, s.active, animated, entry.shapeless);
  if (entry.type != wantType) {
    world.setTypeByHandle(entry.handle, wantType);
    entry.type = wantType;
    entry.driven = false;
    // A type change recomputes the mass from the shapes: centre back to the
    // point.
    if (entry.dynamic())
      applyBodyMass(world, entry.handle, entry.props, entry.volume);
    ++c.retyped;
  }
  entry.animated = animated;

  const Box3DBodyPose &cur = s.cur;
  if (animated && entry.type == b3dop::BodySettings::Type::Kinematic) {
    // Follow the point: a new pose becomes this step's target; when it stops,
    // clear the velocity the target left.
    if (poseDiffers(cur, entry)) {
      world.setTargetTransformByHandle(entry.handle, cur.pos, cur.rot,
                                       st.stepDt);
      entry.setPose(cur);
      entry.driven = true;
      ++c.edited;
    } else if (entry.driven) {
      const float zero[3] = {0.0f, 0.0f, 0.0f};
      world.setVelocityByHandle(entry.handle, zero, zero);
      entry.driven = false;
    }
    anyDriven = anyDriven || entry.driven;
  }
  // Did something else edit P / orient / v / w since we wrote it?
  else if (poseDiffers(cur, entry)) {
    world.setTransformByHandle(entry.handle, cur.pos, cur.rot);
    world.setVelocityByHandle(entry.handle, cur.lin, cur.ang);
    entry.setPose(cur);
    ++c.edited;
  }
}

namespace {

using SlotList = std::vector<Box3DPointSlot>;

// ---- syncObject, step 1 (serial): collect the slots
// -------------------------- One slot per point, in point order, with its
// offset and point number. A slot forgets its cached entry when its offset
// changed. Without `id` the body id is the point number, i.e. the position in
// this walk.
void collectSlots(const GU_Detail *gdp, SlotList &snaps) {
  size_t n = 0;
  for (GA_Iterator it(gdp->getPointRange()); !it.atEnd(); ++it, ++n) {
    if (n == snaps.size())
      snaps.emplace_back();
    Box3DPointSlot &s = snaps[n];
    if (s.off != *it)
      s.entry = nullptr;
    s.off = *it;
    s.bid = static_cast<int64_t>(n);
    s.index = static_cast<int64_t>(n);
  }
  snaps.resize(n);
}

// ---- step 2 (parallel): read the points
// --------------------------------------- One point as the scan sees it: body
// id and key, state, packed primitive, and whether it needs shapes built (a new
// body, or a rebuild: deforming, bullet_autofit_valid = 0, or a convex /
// concave scale change, like Bullet). The mirror is only read here (the apply
// pass changes it).
void readSlot(const ScanContext &ctx, int objectId, Box3DPointSlot &s) {
  const PointAttrs &a = ctx.a;
  if (a.id.isValid())
    s.bid = static_cast<int64_t>(a.id.get(s.off));
  const uint64_t key = b3dop::makeBodyKey(objectId, s.bid);
  if (key != s.key)
    s.entry = nullptr;
  s.key = key;
  const PackedInfo pk =
      ctx.anyPrims ? getPackedInfo(ctx.gdp, s.off) : PackedInfo();
  s.ok = readPointState(ctx.gdp, a, s.off, pk, s.cur, &s.badVelocity);
  s.active = !a.active.isValid() || a.active.get(s.off) != 0;
  s.packed = pk.prim != nullptr;
  if (s.packed) {
    s.primOff = static_cast<int64_t>(pk.primOff);
    std::copy(pk.m3, pk.m3 + 9, s.m3);
    std::copy(pk.scale, pk.scale + 3, s.rowScale);
  }

  s.plan = -1;
  s.deforming = a.deforming.isValid() && a.deforming.get(s.off) != 0;
  s.refitWanted = a.autofitValid.isValid() && a.autofitValid.get(s.off) == 0;
  s.rebuild = false;
  s.dead = a.dead.isValid() && a.dead.get(s.off) != 0;
  if (!s.ok || s.dead)
    return;
  if (!s.entry) {
    const auto it = ctx.st.mirror.find(key);
    if (it != ctx.st.mirror.end())
      s.entry = &it->second;
  }
  if (const Box3DBodyMirror *e = s.entry) {
    s.rebuild = s.deforming || s.refitWanted;
    if (!s.rebuild && e->autoScale) {
      float ps, sc[3];
      double sk[3];
      readPscaleScale(a, s.off, ps, sc);
      s.rebuild =
          sizeKey(s.packed, s.m3, ps, sc, sk) && !sameSizeKey(sk, e->sizeKey);
    }
  }
}

void readSlots(const ScanContext &ctx, int objectId, SlotList &snaps) {
  parallelForEach(
      snaps.size(), [&](size_t i) { readSlot(ctx, objectId, snaps[i]); },
      ctx.anyPrims ? minParallelReadPacked() : minParallelReadPlain());
}

// ---- step 3 (parallel, chunked): plan the shapes of new and rebuilt bodies
// ------ Building hulls is the slow part of a new world and of deforming
// bodies. Box3DWorld's hull calls, ShapeSources and GeorepLookup are
// thread-safe, so the plans are made here in parallel and the apply pass only
// creates bodies. Chunked, so Escape is checked in between.

// The slots that need a plan: new bodies (unless `dead`) and existing ones
// being rebuilt.
std::vector<size_t> slotsNeedingPlans(const SlotList &snaps) {
  std::vector<size_t> want;
  for (size_t i = 0; i < snaps.size(); ++i)
    if (snaps[i].ok && (!snaps[i].entry ? !snaps[i].dead : snaps[i].rebuild))
      want.push_back(i);
  return want;
}

// False if the user cancelled.
bool planSlots(ScanContext &ctx, SlotList &snaps, std::vector<ShapePlan> &plans,
               StepProgress &progress) {
  const std::vector<size_t> want = slotsNeedingPlans(snaps);
  plans.resize(want.size());
  for (size_t c0 = 0; c0 < want.size(); c0 += kPlanChunk) {
    if (progress.check())
      return false;
    const size_t c1 = std::min(want.size(), c0 + kPlanChunk);
    parallelForEach(
        c1 - c0,
        [&](size_t j) {
          Box3DPointSlot &s = snaps[want[c0 + j]];
          plans[c0 + j] = planShapes(ctx, s.off, s.bid, ctx.packedInfo(s.off));
          s.plan = static_cast<int32_t>(c0 + j);
        },
        2);
  }
  return true;
}

// Many new bodies (a fresh world): size the mirror once.
void reserveMirror(Box3DSharedState &st, const SlotList &snaps) {
  size_t fresh = 0;
  for (const Box3DPointSlot &s : snaps)
    fresh += s.entry ? 0 : 1;
  if (fresh >= 64)
    st.mirror.reserve(st.mirror.size() + fresh);
}

// Bodies by point `name`, for joint anchors (only while some object has a joint
// network). The old map is kept to see whether a rename changed what the names
// resolve to.
class NameIndex {
public:
  NameIndex(Box3DSharedState &st, Box3DObjectSync &os, const GU_Detail *gdp)
      : myNameH(gdp, GA_ATTRIB_POINT, "name") {
    myOld.swap(os.byName);
    os.byName.clear();
    os.namesKnown = st.wantNames;
    myNames.bind(myNameH);
    myKeep = st.wantNames && myNames.valid();
  }
  NameIndex(const NameIndex &) = delete;
  NameIndex &operator=(const NameIndex &) = delete;

  // The first point of a name wins.
  void add(Box3DObjectSync &os, GA_Offset off, Box3DBodyMirror *entry) const {
    if (!myKeep)
      return;
    std::string n = myNames.str(off);
    if (!n.empty())
      os.byName.emplace(std::move(n), entry);
  }

  // Joint anchors resolve by point `name`: a renamed piece re-resolves the
  // joints.
  void finish(Box3DSharedState &st, const Box3DObjectSync &os) const {
    if (myKeep && os.byName != myOld)
      ++st.bodyVersion;
  }

private:
  const GA_ROHandleS myNameH;
  StringAttr myNames;
  std::unordered_map<std::string, Box3DBodyMirror *> myOld;
  bool myKeep = false;
};

// ---- step 4 (serial, point order): apply to Box3D and the mirror
// ---------------- An entry already carrying this step's stamp belongs to an
// earlier point (duplicate id). Escape is checked between points (the hulls
// were built by the planning step).
class ApplyPass {
public:
  ApplyPass(ScanContext &ctx, Box3DObjectSync &os, SyncCounts &c,
            std::vector<ShapePlan> &plans, const NameIndex &names, int objectId,
            bool checkProps)
      : myCtx(ctx), mySt(ctx.st), myOs(os), myC(c), myPlans(plans),
        myNames(names), myObjectId(objectId), myCheckProps(checkProps) {}

  // False if the user cancelled.
  bool run(SlotList &snaps, StepProgress &progress) {
    size_t pointIndex = 0;
    for (Box3DPointSlot &s : snaps) {
      if (pointIndex++ % kProgressEvery == 0 && progress.check())
        return false;
      applyPoint(s);
    }
    return true;
  }

  size_t stamped = 0; // entries stamped by this scan
  bool anyDeforming = false;
  bool anyDriven = false;

private:
  using MirrorMap = std::unordered_map<uint64_t, Box3DBodyMirror>;

  // A point's mirror entry: the cached one, else found, or (for a readable
  // point) inserted unstamped.
  struct EntryRef {
    Box3DBodyMirror *entry = nullptr;
    bool fresh = false;       // just inserted, no body yet
    MirrorMap::iterator node; // to erase if the body cannot be built
  };

  EntryRef resolveEntry(Box3DPointSlot &s) {
    EntryRef r;
    r.entry = s.entry;
    if (r.entry)
      return r;
    if (s.ok) {
      // One lookup for "known?" and "make room"; a new node is unstamped.
      auto res = mySt.mirror.try_emplace(s.key);
      r.entry = &res.first->second;
      r.fresh = res.second;
      if (r.fresh)
        r.node = res.first;
    } else {
      auto mit = mySt.mirror.find(s.key); // a bad point never creates a body
      if (mit != mySt.mirror.end())
        r.entry = &mit->second;
    }
    s.entry = r.entry;
    return r;
  }

  // Drops a just-inserted entry that will get no body.
  void dropFresh(const EntryRef &r, Box3DPointSlot &s) {
    if (r.fresh) {
      mySt.mirror.erase(r.node);
      s.entry = nullptr;
    }
  }

  // A point with no body yet: build it from its plan. False if that failed.
  bool addBody(Box3DPointSlot &s, Box3DBodyMirror &entry, bool animated,
               unsigned &writes) {
    const b3dop::BodyProps p = myCtx.props(s.off);
    std::string fallbackNote;
    const b3dop::BodyHandle handle =
        addPointBody(myCtx, s, takePlan(myCtx, s, myPlans), animated, p, entry,
                     fallbackNote);
    if (!handle) {
      ++myC.failed;
      return false;
    }
    if (!fallbackNote.empty())
      ++myC.fallbacks[fallbackNote];
    entry.setPose(s.cur);
    entry.type = wantedType(myCtx.node, s.active, animated, entry.shapeless);
    entry.handle = handle;
    entry.driven = false;
    if (p.computeMass)
      writes |= Box3DPointWrite::kMass;
    ++myC.added;
    return true;
  }

  // Every body this scan keeps: its point as read now, for the write-back and
  // later scans.
  void stampEntry(const Box3DPointSlot &s, Box3DBodyMirror &entry) {
    entry.pointOff = s.off;
    entry.pointId = s.bid;
    entry.pointIndex = s.index;
    entry.objectId = myObjectId;
    entry.stamp = mySt.syncStamp;
    entry.sync = &myOs;
    storePacked(entry, s);
    ++mySt.stampedCount;
    ++stamped;
  }

  void applyPoint(Box3DPointSlot &s) {
    const PointAttrs &a = myCtx.a;
    const GA_Offset off = s.off;

    const EntryRef ref = resolveEntry(s);
    Box3DBodyMirror *entry = ref.entry;

    if (entry && entry->stamp == mySt.syncStamp) {
      ++myC.duplicates; // two points with the same `id`: first wins
      return;
    }

    if (!s.ok) {
      ++myC.badP; // not stamped: the body (if any) is removed
      return;
    }
    if (s.badVelocity)
      ++myC.badVelocity; // simulated, with the bad v / w read as zero

    // `dead`: deleted after the step. Not stamped, so its body goes with the
    // gone points'.
    if (s.dead) {
      Box3DDeletion d;
      d.objectId = myObjectId;
      d.off = off;
      d.pointId = s.bid;
      mySt.deletions.push_back(d);
      dropFresh(ref, s);
      ++myC.dead;
      return;
    }

    const bool animated =
        !s.active && a.animated.isValid() && a.animated.get(off) != 0;
    anyDeforming = anyDeforming || s.deforming;
    unsigned writes = s.refitWanted ? Box3DPointWrite::kAutofitValid : 0u;

    if (ref.fresh && !addBody(s, *entry, animated, writes)) {
      dropFresh(ref, s);
      return;
    }

    stampEntry(s, *entry);
    if (!ref.fresh)
      updateExistingBody(myCtx, mySt, s, *entry, animated, myCheckProps,
                         myPlans, myC, writes, anyDriven);

    if (writes)
      queuePointWrite(mySt, *entry, writes,
                      effectiveMass(entry->props, entry->volume));
    myOs.bodies.push_back(entry);
    myNames.add(myOs, off, entry);
  }

  ScanContext &myCtx;
  Box3DSharedState &mySt;
  Box3DObjectSync &myOs;
  SyncCounts &myC;
  std::vector<ShapePlan> &myPlans;
  const NameIndex &myNames;
  const int myObjectId;
  const bool myCheckProps;
};

} // namespace

// Sync one object's points into the world: collect slots, read, plan shapes,
// apply. Returns early (object half synced) if the user cancels: the caller
// then discards the world (see solveObjectsSubclass).
void syncObject(Box3DSharedState &st, const SIM_Object &obj, SyncCounts &c,
                StepProgress &progress) {
  const SIM_Geometry *geo =
      SIM_DATA_GETCONST(obj, SIM_GEOMETRY_DATANAME, SIM_Geometry);
  if (!geo) {
    B3D_LOG_VERBOSE("  object id=%d has no Geometry data, skipped",
                    obj.getObjectId());
    return;
  }
  SIM_GeometryAutoReadLock lock(geo);
  const GU_Detail *gdp = lock.getGdp();
  if (!gdp)
    return;
  if (isWholeObject(obj)) {
    syncWholeObject(st, obj, *geo, gdp, c);
    return;
  }

  PointAttrs a(gdp);
  const int objectId = obj.getObjectId();
  const SIM_Box3DData *dataNode =
      SIM_DATA_GETCONST(obj, kBox3DDataName, SIM_Box3DData);
  const b3dop::BodySettings node =
      dataNode ? dataNode->toBodySettings() : b3dop::BodySettings();
  const bool addImpact = dataNode && dataNode->getAddImpact();

  // ---- unchanged since we last finished with it? ---------------------------
  // Markers are recorded after a scan and again after our own writes. Same
  // markers: no point can differ from the mirror, so every pass is skipped and
  // the entries stay current under liveStamp.
  Box3DObjectSync &os = st.objectSync[objectId];
  os.visited = st.syncStamp;
  os.addImpact = addImpact;
  os.wholeObject = false;
  const Box3DSyncFingerprint fpNow = makeFingerprint(
      gdp, a, static_cast<int>(node.type), node.props, addImpact);
  if (os.canSkip && fpNow == os.fp && (!st.wantNames || os.namesKnown)) {
    ++c.skipped;
    st.stampedCount += os.entryCount;
    return;
  }
#ifdef BOX3DDOP_DEBUG
  if (os.canSkip)
    B3D_LOG_VERBOSE("  object id=%d: full scan, changed: %s", objectId,
                    describeFingerprintDifference(os.fp, fpNow).c_str());
#endif
  os.canSkip = false;
  os.liveStamp = st.syncStamp;
  const int problems0 = c.failed + c.duplicates + c.badP;
  if (dataNode)
    noteOverriddenData(gdp, os, c);

  a.bindBuildAttrs(gdp);
  ScanContext ctx(st, gdp, a, node);

  // Settings are compared only when a prop marker changed (or deforming bodies
  // need a rebuild anyway); otherwise the scan syncs pose, velocity and type.
  // Convex / concave scale is checked on every scan.
  const bool checkProps =
      !os.propsKnown || !os.fp.samePropMarkers(fpNow) || os.anyDeforming;

  // For the post-step passes; kept as they are while the object is skipped.
  os.bodies.clear();
  const NameIndex names(st, os, gdp);

  SlotList &snaps = st.slots[objectId];
  collectSlots(gdp, snaps);
  readSlots(ctx, objectId, snaps);
  std::vector<ShapePlan> plans;
  if (!planSlots(ctx, snaps, plans, progress))
    return;
  reserveMirror(st, snaps);
  ApplyPass apply(ctx, os, c, plans, names, objectId, checkProps);
  if (!apply.run(snaps, progress))
    return;
  names.finish(st, os);

  // Problems (retried and reported every step), deforming and driven bodies
  // keep it scanning.
  os.entryCount = apply.stamped;
  os.fp = fpNow;
  os.propsKnown = true;
  os.anyDeforming = apply.anyDeforming;
  os.anyDriven = apply.anyDriven;
  os.canSkip = (c.failed + c.duplicates + c.badP) == problems0 &&
               !apply.anyDeforming && !apply.anyDriven;
}

} // namespace box3dsolver
