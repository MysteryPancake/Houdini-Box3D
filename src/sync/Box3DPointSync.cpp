// Box3DPointSync.cpp
//
// One body per point: reading points, the change fingerprint, the slot passes
// (collect, read, plan, settings, apply) and syncObject(), which adds, updates
// and removes an object's bodies.
#include "Box3DSolverInternal.h"
#include <functional>
#include <string_view>

namespace box3dsolver {

// P / rotation / v / w of one point. A packed point's rotation: its
// `restxform`'s times `orient` (the rotation since, as Bullet) once restxform
// is set, else its primitive's. False if P is not finite. A non-finite v or w
// is read as zero (Box3D asserts on it, and a NaN velocity would spread to the
// body's position); *badVelocity says so.
bool readPointState(const GU_Detail *gdp, const PointAttrs &a, GA_Offset off,
                    const PackedInfo &pk, Box3DBodyPose &out, bool *badVelocity,
                    Box3DRestCache *cache) {
  if (badVelocity)
    *badVelocity = false;
  const UT_Vector3 p = gdp->getPos3(off);
  out.pos[0] = p.x();
  out.pos[1] = p.y();
  out.pos[2] = p.z();
  if (!b3dop::finite3(out.pos))
    return false;

  out.rot[0] = out.rot[1] = out.rot[2] = 0.0f;
  out.rot[3] = 1.0f;
  float rest[4];
  bool hasRest = false;
  if (pk.prim && a.orient.isValid() && a.restxform.isValid()) {
    if (cache && cache->known) {
      hasRest = cache->ok;
      b3dop::copy4(rest, cache->rot);
    } else {
      hasRest = restRotation(a.restxform.get(off), rest);
      if (cache) {
        cache->known = true;
        cache->ok = hasRest;
        if (hasRest)
          b3dop::copy4(cache->rot, rest);
      }
    }
  }
  if (hasRest) {
    const UT_QuaternionF q = a.orient.get(off);
    float since[4] = {q.x(), q.y(), q.z(), q.w()};
    b3dop::sanitizeQuat(since);
    box3dRestToAbsolute(rest, since, out.rot);
  } else if (pk.prim) {
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

// Position or orientation differ (as poseDiffers, without the velocities): a
// body whose velocity alone was edited is not moved.
bool placeDiffers(const Box3DBodyPose &a, const Box3DBodyPose &b) {
  const double eps = 1e-5;
  if (b3dop::quatsDiffer(a.rot, b.rot, eps))
    return true;
  for (int i = 0; i < 3; ++i)
    if (std::fabs(a.pos[i] - b.pos[i]) > eps)
      return true;
  return false;
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
  // By name: `a` has only the sync-critical handles bound at this point. One
  // pass over the point attributes rather than a lookup per marker.
  static const std::unordered_map<std::string_view, int> propIndex = [] {
    std::unordered_map<std::string_view, int> index;
    for (int i = 0; i < Box3DSyncFingerprint::kNumPropIds; ++i)
      index.emplace(kPropAttribNames[i], i);
    return index;
  }();
  for (const GA_Attribute *attr :
       gdp->getAttributeDict(GA_ATTRIB_POINT).range(GA_SCOPE_PUBLIC)) {
    const auto found =
        propIndex.find(std::string_view(attr->getName().c_str()));
    if (found != propIndex.end())
      fp.propIds[static_cast<size_t>(found->second)] = attr->getDataId();
  }
  // The primitive list's data id is the marker for packed transforms
  // (intrinsics have no attribute id). Bumping it is up to whoever edits a
  // primitive; an edit that does not is picked up on the next scan instead.
  fp.primsId = gdp->getNumPrimitives() > 0 ? gdp->getPrimitiveList().getDataId()
                                           : Box3DSyncFingerprint::kNoId;
  fp.nodeProps = nodeProps;
  fp.addImpact = addImpact;
  return fp;
}

namespace {

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
  auto ll = [](GA_DataId id) { return static_cast<long long>(id); };
  add("P", ll(was.pId), ll(now.pId));
  add("orient", ll(was.orientId), ll(now.orientId));
  add("v", ll(was.vId), ll(now.vId));
  add("w", ll(was.wId), ll(now.wId));
  add("id", ll(was.idId), ll(now.idId));
  add("active", ll(was.activeId), ll(now.activeId));
  add("body type", was.nodeType, now.nodeType);
  for (size_t i = 0; i < was.propIds.size(); ++i)
    add(kPropAttribNames[i], ll(was.propIds[i]), ll(now.propIds[i]));
  add("primitives", ll(was.primsId), ll(now.primsId));
  if (was.nodeProps != now.nodeProps || was.addImpact != now.addImpact)
    out += out.empty() ? "Box3D Data values" : ", Box3D Data values";
  return out;
}

} // namespace

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

namespace {

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

} // namespace

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

namespace {

// What the body was built from, into its mirror entry.
void storePlan(Box3DBodyMirror &m, const ShapePlan &plan) {
  m.shapeSig = plan.sig;
  m.volume = plan.shapes().totalVolume;
  // Shapes that depend on the body being static (mesh, plane) rebuild when it
  // changes.
  m.concave = plan.type == b3dop::ShapeType::Concave ||
              plan.type == b3dop::ShapeType::Plane;
  m.plannedStatic = plan.plannedStatic;
  m.meshShapes = plan.shapes().mesh;
  m.autoScale = plan.autoScale;
  b3dop::copy3(m.sizeKey, plan.sizeKey);
}

// A new packed body's rest (docs/agents/body-features.md, "Rest transform"):
// the point's `restxform` if set; else its transform now, which the solver then
// writes as `restxform` with `pivot` = P (writeRest, kRest). Plain data with no
// initializers, made in the parallel plan pass (the array of them is not
// cleared first, so its pages are first touched by the threads that fill it).
struct RestInit {
  bool hasRest;
  bool writeRest;
  float restRot[4];
  double restPos[3];
  float restxform[16];
  float pivot[3];
};

// This scan's shape plans and rest data for new and rebuilt bodies, by
// Box3DPointSlot::plan.
struct SlotPlans {
  std::vector<ShapePlan> shapes;
  std::unique_ptr<RestInit[]> rest;
  std::vector<b3dop::BodyProps> props; // by Box3DPointSlot::props
};

// Works out the rest of the new packed body of slot `s`, whose primitive (with
// its pivot) is `pk`.
void computeRestInit(const ScanContext &ctx, const Box3DPointSlot &s,
                     const PackedInfo &pk, RestInit &r) {
  r.hasRest = false;
  r.writeRest = false;
  if (!s.packed || !pk.prim)
    return;
  const PointAttrs &a = ctx.a;
  float rest[4];
  UT_Matrix4F m;
  bool set = false;
  if (a.restxform.isValid()) {
    m = a.restxform.get(s.off);
    set = restRotation(m, rest);
  }
  if (!set) {
    // As the attribute will store it, so the read path splits the same
    // matrix.
    for (int i = 0; i < 16; ++i)
      r.restxform[i] = static_cast<float>(pk.m4[i]);
    m = box3dMatrix4F(r.restxform);
    if (!restRotation(m, rest))
      return;
    for (int i = 0; i < 3; ++i)
      r.pivot[i] = static_cast<float>(s.cur.pos[i]);
    r.writeRest = true;
  }
  b3dop::copy4(r.restRot, rest);
  // The packed pivot (its own space) through restxform: where the body's
  // origin was at rest.
  for (unsigned c = 0; c < 3; ++c)
    r.restPos[c] = pk.pivot[0] * m(0, c) + pk.pivot[1] * m(1, c) +
                   pk.pivot[2] * m(2, c) + m(3, c);
  r.hasRest = true;
}

// The slot's plan from the parallel pre-pass (moved out), else made now.
ShapePlan takePlan(ScanContext &ctx, const Box3DPointSlot &s,
                   SlotPlans &plans) {
  if (s.plan >= 0)
    return std::move(plans.shapes[static_cast<size_t>(s.plan)]);
  const PackedInfo pk = ctx.packedInfo(s.off);
  return planShapes(ctx, s.off, s.bid, pk, s.staticBody);
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
  if (plan.shapes().note)
    fallbackNote = plan.shapes().note;
  entry.props = p;
  entry.shapeless = (plan.type == b3dop::ShapeType::None);
  entry.animated = animated;
  storePlan(entry, plan);

  b3dop::BodySettings b = ctx.node;
  b.type = wantedType(ctx.node, s.active, animated, entry.shapeless);
  b.props = p;
  b.props.density = boxDensity(p, plan.shapes().totalVolume);
  b.startAwake = startAwakeFor(ctx.node, ctx.a, s.off);
  entry.awake = b.startAwake;

  // Animated bodies follow their point instead of starting with its velocity.
  return createBodyWithShapes(world, b, st, plan.shapes(), entry.shapeless, p,
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
  if (plan.shapes().note)
    note = plan.shapes().note;
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
    b.props.density = boxDensity(p, plan.shapes().totalVolume);
    if (addAllShapes(world, h, b, plan.shapes()))
      entry.shapeless = false;
    else
      noShape = true;
  }
  applyBodyMass(world, h, p, plan.shapes().totalVolume, entry.restRot,
                false);  // even with no shapes: clears Box3D's dirty mass
  world.wakeByHandle(h); // let it re-settle
  B3D_DEBUG_LOG_VERBOSE("  body id=%lld: collision shape rebuilt",
                        static_cast<long long>(s.bid));
  return true;
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
    m.mirrored = b3dop::matrixDeterminant(s.m3) < 0.0;
    std::copy(s.rowScale, s.rowScale + 3, m.rowScale);
  }
}

// The mid-simulation changes of an existing body (docs/agents/bodies.md,
// "Mid-simulation updates"): shape rebuild and settings, type, then the
// animated target or an external edit of its pose. Adds to `writes` the point
// attributes to write after the step.
void updateExistingBody(ScanContext &ctx, Box3DSharedState &st,
                        Box3DPointSlot &s, Box3DBodyMirror &entry,
                        bool animated, bool checkProps, SlotPlans &plans,
                        SyncCounts &c, unsigned &writes, bool &anyDriven) {
  b3dop::Box3DWorld &world = ctx.world;
  const PointAttrs &a = ctx.a;
  const GA_Offset off = s.off;

  // Becoming static: retype before the rebuild, which may make a `concave`
  // mesh (Box3D meshes only go on static bodies).
  if (s.rebuild && s.staticBody &&
      retypeBody(world, entry, b3dop::BodySettings::Type::Static))
    ++c.retyped;

  // Shape first (a shapeless body is static); pass 1 decided whether to
  // rebuild.
  const bool rebuild = s.rebuild;
  if (rebuild || checkProps) {
    b3dop::BodyProps p = s.props >= 0
                             ? plans.props[static_cast<size_t>(s.props)]
                             : ctx.props(off);
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

  // Did `active` / `animated` flip? A body with a mesh stays static (its
  // shapes are rebuilt as hulls first, in pass 1's rebuild).
  const b3dop::BodySettings::Type wantType =
      entry.meshShapes
          ? b3dop::BodySettings::Type::Static
          : wantedType(ctx.node, s.active, animated, entry.shapeless);
  if (retypeBody(world, entry, wantType))
    ++c.retyped;
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
    // Moving a body updates its shapes' bounds in the broad phase: only when
    // its place changed, not for an edit of v or w alone.
    if (placeDiffers(cur, entry))
      world.setTransformByHandle(entry.handle, cur.pos, cur.rot);
    world.setVelocityByHandle(entry.handle, cur.lin, cur.ang);
    entry.setPose(cur);
    // A glued piece: its compound is dissolved before the step, the piece
    // keeping this pose.
    if (entry.inCompound)
      entry.glueEdited = true;
    ++c.edited;
  }
}

using SlotList = std::vector<Box3DPointSlot>;

// ---- syncObject, step 1 (serial): collect the slots
// One slot per point, in point order, with its offset and point number. A slot
// forgets its cached entry when its offset changed. Without `id` the body id is
// the point number, i.e. the position in this walk.
void collectSlots(const GU_Detail *gdp, SlotList &snaps) {
  // A big first scan (a fresh world) fills memory nothing has touched yet:
  // its pages are touched on several threads before the serial loop below
  // constructs the slots in them (a zero byte written to the unconstructed
  // capacity, which the slot then overwrites).
  constexpr size_t kPage = 4096;
  const size_t points = static_cast<size_t>(gdp->getNumPoints());
  if (points > snaps.capacity() && points >= 8192) {
    snaps.reserve(points);
    char *base = reinterpret_cast<char *>(snaps.data()) +
                 snaps.size() * sizeof(Box3DPointSlot);
    const size_t bytes = (points - snaps.size()) * sizeof(Box3DPointSlot);
    parallelForEach((bytes + kPage - 1) / kPage,
                    [base](size_t i) { base[i * kPage] = 0; }, 64, 16);
  }
  size_t n = 0;
  for (GA_Iterator it(gdp->getPointRange()); !it.atEnd(); ++it, ++n) {
    if (n == snaps.size())
      snaps.emplace_back();
    Box3DPointSlot &s = snaps[n];
    if (s.off != *it) {
      s.entry = nullptr;
      s.rest.known = false;
    }
    s.off = *it;
    s.bid = static_cast<int64_t>(n);
    s.index = static_cast<int64_t>(n);
  }
  snaps.resize(n);
}

// ---- step 2 (parallel): read the points
// One point as the scan sees it: body id and key, state, packed primitive, and
// whether it needs shapes built (a new body, or a rebuild: deforming,
// bullet_autofit_valid = 0, or a convex / concave scale change, like Bullet).
// The mirror is only read here (the apply pass changes it).
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
  if (!ctx.restReuse)
    s.rest.known = false; // restxform changed: split again
  s.ok = readPointState(ctx.gdp, a, s.off, pk, s.cur, &s.badVelocity, &s.rest);
  s.active = !a.active.isValid() || a.active.get(s.off) != 0;
  s.animated = !s.active && a.animated.isValid() && a.animated.get(s.off) != 0;
  s.staticBody = wantedType(ctx.node, s.active, s.animated, false) ==
                 b3dop::BodySettings::Type::Static;
  s.packed = pk.prim != nullptr;
  s.unpackedPrim =
      !s.packed && ctx.anyPrims && GAisValid(ctx.gdp->pointVertex(s.off));
  s.agentRagdoll = s.packed && isRagdollAgent(pk, ctx.activeAgent, s.off);
  if (s.packed) {
    s.primOff = static_cast<int64_t>(pk.primOff);
    std::copy(pk.m3, pk.m3 + 9, s.m3);
    std::copy(pk.scale, pk.scale + 3, s.rowScale);
  }

  s.plan = -1;
  s.props = -1;
  s.quiet = false;
  s.deforming = a.deforming.isValid() && a.deforming.get(s.off) != 0;
  s.refitWanted = a.autofitValid.isValid() && a.autofitValid.get(s.off) == 0;
  s.rebuild = false;
  s.dead = a.dead.isValid() && a.dead.get(s.off) != 0;
  if (!s.ok || s.dead || s.unpackedPrim || s.agentRagdoll)
    return;
  if (!s.entry) {
    s.entry = ctx.st.mirror.find(key);
  }
  if (const Box3DBodyMirror *e = s.entry) {
    s.rebuild = s.deforming || s.refitWanted;
    // Concave changes shape with the body type (mesh when static, hulls
    // otherwise).
    if (!s.rebuild && e->concave && e->plannedStatic != s.staticBody)
      s.rebuild = true;
    if (!s.rebuild && e->autoScale) {
      float ps, sc[3];
      double sk[3];
      readPscaleScale(a, s.off, ps, sc);
      s.rebuild =
          sizeKey(s.packed, s.m3, ps, sc, sk) && !sameSizeKey(sk, e->sizeKey);
    }
    // A packed piece that turned mirrored (or back): the row lengths do not
    // show it, but its hull is built mirrored to match.
    if (!s.rebuild && s.packed && e->packed &&
        (b3dop::matrixDeterminant(s.m3) < 0.0) != e->mirrored)
      s.rebuild = true;
  }
}

// The problems that keep an object scanning every step (retried and reported).
int problemCount(const SyncCounts &c) {
  return c.failed + c.duplicates + c.badP + c.unpackedPrimPoints +
         static_cast<int>(c.unsupportedObjects.size());
}

void readSlots(const ScanContext &ctx, int objectId, SlotList &snaps) {
  parallelForEach(
      snaps.size(), [&](size_t i) { readSlot(ctx, objectId, snaps[i]); },
      ctx.anyPrims ? minParallelReadPacked() : minParallelReadPlain());
}

// The slots that need a plan: new bodies (unless `dead`) and existing ones
// being rebuilt.
std::vector<size_t> slotsNeedingPlans(const SlotList &snaps) {
  // Looking at every slot is a pass over memory the size of the slot list: done
  // in parallel into flags, then listed.
  constexpr size_t kMinParallelFlags = 8192;
  thread_local std::vector<uint8_t> flagStore; // kept between scans
  std::vector<uint8_t> &flag = flagStore;
  flag.resize(snaps.size());
  box3dParallelFor(
      snaps.size(),
      [&](size_t i) {
        const Box3DPointSlot &s = snaps[i];
        flag[i] = s.ok && !s.unpackedPrim && !s.agentRagdoll &&
                  (!s.entry ? !s.dead : s.rebuild);
      },
      kMinParallelFlags, 1024);
  std::vector<size_t> want;
  for (size_t i = 0; i < flag.size(); ++i)
    if (flag[i])
      want.push_back(i);
  return want;
}

// False if the user cancelled.
bool planSlots(ScanContext &ctx, SlotList &snaps, SlotPlans &plans,
               StepProgress &progress) {
  const std::vector<size_t> want = slotsNeedingPlans(snaps);
  plans.shapes.resize(want.size());
  plans.rest.reset(new RestInit[want.size()]);
  for (size_t c0 = 0; c0 < want.size(); c0 += kPlanChunk) {
    if (progress.check())
      return false;
    const size_t c1 = std::min(want.size(), c0 + kPlanChunk);
    parallelForEach(
        c1 - c0,
        [&](size_t j) {
          Box3DPointSlot &s = snaps[want[c0 + j]];
          PackedInfo pk;
          if (s.packed)
            pk = ctx.packedInfo(s.off);
          plans.shapes[c0 + j] =
              planShapes(ctx, s.off, s.bid, pk, s.staticBody);
          // Only a new body takes its rest from it.
          if (s.entry)
            plans.rest[c0 + j].hasRest = plans.rest[c0 + j].writeRest = false;
          else
            computeRestInit(ctx, s, pk, plans.rest[c0 + j]);
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
    st.mirror.reserve(
        st.mirror.size() + fresh,
        [](size_t count, const auto &f) { box3dParallelFor(count, f, 4); });
}

// ---- step 3b (parallel): settings, and the points with nothing to do
// The settings of the points that need them (new bodies, rebuilds, and every
// body when a settings marker changed) are read here instead of in the apply
// pass. A point whose body exists and whose pose, type and settings are what
// the mirror holds has nothing to apply: its entry is stamped here (each point
// writes only its own entry) and the apply pass only lists it. Only the point
// that last owned the entry qualifies (same offset and id), so a duplicate `id`
// never races and the owner keeps the body. Kinematic bodies, edits, rebuilds,
// deforming and problem points stay with the apply pass.
bool quietSlot(const ScanContext &ctx, const Box3DPointSlot &s,
               const b3dop::BodyProps *props) {
  const Box3DBodyMirror *e = s.entry;
  // The owner first: a point that does not own the entry reads nothing the
  // owner writes here (only its stamp, point index and packed data).
  if (!e || e->pointOff != static_cast<int64_t>(s.off) || e->pointId != s.bid)
    return false;
  if (!e->handle || !s.ok || s.dead || s.unpackedPrim || s.agentRagdoll ||
      s.badVelocity || s.rebuild || s.deforming || s.refitWanted || e->driven ||
      e->wholeObject)
    return false;
  const b3dop::BodySettings::Type want =
      e->meshShapes ? b3dop::BodySettings::Type::Static
                    : wantedType(ctx.node, s.active, s.animated, e->shapeless);
  if (e->type != want || e->animated != s.animated ||
      (s.animated && e->type == b3dop::BodySettings::Type::Kinematic))
    return false;
  if (poseDiffers(s.cur, *e))
    return false;
  if (props) {
    // What updateExistingBody would find to do with the settings.
    if (!(*props == e->props))
      return false;
    if (ctx.a.awake.isValid() && props->enabled &&
        (ctx.a.awake.get(s.off) != 0) != e->awake)
      return false;
  }
  return true;
}

void stampQuietSlots(ScanContext &ctx, Box3DObjectSync &os, int objectId,
                     SlotList &snaps, SlotPlans &plans, bool checkProps) {
  size_t need = 0;
  for (Box3DPointSlot &s : snaps) {
    const bool wants = s.ok && !s.unpackedPrim && !s.agentRagdoll && !s.dead &&
                       (!s.entry || s.rebuild || checkProps);
    s.props = wants ? static_cast<int32_t>(need++) : -1;
  }
  plans.props.resize(need);
  const uint64_t stamp = ctx.st.syncStamp;
  parallelForEach(
      snaps.size(),
      [&](size_t i) {
        Box3DPointSlot &s = snaps[i];
        if (s.props >= 0)
          plans.props[static_cast<size_t>(s.props)] = ctx.props(s.off);
        const b3dop::BodyProps *p =
            (checkProps && s.props >= 0)
                ? &plans.props[static_cast<size_t>(s.props)]
                : nullptr;
        // With settings to compare they must have been read for this point.
        if (checkProps && !p && s.entry)
          return;
        if (!quietSlot(ctx, s, p))
          return;
        Box3DBodyMirror &e = *s.entry;
        // pointOff and pointId already are this point's (it owns the entry).
        e.pointIndex = s.index;
        e.objectId = objectId;
        e.stamp = stamp;
        e.sync = &os;
        storePacked(e, s);
        s.quiet = true;
      },
      ctx.anyPrims ? minParallelReadPacked() : minParallelReadPlain());
}

// Bodies by point `name`, for joint anchors (only while some object has a joint
// network). Each scan hashes the (name, entry) pairs in point order (the order
// the points are visited in does not matter), without copying a string; the map
// (os.byName) is rebuilt only when that hash differs from the one it was built
// from, and the joints re-resolve only if what the names resolve to changed.
class NameIndex {
public:
  NameIndex(Box3DSharedState &st, Box3DObjectSync &os, const GU_Detail *gdp)
      : myNameH(gdp, GA_ATTRIB_POINT, "name") {
    os.namesKnown = st.wantNames;
    myKeep = st.wantNames && myNameH.isValid();
    os.namedPoints.clear();
    if (!myKeep) {
      os.byName.clear();
      os.nameSig = 0; // not built: the next scan that keeps names rebuilds
    }
  }
  NameIndex(const NameIndex &) = delete;
  NameIndex &operator=(const NameIndex &) = delete;

  // The first point of a name wins.
  void add(Box3DObjectSync &os, GA_Offset off, Box3DBodyMirror *entry) const {
    if (!myKeep)
      return;
    // The point's name without a copy; null for none.
    const char *v = myNameH.get(off);
    if (!v || !*v)
      return;
    os.namedPoints.emplace_back(off, entry);
  }

  // Joint anchors resolve by point `name`: a renamed piece re-resolves the
  // joints.
  void finish(Box3DSharedState &st, Box3DObjectSync &os) const {
    if (!myKeep)
      return;
    // Point order, whatever order the points were applied in (body id order).
    auto byOffset = [](const auto &a, const auto &b) {
      return a.first < b.first;
    };
    if (!std::is_sorted(os.namedPoints.begin(), os.namedPoints.end(), byOffset))
      std::sort(os.namedPoints.begin(), os.namedPoints.end(), byOffset);
    uint64_t h = kSeed;
    for (const auto &p : os.namedPoints) {
      h = (h ^ std::hash<std::string_view>()(
                   std::string_view(myNameH.get(p.first)))) *
          kPrime;
      h = (h ^ static_cast<uint64_t>(reinterpret_cast<uintptr_t>(p.second))) *
          kPrime;
    }
    if (h == os.nameSig)
      return;
    std::unordered_map<std::string, Box3DBodyMirror *> fresh;
    fresh.reserve(os.namedPoints.size());
    for (const auto &p : os.namedPoints) {
      const char *name = myNameH.get(p.first);
      fresh.emplace(name ? std::string(name) : std::string(),
                    p.second); // first point wins
    }
    if (fresh != os.byName)
      ++st.bodyVersion;
    // A piece still here under another name, or whose name another piece took,
    // can move a joint's anchor. Pieces added or removed cannot.
    for (const auto &kv : os.byName) {
      const auto it = fresh.find(kv.first);
      if (it != fresh.end() && it->second == kv.second)
        continue;
      if (kv.second->stamp == st.syncStamp) {
        ++st.nameVersion;
        break;
      }
    }
    os.byName.swap(fresh);
    os.nameSig = h;
  }

private:
  static constexpr uint64_t kSeed = 14695981039346656037ull;
  static constexpr uint64_t kPrime = 1099511628211ull;
  const GA_ROHandleS myNameH;
  bool myKeep = false;
};

// ---- step 4 (serial, body id order): apply to Box3D and the mirror
// An entry already carrying this step's stamp belongs to an earlier point
// (duplicate id). Escape is checked between points (the hulls were built by the
// planning step).
class ApplyPass {
public:
  ApplyPass(ScanContext &ctx, Box3DObjectSync &os, SyncCounts &c,
            SlotPlans &plans, const NameIndex &names, int objectId,
            bool checkProps)
      : myCtx(ctx), mySt(ctx.st), myOs(os), myC(c), myPlans(plans),
        myNames(names), myObjectId(objectId), myCheckProps(checkProps) {}

  // False if the user cancelled.
  bool run(SlotList &snaps, StepProgress &progress) {
    // In body id order (point order for equal ids), so the bodies of a scan
    // are created in the same order whatever order the points are in: Box3D's
    // results depend slightly on it. Already sorted (the usual case): no sort.
    thread_local std::vector<uint32_t> order; // scratch kept between scans
    bool sorted = true;
    for (size_t i = 1; i < snaps.size() && sorted; ++i)
      sorted = snaps[i - 1].bid <= snaps[i].bid;
    if (!sorted) {
      order.resize(snaps.size());
      for (size_t i = 0; i < order.size(); ++i)
        order[i] = static_cast<uint32_t>(i);
      std::stable_sort(order.begin(), order.end(), [&](uint32_t x, uint32_t y) {
        return snaps[x].bid < snaps[y].bid;
      });
    }
    for (size_t k = 0; k < snaps.size(); ++k) {
      if (k % kProgressEvery == 0 && progress.check())
        return false;
      applyPoint(sorted ? snaps[k] : snaps[order[k]]);
    }
    return true;
  }

  size_t stamped = 0; // entries stamped by this scan
  bool anyDeforming = false;
  bool anyDriven = false;

private:
  // A point's mirror entry: the cached one, else found, or (for a readable
  // point) inserted unstamped.
  struct EntryRef {
    Box3DBodyMirror *entry = nullptr;
    bool fresh = false; // just inserted, no body yet
    uint32_t slot = 0;  // its mirror slot, to erase if the body cannot be built
  };

  EntryRef resolveEntry(Box3DPointSlot &s) {
    EntryRef r;
    r.entry = s.entry;
    if (r.entry)
      return r;
    if (s.ok) {
      // One lookup for "known?" and "make room"; a new node is unstamped.
      const Box3DMirrorMap::Ref res = mySt.mirror.tryEmplace(s.key);
      r.entry = res.entry;
      r.fresh = res.inserted;
      r.slot = res.index;
    } else {
      r.entry = mySt.mirror.find(s.key); // a bad point never creates a body
    }
    s.entry = r.entry;
    return r;
  }

  // Drops a just-inserted entry that will get no body.
  void dropFresh(const EntryRef &r, Box3DPointSlot &s) {
    if (r.fresh) {
      mySt.mirror.erase(r.slot);
      s.entry = nullptr;
    }
  }

  // A point with no body yet: build it from its plan. False if that failed.
  bool addBody(Box3DPointSlot &s, Box3DBodyMirror &entry, bool animated,
               unsigned &writes) {
    const b3dop::BodyProps p = s.props >= 0
                                   ? myPlans.props[static_cast<size_t>(s.props)]
                                   : myCtx.props(s.off);
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

  // A new packed body's rest, worked out by the plan pass (computeRestInit)
  // or, for a point without a plan, now. A written rest (kRest) is filled into
  // `pending`.
  void initRest(const Box3DPointSlot &s, Box3DBodyMirror &entry,
                unsigned &writes, Box3DPointWrite &pending) {
    RestInit local;
    const RestInit *r = &local;
    if (s.plan >= 0) {
      r = &myPlans.rest[static_cast<size_t>(s.plan)];
    } else {
      const PackedInfo pk = s.packed ? myCtx.packedInfo(s.off) : PackedInfo();
      computeRestInit(myCtx, s, pk, local);
    }
    entry.hasRest = r->hasRest;
    if (r->writeRest) {
      std::copy(r->restxform, r->restxform + 16, pending.restxform);
      std::copy(r->pivot, r->pivot + 3, pending.pivot);
      writes |= Box3DPointWrite::kRest;
    }
    if (r->hasRest) {
      b3dop::copy4(entry.restRot, r->restRot);
      for (unsigned c = 0; c < 3; ++c)
        entry.restPos[c] = r->restPos[c];
    }
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
    const GA_Offset off = s.off;

    // A polygon's (or other unpacked primitive's) point: no body. Not stamped,
    // so a body it had is removed. Counted by syncObject.
    if (s.unpackedPrim || s.agentRagdoll) {
      s.entry = nullptr;
      return;
    }

    if (s.quiet) { // stamped by stampQuietSlots
      ++mySt.stampedCount;
      ++stamped;
      myOs.bodies.push_back(s.entry);
      myNames.add(myOs, off, s.entry);
      return;
    }

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

    // `dead`: out of the simulation, and deleted after the step when reaping.
    // Not stamped, so its body goes with the gone points'.
    if (s.dead) {
      if (mySt.reapDead) {
        Box3DDeletion d;
        d.objectId = myObjectId;
        d.off = off;
        d.pointId = s.bid;
        mySt.deletions.push_back(d);
      }
      dropFresh(ref, s);
      ++myC.dead;
      return;
    }

    anyDeforming = anyDeforming || s.deforming;
    unsigned writes = s.refitWanted ? Box3DPointWrite::kAutofitValid : 0u;

    if (ref.fresh && !addBody(s, *entry, s.animated, writes)) {
      dropFresh(ref, s);
      return;
    }
    Box3DPointWrite rest;
    if (ref.fresh)
      initRest(s, *entry, writes, rest);

    stampEntry(s, *entry);
    if (!ref.fresh)
      updateExistingBody(myCtx, mySt, s, *entry, s.animated, myCheckProps,
                         myPlans, myC, writes, anyDriven);

    if (writes) {
      queuePointWrite(mySt, *entry, writes,
                      effectiveMass(entry->props, entry->volume));
      if (writes & Box3DPointWrite::kRest) {
        Box3DPointWrite &w = mySt.pointWrites.back();
        std::copy(rest.restxform, rest.restxform + 16, w.restxform);
        std::copy(rest.pivot, rest.pivot + 3, w.pivot);
      }
    }
    myOs.bodies.push_back(entry);
    myNames.add(myOs, off, entry);
  }

  ScanContext &myCtx;
  Box3DSharedState &mySt;
  Box3DObjectSync &myOs;
  SyncCounts &myC;
  SlotPlans &myPlans;
  const NameIndex &myNames;
  const int myObjectId;
  const bool myCheckProps;
};

} // namespace

// Sync one object's points into the world: collect slots, read, plan shapes,
// apply. Returns early (object half synced) if the user cancels: the caller
// then discards the world (see solveObjectsSubclass).
// Bodies in one scan before a world that is mostly new is built in bulk.
constexpr size_t kMinBulkCreate = 1024;

void syncObject(Box3DSharedState &st, const SIM_Object &obj, SyncCounts &c,
                StepProgress &progress) {
  const SIM_Geometry *geo =
      SIM_DATA_GETCONST(obj, SIM_GEOMETRY_DATANAME, SIM_Geometry);
  if (!geo) {
    B3D_DEBUG_LOG_VERBOSE("  object id=%d has no Geometry data, skipped",
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
  b3dop::BodySettings node =
      dataNode ? dataNode->toBodySettings() : b3dop::nodeBodyDefaults();
  node.props.shapeBoundsInertia = st.shapeBoundsInertia;
  const bool addImpact = dataNode && dataNode->getAddImpact();

  // ---- unchanged since we last finished with it? ---------------------------
  // Markers are recorded after a scan and again after our own writes. Same
  // markers: no point can differ from the mirror, so every pass is skipped and
  // the entries stay current under liveStamp.
  Box3DObjectSync &os = st.objectSync[objectId];
  os.visited = st.syncStamp;
  os.addImpact = addImpact;
  os.wholeObject = false;
  const Box3DSyncFingerprint fpNow = [&] {
    UT_PerfMonAutoSolveEvent perf(
        progress.solver, "Pre-Solve}/{Updating Bodies}/{Checking Changes");
    return makeFingerprint(gdp, a, static_cast<int>(node.type), node.props,
                           addImpact);
  }();
  if (os.canSkip && fpNow == os.fp && (!st.wantNames || os.namesKnown)) {
    ++c.skipped;
    st.stampedCount += os.entryCount;
    return;
  }
#ifdef BOX3DDOP_DEBUG
  if (os.canSkip)
    B3D_DEBUG_LOG_VERBOSE("  object id=%d: full scan, changed: %s", objectId,
                          describeFingerprintDifference(os.fp, fpNow).c_str());
#endif
  os.canSkip = false;
  ++st.fullScans;
  os.liveStamp = st.syncStamp;
  const int problems0 = problemCount(c);
  if (dataNode)
    noteOverriddenData(gdp, os, c, node);
  else
    os.overridesReported.clear(); // reported afresh if a Box3D Data comes back

  a.bindBuildAttrs(gdp);
  ScanContext ctx(st, gdp, a, node);
  const GA_DataId restId = a.restxform.isValid()
                               ? a.restxform.getAttribute()->getDataId()
                               : Box3DSyncFingerprint::kNoId;
  ctx.restReuse = restId == os.restCacheId;

  // Settings are compared only when a prop marker changed (or deforming bodies
  // need a rebuild anyway); otherwise the scan syncs pose, velocity and type.
  // Convex / concave scale is checked on every scan.
  const bool checkProps =
      !os.propsKnown || !os.fp.samePropMarkers(fpNow) || os.anyDeforming;

  // For the post-step passes; kept as they are while the object is skipped.
  os.bodies.clear();
  const NameIndex names(st, os, gdp);

  SlotList &snaps = st.slots[objectId];
  {
    UT_PerfMonAutoSolveEvent perf(
        progress.solver, "Pre-Solve}/{Updating Bodies}/{Reading Points");
    collectSlots(gdp, snaps);
    readSlots(ctx, objectId, snaps);
  }
  // Unpacked primitives: their points get no body. With no packed point at all
  // the object is not simulated (a warning, as with RBD Bullet Solver): nothing
  // is stamped, so bodies it had are removed, and canSkip stays false, so it is
  // checked (and reported) again every step. With some, those points only.
  {
    int unpacked = 0;
    bool anyPacked = false;
    for (const Box3DPointSlot &s : snaps) {
      unpacked += s.unpackedPrim ? 1 : 0;
      anyPacked = anyPacked || s.packed;
    }
    if (unpacked > 0 && !anyPacked) {
      c.unsupportedObjects.emplace_back(obj.getName().c_str());
      os.entryCount = 0;
      names.finish(st, os); // no names: joints stop resolving to its bodies
      return;
    }
    c.unpackedPrimPoints += unpacked;
  }
  SlotPlans plans;
  bool planned = false;
  {
    UT_PerfMonAutoSolveEvent perf(
        progress.solver, "Pre-Solve}/{Updating Bodies}/{Planning Shapes");
    planned = planSlots(ctx, snaps, plans, progress);
  }
  if (!planned)
    return;
  reserveMirror(st, snaps);
  {
    UT_PerfMonAutoSolveEvent perf(
        progress.solver,
        "Pre-Solve}/{Updating Bodies}/{Checking Unchanged Points");
    stampQuietSlots(ctx, os, objectId, snaps, plans, checkProps);
  }
  ApplyPass apply(ctx, os, c, plans, names, objectId, checkProps);
  // Most of the world is new (a first frame, a rebuild): the bodies'
  // broad-phase proxies are put in the tree together at the end of the pass.
  struct BulkCreate {
    b3dop::Box3DWorld *world = nullptr;
    ~BulkCreate() {
      if (world)
        world->endBulkCreate();
    }
  } bulk;
  if (snaps.size() >= kMinBulkCreate && st.mirror.size() <= snaps.size() / 4) {
    bulk.world = &st.world;
    bulk.world->beginBulkCreate();
  }
  bool applied = false;
  {
    UT_PerfMonAutoSolveEvent perf(
        progress.solver, "Pre-Solve}/{Updating Bodies}/{Applying to Box3D");
    applied = apply.run(snaps, progress);
  }
  if (!applied)
    return;
  names.finish(st, os);
  os.restCacheId = restId; // every slot read split (or reused) at this id

  // Problems (retried and reported every step), deforming and driven bodies
  // keep it scanning.
  os.entryCount = apply.stamped;
  os.anyAgents = false;
  syncAgentBodies(st, os, objectId, gdp, snaps, a, node, c);
  os.fp = fpNow;
  os.propsKnown = true;
  os.anyDeforming = apply.anyDeforming;
  os.anyDriven = apply.anyDriven;
  os.canSkip = problemCount(c) == problems0 && !apply.anyDeforming &&
               !apply.anyDriven && !os.anyAgents;
}

} // namespace box3dsolver
