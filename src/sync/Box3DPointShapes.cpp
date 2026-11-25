// Box3DPointShapes.cpp
//
// A point's shapes: what the point and its primitive say about size, type and
// overrides, and the plan for building them (`planShapes`).
#include "Box3DSolverInternal.h"
#include <functional>
#include <string_view>

namespace box3dsolver {

// Point attributes that replace a Box3D Data value the user changed, on an
// object with a Box3D Data node. RBD Packed Object writes several of them onto
// every point from its own parameters (mass, density, computemass, friction,
// bounce in Houdini 22.0), so a Box3D Data value set for those does nothing;
// this makes that visible. A value left at its default is not reported: then
// the attribute is the only setting (the Box3D Packed Object writes the
// settings that differ from the defaults as attributes, and its Box3D Data
// keeps them all at the defaults). Reported once per object: on its first full
// scan, and again only if the set changes. (`active` is left out: it only takes
// over Body Type when 0.)
void noteOverriddenData(const GU_Detail *gdp, Box3DObjectSync &os,
                        SyncCounts &c, const b3dop::BodySettings &node) {
  const b3dop::BodySettings d = b3dop::nodeBodyDefaults();
  const b3dop::BodyProps &n = node.props, &p = d.props;
  auto differs = [](float a, float b) { return !b3dop::sameFloat(a, b); };
  const bool tangent = differs(n.tangentVelocity[0], p.tangentVelocity[0]) ||
                       differs(n.tangentVelocity[1], p.tangentVelocity[1]) ||
                       differs(n.tangentVelocity[2], p.tangentVelocity[2]);
  const std::pair<const char *, bool> overriding[] = {
      {"mass", differs(n.mass, p.mass)},
      {"density", differs(n.density, p.density)},
      {"computemass", n.computeMass != p.computeMass},
      {"computecom", n.computeCom != p.computeCom},
      {"friction", differs(n.friction, p.friction)},
      {"bounce", differs(n.restitution, p.restitution)},
      {"rollingresistance", differs(n.rollingResistance, p.rollingResistance)},
      {"tangentvelocity", tangent},
      {"lineardamping", differs(n.linearDamping, p.linearDamping)},
      {"angulardamping", differs(n.angularDamping, p.angularDamping)},
      {"gravityscale", differs(n.gravityScale, p.gravityScale)},
      {"enablesleep", n.enableSleep != p.enableSleep},
      {"awake", node.startAwake != d.startAwake},
      {"sleepthreshold", differs(n.sleepThreshold, p.sleepThreshold)},
      {"ccdthreshold", differs(n.ccdThreshold, p.ccdThreshold)},
      {"fastobject", n.fastObject != p.fastObject},
      {"contactrecycling", n.contactRecycling != p.contactRecycling},
      {"inertialtensorstiffness", differs(n.inertiaScale, p.inertiaScale)},
      {"bullet_georep", node.defaultShape != d.defaultShape},
      {"bullet_autofit", node.autoFit != d.autoFit},
      {"bullet_primS", node.prim.hasSize},
      {"bullet_radius", node.prim.hasRadius},
      {"bullet_length", node.prim.hasLength},
      {"bullet_primT", node.prim.hasPos},
      {"bullet_primR", node.prim.hasRot}};
  constexpr size_t kCount = sizeof overriding / sizeof overriding[0];
  bool hit[kCount] = {};
  std::string found;
  for (size_t i = 0; i < kCount; ++i)
    if (overriding[i].second && gdp->findPointAttribute(overriding[i].first)) {
      hit[i] = true;
      found += found.empty() ? "" : " ";
      found += overriding[i].first;
    }
  if (found == os.overridesReported)
    return;
  os.overridesReported = found;
  if (found.empty())
    return;
  ++c.overriddenObjects;
  for (size_t i = 0; i < kCount; ++i)
    if (hit[i])
      ++c.overriddenData[overriding[i].first];
}

namespace {

// Equal signatures mean identical shapes (hull keys, primitive numbers): skip
// the rebuild.
uint64_t shapeSignature(b3dop::ShapeType type, const BuiltShapes &s) {
  uint64_t h = b3dop::mixHash(
      0x9e3779b97f4a7c15ull, static_cast<uint64_t>(static_cast<int>(type)) + 1);
  for (const Piece &pc : s.pieces) {
    if (pc.mesh) {
      h = b3dop::mixHash(
          b3dop::mixHash(h, 3),
          b3dop::hashFloats(pc.mesh->xyz.data(), pc.mesh->xyz.size()));
      h = b3dop::hashBytes(h, pc.mesh->tris.data(),
                           pc.mesh->tris.size() * sizeof(int32_t));
    } else if (pc.isHull)
      h = b3dop::mixHash(b3dop::mixHash(h, 1), pc.hull);
    else {
      h = b3dop::mixHash(b3dop::mixHash(h, 2), b3dop::hashShapeSpec(pc.spec));
    }
  }
  return h ? h : 1;
}

// The explicit primitive attributes (used when there is no packed geometry to
// fit), over Box3D Data's hand-set shape.
b3dop::PrimOverrides readPrimOverrides(const PointAttrs &a, GA_Offset off,
                                       const b3dop::PrimOverrides &node) {
  b3dop::PrimOverrides ov = node;
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
    if (b3dop::finite3(v.data())) {
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

} // namespace

// A point's shapes from its current attributes and packed geometry. Only the
// hull cache is touched, no body.
ShapePlan planShapes(ScanContext &ctx, GA_Offset off, int64_t bodyId,
                     const PackedInfo &pk, bool staticBody) {
  const PointAttrs &a = ctx.a;
  ShapePlan plan;

  plan.type = ctx.node.defaultShape;
  const bool named = ctx.georep.get(a, off, plan.type); // else the default
  // A default hull on a plain point (no packed geometry to hull) is the
  // pscale box, without the "needs a packed primitive" note an explicit
  // bullet_georep gets.
  if (!named && !pk.prim &&
      (plan.type == b3dop::ShapeType::Convex ||
       plan.type == b3dop::ShapeType::Concave))
    plan.type = b3dop::ShapeType::Box;

  // `none`: a static, shapeless body (like Bullet, it still exists for
  // constraints).
  if (plan.type == b3dop::ShapeType::None) {
    plan.sig = shapeSignature(plan.type, plan.shapes());
    return plan;
  }

  float ps, sc[3];
  readPscaleScale(a, off, ps, sc);
  const bool autofit =
      a.autofit.isValid() ? (a.autofit.get(off) != 0) : ctx.node.autoFit;
  plan.plannedStatic = staticBody;
  const b3dop::PrimOverrides ov = readPrimOverrides(a, off, ctx.node.prim);

  // Instances of one packed source with the same scale, pivot, shape type and
  // overrides get the same shapes (the build reads the packed transform only
  // through its scale), and a scan's points are mostly such runs: the last
  // build is kept per thread and reused while the key matches, so the source
  // lookups (a lock on its geometry, its identity, the caches) are made once
  // for the run. Valid for one scan only (the epoch): the geometry cannot
  // change inside it.
  struct ShapeMemo {
    uint64_t epoch = 0;
    uint64_t key = 0;
    bool valid = false;
    std::shared_ptr<const BuiltShapes> built;
    uint64_t sig = 0;
  };
  thread_local ShapeMemo memo;
  double S[3];
  const bool memoable = pk.prim && b3dop::packedScale(pk.m4, S);
  uint64_t key = 0;
  if (memoable) {
    key = b3dop::mixHash(
        0x51ed270b1b4d3a5full,
        reinterpret_cast<uintptr_t>(pk.prim->sharedImplementation()));
    key =
        b3dop::mixHash(key, static_cast<uint64_t>(static_cast<int>(plan.type)));
    key = b3dop::hashBytes(key, S, sizeof(S));
    key = b3dop::hashBytes(key, pk.pivot, sizeof(pk.pivot));
    key = b3dop::hashBytes(key, &ps, sizeof(ps));
    key = b3dop::hashBytes(key, sc, sizeof(sc));
    key = b3dop::mixHash(
        key, (autofit ? 1u : 0u) | (staticBody ? 2u : 0u) |
                 (ov.hasSize ? 4u : 0u) | (ov.hasRot ? 8u : 0u) |
                 (ov.hasPos ? 16u : 0u) | (ov.hasRadius ? 32u : 0u) |
                 (ov.hasLength ? 64u : 0u));
    key = b3dop::hashBytes(key, ov.size, sizeof(ov.size));
    key = b3dop::hashBytes(key, ov.rot, sizeof(ov.rot));
    key = b3dop::hashBytes(key, ov.pos, sizeof(ov.pos));
    key = b3dop::hashBytes(key, &ov.radius, sizeof(ov.radius));
    key = b3dop::hashBytes(key, &ov.length, sizeof(ov.length));
  }
  if (memoable && memo.valid && memo.epoch == ctx.georep.generation &&
      memo.key == key) {
    plan.built = memo.built;
    plan.sig = memo.sig;
  } else {
    plan.built = std::make_shared<const BuiltShapes>(buildShapes(
        ctx.world, ctx.sources, plan.type, ctx.node.hullPerConnectedSet, ps, sc,
        pk, autofit, ov, staticBody, ctx.world.settings().ground));
    plan.sig = shapeSignature(plan.type, plan.shapes());
    if (memoable) {
      memo.epoch = ctx.georep.generation;
      memo.key = key;
      memo.valid = true;
      memo.built = plan.built;
      memo.sig = plan.sig;
    }
  }
  if (plan.shapes().note)
    B3D_DEBUG_LOG("  body id=%lld: %s", static_cast<long long>(bodyId),
                  plan.shapes().note);

  // Convex / concave follow scale changes (as in Houdini); primitives do not.
  plan.autoScale = (plan.type == b3dop::ShapeType::Convex ||
                    plan.type == b3dop::ShapeType::Concave) &&
                   sizeKey(pk.prim != nullptr, pk.m3, ps, sc, plan.sizeKey);
  return plan;
}

} // namespace box3dsolver
