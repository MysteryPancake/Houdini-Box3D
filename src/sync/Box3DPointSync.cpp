// Box3DPointSync.cpp
//
// One body per point: reading points, the change fingerprint, body settings, shape plans and
// syncObject(), which adds, updates and removes an object's bodies.
#include "Box3DSolverInternal.h"

namespace box3dsolver {

// Attributes whose data ids are the fingerprint's prop markers (propIds, in order). The shape
// attributes are left out on purpose: like Bullet, editing them alone changes nothing until
// bullet_autofit_valid = 0 or deforming = 1.
const char *const kPropAttribNames[] = {
    "mass", "friction", "bounce", "lineardamping", "angulardamping", "gravityscale",
    "enablesleep", "enabled", "awake", "deforming", "bullet_autofit_valid",
    "pscale", "scale",      // a plain point's convex / concave fallback box
    "collision_category", "collision_mask", "animated", "density", "computemass",
    "inertialtensorstiffness", "sleepthreshold", "ccdthreshold", "fastobject",
    "collision_group", "dead",
    "contactrecycling", "lock_linear", "lock_angular", "fastrotation",
    "rollingresistance", "tangentvelocity",
    "name",                 // joint anchors find bodies by it
};
static_assert(std::size(kPropAttribNames) == Box3DSyncFingerprint::kNumPropIds,
              "kPropAttribNames and Box3DSyncFingerprint::kNumPropIds must match");

// ---------------------------------------------------------------------------
// Point state
// ---------------------------------------------------------------------------

// P / rotation / v / w of one point (a packed point's rotation from its primitive). False if
// P is not finite. A non-finite v or w is read as zero (Box3D asserts on it, and a NaN
// velocity would spread to the body's position); *badVelocity says so.
bool
readPointState(const GU_Detail *gdp, const PointAttrs &a, GA_Offset off,
               const PackedInfo &pk, Box3DBodyPose &out, bool *badVelocity)
{
    if (badVelocity)
        *badVelocity = false;
    const UT_Vector3 p = gdp->getPos3(off);
    out.pos[0] = p.x();  out.pos[1] = p.y();  out.pos[2] = p.z();
    if (!std::isfinite(out.pos[0]) || !std::isfinite(out.pos[1]) || !std::isfinite(out.pos[2]))
        return false;

    out.rot[0] = out.rot[1] = out.rot[2] = 0.0f;  out.rot[3] = 1.0f;
    if (pk.prim)
    {
        for (int i = 0; i < 4; ++i) out.rot[i] = pk.rot[i];
        b3dop::sanitizeQuat(out.rot);
    }
    else if (a.orient.isValid())
    {
        const UT_QuaternionF q = a.orient.get(off);
        out.rot[0] = q.x();  out.rot[1] = q.y();  out.rot[2] = q.z();  out.rot[3] = q.w();
        b3dop::sanitizeQuat(out.rot);
    }

    out.lin[0] = out.lin[1] = out.lin[2] = 0.0f;
    out.ang[0] = out.ang[1] = out.ang[2] = 0.0f;
    auto finite3 = [](const UT_Vector3 &v) {
        return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
    };
    if (a.v.isValid())
    {
        const UT_Vector3 v = a.v.get(off);
        if (finite3(v))
        {
            out.lin[0] = v.x();  out.lin[1] = v.y();  out.lin[2] = v.z();
        }
        else if (badVelocity)
            *badVelocity = true;
    }
    if (a.w.isValid())
    {
        const UT_Vector3 w = a.w.get(off);
        if (finite3(w))
        {
            out.ang[0] = w.x();  out.ang[1] = w.y();  out.ang[2] = w.z();
        }
        else if (badVelocity)
            *badVelocity = true;
    }
    return true;
}

// Position, orientation or velocity differ beyond float noise (q and -q are the same rotation).
// The quaternions are compared component-wise after aligning signs: a dot-product test would
// only see rotations above ~0.16 degrees (dot = cos(theta / 2)).
bool
poseDiffers(const Box3DBodyPose &a, const Box3DBodyPose &b)
{
    const double eps = 1e-5;
    if (b3dop::quatsDiffer(a.rot, b.rot, eps))
        return true;
    for (int i = 0; i < 3; ++i)
        if (std::fabs(a.pos[i] - b.pos[i]) > eps || std::fabs(a.lin[i] - b.lin[i]) > eps ||
            std::fabs(a.ang[i] - b.ang[i]) > eps)
            return true;
    return false;
}

// The primitive list's data id: the marker for packed transforms (intrinsics have no attribute
// id). Bumping it is up to whoever edits a primitive; an edit that does not is picked up on the
// next scan instead.
GA_DataId
primitiveListDataId(const GU_Detail *gdp)
{
    return gdp->getNumPrimitives() > 0 ? gdp->getPrimitiveList().getDataId() : Box3DSyncFingerprint::kNoId;
}

// The object's change markers (Box3DSyncFingerprint). `a` must be bound on `gdp`.
Box3DSyncFingerprint
makeFingerprint(const GU_Detail *gdp, const PointAttrs &a, int nodeType, uint64_t nodeHash)
{
    // Packed geometry is assumed to keep its topology (a new instance gets a new `id`, which
    // the id / P markers show); plain points are checked for count changes.
    const bool trustTopology = gdp->getNumPrimitives() > 0;
    auto idOf = [](const auto &h) {
        return h.isValid() ? h.getAttribute()->getDataId() : Box3DSyncFingerprint::kNoId;
    };
    Box3DSyncFingerprint fp;
    fp.numPoints  = trustTopology ? -1 : int64_t(gdp->getNumPoints());
    fp.numOffsets = trustTopology ? -1 : int64_t(gdp->getNumPointOffsets());
    const GA_Attribute *P = gdp->getP();
    fp.pId        = P ? P->getDataId() : Box3DSyncFingerprint::kNoId;
    fp.orientId   = idOf(a.orient);
    fp.vId        = idOf(a.v);
    fp.wId        = idOf(a.w);
    fp.idId       = idOf(a.id);
    fp.activeId   = idOf(a.active);
    fp.nodeType   = nodeType;
    // By name: `a` has only the sync-critical handles bound at this point.
    for (int i = 0; i < Box3DSyncFingerprint::kNumPropIds; ++i)
    {
        const GA_Attribute *attr = gdp->findPointAttribute(kPropAttribNames[i]);
        fp.propIds[i] = attr ? attr->getDataId() : Box3DSyncFingerprint::kNoId;
    }
    fp.primsId  = primitiveListDataId(gdp);
    fp.nodeHash = nodeHash;
    return fp;
}

#ifdef BOX3DDOP_DEBUG
// Markers that differ, old -> new, for the debug log.
std::string
describeFingerprintDifference(const Box3DSyncFingerprint &was, const Box3DSyncFingerprint &now)
{
    std::string out;
    auto add = [&](const char *name, long long a, long long b) {
        if (a == b) return;
        char buf[96];
        std::snprintf(buf, sizeof(buf), "%s%s %lld->%lld", out.empty() ? "" : ", ", name, a, b);
        out += buf;
    };
    add("point count",  was.numPoints,  now.numPoints);
    add("offset range", was.numOffsets, now.numOffsets);
    add("P",       (long long)was.pId,      (long long)now.pId);
    add("orient",  (long long)was.orientId, (long long)now.orientId);
    add("v",       (long long)was.vId,      (long long)now.vId);
    add("w",       (long long)was.wId,      (long long)now.wId);
    add("id",      (long long)was.idId,     (long long)now.idId);
    add("active",  (long long)was.activeId, (long long)now.activeId);
    add("body type", was.nodeType, now.nodeType);
    for (int i = 0; i < Box3DSyncFingerprint::kNumPropIds; ++i)
        add(kPropAttribNames[i], (long long)was.propIds[i], (long long)now.propIds[i]);
    add("primitives", (long long)was.primsId, (long long)now.primsId);
    if (was.nodeHash != now.nodeHash)
        out += out.empty() ? "Box3D Data values" : ", Box3D Data values";
    return out;
}
#endif

// ---------------------------------------------------------------------------
// Body settings, shape planning, and mid-simulation updates
// ---------------------------------------------------------------------------

// Box3D asserts on negatives: clamp at 0, ignore non-finite.
float
nonNegative(float v, float fallback)
{
    return std::isfinite(v) ? std::max(v, 0.0f) : fallback;
}

// A point's changeable settings. Building and comparing both read through here, so an
// untouched point compares equal. (`awake` is not a setting: see startAwakeFor.)
Box3DBodyProps
readDynamicProps(const b3dop::BodySettings &node, const PointAttrs &a, GA_Offset off)
{
    Box3DBodyProps p;
    p.mass = node.mass;
    if (a.mass.isValid())
    {
        const float m = float(a.mass.get(off));
        if (m > 0.0f && std::isfinite(m))
            p.mass = m;
    }
    p.friction       = a.friction.isValid()       ? nonNegative(a.friction.get(off),       node.friction)       : node.friction;
    p.restitution    = a.bounce.isValid()         ? nonNegative(a.bounce.get(off),         node.restitution)    : node.restitution;
    p.rollingResistance = a.rollingResistance.isValid() ? nonNegative(a.rollingResistance.get(off), node.rollingResistance)
                                                        : node.rollingResistance;
    for (int i = 0; i < 3; ++i)
        p.tangentVelocity[i] = node.tangentVelocity[i];
    if (a.tangentVelocity.isValid())
    {
        const UT_Vector3 t = a.tangentVelocity.get(off);
        if (std::isfinite(t.x()) && std::isfinite(t.y()) && std::isfinite(t.z()))
            for (int i = 0; i < 3; ++i)
                p.tangentVelocity[i] = float(t(i));
    }
    p.linearDamping  = a.lineardamping.isValid()  ? nonNegative(a.lineardamping.get(off),  node.linearDamping)  : node.linearDamping;
    p.angularDamping = a.angulardamping.isValid() ? nonNegative(a.angulardamping.get(off), node.angularDamping) : node.angularDamping;
    p.gravityScale   = node.gravityScale;
    if (a.gravityscale.isValid())
    {
        const float g = a.gravityscale.get(off);
        if (std::isfinite(g)) p.gravityScale = g;      // may be negative
    }
    p.enableSleep = a.enablesleep.isValid() ? a.enablesleep.get(off) != 0 : node.enableSleep;
    p.enabled     = !a.enabled.isValid() || a.enabled.get(off) != 0;

    p.computeMass = a.computeMass.isValid() ? a.computeMass.get(off) != 0 : node.computeMass;
    p.density     = node.density;
    if (a.density.isValid())
    {
        const float d = a.density.get(off);
        if (d > 0.0f && std::isfinite(d)) p.density = d;
    }
    p.inertiaScale = 1.0f;
    if (a.inertiaStiffness.isValid())
    {
        const float k = a.inertiaStiffness.get(off);
        if (k > 0.0f && std::isfinite(k)) p.inertiaScale = k;
    }
    auto threshold = [&](const GA_ROHandleF &h) {
        if (!h.isValid()) return -1.0f;
        const float v = h.get(off);
        return (v >= 0.0f && std::isfinite(v)) ? v : -1.0f;
    };
    p.sleepThreshold = node.sleepThreshold;
    if (a.sleepThreshold.isValid())
    {
        const float t = threshold(a.sleepThreshold);
        if (t >= 0.0f) p.sleepThreshold = t;
    }
    p.ccdThreshold = node.ccdThreshold;
    if (a.ccdThreshold.isValid())
    {
        const float t = threshold(a.ccdThreshold);
        if (t >= 0.0f) p.ccdThreshold = t;
    }
    p.fastObject = a.fastObject.isValid() ? a.fastObject.get(off) != 0 : node.isBullet;
    p.fastRotation = a.fastRotation.isValid() && a.fastRotation.get(off) != 0;
    p.contactRecycling = a.contactRecycling.isValid() ? a.contactRecycling.get(off) != 0 : node.contactRecycling;
    if (a.collisionCategory.isValid())
        p.filter.category = uint64_t(a.collisionCategory.get(off));
    if (a.collisionMask.isValid())
        p.filter.mask = uint64_t(a.collisionMask.get(off));
    if (a.collisionGroup.isValid())
        p.filter.group = a.collisionGroup.get(off);
    if (a.lockLinear.isValid())
    {
        const UT_Vector3 l = a.lockLinear.get(off);
        for (int i = 0; i < 3; ++i) p.locks.linear[i] = l(i) != 0.0f;
    }
    if (a.lockAngular.isValid())
    {
        const UT_Vector3 l = a.lockAngular.get(off);
        for (int i = 0; i < 3; ++i) p.locks.angular[i] = l(i) != 0.0f;
    }
    return p;
}

// A string attribute read through its shared-string handles where it has them, so per-point work
// can be memoised by handle (an int) rather than by string.
struct StringAttr
{
    const GA_ROHandleS            *handle = nullptr;
    const GA_Attribute            *attr   = nullptr;
    const GA_AIFSharedStringTuple *aif    = nullptr;

    void bind(const GA_ROHandleS &h)
    {
        handle = &h;
        attr   = h.isValid() ? h.getAttribute() : nullptr;
        aif    = attr ? attr->getAIFSharedStringTuple() : nullptr;
    }
    bool valid() const { return attr != nullptr; }
    bool memoisable() const { return !attr || aif; }

    // The point's string handle; -1 for none (empty, or no attribute). Only with memoisable().
    int64_t id(GA_Offset off) const
    {
        if (!attr)
            return -1;
        const GA_StringIndexType h = aif->getHandle(attr, off, 0);
        return h == GA_INVALID_STRING_INDEX ? -1 : int64_t(h);
    }
    std::string str(GA_Offset off) const
    {
        if (!attr)
            return std::string();
        const char *v = handle->get(off);
        return v ? std::string(v) : std::string();
    }
};

// Box3D density: computemass -> density; else mass / volume (density if unusable).
float
boxDensity(const Box3DBodyProps &p, float volume)
{
    if (p.computeMass)
        return p.density;
    return (p.mass > 0.0f && std::isfinite(p.mass) && volume > 0.0f) ? p.mass / volume : p.density;
}

// The mass Houdini reports (written to `mass` with computemass).
float
effectiveMass(const Box3DBodyProps &p, float volume)
{
    return p.computeMass ? p.density * volume : p.mass;
}

// active -> the node's type; inactive -> static, or kinematic if animated; shapeless -> static.
b3dop::BodySettings::Type
wantedType(const b3dop::BodySettings &node, bool active, bool animated, bool shapeless)
{
    if (shapeless)
        return b3dop::BodySettings::Type::Static;
    if (active)
        return node.type;
    return animated ? b3dop::BodySettings::Type::Kinematic : b3dop::BodySettings::Type::Static;
}

// The body's mass after shapes are added / replaced or its type changes. As in Houdini RBD the
// point (pivot) is the centre of mass, so the shapes' centroid is moved there; inertia is
// scaled by inertialtensorstiffness. Static / kinematic bodies only get shape densities.
// setShapeDensity = false for shapes just created with boxDensity(p, volume).
void
applyBodyMass(b3dop::Box3DWorld &world, b3dop::BodyHandle h, const Box3DBodyProps &p, float volume,
              bool setShapeDensity)
{
    world.applyMassByHandle(h, boxDensity(p, volume), true, p.inertiaScale, setShapeDensity);
}

// `awake` for a new body: the point's value, else the Box3D Data default. (Box3D starts a body that
// may not sleep awake whatever this says.)
bool
startAwakeFor(const b3dop::BodySettings &node, const PointAttrs &a, GA_Offset off)
{
    return a.awake.isValid() ? a.awake.get(off) != 0 : node.startAwake;
}

// The node's settings with a point's values applied.
b3dop::BodySettings
settingsFor(const b3dop::BodySettings &node, const Box3DBodyProps &p, b3dop::BodySettings::Type type)
{
    b3dop::BodySettings b = node;
    b.type           = type;
    b.friction       = p.friction;
    b.restitution    = p.restitution;
    b.rollingResistance = p.rollingResistance;
    for (int i = 0; i < 3; ++i)
        b.tangentVelocity[i] = p.tangentVelocity[i];
    b.linearDamping  = p.linearDamping;
    b.angularDamping = p.angularDamping;
    b.gravityScale   = p.gravityScale;
    b.enableSleep    = p.enableSleep;
    b.startEnabled   = p.enabled;
    b.filter         = p.filter;
    b.locks          = p.locks;
    b.sleepThreshold = p.sleepThreshold;
    b.ccdThreshold   = p.ccdThreshold;
    b.isBullet       = p.fastObject;
    b.fastRotation   = p.fastRotation;
    b.contactRecycling = p.contactRecycling;
    return b;
}

// pscale / scale (a packed point's size is its transform; these only size its fallback box).
void
readPscaleScale(const PointAttrs &a, GA_Offset off, float &ps, float sc[3])
{
    ps = a.pscale.isValid() ? float(a.pscale.get(off)) : 1.0f;
    sc[0] = sc[1] = sc[2] = 1.0f;
    if (a.scale.isValid())
    {
        const UT_Vector3 s = a.scale.get(off);
        sc[0] = s.x();  sc[1] = s.y();  sc[2] = s.z();
    }
}

// Within 1e-4 relative (our write-back recomposes packed transforms with float noise).
bool
sameSizeKey(const double a[3], const double b[3])
{
    for (int i = 0; i < 3; ++i)
    {
        const double tol = 1e-4 * std::max(std::fabs(a[i]), std::fabs(b[i]));
        if (!(std::fabs(a[i] - b[i]) <= tol))
            return false;
    }
    return true;
}

// The scale convex / concave shapes depend on: packed row lengths, else pscale * scale.
bool
sizeKey(bool packed, const double m3[9], float ps, const float sc[3], double out[3])
{
    if (packed)
        return b3dop::matrixRowLengths(m3, out);
    for (int i = 0; i < 3; ++i)
        out[i] = double(ps) * double(sc[i]);
    return true;
}

// Equal signatures mean identical shapes (hull keys, primitive numbers): skip the rebuild.
uint64_t
shapeSignature(b3dop::ShapeType type, const BuiltShapes &s)
{
    uint64_t h = b3dop::mixHash(0x9e3779b97f4a7c15ull, uint64_t(int(type)) + 1);
    for (const Piece &pc : s.pieces)
    {
        if (pc.isHull)
            h = b3dop::mixHash(b3dop::mixHash(h, 1), pc.hull);
        else
        {
            const b3dop::ShapeSpec &sp = pc.spec;
            const float f[] = { float(int(sp.type)),
                                sp.halfExtents[0], sp.halfExtents[1], sp.halfExtents[2],
                                sp.radius, sp.halfLength,
                                sp.center[0], sp.center[1], sp.center[2],
                                sp.rot[0], sp.rot[1], sp.rot[2], sp.rot[3] };
            h = b3dop::mixHash(b3dop::mixHash(h, 2), b3dop::hashFloats(f, sizeof(f) / sizeof(f[0])));
        }
    }
    return h ? h : 1;
}

// The explicit primitive attributes (used when there is no packed geometry to fit).
b3dop::PrimOverrides
readPrimOverrides(const PointAttrs &a, GA_Offset off)
{
    b3dop::PrimOverrides ov;
    if (a.primS.isValid())
    {
        const UT_Vector3 v = a.primS.get(off);
        ov.hasSize = true;  ov.size[0] = v.x();  ov.size[1] = v.y();  ov.size[2] = v.z();
    }
    if (a.primR.isValid())
    {
        const UT_Vector3 v = a.primR.get(off);
        // Euler degrees, Houdini's XYZ rotate order. Non-finite: no override.
        if (SYSisFinite(v.x()) && SYSisFinite(v.y()) && SYSisFinite(v.z()))
        {
            const float deg[3] = { v.x(), v.y(), v.z() };
            box3dEulerToQuat(deg, ov.rot);
            ov.hasRot = true;
        }
    }
    if (a.primT.isValid())
    {
        const UT_Vector3 v = a.primT.get(off);
        ov.hasPos = true;  ov.pos[0] = v.x();  ov.pos[1] = v.y();  ov.pos[2] = v.z();
    }
    if (a.radius.isValid())  { ov.hasRadius = true;  ov.radius = a.radius.get(off); }
    if (a.length.isValid())  { ov.hasLength = true;  ov.length = a.length.get(off); }
    return ov;
}

// A point's shapes from its current attributes and packed geometry. Only the hull cache is
// touched, no body.
ShapePlan
planShapes(ScanContext &ctx, GA_Offset off, int64_t bodyId, const PackedInfo &pk)
{
    const PointAttrs &a = ctx.a;
    ShapePlan plan;

    plan.type = ctx.node.defaultShape;
    ctx.georep.get(a, off, plan.type);          // else the node's default

    // `none`: a static, shapeless body (like Bullet, it still exists for constraints).
    if (plan.type == b3dop::ShapeType::None)
    {
        plan.sig = shapeSignature(plan.type, plan.shapes);
        return plan;
    }

    float ps, sc[3];
    readPscaleScale(a, off, ps, sc);
    const bool autofit = a.autofit.isValid() ? (a.autofit.get(off) != 0) : ctx.node.autoFit;
    plan.shapes = buildShapes(ctx.world, ctx.sources, plan.type, ctx.node.hullPerConnectedSet, ps, sc, pk,
                              autofit, readPrimOverrides(a, off));
    if (plan.shapes.note)
        B3D_LOG("  body id=%lld: %s", (long long)bodyId, plan.shapes.note);
    plan.sig = shapeSignature(plan.type, plan.shapes);

    // Convex / concave follow scale changes (as in Houdini); primitives do not.
    plan.autoScale = (plan.type == b3dop::ShapeType::Convex || plan.type == b3dop::ShapeType::Concave) &&
                     sizeKey(pk.prim != nullptr, pk.m3, ps, sc, plan.sizeKey);
    return plan;
}

// Adds the pieces without mass updates (the caller calls applyBodyMass). Returns the count.
int
addShapes(b3dop::Box3DWorld &world, b3dop::BodyHandle h, const b3dop::BodySettings &b, const BuiltShapes &shapes)
{
    int added = 0;
    for (const Piece &pc : shapes.pieces)
        added += (pc.isHull ? world.createHullShape(h, b, pc.hull, false)
                            : world.createShape(h, b, pc.spec, false)) ? 1 : 0;
    return added;
}

// A body with settings `b` at `pose` (its user data `entry`), its `shapes` and mass, then its
// initial velocity when `giveVelocity`: after the mass is final (a moving centre of mass shifts
// the velocity), and not for a static body or one that starts asleep or disabled (a velocity
// would wake it, so `v` only takes effect once something wakes it). A shapeless body (`none`)
// gets no shapes. 0 on failure, leaving no body behind.
b3dop::BodyHandle
createBodyWithShapes(b3dop::Box3DWorld &world, const b3dop::BodySettings &b, const Box3DBodyPose &pose,
                     const BuiltShapes &shapes, bool shapeless, const Box3DBodyProps &p, bool giveVelocity,
                     Box3DBodyMirror &entry)
{
    const b3dop::BodyHandle h = world.createBody(b, pose.pos, pose.rot, &entry);
    if (!h || shapeless)
        return h;
    if (addShapes(world, h, b, shapes) == 0)
    {
        world.destroyBody(h);           // never a shapeless body, except `none`
        return 0;
    }
    applyBodyMass(world, h, p, shapes.totalVolume, false);
    const bool hasVel = !std::all_of(pose.lin, pose.lin + 3, [](float v) { return v == 0.0f; }) ||
                        !std::all_of(pose.ang, pose.ang + 3, [](float v) { return v == 0.0f; });
    if (giveVelocity && hasVel && b.type != b3dop::BodySettings::Type::Static && world.isAwakeByHandle(h))
        world.setVelocityByHandle(h, pose.lin, pose.ang);
    return h;
}

// What the body was built from, into its mirror entry.
void
storePlan(Box3DBodyMirror &m, const ShapePlan &plan)
{
    m.shapeSig  = plan.sig;
    m.volume    = plan.shapes.totalVolume;
    m.autoScale = plan.autoScale;
    for (int i = 0; i < 3; ++i)
        m.sizeKey[i] = plan.sizeKey[i];
}

// The slot's plan from the parallel pre-pass (moved out), else made now.
ShapePlan
takePlan(ScanContext &ctx, const Box3DPointSlot &s, std::vector<ShapePlan> &plans)
{
    if (s.plan >= 0)
        return std::move(plans[size_t(s.plan)]);
    return planShapes(ctx, s.off, s.bid, ctx.packedInfo(s.off));
}

// Create the body for slot `s` (settings `p`), its shapes and initial velocity. `entry` becomes
// the body's user data and records what it was built from. Returns 0 on failure.
b3dop::BodyHandle
addPointBody(ScanContext &ctx, const Box3DPointSlot &s, const ShapePlan &plan, bool animated,
             const Box3DBodyProps &p, Box3DBodyMirror &entry, std::string &fallbackNote)
{
    b3dop::Box3DWorld &world = ctx.world;
    const Box3DBodyPose &st  = s.cur;
    if (plan.shapes.note)
        fallbackNote = plan.shapes.note;
    entry.props     = p;
    entry.shapeless = (plan.type == b3dop::ShapeType::None);
    entry.animated  = animated;
    storePlan(entry, plan);

    b3dop::BodySettings b = settingsFor(ctx.node, p, wantedType(ctx.node, s.active, animated, entry.shapeless));
    b.density    = boxDensity(p, plan.shapes.totalVolume);
    b.startAwake = startAwakeFor(ctx.node, ctx.a, s.off);
    entry.awake  = b.startAwake;

    // Animated bodies follow their point instead of starting with its velocity.
    return createBodyWithShapes(world, b, st, plan.shapes, entry.shapeless, p, !animated, entry);
}

// Rebuild a body's shapes from current attributes (deforming, bullet_autofit_valid = 0, or a
// convex / concave scale change), keeping the body. Identical shapes: nothing happens. Returns
// true if replaced; `noShape` if none could be added (the body is then static).
bool
rebuildPointShapes(ScanContext &ctx, const Box3DPointSlot &s, const ShapePlan &plan,
                   const Box3DBodyProps &p, Box3DBodyMirror &entry, std::string &note, bool &noShape)
{
    b3dop::Box3DWorld &world = ctx.world;
    noShape = false;
    if (plan.shapes.note)
        note = plan.shapes.note;
    if (plan.sig == entry.shapeSig)
    {
        storePlan(entry, plan);     // same shapes; refresh the size key
        return false;
    }

    const b3dop::BodyHandle h = entry.handle;
    world.destroyShapesByHandle(h);
    storePlan(entry, plan);
    entry.shapeless = true;
    if (plan.type != b3dop::ShapeType::None)
    {
        b3dop::BodySettings b = settingsFor(ctx.node, p, entry.type);
        b.density = boxDensity(p, plan.shapes.totalVolume);
        if (addShapes(world, h, b, plan.shapes) > 0)
            entry.shapeless = false;
        else
            noShape = true;
    }
    applyBodyMass(world, h, p, plan.shapes.totalVolume, false);    // even with no shapes: clears Box3D's dirty mass
    world.wakeByHandle(h);          // let it re-settle
    B3D_LOG_VERBOSE("  body id=%lld: collision shape rebuilt", (long long)s.bid);
    return true;
}

// Push the settings that differ. After a rebuild the new shapes already carry material, mass
// and filter profile. Returns kPropsChanged / kMassChanged.
enum : unsigned { kPropsChanged = 1, kMassChanged = 2 };

unsigned
applyPropChanges(b3dop::Box3DWorld &world, Box3DBodyMirror &e, const Box3DBodyProps &p, bool shapesRebuilt)
{
    const Box3DBodyProps &o = e.props;
    const b3dop::BodyHandle h = e.handle;
    unsigned changed = shapesRebuilt ? unsigned(kPropsChanged) : 0u;
    const bool massChanged = p.mass != o.mass || p.computeMass != o.computeMass || p.density != o.density ||
                             p.inertiaScale != o.inertiaScale;
    if (!shapesRebuilt)
    {
        if (p.friction != o.friction || p.restitution != o.restitution)
        {
            world.setMaterialByHandle(h, p.friction, p.restitution);
            changed |= kPropsChanged;
        }
        if (massChanged)
        {
            applyBodyMass(world, h, p, e.volume);
            changed |= kPropsChanged | kMassChanged;
        }
        if (p.filter != o.filter)
        {
            world.setFilterByHandle(h, p.filter);
            changed |= kPropsChanged;
        }
    }
    else if (massChanged)
        changed |= kMassChanged;
    if (p.linearDamping != o.linearDamping || p.angularDamping != o.angularDamping)
    {
        world.setDampingByHandle(h, p.linearDamping, p.angularDamping);
        changed |= kPropsChanged;
    }
    if (p.gravityScale != o.gravityScale)
    {
        world.setGravityScaleByHandle(h, p.gravityScale);
        changed |= kPropsChanged;
    }
    if (p.ccdThreshold != o.ccdThreshold)
    {
        world.setCcdThresholdByHandle(h, p.ccdThreshold);
        changed |= kPropsChanged;
    }
    if (p.sleepThreshold != o.sleepThreshold)
    {
        world.setSleepThresholdByHandle(h, p.sleepThreshold);
        changed |= kPropsChanged;
    }
    if (p.fastRotation != o.fastRotation)
    {
        world.setFastRotationByHandle(h, p.fastRotation);
        changed |= kPropsChanged;
    }
    if (p.fastObject != o.fastObject)
    {
        world.setBulletByHandle(h, p.fastObject);
        changed |= kPropsChanged;
    }
    if (p.rollingResistance != o.rollingResistance || !std::equal(p.tangentVelocity, p.tangentVelocity + 3, o.tangentVelocity))
    {
        world.setSurfaceByHandle(h, p.rollingResistance, p.tangentVelocity);
        changed |= kPropsChanged;
    }
    if (p.locks != o.locks)
    {
        world.setMotionLocksByHandle(h, p.locks);
        changed |= kPropsChanged;
    }
    if (p.contactRecycling != o.contactRecycling)
    {
        world.setContactRecyclingByHandle(h, p.contactRecycling);
        changed |= kPropsChanged;
    }
    if (p.enableSleep != o.enableSleep)
    {
        world.setSleepEnabledByHandle(h, p.enableSleep);
        changed |= kPropsChanged;
    }
    if (p.enabled != o.enabled)
    {
        world.setEnabledByHandle(h, p.enabled);
        if (p.enabled)
            e.awake = true;             // Box3D enables a body awake; the awake check after this applies a 0
        changed |= kPropsChanged;
    }
    e.props = p;
    return changed;
}

// Queue a point attribute write for after the step.
void
queuePointWrite(Box3DSharedState &st, Box3DBodyMirror &m, unsigned flags, float mass)
{
    st.pointWrites.emplace_back();
    Box3DPointWrite &w = st.pointWrites.back();
    w.entry    = &m;
    w.objectId = m.objectId;
    w.flags    = flags;
    w.mass     = mass;
}

// The point's packed primitive, as pass 1 read it, for the write-back.
void
storePacked(Box3DBodyMirror &m, const Box3DPointSlot &s)
{
    m.packed  = s.packed;
    m.primOff = s.primOff;
    if (s.packed)
    {
        std::copy(s.m3, s.m3 + 9, m.m3);
        std::copy(s.rowScale, s.rowScale + 3, m.rowScale);
    }
}

// Box3D Data values that change existing bodies (the fingerprint's nodeHash).
uint64_t
nodeSettingsHash(const b3dop::BodySettings &n, bool addImpact)
{
    const float v[] = { n.mass, n.friction, n.restitution, n.linearDamping, n.angularDamping, n.gravityScale,
                        n.density, n.computeMass ? 1.0f : 0.0f, n.sleepThreshold, n.isBullet ? 1.0f : 0.0f,
                        n.contactRecycling ? 1.0f : 0.0f, n.ccdThreshold, n.enableSleep ? 1.0f : 0.0f,
                        n.rollingResistance, n.tangentVelocity[0], n.tangentVelocity[1], n.tangentVelocity[2] };
    return b3dop::mixHash(b3dop::hashFloats(v, sizeof(v) / sizeof(v[0])), addImpact ? 1 : 2);
}

// The mid-simulation changes of an existing body (AGENTS.md, "Mid-simulation updates"): shape
// rebuild and settings, type, then the animated target or an external edit of its pose.
// Adds to `writes` the point attributes to write after the step.
void
updateExistingBody(ScanContext &ctx, Box3DSharedState &st, Box3DPointSlot &s, Box3DBodyMirror &entry,
                   bool animated, bool checkProps, std::vector<ShapePlan> &plans, SyncCounts &c,
                   unsigned &writes, bool &anyDriven)
{
    b3dop::Box3DWorld &world = ctx.world;
    const PointAttrs  &a     = ctx.a;
    const GA_Offset    off   = s.off;

    // Shape first (a shapeless body is static); pass 1 decided whether to rebuild.
    const bool rebuild = s.rebuild;
    if (rebuild || checkProps)
    {
        Box3DBodyProps p = ctx.props(off);
        bool shapesRebuilt = false;
        if (rebuild)
        {
            std::string note;
            bool        noShape = false;
            shapesRebuilt = rebuildPointShapes(ctx, s, takePlan(ctx, s, plans), p, entry, note, noShape);
            if (shapesRebuilt)
            {
                ++c.rebuilt;
                if (!note.empty())
                    ++c.fallbacks[note];
                if (noShape)
                    ++c.fallbacks["the rebuilt collision shape could not be created; the body is static until it gets one"];
            }
        }
        const unsigned changed = applyPropChanges(world, entry, p, shapesRebuilt);
        if (changed)
            ++c.updated;
        if (p.computeMass && (shapesRebuilt || (changed & kMassChanged)))
            writes |= Box3DPointWrite::kMass;
        // `awake` written since the solver last read or wrote it: 1 wakes the body, 0 puts it to sleep
        // (with whatever touches it). A disabled body is skipped until it is enabled.
        if (a.awake.isValid() && p.enabled)
        {
            const bool want = a.awake.get(off) != 0;
            if (want != entry.awake)
            {
                if (want)
                    world.wakeByHandle(entry.handle);
                else
                    world.sleepByHandle(entry.handle);
                entry.awake = want;
            }
        }
    }

    // Did `active` / `animated` flip?
    const b3dop::BodySettings::Type wantType = wantedType(ctx.node, s.active, animated, entry.shapeless);
    if (entry.type != wantType)
    {
        world.setTypeByHandle(entry.handle, wantType);
        entry.type   = wantType;
        entry.driven = false;
        // A type change recomputes the mass from the shapes: centre back to the point.
        if (entry.dynamic())
            applyBodyMass(world, entry.handle, entry.props, entry.volume);
        ++c.retyped;
    }
    entry.animated = animated;

    const Box3DBodyPose &cur = s.cur;
    if (animated && entry.type == b3dop::BodySettings::Type::Kinematic)
    {
        // Follow the point: a new pose becomes this step's target; when it stops, clear the
        // velocity the target left.
        if (poseDiffers(cur, entry))
        {
            world.setTargetTransformByHandle(entry.handle, cur.pos, cur.rot, st.stepDt);
            static_cast<Box3DBodyPose &>(entry) = cur;
            entry.driven = true;
            ++c.edited;
        }
        else if (entry.driven)
        {
            const float zero[3] = { 0.0f, 0.0f, 0.0f };
            world.setVelocityByHandle(entry.handle, zero, zero);
            entry.driven = false;
        }
        anyDriven = anyDriven || entry.driven;
    }
    // Did something else edit P / orient / v / w since we wrote it?
    else if (poseDiffers(cur, entry))
    {
        world.setTransformByHandle(entry.handle, cur.pos, cur.rot);
        world.setVelocityByHandle(entry.handle, cur.lin, cur.ang);
        static_cast<Box3DBodyPose &>(entry) = cur;
        ++c.edited;
    }
}

// Sync one object's points into the world. Returns early (object half synced) if the user
// cancels: the caller then discards the world (see solveObjectsSubclass).
void
syncObject(Box3DSharedState &st, const SIM_Object &obj, SyncCounts &c, StepProgress &progress)
{
    const SIM_Geometry *geo = SIM_DATA_GETCONST(obj, SIM_GEOMETRY_DATANAME, SIM_Geometry);
    if (!geo)
    {
        B3D_LOG_VERBOSE("  object id=%d has no Geometry data, skipped", obj.getObjectId());
        return;
    }
    SIM_GeometryAutoReadLock lock(geo);
    const GU_Detail *gdp = lock.getGdp();
    if (!gdp)
        return;
    if (isWholeObject(obj))
    {
        syncWholeObject(st, obj, *geo, gdp, c);
        return;
    }

    PointAttrs a(gdp);
    const int  objectId = obj.getObjectId();
    const SIM_Box3DData *dataNode = SIM_DATA_GETCONST(obj, kBox3DDataName, SIM_Box3DData);
    const b3dop::BodySettings node = dataNode ? dataNode->toBodySettings() : b3dop::BodySettings();
    const bool addImpact = dataNode && dataNode->getAddImpact();

    // ---- unchanged since we last finished with it? ---------------------------
    // Markers are recorded after a scan and again after our own writes. Same markers: no point
    // can differ from the mirror, so both passes are skipped and the entries stay current
    // under liveStamp.
    Box3DObjectSync &os = st.objectSync[objectId];
    os.visited   = st.syncStamp;
    os.addImpact = addImpact;
    os.wholeObject = false;
    const Box3DSyncFingerprint fpNow = makeFingerprint(gdp, a, int(node.type), nodeSettingsHash(node, addImpact));
    if (os.canSkip && fpNow == os.fp && (!st.wantNames || os.namesKnown))
    {
        ++c.skipped;
        st.stampedCount += os.entryCount;
        return;
    }
#ifdef BOX3DDOP_DEBUG
    if (os.canSkip)
        B3D_LOG_VERBOSE("  object id=%d: full scan, changed: %s", objectId,
                describeFingerprintDifference(os.fp, fpNow).c_str());
#endif
    os.canSkip   = false;
    os.liveStamp = st.syncStamp;
    size_t stamped = 0;                 // entries stamped by this scan
    const int problems0 = c.failed + c.duplicates + c.badP;

    a.bindBuildAttrs(gdp);
    ScanContext ctx(st, gdp, a, node);
    const bool anyPrims = ctx.anyPrims;

    // Settings are compared only when a prop marker changed (or deforming bodies need a rebuild
    // anyway); otherwise the scan syncs pose, velocity and type. Convex / concave scale is
    // checked on every scan.
    const bool checkProps = !os.propsKnown || !os.fp.samePropMarkers(fpNow) || os.anyDeforming;
    bool       anyDeforming = false;
    bool       anyDriven    = false;

    // For the post-step passes; kept as they are while the object is skipped.
    os.bodies.clear();
    // Bodies by `name`, for joint anchors (only while some object has a joint network). The old
    // map is kept to see whether a rename changed what the names resolve to.
    std::unordered_map<std::string, Box3DBodyMirror *> oldByName;
    oldByName.swap(os.byName);
    os.byName.clear();
    os.namesKnown = st.wantNames;
    const GA_ROHandleS nameH(gdp, GA_ATTRIB_POINT, "name");
    StringAttr         names;
    names.bind(nameH);
    const bool         keepNames = st.wantNames && names.valid();

    // ---- pass 1 (parallel): read every point ---------------------------------
    // Offsets first (serial); a slot forgets its cached entry when its offset or key changed.
    // Without `id` the body id is the point number, i.e. the position in this walk.
    std::vector<Box3DPointSlot> &snaps = st.slots[objectId];
    {
        size_t n = 0;
        for (GA_Iterator it(gdp->getPointRange()); !it.atEnd(); ++it, ++n)
        {
            if (n == snaps.size())
                snaps.emplace_back();
            Box3DPointSlot &s = snaps[n];
            if (s.off != *it)
                s.entry = nullptr;
            s.off = *it;
            s.bid = int64_t(n);
            s.index = int64_t(n);
        }
        snaps.resize(n);
    }

    parallelForEach(snaps.size(), [&](size_t i) {
        Box3DPointSlot &s = snaps[i];
        if (a.id.isValid())
            s.bid = int64_t(a.id.get(s.off));
        const uint64_t key = b3dop::makeBodyKey(objectId, s.bid);
        if (key != s.key)
            s.entry = nullptr;
        s.key = key;
        const PackedInfo pk = anyPrims ? getPackedInfo(gdp, s.off) : PackedInfo();
        s.ok     = readPointState(gdp, a, s.off, pk, s.cur, &s.badVelocity);
        s.active = !a.active.isValid() || a.active.get(s.off) != 0;
        s.packed = pk.prim != nullptr;
        if (s.packed)
        {
            s.primOff = int64_t(pk.primOff);
            std::copy(pk.m3, pk.m3 + 9, s.m3);
            std::copy(pk.scale, pk.scale + 3, s.rowScale);
        }

        // Does the point need shapes built: a new body, or a rebuild (deforming,
        // bullet_autofit_valid = 0, or a convex / concave scale change, like Bullet)? The mirror
        // is only read here (pass 2 changes it).
        s.plan        = -1;
        s.deforming   = a.deforming.isValid() && a.deforming.get(s.off) != 0;
        s.refitWanted = a.autofitValid.isValid() && a.autofitValid.get(s.off) == 0;
        s.rebuild     = false;
        if (!s.ok || (a.dead.isValid() && a.dead.get(s.off) != 0))
            return;
        if (!s.entry)
        {
            const auto it = st.mirror.find(key);
            if (it != st.mirror.end())
                s.entry = &it->second;
        }
        if (const Box3DBodyMirror *e = s.entry)
        {
            s.rebuild = s.deforming || s.refitWanted;
            if (!s.rebuild && e->autoScale)
            {
                float  ps, sc[3];
                double sk[3];
                readPscaleScale(a, s.off, ps, sc);
                s.rebuild = sizeKey(s.packed, s.m3, ps, sc, sk) && !sameSizeKey(sk, e->sizeKey);
            }
        }
    }, anyPrims ? kMinParallelReadPacked : kMinParallelReadPlain);

    // ---- pass 1b (parallel): plan the shapes of new and rebuilt bodies -----------
    // Building hulls is the slow part of a new world and of deforming bodies. Box3DWorld's hull
    // calls, ShapeSources and GeorepLookup are thread-safe, so the plans are made here in
    // parallel and pass 2 only creates bodies. Chunked, so Escape is checked in between.
    std::vector<ShapePlan> plans;
    {
        std::vector<size_t> want;
        for (size_t i = 0; i < snaps.size(); ++i)
            if (snaps[i].ok && (!snaps[i].entry ? !(a.dead.isValid() && a.dead.get(snaps[i].off) != 0)
                                                : snaps[i].rebuild))
                want.push_back(i);
        plans.resize(want.size());
        for (size_t c0 = 0; c0 < want.size(); c0 += kPlanChunk)
        {
            if (progress.check())
                return;
            const size_t c1 = std::min(want.size(), c0 + kPlanChunk);
            parallelForEach(c1 - c0, [&](size_t j) {
                Box3DPointSlot &s = snaps[want[c0 + j]];
                plans[c0 + j] = planShapes(ctx, s.off, s.bid, ctx.packedInfo(s.off));
                s.plan = int32_t(c0 + j);
            }, 2);
        }
    }

    // Many new bodies (a fresh world): size the mirror once.
    {
        size_t fresh = 0;
        for (const Box3DPointSlot &s : snaps)
            fresh += s.entry ? 0 : 1;
        if (fresh >= 64)
            st.mirror.reserve(st.mirror.size() + fresh);
    }

    // ---- pass 2 (serial, point order): apply to Box3D and the mirror ----------
    // An entry already carrying this step's stamp belongs to an earlier point (duplicate id).
    // Escape is checked here, between points (the hulls were built by the pre-pass).
    size_t pointIndex = 0;
    for (Box3DPointSlot &s : snaps)
    {
        if (pointIndex++ % kProgressEvery == 0 && progress.check())
            return;

        const GA_Offset off = s.off;

        Box3DBodyMirror *entry = s.entry;
        bool freshEntry = false;        // `entry` was just inserted, no body yet
        auto newIt = st.mirror.end();   // its node, to erase if the body cannot be built
        if (!entry)
        {
            if (s.ok)
            {
                // One lookup for "known?" and "make room"; a new node is unstamped.
                auto res   = st.mirror.try_emplace(s.key);
                entry      = &res.first->second;
                freshEntry = res.second;
                if (freshEntry)
                    newIt = res.first;
            }
            else
            {
                auto mit = st.mirror.find(s.key);   // a bad point never creates a body
                if (mit != st.mirror.end())
                    entry = &mit->second;
            }
            s.entry = entry;
        }
        // Drops a just-inserted entry that will get no body.
        auto dropFresh = [&]() {
            if (freshEntry)
            {
                st.mirror.erase(newIt);
                s.entry = nullptr;
            }
        };

        if (entry && entry->stamp == st.syncStamp)
        {
            ++c.duplicates;         // two points with the same `id`: first wins
            continue;
        }

        if (!s.ok)
        {
            ++c.badP;               // not stamped: the body (if any) is removed
            continue;
        }
        if (s.badVelocity)
            ++c.badVelocity;        // simulated, with the bad v / w read as zero

        // `dead`: deleted after the step. Not stamped, so its body goes with the gone points'.
        if (a.dead.isValid() && a.dead.get(off) != 0)
        {
            Box3DDeletion d;
            d.objectId = objectId;
            d.off      = off;
            d.pointId  = s.bid;
            st.deletions.push_back(d);
            dropFresh();
            ++c.dead;
            continue;
        }

        const bool animated = !s.active && a.animated.isValid() && a.animated.get(off) != 0;
        anyDeforming = anyDeforming || s.deforming;
        unsigned writes = s.refitWanted ? Box3DPointWrite::kAutofitValid : 0u;

        if (freshEntry)
        {
            const Box3DBodyProps p = ctx.props(off);
            std::string fallbackNote;
            const b3dop::BodyHandle handle = addPointBody(ctx, s, takePlan(ctx, s, plans), animated, p, *entry,
                                                          fallbackNote);
            if (!handle)
            {
                ++c.failed;
                dropFresh();
                continue;
            }
            if (!fallbackNote.empty())
                ++c.fallbacks[fallbackNote];
            static_cast<Box3DBodyPose &>(*entry) = s.cur;
            entry->type     = wantedType(node, s.active, animated, entry->shapeless);
            entry->handle   = handle;
            entry->driven   = false;
            if (p.computeMass)
                writes |= Box3DPointWrite::kMass;
            ++c.added;
        }

        // Every body this scan keeps: its point as read now, for the write-back and later scans.
        entry->pointOff   = off;
        entry->pointId    = s.bid;
        entry->pointIndex = s.index;
        entry->objectId   = objectId;
        entry->stamp      = st.syncStamp;
        entry->sync       = &os;
        storePacked(*entry, s);
        ++st.stampedCount;
        ++stamped;

        if (!freshEntry)
            updateExistingBody(ctx, st, s, *entry, animated, checkProps, plans, c, writes, anyDriven);

        if (writes)
            queuePointWrite(st, *entry, writes, effectiveMass(entry->props, entry->volume));
        os.bodies.push_back(entry);
        if (keepNames)
        {
            std::string n = names.str(off);
            if (!n.empty())
                os.byName.emplace(std::move(n), entry);     // the first point of a name wins
        }
    }

    // Joint anchors resolve by point `name`: a renamed piece re-resolves the joints.
    if (keepNames && os.byName != oldByName)
        ++st.bodyVersion;

    // Problems (retried and reported every step), deforming and driven bodies keep it scanning.
    os.entryCount   = stamped;
    os.fp           = fpNow;
    os.propsKnown   = true;
    os.anyDeforming = anyDeforming;
    os.anyDriven    = anyDriven;
    os.canSkip      = (c.failed + c.duplicates + c.badP) == problems0 && !anyDeforming && !anyDriven;
}

} // namespace box3dsolver
