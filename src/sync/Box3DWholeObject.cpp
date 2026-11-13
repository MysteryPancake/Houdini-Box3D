// Box3DWholeObject.cpp
//
// One body per DOP object (RBD Object, RBD Point Object, ...): its pose in the object's
// Position data. Sync and write-back (AGENTS.md, "Whole-object bodies").
#include "Box3DSolverInternal.h"
#include "core/Box3DRbdObject.h"

#include <type_traits>

namespace box3dsolver {

// ---------------------------------------------------------------------------
// Whole-object bodies (RBD Object, RBD Point Object, ...)
// ---------------------------------------------------------------------------

// The point id of a whole object's single body in the mirror key.
constexpr int64_t kWholeObjectId = -1;

// True for an object that is one rigid body as a whole rather than one body per point: it has a
// collider label (Colliders data, as RBD Object and its relatives attach) other than
// PackedObject (RBD Packed Object). An object without one (an Empty Object with a SOP Geometry)
// keeps its points as bodies.
bool
isWholeObject(const SIM_Object &obj)
{
    const SIM_ColliderLabel *label = SIM_DATA_GETCONST(obj, SIM_COLLIDERS_DATANAME, SIM_ColliderLabel);
    if (!label)
        return false;
    const UT_StringHolder name = label->getColliderLabel();
    return !(name.isstring() && std::strcmp(name.c_str(), SIM_COLLIDERLABEL_PACKEDOBJECT) == 0);
}

// The body's pose from the object's Position data: the world position of the pivot and the
// orientation, plus the velocities of a SIM_Motion (RBD Object's RBD State is one). False if the
// object has no Position data (pose and pivot untouched then).
bool
readWholePose(const SIM_Object &obj, double pivot[3], Box3DBodyPose &out)
{
    const SIM_Position *pos = SIM_DATA_GETCONST(obj, SIM_POSITION_DATANAME, SIM_Position);
    if (!pos)
        return false;
    UT_Vector3 pv(0.0f, 0.0f, 0.0f);
    pos->getPivot(pv);
    UT_DMatrix4 xf;
    pos->getTransform(xf);
    double m[16];
    box3dToArray(xf, m);
    const double pvd[3] = { pv.x(), pv.y(), pv.z() };
    double wp[3];
    b3dop::xformPoint(m, pvd, wp);
    if (!std::isfinite(wp[0]) || !std::isfinite(wp[1]) || !std::isfinite(wp[2]))
        return false;
    for (int i = 0; i < 3; ++i)
    {
        pivot[i]   = pvd[i];
        out.pos[i] = wp[i];
    }
    UT_Quaternion q;
    pos->getOrientation(q);
    out.rot[0] = float(q.x());  out.rot[1] = float(q.y());  out.rot[2] = float(q.z());  out.rot[3] = float(q.w());
    b3dop::sanitizeQuat(out.rot);
    for (int i = 0; i < 3; ++i)
        out.lin[i] = out.ang[i] = 0.0f;
    if (const SIM_Motion *mo = SIM_DATA_GETCONST(obj, SIM_POSITION_DATANAME, SIM_Motion))
    {
        const UT_Vector3 v = mo->getVelocity();
        const UT_Vector3 w = mo->getAngularVelocity();      // radians per second
        const float lin[3] = { v.x(), v.y(), v.z() }, ang[3] = { w.x(), w.y(), w.z() };
        if (b3dop::finite3(lin) && b3dop::finite3(ang))
            for (int i = 0; i < 3; ++i)
            {
                out.lin[i] = lin[i];
                out.ang[i] = ang[i];
            }
    }
    return true;
}

// The RBD Object's own settings (used without Box3D Data): mass from its RBD State (Position),
// friction and bounce from PhysicalParms, the shape from its geometry's BulletData. Each option is
// read if the data has it (b3dop::RbdObjectSettings names them).
b3dop::RbdObjectSettings
readRbdObjectSettings(const SIM_Object &obj, const SIM_Geometry &geo)
{
    b3dop::RbdObjectSettings r;
    auto number = [](const SIM_Data *d, const char *name, bool &has, auto &out) {
        fpreal64 v = 0.0;
        if (d && box3dDataOption(*d, name, v) && std::isfinite(v))
        {
            has = true;
            out = static_cast<std::remove_reference_t<decltype(out)>>(v);
        }
    };
    auto toggle = [](const SIM_Data *d, const char *name, bool &has, bool &out) {
        fpreal64 v = 0.0;
        if (d && box3dDataOption(*d, name, v))
        {
            has = true;
            out = v != 0.0;
        }
    };
    auto vec = [](const SIM_Data *d, const char *name, float out[3]) {
        UT_Vector3D v;
        if (d && box3dDataOption(*d, name, v) && std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z()))
            for (int i = 0; i < 3; ++i)
                out[i] = float(v(i));
    };

    const SIM_Data *state = obj.getConstNamedSubData(SIM_POSITION_DATANAME);
    toggle(state, "computemass", r.hasComputeMass, r.computeMass);
    number(state, "density", r.hasDensity, r.density);
    number(state, "mass", r.hasMass, r.mass);
    number(state, "inertialtensorstiffness", r.hasInertia, r.inertiaStiffness);

    const SIM_Data *phys = obj.getConstNamedSubData("PhysicalParms");
    number(phys, "friction", r.hasFriction, r.friction);
    number(phys, "bounce", r.hasBounce, r.bounce);

    const SIM_Data *bullet = geo.getConstNamedSubData("BulletData");
    if (bullet)
    {
        UT_StringHolder georep;
        if (box3dDataOption(*bullet, "bullet_georep", georep) && georep.isstring())
        {
            r.hasGeorep = true;
            r.georep    = std::string(georep.c_str());
        }
        toggle(bullet, "bullet_autofit", r.hasAutofit, r.autofit);
        toggle(bullet, "bullet_groupconnected", r.hasGroupConnected, r.groupConnected);
        vec(bullet, "bullet_primS", r.primS);
        vec(bullet, "bullet_primT", r.primT);
        float deg[3] = { 0, 0, 0 };
        vec(bullet, "bullet_primR", deg);
        box3dEulerToQuat(deg, r.primRot);       // Euler XYZ degrees, as for points
        bool unused = false;
        number(bullet, "bullet_radius", unused, r.radius);
        number(bullet, "bullet_length", unused, r.length);
    }
    return r;
}

// A whole object's shapes: the hand-set box / sphere / capsule (Bullet Data with AutoFit off), else
// built from the geometry.
BuiltShapes
wholeObjectShapes(b3dop::Box3DWorld &world, const GU_Detail *gdp, const double m[16], const double pivot[3],
                  const b3dop::WholeObjectSetup &setup)
{
    if (setup.explicitShape)
    {
        BuiltShapes out;
        Piece pc;
        pc.spec   = setup.spec;
        pc.volume = b3dop::shapeVolume(pc.spec);
        out.totalVolume = pc.volume;
        out.pieces.push_back(pc);
        return out;
    }
    return buildWholeObjectShapes(world, gdp, m, pivot, setup.node.defaultShape, setup.node.hullPerConnectedSet);
}

// What a whole object's shapes are built from: the geometry, its own transform, the pivot and the
// shape settings (Box3D Data's, or the RBD Object's Bullet Data). Equal: nothing to rebuild.
uint64_t
wholeObjectShapeSig(const GU_Detail *gdp, const double m[16], const double pivot[3], const b3dop::WholeObjectSetup &setup)
{
    const b3dop::BodySettings &node = setup.node;
    using b3dop::mixHash;
    uint64_t h = mixHash(b3dop::kHashSeed, uint64_t(gdp->getUniqueId()));
    const GA_Attribute *P = gdp->getP();
    h = mixHash(h, P ? uint64_t(P->getDataId()) : ~0ull);
    h = mixHash(h, uint64_t(gdp->getPrimitiveList().getDataId()));
    h = mixHash(h, uint64_t(gdp->getNumPoints()));
    h = mixHash(h, uint64_t(gdp->getNumPrimitives()));
    for (int i = 0; i < 16; ++i)
        h = b3dop::mixDouble(h, m[i]);
    for (int i = 0; i < 3; ++i)
        h = b3dop::mixDouble(h, pivot[i]);
    h = mixHash(h, uint64_t(node.defaultShape));
    h = mixHash(h, node.hullPerConnectedSet ? 1u : 2u);
    if (setup.explicitShape)
    {
        const b3dop::ShapeSpec &sp = setup.spec;
        const float f[] = { float(int(sp.type)), sp.halfExtents[0], sp.halfExtents[1], sp.halfExtents[2], sp.radius,
                            sp.halfLength, sp.center[0], sp.center[1], sp.center[2],
                            sp.rot[0], sp.rot[1], sp.rot[2], sp.rot[3] };
        h = mixHash(mixHash(h, 3u), b3dop::hashFloats(f, sizeof(f) / sizeof(f[0])));
    }
    return h ? h : 1;
}

// Sync an object that is one body (isWholeObject): its geometry gives the shapes, its Position
// data (RBD State for an RBD Object) the pose, pivot and velocities, Box3D Data the settings
// (there are no point attributes). Without Box3D Data the RBD Object's own mass, friction, bounce
// and Bullet Data shape are used (readRbdObjectSettings). Without Position data the object stays
// where its geometry is, about the centre of its bounding box, until the write-back creates the data.
void
syncWholeObject(Box3DSharedState &st, const SIM_Object &obj, const SIM_Geometry &geo, const GU_Detail *gdp,
                SyncCounts &c)
{
    b3dop::Box3DWorld &world = st.world;
    const int objectId = obj.getObjectId();
    const SIM_Box3DData *dataNode = SIM_DATA_GETCONST(obj, kBox3DDataName, SIM_Box3DData);
    // Box3D Data wins whenever it is attached; otherwise the RBD Object's own settings.
    b3dop::WholeObjectSetup setup;
    if (dataNode)
        setup.node = dataNode->toBodySettings();
    else
        setup = b3dop::rbdObjectSetup(readRbdObjectSettings(obj, geo));
    const b3dop::BodySettings &node = setup.node;

    Box3DObjectSync &os = st.objectSync[objectId];
    os.visited     = st.syncStamp;
    os.liveStamp   = st.syncStamp;
    os.addImpact   = dataNode && dataNode->getAddImpact();
    os.wholeObject = true;
    os.canSkip     = false;
    os.propsKnown  = false;
    os.entryCount  = 0;
    os.bodies.clear();
    os.byName.clear();              // anchors name the object itself (Anchors: one body)
    os.namesKnown  = st.wantNames;

    // Geometry -> object space.
    UT_DMatrix4 own;
    geo.getTransform(own);
    double m[16];
    box3dToArray(own, m);
    for (double d : m)
        if (!std::isfinite(d))
        {
            ++c.failedObjects;
            return;
        }

    const uint64_t key = b3dop::makeBodyKey(objectId, kWholeObjectId);
    auto found = st.mirror.find(key);
    Box3DBodyMirror *entry = found != st.mirror.end() ? &found->second : nullptr;

    // Pose and pivot: Position data, else (no write-back yet) the object where its geometry is.
    double        pivot[3] = { 0, 0, 0 };
    Box3DBodyPose cur;
    const bool    hasPosition = readWholePose(obj, pivot, cur);
    if (!hasPosition)
    {
        if (entry)
            std::copy(entry->pivot, entry->pivot + 3, pivot);
        else
        {
            double lo[3] = { 0, 0, 0 }, hi[3] = { 0, 0, 0 };
            bool any = false;
            for (GA_Iterator it(gdp->getPointRange()); !it.atEnd(); ++it)
            {
                const UT_Vector3 p = gdp->getPos3(*it);
                const double pd[3] = { p.x(), p.y(), p.z() };
                double o[3];
                b3dop::xformPoint(m, pd, o);
                for (int i = 0; i < 3; ++i)
                {
                    lo[i] = any ? std::min(lo[i], o[i]) : o[i];
                    hi[i] = any ? std::max(hi[i], o[i]) : o[i];
                }
                any = true;
            }
            for (int i = 0; i < 3; ++i)
                pivot[i] = 0.5 * (lo[i] + hi[i]);
        }
        cur = Box3DBodyPose();
        std::copy(pivot, pivot + 3, cur.pos);
    }

    PointAttrs none(gdp);           // build attributes left unbound: Box3D Data values only
    Box3DBodyProps p = readDynamicProps(node, none, GA_Offset(0));
    p.inertiaScale = setup.inertiaScale;
    const uint64_t sig = wholeObjectShapeSig(gdp, m, pivot, setup);

    if (!entry)
    {
        BuiltShapes shapes = wholeObjectShapes(world, gdp, m, pivot, setup);
        auto res = st.mirror.try_emplace(key);
        entry = &res.first->second;
        entry->props       = p;
        entry->volume      = shapes.totalVolume;
        entry->shapeSig    = sig;
        entry->shapeless   = shapes.pieces.empty();
        entry->wholeObject = true;
        std::copy(pivot, pivot + 3, entry->pivot);

        const b3dop::BodySettings::Type type = entry->shapeless ? b3dop::BodySettings::Type::Static : node.type;
        b3dop::BodySettings b = settingsFor(node, p, type);
        b.density    = boxDensity(p, shapes.totalVolume);
        b.startAwake = node.startAwake;
        entry->awake = b.startAwake;
        const b3dop::BodyHandle h = createBodyWithShapes(world, b, cur, shapes, entry->shapeless, p, true, *entry);
        if (!h)
        {
            st.mirror.erase(res.first);
            ++c.failedObjects;
            return;
        }
        if (shapes.note)
            ++c.objectFallbacks[shapes.note];
        entry->setPose(cur);
        entry->handle = h;
        entry->type   = type;
        ++c.added;
    }
    else
    {
        // Geometry, transform, pivot or shape settings changed: new shapes on the same body.
        bool rebuilt = false;
        if (entry->shapeSig != sig)
        {
            BuiltShapes shapes = wholeObjectShapes(world, gdp, m, pivot, setup);
            world.destroyShapesByHandle(entry->handle);
            b3dop::BodySettings b = settingsFor(node, p, entry->type);
            b.density = boxDensity(p, shapes.totalVolume);
            entry->shapeless = addShapes(world, entry->handle, b, shapes) == 0;
            applyBodyMass(world, entry->handle, p, shapes.totalVolume, false);    // even with none: clears Box3D's dirty mass
            entry->volume    = shapes.totalVolume;
            entry->shapeSig  = sig;
            std::copy(pivot, pivot + 3, entry->pivot);
            world.wakeByHandle(entry->handle);
            if (shapes.note)
                ++c.objectFallbacks[shapes.note];
            rebuilt = true;
            ++c.rebuilt;
        }
        if (applyPropChanges(world, *entry, p, rebuilt))
            ++c.updated;

        const b3dop::BodySettings::Type wantType = entry->shapeless ? b3dop::BodySettings::Type::Static : node.type;
        if (entry->type != wantType)
        {
            world.setTypeByHandle(entry->handle, wantType);
            entry->type = wantType;
            if (entry->dynamic())
                applyBodyMass(world, entry->handle, entry->props, entry->volume);
            ++c.retyped;
        }

        // Position data edited by another node (keyframes, a Position DOP set to Set Always).
        if (poseDiffers(cur, *entry))
        {
            world.setTransformByHandle(entry->handle, cur.pos, cur.rot);
            world.setVelocityByHandle(entry->handle, cur.lin, cur.ang);
            entry->setPose(cur);
            ++c.edited;
        }
    }

    entry->pointOff   = -1;
    entry->pointId    = kWholeObjectId;
    entry->pointIndex = -1;
    entry->objectId   = objectId;
    entry->stamp      = st.syncStamp;
    entry->sync       = &os;
    entry->packed     = false;
    ++st.stampedCount;
    os.entryCount = 1;
    os.bodies.push_back(entry);
}

// After the step: a whole object's pose and velocities into its Position data, which its geometry
// follows (positionpath ../Position). Missing data is created as a SIM_Motion; Position data that
// is not a SIM_PositionSimple is left alone (false). The mirror records what the next sync reads.
bool
writeWholeObject(SIM_Object &obj, Box3DBodyMirror &e, const double pos[3], const float rot[4], const float lin[3],
                 const float ang[3])
{
    SIM_PositionSimple *ps = SIM_DATA_GET(obj, SIM_POSITION_DATANAME, SIM_PositionSimple);
    if (!ps)
    {
        if (obj.getConstNamedSubData(SIM_POSITION_DATANAME))
            return false;
        ps = SIM_DATA_CREATE(obj, SIM_POSITION_DATANAME, SIM_Motion, 0);
        if (!ps)
            return false;
    }
    ps->setPivot(UT_Vector3D(e.pivot[0], e.pivot[1], e.pivot[2]));
    ps->setOrientation(UT_QuaternionF(rot[0], rot[1], rot[2], rot[3]));
    // SIM_PositionSimple rotates about the pivot and then translates, so the translation is the
    // pivot's displacement. Checked against Houdini's own transform: if the pivot does not land
    // on the body, `position` must be the pivot's world position instead.
    ps->setPosition(UT_Vector3D(pos[0] - e.pivot[0], pos[1] - e.pivot[1], pos[2] - e.pivot[2]));
    {
        UT_DMatrix4 xf;
        ps->getTransform(xf);
        double m[16], wp[3];
        box3dToArray(xf, m);
        b3dop::xformPoint(m, e.pivot, wp);
        bool lands = true;
        for (int i = 0; i < 3; ++i)
            lands = lands && std::fabs(wp[i] - pos[i]) <= 1e-4 * std::max(1.0, std::fabs(pos[i]));
        if (!lands)
            ps->setPosition(UT_Vector3D(pos[0], pos[1], pos[2]));
    }
    SIM_Motion *mo = SIM_DATA_CAST(ps, SIM_Motion);
    if (mo)
    {
        mo->setVelocity(UT_Vector3(lin[0], lin[1], lin[2]));
        mo->setAngularVelocity(UT_Vector3(ang[0], ang[1], ang[2]));    // radians per second
    }

    // What the next sync reads back (float storage), so our write is not seen as an edit.
    double pv[3];
    Box3DBodyPose back;
    if (readWholePose(obj, pv, back))
        e.setPose(back);
    return true;
}

} // namespace box3dsolver
