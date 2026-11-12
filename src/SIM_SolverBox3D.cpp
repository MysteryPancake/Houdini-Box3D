// SIM_SolverBox3D.cpp
#include "SIM_SolverBox3D.h"
#include "SIM_Box3DWorld.h"
#include "SIM_Box3DData.h"
#include "Box3DDebug.h"

#include <initializer_list>
#include <vector>

#include <GA/GA_AIFSharedStringTuple.h>
#include <GA/GA_Handle.h>
#include <GA/GA_Iterator.h>
#include <GA/GA_Primitive.h>
#include <GU/GU_Detail.h>
#include <GU/GU_DetailHandle.h>
#include <GU/GU_PackedFragment.h>
#include <GU/GU_PackedImpl.h>
#include <GU/GU_PrimPacked.h>
#include <PRM/PRM_Conditional.h>   // PRM_Template.h only forward-declares PRM_Conditional
#include <PRM/PRM_Include.h>
#include <SIM/SIM_CacheOptions.h>
#include <SIM/SIM_Data.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Engine.h>
#include <SIM/SIM_Error.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_Impacts.h>
#include <SIM/SIM_Names.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_Time.h>
#include <SIM/SIM_Utils.h>
#include <UT/UT_Error.h>
#include <UT/UT_Matrix3.h>
#include <UT/UT_ParallelUtil.h>
#include <UT/UT_Matrix4.h>
#include <UT/UT_Quaternion.h>
#include <UT/UT_XformOrder.h>
#include <UT/UT_Vector3.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <iterator>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

// Name the per-object default settings (SIM_Box3DData) and the world are
// attached under.
const char *const kWorldDataName = "Box3DWorld";
const char *const kDataName      = "Box3DData";

// A snapshot of one point's rigid-body state.
using DopBodyState = Box3DBodyMirror;

// SIM_GeometryAutoReadLock/WriteLock::getGdp() differ in whether they return a
// pointer or a reference. These make both spell the same way.
template <class T> T *gdpPtr(T *p) { return p; }
template <class T> T *gdpPtr(T &r) { return &r; }

// Point attributes the solver understands. Names follow Houdini's RBD
// conventions where one exists. Every one is optional; an invalid handle means
// "use the default".
struct PointAttrs
{
    GA_ROHandleI  id;          // int    point id; else the point number
    GA_ROHandleI  active;      // int    0 = static, else dynamic (default 1)
    GA_ROHandleF  mass;        // float  -> density = mass / shape volume
    GA_ROHandleF  friction;    // float
    GA_ROHandleF  bounce;      // float  -> Box3D restitution
    GA_ROHandleF  lineardamping;   // float
    GA_ROHandleF  angulardamping;  // float
    GA_ROHandleF  gravityscale;    // float
    GA_ROHandleI  enablesleep;     // int    0 = never sleeps, else may sleep
    GA_ROHandleI  fastObject;      // int    fastobject: continuous collision against moving bodies too
    GA_ROHandleF  sleepThreshold;  // float  sleepthreshold: speed below which the body may sleep
    GA_ROHandleF  pscale;      // float  uniform size (default 1)
    GA_ROHandleV3 scale;       // vector non-uniform size (default 1 1 1)
    GA_ROHandleV3 v;           // vector linear velocity
    GA_ROHandleV3 w;           // vector angular velocity, radians / second
    GA_ROHandleQ  orient;      // quat   rotation
    GA_ROHandleS  georep;      // string bullet_georep: convexhull, concave, compound, box, sphere,
                               //        capsule, none (see b3dop::parseShapeType; other values,
                               //        e.g. cylinder / plane, use the node's Default Shape)
    // Explicit box / sphere / capsule shape (Houdini's Bullet Data with AutoFit off).
    // See b3dop::PrimOverrides for the conventions.
    GA_ROHandleI  autofit;     // int    bullet_autofit: 1 = fit to the packed geometry's bounds
    GA_ROHandleV3 primS;       // vector bullet_primS: box half extents
    GA_ROHandleV3 primR;       // vector bullet_primR: shape rotation, Euler degrees XYZ
    GA_ROHandleV3 primT;       // vector bullet_primT: shape offset from the body origin
    GA_ROHandleF  radius;      // float  bullet_radius: sphere / capsule radius
    GA_ROHandleF  length;      // float  bullet_length: capsule straight section along Y
    // Mid-simulation controls (Houdini RBD Packed Object names).
    GA_ROHandleI  wantDeactivate;  // int bullet_want_deactivate: 1 = may sleep, 0 = kept awake (read every scan)
    GA_ROHandleI  ignore;          // int bullet_ignore: nonzero = taken out of the simulation (read every scan)
    GA_ROHandleI  deforming;       // int deforming: nonzero = shape rebuilt from the geometry every step
    GA_ROHandleI  autofitValid;    // int bullet_autofit_valid: 0 = rebuild the shape once; the solver writes 1 back
    GA_ROHandleS  collisionGroup;  // string collisiongroup: a name collisionignore / activationignore patterns can match
    GA_ROHandleS  collisionIgnore; // string collisionignore: patterns of bodies this one does not collide with
    GA_ROHandleI  animated;        // int animated: an inactive point follows its point's P / rotation (kinematic)
    GA_ROHandleF  density;         // float density: used when computemass is on
    GA_ROHandleI  computeMass;     // int computemass: mass = density * volume
    GA_ROHandleF  inertiaStiffness;    // float inertialtensorstiffness: scales the inertia tensor
    GA_ROHandleF  minActivation;   // float min_activation_impulse
    GA_ROHandleS  activationIgnore;    // string activationignore
    GA_ROHandleI  dead;            // int dead: delete the point (and its body)
    GA_ROHandleI  sleeping;        // int bullet_sleeping: written by the solver; 0 written by the user wakes the body

    // The constructor binds only what the skip check and the write-back need: id, active,
    // v, w and orient. Everything else is bound by bindBuildAttrs() when an object is
    // actually scanned (a skipped object never looks at them).
    explicit PointAttrs(const GU_Detail *gdp)
        : id(gdp, GA_ATTRIB_POINT, "id"),
          active(gdp, GA_ATTRIB_POINT, "active"),
          v(gdp, GA_ATTRIB_POINT, "v"),
          w(gdp, GA_ATTRIB_POINT, "w"),
          orient(gdp, GA_ATTRIB_POINT, "orient")
    {
    }

    void bindBuildAttrs(const GU_Detail *gdp)
    {
        mass           = GA_ROHandleF(gdp, GA_ATTRIB_POINT, "mass");
        friction       = GA_ROHandleF(gdp, GA_ATTRIB_POINT, "friction");
        bounce         = GA_ROHandleF(gdp, GA_ATTRIB_POINT, "bounce");
        lineardamping  = GA_ROHandleF(gdp, GA_ATTRIB_POINT, "lineardamping");
        angulardamping = GA_ROHandleF(gdp, GA_ATTRIB_POINT, "angulardamping");
        gravityscale   = GA_ROHandleF(gdp, GA_ATTRIB_POINT, "gravityscale");
        enablesleep    = GA_ROHandleI(gdp, GA_ATTRIB_POINT, "enablesleep");
        fastObject     = GA_ROHandleI(gdp, GA_ATTRIB_POINT, "fastobject");
        sleepThreshold = GA_ROHandleF(gdp, GA_ATTRIB_POINT, "sleepthreshold");
        pscale         = GA_ROHandleF(gdp, GA_ATTRIB_POINT, "pscale");
        scale          = GA_ROHandleV3(gdp, GA_ATTRIB_POINT, "scale");
        georep         = GA_ROHandleS(gdp, GA_ATTRIB_POINT, "bullet_georep");
        autofit        = GA_ROHandleI(gdp, GA_ATTRIB_POINT, "bullet_autofit");
        primS          = GA_ROHandleV3(gdp, GA_ATTRIB_POINT, "bullet_primS");
        primR          = GA_ROHandleV3(gdp, GA_ATTRIB_POINT, "bullet_primR");
        primT          = GA_ROHandleV3(gdp, GA_ATTRIB_POINT, "bullet_primT");
        radius         = GA_ROHandleF(gdp, GA_ATTRIB_POINT, "bullet_radius");
        length         = GA_ROHandleF(gdp, GA_ATTRIB_POINT, "bullet_length");
        wantDeactivate = GA_ROHandleI(gdp, GA_ATTRIB_POINT, "bullet_want_deactivate");
        ignore         = GA_ROHandleI(gdp, GA_ATTRIB_POINT, "bullet_ignore");
        deforming      = GA_ROHandleI(gdp, GA_ATTRIB_POINT, "deforming");
        autofitValid   = GA_ROHandleI(gdp, GA_ATTRIB_POINT, "bullet_autofit_valid");
        collisionGroup = GA_ROHandleS(gdp, GA_ATTRIB_POINT, "collisiongroup");
        collisionIgnore = GA_ROHandleS(gdp, GA_ATTRIB_POINT, "collisionignore");
        animated       = GA_ROHandleI(gdp, GA_ATTRIB_POINT, "animated");
        density        = GA_ROHandleF(gdp, GA_ATTRIB_POINT, "density");
        computeMass    = GA_ROHandleI(gdp, GA_ATTRIB_POINT, "computemass");
        inertiaStiffness = GA_ROHandleF(gdp, GA_ATTRIB_POINT, "inertialtensorstiffness");
        minActivation  = GA_ROHandleF(gdp, GA_ATTRIB_POINT, "min_activation_impulse");
        activationIgnore = GA_ROHandleS(gdp, GA_ATTRIB_POINT, "activationignore");
        dead           = GA_ROHandleI(gdp, GA_ATTRIB_POINT, "dead");
        sleeping       = GA_ROHandleI(gdp, GA_ATTRIB_POINT, "bullet_sleeping");
    }
};

// Point attributes whose data ids mark "a body setting or shape may have changed" (the
// fingerprint's propIds, in this order). bullet_georep / bullet_autofit / bullet_prim* /
// bullet_radius / bullet_length are not here on purpose: like Bullet, editing them alone
// changes nothing until bullet_autofit_valid = 0 or deforming = 1, which are.
const char *const kPropAttribNames[Box3DSyncFingerprint::kNumPropIds] = {
    "mass", "friction", "bounce", "lineardamping", "angulardamping", "gravityscale",
    "bullet_want_deactivate", "bullet_ignore", "deforming", "bullet_autofit_valid",
    "pscale", "scale",      // a plain point's convex / concave fallback box follows these
    "collisiongroup", "collisionignore", "animated", "density", "computemass",
    "inertialtensorstiffness", "sleepthreshold", "fastobject",
    "min_activation_impulse", "activationignore", "dead", "bullet_sleeping",
};

// ---------------------------------------------------------------------------
// Packed primitives
//
// A packed prim is ONE point + ONE primitive. Its geometry lives inside the
// primitive, its position is the point's P, and its rotation / scale live in
// the primitive's intrinsic transform, NOT in an `orient` attribute. So for a
// packed point the solver
//   * reads the rotation from the primitive's full transform,
//   * writes P to the point and the rotation to the primitive's local transform,
//   * builds hulls from the packed geometry, relative to the pivot.
//
// Everything that touches GU_PrimPacked is in this section; the HDK
// signatures it relies on are listed below.
// ---------------------------------------------------------------------------

struct PackedInfo
{
    const GU_PrimPacked *prim       = nullptr;       // null: a plain point
    GA_Offset            primOff    = GA_INVALID_OFFSET;
    double               P[3]       = { 0, 0, 0 };     // the point's position = the pivot in world space
    double               pivot[3]   = { 0, 0, 0 };     // GEO_PrimPacked::getPivot, the packed geometry's own space. NOT filled by getPackedInfo: call readPackedPivot first
    double               m4[16]     = { 1, 0, 0, 0,  0, 1, 0, 0,  0, 0, 1, 0,  0, 0, 0, 1 };
    float                rot[4]     = { 0, 0, 0, 1 };  // rotation split out of m4's 3x3
    double               scale[3]   = { 1, 1, 1 };
};

// HDK signatures used for packed prims (checked against the HDK class
// reference, sidefx.com/docs/hdk):
//   static bool GU_PrimPacked::isPackedPrimitive(const GA_Primitive &prim);
//   void GU_PrimPacked::getFullTransform4(UT_Matrix4D &xform) const;
//   void GEO_PrimPacked::setLocalTransform(const UT_Matrix3D &matrix);   // public, inherited
//   void GEO_PrimPacked::getPivot(UT_Vector3 &pos) const;
//   void GU_PrimPacked::transformDirty();                                // public

// Reads the primitive's full transform (local * pivot/instance) into row-major m[16].
void
fetchFull4(const GU_PrimPacked &p, double out[16])
{
    UT_Matrix4D m;
    p.getFullTransform4(m);
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out[r * 4 + c] = m(r, c);
}

// Writes the packed primitive's rotation: its local transform is a 3x3.
void
setLocal3(GU_PrimPacked &p, const double m[9])
{
    UT_Matrix3D mat;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            mat(r, c) = m[r * 3 + c];
    p.setLocalTransform(mat);
    // The primitive caches state derived from its transform (bounds, for the
    // viewport and culling); tell it the transform changed.
    p.transformDirty();
}

// Below this many points a parallel pass costs more than it saves. Starting points,
// not measured: the sync read is cheap for plain points (a few attribute reads) and
// costly for packed ones (a primitive transform each); the write-back does more work
// per point than either.
constexpr size_t kMinParallelPoints      = 2048;    // default
constexpr size_t kMinParallelReadPlain   = 8192;    // sync read, no primitives in the geometry
constexpr size_t kMinParallelReadPacked  = 2048;    // sync read, geometry has primitives
constexpr size_t kMinParallelWrite       = 1024;    // write-back

// Runs f(i) for i in [0, n), threaded on Houdini's TBB pool when n is large.
// f must only read shared data, or write to element i of its own output; the
// serial passes around each call are where the Box3D world and the mirror are touched.
template <class F>
void
parallelForEach(size_t n, const F &f, size_t minParallel = kMinParallelPoints)
{
    if (n < minParallel)
    {
        for (size_t i = 0; i < n; ++i)
            f(i);
        return;
    }
    UTparallelFor(UT_BlockedRange<size_t>(0, n), [&f](const UT_BlockedRange<size_t> &r) {
        for (size_t i = r.begin(); i != r.end(); ++i)
            f(i);
    });
}

// Is this point's primitive a packed primitive? Fills everything the solver
// needs from it EXCEPT the pivot (see readPackedPivot): getPackedInfo runs for
// every point every step, and the pivot is only needed to build a hull. A plain point (no vertex, or a non-packed primitive) returns
// a PackedInfo with prim == nullptr.
PackedInfo
getPackedInfo(const GU_Detail *gdp, GA_Offset ptoff)
{
    PackedInfo pk;
    const GA_Offset vtx = gdp->pointVertex(ptoff);
    if (!GAisValid(vtx))
        return pk;
    const GA_Offset po = gdp->vertexPrimitive(vtx);
    if (!GAisValid(po))
        return pk;
    const GA_Primitive *prim = gdp->getPrimitive(po);
    if (!prim || !GU_PrimPacked::isPackedPrimitive(*prim))
        return pk;

    pk.prim    = static_cast<const GU_PrimPacked *>(prim);
    pk.primOff = po;
    const UT_Vector3 pos = gdp->getPos3(ptoff);
    pk.P[0] = pos.x();  pk.P[1] = pos.y();  pk.P[2] = pos.z();
    fetchFull4(*pk.prim, pk.m4);
    const double m3[9] = { pk.m4[0], pk.m4[1], pk.m4[2],
                           pk.m4[4], pk.m4[5], pk.m4[6],
                           pk.m4[8], pk.m4[9], pk.m4[10] };
    b3dop::matrix3ToQuatScale(m3, pk.rot, pk.scale);
    return pk;
}

// Fills pk.pivot. Only needed when a body's hull is built, which happens once
// per body, so it is a separate call instead of part of getPackedInfo.
void
readPackedPivot(PackedInfo &pk)
{
    if (!pk.prim)
        return;
    UT_Vector3 pv(0, 0, 0);
    pk.prim->getPivot(pv);
    pk.pivot[0] = pv.x();  pk.pivot[1] = pv.y();  pk.pivot[2] = pv.z();
}

// ---------------------------------------------------------------------------
// Collision source geometry
// ---------------------------------------------------------------------------

// The points of one convex piece, as raw xyz in the space they are stored in.
struct NamedPiece
{
    std::string                name;
    std::vector<float>         xyz;
    uint64_t                   hash = 0;   // of xyz; set once per shared source (see innerPieces)
    std::unordered_set<GA_Offset> seen;    // points already added (a point is shared by many prims)
};

struct PieceGroup
{
    std::vector<NamedPiece>                    pieces;
    std::unordered_map<std::string, size_t>    byName;
    uint64_t                                   hash = 0;   // of all pieces in order: the key of the merged (convex) hull
};

// The `seen` sets are only needed while collecting. Hash each piece, and all of
// them together, once: Houdini shares one packed source between its instances,
// so this runs once per source (or fragment), not once per instance.
void
hashPieces(PieceGroup &out)
{
    out.hash = 0;
    for (NamedPiece &np : out.pieces)
    {
        np.seen.clear();
        np.hash = b3dop::hashFloats(np.xyz.data(), np.xyz.size());
        out.hash = b3dop::mixHash(out.hash, np.hash);
    }
}

// The fragment implementation of a packed primitive, or null if it is not one.
const GU_PackedFragment *
asFragment(const GU_PrimPacked &prim)
{
    return dynamic_cast<const GU_PackedFragment *>(prim.sharedImplementation());
}

// Group the points of a detail's primitives into pieces by the primitive
// string attribute `name` (prims without one all land in one piece). A detail
// with points but no primitives becomes one piece of all its points.
void
collectPieces(const GU_Detail *gdp, PieceGroup &out)
{
    const GA_ROHandleS nameH(gdp, GA_ATTRIB_PRIMITIVE, "name");

    auto addPoint = [&](PieceGroup &g, const std::string &name, GA_Offset pt) {
        auto it = g.byName.find(name);
        if (it == g.byName.end())
        {
            it = g.byName.emplace(name, g.pieces.size()).first;
            g.pieces.emplace_back();
            g.pieces.back().name = name;
        }
        NamedPiece &np = g.pieces[it->second];
        if (!np.seen.insert(pt).second)
            return;
        const UT_Vector3 p = gdp->getPos3(pt);
        np.xyz.push_back(p.x());  np.xyz.push_back(p.y());  np.xyz.push_back(p.z());
    };

    bool anyPrim = false;
    for (GA_Iterator it(gdp->getPrimitiveRange()); !it.atEnd(); ++it)
    {
        const GA_Offset po = *it;
        const GA_Primitive *prim = gdp->getPrimitive(po);
        if (!prim)
            continue;
        anyPrim = true;

        std::string name;
        if (nameH.isValid())
            if (const char *n = nameH.get(po))
                name = n;

        const GA_Size nv = prim->getVertexCount();
        for (GA_Size i = 0; i < nv; ++i)
            addPoint(out, name, prim->getPointOffset(i));
    }

    if (!anyPrim)
        for (GA_Iterator it(gdp->getPointRange()); !it.atEnd(); ++it)
            addPoint(out, std::string(), *it);

    hashPieces(out);
}

// Group the points of the given primitives into pieces by connectivity: one
// piece per set of primitives that share points (see b3dop::connectedSets).
void
collectConnectedPieces(const GU_Detail *gdp, const GA_Range &prims, PieceGroup &out)
{
    std::vector<const GA_Primitive *>   list;
    std::vector<std::vector<GA_Offset>> primPoints;
    for (GA_Iterator it(prims); !it.atEnd(); ++it)
    {
        const GA_Primitive *prim = gdp->getPrimitive(*it);
        if (!prim)
            continue;
        list.push_back(prim);
        primPoints.emplace_back();
        const GA_Size nv = prim->getVertexCount();
        for (GA_Size i = 0; i < nv; ++i)
            primPoints.back().push_back(prim->getPointOffset(i));
    }

    const std::vector<int> setOf = b3dop::connectedSets(primPoints);
    for (size_t i = 0; i < list.size(); ++i)
    {
        while (out.pieces.size() <= size_t(setOf[i]))
        {
            out.pieces.emplace_back();
            out.pieces.back().name = std::to_string(out.pieces.size() - 1);
        }
        NamedPiece &np = out.pieces[setOf[i]];
        for (const GA_Offset pt : primPoints[i])
        {
            if (!np.seen.insert(pt).second)
                continue;
            const UT_Vector3 p = gdp->getPos3(pt);
            np.xyz.push_back(p.x());  np.xyz.push_back(p.y());  np.xyz.push_back(p.z());
        }
    }
    hashPieces(out);
}

// A packed FRAGMENT embeds the whole source detail but stands for only the
// primitives in its range (Pack SOP with a `name` attribute: one fragment per
// name, all sharing one source). Its collision geometry is those primitives'
// points: ONE piece.
void
collectFragmentPiece(const GU_Detail *gdp, const GA_Range &range, const std::string &name,
                     PieceGroup &out)
{
    out.pieces.emplace_back();
    NamedPiece &np = out.pieces.back();
    np.name = name;
    out.byName.emplace(name, 0);
    for (GA_Iterator it(range); !it.atEnd(); ++it)
    {
        const GA_Primitive *prim = gdp->getPrimitive(*it);
        if (!prim)
            continue;
        const GA_Size nv = prim->getVertexCount();
        for (GA_Size i = 0; i < nv; ++i)
        {
            const GA_Offset pt = prim->getPointOffset(i);
            if (!np.seen.insert(pt).second)
                continue;
            const UT_Vector3 p = gdp->getPos3(pt);
            np.xyz.push_back(p.x());  np.xyz.push_back(p.y());  np.xyz.push_back(p.z());
        }
    }
    hashPieces(out);
}

// Per-object, per-sync cache of collision source geometry. Lives for one
// syncObject() call. The only source is the geometry packed inside a point's
// primitive; whether that is a simplified proxy or the full-detail geometry is
// up to whoever built the Geometry input, the solver treats both the same.
struct ShapeSources
{
    // Packed geometry, collected once per distinct source detail. The handle
    // is kept in the entry so the detail (and so the pointer used as the key)
    // stays alive for the whole call.
    struct InnerEntry
    {
        GU_ConstDetailHandle handle;
        PieceGroup           pieces;             // grouped by the `name` attribute
        PieceGroup           connected;          // one piece per set of connected primitives
        bool                 connectedBuilt = false;
    };
    std::unordered_map<const GU_Detail *, InnerEntry> inner;
    // Fragments: (shared source detail, fragment name) -> that fragment's piece.
    std::map<std::pair<const GU_Detail *, std::string>, InnerEntry> fragments;

    // Returns the pieces of a packed primitive's own geometry, or null. With
    // `connected` the pieces are the sets of connected primitives rather than the
    // groups by `name`.
    const PieceGroup *innerPieces(const GU_PrimPacked &prim, bool connected)
    {
        GU_ConstDetailHandle handle = prim.getPackedDetail();
        GU_DetailHandleAutoReadLock lock(handle);
        const GU_Detail *gdp = gdpPtr(lock.getGdp());
        if (!gdp)
            return nullptr;

        // A packed fragment: the detail is the WHOLE source, shared by every
        // fragment of it, so only this fragment's own primitives are used.
        const GU_PackedFragment *frag = asFragment(prim);
        InnerEntry              *entry = nullptr;
        if (frag)
        {
            const std::string name = frag->isIntValue() ? std::to_string(frag->intValue())
                                                        : std::string(frag->name().c_str());
            const auto key = std::make_pair(gdp, name);
            auto fit = fragments.find(key);
            if (fit == fragments.end())
            {
                InnerEntry e;
                e.handle = handle;
                collectFragmentPiece(gdp, frag->getPrimitiveRange(), name, e.pieces);
                fit = fragments.emplace(key, std::move(e)).first;
            }
            entry = &fit->second;
        }
        else
        {
            auto it = inner.find(gdp);
            if (it == inner.end())
            {
                InnerEntry e;
                e.handle = handle;
                collectPieces(gdp, e.pieces);
                it = inner.emplace(gdp, std::move(e)).first;
            }
            entry = &it->second;
        }

        if (!connected)
            return entry->pieces.pieces.empty() || entry->pieces.pieces[0].xyz.empty()
                       ? nullptr : &entry->pieces;

        if (!entry->connectedBuilt)
        {
            if (frag)
                collectConnectedPieces(gdp, frag->getPrimitiveRange(), entry->connected);
            else
            {
                collectConnectedPieces(gdp, gdp->getPrimitiveRange(), entry->connected);
                if (entry->connected.pieces.empty())        // points only, no primitives
                    collectPieces(gdp, entry->connected);
            }
            entry->connectedBuilt = true;
        }
        return entry->connected.pieces.empty() ? nullptr : &entry->connected;
    }
};

// ---------------------------------------------------------------------------
// Point state
// ---------------------------------------------------------------------------

// Read P / orient / v / w of one point. Returns false if P is not finite.
// For a packed point the rotation comes from the primitive, not `orient`.
bool
readPointState(const GU_Detail *gdp, const PointAttrs &a, GA_Offset off,
               const PackedInfo &pk, Box3DBodyPose &out)
{
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
    if (a.v.isValid())
    {
        const UT_Vector3 v = a.v.get(off);
        out.lin[0] = v.x();  out.lin[1] = v.y();  out.lin[2] = v.z();
    }
    if (a.w.isValid())
    {
        const UT_Vector3 w = a.w.get(off);
        out.ang[0] = w.x();  out.ang[1] = w.y();  out.ang[2] = w.z();
    }
    return true;
}

// Returns a short description of what differs, or nullptr if the same.
const char *
describeStateDifference(const Box3DBodyPose &a, const Box3DBodyPose &b)
{
    const double eps = 1e-5;
    for (int i = 0; i < 3; ++i)
        if (std::fabs(a.pos[i] - b.pos[i]) > eps) return "position";

    // q and -q are the same rotation.
    double dot = 0.0;
    for (int i = 0; i < 4; ++i) dot += double(a.rot[i]) * double(b.rot[i]);
    if (std::fabs(dot) < 1.0 - 1e-6) return "orientation";

    for (int i = 0; i < 3; ++i)
        if (std::fabs(a.lin[i] - b.lin[i]) > eps) return "linear velocity";
    for (int i = 0; i < 3; ++i)
        if (std::fabs(a.ang[i] - b.ang[i]) > eps) return "angular velocity";
    return nullptr;
}

// What the sync pass depends on, as cheap change markers (see Box3DSyncFingerprint).
// `a` must be bound on `gdp`. An absent (or wrong-typed) attribute is its own marker value.
// The primitive list's change marker. A packed primitive's transform (and so its scale) is
// an intrinsic, not an attribute with a data id; this is the closest marker the detail has.
// GA_PrimitiveList::getDataId() const and GA_Detail::getPrimitiveList() const are in the HDK
// reference. The HDK says bumpDataId() is how primitives / intrinsic data are marked dirty,
// i.e. it is up to whoever edits a packed transform. Houdini's own nodes are expected to do
// that, but it is not guaranteed for every route (e.g. custom HDK code that only calls
// setLocalTransform): such an edit is then picked up on the next scan instead of this step.
GA_DataId
primitiveListDataId(const GU_Detail *gdp)
{
    return gdp->getNumPrimitives() > 0 ? gdp->getPrimitiveList().getDataId() : Box3DSyncFingerprint::kNoId;
}

Box3DSyncFingerprint
makeFingerprint(const GU_Detail *gdp, const PointAttrs &a, int nodeType, uint64_t nodeHash)
{
    // Geometry with primitives (packed instances) is ASSUMED to keep its point count and
    // topology: whoever changes it must create a new instance with a new `id`, which shows
    // up in the `id` / P markers. Plain points are still checked for count changes.
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
    // Looked up by name (a dozen lookups per object per step): `a` binds only the
    // sync-critical handles before the skip check.
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
// Every marker that differs, with old -> new values, for the debug log.
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
// Building the Box3D shapes for one point
// ---------------------------------------------------------------------------

// One collision piece of a body: a hull, or a box / sphere (the fallback when a
// piece cannot be hulled, and the shape for box / sphere points).
struct Piece
{
    bool              isHull = false;
    b3dop::HullId     hull   = 0;
    b3dop::ShapeSpec  spec;
    float             volume = 0.0f;
};

struct BuiltShapes
{
    std::vector<Piece> pieces;
    float              totalVolume = 0.0f;
    const char        *note = nullptr;      // set when something fell back; logged once per body
};

// Merge pieces into one point set.
void
mergePieces(const std::vector<std::vector<float>> &pieces, std::vector<float> &out)
{
    out.clear();
    for (const auto &p : pieces)
        out.insert(out.end(), p.begin(), p.end());
}

// Turn one set of body-local points into a Piece: a hull if possible, else a
// box fitted to the points. Returns false if there are no usable points.
bool
pieceFromPoints(b3dop::Box3DWorld &world, const std::vector<float> &pts, Piece &out, const char *&note)
{
    const int n = int(pts.size() / 3);
    float vol = 0.0f;
    const b3dop::HullId h = world.createHull(pts.data(), n, &vol);
    if (h)
    {
        out.isHull = true;
        out.hull   = h;
        out.volume = vol;
        return true;
    }
    // Fewer than 4 points, flat, or too complex to hull: fall back to the bounds.
    if (b3dop::fitShapeToPoints(b3dop::ShapeType::Box, pts.data(), n, out.spec))
    {
        out.isHull = false;
        out.volume = b3dop::shapeVolume(out.spec);
        note = "a piece could not be hulled (flat, < 4 points or too complex); used its bounding box";
        return true;
    }
    return false;
}

// Decide what shapes a point's body gets. See AGENTS.md, "Pieces and packed geometry".
//
//  box / sphere
//  / capsule    : packed point with autofit -> fitted to the packed geometry's bounds
//                 otherwise (plain point, or autofit off) -> the unit shape scaled by
//                 pscale * scale, then any explicit bullet_primS / radius / length /
//                 primT / primR overrides (b3dop::PrimOverrides)
//  convex       : ONE hull of the packed geometry, or with hullPerConnected one
//                 hull per set of connected primitives (several shapes on one body)
//  concave      : one hull PER PIECE of the packed geometry (pieces grouped by
//                 the prim `name` attribute); several shapes on one body
//
// The packed geometry is the collision geometry, whether it is a simplified
// proxy or the full-detail mesh. A convex / concave point that is not packed
// (or whose packed geometry is empty) falls back to a box so the body still
// exists.
BuiltShapes
buildShapes(b3dop::Box3DWorld &world, ShapeSources &src, b3dop::ShapeType type,
            bool hullPerConnected, float pscale, const float scale[3], const PackedInfo &pk,
            bool autofit, const b3dop::PrimOverrides &ov)
{
    using namespace b3dop;
    BuiltShapes out;

    // Convex / concave are built from point data (hulls); box / sphere / capsule from
    // the packed geometry's bounds (autofit) or from size attributes.
    const bool wantGeometry = (type == ShapeType::Convex || type == ShapeType::Concave);

    // 1. The source: the packed primitive's own geometry (any shape type on a
    //    packed point). A plain point has none.
    //    Convex with hullPerConnected splits it into connected sets of primitives.
    const bool        mergeAll = (type == ShapeType::Convex) && !hullPerConnected;
    //    Box / sphere / capsule only need it when they autofit to it.
    const PieceGroup *group    = (pk.prim && (wantGeometry || autofit))
                                     ? src.innerPieces(*pk.prim, type == ShapeType::Convex && hullPerConnected)
                                     : nullptr;

    // 1b. Convex / concave on a packed point: the hull of each instance of one
    //     source is a function of (source, pivot, scale), so look it up by that
    //     key and build the body-local points only on a miss.
    double S[3];
    if (group && wantGeometry && packedScale(pk.m4, S))
    {
        auto addKeyed = [&](uint64_t sourceHash, uint32_t kind,
                            const std::vector<const NamedPiece *> &from) {
            const uint64_t key = packedHullKey(sourceHash, pk.pivot, S, kind);
            Piece pc;
            float vol = 0.0f;
            if (const HullId h = world.findHull(key, &vol))
            {
                pc.isHull = true;  pc.hull = h;  pc.volume = vol;
            }
            else
            {
                std::vector<std::vector<float>> parts;
                for (const NamedPiece *np : from)
                {
                    std::vector<float> local;
                    packedPointsToBodyLocal(np->xyz.data(), int(np->xyz.size() / 3), pk.m4, pk.pivot, local);
                    parts.push_back(std::move(local));
                }
                std::vector<float> pts;
                mergePieces(parts, pts);
                const int n = int(pts.size() / 3);
                const HullId built = world.createHullWithKey(key, pts.data(), n, &vol);
                if (built)
                {
                    pc.isHull = true;  pc.hull = built;  pc.volume = vol;
                }
                else if (fitShapeToPoints(ShapeType::Box, pts.data(), n, pc.spec))
                {
                    pc.isHull = false;
                    pc.volume = shapeVolume(pc.spec);
                    out.note = "a piece could not be hulled (flat, < 4 points or too complex); used its bounding box";
                }
                else
                    return;
            }
            out.totalVolume += pc.volume;
            out.pieces.push_back(pc);
        };

        if (mergeAll)
        {
            std::vector<const NamedPiece *> all;
            for (const NamedPiece &np : group->pieces)
                if (!np.xyz.empty()) all.push_back(&np);
            if (!all.empty())
                addKeyed(group->hash, 1, all);
        }
        else
            for (const NamedPiece &np : group->pieces)
                if (!np.xyz.empty())
                    addKeyed(np.hash, 2, { &np });

        if (!out.pieces.empty())
            return out;
        // nothing usable: fall through to the unit-shape fallback below
    }

    // 2. Its points, in BODY-LOCAL space, one set per piece.
    std::vector<std::vector<float>> sets;
    if (group && out.pieces.empty())
    {
        for (const NamedPiece &np : group->pieces)
        {
            const int n = int(np.xyz.size() / 3);
            if (n == 0)
                continue;
            // Packed geometry is in the primitive's own space: relative to the
            // pivot, scale baked in, rotation out (and quantized so instances
            // at different rotations share one hull).
            std::vector<float> local;
            packedPointsToBodyLocal(np.xyz.data(), n, pk.m4, pk.pivot, local);
            sets.push_back(std::move(local));
        }
    }

    // 3. Shapes from the point sets.
    if (wantGeometry && !sets.empty())
    {
        std::vector<std::vector<float>> hulls;
        if (mergeAll)
        {
            hulls.emplace_back();
            mergePieces(sets, hulls.back());     // one hull from everything
        }
        else
            hulls = std::move(sets);             // one hull per piece

        for (const auto &pts : hulls)
        {
            Piece pc;
            if (pieceFromPoints(world, pts, pc, out.note))
            {
                out.totalVolume += pc.volume;
                out.pieces.push_back(pc);
            }
        }
    }
    else if (!wantGeometry && !sets.empty())
    {
        // box / sphere on a packed point: fit to the packed geometry's bounds.
        std::vector<float> all;
        mergePieces(sets, all);
        Piece pc;
        if (fitShapeToPoints(type, all.data(), int(all.size() / 3), pc.spec))
        {
            pc.volume = shapeVolume(pc.spec);
            out.totalVolume += pc.volume;
            out.pieces.push_back(pc);
        }
    }

    // 4. Nothing usable: the unit shape scaled by pscale * scale.
    if (out.pieces.empty())
    {
        if (wantGeometry)
            out.note = pk.prim ? "packed geometry is empty; used a box"
                               : "convex / concave needs a packed primitive; used a box";
        Piece pc;
        pc.spec   = makeShapeSpec(wantGeometry ? ShapeType::Box : type, pscale, scale);
        if (!wantGeometry)
            applyPrimOverrides(pc.spec, ov);    // explicit size / offset / rotation, if given
        pc.volume = shapeVolume(pc.spec);
        out.totalVolume = pc.volume;
        out.pieces.push_back(pc);
    }
    return out;
}

// Reads a point's shape type from `bullet_georep` without touching the string. The attribute is
// a shared-string attribute: every point stores an integer handle into one table of distinct
// strings (GA_AIFSharedStringTuple::getHandle / getTableString). So the per-point work is one
// integer read and a lookup in a tiny per-scan map; each distinct handle is turned into text
// and parsed once. Handles belong to one attribute's table and may differ between copies of the
// geometry, so the handle map lives for one scan only; the text -> type memo
// (Box3DSharedState::shapeTypes) lives as long as the world. Without the shared-string
// interface it reads the string and uses the memo directly. UNVERIFIED in Houdini: compiled
// against the HDK class reference only.
struct GeorepLookup
{
    const GA_Attribute            *attr = nullptr;
    const GA_AIFSharedStringTuple *aif  = nullptr;
    struct Parsed { bool ok = false; b3dop::ShapeType type = b3dop::ShapeType::Box; };
    std::unordered_map<int64_t, Parsed> byHandle;

    void bind(const PointAttrs &a)
    {
        attr = a.georep.isValid() ? a.georep.getAttribute() : nullptr;
        aif  = attr ? attr->getAIFSharedStringTuple() : nullptr;
    }

    // False when the point has no usable value (attribute missing, empty or unknown string):
    // `out` is then left as the caller's default.
    bool get(const PointAttrs &a, GA_Offset off, b3dop::ShapeTypeCache &memo, b3dop::ShapeType &out)
    {
        if (!attr)
            return false;
        if (!aif)
            return memo.parse(a.georep.get(off), out);
        const GA_StringIndexType handle = aif->getHandle(attr, off, 0);
        if (handle == GA_INVALID_STRING_INDEX)
            return false;
        auto it = byHandle.find(int64_t(handle));
        if (it == byHandle.end())
        {
            Parsed p;
            p.ok = memo.parse(aif->getTableString(attr, handle), p.type);
            it = byHandle.emplace(int64_t(handle), p).first;
        }
        if (it->second.ok)
            out = it->second.type;
        return it->second.ok;
    }
};

// ---------------------------------------------------------------------------
// Body settings, shape planning, and mid-simulation updates
// ---------------------------------------------------------------------------

// Box3D asserts on negative friction / restitution / damping: attribute values are
// clamped at 0 and non-finite ones ignored.
float
nonNegative(float v, float fallback)
{
    return std::isfinite(v) ? std::max(v, 0.0f) : fallback;
}

// The settings of one point that may change during the simulation. Building a body and
// comparing an existing body against its point both read through here, so an untouched
// point always compares equal to what its body was built with.
//
// sleepOk is the CREATION rule (bullet_want_deactivate, else enablesleep, else the node);
// a scan of an existing body only takes bullet_want_deactivate from it (enablesleep is a
// creation-time setting, as in Bullet).
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
    p.linearDamping  = a.lineardamping.isValid()  ? nonNegative(a.lineardamping.get(off),  node.linearDamping)  : node.linearDamping;
    p.angularDamping = a.angulardamping.isValid() ? nonNegative(a.angulardamping.get(off), node.angularDamping) : node.angularDamping;
    p.gravityScale   = node.gravityScale;
    if (a.gravityscale.isValid())
    {
        const float g = a.gravityscale.get(off);
        if (std::isfinite(g)) p.gravityScale = g;      // may be negative (floats upward)
    }
    p.sleepOk = node.enableSleep;
    if (a.enablesleep.isValid())    p.sleepOk = a.enablesleep.get(off) != 0;
    if (a.wantDeactivate.isValid()) p.sleepOk = a.wantDeactivate.get(off) != 0;
    p.ignored = a.ignore.isValid() && a.ignore.get(off) != 0;

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
    p.fastObject = a.fastObject.isValid() ? a.fastObject.get(off) != 0 : node.isBullet;
    p.minActivationImpulse = 0.0f;
    if (a.minActivation.isValid())
    {
        const float m = a.minActivation.get(off);
        if (m > 0.0f && std::isfinite(m)) p.minActivationImpulse = m;
    }
    p.activationIgnore.clear();
    if (a.activationIgnore.isValid())
        if (const char *s = a.activationIgnore.get(off))
            p.activationIgnore = s;
    return p;
}

// A point's collision filter profile: the object's own profile (`base`, index `baseIdx`) with
// the point's collisiongroup / collisionignore, if it has them.
int
pointFilterProfile(b3dop::Box3DWorld &world, const PointAttrs &a, GA_Offset off,
                   const b3dop::FilterProfile &base, int baseIdx)
{
    if (!a.collisionGroup.isValid() && !a.collisionIgnore.isValid())
        return baseIdx;
    b3dop::FilterProfile fp = base;
    if (a.collisionGroup.isValid())
        if (const char *g = a.collisionGroup.get(off))
            fp.group = g;
    if (a.collisionIgnore.isValid())
        if (const char *ig = a.collisionIgnore.get(off))
            fp.ignore = ig;
    return world.filterProfile(fp);
}

// The density Box3D gets (it computes mass = density * volume; for a compound the volume
// is the sum of the pieces). computemass: the density itself; otherwise mass / volume, and
// the density if the mass or volume is unusable.
float
boxDensity(const Box3DBodyProps &p, float volume)
{
    if (p.computeMass)
        return p.density;
    return (p.mass > 0.0f && std::isfinite(p.mass) && volume > 0.0f) ? p.mass / volume : p.density;
}

// The body's mass as Houdini would report it (written back to `mass` with computemass).
float
effectiveMass(const Box3DBodyProps &p, float volume)
{
    return p.computeMass ? p.density * volume : p.mass;
}

// The type a point's body should have: `active` gives the node's type; an inactive point is
// static, or kinematic when `animated` (it follows its point). Shapeless bodies are static.
b3dop::BodySettings::Type
wantedType(const b3dop::BodySettings &node, bool active, bool animated, bool shapeless)
{
    if (shapeless)
        return b3dop::BodySettings::Type::Static;
    if (active)
        return node.type;
    return animated ? b3dop::BodySettings::Type::Kinematic : b3dop::BodySettings::Type::Static;
}

// The body's mass, after its shapes were added (always with updateMass = false) or replaced,
// or after anything that recomputes it from the shapes (a type change). As in Houdini RBD, the
// point (the packed primitive's anchor / pivot) IS the centre of mass: Box3D's centroid of the
// shapes is moved to the body origin, with the inertia shifted to match, and scaled by
// inertialtensorstiffness. Static and kinematic bodies only get their shape densities.
void
applyBodyMass(b3dop::Box3DWorld &world, b3dop::BodyHandle h, const Box3DBodyProps &p, float volume)
{
    world.applyMassByHandle(h, boxDensity(p, volume), true, p.inertiaScale);
}

// The node's settings with a point's values applied.
b3dop::BodySettings
settingsFor(const b3dop::BodySettings &node, const Box3DBodyProps &p, b3dop::BodySettings::Type type)
{
    b3dop::BodySettings b = node;
    b.type           = type;
    b.friction       = p.friction;
    b.restitution    = p.restitution;
    b.linearDamping  = p.linearDamping;
    b.angularDamping = p.angularDamping;
    b.gravityScale   = p.gravityScale;
    b.enableSleep    = p.sleepOk;
    b.filterProfile  = p.filterProfile;
    b.sleepThreshold = p.sleepThreshold;
    b.isBullet       = p.fastObject;
    return b;
}

// pscale / scale: size plain points. A packed point's size is its primitive's transform,
// so there they only matter as the fallback box.
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

// The scale a convex / concave body's shapes depend on: a packed primitive's per-axis scale
// (the row lengths of its 3x3), else pscale * scale (the fallback box of a plain point).
bool
sizeKeyFromM3(const double m3[9], double out[3])
{
    float q[4];
    return b3dop::matrix3ToQuatScale(m3, q, out);
}

void
sizeKeyFromAttrs(float ps, const float sc[3], double out[3])
{
    for (int i = 0; i < 3; ++i)
        out[i] = double(ps) * double(sc[i]);
}

// Equal within 1e-4 relative: our own write-back recomposes packed transforms, and float
// noise in the row lengths must not count as a scale change.
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

// The size key of an existing body's point, from what pass 1 read.
bool
currentSizeKey(const PointAttrs &a, const Box3DPointSlot &s, double out[3])
{
    if (s.packed)
        return sizeKeyFromM3(s.m3, out);
    float ps, sc[3];
    readPscaleScale(a, s.off, ps, sc);
    sizeKeyFromAttrs(ps, sc, out);
    return true;
}

// What a point's body is made of: its shape type and pieces, plus what the mirror keeps to
// recognise the same shapes again (sig) and to follow scale changes (autoScale, sizeKey).
struct ShapePlan
{
    b3dop::ShapeType type = b3dop::ShapeType::Box;
    BuiltShapes      shapes;            // no pieces for `none`
    uint64_t         sig  = 0;
    bool             autoScale = false;
    double           sizeKey[3] = { 1, 1, 1 };
};

// Identifies a set of built shapes. Hulls are keyed by content (or by source, pivot and
// scale for packed instances) and primitives by their numbers, so equal signatures mean
// Box3D would get identical shapes and a rebuild can be skipped.
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

// Decide a point's shapes from its current attributes (bullet_georep, bullet_autofit,
// bullet_prim*, bullet_radius, bullet_length, pscale, scale, packed geometry). Hulls it
// needs are created in (or found in) the world's cache; no body is touched.
ShapePlan
planShapes(b3dop::Box3DWorld &world, ShapeSources &src, GeorepLookup &georep, b3dop::ShapeTypeCache &shapeTypes,
           const b3dop::BodySettings &node, const PointAttrs &a, GA_Offset off, int64_t bodyId,
           const PackedInfo &pk)
{
    ShapePlan plan;

    // Shape: `bullet_georep` string attribute, else the node's default.
    plan.type = node.defaultShape;
    georep.get(a, off, shapeTypes, plan.type);       // unknown / missing -> keep the default

    // `none`: a static body with no collision shape (like Bullet, where it still
    // exists for constraints). The one case where a body is allowed to be shapeless.
    if (plan.type == b3dop::ShapeType::None)
    {
        plan.sig = shapeSignature(plan.type, plan.shapes);
        return plan;
    }

    float ps, sc[3];
    readPscaleScale(a, off, ps, sc);

    // AutoFit: the packed geometry's bounds decide the box / sphere / capsule. The
    // attribute overrides the node's setting; without a packed primitive there is
    // no geometry to fit, so the explicit attributes (or pscale / scale) apply.
    const bool autofit = a.autofit.isValid() ? (a.autofit.get(off) != 0) : node.autoFit;
    b3dop::PrimOverrides ov;
    if (a.primS.isValid())
    {
        const UT_Vector3 v = a.primS.get(off);
        ov.hasSize = true;  ov.size[0] = v.x();  ov.size[1] = v.y();  ov.size[2] = v.z();
    }
    if (a.primR.isValid())
    {
        const UT_Vector3 v = a.primR.get(off);
        // Euler degrees -> radians; Houdini's own XYZ rotate order builds the quaternion.
        // A non-finite angle leaves the override off (the shape keeps its rotation).
        if (SYSisFinite(v.x()) && SYSisFinite(v.y()) && SYSisFinite(v.z()))
        {
            const UT_Vector3F rad(SYSdegToRad(v.x()), SYSdegToRad(v.y()), SYSdegToRad(v.z()));
            const UT_QuaternionF q(rad, UT_XformOrder(UT_XformOrder::TRS, UT_XformOrder::XYZ));
            ov.hasRot = true;
            ov.rot[0] = q.x();  ov.rot[1] = q.y();  ov.rot[2] = q.z();  ov.rot[3] = q.w();
        }
    }
    if (a.primT.isValid())
    {
        const UT_Vector3 v = a.primT.get(off);
        ov.hasPos = true;  ov.pos[0] = v.x();  ov.pos[1] = v.y();  ov.pos[2] = v.z();
    }
    if (a.radius.isValid())  { ov.hasRadius = true;  ov.radius = a.radius.get(off); }
    if (a.length.isValid())  { ov.hasLength = true;  ov.length = a.length.get(off); }

    plan.shapes = buildShapes(world, src, plan.type, node.hullPerConnectedSet, ps, sc, pk, autofit, ov);
    (void)bodyId;                   // only used by the debug log
    if (plan.shapes.note)
        B3D_LOG("  body id=%lld: %s", (long long)bodyId, plan.shapes.note);
    plan.sig = shapeSignature(plan.type, plan.shapes);

    // Convex / concave follow scale changes by themselves (Houdini: "the solver updates the
    // collision geometry and inertial tensor"); auto-fitted and explicit primitives do not.
    plan.autoScale = (plan.type == b3dop::ShapeType::Convex || plan.type == b3dop::ShapeType::Concave);
    if (plan.autoScale)
    {
        if (pk.prim)
        {
            const double m3[9] = { pk.m4[0], pk.m4[1], pk.m4[2],
                                   pk.m4[4], pk.m4[5], pk.m4[6],
                                   pk.m4[8], pk.m4[9], pk.m4[10] };
            plan.autoScale = sizeKeyFromM3(m3, plan.sizeKey);
        }
        else
            sizeKeyFromAttrs(ps, sc, plan.sizeKey);
    }
    return plan;
}

// Add a plan's pieces to a body, all without Box3D's per-shape mass update: the caller applies
// the mass once afterwards (applyBodyMass, which also puts the centre of mass at the point).
// Returns how many pieces were added.
int
addPlannedShapes(b3dop::Box3DWorld &world, b3dop::BodyHandle h, const b3dop::BodySettings &b, const ShapePlan &plan)
{
    int added = 0;
    for (const Piece &pc : plan.shapes.pieces)
        added += (pc.isHull ? world.createHullShape(h, b, pc.hull, false)
                            : world.createShape(h, b, pc.spec, false)) ? 1 : 0;
    return added;
}

// Keep in the mirror entry what the body was built from (see Box3DBodyMirror).
void
storePlan(DopBodyState &m, const ShapePlan &plan)
{
    m.shapeSig  = plan.sig;
    m.volume    = plan.shapes.totalVolume;
    m.autoScale = plan.autoScale;
    for (int i = 0; i < 3; ++i)
        m.sizeKey[i] = plan.sizeKey[i];
}

// Create the body + shapes for one point and apply its initial velocity. `p` is the point's
// settings (readDynamicProps + filter profile). `entry` is the body's mirror entry: it becomes
// the body's user data and receives what the body was built from (props, shape signature,
// volume, ...), which later scans compare against.
bool
addPointBody(b3dop::Box3DWorld &world, ShapeSources &src, GeorepLookup &georep, b3dop::ShapeTypeCache &shapeTypes,
             const b3dop::BodySettings &node,
             const PointAttrs &a, GA_Offset off, int64_t bodyId,
             const PackedInfo &pk, const Box3DBodyPose &st, bool active, bool animated,
             const Box3DBodyProps &p, DopBodyState &entry, b3dop::BodyHandle &outHandle,
             std::string *fallbackNote = nullptr)
{
    outHandle = 0;
    const ShapePlan plan = planShapes(world, src, georep, shapeTypes, node, a, off, bodyId, pk);
    if (plan.shapes.note && fallbackNote)
        *fallbackNote = plan.shapes.note;       // surfaced to the user as one warning per kind
    entry.props     = p;
    entry.shapeless = (plan.type == b3dop::ShapeType::None);
    entry.animated  = animated;
    storePlan(entry, plan);

    b3dop::BodySettings b = settingsFor(node, p, wantedType(node, active, animated, entry.shapeless));

    if (entry.shapeless)
    {
        outHandle = world.createBody(b, st.pos, st.rot, &entry);
        if (outHandle && p.ignored)
            world.setEnabledByHandle(outHandle, false);
        return outHandle != 0;
    }

    b.density = boxDensity(p, plan.shapes.totalVolume);

    // The settings `b` stay a local: the handle API takes them per shape, so nothing about
    // the body is remembered by the world.
    const b3dop::BodyHandle h = world.createBody(b, st.pos, st.rot, &entry);
    if (!h)
        return false;
    if (addPlannedShapes(world, h, b, plan) == 0)
    {
        world.destroyBody(h);       // never leave a shapeless body behind
        return false;
    }
    applyBodyMass(world, h, p, plan.shapes.totalVolume);

    // Initial velocity, set after the mass is final (Box3D shifts a body's velocity when its
    // centre of mass moves, so this order keeps the values exactly as given). A fresh body
    // already has zero velocity and a static one ignores it, so those skip the call. An
    // animated body is moved by its point, not by `v`.
    const bool hasVel = st.lin[0] != 0.0f || st.lin[1] != 0.0f || st.lin[2] != 0.0f ||
                        st.ang[0] != 0.0f || st.ang[1] != 0.0f || st.ang[2] != 0.0f;
    if (hasVel && b.type != b3dop::BodySettings::Type::Static && !animated)
        world.setVelocityByHandle(h, st.lin, st.ang);
    if (p.ignored)
        world.setEnabledByHandle(h, false);     // bullet_ignore: exists, but takes no part
    outHandle = h;
    return true;
}

// Rebuild an existing body's collision shapes from its point's current attributes and
// geometry (deforming = 1, bullet_autofit_valid = 0, or a convex / concave scale change).
// The body itself (and so its pose, velocity and identity) stays. If the new plan gives the
// same shapes nothing is touched. Returns true if the shapes were replaced; `noShape` is set
// when the new shapes could not be added (the body is then shapeless, so static).
bool
rebuildPointShapes(b3dop::Box3DWorld &world, ShapeSources &src, GeorepLookup &georep,
                   b3dop::ShapeTypeCache &shapeTypes, const b3dop::BodySettings &node,
                   const PointAttrs &a, GA_Offset off, int64_t bodyId, const PackedInfo &pk,
                   const Box3DBodyProps &p, DopBodyState &entry, std::string &note, bool &noShape)
{
    noShape = false;
    const ShapePlan plan = planShapes(world, src, georep, shapeTypes, node, a, off, bodyId, pk);
    if (plan.shapes.note)
        note = plan.shapes.note;
    if (plan.sig == entry.shapeSig)
    {
        storePlan(entry, plan);     // same shapes; refresh the size key it was measured at
        return false;
    }

    const b3dop::BodyHandle h = entry.handle;
    world.destroyShapesByHandle(h);
    storePlan(entry, plan);
    entry.shapeless = true;
    if (plan.type != b3dop::ShapeType::None)
    {
        b3dop::BodySettings b = settingsFor(node, p, entry.type);
        b.density = boxDensity(p, plan.shapes.totalVolume);
        if (addPlannedShapes(world, h, b, plan) > 0)
            entry.shapeless = false;
        else
            noShape = true;
    }
    applyBodyMass(world, h, p, plan.shapes.totalVolume);    // also with no shapes: settles the mass Box3D marked dirty
    world.wakeByHandle(h);          // new shapes need fresh contacts; a resting body must re-settle
    B3D_LOG_VERBOSE("  body id=%lld: collision shape rebuilt", (long long)bodyId);
    return true;
}

// Push the settings that differ from what the body has. With `shapesRebuilt` the new shapes
// already carry the current friction / restitution / density / filter profile, and the mass
// properties and sleep thresholds were applied with them. Returns kPropsChanged and / or
// kMassChanged.
enum : unsigned { kPropsChanged = 1, kMassChanged = 2 };

unsigned
applyPropChanges(b3dop::Box3DWorld &world, DopBodyState &e, const Box3DBodyProps &p, bool shapesRebuilt)
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
        if (p.filterProfile != o.filterProfile)
        {
            world.setFilterProfileByHandle(h, p.filterProfile);
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
    if (p.sleepThreshold != o.sleepThreshold)
    {
        world.setSleepThresholdByHandle(h, p.sleepThreshold);
        changed |= kPropsChanged;
    }
    if (p.fastObject != o.fastObject)
    {
        world.setBulletByHandle(h, p.fastObject);
        changed |= kPropsChanged;
    }
    if (p.sleepOk != o.sleepOk)
    {
        world.setSleepEnabledByHandle(h, p.sleepOk);
        changed |= kPropsChanged;
    }
    if (p.ignored != o.ignored)
    {
        world.setEnabledByHandle(h, !p.ignored);
        changed |= kPropsChanged;
    }
    if (p.minActivationImpulse != o.minActivationImpulse || p.activationIgnore != o.activationIgnore)
        changed |= kPropsChanged;       // read after the step (impacts / activation); nothing to push
    e.props = p;
    return changed;
}

// Queue a point attribute write for after the step (see Box3DPointWrite).
void
queuePointWrite(Box3DSharedState &st, DopBodyState &m, unsigned flags, float mass = 0.0f)
{
    st.pointWrites.emplace_back();
    Box3DPointWrite &w = st.pointWrites.back();
    w.entry    = &m;
    w.objectId = m.objectId;
    w.flags    = flags;
    w.mass     = mass;
}

// Keep what the read pass learned about the point's packed primitive in its mirror
// entry, for the write-back.
void
storePacked(Box3DBodyMirror &m, const Box3DPointSlot &s)
{
    m.packed  = s.packed;
    m.primOff = s.primOff;
    if (s.packed)
        for (int i = 0; i < 9; ++i)
            m.m3[i] = s.m3[i];
}

struct SyncCounts
{
    int added = 0, edited = 0, retyped = 0, failed = 0, duplicates = 0, badP = 0;
    int rebuilt = 0;            // existing bodies whose collision shapes were replaced
    int dead = 0;               // points marked `dead` (deleted after the step)
    int updated = 0;            // existing bodies whose settings (mass, friction, ...) changed
    int skipped = 0;            // objects whose scan was skipped (nothing changed)
    std::unordered_map<std::string, int> fallbacks;     // shape fallback note -> bodies that used it
};

// Sync one object's points into the world. Returns true if any of its points
// is a body we will write back (dynamic).
bool
syncObject(Box3DSharedState &st, const SIM_Object &obj, SyncCounts &c)
{
    const SIM_Geometry *geo = SIM_DATA_GETCONST(obj, SIM_GEOMETRY_DATANAME, SIM_Geometry);
    if (!geo)
    {
        B3D_LOG_VERBOSE("  object id=%d has no Geometry data, skipped", obj.getObjectId());
        return false;
    }
    SIM_GeometryAutoReadLock lock(geo);
    const GU_Detail *gdp = gdpPtr(lock.getGdp());
    if (!gdp)
        return false;

    PointAttrs a(gdp);          // the other attributes are bound once the object is known to need a scan
    const bool anyPrims = gdp->getNumPrimitives() > 0;     // no primitives -> no packed points to look for
    const int  objectId = obj.getObjectId();
    // Default settings for all points of an object (the Box3D Data DOP node), or
    // built-in defaults if the object has none.
    // The full settings (a dozen option reads) are only needed by a scan, so they are built
    // after the skip check; the skip check itself only needs the body type.
    const SIM_Box3DData *dataNode = SIM_DATA_GETCONST(obj, kDataName, SIM_Box3DData);
    int nodeType = int(b3dop::BodySettings().type);
    uint64_t nodeHash = 0;      // the node values that change existing bodies mid-simulation
    if (dataNode)
    {
        const int bt = int(dataNode->getBodyType());       // same mapping as toBodySettings()
        nodeType = int(bt == 0 ? b3dop::BodySettings::Type::Static
                     : bt == 1 ? b3dop::BodySettings::Type::Kinematic
                               : b3dop::BodySettings::Type::Dynamic);
        const float v[10] = { float(dataNode->getMass()), float(dataNode->getFriction()),
                              float(dataNode->getBounce()), float(dataNode->getLinearDamping()),
                              float(dataNode->getAngularDamping()), float(dataNode->getGravityScale()),
                              float(dataNode->getDensity()), dataNode->getComputeMass() ? 1.0f : 0.0f,
                              float(dataNode->getSleepThreshold()), dataNode->getFastObject() ? 1.0f : 0.0f };
        nodeHash = b3dop::hashFloats(v, 10);
        // Add Impact Data decides whether the scan lists the object's bodies for recordImpacts.
        nodeHash = b3dop::mixHash(nodeHash, dataNode->getAddImpact() ? 1 : 2);
    }
    b3dop::Box3DWorld &world = st.world;
    bool hasDynamic = false;

    // ---- nothing changed since we last finished with this object? -------------
    // After a full scan (below) the object's change markers are recorded, and the
    // write-back records them again after bumping the ids of what it wrote. If the next
    // step finds the same markers, no point can differ from the mirror, so passes 1 and 2
    // are skipped: the mirror entries stay current under the stamp they already carry
    // (liveStamp) and are counted without being touched. Packed instances are assumed to
    // keep their point count and topology (see makeFingerprint). A packed primitive's own
    // rotation / scale is only covered by the primitive list's data id (primitiveListDataId,
    // unverified): if an edit does not bump it, it is noticed on the next scan instead.
    // The markers include the attributes that change body settings mid-simulation (mass,
    // friction, ..., bullet_autofit_valid, deforming) and the Box3D Data node's values.
    Box3DObjectSync &os = st.objectSync[objectId];
    os.visited   = st.syncStamp;
    os.addImpact = dataNode && dataNode->getAddImpact();
    const Box3DSyncFingerprint fpNow = makeFingerprint(gdp, a, nodeType, nodeHash);
    if (os.canSkip && fpNow == os.fp)
    {
        ++c.skipped;
        st.stampedCount += os.entryCount;
        return os.hasDynamic;
    }
#ifdef BOX3DDOP_DEBUG
    if (os.canSkip)
        B3D_LOG_VERBOSE("  object id=%d: full scan, changed: %s", objectId,
                describeFingerprintDifference(os.fp, fpNow).c_str());
#endif
    const b3dop::BodySettings node = dataNode ? dataNode->toBodySettings() : b3dop::BodySettings();
    os.canSkip   = false;
    os.liveStamp = st.syncStamp;        // entries stamped below are current under this
    size_t stamped = 0;                 // entries stamped by this scan
    const int problems0 = c.failed + c.duplicates + c.badP;

    ShapeSources sources;       // packed geometry, collected lazily per distinct source
    GeorepLookup georep;                // handle -> shape type for this scan (see GeorepLookup)
    a.bindBuildAttrs(gdp);
    georep.bind(a);

    // Compare every existing body's settings with its point only when something that can
    // change them was touched (a prop marker or the node's values), or when the object has
    // deforming bodies (rebuilt every step anyway). Otherwise the scan only syncs pose,
    // velocity and `active`. Convex / concave scale is checked on every scan (it costs a
    // matrix split of what pass 1 already read).
    const bool checkProps = !os.propsKnown || !os.fp.samePropMarkers(fpNow) || os.anyDeforming;
    bool       anyDeforming = false;
    bool       anyDriven    = false;

    // The object's own collision filter profile (its name and id); points with collisiongroup /
    // collisionignore get their own (pointFilterProfile).
    b3dop::FilterProfile baseProfile;
    {
        const UT_String &nm = obj.getName();
        if (nm.isstring())
            baseProfile.objectName = nm.buffer();
        baseProfile.objectId = std::to_string(objectId);
    }
    const int baseProfileIdx = world.filterProfile(baseProfile);

    // Bodies the post-step passes look at (impacts, activation by impact); rebuilt by every scan
    // and kept as they are while the object is skipped.
    os.impacts.clear();
    os.activation.clear();

    // ---- pass 1 (parallel): read every point's attributes -----------------
    // Reads only. The slots persist between steps (no allocation in a steady
    // state) and each caches its mirror entry. Offsets are gathered serially first
    // (cheap), so pass 1 can index by position; a cached entry is dropped when
    // its slot's offset or key changed. When there is no `id` attribute the body
    // id is the point number, which equals the position in a full point-range
    // walk; it is filled here so the parallel pass never calls pointIndex().
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
        s.ok     = readPointState(gdp, a, s.off, pk, s.cur);
        s.active = !a.active.isValid() || a.active.get(s.off) != 0;
        s.packed = pk.prim != nullptr;
        if (s.packed)
        {
            s.primOff = int64_t(pk.primOff);
            s.m3[0] = pk.m4[0];  s.m3[1] = pk.m4[1];  s.m3[2] = pk.m4[2];
            s.m3[3] = pk.m4[4];  s.m3[4] = pk.m4[5];  s.m3[5] = pk.m4[6];
            s.m3[6] = pk.m4[8];  s.m3[7] = pk.m4[9];  s.m3[8] = pk.m4[10];
        }
    }, anyPrims ? kMinParallelReadPacked : kMinParallelReadPlain);

    // A rebuild (fresh world, empty mirror) adds a body per point: size the mirror once instead
    // of rehashing it as it grows. Only slots without a cached entry can be new bodies, so a
    // steady-state scan (entries cached) asks for nothing; reserve() is a no-op when the
    // buckets already suffice.
    {
        size_t fresh = 0;
        for (const Box3DPointSlot &s : snaps)
            fresh += s.entry ? 0 : 1;
        if (fresh >= 64)
        {
            st.mirror.reserve(st.mirror.size() + fresh);
            st.world.reserve(fresh);
        }
    }

    // ---- pass 2 (serial): apply to Box3D and the mirror -------------------
    // Runs in point order, so "first duplicate id wins" holds. An entry whose
    // stamp is already the current one was claimed by an earlier point this step.
    for (Box3DPointSlot &s : snaps)
    {
        const GA_Offset off = s.off;
        const int64_t   bid = s.bid;
        const uint64_t  key = s.key;
        Box3DBodyPose  &cur = s.cur;

        DopBodyState *entry = s.entry;
        bool freshEntry = false;        // `entry` was just inserted for a body that does not exist yet
        auto newIt = st.mirror.end();   // ... and this is its node, in case the body cannot be built
        if (!entry)
        {
            if (s.ok)
            {
                // One hash lookup for both "known?" and "make room": a new node starts out
                // unstamped (syncStamp is never 0), so the duplicate check below ignores it.
                auto res   = st.mirror.try_emplace(key);
                entry      = &res.first->second;
                freshEntry = res.second;
                if (freshEntry)
                    newIt = res.first;
            }
            else
            {
                auto mit = st.mirror.find(key);     // a bad point never creates a body
                if (mit != st.mirror.end())
                    entry = &mit->second;
            }
            s.entry = entry;
        }

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

        // `dead`: the point is deleted after the step. Its body (if any) is not stamped, so it is
        // removed with the other bodies whose point is gone.
        if (a.dead.isValid() && a.dead.get(off) != 0)
        {
            Box3DDeletion d;
            d.objectId = objectId;
            d.off      = off;
            d.pointId  = bid;
            st.deletions.push_back(d);
            if (freshEntry)
            {
                st.mirror.erase(newIt);
                s.entry = nullptr;
            }
            ++c.dead;
            continue;
        }

        const bool active   = s.active;
        const bool animated = !active && a.animated.isValid() && a.animated.get(off) != 0;
        // Per point, read only when the attribute exists (RBD Packed Object creates both).
        const bool deforming   = a.deforming.isValid() && a.deforming.get(off) != 0;
        const bool refitWanted = a.autofitValid.isValid() && a.autofitValid.get(off) == 0;
        anyDeforming = anyDeforming || deforming;

        if (freshEntry)
        {
            PackedInfo pk = anyPrims ? getPackedInfo(gdp, off) : PackedInfo();    // only new bodies need it again
            readPackedPivot(pk);        // the hull is built from it
            Box3DBodyProps p = readDynamicProps(node, a, off);
            p.filterProfile = pointFilterProfile(world, a, off, baseProfile, baseProfileIdx);
            std::string fallbackNote;
            // The entry's address is stable (unordered_map node), so it goes into the body's
            // user data at creation: write-back maps a move event back to it from there.
            b3dop::BodyHandle handle = 0;
            if (!addPointBody(world, sources, georep, st.shapeTypes, node, a, off, bid, pk, cur, active, animated,
                              p, *entry, handle, &fallbackNote))
            {
                ++c.failed;
                st.mirror.erase(newIt);         // no body: drop the placeholder entry again
                s.entry = nullptr;
                continue;
            }
            if (!fallbackNote.empty())
                ++c.fallbacks[fallbackNote];
            DopBodyState &m = *entry;
            const b3dop::BodySettings::Type type = wantedType(node, active, animated, m.shapeless);
            static_cast<Box3DBodyPose &>(m) = cur;
            m.dynamic  = (type == b3dop::BodySettings::Type::Dynamic);
            m.type     = type;
            m.handle   = handle;
            m.pointOff = off;
            m.pointId  = bid;
            m.pointIndex = s.index;
            m.objectId = objectId;
            m.stamp    = st.syncStamp;
            m.sleeping = false;
            m.driven   = false;
            storePacked(m, s);
            ++st.stampedCount;
            ++stamped;
            s.entry = &m;
            ++c.added;
            hasDynamic = hasDynamic || m.dynamic;
            const unsigned writes = (refitWanted ? Box3DPointWrite::kAutofitValid : 0u) |
                                    (p.computeMass ? Box3DPointWrite::kMass : 0u);
            if (writes)
                queuePointWrite(st, m, writes, effectiveMass(p, m.volume));
            if (os.addImpact)
                os.impacts.push_back(&m);
            if (!active && p.minActivationImpulse > 0.0f)
                os.activation.push_back(&m);
            continue;
        }

        // Existing body.
        entry->pointOff = off;
        entry->pointId  = bid;
        entry->pointIndex = s.index;
        entry->objectId = objectId;
        entry->stamp    = st.syncStamp;
        storePacked(*entry, s);
        ++st.stampedCount;
        ++stamped;

        // ---- mid-simulation changes (see "Mid-simulation updates" in AGENTS.md) ----
        // Shape first: a rebuild can leave a body shapeless, which decides its type below.
        // Like Bullet, a shape is only rebuilt when asked (deforming, bullet_autofit_valid = 0)
        // or, for convex / concave, when its scale changed. Editing bullet_georep, bullet_prim*
        // etc. alone does nothing; the point `id` has no special values either.
        bool rebuild = deforming || refitWanted;
        if (!rebuild && entry->autoScale)
        {
            double key[3];
            rebuild = currentSizeKey(a, s, key) && !sameSizeKey(key, entry->sizeKey);
        }
        unsigned writes = refitWanted ? Box3DPointWrite::kAutofitValid : 0u;
        if (rebuild || checkProps)
        {
            Box3DBodyProps p = readDynamicProps(node, a, off);
            p.filterProfile = pointFilterProfile(world, a, off, baseProfile, baseProfileIdx);
            if (!a.wantDeactivate.isValid())
                p.sleepOk = entry->props.sleepOk;       // enablesleep is creation-time only
            bool shapesRebuilt = false;
            if (rebuild)
            {
                PackedInfo pk = anyPrims ? getPackedInfo(gdp, off) : PackedInfo();
                readPackedPivot(pk);
                std::string note;
                bool        noShape = false;
                shapesRebuilt = rebuildPointShapes(world, sources, georep, st.shapeTypes, node, a, off, bid,
                                                   pk, p, *entry, note, noShape);
                if (shapesRebuilt)
                {
                    ++c.rebuilt;
                    if (!note.empty())
                        ++c.fallbacks[note];
                    if (noShape)
                        ++c.fallbacks["the rebuilt collision shape could not be created; the body is static until it gets one"];
                }
            }
            const unsigned changed = applyPropChanges(world, *entry, p, shapesRebuilt);
            if (changed)
                ++c.updated;
            if (p.computeMass && (shapesRebuilt || (changed & kMassChanged)))
                writes |= Box3DPointWrite::kMass;
            // bullet_sleeping set to 0 by the user wakes a body the solver put to sleep.
            if (entry->sleeping && a.sleeping.isValid() && a.sleeping.get(off) == 0)
            {
                world.wakeByHandle(entry->handle);
                entry->sleeping = false;
            }
        }
        if (writes)
            queuePointWrite(st, *entry, writes, effectiveMass(entry->props, entry->volume));
        if (os.addImpact)
            os.impacts.push_back(entry);
        if (!active && entry->props.minActivationImpulse > 0.0f)
            os.activation.push_back(entry);

        const b3dop::BodySettings::Type wantType = wantedType(node, active, animated, entry->shapeless);

        // Did `active` / `animated` flip?
        if (entry->type != wantType)
        {
            world.setTypeByHandle(entry->handle, wantType);
            entry->type    = wantType;
            entry->dynamic = (wantType == b3dop::BodySettings::Type::Dynamic);
            entry->driven  = false;
            // A type change recomputes the mass from the shapes (centre at their centroid): put
            // the centre back at the point.
            if (entry->dynamic)
                applyBodyMass(world, entry->handle, entry->props, entry->volume);
            ++c.retyped;
        }
        entry->animated = animated;

        if (animated && entry->type == b3dop::BodySettings::Type::Kinematic)
        {
            // An animated body follows its point: a new pose becomes a target the body moves to
            // over this step (so it pushes what it touches). When the point stops moving, the
            // velocity the target left behind is cleared.
            if (describeStateDifference(cur, *entry))
            {
                world.setTargetTransformByHandle(entry->handle, cur.pos, cur.rot, st.stepDt);
                static_cast<Box3DBodyPose &>(*entry) = cur;
                entry->driven = true;
                ++c.edited;
            }
            else if (entry->driven)
            {
                const float zero[3] = { 0.0f, 0.0f, 0.0f };
                world.setVelocityByHandle(entry->handle, zero, zero);
                entry->driven = false;
            }
            anyDriven = anyDriven || entry->driven;
        }
        // Did something else edit P / orient / v / w since we wrote it?
        else if (describeStateDifference(cur, *entry))
        {
            world.setTransformByHandle(entry->handle, cur.pos, cur.rot);
            world.setVelocityByHandle(entry->handle, cur.lin, cur.ang);
            static_cast<Box3DBodyPose &>(*entry) = cur;
            ++c.edited;
        }
        hasDynamic = hasDynamic || entry->dynamic;
    }

    // Record the markers for the next step. Bodies that failed, duplicated an id or had a
    // bad P are retried / reported every step, so such an object keeps scanning.
    // An object with deforming bodies rebuilds them every step, so it never skips.
    os.entryCount   = stamped;
    os.hasDynamic   = hasDynamic;
    os.fp           = fpNow;
    os.propsKnown   = true;
    os.anyDeforming = anyDeforming;
    os.anyDriven    = anyDriven;
    os.canSkip      = (c.failed + c.duplicates + c.badP) == problems0 && !anyDeforming && !anyDriven;
    return hasDynamic;
}

// Make sure `orient`, `v` and `w` exist on the points so results have
// somewhere to go. Returns true if `orient` was just created.
bool
ensureResultAttribs(GU_Detail *gdp)
{
    bool createdOrient = false;

    if (!gdp->findPointAttribute("v"))
    {
        gdp->addFloatTuple(GA_ATTRIB_POINT, "v", 3);
        if (GA_Attribute *attr = gdp->findPointAttribute("v"))
            attr->setTypeInfo(GA_TYPE_VECTOR);
    }
    if (!gdp->findPointAttribute("w"))
    {
        gdp->addFloatTuple(GA_ATTRIB_POINT, "w", 3);
        if (GA_Attribute *attr = gdp->findPointAttribute("w"))
            attr->setTypeInfo(GA_TYPE_VECTOR);
    }
    if (!gdp->findPointAttribute("orient"))
    {
        gdp->addFloatTuple(GA_ATTRIB_POINT, "orient", 4);
        if (GA_Attribute *attr = gdp->findPointAttribute("orient"))
            attr->setTypeInfo(GA_TYPE_QUATERNION);
        createdOrient = true;
    }
    return createdOrient;
}

// Harden only the parts of P / orient / v / w that the parallel write pass will touch.
// The HDK has no single-page call; hardenAllPages takes an offset range (the same thing
// GA_AutoHardenForThreading wraps), so the written offsets are grouped into buckets, runs of
// neighbouring buckets become ranges, and each range is hardened. The bucket size is only a
// grouping unit (the HDK widens a range to whole pages itself), so it need not equal the
// real page size: too big hardens a little extra, too small just makes more calls. When
// nearly every bucket is hit (awake bodies scattered over the whole geometry) one full
// call is cheaper than many ranges. `end` is passed as the first offset after the last
// bucket, which is correct whether the HDK reads it as exclusive or inclusive.
void
hardenWrittenPages(const GU_Detail *gdp, const std::vector<Box3DWriteItem> &items, size_t begin, size_t end,
                   std::initializer_list<GA_Attribute *> attrs)
{
    constexpr int64_t kBucketBits = 10;                     // 1024 offsets
    const int64_t numOffsets = int64_t(gdp->getNumPointOffsets());
    const size_t  numBuckets = size_t((numOffsets >> kBucketBits) + 1);
    std::vector<char> hit(numBuckets, 0);
    size_t hitCount = 0;
    for (size_t k = begin; k < end; ++k)
    {
        const GA_Offset off = items[k].off;
        if (off == GA_INVALID_OFFSET || int64_t(off) < 0 || int64_t(off) >= numOffsets)
            continue;
        char &h = hit[size_t(int64_t(off) >> kBucketBits)];
        hitCount += h ? 0 : 1;
        h = 1;
    }
    if (hitCount == 0)
        return;

    const bool nearlyAll = hitCount * 10 >= numBuckets * 9;
    for (GA_Attribute *attr : attrs)
    {
        if (!attr)
            continue;
        if (nearlyAll)
        {
            attr->hardenAllPages();
            continue;
        }
        for (size_t b = 0; b < numBuckets;)
        {
            if (!hit[b]) { ++b; continue; }
            size_t e = b;
            while (e + 1 < numBuckets && hit[e + 1])
                ++e;
            attr->hardenAllPages(GA_Offset(int64_t(b) << kBucketBits),
                                 GA_Offset(int64_t(e + 1) << kBucketBits));
            b = e + 1;
        }
    }
}

// Sets each item's point offset in the geometry that is about to be written. The
// sync pass stored an offset in every mirror entry, and the writable geometry is
// normally an exact copy of what was read, so the stored offset still names the same
// point. That is checked (offset in range, and the point's id still matches) because
// a copy that renumbered points would otherwise write results to the wrong ones. A
// stale offset is looked up again by the body's point id. An item whose point cannot
// be found gets GA_INVALID_OFFSET and is skipped.
template <class Item>       // Box3DWriteItem or Box3DPointWrite: needs .entry and .off
void
resolvePointOffsets(const GU_Detail *gdp, const PointAttrs &a,
                    std::vector<Item> &items, size_t begin, size_t end)
{
    std::unordered_map<int64_t, GA_Offset> byId;     // built only if an `id` lookup is needed
    bool builtById = false;

    for (size_t k = begin; k < end; ++k)
    {
        Item                  &w = items[k];
        const Box3DBodyMirror &m = *w.entry;
        const GA_Offset        stored = GA_Offset(m.pointOff);

        bool ok = stored >= 0 && stored < gdp->getNumPointOffsets() &&
                  !gdp->getPointMap().isOffsetVacant(stored);
        if (ok)
            ok = a.id.isValid() ? int64_t(a.id.get(stored)) == m.pointId
                                : int64_t(gdp->pointIndex(stored)) == m.pointId;
        if (ok)
        {
            w.off = stored;
            continue;
        }

        w.off = GA_INVALID_OFFSET;
        if (!a.id.isValid())
        {
            if (m.pointId >= 0 && m.pointId < int64_t(gdp->getNumPoints()))
                w.off = gdp->pointOffset(GA_Index(m.pointId));
            continue;
        }
        if (!builtById)
        {
            builtById = true;
            for (GA_Iterator it(gdp->getPointRange()); !it.atEnd(); ++it)
                byId.emplace(int64_t(a.id.get(*it)), *it);     // first point of a duplicate id wins
        }
        const auto f = byId.find(m.pointId);
        if (f != byId.end())
            w.off = f->second;
    }
}

// Returns a writable SIM_GeometryCopy for the object's "Geometry" data.
//
// Objects from RBD Packed Object (and other SOP-referencing DOPs) carry a
// Geometry that is a SIM_Geometry but not a SIM_GeometryCopy, so a plain
// SIM_DATA_GET<SIM_GeometryCopy> returns null. SIM_DATA_RETURN_EXISTING returns
// the data untouched when it already is a SIM_GeometryCopy. When it is the wrong
// type, a new SIM_GeometryCopy replaces it, and SIM_DATA_ADOPT_EXISTING_ON_DELETE
// copies the old geometry into the new data. This conversion happens once per
// object; afterwards the data is already a SIM_GeometryCopy.
SIM_GeometryCopy *
getOrMakeWritableGeometry(SIM_Object &obj)
{
    if (SIM_GeometryCopy *geo = SIM_DATA_GET(obj, SIM_GEOMETRY_DATANAME, SIM_GeometryCopy))
        return geo;

    if (!SIM_DATA_GETCONST(obj, SIM_GEOMETRY_DATANAME, SIM_Geometry))
        return nullptr;     // no Geometry at all: nothing to convert

    SIM_GeometryCopy *geo = SIM_DATA_CREATE(obj, SIM_GEOMETRY_DATANAME, SIM_GeometryCopy,
            SIM_DATA_RETURN_EXISTING | SIM_DATA_ADOPT_EXISTING_ON_DELETE);
    if (geo)
        B3D_LOG("  writeBack: object id=%d Geometry converted to a writable SIM_GeometryCopy",
                obj.getObjectId());
    return geo;
}


// Re-record an object's fingerprint after the solver wrote to its geometry, from a read of
// the geometry as the next step will see it (call with every write lock released: releasing
// one may bump ids again). Otherwise our own write would look like an external edit: it
// would defeat the sync skip, and a written prop marker (bullet_autofit_valid) would make
// the next scan compare every body's settings.
void
recordFingerprintAfterWrite(Box3DSharedState &st, const SIM_Object &obj)
{
    const auto f = st.objectSync.find(obj.getObjectId());
    if (f == st.objectSync.end() || f->second.visited != st.syncStamp)
        return;
    const SIM_Geometry *geo = SIM_DATA_GETCONST(obj, SIM_GEOMETRY_DATANAME, SIM_Geometry);
    if (!geo)
        return;
    SIM_GeometryAutoReadLock rlock(geo);
    const GU_Detail *gdp = gdpPtr(rlock.getGdp());
    if (!gdp)
        return;
    const PointAttrs a(gdp);
    f->second.fp = makeFingerprint(gdp, a, f->second.fp.nodeType, f->second.fp.nodeHash);
}

// `objects` is sorted by object id, so an object is found by binary search. (A linear scan
// is the fallback should that ever not hold.)
SIM_Object *
findObjectById(SIM_ObjectArray &objects, int id)
{
    exint lo = 0, hi = objects.entries();
    while (lo < hi)
    {
        const exint mid = lo + (hi - lo) / 2;
        const int   mid_id = objects(mid)->getObjectId();
        if (mid_id == id) return objects(mid);
        if (mid_id < id) lo = mid + 1; else hi = mid;
    }
    for (exint i = 0; i < objects.entries(); ++i)
        if (objects(i)->getObjectId() == id)
            return objects(i);
    return nullptr;
}

// Make the object's next sync scan (not skip): after a write that the scan must act on
// (active = 1, dead = 1, deleted points), since re-recording the fingerprint hides the write.
void
forceScan(Box3DSharedState &st, int objectId)
{
    const auto f = st.objectSync.find(objectId);
    if (f != st.objectSync.end())
        f->second.canSkip = false;
}

// Runs of items with the same objectId, sorted first if needed. f(begin, end) per run.
template <class Item, class F>
void
forEachObjectRun(std::vector<Item> &items, const F &f)
{
    const auto byObject = [](const Item &x, const Item &y) { return x.objectId < y.objectId; };
    if (!std::is_sorted(items.begin(), items.end(), byObject))     // the sync fills them in object order
        std::stable_sort(items.begin(), items.end(), byObject);
    for (size_t runBegin = 0; runBegin < items.size();)
    {
        size_t runEnd = runBegin;
        while (runEnd < items.size() && items[runEnd].objectId == items[runBegin].objectId)
            ++runEnd;
        f(runBegin, runEnd);
        runBegin = runEnd;
    }
}

// Add Impact Data (Box3D Data): every contact point of this step that carried impulse goes into
// the object's "Impacts" data (SIM_Impacts, what Bullet fills), one impact per contact point.
// As in Houdini, the data is only there on a step with at least one impact: it is created fresh
// for such a step and removed otherwise, so it never holds an earlier step's impacts. Contacts
// where neither body is awake (a resting, sleeping pile) are not impacts. The normal is the
// surface normal of what was hit, pointing at this body. No Feedback data: that is for mutual
// interaction with objects of other solvers, which this solver does not do.
void
recordImpacts(Box3DSharedState &st, SIM_ObjectArray &objects, const SIM_Time &now)
{
    struct Hit { const DopBodyState *self; b3dop::ContactPoint cp; };
    std::vector<Hit> hits;
    std::vector<int> wrote;
    for (auto &kv : st.objectSync)
    {
        const Box3DObjectSync &os = kv.second;
        if (os.visited != st.syncStamp || !os.addImpact || os.impacts.empty())
            continue;
        hits.clear();
        for (const DopBodyState *e : os.impacts)
        {
            if (!e->handle)
                continue;
            const bool selfAwake = st.world.isAwakeByHandle(e->handle);
            if (e->dynamic && !selfAwake)
                continue;                       // asleep: nothing new happened to it
            st.world.getContactsByHandle(e->handle, st.contacts);
            for (const b3dop::ContactPoint &cp : st.contacts)
                if (selfAwake || st.world.isAwakeByHandle(cp.other))
                    hits.push_back({ e, cp });
        }
        if (hits.empty())
            continue;
        SIM_Object *obj = findObjectById(objects, kv.first);
        if (!obj)
            continue;
        SIM_Impacts *imp = SIM_DATA_CREATE(*obj, SIM_IMPACTS_DATANAME, SIM_Impacts, 0);
        if (!imp)
            continue;
        wrote.push_back(kv.first);
        for (const Hit &h : hits)
        {
            const b3dop::ContactPoint &cp = h.cp;
            const DopBodyState *o = static_cast<const DopBodyState *>(cp.otherUserData);
            imp->addPointImpact(UT_Vector3(float(cp.pos[0]), float(cp.pos[1]), float(cp.pos[2])),
                                UT_Vector3(-cp.normal[0], -cp.normal[1], -cp.normal[2]),
                                cp.impulse, int(h.self->pointIndex),
                                o ? o->objectId : -1, o ? int(o->pointIndex) : -1,
                                -1, 0.0, 0.0, 0.0, now, 0);
        }
    }
    // Objects that had Impacts data last step but no impact now (or the option was turned off).
    std::sort(wrote.begin(), wrote.end());
    for (const int id : st.impactObjects)
        if (!std::binary_search(wrote.begin(), wrote.end(), id))
            if (SIM_Object *obj = findObjectById(objects, id))
                obj->removeNamedSubData(SIM_IMPACTS_DATANAME);
    st.impactObjects.swap(wrote);
}

// min_activation_impulse: an inactive body hit with at least that impulse becomes active, as
// in Bullet. Hits from bodies matching its activationignore patterns (collision group, object
// name or id) do not count. Box3D only reports the impulse after the step, so the body turns
// dynamic now and moves from the next step on (Bullet can activate it within the step). The
// point's `active` is set to 1 after the step.
void
activateByImpact(Box3DSharedState &st)
{
    for (auto &kv : st.objectSync)
    {
        const Box3DObjectSync &os = kv.second;
        if (os.visited != st.syncStamp)
            continue;
        for (DopBodyState *e : os.activation)
        {
            if (!e->handle || e->dynamic || e->props.minActivationImpulse <= 0.0f)
                continue;
            st.world.getContactsByHandle(e->handle, st.contacts);
            bool hit = false;
            for (const b3dop::ContactPoint &cp : st.contacts)
            {
                if (cp.impulse < e->props.minActivationImpulse)
                    continue;
                const DopBodyState *o = static_cast<const DopBodyState *>(cp.otherUserData);
                if (o && !e->props.activationIgnore.empty())
                    if (const b3dop::FilterProfile *fp = st.world.filterProfileData(o->props.filterProfile))
                        if (b3dop::patternsMatchProfile(e->props.activationIgnore, *fp))
                            continue;
                hit = true;
                break;
            }
            if (!hit)
                continue;
            st.world.setTypeByHandle(e->handle, b3dop::BodySettings::Type::Dynamic);
            e->type     = b3dop::BodySettings::Type::Dynamic;
            e->dynamic  = true;
            e->animated = false;
            e->driven   = false;
            applyBodyMass(st.world, e->handle, e->props, e->volume);
            st.world.wakeByHandle(e->handle);
            queuePointWrite(st, *e, Box3DPointWrite::kActivate);
        }
    }
}

// The point attribute writes the sync and the activation queued (Box3DPointWrite):
// bullet_autofit_valid = 1 (as Bullet does once a fit is computed), `mass` with computemass,
// and `active` = 1 for activated bodies. Covers every such point, asleep, static or ignored.
// A point that cannot be found keeps its old value (a missed bullet_autofit_valid = 1 just
// refits again next step, which is harmless).
void
writePointAttrs(Box3DSharedState &st, SIM_ObjectArray &objects)
{
    std::vector<Box3DPointWrite> &items = st.pointWrites;
    if (items.empty())
        return;
    forEachObjectRun(items, [&](size_t begin, size_t end) {
        SIM_Object *obj = findObjectById(objects, items[begin].objectId);
        if (!obj)
            return;
        SIM_GeometryCopy *geo = getOrMakeWritableGeometry(*obj);
        if (!geo)
            return;             // writeResultsBack has already warned about this object
        unsigned want = 0;
        for (size_t k = begin; k < end; ++k)
            want |= items[k].flags;
        unsigned wrote = 0;
        {
            SIM_GeometryAutoWriteLock lock(geo);
            GU_Detail *gdp = gdpPtr(lock.getGdp());
            if (!gdp)
                return;
            if ((want & Box3DPointWrite::kMass) && !gdp->findPointAttribute("mass"))
                gdp->addFloatTuple(GA_ATTRIB_POINT, "mass", 1);     // Houdini reports the computed mass here
            GA_RWHandleI valid(gdp, GA_ATTRIB_POINT, "bullet_autofit_valid");
            GA_RWHandleF mass(gdp, GA_ATTRIB_POINT, "mass");
            GA_RWHandleI act(gdp, GA_ATTRIB_POINT, "active");
            const PointAttrs a(gdp);
            resolvePointOffsets(gdp, a, items, begin, end);
            for (size_t k = begin; k < end; ++k)
            {
                const Box3DPointWrite &w = items[k];
                if (w.off == GA_INVALID_OFFSET)
                    continue;
                if ((w.flags & Box3DPointWrite::kAutofitValid) && valid.isValid())
                {
                    valid.set(w.off, 1);
                    wrote |= Box3DPointWrite::kAutofitValid;
                }
                if ((w.flags & Box3DPointWrite::kMass) && mass.isValid())
                {
                    mass.set(w.off, w.mass);
                    wrote |= Box3DPointWrite::kMass;
                }
                if ((w.flags & Box3DPointWrite::kActivate) && act.isValid())
                {
                    act.set(w.off, 1);
                    wrote |= Box3DPointWrite::kActivate;
                }
            }
            if (wrote & Box3DPointWrite::kAutofitValid) valid.getAttribute()->bumpDataId();
            if (wrote & Box3DPointWrite::kMass)         mass.getAttribute()->bumpDataId();
            if (wrote & Box3DPointWrite::kActivate)     act.getAttribute()->bumpDataId();
        }
        if (wrote)
            recordFingerprintAfterWrite(st, *obj);
        if (wrote & Box3DPointWrite::kActivate)
            forceScan(st, obj->getObjectId());      // the scan must see active = 1 and keep the body dynamic
    });
    items.clear();
}

// Enable Aging: like Bullet, add the step to every point's `age` (created if missing) and set
// `dead` = 1 once age exceeds a positive `life`; the next step deletes those points.
void
ageObjects(Box3DSharedState &st, SIM_ObjectArray &objects, float dt)
{
    for (exint i = 0; i < objects.entries(); ++i)
    {
        SIM_Object *obj = objects(i);
        if (!SIM_DATA_GETCONST(*obj, SIM_GEOMETRY_DATANAME, SIM_Geometry))
            continue;
        SIM_GeometryCopy *geo = getOrMakeWritableGeometry(*obj);
        if (!geo)
            continue;
        bool markedDead = false;
        {
            SIM_GeometryAutoWriteLock lock(geo);
            GU_Detail *gdp = gdpPtr(lock.getGdp());
            if (!gdp)
                continue;
            if (!gdp->findPointAttribute("age"))
                gdp->addFloatTuple(GA_ATTRIB_POINT, "age", 1);
            const bool hasLife = gdp->findPointAttribute("life") != nullptr;
            if (hasLife && !gdp->findPointAttribute("dead"))
                gdp->addIntTuple(GA_ATTRIB_POINT, "dead", 1);
            GA_RWHandleF age(gdp, GA_ATTRIB_POINT, "age");
            GA_ROHandleF life(gdp, GA_ATTRIB_POINT, "life");
            GA_RWHandleI dead(gdp, GA_ATTRIB_POINT, "dead");
            if (!age.isValid())
                continue;
            for (GA_Iterator it(gdp->getPointRange()); !it.atEnd(); ++it)
            {
                const GA_Offset off = *it;
                const float a = age.get(off) + dt;
                age.set(off, a);
                if (life.isValid() && dead.isValid())
                {
                    const float l = life.get(off);
                    if (l > 0.0f && a > l && dead.get(off) == 0)
                    {
                        dead.set(off, 1);
                        markedDead = true;
                    }
                }
            }
            age.getAttribute()->bumpDataId();
            if (markedDead)
                dead.getAttribute()->bumpDataId();
        }
        recordFingerprintAfterWrite(st, *obj);
        if (markedDead)
            forceScan(st, obj->getObjectId());
    }
}

// `dead` points: their bodies were removed by the sync; now delete the points (a packed point
// together with its primitive). Each is checked to still be the same point (offset in range,
// same `id` or point number) before anything is deleted.
void
deleteDeadPoints(Box3DSharedState &st, SIM_ObjectArray &objects)
{
    std::vector<Box3DDeletion> &items = st.deletions;
    if (items.empty())
        return;
    forEachObjectRun(items, [&](size_t begin, size_t end) {
        SIM_Object *obj = findObjectById(objects, items[begin].objectId);
        if (!obj)
            return;
        SIM_GeometryCopy *geo = getOrMakeWritableGeometry(*obj);
        if (!geo)
            return;
        bool deleted = false;
        {
            SIM_GeometryAutoWriteLock lock(geo);
            GU_Detail *gdp = gdpPtr(lock.getGdp());
            if (!gdp)
                return;
            const GA_ROHandleI id(gdp, GA_ATTRIB_POINT, "id");
            // Check every point first: deleting changes point numbers.
            std::vector<GA_Offset> doomed;
            for (size_t k = begin; k < end; ++k)
            {
                const GA_Offset off = items[k].off;
                bool ok = off >= 0 && off < gdp->getNumPointOffsets() && !gdp->getPointMap().isOffsetVacant(off);
                if (ok)
                    ok = id.isValid() ? int64_t(id.get(off)) == items[k].pointId
                                      : int64_t(gdp->pointIndex(off)) == items[k].pointId;
                if (ok)
                    doomed.push_back(off);
            }
            for (const GA_Offset off : doomed)
            {
                const GA_Offset vtx  = gdp->pointVertex(off);
                GA_Primitive   *prim = GAisValid(vtx) ? gdp->getPrimitive(gdp->vertexPrimitive(vtx)) : nullptr;
                if (prim && GU_PrimPacked::isPackedPrimitive(*prim))
                    gdp->destroyPrimitive(*prim, true);         // and its point
                else
                    gdp->destroyPointOffset(off, GA_Detail::GA_DESTROY_DEGENERATE);
                deleted = true;
            }
            if (deleted)
                gdp->bumpAllDataIds();
        }
        if (deleted)
        {
            recordFingerprintAfterWrite(st, *obj);
            forceScan(st, obj->getObjectId());      // point numbers changed
        }
    });
    items.clear();
}
} // namespace

// ---------------------------------------------------------------------------

SIM_SolverBox3D::SIM_SolverBox3D(const SIM_DataFactory *factory)
    : BaseClass(factory),
      SIM_OptionsUser(this)
{
}

SIM_SolverBox3D::~SIM_SolverBox3D()
{
}

const SIM_DopDescription *
SIM_SolverBox3D::getDescription()
{
    static PRM_Name    theGravityName("gravity", "Gravity");
    static PRM_Default theGravityDefault[] = { 0.0, -9.80665, 0.0 };

    static PRM_Name    theSubStepsName("substeps", "Substeps");
    static PRM_Default theSubStepsDefault(4);
    // Box3D clamps substeps to >= 1 (b3World_Step), so 0 is never meaningful.
    // Hard minimum 1; the slider stops at 16 but typing a larger value is allowed.
    static PRM_Range   theSubStepsRange(PRM_RANGE_RESTRICTED, 1, PRM_RANGE_UI, 16);

    static PRM_Name    theAutoThreadsName("autothreads", "Use All Threads");
    static PRM_Default theAutoThreadsDefault(1);

    static PRM_Name    theThreadsName("threads", "Threads");
    static PRM_Default theThreadsDefault(8);
    // Box3D clamps workers to [1, B3_MAX_WORKERS], and B3_MAX_WORKERS is 32.
    static PRM_Range   theThreadsRange(PRM_RANGE_RESTRICTED, 1, PRM_RANGE_RESTRICTED, 32);
    // Greyed out while "Use All Threads" is ticked.
    static PRM_Conditional theThreadsDisable("{ autothreads == 1 }", PRM_CONDTYPE_DISABLE);

    static PRM_Name    theSleepName("enablesleep", "Allow Sleeping");
    static PRM_Default theSleepDefault(1);

    // Time Scale: like Bullet's, multiplies the time step the physics advances by.
    static PRM_Name    theTimeScaleName("timescale", "Time Scale");
    static PRM_Default theTimeScaleDefault(1.0);
    static PRM_Range   theTimeScaleRange(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 2);

    // Box3D's contact speed: the fastest overlapping shapes are pushed apart. Lower values
    // separate pieces that start out overlapping more gently.
    static PRM_Name    thePushSpeedName("contactpushspeed", "Overlap Push Speed");
    static PRM_Default thePushSpeedDefault(3.0);
    static PRM_Range   thePushSpeedRange(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 10);

    static PRM_Name    theAgingName("enableaging", "Enable Aging");
    static PRM_Default theAgingDefault(0);

    // Box3D's own solver tuning (Box3D defaults). Bullet's ERP / CFM / iterations have no
    // equivalent: Box3D's solver is soft-step with substeps, tuned by these.
    static PRM_Name    theStiffnessName("contactstiffness", "Contact Stiffness");
    static PRM_Default theStiffnessDefault(30.0);
    static PRM_Range   theStiffnessRange(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 120);
    static PRM_Name    theDampingName("contactdamping", "Contact Damping Ratio");
    static PRM_Default theDampingDefault(10.0);
    static PRM_Range   theDampingRange(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 20);
    static PRM_Name    theBounceThresholdName("bouncethreshold", "Bounce Threshold");
    static PRM_Default theBounceThresholdDefault(1.0);
    static PRM_Range   theBounceThresholdRange(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 5);
    static PRM_Name    theMaxSpeedName("maxlinearspeed", "Max Linear Speed");
    static PRM_Default theMaxSpeedDefault(400.0);
    static PRM_Range   theMaxSpeedRange(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 1000);
    static PRM_Name    theContinuousName("enablecontinuous", "Continuous Collision");
    static PRM_Default theContinuousDefault(1);
    static PRM_Name    theSpeculativeName("enablespeculative", "Speculative Contacts");
    static PRM_Default theSpeculativeDefault(1);

    static PRM_Template theTemplates[] = {
        PRM_Template(PRM_FLT_J,  3, &theGravityName,  theGravityDefault),
        PRM_Template(PRM_INT_J,  1, &theSubStepsName, &theSubStepsDefault, 0, &theSubStepsRange),
        PRM_Template(PRM_TOGGLE, 1, &theAutoThreadsName, &theAutoThreadsDefault),
        PRM_Template(PRM_INT_J,  1, &theThreadsName,  &theThreadsDefault, 0, &theThreadsRange,
                     0, 0, 1,
                     "Number of Box3D worker threads. Only used when Use All Threads is off.",
                     &theThreadsDisable),
        PRM_Template(PRM_TOGGLE, 1, &theSleepName,    &theSleepDefault),
        PRM_Template(PRM_FLT_J,  1, &theTimeScaleName, &theTimeScaleDefault, 0, &theTimeScaleRange, 0, 0, 1,
                     "Multiplies the time step: below 1 is slow motion, above 1 fast motion."),
        PRM_Template(PRM_FLT_J,  1, &thePushSpeedName, &thePushSpeedDefault, 0, &thePushSpeedRange, 0, 0, 1,
                     "How fast overlapping pieces are pushed apart. Lower it if pieces that start out overlapping fly apart."),
        PRM_Template(PRM_TOGGLE, 1, &theAgingName,    &theAgingDefault, 0, 0, 0, 0, 1,
                     "Add the time step to each point's age, and mark points dead once age exceeds life."),
        PRM_Template(PRM_FLT_J,  1, &theStiffnessName, &theStiffnessDefault, 0, &theStiffnessRange, 0, 0, 1,
                     "How stiff contacts are (cycles per second). Higher values recover from overlap faster but can jitter."),
        PRM_Template(PRM_FLT_J,  1, &theDampingName, &theDampingDefault, 0, &theDampingRange, 0, 0, 1,
                     "How much contacts damp overlap recovery. Lower values recover faster but more energetically."),
        PRM_Template(PRM_FLT_J,  1, &theBounceThresholdName, &theBounceThresholdDefault, 0, &theBounceThresholdRange, 0, 0, 1,
                     "Collisions slower than this speed don't bounce, so resting pieces settle instead of jittering."),
        PRM_Template(PRM_FLT_J,  1, &theMaxSpeedName, &theMaxSpeedDefault, 0, &theMaxSpeedRange, 0, 0, 1,
                     "No body moves faster than this."),
        PRM_Template(PRM_TOGGLE, 1, &theContinuousName, &theContinuousDefault, 0, 0, 0, 0, 1,
                     "Stop fast bodies passing through static ones (and, for Fast Object bodies, moving ones)."),
        PRM_Template(PRM_TOGGLE, 1, &theSpeculativeName, &theSpeculativeDefault, 0, 0, 0, 0, 1,
                     "Create contacts just before shapes touch. Leave on unless you see bodies catch on invisible edges."),
        PRM_Template()
    };

    static SIM_DopDescription theDopDescription(
        true,                   // true: create a DOP node for this solver
        "box3d_solver",         // internal node name
        "Box3D Solver",         // node label
        "Solver",               // data name
        classname(),
        theTemplates);

    return &theDopDescription;
}

int
SIM_SolverBox3D::getRequestedCacheSubclass() const
{
    // SIM_Solver's default is 0. The header describes this as "the amount of
    // history required by this solver". We ask for 1 timestep of history so
    // the engine keeps the previous step (the world we continue from) alive.
    // To experiment without rebuilding (debug builds only):
    //     set BOX3DDOP_REQUESTED_CACHE=0
    const int value = B3D_ENV_INT("BOX3DDOP_REQUESTED_CACHE", 1);
    return value;
}

int
SIM_SolverBox3D::currentWorkerCount() const
{
    // Box3D runs its own thread pool (not Houdini's). "Use All Threads" means every
    // logical core; otherwise use the Threads parm. Read once per step, so the core
    // count is looked up once.
    static const int cores = int(std::thread::hardware_concurrency());
    int n = getAutoThreads() ? cores : int(getThreads());
    return n < 1 ? 1 : (n > 32 ? 32 : n);       // Box3D's maximum is 32
}

b3dop::WorldSettings
SIM_SolverBox3D::currentWorldSettings() const
{
    b3dop::WorldSettings s;
    const UT_Vector3 g = getGravity();
    s.gravity[0]  = g.x();
    s.gravity[1]  = g.y();
    s.gravity[2]  = g.z();
    s.subSteps    = static_cast<int>(getSubSteps());
    s.workerCount = currentWorkerCount();
    s.enableSleep = getEnableSleep();
    s.contactPushSpeed = float(getContactPushSpeed());
    s.contactHertz     = float(getContactStiffness());
    s.contactDampingRatio  = float(getContactDamping());
    s.restitutionThreshold = float(getBounceThreshold());
    s.maxLinearSpeed   = float(getMaxLinearSpeed());
    s.enableContinuous = getEnableContinuous();
    s.enableSpeculative = getEnableSpeculative();
    return s;
}

// ---------------------------------------------------------------------------
// The step
// ---------------------------------------------------------------------------

SIM_Solver::SIM_Result
SIM_SolverBox3D::solveObjectsSubclass(SIM_Engine &engine,
                                      SIM_ObjectArray &objects,
                                      SIM_ObjectArray & /*newobjects*/,
                                      SIM_ObjectArray & /*feedbacktoobjects*/,
                                      const SIM_Time &timestep)
{
    // Whatever the engine reports as "now" during a solve. We never assume
    // whether that is the start or the end of the step; see canContinue().
    const double now = engine.getSimulationTime();
    const double dt  = timestep;

    if (objects.entries() == 0)
        return SIM_SOLVER_SUCCESS;

    SIM_Box3DWorld *worldData = findOrCreateWorldData(objects);
    if (!worldData)
    {
        addError(objects(0), SIM_MESSAGE, "Box3D: could not create world data", UT_ERROR_ABORT);
        return SIM_SOLVER_FAIL;
    }

    // ---- is the cached world usable? ---------------------------------------
    char        reasonBuf[200];
    const char *reason = nullptr;
    {
        Box3DSharedState *st = worldData->state();
        if (!st || !st->world.isValid())
        {
            reason = "no valid world (first step, reset, or copied from empty)";
        }
        else
        {
            std::lock_guard<std::mutex> lock(st->mutex);
            if (st->world.settings().workerCount != currentWorkerCount())
            {
                // Box3D sizes its thread pool when the world is created, so a
                // new thread count means a new world. The DOP objects hold the
                // state, so the rebuild only loses warm-starting and sleep.
                reason = "thread count changed";
            }
            else if (!st->canContinue(now, dt))
            {
                std::snprintf(reasonBuf, sizeof(reasonBuf),
                              "stamp mismatch: stamped=%d stamp=%g now=%g dt=%g (now-stamp=%g)",
                              (int)st->stamped, st->stampTime, now, dt, now - st->stampTime);
                reason = reasonBuf;
            }
        }
    }

    if (reason)
        createFreshWorld(*worldData, reason);
    Box3DSharedState *st = worldData->state();
    if (!st || !st->world.isValid())
    {
        addError(objects(0), SIM_MESSAGE, "Box3D: world creation failed", UT_ERROR_ABORT);
        return SIM_SOLVER_FAIL;
    }

    std::lock_guard<std::mutex> lock(st->mutex);

    // Time Scale: the physics advances by dt * scale (Bullet's "Time Scale"). Animated bodies
    // reach their targets over that step.
    const double timeScale = std::max(0.0, double(getTimeScale()));
    st->stepDt = float(dt * timeScale);
    // Every world setting but the thread count can change on a live world (animated parameters).
    st->world.applyTuning(currentWorldSettings());

    std::vector<char> objectHasDynamic;
    syncFromGeometry(*worldData, objects, objectHasDynamic);
    stepWorld(*worldData, st->stepDt);
    writeResultsBack(*worldData, objects, objectHasDynamic);
    // After the step, in this order: impacts and activation read Box3D's contacts; then the
    // queued point writes (bullet_autofit_valid, computed mass, active); aging; deleting dead points.
    recordImpacts(*st, objects, engine.getSimulationTime());
    activateByImpact(*st);
    writePointAttrs(*st, objects);
    if (getEnableAging())
        ageObjects(*st, objects, float(dt));
    deleteDeadPoints(*st, objects);

    st->stampTime = now;
    st->stamped   = true;

    publishEvents(*worldData, objects);
    return SIM_SOLVER_SUCCESS;
}

// ---------------------------------------------------------------------------

SIM_Box3DWorld *
SIM_SolverBox3D::findOrCreateWorldData(SIM_ObjectArray &objects)
{
    // objects is sorted by object id. Use the first object that already
    // carries world data, so the carrier can change without losing the world.
    // The const accessor does not trigger a copy; only the chosen carrier is
    // fetched non-const, which is the access that causes the (cheap) copy.
    for (exint i = 0; i < objects.entries(); ++i)
    {
        SIM_Object *obj = objects(i);
        if (SIM_DATA_GETCONST(*obj, kWorldDataName, SIM_Box3DWorld))
        {
            SIM_Box3DWorld *w = SIM_DATA_GET(*obj, kWorldDataName, SIM_Box3DWorld);
            return w;
        }
    }

    SIM_Object *carrier = objects(0);
    SIM_Box3DWorld *w = SIM_DATA_CREATE(*carrier, kWorldDataName, SIM_Box3DWorld, 0);
    return w;
}

void
SIM_SolverBox3D::createFreshWorld(SIM_Box3DWorld &worldData, const char *reason)
{
    (void)reason;       // only used by the debug log
    B3D_COUNT(rebuilds);
    B3D_LOG("REBUILD #%d: %s", B3D_COUNTER(rebuilds), reason);

    // Fresh state; the previous one (if any) is left alone for whichever
    // timestep data still points at it. Its mirror is empty, so
    // syncFromGeometry() adds every point as a new body.
    worldData.createWorld(currentWorldSettings());
}

void
SIM_SolverBox3D::syncFromGeometry(SIM_Box3DWorld &worldData, SIM_ObjectArray &objects,
                                  std::vector<char> &objectHasDynamic)
{
    Box3DSharedState *st = worldData.state();

    // TODO: the engine says objects in `newobjects` should not be solved on the
    // step they appear in, so they keep their initial conditions. A rigid body
    // world has no per-body "skip this step"; decide how to treat them (e.g.
    // step them as kinematic for one step).

    SyncCounts                   counts;
    objectHasDynamic.assign(size_t(objects.entries()), 0);

    ++st->syncStamp;            // entries synced below carry this stamp
    st->stampedCount = 0;
    st->pointWrites.clear();    // filled by the scans below, used after the step
    st->deletions.clear();
    for (exint i = 0; i < objects.entries(); ++i)
        objectHasDynamic[size_t(i)] = syncObject(*st, *objects(i), counts) ? 1 : 0;
    st->world.finishFilterProfiles();       // collision filter answers for any new profiles, before the step

    // Bodies whose point (or whole object) is gone: the entries nobody stamped.
    // When every entry was stamped there is nothing to scan for.
    int removed = 0;
    if (st->stampedCount != st->mirror.size())
    {
        for (auto it = st->mirror.begin(); it != st->mirror.end();)
        {
            if (!st->isLive(it->second))
            {
                st->world.destroyBody(it->second.handle);
                it = st->mirror.erase(it);
                ++removed;
            }
            else
                ++it;
        }
        // The slots cache pointers to entries; some of them may just have died.
        st->slots.clear();
    }

    // Objects this sync did not see (gone, or without Geometry) lose their records: if one
    // came back it must scan, because its bodies were just removed above.
    for (auto it = st->objectSync.begin(); it != st->objectSync.end();)
        it = (it->second.visited == st->syncStamp) ? std::next(it) : st->objectSync.erase(it);

    // Hulls built for bodies that were then removed (or never created) are
    // no longer needed; hulls still used by a body stay cached for the next
    // body that instances the same geometry.
    if (counts.added || counts.failed || counts.rebuilt || removed)
        st->world.pruneHulls();     // hulls only become unused when a body is added (a failed one), rebuilt or removed

    // Problems the user should see on the node, not only in the debug log.
    // One warning per kind per sync, with a count. Fallback shapes are only
    // reported when a body is created, so they do not repeat every frame.
    auto warn = [&](const std::string &msg) {
        addError(objects(0), SIM_MESSAGE, msg.c_str(), UT_ERROR_WARNING);
    };
    for (const auto &fb : counts.fallbacks)
        warn("Box3D: " + std::to_string(fb.second) + " point(s): " + fb.first);
    if (counts.failed)
        warn("Box3D: " + std::to_string(counts.failed) + " point(s) could not be turned into a body and are not simulated");
    if (counts.duplicates)
        warn("Box3D: " + std::to_string(counts.duplicates) + " point(s) share an `id` with another point; only the first of each is simulated");
    if (counts.badP)
        warn("Box3D: " + std::to_string(counts.badP) + " point(s) have a non-finite P and are skipped");
    if (counts.dead && !st->deletions.empty())
        B3D_LOG("  sync: %d dead point(s) removed", counts.dead);

    if (counts.skipped)
        B3D_LOG_VERBOSE("  sync: %d of %d object(s) unchanged, scan skipped", counts.skipped, int(objects.entries()));
    // Adding / removing bodies is an event worth a line; edits, retypes and the problem
    // counts can repeat every step, so those lines need BOX3DDOP_VERBOSE=1.
    if (counts.added || counts.edited || counts.retyped || counts.failed ||
        counts.duplicates || counts.badP || counts.rebuilt || counts.updated || removed)
    {
        const bool event = counts.added || removed || counts.rebuilt;
        (void)event;                // only used by the debug log
        B3D_LOG_IF(event, "  sync: +%d added, -%d removed, %d edited externally, %d type changes, "
                "%d shapes rebuilt, %d settings changed, "
                "%d failed, %d duplicate ids, %d non-finite P  (bodies now %d)",
                counts.added, removed, counts.edited, counts.retyped, counts.rebuilt, counts.updated,
                counts.failed, counts.duplicates, counts.badP, st->world.bodyCount());
    }
}

void
SIM_SolverBox3D::stepWorld(SIM_Box3DWorld &worldData, float timestep)
{
    worldData.state()->world.step(timestep);
}

void
SIM_SolverBox3D::writeResultsBack(SIM_Box3DWorld &worldData, SIM_ObjectArray &objects,
                                  const std::vector<char> & /*objectHasDynamic*/)
{
    Box3DSharedState *st = worldData.state();
    b3dop::Box3DWorld &world = st->world;

    // ---- pass 0 (serial): collect the bodies that moved ----------------------
    // Box3D's body move events are one contiguous array holding every awake body
    // and its final transform; sleeping bodies are absent, and their points already
    // hold the values written when they fell asleep (that last event is included).
    // Each body's user data is its mirror entry, which knows its point. The events
    // are cleared at the start of every step, so after a step that did not run this
    // is empty.
    const int moveCount = world.getMoveEvents(st->moves);     // one call, scratch reused
    if (moveCount == 0)
        return;

    std::vector<Box3DWriteItem> &items = st->writeItems;       // reused between steps
    items.clear();
    items.reserve(size_t(moveCount));
    std::vector<Box3DPackedWrite> &packedWrites = st->packedWrites;     // side data, only packed points
    packedWrites.clear();
    int missing = 0;
    // "Seen by this step's sync" is per object now (a skipped object's entries keep the
    // stamp of its last scan). Moves of one object tend to be contiguous, so the record
    // lookup is cached for the last object id.
    int                    lastObj = -1;
    const Box3DObjectSync *lastOs  = nullptr;
    for (const b3dop::BodyMove &mv : st->moves)
    {
        if (!mv.userData)
            continue;
        DopBodyState *entry = static_cast<DopBodyState *>(mv.userData);
        if (!entry->dynamic)
            continue;                       // static / kinematic: nothing to write
        if (entry->objectId != lastObj || !lastOs)
        {
            lastObj = entry->objectId;
            const auto f = st->objectSync.find(lastObj);
            lastOs = (f != st->objectSync.end()) ? &f->second : nullptr;
        }
        if (!lastOs || lastOs->visited != st->syncStamp || entry->stamp != lastOs->liveStamp)
            continue;                       // not seen by this step's sync
        if (!entry->handle)
        {
            ++missing;                      // leave the point alone; next sync re-snapshots it
            continue;
        }
        items.emplace_back();
        Box3DWriteItem &w = items.back();
        w.entry    = entry;
        w.objectId = entry->objectId;
        if (entry->packed)
        {
            w.packedIdx = int(packedWrites.size());
            packedWrites.emplace_back();
        }
        for (int i = 0; i < 3; ++i) w.s.pos[i] = mv.pos[i];
        for (int i = 0; i < 4; ++i) w.s.rot[i] = mv.rot[i];
        // bullet_sleeping follows Box3D: a body's last move event says it fell asleep, and a
        // body that moves again is awake. Only changes are written.
        if (mv.fellAsleep != entry->sleeping)
        {
            w.sleepState    = mv.fellAsleep ? 1 : 0;
            entry->sleeping = mv.fellAsleep;
        }
    }
    if (items.empty())
        return;

    // Move events carry no velocity, and Box3D has no bulk getter: two API calls per body.
    // The step is over, and Box3D allows read-only calls from several threads, so the
    // fetch is threaded when there are many bodies. Each task writes only its own item.
    parallelForEach(items.size(), [&](size_t i) {
        Box3DWriteItem &w = items[i];
        world.getVelocityByHandle(w.entry->handle, w.s.lin, w.s.ang);
    }, kMinParallelWrite);

    // One run of items per object, so each object's geometry is locked once. A
    // single object (the usual case) needs no sort.
    bool oneObject = true;
    for (const Box3DWriteItem &w : items)
        if (w.objectId != items.front().objectId) { oneObject = false; break; }
    const auto byObject = [](const Box3DWriteItem &x, const Box3DWriteItem &y) { return x.objectId < y.objectId; };
    if (!oneObject && !std::is_sorted(items.begin(), items.end(), byObject))    // sorting allocates; often not needed
        std::stable_sort(items.begin(), items.end(), byObject);

    // `objects` is sorted by object id, so an object is found by binary search; no map to
    // build every step. (A linear scan is the fallback should that ever not hold.)
    auto findObject = [&](int id) -> SIM_Object * {
        exint lo = 0, hi = objects.entries();
        while (lo < hi)
        {
            const exint mid = lo + (hi - lo) / 2;
            const int   mid_id = objects(mid)->getObjectId();
            if (mid_id == id) return objects(mid);
            if (mid_id < id) lo = mid + 1; else hi = mid;
        }
        for (exint i = 0; i < objects.entries(); ++i)
            if (objects(i)->getObjectId() == id)
                return objects(i);
        return nullptr;
    };

    std::vector<SIM_Object *> refresh;      // objects whose geometry we wrote (see the end)
    for (size_t runBegin = 0; runBegin < items.size();)
    {
        size_t runEnd = runBegin;
        while (runEnd < items.size() && items[runEnd].objectId == items[runBegin].objectId)
            ++runEnd;
        const size_t begin = runBegin, end = runEnd;
        runBegin = runEnd;

        SIM_Object *obj = findObject(items[begin].objectId);
        if (!obj)
            continue;

        SIM_GeometryCopy *geo = getOrMakeWritableGeometry(*obj);
        if (!geo)
        {
            B3D_LOG_VERBOSE("  writeBack: object id=%d Geometry is not writable and could not be converted",
                    obj->getObjectId());
            addError(obj, SIM_MESSAGE,
                     "Box3D: this object's Geometry is not writable and could not be converted to a SIM_GeometryCopy; results are not written back",
                     UT_ERROR_WARNING);
            continue;
        }

        SIM_GeometryAutoWriteLock lock(geo);
        GU_Detail *gdp = gdpPtr(lock.getGdp());
        if (!gdp)
            continue;
        refresh.push_back(obj);     // fingerprint is re-recorded after the lock is released (below)

        const bool createdOrient = ensureResultAttribs(gdp);

        const PointAttrs a(gdp);          // bound AFTER the attributes exist (only the sync-critical ones are needed here)
        const bool anyPrims = gdp->getNumPrimitives() > 0;
        GA_RWHandleV3 vOut(gdp, GA_ATTRIB_POINT, "v");
        GA_RWHandleV3 wOut(gdp, GA_ATTRIB_POINT, "w");
        GA_RWHandleQ  qOut(gdp, GA_ATTRIB_POINT, "orient");

        if (createdOrient && qOut.isValid())
        {
            // A new quaternion attribute defaults to 0,0,0,0, which is not a
            // rotation. Start every point (static ones too) at identity.
            for (GA_Iterator it(gdp->getPointRange()); !it.atEnd(); ++it)
                qOut.set(*it, UT_QuaternionF(0.0f, 0.0f, 0.0f, 1.0f));
        }

        // ---- pass 1 (serial): find each body's point in the geometry --------
        resolvePointOffsets(gdp, a, items, begin, end);
        int unresolved = 0;
        for (size_t k = begin; k < end; ++k)
            if (items[k].off == GA_INVALID_OFFSET)
                ++unresolved;
        missing += unresolved;

        // ---- pass 2 (parallel): write attributes, one task per point ---------
        // Every item is a distinct body, so no two tasks write the same point or
        // the same mirror entry. Pages must be hardened first, or the writes would
        // race on copy-on-write pages that are still shared.
        GA_RWHandleV3 pOut(gdp, GA_ATTRIB_POINT, "P");
        GA_RWHandleI  sleepOut(gdp, GA_ATTRIB_POINT, "bullet_sleeping");      // kept up to date if it exists
        bool anySleepWrite = false;
        if (sleepOut.isValid())
            for (size_t k = begin; k < end && !anySleepWrite; ++k)
                anySleepWrite = items[k].sleepState >= 0 && items[k].off != GA_INVALID_OFFSET;
#ifdef BOX3DDOP_DEBUG
        const bool verifyReadback = B3D_ENV_INT("BOX3DDOP_VERIFY_READBACK", 0) != 0;
#endif
        // Copy-on-write pages that are still shared would race, so a pass that really runs in
        // parallel hardens first. Only the offset ranges the awake bodies write are hardened
        // (hardenWrittenPages), not every page of four attributes: on a mostly sleeping scene
        // that is a small part of them. A serial pass does not need it (set() unshares the
        // pages it touches).
        if (end - begin >= kMinParallelWrite)
            hardenWrittenPages(gdp, items, begin, end,
                               { pOut.isValid() ? pOut.getAttribute() : nullptr,
                                 qOut.isValid() ? qOut.getAttribute() : nullptr,
                                 vOut.isValid() ? vOut.getAttribute() : nullptr,
                                 wOut.isValid() ? wOut.getAttribute() : nullptr,
                                 anySleepWrite ? sleepOut.getAttribute() : nullptr });

        parallelForEach(end - begin, [&](size_t j) {
            Box3DWriteItem      &w   = items[begin + j];
            if (w.off == GA_INVALID_OFFSET)
                return;
            const GA_Offset      off = w.off;
            const Box3DBodyPose &s   = w.s;

            if (pOut.isValid()) pOut.set(off, UT_Vector3(float(s.pos[0]), float(s.pos[1]), float(s.pos[2])));
            if (qOut.isValid()) qOut.set(off, UT_QuaternionF(s.rot[0], s.rot[1], s.rot[2], s.rot[3]));
            if (vOut.isValid()) vOut.set(off, UT_Vector3(s.lin[0], s.lin[1], s.lin[2]));
            if (wOut.isValid()) wOut.set(off, UT_Vector3(s.ang[0], s.ang[1], s.ang[2]));
            if (anySleepWrite && w.sleepState >= 0) sleepOut.set(off, int(w.sleepState));

            // A packed primitive's rotation is NOT the point's `orient`: it is
            // the primitive's own transform. Work out the new matrix here (pure
            // math), but do not write it: that is pass 3. Keep its scale, replace the
            // rotation. Packed instances are assumed to keep their topology, so the
            // primitive offset and 3x3 the last sync stored in the entry are used as they
            // are (no vertex / primitive lookups here); pass 3 still checks the primitive.
            Box3DBodyMirror  &me     = *w.entry;
            Box3DPackedWrite *pw     = w.packedIdx >= 0 ? &packedWrites[size_t(w.packedIdx)] : nullptr;
            const bool        packed = anyPrims && me.packed && pw;
            float             packedRot[4] = { 0, 0, 0, 1 };
            if (packed)
            {
                b3dop::composeLocal3(s.rot, me.m3, pw->newM3);
                pw->primOff = GA_Offset(me.primOff);
                pw->has     = true;
                // What the primitive will hold, without reading it back: the rotation of
                // the matrix pass 3 writes. The entry's 3x3 follows it, because a skipped
                // sync no longer refreshes it every step.
                double sc[3];
                b3dop::matrix3ToQuatScale(pw->newM3, packedRot, sc);
                for (int i = 0; i < 9; ++i) me.m3[i] = pw->newM3[i];
            }

            // What the points now hold, without reading them back. This must equal what
            // readPointState would return, or next step's edit detection sees our own write
            // as an external edit. The pieces: P goes through float (storedPosition); v and w
            // are floats already; orient is stored as written and read through sanitizeQuat; a
            // packed point's rotation is the rotation of the matrix pass 3 writes (packedRot,
            // computed above), also through sanitizeQuat. A non-finite P is read as "absent",
            // in which case the mirror keeps what Box3D reported.
            Box3DBodyPose &mirror = me;
            double sp[3];
            if (b3dop::storedPosition(s.pos, sp))
            {
                for (int i = 0; i < 3; ++i) mirror.pos[i] = sp[i];
                // An output attribute that could not be bound was never written, and the read
                // path then returns its default (identity / zero), so the prediction does too.
                const float identity[4] = { 0, 0, 0, 1 };
                const float *rot = packed ? packedRot : (qOut.isValid() ? s.rot : identity);
                for (int i = 0; i < 4; ++i) mirror.rot[i] = rot[i];
                b3dop::sanitizeQuat(mirror.rot);
                for (int i = 0; i < 3; ++i)
                {
                    mirror.lin[i] = vOut.isValid() ? s.lin[i] : 0.0f;
                    mirror.ang[i] = wOut.isValid() ? s.ang[i] : 0.0f;
                }
            }
            else
                mirror = s;

#ifdef BOX3DDOP_DEBUG
            if (verifyReadback)
            {
                // Debug check (BOX3DDOP_VERIFY_READBACK=1): the old way, compared bit for bit.
                Box3DBodyPose stored;
                PackedInfo pk;
                if (packed)
                {
                    pk.prim = static_cast<const GU_PrimPacked *>(gdp->getPrimitive(pw->primOff));
                    for (int i = 0; i < 4; ++i) pk.rot[i] = packedRot[i];
                }
                if (readPointState(gdp, a, off, pk, stored) &&
                    (std::memcmp(stored.pos, mirror.pos, sizeof(stored.pos)) != 0 ||
                     std::memcmp(stored.rot, mirror.rot, sizeof(stored.rot)) != 0 ||
                     std::memcmp(stored.lin, mirror.lin, sizeof(stored.lin)) != 0 ||
                     std::memcmp(stored.ang, mirror.ang, sizeof(stored.ang)) != 0))
                {
                    B3D_LOG("  READBACK MISMATCH object=%d point=%lld: predicted pos=(%.9g %.9g %.9g) rot=(%.9g %.9g %.9g %.9g) "
                            "read pos=(%.9g %.9g %.9g) rot=(%.9g %.9g %.9g %.9g)",
                            w.objectId, (long long)w.entry->pointId,
                            mirror.pos[0], mirror.pos[1], mirror.pos[2],
                            mirror.rot[0], mirror.rot[1], mirror.rot[2], mirror.rot[3],
                            stored.pos[0], stored.pos[1], stored.pos[2],
                            stored.rot[0], stored.rot[1], stored.rot[2], stored.rot[3]);
                }
            }
#endif
        }, kMinParallelWrite);

        for (GA_Attribute *attr : { pOut.isValid() ? pOut.getAttribute() : nullptr,
                                    qOut.isValid() ? qOut.getAttribute() : nullptr,
                                    vOut.isValid() ? vOut.getAttribute() : nullptr,
                                    wOut.isValid() ? wOut.getAttribute() : nullptr,
                                    anySleepWrite ? sleepOut.getAttribute() : nullptr })
            if (attr)
                attr->bumpDataId();

        // ---- pass 3 (serial): packed primitive rotation ----------------------
        // setLocalTransform + transformDirty stay serial: the HDK pages do not
        // say whether they touch shared state.
        bool wrotePacked = false;
        for (size_t k = begin; k < end; ++k)
        {
            const Box3DWriteItem &w = items[k];
            if (w.packedIdx < 0 || !packedWrites[size_t(w.packedIdx)].has)
                continue;
            const Box3DPackedWrite &pw = packedWrites[size_t(w.packedIdx)];
            const GA_Primitive *raw = (pw.primOff >= 0 && pw.primOff < gdp->getNumPrimitiveOffsets())
                                          ? gdp->getPrimitive(pw.primOff) : nullptr;
            if (!raw || !GU_PrimPacked::isPackedPrimitive(*raw))
            {
                ++missing;          // topology changed without a new instance id: skip, do not crash
                continue;
            }
            setLocal3(*static_cast<GU_PrimPacked *>(gdp->getPrimitive(pw.primOff)), pw.newM3);
            wrotePacked = true;
        }
        // The HDK documents GA_PrimitiveList::bumpDataId() as the way to mark primitives or
        // their intrinsic data dirty; setLocalTransform does not do it. Without it, anything
        // caching on the primitive list's data id would not see the new rotations. The
        // fingerprint re-recorded below picks up the new id, so this does not defeat the skip.
        if (wrotePacked)
            gdp->getPrimitiveList().bumpDataId();
    }

    // The ids changed because of OUR write (and releasing a write lock may bump them again
    // too), so record them now that every lock is released, from a read of the geometry as
    // the next step will see it. Otherwise our own write would defeat the sync skip.
    for (SIM_Object *obj : refresh)
        recordFingerprintAfterWrite(*st, *obj);

    if (missing)
    {
        B3D_LOG_VERBOSE("  writeBack: %d moved bodies had no readable state or no point to write to", missing);
        addError(objects(0), SIM_MESSAGE,
                 ("Box3D: " + std::to_string(missing) + " body(ies) could not be matched to a point; their results were not written back").c_str(),
                 UT_ERROR_WARNING);
    }
}

void
SIM_SolverBox3D::publishEvents(SIM_Box3DWorld & /*worldData*/, SIM_ObjectArray & /*objects*/)
{
    // TODO: begin/end touch, hit, sensor, and sleep events as DOP data.
}
