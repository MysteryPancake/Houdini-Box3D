// SIM_SolverBox3D.cpp
#include "SIM_SolverBox3D.h"
#include "SIM_Box3DWorld.h"
#include "SIM_Box3DData.h"
#include "Box3DDebug.h"

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
    GA_ROHandleI  isbullet;        // int    nonzero = continuous collision for fast bodies
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

    // Binds only what every step needs for existing bodies: id, active, v, w and
    // orient. Everything else is for building a body, so it is bound by
    // bindBuildAttrs() the first time a body is created (a step that only syncs
    // and writes back saves about 17 attribute lookups per object).
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
        isbullet       = GA_ROHandleI(gdp, GA_ATTRIB_POINT, "isbullet");
        pscale         = GA_ROHandleF(gdp, GA_ATTRIB_POINT, "pscale");
        scale          = GA_ROHandleV3(gdp, GA_ATTRIB_POINT, "scale");
        georep         = GA_ROHandleS(gdp, GA_ATTRIB_POINT, "bullet_georep");
        autofit        = GA_ROHandleI(gdp, GA_ATTRIB_POINT, "bullet_autofit");
        primS          = GA_ROHandleV3(gdp, GA_ATTRIB_POINT, "bullet_primS");
        primR          = GA_ROHandleV3(gdp, GA_ATTRIB_POINT, "bullet_primR");
        primT          = GA_ROHandleV3(gdp, GA_ATTRIB_POINT, "bullet_primT");
        radius         = GA_ROHandleF(gdp, GA_ATTRIB_POINT, "bullet_radius");
        length         = GA_ROHandleF(gdp, GA_ATTRIB_POINT, "bullet_length");
    }
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
Box3DSyncFingerprint
makeFingerprint(const GU_Detail *gdp, const PointAttrs &a, int nodeType)
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

// Create the body + shapes for one point and apply its initial velocity.
bool
addPointBody(b3dop::Box3DWorld &world, ShapeSources &src, const b3dop::BodySettings &node,
             const PointAttrs &a, GA_Offset off, int64_t bodyId, uint64_t key,
             const PackedInfo &pk, const Box3DBodyPose &st, bool active,
             std::string *fallbackNote = nullptr)
{
    b3dop::BodySettings b = node;
    b.type = active ? node.type : b3dop::BodySettings::Type::Static;     // `active` = 0 makes a point static
    // Per-point overrides of the node's values. Box3D asserts on negative
    // friction / restitution / damping, so clamp attribute values at 0 and
    // ignore non-finite ones.
    auto nonNegative = [](float v, float fallback) {
        return std::isfinite(v) ? std::max(v, 0.0f) : fallback;
    };
    if (a.friction.isValid())       b.friction       = nonNegative(a.friction.get(off),       b.friction);
    if (a.bounce.isValid())         b.restitution    = nonNegative(a.bounce.get(off),         b.restitution);
    if (a.lineardamping.isValid())  b.linearDamping  = nonNegative(a.lineardamping.get(off),  b.linearDamping);
    if (a.angulardamping.isValid()) b.angularDamping = nonNegative(a.angulardamping.get(off), b.angularDamping);
    if (a.gravityscale.isValid())
    {
        const float g = a.gravityscale.get(off);
        if (std::isfinite(g)) b.gravityScale = g;      // may be negative (floats upward)
    }
    if (a.enablesleep.isValid())    b.enableSleep    = a.enablesleep.get(off) != 0;
    if (a.isbullet.isValid())       b.isBullet       = a.isbullet.get(off) != 0;

    // Shape: `bullet_georep` string attribute, else the node's default.
    b3dop::ShapeType type = node.defaultShape;
    if (a.georep.isValid())
        b3dop::parseShapeType(a.georep.get(off), type);  // unknown -> keep default

    // `none`: a static body with no collision shape (like Bullet, where it still
    // exists for constraints). The one case where a body is allowed to be shapeless.
    if (type == b3dop::ShapeType::None)
    {
        b.type = b3dop::BodySettings::Type::Static;
        return world.addBody(key, b, st.pos, st.rot);
    }

    // pscale / scale size plain points. A packed point's size is its
    // primitive's transform, so they only matter as the fallback box there.
    const float ps = a.pscale.isValid() ? float(a.pscale.get(off)) : 1.0f;
    float sc[3] = { 1.0f, 1.0f, 1.0f };
    if (a.scale.isValid())
    {
        const UT_Vector3 s = a.scale.get(off);
        sc[0] = s.x();  sc[1] = s.y();  sc[2] = s.z();
    }

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

    const BuiltShapes shapes = buildShapes(world, src, type, node.hullPerConnectedSet, ps, sc, pk,
                                           autofit, ov);
    (void)bodyId;                   // only used by the debug log
    if (shapes.note)
    {
        B3D_LOG("  body id=%lld: %s", (long long)bodyId, shapes.note);
        if (fallbackNote)
            *fallbackNote = shapes.note;    // surfaced to the user as one warning per kind
    }

    // mass -> density (Box3D computes mass from density * volume; for a
    // compound the volume is the sum of the pieces). The node's `mass` is the
    // default for every body; the `mass` point attribute overrides it.
    float mass = node.mass;
    if (a.mass.isValid())
    {
        const float m = float(a.mass.get(off));
        if (m > 0.0f && std::isfinite(m))
            mass = m;
    }
    if (mass > 0.0f && std::isfinite(mass) && shapes.totalVolume > 0.0f)
        b.density = mass / shapes.totalVolume;

    if (!world.addBody(key, b, st.pos, st.rot))
        return false;

    int added = 0;
    for (const Piece &pc : shapes.pieces)
        added += (pc.isHull ? world.addHullShape(key, pc.hull) : world.addShape(key, pc.spec)) ? 1 : 0;
    if (added == 0)
    {
        world.removeBody(key);      // never leave a shapeless body behind
        return false;
    }
    world.setVelocity(key, st.lin, st.ang);     // no-op for static bodies
    return true;
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
        B3D_LOG("  object id=%d has no Geometry data, skipped", obj.getObjectId());
        return false;
    }
    SIM_GeometryAutoReadLock lock(geo);
    const GU_Detail *gdp = gdpPtr(lock.getGdp());
    if (!gdp)
        return false;

    PointAttrs a(gdp);          // the build attributes are bound on the first new body
    bool       buildAttrsBound = false;
    const bool anyPrims = gdp->getNumPrimitives() > 0;     // no primitives -> no packed points to look for
    const int  objectId = obj.getObjectId();
    // Default settings for all points of an object (the Box3D Data DOP node), or
    // built-in defaults if the object has none.
    // The full settings (a dozen option reads) are only needed by a scan, so they are built
    // after the skip check; the skip check itself only needs the body type.
    const SIM_Box3DData *dataNode = SIM_DATA_GETCONST(obj, kDataName, SIM_Box3DData);
    int nodeType = int(b3dop::BodySettings().type);
    if (dataNode)
    {
        const int bt = int(dataNode->getBodyType());       // same mapping as toBodySettings()
        nodeType = int(bt == 0 ? b3dop::BodySettings::Type::Static
                     : bt == 1 ? b3dop::BodySettings::Type::Kinematic
                               : b3dop::BodySettings::Type::Dynamic);
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
    // rotation / scale is NOT covered by any marker: changing it without touching P, `id`
    // or another marked attribute is not noticed on a skipped step.
    Box3DObjectSync &os = st.objectSync[objectId];
    os.visited = st.syncStamp;
    const Box3DSyncFingerprint fpNow = makeFingerprint(gdp, a, nodeType);
    if (os.canSkip && fpNow == os.fp)
    {
        ++c.skipped;
        st.stampedCount += os.entryCount;
        return os.hasDynamic;
    }
#ifdef BOX3DDOP_DEBUG
    if (os.canSkip)
        B3D_LOG("  object id=%d: full scan, changed: %s", objectId,
                describeFingerprintDifference(os.fp, fpNow).c_str());
#endif
    const b3dop::BodySettings node = dataNode ? dataNode->toBodySettings() : b3dop::BodySettings();
    os.canSkip   = false;
    os.liveStamp = st.syncStamp;        // entries stamped below are current under this
    size_t stamped = 0;                 // entries stamped by this scan
    const int problems0 = c.failed + c.duplicates + c.badP;

    ShapeSources sources;       // packed geometry, collected lazily per distinct source

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
        if (!entry)
        {
            auto mit = st.mirror.find(key);
            if (mit != st.mirror.end())
                entry = &mit->second;
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

        const bool active = s.active;
        const b3dop::BodySettings::Type wantType =
            active ? node.type : b3dop::BodySettings::Type::Static;

        if (!entry)
        {
            if (!buildAttrsBound)
            {
                a.bindBuildAttrs(gdp);
                buildAttrsBound = true;
            }
            PackedInfo pk = anyPrims ? getPackedInfo(gdp, off) : PackedInfo();    // only new bodies need it again
            readPackedPivot(pk);        // the hull is built from it
            std::string fallbackNote;
            if (!addPointBody(world, sources, node, a, off, bid, key, pk, cur, active, &fallbackNote))
            {
                ++c.failed;
                continue;
            }
            if (!fallbackNote.empty())
                ++c.fallbacks[fallbackNote];
            DopBodyState &m = st.mirror[key];       // node address is stable from here on
            static_cast<Box3DBodyPose &>(m) = cur;
            m.dynamic  = (wantType == b3dop::BodySettings::Type::Dynamic);
            m.type     = wantType;
            m.handle   = world.bodyHandle(key);
            m.pointOff = off;
            m.pointId  = bid;
            m.objectId = objectId;
            m.stamp    = st.syncStamp;
            storePacked(m, s);
            ++st.stampedCount;
            ++stamped;
            world.releaseBuildState(key);           // shapes are all added
            world.setBodyUserData(m.handle, &m);    // lets write-back map a move event to this entry
            s.entry = &m;
            ++c.added;
            hasDynamic = hasDynamic || m.dynamic;
            continue;
        }

        // Existing body.
        entry->pointOff = off;
        entry->pointId  = bid;
        entry->objectId = objectId;
        entry->stamp    = st.syncStamp;
        storePacked(*entry, s);
        ++st.stampedCount;
        ++stamped;

        // Did `active` flip?
        if (entry->type != wantType)
        {
            world.setBodyType(key, wantType);
            entry->type    = wantType;
            entry->dynamic = (wantType == b3dop::BodySettings::Type::Dynamic);
            ++c.retyped;
        }

        // Did something else edit P / orient / v / w since we wrote it?
        if (describeStateDifference(cur, *entry))
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
    os.entryCount = stamped;
    os.hasDynamic = hasDynamic;
    os.fp         = fpNow;
    os.canSkip    = (c.failed + c.duplicates + c.badP) == problems0;
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

// Sets each item's point offset in the geometry that is about to be written. The
// sync pass stored an offset in every mirror entry, and the writable geometry is
// normally an exact copy of what was read, so the stored offset still names the same
// point. That is checked (offset in range, and the point's id still matches) because
// a copy that renumbered points would otherwise write results to the wrong ones. A
// stale offset is looked up again by the body's point id. An item whose point cannot
// be found gets GA_INVALID_OFFSET and is skipped.
void
resolvePointOffsets(const GU_Detail *gdp, const PointAttrs &a,
                    std::vector<Box3DWriteItem> &items, size_t begin, size_t end)
{
    std::unordered_map<int64_t, GA_Offset> byId;     // built only if an `id` lookup is needed
    bool builtById = false;

    for (size_t k = begin; k < end; ++k)
    {
        Box3DWriteItem        &w = items[k];
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

    static PRM_Template theTemplates[] = {
        PRM_Template(PRM_FLT_J,  3, &theGravityName,  theGravityDefault),
        PRM_Template(PRM_INT_J,  1, &theSubStepsName, &theSubStepsDefault, 0, &theSubStepsRange),
        PRM_Template(PRM_TOGGLE, 1, &theAutoThreadsName, &theAutoThreadsDefault),
        PRM_Template(PRM_INT_J,  1, &theThreadsName,  &theThreadsDefault, 0, &theThreadsRange,
                     0, 0, 1,
                     "Number of Box3D worker threads. Only used when Use All Threads is off.",
                     &theThreadsDisable),
        PRM_Template(PRM_TOGGLE, 1, &theSleepName,    &theSleepDefault),
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

    std::vector<char> objectHasDynamic;
    syncFromGeometry(*worldData, objects, objectHasDynamic);
    stepWorld(*worldData, static_cast<float>(dt));
    writeResultsBack(*worldData, objects, objectHasDynamic);

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
    for (exint i = 0; i < objects.entries(); ++i)
        objectHasDynamic[size_t(i)] = syncObject(*st, *objects(i), counts) ? 1 : 0;

    // Bodies whose point (or whole object) is gone: the entries nobody stamped.
    // When every entry was stamped there is nothing to scan for.
    int removed = 0;
    if (st->stampedCount != st->mirror.size())
    {
        for (auto it = st->mirror.begin(); it != st->mirror.end();)
        {
            if (!st->isLive(it->second))
            {
                st->world.removeBody(it->first);
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
    if (counts.added || counts.failed || removed)
        st->world.pruneHulls();     // hulls only become unused when a body is added (a failed one) or removed

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

    if (counts.skipped)
        B3D_LOG("  sync: %d of %d object(s) unchanged, scan skipped", counts.skipped, int(objects.entries()));
    if (counts.added || counts.edited || counts.retyped || counts.failed ||
        counts.duplicates || counts.badP || removed)
    {
        B3D_LOG("  sync: +%d added, -%d removed, %d edited externally, %d type changes, "
                "%d failed, %d duplicate ids, %d non-finite P  (bodies now %d)",
                counts.added, removed, counts.edited, counts.retyped, counts.failed,
                counts.duplicates, counts.badP, st->world.bodyCount());
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
        for (int i = 0; i < 3; ++i) w.s.pos[i] = mv.pos[i];
        for (int i = 0; i < 4; ++i) w.s.rot[i] = mv.rot[i];
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
            B3D_LOG("  writeBack: object id=%d Geometry is not writable and could not be converted",
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
#ifdef BOX3DDOP_DEBUG
        const bool verifyReadback = B3D_ENV_INT("BOX3DDOP_VERIFY_READBACK", 0) != 0;
#endif
        // Hardening copies every shared page of the attribute, however few points are
        // written. A serial pass does not need it (set() unshares the pages it touches), so
        // only a pass that will really run in parallel pays for it.
        if (end - begin >= kMinParallelWrite)
            for (GA_Attribute *attr : { pOut.isValid() ? pOut.getAttribute() : nullptr,
                                        qOut.isValid() ? qOut.getAttribute() : nullptr,
                                        vOut.isValid() ? vOut.getAttribute() : nullptr,
                                        wOut.isValid() ? wOut.getAttribute() : nullptr })
                if (attr)
                    attr->hardenAllPages();

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

            // A packed primitive's rotation is NOT the point's `orient`: it is
            // the primitive's own transform. Work out the new matrix here (pure
            // math), but do not write it: that is pass 3. Keep its scale, replace the
            // rotation. Packed instances are assumed to keep their topology, so the
            // primitive offset and 3x3 the last sync stored in the entry are used as they
            // are (no vertex / primitive lookups here); pass 3 still checks the primitive.
            Box3DBodyMirror &me     = *w.entry;
            const bool       packed = anyPrims && me.packed;
            float            packedRot[4] = { 0, 0, 0, 1 };
            if (packed)
            {
                b3dop::composeLocal3(s.rot, me.m3, w.newM3);
                w.primOff = GA_Offset(me.primOff);
                w.hasM3   = true;
                // What the primitive will hold, without reading it back: the rotation of
                // the matrix pass 3 writes. The entry's 3x3 follows it, because a skipped
                // sync no longer refreshes it every step.
                double sc[3];
                b3dop::matrix3ToQuatScale(w.newM3, packedRot, sc);
                for (int i = 0; i < 9; ++i) me.m3[i] = w.newM3[i];
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
                    pk.prim = static_cast<const GU_PrimPacked *>(gdp->getPrimitive(w.primOff));
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
                                    wOut.isValid() ? wOut.getAttribute() : nullptr })
            if (attr)
                attr->bumpDataId();

        // ---- pass 3 (serial): packed primitive rotation ----------------------
        // setLocalTransform + transformDirty stay serial: the HDK pages do not
        // say whether they touch shared state.
        for (size_t k = begin; k < end; ++k)
        {
            const Box3DWriteItem &w = items[k];
            if (!w.hasM3)
                continue;
            const GA_Primitive *raw = (w.primOff >= 0 && w.primOff < gdp->getNumPrimitiveOffsets())
                                          ? gdp->getPrimitive(w.primOff) : nullptr;
            if (!raw || !GU_PrimPacked::isPackedPrimitive(*raw))
            {
                ++missing;          // topology changed without a new instance id: skip, do not crash
                continue;
            }
            setLocal3(*static_cast<GU_PrimPacked *>(gdp->getPrimitive(w.primOff)), w.newM3);
        }
    }

    // The ids changed because of OUR write (and releasing a write lock may bump them again
    // too), so record them now that every lock is released, from a read of the geometry as
    // the next step will see it. Otherwise our own write would defeat the sync skip.
    for (SIM_Object *obj : refresh)
    {
        const auto f = st->objectSync.find(obj->getObjectId());
        if (f == st->objectSync.end() || !f->second.canSkip)
            continue;
        const SIM_Geometry *geo = SIM_DATA_GETCONST(*obj, SIM_GEOMETRY_DATANAME, SIM_Geometry);
        if (!geo)
            continue;
        SIM_GeometryAutoReadLock rlock(geo);
        const GU_Detail *gdp = gdpPtr(rlock.getGdp());
        if (!gdp)
            continue;
        const PointAttrs a(gdp);
        f->second.fp = makeFingerprint(gdp, a, f->second.fp.nodeType);
    }

    if (missing)
    {
        B3D_LOG("  writeBack: %d moved bodies had no readable state or no point to write to", missing);
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
