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
#include <UT/UT_Interrupt.h>
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
#include <UT/UT_Thread.h>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// File scope (not the anonymous namespace): the header forward-declares it.
// Escape during one solve. Like SideFX's own SIM code (SIM_VolumetricConnectedComponentBuilder),
// no operation of our own: opInterrupt() under the one Houdini already runs while stepping, so
// its "Stepping simulation" message and progress bar stay as they are (no percent passed).
// Checked only from serial code, between whole points (never inside a Box3D call).
struct StepProgress
{
    UT_Interrupt *boss      = nullptr;
    bool          cancelled = false;        // latched: once true, every check says so

    bool check()
    {
        if (!cancelled && boss)
            cancelled = boss->opInterrupt() != 0;
        return cancelled;
    }
};

namespace {

// Data names of the world and of the per-object defaults (SIM_Box3DData).
const char *const kWorldDataName = "Box3DWorld";
const char *const kDataName      = "Box3DData";

using DopBodyState = Box3DBodyMirror;

// Packed 3x3 (row-vector, M = S * R) split into per-row scale and a unit quaternion with the
// HDK, so packed rotations follow Houdini's own quaternion convention. Shear and mirroring
// are dropped; false (identity, scale 1) if degenerate.
bool
splitPackedMatrix(const double m3[9], float q[4], double scale[3])
{
    q[0] = q[1] = q[2] = 0.0f;  q[3] = 1.0f;
    scale[0] = scale[1] = scale[2] = 1.0;
    UT_Matrix3D m(m3[0], m3[1], m3[2], m3[3], m3[4], m3[5], m3[6], m3[7], m3[8]);
    const double det = m.determinant();
    if (!std::isfinite(det) || std::fabs(det) < 1e-30)
        return false;
    if (det < 0.0)                                  // mirrored: flip the third row
        for (int c = 0; c < 3; ++c)
            m(2, c) = -m(2, c);
    UT_Vector3D s;
    m.extractScales(s);                             // m is left a pure rotation
    UT_QuaternionD rq;
    rq.updateFromRotationMatrix(m);
    rq.normalize();
    q[0] = float(rq.x());  q[1] = float(rq.y());  q[2] = float(rq.z());  q[3] = float(rq.w());
    scale[0] = std::fabs(s.x());  scale[1] = std::fabs(s.y());  scale[2] = std::fabs(s.z());
    return true;
}

// oldM3's per-row scale with q's rotation: a packed primitive's new local 3x3.
void
composePackedMatrix(const float q[4], const double oldM3[9], double out[9])
{
    float  oq[4];
    double sc[3];
    splitPackedMatrix(oldM3, oq, sc);               // sc stays 1 if oldM3 is degenerate
    UT_DMatrix3 r;
    UT_QuaternionD(q[0], q[1], q[2], q[3]).getRotationMatrix(r);
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
            out[row * 3 + col] = sc[row] * r(row, col);
}

// Point attributes the solver reads (Houdini RBD names). All optional: invalid = default.
struct PointAttrs
{
    GA_ROHandleI  id;          // else the point number
    GA_ROHandleI  active;      // 0 = inactive (static, or kinematic if animated)
    GA_ROHandleF  mass;
    GA_ROHandleF  friction;
    GA_ROHandleF  bounce;      // Box3D restitution
    GA_ROHandleF  lineardamping;
    GA_ROHandleF  angulardamping;
    GA_ROHandleF  gravityscale;
    GA_ROHandleI  enablesleep;     // creation only
    GA_ROHandleI  fastObject;      // fastobject
    GA_ROHandleF  sleepThreshold;  // sleepthreshold
    GA_ROHandleF  pscale;
    GA_ROHandleV3 scale;
    GA_ROHandleV3 v;
    GA_ROHandleV3 w;           // radians / second
    GA_ROHandleQ  orient;      // ignored for packed points (the primitive's transform is used)
    GA_ROHandleS  georep;      // bullet_georep (b3dop::parseShapeType)
    GA_ROHandleI  autofit;     // bullet_autofit
    GA_ROHandleV3 primS;       // bullet_prim* / radius / length: see b3dop::PrimOverrides
    GA_ROHandleV3 primR;
    GA_ROHandleV3 primT;
    GA_ROHandleF  radius;
    GA_ROHandleF  length;
    GA_ROHandleI  wantDeactivate;  // bullet_want_deactivate: 1 may sleep, 0 kept awake
    GA_ROHandleI  ignore;          // bullet_ignore
    GA_ROHandleI  deforming;       // rebuild the shape every step
    GA_ROHandleI  autofitValid;    // bullet_autofit_valid: 0 = rebuild once (written back as 1)
    GA_ROHandleS  collisionGroup;
    GA_ROHandleS  collisionIgnore;
    GA_ROHandleI  animated;
    GA_ROHandleF  density;
    GA_ROHandleI  computeMass;
    GA_ROHandleF  inertiaStiffness;    // inertialtensorstiffness
    GA_ROHandleF  minActivation;   // min_activation_impulse
    GA_ROHandleS  activationIgnore;
    GA_ROHandleI  dead;
    GA_ROHandleI  sleeping;        // bullet_sleeping: a user-written 0 wakes the body

    // The constructor binds what the skip check and write-back need; bindBuildAttrs() the rest,
    // once an object is actually scanned.
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

// Attributes whose data ids are the fingerprint's prop markers (propIds, in order). The shape
// attributes are left out on purpose: like Bullet, editing them alone changes nothing until
// bullet_autofit_valid = 0 or deforming = 1.
const char *const kPropAttribNames[Box3DSyncFingerprint::kNumPropIds] = {
    "mass", "friction", "bounce", "lineardamping", "angulardamping", "gravityscale",
    "bullet_want_deactivate", "bullet_ignore", "deforming", "bullet_autofit_valid",
    "pscale", "scale",      // a plain point's convex / concave fallback box
    "collisiongroup", "collisionignore", "animated", "density", "computemass",
    "inertialtensorstiffness", "sleepthreshold", "fastobject",
    "min_activation_impulse", "activationignore", "dead", "bullet_sleeping",
};

// ---------------------------------------------------------------------------
// Packed primitives
//
// A packed point: one point + one primitive. Position is the point's P (the pivot, the centre
// of mass); rotation and scale are the primitive's transform, not `orient`. The solver reads
// the rotation from it, writes it back to it, and builds hulls around the pivot.
// ---------------------------------------------------------------------------

struct PackedInfo
{
    const GU_PrimPacked *prim       = nullptr;       // null: a plain point
    GA_Offset            primOff    = GA_INVALID_OFFSET;
    double               P[3]       = { 0, 0, 0 };     // the pivot in world space
    double               pivot[3]   = { 0, 0, 0 };     // in the packed geometry's space; filled by readPackedPivot only
    double               m4[16]     = { 1, 0, 0, 0,  0, 1, 0, 0,  0, 0, 1, 0,  0, 0, 0, 1 };
    float                rot[4]     = { 0, 0, 0, 1 };  // rotation split out of m4's 3x3
    double               scale[3]   = { 1, 1, 1 };
};

// The primitive's full transform, row-major.
void
fetchFull4(const GU_PrimPacked &p, double out[16])
{
    UT_Matrix4D m;
    p.getFullTransform4(m);
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out[r * 4 + c] = m(r, c);
}

// Writes the packed primitive's local 3x3.
void
setLocal3(GU_PrimPacked &p, const double m[9])
{
    UT_Matrix3D mat;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            mat(r, c) = m[r * 3 + c];
    p.setLocalTransform(mat);
    // It caches bounds derived from the transform.
    p.transformDirty();
}

// Below these counts a pass runs serially (untuned starting points).
constexpr size_t kMinParallelPoints      = 2048;    // default
constexpr size_t kMinParallelReadPlain   = 8192;    // sync read, no primitives in the geometry
constexpr size_t kMinParallelReadPacked  = 2048;    // sync read, geometry has primitives
constexpr size_t kMinParallelWrite       = 1024;    // write-back

// f(i) for i in [0, n), on Houdini's TBB pool when n is large. f may only read shared data and
// write element i of its own output.
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

// The point's packed primitive, if any (prim == nullptr otherwise), without the pivot.
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
    splitPackedMatrix(m3, pk.rot, pk.scale);
    return pk;
}

// Fills pk.pivot (only needed to build shapes).
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

// One piece's points, raw xyz in their own space.
struct NamedPiece
{
    std::string                name;
    std::vector<float>         xyz;
    uint64_t                   hash = 0;   // of xyz
    std::unordered_set<GA_Offset> seen;    // while collecting: points already added
};

struct PieceGroup
{
    std::vector<NamedPiece>                    pieces;
    std::unordered_map<std::string, size_t>    byName;
    uint64_t                                   hash = 0;   // of all pieces: the merged hull's key
};

// Hash each piece and all of them together (once per shared source); drop the `seen` sets.
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

// Pieces by the primitive `name` attribute (unnamed prims share one; no prims: one piece).
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

// One piece per set of connected primitives (b3dop::connectedSets).
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

// A packed fragment shares the whole source detail but stands for the primitives in its range:
// those points, as one piece.
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

// Collision source geometry (what is packed in each point's primitive), cached for one
// syncObject() call.
struct ShapeSources
{
    // Per distinct source detail; the handle keeps the detail (the key) alive.
    struct InnerEntry
    {
        GU_ConstDetailHandle handle;
        PieceGroup           pieces;             // grouped by the `name` attribute
        PieceGroup           connected;          // one piece per set of connected primitives
        bool                 connectedBuilt = false;
    };
    std::unordered_map<const GU_Detail *, InnerEntry> inner;
    // Fragments: (shared source detail, first primitive offset) -> that fragment's piece.
    std::map<std::pair<const GU_Detail *, GA_Offset>, InnerEntry> fragments;

    // The primitive's pieces (by `name`, or connected sets), or null.
    const PieceGroup *innerPieces(const GU_PrimPacked &prim, bool connected)
    {
        // A packed fragment: read the WHOLE source detail (detailPtr), shared by every fragment
        // of it, and only this fragment's own primitives in it. Not getPackedDetail(): for a
        // fragment that is a different detail from the one getPrimitiveRange() indexes, so the
        // range's offsets picked the wrong primitives or none (none: the pscale * scale box).
        const GU_PackedFragment *frag = asFragment(prim);
        GU_ConstDetailHandle handle = frag ? frag->detailPtr() : prim.getPackedDetail();
        GU_DetailHandleAutoReadLock lock(handle);
        const GU_Detail *gdp = lock.getGdp();
        if (!gdp)
            return nullptr;

        InnerEntry              *entry = nullptr;
        if (frag)
        {
            // GU_PackedFragment does not expose its attribute value; its primitive range does
            // identify it (no two fragments of one source share a primitive).
            const GA_Range range = frag->getPrimitiveRange();
            const GA_Iterator first(range);
            const GA_Offset   firstOff = first.atEnd() ? GA_INVALID_OFFSET : *first;
            const auto key = std::make_pair(gdp, firstOff);
            auto fit = fragments.find(key);
            if (fit == fragments.end())
            {
                InnerEntry e;
                e.handle = handle;
                collectFragmentPiece(gdp, range, std::to_string(int64_t(firstOff)), e.pieces);
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

// P / rotation / v / w of one point (a packed point's rotation from its primitive). False if
// P is not finite. A non-finite v or w is read as zero (Box3D asserts on it, and a NaN
// velocity would spread to the body's position); *badVelocity says so.
bool
readPointState(const GU_Detail *gdp, const PointAttrs &a, GA_Offset off,
               const PackedInfo &pk, Box3DBodyPose &out, bool *badVelocity = nullptr)
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

// What differs, or nullptr.
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
// Building the Box3D shapes for one point
// ---------------------------------------------------------------------------

// One collision piece: a hull, or a box / sphere / capsule.
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
    const char        *note = nullptr;      // set when something fell back
};

// Merge pieces into one point set.
void
mergePieces(const std::vector<std::vector<float>> &pieces, std::vector<float> &out)
{
    out.clear();
    for (const auto &p : pieces)
        out.insert(out.end(), p.begin(), p.end());
}

// Body-local points -> a hull, else a box fitted to them. False if no usable points.
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

// A point's shapes (AGENTS.md, "Pieces and packed geometry"):
//  box / sphere / capsule: fitted to the packed geometry with autofit, else pscale * scale
//                          plus the bullet_prim* / radius / length overrides
//  convex:  one hull of the packed geometry (hullPerConnected: one per connected set)
//  concave: one hull per `name` piece
// Convex / concave without packed geometry fall back to a box.
BuiltShapes
buildShapes(b3dop::Box3DWorld &world, ShapeSources &src, b3dop::ShapeType type,
            bool hullPerConnected, float pscale, const float scale[3], const PackedInfo &pk,
            bool autofit, const b3dop::PrimOverrides &ov)
{
    using namespace b3dop;
    BuiltShapes out;

    const bool wantGeometry = (type == ShapeType::Convex || type == ShapeType::Concave);

    // 1. The packed geometry (none for a plain point); connected sets for hullPerConnected.
    const bool        mergeAll = (type == ShapeType::Convex) && !hullPerConnected;
    //    Box / sphere / capsule only need it to autofit.
    const PieceGroup *group    = (pk.prim && (wantGeometry || autofit))
                                     ? src.innerPieces(*pk.prim, type == ShapeType::Convex && hullPerConnected)
                                     : nullptr;

    // 1b. Packed convex / concave: hulls keyed by (source, pivot, scale); points built on a miss.
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
        // nothing usable: fall through
    }

    // 2. Body-local points, one set per piece.
    std::vector<std::vector<float>> sets;
    if (group && out.pieces.empty())
    {
        for (const NamedPiece &np : group->pieces)
        {
            const int n = int(np.xyz.size() / 3);
            if (n == 0)
                continue;
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
        // box / sphere / capsule on a packed point: fit to the bounds.
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

    // 4. Nothing usable: pscale * scale.
    if (out.pieces.empty())
    {
        if (wantGeometry)
            out.note = pk.prim ? "packed geometry is empty; used a box"
                               : "convex / concave needs a packed primitive; used a box";
        Piece pc;
        pc.spec   = makeShapeSpec(wantGeometry ? ShapeType::Box : type, pscale, scale);
        if (!wantGeometry)
            applyPrimOverrides(pc.spec, ov);
        pc.volume = shapeVolume(pc.spec);
        out.totalVolume = pc.volume;
        out.pieces.push_back(pc);
    }
    return out;
}

// A point's `bullet_georep` via the shared-string handle (an int read), each distinct handle
// parsed once per scan (handles are per attribute table, so the map is per scan). Falls back to
// the string and the world-long memo without the shared-string interface.
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

    // False (out untouched) when missing, empty or unknown.
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

// Box3D asserts on negatives: clamp at 0, ignore non-finite.
float
nonNegative(float v, float fallback)
{
    return std::isfinite(v) ? std::max(v, 0.0f) : fallback;
}

// A point's changeable settings. Building and comparing both read through here, so an
// untouched point compares equal. sleepOk is the creation rule; scans of existing bodies only
// take bullet_want_deactivate (enablesleep is creation only, as in Bullet).
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
        if (std::isfinite(g)) p.gravityScale = g;      // may be negative
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

// The object's profile, or a point's own when it has collisiongroup / collisionignore.
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

// The scale convex / concave shapes depend on: packed row lengths, else pscale * scale.
bool
sizeKeyFromM3(const double m3[9], double out[3])
{
    return b3dop::matrixRowLengths(m3, out);
}

void
sizeKeyFromAttrs(float ps, const float sc[3], double out[3])
{
    for (int i = 0; i < 3; ++i)
        out[i] = double(ps) * double(sc[i]);
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

// From what pass 1 read.
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

// A point's shapes, plus their signature and scale key.
struct ShapePlan
{
    b3dop::ShapeType type = b3dop::ShapeType::Box;
    BuiltShapes      shapes;            // no pieces for `none`
    uint64_t         sig  = 0;
    bool             autoScale = false;
    double           sizeKey[3] = { 1, 1, 1 };
};

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

// A point's shapes from its current attributes and packed geometry. Only the hull cache is
// touched, no body.
ShapePlan
planShapes(b3dop::Box3DWorld &world, ShapeSources &src, GeorepLookup &georep, b3dop::ShapeTypeCache &shapeTypes,
           const b3dop::BodySettings &node, const PointAttrs &a, GA_Offset off, int64_t bodyId,
           const PackedInfo &pk)
{
    ShapePlan plan;

    plan.type = node.defaultShape;
    georep.get(a, off, shapeTypes, plan.type);       // else the node's default

    // `none`: a static, shapeless body (like Bullet, it still exists for constraints).
    if (plan.type == b3dop::ShapeType::None)
    {
        plan.sig = shapeSignature(plan.type, plan.shapes);
        return plan;
    }

    float ps, sc[3];
    readPscaleScale(a, off, ps, sc);

    // Without packed geometry there is nothing to fit: the explicit attributes apply.
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
        // Euler degrees, Houdini's XYZ rotate order. Non-finite: no override.
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

    // Convex / concave follow scale changes (as in Houdini); primitives do not.
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

// Adds the pieces without mass updates (the caller calls applyBodyMass). Returns the count.
int
addPlannedShapes(b3dop::Box3DWorld &world, b3dop::BodyHandle h, const b3dop::BodySettings &b, const ShapePlan &plan)
{
    int added = 0;
    for (const Piece &pc : plan.shapes.pieces)
        added += (pc.isHull ? world.createHullShape(h, b, pc.hull, false)
                            : world.createShape(h, b, pc.spec, false)) ? 1 : 0;
    return added;
}

// What the body was built from, into its mirror entry.
void
storePlan(DopBodyState &m, const ShapePlan &plan)
{
    m.shapeSig  = plan.sig;
    m.volume    = plan.shapes.totalVolume;
    m.autoScale = plan.autoScale;
    for (int i = 0; i < 3; ++i)
        m.sizeKey[i] = plan.sizeKey[i];
}

// Create a point's body (settings `p`), its shapes and initial velocity. `entry` becomes the
// body's user data and records what it was built from.
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
        *fallbackNote = plan.shapes.note;
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

    const b3dop::BodyHandle h = world.createBody(b, st.pos, st.rot, &entry);
    if (!h)
        return false;
    if (addPlannedShapes(world, h, b, plan) == 0)
    {
        world.destroyBody(h);       // never a shapeless body
        return false;
    }
    applyBodyMass(world, h, p, plan.shapes.totalVolume);

    // After the mass is final (a moving centre of mass shifts the velocity). Animated bodies
    // follow their point instead.
    const bool hasVel = st.lin[0] != 0.0f || st.lin[1] != 0.0f || st.lin[2] != 0.0f ||
                        st.ang[0] != 0.0f || st.ang[1] != 0.0f || st.ang[2] != 0.0f;
    if (hasVel && b.type != b3dop::BodySettings::Type::Static && !animated)
        world.setVelocityByHandle(h, st.lin, st.ang);
    if (p.ignored)
        world.setEnabledByHandle(h, false);     // bullet_ignore
    outHandle = h;
    return true;
}

// Rebuild a body's shapes from current attributes (deforming, bullet_autofit_valid = 0, or a
// convex / concave scale change), keeping the body. Identical shapes: nothing happens. Returns
// true if replaced; `noShape` if none could be added (the body is then static).
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
        storePlan(entry, plan);     // same shapes; refresh the size key
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
    applyBodyMass(world, h, p, plan.shapes.totalVolume);    // even with no shapes: clears Box3D's dirty mass
    world.wakeByHandle(h);          // let it re-settle
    B3D_LOG_VERBOSE("  body id=%lld: collision shape rebuilt", (long long)bodyId);
    return true;
}

// Push the settings that differ. After a rebuild the new shapes already carry material, mass
// and filter profile. Returns kPropsChanged / kMassChanged.
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
        changed |= kPropsChanged;       // read after the step; nothing to push
    e.props = p;
    return changed;
}

// Queue a point attribute write for after the step.
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

// The point's packed primitive, as pass 1 read it, for the write-back.
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
    int rebuilt = 0;            // shapes replaced
    int dead = 0;               // `dead` points
    int badVelocity = 0;        // non-finite v or w, read as zero
    int updated = 0;            // settings changed
    int skipped = 0;            // objects not scanned
    std::unordered_map<std::string, int> fallbacks;     // note -> count
};


constexpr size_t kProgressEvery = 64;       // points between Escape checks in the sync

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

    PointAttrs a(gdp);
    const bool anyPrims = gdp->getNumPrimitives() > 0;     // else no packed points
    const int  objectId = obj.getObjectId();
    // The skip check needs only the node's body type and a hash of its changeable values; the
    // full settings are read after it.
    const SIM_Box3DData *dataNode = SIM_DATA_GETCONST(obj, kDataName, SIM_Box3DData);
    int nodeType = int(b3dop::BodySettings().type);
    uint64_t nodeHash = 0;
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
        nodeHash = b3dop::mixHash(nodeHash, dataNode->getAddImpact() ? 1 : 2);
    }
    b3dop::Box3DWorld &world = st.world;

    // ---- unchanged since we last finished with it? ---------------------------
    // Markers are recorded after a scan and again after our own writes. Same markers: no point
    // can differ from the mirror, so both passes are skipped and the entries stay current
    // under liveStamp.
    Box3DObjectSync &os = st.objectSync[objectId];
    os.visited   = st.syncStamp;
    os.addImpact = dataNode && dataNode->getAddImpact();
    const Box3DSyncFingerprint fpNow = makeFingerprint(gdp, a, nodeType, nodeHash);
    if (os.canSkip && fpNow == os.fp)
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
    const b3dop::BodySettings node = dataNode ? dataNode->toBodySettings() : b3dop::BodySettings();
    os.canSkip   = false;
    os.liveStamp = st.syncStamp;
    size_t stamped = 0;                 // entries stamped by this scan
    const int problems0 = c.failed + c.duplicates + c.badP;

    ShapeSources sources;
    GeorepLookup georep;
    a.bindBuildAttrs(gdp);
    georep.bind(a);

    // Settings are compared only when a prop marker changed (or deforming bodies need a rebuild
    // anyway); otherwise the scan syncs pose, velocity and type. Convex / concave scale is
    // checked on every scan.
    const bool checkProps = !os.propsKnown || !os.fp.samePropMarkers(fpNow) || os.anyDeforming;
    bool       anyDeforming = false;
    bool       anyDriven    = false;

    // The object's collision filter profile (name, id).
    b3dop::FilterProfile baseProfile;
    {
        const UT_String &nm = obj.getName();
        if (nm.isstring())
            baseProfile.objectName = nm.buffer();
        baseProfile.objectId = std::to_string(objectId);
    }
    const int baseProfileIdx = world.filterProfile(baseProfile);

    // For the post-step passes; kept as they are while the object is skipped.
    os.impacts.clear();
    os.activation.clear();

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
            s.m3[0] = pk.m4[0];  s.m3[1] = pk.m4[1];  s.m3[2] = pk.m4[2];
            s.m3[3] = pk.m4[4];  s.m3[4] = pk.m4[5];  s.m3[5] = pk.m4[6];
            s.m3[6] = pk.m4[8];  s.m3[7] = pk.m4[9];  s.m3[8] = pk.m4[10];
        }
    }, anyPrims ? kMinParallelReadPacked : kMinParallelReadPlain);

    // Many new bodies (a fresh world): size the mirror once.
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

    // ---- pass 2 (serial, point order): apply to Box3D and the mirror ----------
    // An entry already carrying this step's stamp belongs to an earlier point (duplicate id).
    // Escape is checked here, between points: building hulls for a new world is the slow part.
    size_t pointIndex = 0;
    for (Box3DPointSlot &s : snaps)
    {
        if (pointIndex++ % kProgressEvery == 0 && progress.check())
            return;

        const GA_Offset off = s.off;
        const int64_t   bid = s.bid;
        const uint64_t  key = s.key;
        Box3DBodyPose  &cur = s.cur;

        DopBodyState *entry = s.entry;
        bool freshEntry = false;        // `entry` was just inserted, no body yet
        auto newIt = st.mirror.end();   // its node, to erase if the body cannot be built
        if (!entry)
        {
            if (s.ok)
            {
                // One lookup for "known?" and "make room"; a new node is unstamped.
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
        if (s.badVelocity)
            ++c.badVelocity;        // simulated, with the bad v / w read as zero

        // `dead`: deleted after the step. Not stamped, so its body goes with the gone points'.
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
        const bool deforming   = a.deforming.isValid() && a.deforming.get(off) != 0;
        const bool refitWanted = a.autofitValid.isValid() && a.autofitValid.get(off) == 0;
        anyDeforming = anyDeforming || deforming;

        if (freshEntry)
        {
            PackedInfo pk = anyPrims ? getPackedInfo(gdp, off) : PackedInfo();
            readPackedPivot(pk);
            Box3DBodyProps p = readDynamicProps(node, a, off);
            p.filterProfile = pointFilterProfile(world, a, off, baseProfile, baseProfileIdx);
            std::string fallbackNote;
            b3dop::BodyHandle handle = 0;
            if (!addPointBody(world, sources, georep, st.shapeTypes, node, a, off, bid, pk, cur, active, animated,
                              p, *entry, handle, &fallbackNote))
            {
                ++c.failed;
                st.mirror.erase(newIt);         // drop the placeholder
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

        // ---- mid-simulation changes (AGENTS.md, "Mid-simulation updates") --------
        // Shape first (a shapeless body is static). Rebuilt only on deforming, bullet_autofit_valid
        // = 0 or a convex / concave scale change, like Bullet.
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
                p.sleepOk = entry->props.sleepOk;       // enablesleep is creation only
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
            // A user-written bullet_sleeping = 0 wakes the body.
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
            // A type change recomputes the mass from the shapes: centre back to the point.
            if (entry->dynamic)
                applyBodyMass(world, entry->handle, entry->props, entry->volume);
            ++c.retyped;
        }
        entry->animated = animated;

        if (animated && entry->type == b3dop::BodySettings::Type::Kinematic)
        {
            // Follow the point: a new pose becomes this step's target; when it stops, clear the
            // velocity the target left.
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
    }

    // Problems (retried and reported every step), deforming and driven bodies keep it scanning.
    os.entryCount   = stamped;
    os.fp           = fpNow;
    os.propsKnown   = true;
    os.anyDeforming = anyDeforming;
    os.anyDriven    = anyDriven;
    os.canSkip      = (c.failed + c.duplicates + c.badP) == problems0 && !anyDeforming && !anyDriven;
}

// Creates `orient`, `v`, `w` if missing. True if `orient` was created.
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

// Harden only the offset ranges the parallel write pass touches (hardenAllPages takes a range).
// Written offsets are bucketed by 1024 and runs of buckets hardened; nearly all hit: one full
// call. `end` is the first offset after a run, right whether the HDK reads it as exclusive or not.
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

// Each item's point offset in the geometry about to be written: the entry's stored offset if
// it still names the point (in range, same id / number), else looked up by id;
// GA_INVALID_OFFSET if not found.
template <class Item>       // needs .entry and .off
void
resolvePointOffsets(const GU_Detail *gdp, const PointAttrs &a,
                    std::vector<Item> &items, size_t begin, size_t end)
{
    std::unordered_map<int64_t, GA_Offset> byId;     // built only when needed
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

// A writable SIM_GeometryCopy for the object's Geometry. Objects from RBD Packed Object carry
// a plain SIM_Geometry: it is replaced once by a SIM_GeometryCopy that adopts its geometry.
SIM_GeometryCopy *
getOrMakeWritableGeometry(SIM_Object &obj)
{
    if (SIM_GeometryCopy *geo = SIM_DATA_GET(obj, SIM_GEOMETRY_DATANAME, SIM_GeometryCopy))
        return geo;

    if (!SIM_DATA_GETCONST(obj, SIM_GEOMETRY_DATANAME, SIM_Geometry))
        return nullptr;

    SIM_GeometryCopy *geo = SIM_DATA_CREATE(obj, SIM_GEOMETRY_DATANAME, SIM_GeometryCopy,
            SIM_DATA_RETURN_EXISTING | SIM_DATA_ADOPT_EXISTING_ON_DELETE);
    if (geo)
        B3D_LOG("  writeBack: object id=%d Geometry converted to a writable SIM_GeometryCopy",
                obj.getObjectId());
    return geo;
}


// Re-record the fingerprint after our own write, so it does not look like an external edit.
// Call with every write lock released (releasing one may bump ids).
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
    const GU_Detail *gdp = rlock.getGdp();
    if (!gdp)
        return;
    const PointAttrs a(gdp);
    f->second.fp = makeFingerprint(gdp, a, f->second.fp.nodeType, f->second.fp.nodeHash);
}

// Binary search (`objects` is sorted by id), linear as a fallback.
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

// Make the next sync scan the object, after a write the scan must act on (active, dead, deletions).
void
forceScan(Box3DSharedState &st, int objectId)
{
    const auto f = st.objectSync.find(objectId);
    if (f != st.objectSync.end())
        f->second.canSkip = false;
}

// f(begin, end) per run of items with the same objectId (sorted first if needed).
template <class Item, class F>
void
forEachObjectRun(std::vector<Item> &items, const F &f)
{
    const auto byObject = [](const Item &x, const Item &y) { return x.objectId < y.objectId; };
    if (!std::is_sorted(items.begin(), items.end(), byObject))
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

// Add Impact Data: this step's contact points with impulse go into a fresh "Impacts" data
// (SIM_Impacts), only on steps with at least one; otherwise the data is removed. Contacts where
// neither body is awake are skipped. Normal: of the surface hit, pointing at this body. No
// Feedback data (that is for interaction with other solvers).
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
                continue;                       // asleep
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
    // Had Impacts last step, none now (or the option is off).
    std::sort(wrote.begin(), wrote.end());
    for (const int id : st.impactObjects)
        if (!std::binary_search(wrote.begin(), wrote.end(), id))
            if (SIM_Object *obj = findObjectById(objects, id))
                obj->removeNamedSubData(SIM_IMPACTS_DATANAME);
    st.impactObjects.swap(wrote);
}

// An inactive body hit with at least min_activation_impulse (not by a body matching
// activationignore) turns dynamic now and moves from the next step (Box3D reports impulses
// after the step; Bullet can activate within it). Its `active` is set to 1.
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

// The queued point writes: bullet_autofit_valid = 1, the computed `mass`, `active` = 1. A point
// that cannot be found keeps its value (a missed bullet_autofit_valid just refits again).
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
            return;             // writeResultsBack warned already
        unsigned want = 0;
        for (size_t k = begin; k < end; ++k)
            want |= items[k].flags;
        unsigned wrote = 0;
        {
            SIM_GeometryAutoWriteLock lock(geo);
            GU_Detail *gdp = &lock.getGdp();
            if (!gdp)
                return;
            if ((want & Box3DPointWrite::kMass) && !gdp->findPointAttribute("mass"))
                gdp->addFloatTuple(GA_ATTRIB_POINT, "mass", 1);
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
            forceScan(st, obj->getObjectId());      // so the scan keeps the body dynamic
    });
    items.clear();
}

// Enable Aging: age += dt on every point; dead = 1 once age exceeds a positive life.
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
            GU_Detail *gdp = &lock.getGdp();
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

// Delete the `dead` points (packed ones with their primitive), each checked to still be the
// same point first.
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
            GU_Detail *gdp = &lock.getGdp();
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
    // Minimum 1 (Box3D clamps to it); larger than 16 can be typed.
    static PRM_Range   theSubStepsRange(PRM_RANGE_RESTRICTED, 1, PRM_RANGE_UI, 16);

    static PRM_Name    theAutoThreadsName("autothreads", "Use All Threads");
    static PRM_Default theAutoThreadsDefault(1);

    static PRM_Name    theThreadsName("threads", "Threads");
    static PRM_Default theThreadsDefault(8);
    // Box3D's limit is 32.
    static PRM_Range   theThreadsRange(PRM_RANGE_RESTRICTED, 1, PRM_RANGE_RESTRICTED, 32);
    // Greyed out while "Use All Threads" is ticked.
    static PRM_Conditional theThreadsDisable("{ autothreads == 1 }", PRM_CONDTYPE_DISABLE);

    static PRM_Name    theSleepName("enablesleep", "Allow Sleeping");
    static PRM_Default theSleepDefault(1);

    static PRM_Name    theTimeScaleName("timescale", "Time Scale");
    static PRM_Default theTimeScaleDefault(1.0);
    static PRM_Range   theTimeScaleRange(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 2);

    static PRM_Name    thePushSpeedName("contactpushspeed", "Overlap Push Speed");
    static PRM_Default thePushSpeedDefault(3.0);
    static PRM_Range   thePushSpeedRange(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 10);

    static PRM_Name    theAgingName("enableaging", "Enable Aging");
    static PRM_Default theAgingDefault(0);

    // Box3D's solver tuning, with Box3D's defaults (Bullet's ERP / CFM have no equivalent).
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
        PRM_Template(PRM_FLT_J,  3, &theGravityName,  theGravityDefault, 0, 0, 0, 0, 1,
                     "Acceleration applied to every dynamic body, in units per second squared. "
                     "Scale it per body with Gravity Scale on Box3D Data."),
        PRM_Template(PRM_INT_J,  1, &theSubStepsName, &theSubStepsDefault, 0, &theSubStepsRange, 0, 0, 1,
                     "Solver passes per time step. More substeps make stacks and fast collisions more stable "
                     "but take longer."),
        PRM_Template(PRM_TOGGLE, 1, &theAutoThreadsName, &theAutoThreadsDefault, 0, 0, 0, 0, 1,
                     "Use one Box3D worker thread per CPU core (up to 32). Turn off to set Threads yourself."),
        PRM_Template(PRM_INT_J,  1, &theThreadsName,  &theThreadsDefault, 0, &theThreadsRange,
                     0, 0, 1,
                     "Number of Box3D worker threads. Only used when Use All Threads is off.",
                     &theThreadsDisable),
        PRM_Template(PRM_TOGGLE, 1, &theSleepName,    &theSleepDefault, 0, 0, 0, 0, 1,
                     "Let bodies that have come to rest stop simulating until something touches them. "
                     "Turning this off keeps every body awake, whatever each body's own setting."),
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
    // Keep one past timestep (the world we continue from). Debug builds: BOX3DDOP_REQUESTED_CACHE.
    const int value = B3D_ENV_INT("BOX3DDOP_REQUESTED_CACHE", 1);
    return value;
}

int
SIM_SolverBox3D::currentWorkerCount() const
{
    // Parallel tasks per step, run on Houdini's TBB pool: all Houdini threads (honours
    // HOUDINI_MAXTHREADS), or the Threads parm, never more than Houdini allows.
    const int houdini = std::max(1, int(UT_Thread::getNumProcessors()));
    int n = getAutoThreads() ? houdini : int(getThreads());
    n = std::min(n, houdini);
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
    // Start or end of the step: canContinue() does not care.
    const double now = engine.getSimulationTime();
    const double dt  = timestep;

    if (objects.entries() == 0)
        return SIM_SOLVER_SUCCESS;

    // Box3D assertions logged during this solve (RelWithDebInfo builds; see Box3DWorld).
    const int assertsBefore = b3dop::Box3DWorld::assertCount();
    struct AssertReport
    {
        SIM_SolverBox3D &solver; SIM_ObjectArray &objects; int before;
        ~AssertReport()
        {
            const int n = b3dop::Box3DWorld::assertCount() - before;
            if (n > 0 && objects.entries() > 0)
                solver.addError(objects(0), SIM_MESSAGE,
                    ("Box3D: " + std::to_string(n) + " internal Box3D assertion(s) failed this step and were "
                     "ignored (details in Houdini's console). Results may be wrong.").c_str(),
                    UT_ERROR_WARNING);
        }
    } assertReport{ *this, objects, assertsBefore };

    // Escape (see StepProgress).
    StepProgress progress;
    progress.boss = UTgetInterrupt();
    if (progress.check())
        return SIM_SOLVER_FAIL;         // cancelled before anything changed

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
                // The pool is sized at creation; a rebuild only loses warm starting and sleep.
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

    // A failed rebuild leaves the old state in place, valid but from another time: never
    // continue it.
    if (reason && !createFreshWorld(*worldData, reason))
    {
        reportWorldCreationFailure(objects);
        return SIM_SOLVER_FAIL;
    }
    Box3DSharedState *st = worldData->state();
    if (!st || !st->world.isValid())
    {
        reportWorldCreationFailure(objects);
        return SIM_SOLVER_FAIL;
    }

    std::lock_guard<std::mutex> lock(st->mutex);

    // The physics advances by dt * Time Scale.
    const double timeScale = std::max(0.0, double(getTimeScale()));
    st->stepDt = float(dt * timeScale);
    // Solver parameters may be animated; only the thread count needs a new world.
    const b3dop::WorldSettings tuning = currentWorldSettings();
    if (!std::isfinite(tuning.gravity[0]) || !std::isfinite(tuning.gravity[1]) || !std::isfinite(tuning.gravity[2]))
        addError(objects(0), SIM_MESSAGE,
                 "Box3D: Gravity is not a finite number; the previous gravity is kept", UT_ERROR_WARNING);
    st->world.applyTuning(tuning);      // ignores non-finite gravity

    // Cancelled mid-sync, or before the (uninterruptible) Box3D step: the world may be half
    // updated, so no step may continue from it. Unstamped, the next solve that would have
    // continued it builds a fresh world from its geometry instead.
    auto cancel = [&]() {
        st->stamped = false;
        addError(objects(0), SIM_MESSAGE, "Box3D: step cancelled", UT_ERROR_MESSAGE);
        return SIM_SOLVER_FAIL;
    };
    if (!syncFromGeometry(*worldData, objects, progress))
        return cancel();
    if (progress.check())
        return cancel();

    // From here the step runs to the end (the write-back is quick), so the cached result is
    // always a whole step.
    stepWorld(*worldData, st->stepDt);
    writeResultsBack(*worldData, objects);
    // Contacts are read before the queued point writes, aging and deletions.
    recordImpacts(*st, objects, engine.getSimulationTime());
    activateByImpact(*st);
    writePointAttrs(*st, objects);
    if (getEnableAging())
        ageObjects(*st, objects, float(dt));
    deleteDeadPoints(*st, objects);

    st->stampTime = now;
    st->stamped   = true;
    return SIM_SOLVER_SUCCESS;
}

// ---------------------------------------------------------------------------

SIM_Box3DWorld *
SIM_SolverBox3D::findOrCreateWorldData(SIM_ObjectArray &objects)
{
    // The first object carrying world data (the carrier may change). Only the carrier is
    // fetched non-const, which is what copies the (cheap, sharing) data.
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

bool
SIM_SolverBox3D::createFreshWorld(SIM_Box3DWorld &worldData, const char *reason)
{
    (void)reason;       // only used by the debug log
    B3D_COUNT(rebuilds);
    B3D_LOG("REBUILD #%d: %s", B3D_COUNTER(rebuilds), reason);

    // The old state stays for other timesteps; the empty mirror makes every point a new body.
    return worldData.createWorld(currentWorldSettings());
}

void
SIM_SolverBox3D::reportWorldCreationFailure(SIM_ObjectArray &objects)
{
    const int live = b3dop::Box3DWorld::liveWorldCount();
    const int max  = b3dop::Box3DWorld::maxWorldCount();
    if (live >= max)
    {
        const std::string msg =
            "Box3D: world limit reached (" + std::to_string(live) + " of " + std::to_string(max) +
            " Box3D worlds alive). Each rebuild keeps the previous world while cached frames use it. "
            "Reset the simulation or clear the DOP cache to free them, or raise BOX3DDOP_MAX_WORLDS "
            "when building the plugin.";
        addError(objects(0), SIM_MESSAGE, msg.c_str(), UT_ERROR_ABORT);
    }
    else
        addError(objects(0), SIM_MESSAGE, "Box3D: world creation failed", UT_ERROR_ABORT);
}

bool
SIM_SolverBox3D::syncFromGeometry(SIM_Box3DWorld &worldData, SIM_ObjectArray &objects,
                                  StepProgress &progress)
{
    Box3DSharedState *st = worldData.state();

    // TODO: objects in `newobjects` should keep their initial state on their first step
    // (e.g. step them as kinematic once).

    SyncCounts                   counts;

    ++st->syncStamp;
    st->stampedCount = 0;
    st->pointWrites.clear();
    st->deletions.clear();
    for (exint i = 0; i < objects.entries(); ++i)
    {
        if (progress.check())
            return false;
        syncObject(*st, *objects(i), counts, progress);
        if (progress.cancelled)
            return false;       // half synced: the caller discards the world
    }
    st->world.finishFilterProfiles();       // before the step

    // Remove bodies whose point (or object) is gone: the unstamped entries.
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
        // Slots may cache pointers to erased entries.
        st->slots.clear();
    }

    // Objects not seen lose their records (their bodies were just removed).
    for (auto it = st->objectSync.begin(); it != st->objectSync.end();)
        it = (it->second.visited == st->syncStamp) ? std::next(it) : st->objectSync.erase(it);

    // Free hulls no body uses (only adding, rebuilding or removing bodies can leave some).
    if (counts.added || counts.failed || counts.rebuilt || removed)
        st->world.pruneHulls();

    // Node warnings, one per kind with a count. Fallbacks only on the step a shape is built.
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
    if (counts.badVelocity)
        warn("Box3D: " + std::to_string(counts.badVelocity) + " point(s) have a non-finite v or w; it was treated as zero");
    if (counts.dead && !st->deletions.empty())
        B3D_LOG("  sync: %d dead point(s) removed", counts.dead);

    if (counts.skipped)
        B3D_LOG_VERBOSE("  sync: %d of %d object(s) unchanged, scan skipped", counts.skipped, int(objects.entries()));
    // Adds, removals and rebuilds are logged; the rest can repeat every step (BOX3DDOP_VERBOSE=1).
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
    return true;
}

void
SIM_SolverBox3D::stepWorld(SIM_Box3DWorld &worldData, float timestep)
{
    worldData.state()->world.step(timestep);
}

void
SIM_SolverBox3D::writeResultsBack(SIM_Box3DWorld &worldData, SIM_ObjectArray &objects)
{
    Box3DSharedState *st = worldData.state();
    b3dop::Box3DWorld &world = st->world;

    // ---- pass 0 (serial): the bodies that moved -----------------------------
    // Box3D's move events: every awake body and its transform (a body falling asleep appears
    // once more). Each body's user data is its mirror entry.
    const int moveCount = world.getMoveEvents(st->moves);
    if (moveCount == 0)
        return;

    std::vector<Box3DWriteItem> &items = st->writeItems;
    items.clear();
    items.reserve(size_t(moveCount));
    std::vector<Box3DPackedWrite> &packedWrites = st->packedWrites;
    packedWrites.clear();
    int missing = 0;
    // "Seen by this sync" is per object (Box3DSharedState::isLive); the last record is cached.
    int                    lastObj = -1;
    const Box3DObjectSync *lastOs  = nullptr;
    for (const b3dop::BodyMove &mv : st->moves)
    {
        if (!mv.userData)
            continue;
        DopBodyState *entry = static_cast<DopBodyState *>(mv.userData);
        if (!entry->dynamic)
            continue;                       // only dynamic bodies are written
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
            ++missing;
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
        // bullet_sleeping: only changes are written.
        if (mv.fellAsleep != entry->sleeping)
        {
            w.sleepState    = mv.fellAsleep ? 1 : 0;
            entry->sleeping = mv.fellAsleep;
        }
    }
    if (items.empty())
        return;

    // Velocities (not in move events): Box3D allows read-only calls from several threads.
    parallelForEach(items.size(), [&](size_t i) {
        Box3DWriteItem &w = items[i];
        world.getVelocityByHandle(w.entry->handle, w.s.lin, w.s.ang);
    }, kMinParallelWrite);

    // Group by object, so each geometry is locked once.
    bool oneObject = true;
    for (const Box3DWriteItem &w : items)
        if (w.objectId != items.front().objectId) { oneObject = false; break; }
    const auto byObject = [](const Box3DWriteItem &x, const Box3DWriteItem &y) { return x.objectId < y.objectId; };
    if (!oneObject && !std::is_sorted(items.begin(), items.end(), byObject))
        std::stable_sort(items.begin(), items.end(), byObject);

    std::vector<SIM_Object *> refresh;      // objects written to
    for (size_t runBegin = 0; runBegin < items.size();)
    {
        size_t runEnd = runBegin;
        while (runEnd < items.size() && items[runEnd].objectId == items[runBegin].objectId)
            ++runEnd;
        const size_t begin = runBegin, end = runEnd;
        runBegin = runEnd;

        SIM_Object *obj = findObjectById(objects, items[begin].objectId);
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
        GU_Detail *gdp = &lock.getGdp();
        if (!gdp)
            continue;
        refresh.push_back(obj);

        const bool createdOrient = ensureResultAttribs(gdp);

        const PointAttrs a(gdp);          // after the attributes exist
        const bool anyPrims = gdp->getNumPrimitives() > 0;
        GA_RWHandleV3 vOut(gdp, GA_ATTRIB_POINT, "v");
        GA_RWHandleV3 wOut(gdp, GA_ATTRIB_POINT, "w");
        GA_RWHandleQ  qOut(gdp, GA_ATTRIB_POINT, "orient");

        if (createdOrient && qOut.isValid())
        {
            // A new quaternion attribute is 0,0,0,0: start every point at identity.
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

        // ---- pass 2 (parallel): write attributes, one task per body ------------
        GA_RWHandleV3 pOut(gdp, GA_ATTRIB_POINT, "P");
        GA_RWHandleI  sleepOut(gdp, GA_ATTRIB_POINT, "bullet_sleeping");      // only if it exists
        bool anySleepWrite = false;
        if (sleepOut.isValid())
            for (size_t k = begin; k < end && !anySleepWrite; ++k)
                anySleepWrite = items[k].sleepState >= 0 && items[k].off != GA_INVALID_OFFSET;
#ifdef BOX3DDOP_DEBUG
        const bool verifyReadback = B3D_ENV_INT("BOX3DDOP_VERIFY_READBACK", 0) != 0;
#endif
        // Shared copy-on-write pages would race: a parallel pass hardens what it writes first.
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

            // Packed: the new 3x3 (old scale, new rotation) is computed here and written in
            // pass 3, using the primitive and 3x3 the last sync stored.
            Box3DBodyMirror  &me     = *w.entry;
            Box3DPackedWrite *pw     = w.packedIdx >= 0 ? &packedWrites[size_t(w.packedIdx)] : nullptr;
            const bool        packed = anyPrims && me.packed && pw;
            float             packedRot[4] = { 0, 0, 0, 1 };
            if (packed)
            {
                composePackedMatrix(s.rot, me.m3, pw->newM3);
                pw->primOff = GA_Offset(me.primOff);
                pw->has     = true;
                // What the primitive will hold; the entry's 3x3 follows (a skipped sync does
                // not refresh it).
                double sc[3];
                splitPackedMatrix(pw->newM3, packedRot, sc);
                for (int i = 0; i < 9; ++i) me.m3[i] = pw->newM3[i];
            }

            // Predict what readPointState will return (else our write looks like an external
            // edit): P via float, rotations via sanitizeQuat. Non-finite P: keep Box3D's values.
            Box3DBodyPose &mirror = me;
            double sp[3];
            if (b3dop::storedPosition(s.pos, sp))
            {
                for (int i = 0; i < 3; ++i) mirror.pos[i] = sp[i];
                // An unbound output was not written: the read path returns its default.
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
                // BOX3DDOP_VERIFY_READBACK=1: read back and compare bit for bit.
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
        // Serial: the HDK does not document setLocalTransform as thread-safe.
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
                ++missing;          // topology changed without a new id
                continue;
            }
            setLocal3(*static_cast<GU_PrimPacked *>(gdp->getPrimitive(pw.primOff)), pw.newM3);
            wrotePacked = true;
        }
        // setLocalTransform does not mark the primitive list dirty; the HDK leaves that to us.
        if (wrotePacked)
            gdp->getPrimitiveList().bumpDataId();
    }

    // Re-record the fingerprints now that every lock is released.
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
