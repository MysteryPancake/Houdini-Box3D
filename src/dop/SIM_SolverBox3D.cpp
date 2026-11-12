// SIM_SolverBox3D.cpp
#include "SIM_SolverBox3D.h"
#include "SIM_Box3DWorld.h"
#include "SIM_Box3DData.h"
#include "core/Box3DDebug.h"
#include "sync/Box3DAffectors.h"
#include "sync/Box3DJointSync.h"

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
#include <SIM/SIM_DataFilter.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Engine.h>
#include <SIM/SIM_Error.h>
#include <SIM/SIM_Force.h>
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
#include <UT/UT_Matrix4.h>
#include <UT/UT_ParallelUtil.h>
#include <UT/UT_Quaternion.h>
#include <UT/UT_Thread.h>
#include <UT/UT_Vector3.h>
#include <UT/UT_XformOrder.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <iterator>
#include <map>
#include <mutex>
#include <string>
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

// Data name of the world (Box3D Data's is kBox3DDataName).
const char *const kWorldDataName = "Box3DWorld";

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

// A per-row scale (splitPackedMatrix's, kept from the sync) with q's rotation: a packed
// primitive's new local 3x3.
void
composePackedMatrix(const float q[4], const double sc[3], double out[9])
{
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
    GA_ROHandleI  fastRotation;    // fastrotation
    GA_ROHandleI  contactRecycling;    // contactrecycling
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
    GA_ROHandleID collisionCategory;   // b3Filter: read as 64-bit (a 32-bit -1 is every bit)
    GA_ROHandleID collisionMask;
    GA_ROHandleI  collisionGroup;
    GA_ROHandleV3 lockLinear;          // lock_linear / lock_angular: per world axis, nonzero = locked
    GA_ROHandleV3 lockAngular;
    GA_ROHandleI  animated;
    GA_ROHandleF  density;
    GA_ROHandleI  computeMass;
    GA_ROHandleF  inertiaStiffness;    // inertialtensorstiffness
    GA_ROHandleF  minActivation;   // min_activation_impulse
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
        fastRotation   = GA_ROHandleI(gdp, GA_ATTRIB_POINT, "fastrotation");
        contactRecycling = GA_ROHandleI(gdp, GA_ATTRIB_POINT, "contactrecycling");
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
        collisionCategory = GA_ROHandleID(gdp, GA_ATTRIB_POINT, "collision_category");
        collisionMask  = GA_ROHandleID(gdp, GA_ATTRIB_POINT, "collision_mask");
        collisionGroup = GA_ROHandleI(gdp, GA_ATTRIB_POINT, "collision_group");
        lockLinear     = GA_ROHandleV3(gdp, GA_ATTRIB_POINT, "lock_linear");
        lockAngular    = GA_ROHandleV3(gdp, GA_ATTRIB_POINT, "lock_angular");
        animated       = GA_ROHandleI(gdp, GA_ATTRIB_POINT, "animated");
        density        = GA_ROHandleF(gdp, GA_ATTRIB_POINT, "density");
        computeMass    = GA_ROHandleI(gdp, GA_ATTRIB_POINT, "computemass");
        inertiaStiffness = GA_ROHandleF(gdp, GA_ATTRIB_POINT, "inertialtensorstiffness");
        minActivation  = GA_ROHandleF(gdp, GA_ATTRIB_POINT, "min_activation_impulse");
        dead           = GA_ROHandleI(gdp, GA_ATTRIB_POINT, "dead");
        sleeping       = GA_ROHandleI(gdp, GA_ATTRIB_POINT, "bullet_sleeping");
    }
};

// Attributes whose data ids are the fingerprint's prop markers (propIds, in order). The shape
// attributes are left out on purpose: like Bullet, editing them alone changes nothing until
// bullet_autofit_valid = 0 or deforming = 1.
const char *const kPropAttribNames[] = {
    "mass", "friction", "bounce", "lineardamping", "angulardamping", "gravityscale",
    "bullet_want_deactivate", "bullet_ignore", "deforming", "bullet_autofit_valid",
    "pscale", "scale",      // a plain point's convex / concave fallback box
    "collision_category", "collision_mask", "animated", "density", "computemass",
    "inertialtensorstiffness", "sleepthreshold", "fastobject",
    "min_activation_impulse", "collision_group", "dead", "bullet_sleeping",
    "contactrecycling", "lock_linear", "lock_angular", "fastrotation",
    "name",                 // joint anchors find bodies by it
};
static_assert(std::size(kPropAttribNames) == Box3DSyncFingerprint::kNumPropIds,
              "kPropAttribNames and Box3DSyncFingerprint::kNumPropIds must match");

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
    double               m3[9]      = { 1, 0, 0,  0, 1, 0,  0, 0, 1 };    // m4's upper 3x3
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
parallelForEach(size_t n, const F &f, size_t minParallel = kMinParallelPoints, size_t grain = 1)
{
    if (n < minParallel)
    {
        for (size_t i = 0; i < n; ++i)
            f(i);
        return;
    }
    UTparallelFor(UT_BlockedRange<size_t>(0, n, grain), [&f](const UT_BlockedRange<size_t> &r) {
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
    b3dop::upper3x3(pk.m4, pk.m3);
    splitPackedMatrix(pk.m3, pk.rot, pk.scale);
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
    std::string            name;
    std::vector<GA_Offset> points;      // while collecting (finishPieces reads and drops them)
    std::vector<float>     xyz;
    uint64_t               hash = 0;    // of xyz
};

struct PieceGroup
{
    std::vector<NamedPiece>                    pieces;
    std::unordered_map<std::string, size_t>    byName;
    uint64_t                                   hash = 0;   // of all pieces: the merged hull's key

    NamedPiece &piece(const std::string &name)
    {
        auto it = byName.find(name);
        if (it == byName.end())
        {
            it = byName.emplace(name, pieces.size()).first;
            pieces.emplace_back();
            pieces.back().name = name;
        }
        return pieces[it->second];
    }
};

void
appendPrimPoints(const GA_Primitive &prim, NamedPiece &np)
{
    const GA_Size nv = prim.getVertexCount();
    for (GA_Size i = 0; i < nv; ++i)
        np.points.push_back(prim.getPointOffset(i));
}

// Each piece's points once (in offset order) into xyz, then the hashes of each piece and of all
// of them together (once per shared source).
void
finishPieces(const GU_Detail *gdp, PieceGroup &g)
{
    g.hash = 0;
    for (NamedPiece &np : g.pieces)
    {
        std::sort(np.points.begin(), np.points.end());
        np.points.erase(std::unique(np.points.begin(), np.points.end()), np.points.end());
        np.xyz.clear();
        np.xyz.reserve(np.points.size() * 3);
        for (const GA_Offset pt : np.points)
        {
            const UT_Vector3 p = gdp->getPos3(pt);
            np.xyz.push_back(p.x());  np.xyz.push_back(p.y());  np.xyz.push_back(p.z());
        }
        np.points = std::vector<GA_Offset>();
        np.hash = b3dop::hashFloats(np.xyz.data(), np.xyz.size());
        g.hash  = b3dop::mixHash(g.hash, np.hash);
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
    bool anyPrim = false;
    for (GA_Iterator it(gdp->getPrimitiveRange()); !it.atEnd(); ++it)
    {
        const GA_Primitive *prim = gdp->getPrimitive(*it);
        if (!prim)
            continue;
        anyPrim = true;
        std::string name;
        if (nameH.isValid())
            if (const char *n = nameH.get(*it))
                name = n;
        appendPrimPoints(*prim, out.piece(name));
    }
    if (!anyPrim)
    {
        NamedPiece &np = out.piece(std::string());
        for (GA_Iterator it(gdp->getPointRange()); !it.atEnd(); ++it)
            np.points.push_back(*it);
    }
    finishPieces(gdp, out);
}

// One piece per set of connected primitives (b3dop::connectedSets).
void
collectConnectedPieces(const GU_Detail *gdp, const GA_Range &prims, PieceGroup &out)
{
    std::vector<std::vector<GA_Offset>> primPoints;
    for (GA_Iterator it(prims); !it.atEnd(); ++it)
    {
        const GA_Primitive *prim = gdp->getPrimitive(*it);
        if (!prim)
            continue;
        primPoints.emplace_back();
        const GA_Size nv = prim->getVertexCount();
        for (GA_Size i = 0; i < nv; ++i)
            primPoints.back().push_back(prim->getPointOffset(i));
    }

    const std::vector<int> setOf = b3dop::connectedSets(primPoints);
    for (size_t i = 0; i < primPoints.size(); ++i)
    {
        NamedPiece &np = out.piece(std::to_string(setOf[i]));
        np.points.insert(np.points.end(), primPoints[i].begin(), primPoints[i].end());
    }
    finishPieces(gdp, out);
}

// A packed fragment shares the whole source detail but stands for the primitives in its range:
// those points, as one piece.
void
collectFragmentPiece(const GU_Detail *gdp, const GA_Range &range, const std::string &name,
                     PieceGroup &out)
{
    NamedPiece &np = out.piece(name);
    for (GA_Iterator it(range); !it.atEnd(); ++it)
        if (const GA_Primitive *prim = gdp->getPrimitive(*it))
            appendPrimPoints(*prim, np);
    finishPieces(gdp, out);
}

// What a source's pieces hash to, so a packed hull can be looked up without reading the source
// geometry. Non-empty pieces only, in piece order.
struct SourceSummary
{
    uint64_t              groupHash = 0;
    std::vector<uint64_t> pieceHashes;
};

// Source summaries by the source detail's identity (unique id, P and primitive list data ids,
// counts, fragment, connected sets or not). Process-wide, so a rebuilt world skips reading
// sources whose hulls are still cached. Relies on whoever edits a source bumping its data ids,
// as Houdini's own caches do.
class SourceSummaryCache
{
public:
    static SourceSummaryCache &instance()
    {
        static SourceSummaryCache c;
        return c;
    }
    bool find(uint64_t key, SourceSummary &out)
    {
        std::lock_guard<std::mutex> lock(myMutex);
        auto it = myMap.find(key);
        if (it == myMap.end())
            return false;
        out = it->second;
        return true;
    }
    void insert(uint64_t key, const PieceGroup &g)
    {
        SourceSummary sum;
        sum.groupHash = g.hash;
        for (const NamedPiece &np : g.pieces)
            if (!np.xyz.empty())
                sum.pieceHashes.push_back(np.hash);
        std::lock_guard<std::mutex> lock(myMutex);
        if (myMap.size() >= kMaxEntries)
            myMap.clear();
        myMap[key] = std::move(sum);
    }

private:
    static constexpr size_t kMaxEntries = 65536;
    std::mutex                                  myMutex;
    std::unordered_map<uint64_t, SourceSummary> myMap;
};

// The identity SourceSummaryCache is keyed by.
uint64_t
sourceIdentity(const GU_Detail *gdp, GA_Offset fragmentFirst, bool connected)
{
    const GA_Attribute *P = gdp->getP();
    uint64_t h = b3dop::mixHash(b3dop::kHashSeed, uint64_t(gdp->getUniqueId()));
    h = b3dop::mixHash(h, P ? uint64_t(P->getDataId()) : 0);
    h = b3dop::mixHash(h, uint64_t(gdp->getPrimitiveList().getDataId()));
    h = b3dop::mixHash(h, uint64_t(gdp->getNumPoints()));
    h = b3dop::mixHash(h, uint64_t(gdp->getNumPrimitives()));
    h = b3dop::mixHash(h, uint64_t(int64_t(fragmentFirst)));
    return b3dop::mixHash(h, connected ? 1 : 2);
}

// Collision source geometry (what is packed in each point's primitive), cached for one
// syncObject() call. Thread-safe: the shape pre-pass plans points in parallel.
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
    std::mutex                                        mutex;
    std::unordered_map<const GU_Detail *, InnerEntry> inner;
    // Fragments: (shared source detail, first primitive offset) -> that fragment's piece.
    std::map<std::pair<const GU_Detail *, GA_Offset>, InnerEntry> fragments;

    // The primitive's source detail. A packed fragment shares the WHOLE source detail
    // (detailPtr) with every fragment of it and stands for its own primitives in it. Not
    // getPackedDetail(): for a fragment that is a different detail from the one
    // getPrimitiveRange() indexes, so the range's offsets picked the wrong primitives or none
    // (none: the pscale * scale box).
    static GU_ConstDetailHandle sourceHandle(const GU_PrimPacked &prim, const GU_PackedFragment *frag)
    {
        return frag ? frag->detailPtr() : prim.getPackedDetail();
    }
    // GU_PackedFragment does not expose its attribute value; its primitive range does identify
    // it (no two fragments of one source share a primitive). GA_INVALID_OFFSET for a non-fragment.
    static GA_Offset fragmentFirst(const GU_PackedFragment *frag)
    {
        if (!frag)
            return GA_INVALID_OFFSET;
        const GA_Iterator first(frag->getPrimitiveRange());
        return first.atEnd() ? GA_INVALID_OFFSET : *first;
    }

    // The source's summary, if one was recorded for it (no geometry is read).
    static bool summary(const GU_PrimPacked &prim, bool connected, SourceSummary &out)
    {
        const GU_PackedFragment *frag = asFragment(prim);
        GU_DetailHandleAutoReadLock lock(sourceHandle(prim, frag));
        const GU_Detail *gdp = lock.getGdp();
        return gdp && SourceSummaryCache::instance().find(sourceIdentity(gdp, fragmentFirst(frag), connected), out);
    }

    // The primitive's pieces (by `name`, or connected sets), or null. Records the summary.
    const PieceGroup *innerPieces(const GU_PrimPacked &prim, bool connected)
    {
        const GU_PackedFragment *frag = asFragment(prim);
        GU_ConstDetailHandle handle = sourceHandle(prim, frag);
        GU_DetailHandleAutoReadLock lock(handle);
        const GU_Detail *gdp = lock.getGdp();
        if (!gdp)
            return nullptr;

        std::lock_guard<std::mutex> guard(mutex);
        const GA_Offset firstOff = fragmentFirst(frag);
        InnerEntry *entry = nullptr;
        bool        fresh = false;
        if (frag)
        {
            const auto key = std::make_pair(gdp, firstOff);
            auto fit = fragments.find(key);
            if (fit == fragments.end())
            {
                InnerEntry e;
                e.handle = handle;
                collectFragmentPiece(gdp, frag->getPrimitiveRange(), std::to_string(int64_t(firstOff)), e.pieces);
                fit   = fragments.emplace(key, std::move(e)).first;
                fresh = true;
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
                it    = inner.emplace(gdp, std::move(e)).first;
                fresh = true;
            }
            entry = &it->second;
        }

        if (!connected)
        {
            if (fresh)
                SourceSummaryCache::instance().insert(sourceIdentity(gdp, firstOff, false), entry->pieces);
            return entry->pieces.pieces.empty() || entry->pieces.pieces[0].xyz.empty()
                       ? nullptr : &entry->pieces;
        }

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
            SourceSummaryCache::instance().insert(sourceIdentity(gdp, firstOff, true), entry->connected);
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

// Position, orientation or velocity differ beyond float noise (q and -q are the same rotation).
bool
poseDiffers(const Box3DBodyPose &a, const Box3DBodyPose &b)
{
    const double eps = 1e-5;
    double dot = 0.0;
    for (int i = 0; i < 4; ++i) dot += double(a.rot[i]) * double(b.rot[i]);
    if (std::fabs(dot) < 1.0 - 1e-6)
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

const char *const kHullFallbackNote =
    "a piece could not be hulled (flat, < 4 points or too complex); used its bounding box";

// A hull of body-local points, cached under `key` (by content when 0), else a box fitted to the
// points. fill(pts) produces the points; with a key it is only called when the hull is not cached
// yet. `budgetGroup`: hulls of one source share their vertex budget (Box3DWorld). False if there
// are no usable points.
template <class Fill>
bool
pieceFromPoints(b3dop::Box3DWorld &world, uint64_t key, uint64_t budgetGroup, const Fill &fill,
                std::vector<float> &pts, Piece &out, const char *&note)
{
    float vol = 0.0f;
    b3dop::HullId h = key ? world.findHull(key, &vol) : 0;
    if (!h)
    {
        fill(pts);
        const int n = int(pts.size() / 3);
        h = key ? world.createHullWithKey(key, pts.data(), n, &vol, budgetGroup)
                : world.createHull(pts.data(), n, &vol);
        if (!h)
        {
            // Fewer than 4 points, flat, or too complex to hull: fall back to the bounds.
            if (!b3dop::fitShapeToPoints(b3dop::ShapeType::Box, pts.data(), n, out.spec))
                return false;
            out.isHull = false;
            out.volume = b3dop::shapeVolume(out.spec);
            note = kHullFallbackNote;
            return true;
        }
    }
    out.isHull = true;
    out.hull   = h;
    out.volume = vol;
    return true;
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
    auto add = [&](const Piece &pc) {
        out.totalVolume += pc.volume;
        out.pieces.push_back(pc);
    };

    const bool wantGeometry = (type == ShapeType::Convex || type == ShapeType::Concave);
    const bool mergeAll     = (type == ShapeType::Convex) && !hullPerConnected;
    const bool connected    = (type == ShapeType::Convex) && hullPerConnected;

    // Packed convex / concave hulls are keyed by (source, pivot, scale), so instances share them
    // and the points are built only on a miss. A degenerate transform has no scale: by content.
    double S[3];
    const bool keyed = pk.prim && wantGeometry && packedScale(pk.m4, S);

    // A source seen before (in any world): its summary gives the keys, and when every hull is
    // still cached the source geometry is not read at all.
    if (keyed)
    {
        SourceSummary sum;
        if (ShapeSources::summary(*pk.prim, connected, sum) && !sum.pieceHashes.empty())
        {
            std::vector<Piece> found;
            auto lookup = [&](uint64_t sourceHash, uint32_t kind) {
                Piece pc;
                pc.hull   = world.findHull(packedHullKey(sourceHash, pk.pivot, S, kind), &pc.volume);
                pc.isHull = pc.hull != 0;
                if (pc.isHull)
                    found.push_back(pc);
                return pc.isHull;
            };
            bool all = true;
            if (mergeAll)
                all = lookup(sum.groupHash, 1);
            else
                for (const uint64_t h : sum.pieceHashes)
                    if (!(all = lookup(h, 2)))
                        break;
            if (all)
            {
                for (const Piece &pc : found)
                    add(pc);
                return out;
            }
        }
    }

    // The packed geometry's non-empty pieces (none for a plain point; box / sphere / capsule only
    // need them to autofit). Connected sets for hullPerConnected.
    std::vector<const NamedPiece *> used;
    const PieceGroup *group = (pk.prim && (wantGeometry || autofit)) ? src.innerPieces(*pk.prim, connected)
                                                                     : nullptr;
    if (group)
        for (const NamedPiece &np : group->pieces)
            if (!np.xyz.empty())
                used.push_back(&np);

    // Body-local points of `from`, merged.
    std::vector<float> pts, local;
    auto toBodyLocal = [&](const std::vector<const NamedPiece *> &from, std::vector<float> &dst) {
        dst.clear();
        for (const NamedPiece *np : from)
        {
            packedPointsToBodyLocal(np->xyz.data(), int(np->xyz.size() / 3), pk.m4, pk.pivot, local);
            dst.insert(dst.end(), local.begin(), local.end());
        }
    };

    if (wantGeometry && !used.empty())
    {
        auto addHull = [&](uint64_t sourceHash, uint32_t kind, const std::vector<const NamedPiece *> &from) {
            const uint64_t key = keyed ? packedHullKey(sourceHash, pk.pivot, S, kind) : 0;
            Piece pc;
            if (pieceFromPoints(world, key, b3dop::mixHash(sourceHash, kind),
                                [&](std::vector<float> &dst) { toBodyLocal(from, dst); }, pts, pc, out.note))
                add(pc);
        };
        if (mergeAll)
            addHull(group->hash, 1, used);          // one hull from everything
        else
            for (const NamedPiece *np : used)
                addHull(np->hash, 2, { np });       // one hull per piece
    }
    else if (!wantGeometry && !used.empty())
    {
        // Box / sphere / capsule on a packed point with autofit: fit to the bounds.
        toBodyLocal(used, pts);
        Piece pc;
        if (fitShapeToPoints(type, pts.data(), int(pts.size() / 3), pc.spec))
        {
            pc.volume = shapeVolume(pc.spec);
            add(pc);
        }
    }

    // Nothing usable: pscale * scale.
    if (out.pieces.empty())
    {
        if (wantGeometry)
            out.note = pk.prim ? "packed geometry is empty; used a box"
                               : "convex / concave needs a packed primitive; used a box";
        Piece pc;
        pc.spec = makeShapeSpec(wantGeometry ? ShapeType::Box : type, pscale, scale);
        if (!wantGeometry)
            applyPrimOverrides(pc.spec, ov);
        pc.volume = shapeVolume(pc.spec);
        add(pc);
    }
    return out;
}

// A point's `bullet_georep` via the shared-string handle (an int read), each distinct handle
// parsed once per scan (handles are per attribute table, so the map is per scan). Without the
// shared-string interface the string is parsed directly.
struct GeorepLookup
{
    const GA_Attribute            *attr = nullptr;
    const GA_AIFSharedStringTuple *aif  = nullptr;
    struct Parsed { bool ok = false; b3dop::ShapeType type = b3dop::ShapeType::Box; };
    std::mutex                          mutex;      // get() runs in the parallel shape pre-pass
    std::unordered_map<int64_t, Parsed> byHandle;

    void bind(const PointAttrs &a)
    {
        attr = a.georep.isValid() ? a.georep.getAttribute() : nullptr;
        aif  = attr ? attr->getAIFSharedStringTuple() : nullptr;
    }

    // False (out untouched) when missing, empty or unknown.
    bool get(const PointAttrs &a, GA_Offset off, b3dop::ShapeType &out)
    {
        if (!attr)
            return false;
        if (!aif)
            return b3dop::parseShapeType(a.georep.get(off), out);
        const GA_StringIndexType handle = aif->getHandle(attr, off, 0);
        if (handle == GA_INVALID_STRING_INDEX)
            return false;
        std::lock_guard<std::mutex> lock(mutex);
        auto it = byHandle.find(int64_t(handle));
        if (it == byHandle.end())
        {
            Parsed p;
            p.ok = b3dop::parseShapeType(aif->getTableString(attr, handle), p.type);
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
    p.fastRotation = a.fastRotation.isValid() && a.fastRotation.get(off) != 0;
    p.contactRecycling = a.contactRecycling.isValid() ? a.contactRecycling.get(off) != 0 : node.contactRecycling;
    p.minActivationImpulse = 0.0f;
    if (a.minActivation.isValid())
    {
        const float m = a.minActivation.get(off);
        if (m > 0.0f && std::isfinite(m)) p.minActivationImpulse = m;
    }
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
              bool setShapeDensity = true)
{
    world.applyMassByHandle(h, boxDensity(p, volume), true, p.inertiaScale, setShapeDensity);
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
    b.filter         = p.filter;
    b.locks          = p.locks;
    b.sleepThreshold = p.sleepThreshold;
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

// What every point of one syncObject() scan shares.
struct ScanContext
{
    Box3DSharedState          &st;
    b3dop::Box3DWorld         &world;
    const GU_Detail           *gdp;
    const PointAttrs          &a;
    const b3dop::BodySettings &node;
    const bool                 anyPrims;           // else no packed points
    ShapeSources               sources;
    GeorepLookup               georep;

    ScanContext(Box3DSharedState &state, const GU_Detail *g, const PointAttrs &attrs,
                const b3dop::BodySettings &n)
        : st(state), world(state.world), gdp(g), a(attrs), node(n), anyPrims(g->getNumPrimitives() > 0)
    {
        georep.bind(a);
    }

    // The point's packed primitive with its pivot (what building shapes needs).
    PackedInfo packedInfo(GA_Offset off) const
    {
        PackedInfo pk = anyPrims ? getPackedInfo(gdp, off) : PackedInfo();
        readPackedPivot(pk);
        return pk;
    }

    // The point's changeable settings.
    Box3DBodyProps props(GA_Offset off) const { return readDynamicProps(node, a, off); }
};

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
    b.density = boxDensity(p, plan.shapes.totalVolume);

    const b3dop::BodyHandle h = world.createBody(b, st.pos, st.rot, &entry);
    if (!h)
        return 0;
    if (!entry.shapeless)
    {
        if (addPlannedShapes(world, h, b, plan) == 0)
        {
            world.destroyBody(h);       // never a shapeless body, except `none`
            return 0;
        }
        applyBodyMass(world, h, p, plan.shapes.totalVolume, false);

        // After the mass is final (a moving centre of mass shifts the velocity). Animated bodies
        // follow their point instead.
        const bool hasVel = st.lin[0] != 0.0f || st.lin[1] != 0.0f || st.lin[2] != 0.0f ||
                            st.ang[0] != 0.0f || st.ang[1] != 0.0f || st.ang[2] != 0.0f;
        if (hasVel && b.type != b3dop::BodySettings::Type::Static && !animated)
            world.setVelocityByHandle(h, st.lin, st.ang);
    }
    if (p.ignored)
        world.setEnabledByHandle(h, false);     // bullet_ignore
    return h;
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
        if (addPlannedShapes(world, h, b, plan) > 0)
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
    if (p.minActivationImpulse != o.minActivationImpulse)
        changed |= kPropsChanged;       // read after the step; nothing to push
    e.props = p;
    return changed;
}

// Queue a point attribute write for after the step.
void
queuePointWrite(Box3DSharedState &st, Box3DBodyMirror &m, unsigned flags, float mass = 0.0f)
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
constexpr size_t kPlanChunk     = 256;      // shape plans between Escape checks in the pre-pass

// Box3D Data values that change existing bodies (the fingerprint's nodeHash).
uint64_t
nodeSettingsHash(const b3dop::BodySettings &n, bool addImpact)
{
    const float v[] = { n.mass, n.friction, n.restitution, n.linearDamping, n.angularDamping, n.gravityScale,
                        n.density, n.computeMass ? 1.0f : 0.0f, n.sleepThreshold, n.isBullet ? 1.0f : 0.0f,
                        n.contactRecycling ? 1.0f : 0.0f };
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
        if (!a.wantDeactivate.isValid())
            p.sleepOk = entry.props.sleepOk;        // enablesleep is creation only
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
        // A user-written bullet_sleeping = 0 wakes the body.
        if (entry.sleeping && a.sleeping.isValid() && a.sleeping.get(off) == 0)
        {
            world.wakeByHandle(entry.handle);
            entry.sleeping = false;
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
    // Bodies by `name`, for joint anchors (only while some object has a joint network).
    os.byName.clear();
    os.namesKnown = st.wantNames;
    const GA_ROHandleS nameH(gdp, GA_ATTRIB_POINT, "name");
    StringAttr         names;
    names.bind(nameH);
    const bool         keepNames = st.wantNames && names.valid();
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
            entry->sleeping = false;
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
        if (!s.active && entry->props.minActivationImpulse > 0.0f)
            os.activation.push_back(entry);
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
// (SIM_Impacts) on the objects with the option on, only on steps with at least one; otherwise the
// data is removed. Contacts are read from the awake bodies (this step's move events, in
// st.moves) and recorded for both bodies, so sleeping bodies cost nothing and each pair is read
// once. Contacts where neither body is awake are skipped, as are asleep dynamic bodies. Normal:
// of the surface hit, pointing at the body that gets the impact. No Feedback data (that is for
// interaction with other solvers).
void
recordImpacts(Box3DSharedState &st, SIM_ObjectArray &objects, const SIM_Time &now)
{
    // `fromOther`: cp was read from the other body's side.
    struct Hit { const Box3DBodyMirror *self, *other; b3dop::ContactPoint cp; bool fromOther; };
    std::unordered_map<int, std::vector<Hit>> hitsOf;

    bool any = false;
    for (const auto &kv : st.objectSync)
        any = any || (kv.second.visited == st.syncStamp && kv.second.addImpact);
    if (any)
    {
        b3dop::Box3DWorld &world = st.world;
        const bool speedFilter = world.settings().impactMinSpeed > 0.0f;
        if (speedFilter)
            world.updateHitEvents();
        for (const b3dop::BodyMove &mv : st.moves)
        {
            const Box3DBodyMirror *e = static_cast<const Box3DBodyMirror *>(mv.userData);
            if (!e || !e->handle || !st.isLive(*e) || !world.isAwakeByHandle(e->handle))
                continue;
            const bool selfWants = e->sync->addImpact;
            world.getContactsByHandle(e->handle, st.contacts);
            for (const b3dop::ContactPoint &cp : st.contacts)
            {
                const Box3DBodyMirror *o = static_cast<const Box3DBodyMirror *>(cp.otherUserData);
                // The other side, unless it is awake (it records its own) or an asleep dynamic body.
                const bool otherWants = o && st.isLive(*o) && o->sync->addImpact && !o->dynamic() &&
                                        !world.isAwakeByHandle(cp.other);
                if ((!selfWants && !otherWants) || (speedFilter && !world.hadHitEvent(e->handle, cp.other)))
                    continue;
                if (selfWants)
                    hitsOf[e->objectId].push_back({ e, o, cp, false });
                if (otherWants)
                    hitsOf[o->objectId].push_back({ o, e, cp, true });
            }
        }
    }

    std::vector<int> wrote;
    for (const auto &kv : hitsOf)
    {
        SIM_Object *obj = findObjectById(objects, kv.first);
        if (!obj)
            continue;
        SIM_Impacts *imp = SIM_DATA_CREATE(*obj, SIM_IMPACTS_DATANAME, SIM_Impacts, 0);
        if (!imp)
            continue;
        wrote.push_back(kv.first);
        for (const Hit &h : kv.second)
        {
            // cp.normal points from the body it was read from into the other one.
            const b3dop::ContactPoint &cp = h.cp;
            const float sgn = h.fromOther ? 1.0f : -1.0f;
            imp->addPointImpact(UT_Vector3(float(cp.pos[0]), float(cp.pos[1]), float(cp.pos[2])),
                                UT_Vector3(sgn * cp.normal[0], sgn * cp.normal[1], sgn * cp.normal[2]),
                                cp.impulse, int(h.self->pointIndex),
                                h.other ? h.other->objectId : -1, h.other ? int(h.other->pointIndex) : -1,
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

// An inactive body hit with at least min_activation_impulse turns dynamic now and moves from the next step (Box3D reports impulses
// after the step; Bullet can activate within it). Its `active` is set to 1.
void
activateByImpact(Box3DSharedState &st)
{
    for (auto &kv : st.objectSync)
    {
        const Box3DObjectSync &os = kv.second;
        if (os.visited != st.syncStamp)
            continue;
        for (Box3DBodyMirror *e : os.activation)
        {
            if (!e->handle || e->dynamic() || e->props.minActivationImpulse <= 0.0f)
                continue;
            st.world.getContactsByHandle(e->handle, st.contacts);
            bool hit = false;
            for (const b3dop::ContactPoint &cp : st.contacts)
            {
                if (cp.impulse < e->props.minActivationImpulse)
                    continue;
                hit = true;
                break;
            }
            if (!hit)
                continue;
            st.world.setTypeByHandle(e->handle, b3dop::BodySettings::Type::Dynamic);
            e->type     = b3dop::BodySettings::Type::Dynamic;
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
            std::vector<GA_Offset> offs;
            offs.reserve(size_t(gdp->getNumPoints()));
            for (GA_Iterator it(gdp->getPointRange()); !it.atEnd(); ++it)
                offs.push_back(*it);
            // Shared copy-on-write pages would race: harden what the parallel pass writes.
            const bool killing = life.isValid() && dead.isValid();
            if (offs.size() >= kMinParallelPoints)
            {
                age.getAttribute()->hardenAllPages();
                if (killing)
                    dead.getAttribute()->hardenAllPages();
            }
            std::atomic<bool> anyDead{ false };
            parallelForEach(offs.size(), [&](size_t i) {
                const GA_Offset off = offs[i];
                const float a = age.get(off) + dt;
                age.set(off, a);
                if (killing)
                {
                    const float l = life.get(off);
                    if (l > 0.0f && a > l && dead.get(off) == 0)
                    {
                        dead.set(off, 1);
                        anyDead.store(true, std::memory_order_relaxed);
                    }
                }
            }, kMinParallelPoints, 1024);
            markedDead = anyDead.load();
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

// Collects the SIM_Force data an object carries (Gravity Force, Wind Force, Fan Force, POP
// Force, any custom force DOP): the lookup Houdini's own solvers use (see SIM_SolverHair).
class ForceCollector : public SIM_EachDataCallback
{
public:
    explicit ForceCollector(std::vector<const SIM_Force *> &out) : myOut(out) {}

    void callbackConst(const SIM_Data *data, const char *) override
    {
        if (const SIM_Force *f = SIM_DATA_CASTCONST(data, SIM_Force))
            myOut.push_back(f);
    }

private:
    std::vector<const SIM_Force *> &myOut;
};

struct ForceStats
{
    int bodies    = 0;      // bodies that got a force or torque
    int nonFinite = 0;      // bodies whose force DOPs returned NaN / inf (those are skipped)
};

// The data ids of an object's `force` / `torque` attributes (kNoId when absent).
void
forceAttribIds(const GU_Detail *gdp, GA_DataId &forceId, GA_DataId &torqueId)
{
    const GA_Attribute *f = gdp->findPointAttribute("force");
    const GA_Attribute *t = gdp->findPointAttribute("torque");
    forceId  = f ? f->getDataId() : Box3DSyncFingerprint::kNoId;
    torqueId = t ? t->getDataId() : Box3DSyncFingerprint::kNoId;
}

// Force DOPs: every dynamic body takes the sum of the SIM_Force data on its object, evaluated at
// its centre of mass with its mass and velocities, the way Houdini's solvers do, plus its point's
// `force` / `torque` attributes. The force and torque go into Box3D for the coming step (it
// integrates them in every substep and clears them afterwards), so this runs after the sync and
// right before the step. A Gravity Force DOP adds to the solver's own Gravity: set that to 0 when
// using one. Only the bodies of objects that have forces are visited (Box3DObjectSync::bodies).
ForceStats
applyExternalForces(Box3DSharedState &st, SIM_ObjectArray &objects, bool wake)
{
    ForceStats stats;
    b3dop::Box3DWorld &world = st.world;

    // A point's non-zero `force` / `torque` (what POP Force and other POP / SOP nodes write).
    struct PointForce
    {
        GA_Offset  off;
        UT_Vector3 force, torque;
    };
    std::vector<const SIM_Force *> forces;
    std::vector<PointForce>        pointForces;    // sorted by offset
    std::vector<SIM_Object *>      cleared;

    for (exint i = 0; i < objects.entries(); ++i)
    {
        SIM_Object *obj = objects(i);
        const auto osIt = st.objectSync.find(obj->getObjectId());
        if (osIt == st.objectSync.end() || osIt->second.visited != st.syncStamp || osIt->second.bodies.empty())
            continue;
        Box3DObjectSync &os = osIt->second;

        forces.clear();
        ForceCollector collector(forces);
        obj->forEachConstSubData(collector, SIM_DataFilterByType("SIM_Force"), SIM_FORCES_DATANAME,
                                 SIM_DataFilterNone());

        // Point forces. We clear them after applying them; while their data ids are still the ones
        // we recorded then, nothing has written new ones and the read is skipped.
        pointForces.clear();
        if (const SIM_Geometry *geo = SIM_DATA_GETCONST(*obj, SIM_GEOMETRY_DATANAME, SIM_Geometry))
        {
            SIM_GeometryAutoReadLock lock(geo);
            if (const GU_Detail *gdp = lock.getGdp())
            {
                GA_DataId fId, tId;
                forceAttribIds(gdp, fId, tId);
                const bool any = fId != Box3DSyncFingerprint::kNoId || tId != Box3DSyncFingerprint::kNoId;
                if (any && !(fId == os.forceId && tId == os.torqueId))
                {
                    const GA_ROHandleV3 fh(gdp, GA_ATTRIB_POINT, "force");
                    const GA_ROHandleV3 th(gdp, GA_ATTRIB_POINT, "torque");
                    const UT_Vector3    zero(0.0f, 0.0f, 0.0f);
                    auto nonZero = [](const UT_Vector3 &v) { return v.x() != 0.0f || v.y() != 0.0f || v.z() != 0.0f; };
                    for (GA_Iterator it(gdp->getPointRange()); !it.atEnd(); ++it)
                    {
                        const UT_Vector3 f = fh.isValid() ? fh.get(*it) : zero;
                        const UT_Vector3 t = th.isValid() ? th.get(*it) : zero;
                        if (nonZero(f) || nonZero(t))
                            pointForces.push_back({ *it, f, t });
                    }
                    if (pointForces.empty())        // all zero: no need to look again until they change
                    {
                        os.forceId  = fId;
                        os.torqueId = tId;
                    }
                }
            }
        }
        if (forces.empty() && pointForces.empty())
            continue;

        for (Box3DBodyMirror *e : os.bodies)
        {
            if (!e->handle || !e->dynamic() || e->props.ignored)
                continue;
            if (!wake && !world.isAwakeByHandle(e->handle))
                continue;                       // Box3D would ignore the force anyway

            UT_Vector3 force(0.0f, 0.0f, 0.0f);
            UT_Vector3 torque(0.0f, 0.0f, 0.0f);
            if (!pointForces.empty())
            {
                const GA_Offset po = GA_Offset(e->pointOff);
                const auto it = std::lower_bound(pointForces.begin(), pointForces.end(), po,
                                                 [](const PointForce &pf, GA_Offset o) { return pf.off < o; });
                if (it != pointForces.end() && it->off == po)
                {
                    force  += it->force;
                    torque += it->torque;
                }
            }
            if (!forces.empty())
            {
                const float mass = world.massByHandle(e->handle);
                double com[3];
                float  lin[3], ang[3];
                if (mass > 0.0f && std::isfinite(mass) && world.getCenterOfMassByHandle(e->handle, com) &&
                    world.getVelocityByHandle(e->handle, lin, ang))
                {
                    const UT_Vector3 pos(static_cast<float>(com[0]), static_cast<float>(com[1]), static_cast<float>(com[2]));
                    const UT_Vector3 vel(lin[0], lin[1], lin[2]);
                    const UT_Vector3 angvel(ang[0], ang[1], ang[2]);
                    for (const SIM_Force *sf : forces)
                    {
                        UT_Vector3 f(0.0f, 0.0f, 0.0f), t(0.0f, 0.0f, 0.0f);
                        sf->getForce(*obj, pos, vel, angvel, mass, f, t);
                        force  += f;
                        torque += t;
                    }
                }
            }

            const float fv[3] = { float(force.x()),  float(force.y()),  float(force.z()) };
            const float tv[3] = { float(torque.x()), float(torque.y()), float(torque.z()) };
            if (!b3dop::finite3(fv) || !b3dop::finite3(tv))
            {
                ++stats.nonFinite;
                continue;
            }
            if (fv[0] == 0.0f && fv[1] == 0.0f && fv[2] == 0.0f && tv[0] == 0.0f && tv[1] == 0.0f && tv[2] == 0.0f)
                continue;
            world.applyForceByHandle(e->handle, fv, tv, wake);
            ++stats.bodies;
        }

        // Clear what was read (every body, applied or not), or a POP Force that only adds to
        // `force` would pile up from step to step.
        if (pointForces.empty())
            continue;
        SIM_GeometryCopy *geo = getOrMakeWritableGeometry(*obj);
        if (!geo)
            continue;
        {
            SIM_GeometryAutoWriteLock lock(geo);
            GU_Detail *gdp = &lock.getGdp();
            GA_RWHandleV3 fh(gdp, GA_ATTRIB_POINT, "force");
            GA_RWHandleV3 th(gdp, GA_ATTRIB_POINT, "torque");
            const UT_Vector3 zero(0.0f, 0.0f, 0.0f);
            for (const PointForce &pf : pointForces)
            {
                if (fh.isValid()) fh.set(pf.off, zero);
                if (th.isValid()) th.set(pf.off, zero);
            }
            if (fh.isValid()) fh.getAttribute()->bumpDataId();
            if (th.isValid()) th.getAttribute()->bumpDataId();
        }
        cleared.push_back(obj);
    }

    // With the locks released (releasing may bump ids): the cleared ids, and the fingerprints.
    for (SIM_Object *obj : cleared)
    {
        recordFingerprintAfterWrite(st, *obj);
        const SIM_Geometry *geo = SIM_DATA_GETCONST(*obj, SIM_GEOMETRY_DATANAME, SIM_Geometry);
        const auto osIt = st.objectSync.find(obj->getObjectId());
        if (!geo || osIt == st.objectSync.end())
            continue;
        SIM_GeometryAutoReadLock lock(geo);
        if (const GU_Detail *gdp = lock.getGdp())
            forceAttribIds(gdp, osIt->second.forceId, osIt->second.torqueId);
    }
    return stats;
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

    static PRM_Name    theUseForcesName("useforces", "Use Force DOPs");
    static PRM_Default theUseForcesDefault(1);
    static PRM_Name    theForcesWakeName("forceswake", "Forces Wake Bodies");
    static PRM_Default theForcesWakeDefault(0);
    // Greyed out while Use Force DOPs is off.
    static PRM_Conditional theForcesWakeDisable("{ useforces == 0 }", PRM_CONDTYPE_DISABLE);

    static PRM_Name    theSubStepsName("substeps", "Substeps");
    static PRM_Default theSubStepsDefault(4);
    // Minimum 1 (Box3D clamps to it); larger than 16 can be typed.
    static PRM_Range   theSubStepsRange(PRM_RANGE_RESTRICTED, 1, PRM_RANGE_UI, 16);

    static PRM_Name    theAutoThreadsName("autothreads", "Use All Threads");
    static PRM_Default theAutoThreadsDefault(0);

    static PRM_Name    theThreadsName("threads", "Threads");
    static PRM_Default theThreadsDefault(b3dop::kDefaultWorkerCount);
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
    static PRM_Name    theRecycleName("contactrecycledistance", "Contact Recycle Distance");
    static PRM_Default theRecycleDefault(0.05);
    static PRM_Range   theRecycleRange(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 0.2);
    static PRM_Name    theMaxSpeedName("maxlinearspeed", "Max Linear Speed");
    static PRM_Default theMaxSpeedDefault(400.0);
    static PRM_Range   theMaxSpeedRange(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 1000);
    static PRM_Name    theContinuousName("enablecontinuous", "Continuous Collision");
    static PRM_Default theContinuousDefault(1);
    static PRM_Name    theSpeculativeName("enablespeculative", "Speculative Contacts");
    static PRM_Default theSpeculativeDefault(1);

    static PRM_Name    theRestIterName("restitutioniterations", "Bounce Iterations");
    static PRM_Default theRestIterDefault(2);
    static PRM_Range   theRestIterRange(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 10);
    static PRM_Name    theRestPropName("restitutionpropagation", "Bounce Propagation");
    static PRM_Default theRestPropDefault(0);
    static PRM_Name    theIsoName("isotropicinertia", "Isotropic Inertia Tolerance");
    static PRM_Default theIsoDefault(0.2);
    static PRM_Range   theIsoRange(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_RESTRICTED, 1);
    static PRM_Name    theWarmName("enablewarmstarting", "Warm Starting");
    static PRM_Default theWarmDefault(1);
    static PRM_Name    theImpactSpeedName("impactminspeed", "Impact Min Speed");
    static PRM_Default theImpactSpeedDefault(0.0);
    static PRM_Range   theImpactSpeedRange(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 10);
    static PRM_Name    theFrictionMixName("frictionmixing", "Friction Mixing");
    static PRM_Name    theFrictionMixChoices[] = {
        PRM_Name("0", "Geometric Mean"), PRM_Name("1", "Minimum"), PRM_Name("2", "Maximum"),
        PRM_Name("3", "Average"), PRM_Name("4", "Multiply"), PRM_Name(0)
    };
    static PRM_ChoiceList theFrictionMixMenu(PRM_CHOICELIST_SINGLE, theFrictionMixChoices);
    static PRM_Default theFrictionMixDefault(0);
    static PRM_Name    theRestMixName("restitutionmixing", "Bounce Mixing");
    static PRM_Name    theRestMixChoices[] = {
        PRM_Name("0", "Maximum"), PRM_Name("1", "Minimum"), PRM_Name("2", "Average"),
        PRM_Name("3", "Multiply"), PRM_Name("4", "Geometric Mean"), PRM_Name(0)
    };
    static PRM_ChoiceList theRestMixMenu(PRM_CHOICELIST_SINGLE, theRestMixChoices);
    static PRM_Default theRestMixDefault(0);

    // Tabs: each PRM_Default is (number of parameters that follow, tab label).
    static PRM_Name    theTabsName("b3dtabs");
    static PRM_Default theTabsDefaults[] = {
        PRM_Default(7, "Simulation"),
        PRM_Default(4, "Performance"),
        PRM_Default(5, "Contacts"),
        PRM_Default(5, "Bounce & Friction"),
        PRM_Default(3, "Advanced")
    };

    static PRM_Template theTemplates[] = {
        PRM_Template(PRM_SWITCHER, int(sizeof(theTabsDefaults) / sizeof(PRM_Default)), &theTabsName, theTabsDefaults),
        // --- Simulation ---
        PRM_Template(PRM_FLT_J,  3, &theGravityName,  theGravityDefault, 0, 0, 0, 0, 1,
                     "Acceleration applied to every dynamic body, in units per second squared. "
                     "Scale it per body with Gravity Scale on Box3D Data."),
        PRM_Template(PRM_TOGGLE, 1, &theUseForcesName, &theUseForcesDefault, 0, 0, 0, 0, 1,
                     "Apply Force DOPs connected to the objects (Gravity, Wind, Fan, POP Force). Set Gravity above to 0 when using a Gravity Force."),
        PRM_Template(PRM_TOGGLE, 1, &theForcesWakeName, &theForcesWakeDefault, 0, 0, 0, 0, 1,
                     "Let forces wake sleeping bodies. Turn on for wind or fans that should blow resting pieces away.",
                     &theForcesWakeDisable),
        PRM_Template(PRM_INT_J,  1, &theSubStepsName, &theSubStepsDefault, 0, &theSubStepsRange, 0, 0, 1,
                     "Solver passes per time step. More substeps make stacks and fast collisions more stable "
                     "but take longer."),
        PRM_Template(PRM_FLT_J,  1, &theTimeScaleName, &theTimeScaleDefault, 0, &theTimeScaleRange, 0, 0, 1,
                     "Multiplies the time step: below 1 is slow motion, above 1 fast motion."),
        PRM_Template(PRM_TOGGLE, 1, &theAgingName,    &theAgingDefault, 0, 0, 0, 0, 1,
                     "Add the time step to each point's age, and mark points dead once age exceeds life."),
        PRM_Template(PRM_FLT_J,  1, &theMaxSpeedName, &theMaxSpeedDefault, 0, &theMaxSpeedRange, 0, 0, 1,
                     "No body moves faster than this."),
        // --- Performance ---
        PRM_Template(PRM_TOGGLE, 1, &theAutoThreadsName, &theAutoThreadsDefault, 0, 0, 0, 0, 1,
                     "Use one Box3D worker thread per CPU core (up to 32). Usually only faster for very large scenes."),
        PRM_Template(PRM_INT_J,  1, &theThreadsName,  &theThreadsDefault, 0, &theThreadsRange,
                     0, 0, 1,
                     "Number of Box3D worker threads. More helps large scenes; small ones can run faster with fewer.",
                     &theThreadsDisable),
        PRM_Template(PRM_TOGGLE, 1, &theSleepName,    &theSleepDefault, 0, 0, 0, 0, 1,
                     "Let resting bodies stop simulating until touched. Off keeps every body awake."),
        PRM_Template(PRM_FLT_J,  1, &theRecycleName, &theRecycleDefault, 0, &theRecycleRange, 0, 0, 1,
                     "Reuse contacts for pairs that barely moved, which is faster but slightly less exact. 0 turns it off for every body."),
        // --- Contacts ---
        PRM_Template(PRM_FLT_J,  1, &theStiffnessName, &theStiffnessDefault, 0, &theStiffnessRange, 0, 0, 1,
                     "How stiff contacts are (cycles per second). Higher values recover from overlap faster but can jitter."),
        PRM_Template(PRM_FLT_J,  1, &theDampingName, &theDampingDefault, 0, &theDampingRange, 0, 0, 1,
                     "How much contacts damp overlap recovery. Lower values recover faster but more energetically."),
        PRM_Template(PRM_FLT_J,  1, &thePushSpeedName, &thePushSpeedDefault, 0, &thePushSpeedRange, 0, 0, 1,
                     "How fast overlapping pieces are pushed apart. Lower it if pieces that start out overlapping fly apart."),
        PRM_Template(PRM_TOGGLE, 1, &theContinuousName, &theContinuousDefault, 0, 0, 0, 0, 1,
                     "Stop fast bodies passing through static ones (and, for Fast Object bodies, moving ones)."),
        PRM_Template(PRM_TOGGLE, 1, &theSpeculativeName, &theSpeculativeDefault, 0, 0, 0, 0, 1,
                     "Create contacts just before shapes touch. Leave on unless you see bodies catch on invisible edges."),
        // --- Bounce & Friction ---
        PRM_Template(PRM_FLT_J,  1, &theBounceThresholdName, &theBounceThresholdDefault, 0, &theBounceThresholdRange, 0, 0, 1,
                     "Collisions slower than this speed don't bounce, so resting pieces settle instead of jittering."),
        PRM_Template(PRM_INT_J,  1, &theRestIterName, &theRestIterDefault, 0, &theRestIterRange, 0, 0, 1,
                     "Extra solver passes that compute bounce. More passes give better bounce on stacked pieces."),
        PRM_Template(PRM_TOGGLE, 1, &theRestPropName, &theRestPropDefault, 0, 0, 0, 0, 1,
                     "Pass bounce on through stacks of touching pieces."),
        PRM_Template(PRM_ORD,    1, &theFrictionMixName, &theFrictionMixDefault, &theFrictionMixMenu, 0, 0, 0, 1,
                     "How the friction of two touching bodies is combined."),
        PRM_Template(PRM_ORD,    1, &theRestMixName, &theRestMixDefault, &theRestMixMenu, 0, 0, 0, 1,
                     "How the bounce of two touching bodies is combined."),
        // --- Advanced ---
        PRM_Template(PRM_FLT_J,  1, &theIsoName, &theIsoDefault, 0, &theIsoRange, 0, 0, 1,
                     "Stops nearly spherical bodies spinning oddly. 0 is a sphere, 0.5 a 1x1x2 box; only affects new bodies or mass changes."),
        PRM_Template(PRM_TOGGLE, 1, &theWarmName, &theWarmDefault, 0, 0, 0, 0, 1,
                     "Advanced: reuse last step's contact forces. Leave on; turning it off makes stacks less stable."),
        PRM_Template(PRM_FLT_J,  1, &theImpactSpeedName, &theImpactSpeedDefault, 0, &theImpactSpeedRange, 0, 0, 1,
                     "Impact Data only includes pairs that collide faster than this. 0 includes every contact."),
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
    s.contactStiffness = float(getContactStiffness());
    s.contactDampingRatio  = float(getContactDamping());
    s.restitutionThreshold = float(getBounceThreshold());
    s.contactRecycleDistance = float(getContactRecycleDistance());
    s.maxLinearSpeed   = float(getMaxLinearSpeed());
    s.enableContinuous = getEnableContinuous();
    s.enableSpeculative = getEnableSpeculative();
    s.restitutionIterations  = int(getRestitutionIterations());
    s.restitutionPropagation = getRestitutionPropagation();
    s.isotropicInertiaTolerance = float(getIsotropicInertia());
    s.warmStarting     = getEnableWarmStarting();
    s.impactMinSpeed   = float(getImpactMinSpeed());
    s.frictionMixing   = int(getFrictionMixing());
    s.restitutionMixing = int(getRestitutionMixing());
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

    // Force DOPs go into the bodies now, for this step only (Box3D clears them afterwards).
    if (getUseForces())
    {
        const ForceStats fs = applyExternalForces(*st, objects, getForcesWake());
        if (fs.nonFinite)
            addError(objects(0), SIM_MESSAGE,
                     ("Box3D: " + std::to_string(fs.nonFinite) + " body(ies) got a non-finite force or torque "
                      "from a Force DOP; it was ignored").c_str(), UT_ERROR_WARNING);
    }

    // From here the step runs to the end (the write-back is quick), so the cached result is
    // always a whole step.
    st->world.step(st->stepDt);
    st->world.getMoveEvents(st->moves);     // read once: the write-back and impacts use them
    B3D_LOG_VERBOSE("  step: %d contact(s) recycled", st->world.recycledContactCount());
    writeResultsBack(*worldData, objects);
    if (const int broke = jointsAfterStep(*st, objects))
        B3D_LOG("  joints: %d broke", broke);
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
    st->wantNames = anyJointNetwork(objects);
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

    // Static colliders from affector objects (Static Object DOP etc. with a collision relationship).
    Box3DAffectorCounts affectorCounts;
    syncAffectorBodies(*st, objects, affectorCounts);
    if (affectorCounts.failed || affectorCounts.noGeo)
        addError(objects(0), SIM_MESSAGE,
                 ("Box3D: " + std::to_string(affectorCounts.failed + affectorCounts.noGeo) +
                  " affector object(s) have no usable collision geometry (they need polygons) and are ignored")
                     .c_str(),
                 UT_ERROR_WARNING);
    if (affectorCounts.added || affectorCounts.removed || affectorCounts.moved)
        B3D_LOG("  affectors: %d, +%d built, %d moved, -%d removed, %d baked", affectorCounts.total,
                affectorCounts.added, affectorCounts.moved, affectorCounts.removed, affectorCounts.baked);

    // Remove bodies whose point (or object) is gone: the unstamped entries.
    int removed = 0;
    if (st->stampedCount != st->mirror.size())
    {
        std::unordered_set<int> touched;        // objects that lost bodies
        for (auto it = st->mirror.begin(); it != st->mirror.end();)
        {
            if (!st->isLive(it->second))
            {
                touched.insert(it->second.objectId);
                st->world.destroyBody(it->second.handle);
                it = st->mirror.erase(it);
                ++removed;
            }
            else
                ++it;
        }
        // Those objects' slots may cache pointers to erased entries; other objects keep theirs.
        for (const int id : touched)
        {
            const auto sl = st->slots.find(id);
            if (sl != st->slots.end())
                for (Box3DPointSlot &slot : sl->second)
                    slot.entry = nullptr;
        }
    }

    // Objects not seen lose their records (their bodies were just removed).
    for (auto it = st->objectSync.begin(); it != st->objectSync.end();)
        it = (it->second.visited == st->syncStamp) ? std::next(it) : st->objectSync.erase(it);

    // Joints, once the bodies they name are current. A changed set of bodies re-reads the networks.
    if (counts.added || removed || affectorCounts.added || affectorCounts.removed)
        ++st->bodyVersion;
    Box3DJointCounts jointCounts;
    syncJoints(*st, objects, jointCounts);
    {
        auto warnJoints = [&](int n, const char *what) {
            if (n)
                addError(objects(0), SIM_MESSAGE, ("Box3D: " + std::to_string(n) + " joint primitive(s) " + what).c_str(),
                         UT_ERROR_WARNING);
        };
        warnJoints(jointCounts.unknown, "have a missing or unknown joint_type and are ignored");
        warnJoints(jointCounts.unresolved, "name a piece or object that does not exist and are ignored");
        warnJoints(jointCounts.notLines, "are not two-point lines and are ignored");
        warnJoints(jointCounts.failed, "could not be made (both ends on the world, or the same body twice)");
        if (jointCounts.shared)
            addError(objects(0), SIM_MESSAGE,
                     "Box3D: a joint network is attached to several objects; it is used once, and feedback and "
                     "broken joints are not written back to it",
                     UT_ERROR_WARNING);
        if (jointCounts.created || jointCounts.removed || jointCounts.updated)
            B3D_LOG("  joints: %d network(s), +%d made, %d updated, -%d removed (joints now %d)", jointCounts.networks,
                    jointCounts.created, jointCounts.updated, jointCounts.removed, st->world.jointCount());
    }

    // The shapes are made: let go of the hulls this sync held (the shared cache keeps them).
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
        B3D_LOG_IF(event, "  sync: +%d added, -%d removed, %d edited externally, %d type changes, "
                "%d shapes rebuilt, %d settings changed, "
                "%d failed, %d duplicate ids, %d non-finite P  (bodies now %d)",
                counts.added, removed, counts.edited, counts.retyped, counts.rebuilt, counts.updated,
                counts.failed, counts.duplicates, counts.badP, st->world.bodyCount());
    }
    return true;
}

void
SIM_SolverBox3D::writeResultsBack(SIM_Box3DWorld &worldData, SIM_ObjectArray &objects)
{
    Box3DSharedState *st = worldData.state();
    b3dop::Box3DWorld &world = st->world;

    // ---- pass 0 (serial): the bodies that moved -----------------------------
    // Box3D's move events (read into st->moves right after the step): every awake body and its
    // transform (a body falling asleep appears once more). Each body's user data is its entry.
    const int moveCount = int(st->moves.size());
    if (moveCount == 0)
        return;

    std::vector<Box3DWriteItem> &items = st->writeItems;
    items.clear();
    items.reserve(size_t(moveCount));
    std::vector<Box3DPackedWrite> &packedWrites = st->packedWrites;
    packedWrites.clear();
    int missing = 0;
    for (int mi = 0; mi < moveCount; ++mi)
    {
        const b3dop::BodyMove &mv = st->moves[size_t(mi)];
        if (!mv.userData)
            continue;
        Box3DBodyMirror *entry = static_cast<Box3DBodyMirror *>(mv.userData);
        if (!entry->dynamic() || !st->isLive(*entry))
            continue;                       // only dynamic bodies seen by this step's sync are written
        if (!entry->handle)
        {
            ++missing;
            continue;
        }
        items.emplace_back();
        Box3DWriteItem &w = items.back();
        w.entry    = entry;
        w.move     = mi;
        w.objectId = entry->objectId;
        if (entry->packed)
        {
            w.packedIdx = int(packedWrites.size());
            packedWrites.emplace_back();
        }
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
        world.getVelocityByHandle(w.entry->handle, w.lin, w.ang);
    }, kMinParallelWrite);

    // Grouped by object, so each geometry is locked once.
    std::vector<SIM_Object *> refresh;      // objects written to
    forEachObjectRun(items, [&](size_t begin, size_t end) {
        SIM_Object *obj = findObjectById(objects, items[begin].objectId);
        if (!obj)
            return;

        SIM_GeometryCopy *geo = getOrMakeWritableGeometry(*obj);
        if (!geo)
        {
            B3D_LOG_VERBOSE("  writeBack: object id=%d Geometry is not writable and could not be converted",
                    obj->getObjectId());
            addError(obj, SIM_MESSAGE,
                     "Box3D: this object's Geometry is not writable and could not be converted to a SIM_GeometryCopy; results are not written back",
                     UT_ERROR_WARNING);
            return;
        }

        SIM_GeometryAutoWriteLock lock(geo);
        GU_Detail *gdp = &lock.getGdp();
        if (!gdp)
            return;
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
            Box3DWriteItem &w = items[begin + j];
            if (w.off == GA_INVALID_OFFSET)
                return;
            const GA_Offset        off = w.off;
            const b3dop::BodyMove &mv  = st->moves[size_t(w.move)];

            if (pOut.isValid()) pOut.set(off, UT_Vector3(float(mv.pos[0]), float(mv.pos[1]), float(mv.pos[2])));
            if (qOut.isValid()) qOut.set(off, UT_QuaternionF(mv.rot[0], mv.rot[1], mv.rot[2], mv.rot[3]));
            if (vOut.isValid()) vOut.set(off, UT_Vector3(w.lin[0], w.lin[1], w.lin[2]));
            if (wOut.isValid()) wOut.set(off, UT_Vector3(w.ang[0], w.ang[1], w.ang[2]));
            if (anySleepWrite && w.sleepState >= 0) sleepOut.set(off, int(w.sleepState));

            // Packed: the new 3x3 (the scale the sync read, the new rotation) is computed here and
            // written in pass 3, to the primitive the last sync stored.
            Box3DBodyMirror  &me     = *w.entry;
            Box3DPackedWrite *pw     = w.packedIdx >= 0 ? &packedWrites[size_t(w.packedIdx)] : nullptr;
            const bool        packed = anyPrims && me.packed && pw;
            if (packed)
            {
                composePackedMatrix(mv.rot, me.rowScale, pw->newM3);
                pw->primOff = GA_Offset(me.primOff);
                pw->has     = true;
                std::copy(pw->newM3, pw->newM3 + 9, me.m3);     // a skipped sync does not refresh it
            }

            // What the read path will see, so our own write does not look like an external edit:
            // P through float; the rotation as written (poseDiffers allows for the float noise of
            // a packed transform's round trip and for q vs -q). Non-finite P: Box3D's values.
            Box3DBodyPose &mirror = me;
            const bool hasRot = packed || qOut.isValid();       // else the read path sees identity
            const float identity[4] = { 0, 0, 0, 1 };
            std::copy(hasRot ? mv.rot : identity, (hasRot ? mv.rot : identity) + 4, mirror.rot);
            b3dop::sanitizeQuat(mirror.rot);
            for (int i = 0; i < 3; ++i)
            {
                mirror.lin[i] = vOut.isValid() ? w.lin[i] : 0.0f;
                mirror.ang[i] = wOut.isValid() ? w.ang[i] : 0.0f;
            }
            if (!b3dop::storedPosition(mv.pos, mirror.pos))
                std::copy(mv.pos, mv.pos + 3, mirror.pos);

#ifdef BOX3DDOP_DEBUG
            if (verifyReadback)
            {
                // BOX3DDOP_VERIFY_READBACK=1: read back; a difference poseDiffers sees would make
                // the next scan treat our own write as an external edit.
                Box3DBodyPose stored;
                PackedInfo pk;
                if (packed)
                {
                    pk.prim = static_cast<const GU_PrimPacked *>(gdp->getPrimitive(pw->primOff));
                    double sc[3];
                    splitPackedMatrix(pw->newM3, pk.rot, sc);
                }
                if (readPointState(gdp, a, off, pk, stored) && poseDiffers(stored, mirror))
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
    });

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
