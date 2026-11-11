// SIM_SolverBox3D.cpp
#include "SIM_SolverBox3D.h"
#include "SIM_Box3DWorld.h"
#include "SIM_Box3DBody.h"
#include "Box3DDebug.h"

#include <GA/GA_Handle.h>
#include <GA/GA_Iterator.h>
#include <GA/GA_Primitive.h>
#include <GU/GU_Detail.h>
#include <GU/GU_DetailHandle.h>
#include <GU/GU_PackedFragment.h>
#include <GU/GU_PackedImpl.h>
#include <GU/GU_PrimPacked.h>
#include <PRM/PRM_Include.h>
#include <SIM/SIM_CacheOptions.h>
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
#include <UT/UT_Vector3.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
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

// Name the per-object default settings (SIM_Box3DBody) and the world are
// attached under.
const char *const kWorldDataName = "Box3DWorld";
const char *const kBodyDataName  = "Box3DBody";

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
    GA_ROHandleF  pscale;      // float  uniform size (default 1)
    GA_ROHandleV3 scale;       // vector non-uniform size (default 1 1 1)
    GA_ROHandleV3 v;           // vector linear velocity
    GA_ROHandleV3 w;           // vector angular velocity, radians / second
    GA_ROHandleQ  orient;      // quat   rotation
    GA_ROHandleS  shape;       // string "box", "sphere"

    explicit PointAttrs(const GU_Detail *gdp)
        : id(gdp, GA_ATTRIB_POINT, "id"),
          active(gdp, GA_ATTRIB_POINT, "active"),
          mass(gdp, GA_ATTRIB_POINT, "mass"),
          friction(gdp, GA_ATTRIB_POINT, "friction"),
          bounce(gdp, GA_ATTRIB_POINT, "bounce"),
          pscale(gdp, GA_ATTRIB_POINT, "pscale"),
          scale(gdp, GA_ATTRIB_POINT, "scale"),
          v(gdp, GA_ATTRIB_POINT, "v"),
          w(gdp, GA_ATTRIB_POINT, "w"),
          orient(gdp, GA_ATTRIB_POINT, "orient"),
          shape(gdp, GA_ATTRIB_POINT, "shape")
    {
    }
};

bool
isFinite3(const double *p)
{
    return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}

// Box3D asserts on non-unit quaternions. Houdini users hand us zeros (an
// attribute that was created with default 0) or slightly off-unit values.
void
sanitizeQuat(float q[4])
{
    double len2 = 0.0;
    for (int i = 0; i < 4; ++i) len2 += double(q[i]) * double(q[i]);
    if (!(len2 > 1e-12) || !std::isfinite(len2))
    {
        q[0] = q[1] = q[2] = 0.0f;  q[3] = 1.0f;
        return;
    }
    const float inv = float(1.0 / std::sqrt(len2));
    for (int i = 0; i < 4; ++i) q[i] *= inv;
}

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
// Everything that touches GU_PrimPacked is in this section, and the calls
// whose exact HDK signature could not be checked without a Houdini build are
// written so a different signature does not break the build (see the
// *Packed* helpers below); the fallback is logged once and described in the
// README.
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
    bool                 rotationFromPrim = false;     // body rotation = prim's, not `orient`
};

// GU_PrimPacked::isPackedPrimitive takes the primitive by reference in the
// class reference; the pointer form is tried second.
template <class P>
auto isPackedImpl(const P *p, int) -> decltype(GU_PrimPacked::isPackedPrimitive(*p), bool())
{
    return GU_PrimPacked::isPackedPrimitive(*p);
}
template <class P>
auto isPackedImpl(const P *p, long) -> decltype(GU_PrimPacked::isPackedPrimitive(p), bool())
{
    return GU_PrimPacked::isPackedPrimitive(p);
}
bool isPackedPrim(const GA_Primitive *p) { return p && isPackedImpl(p, 0); }

// getFullTransform4 fills a UT_Matrix4D (or F, depending on the build).
template <class P>
auto fetchFull4(const P &p, double out[16], int)
    -> decltype(p.getFullTransform4(std::declval<UT_Matrix4D &>()), bool())
{
    UT_Matrix4D m;
    p.getFullTransform4(m);
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out[r * 4 + c] = double(m(r, c));
    return true;
}
template <class P>
auto fetchFull4(const P &p, double out[16], long)
    -> decltype(p.getFullTransform4(std::declval<UT_Matrix4F &>()), bool())
{
    UT_Matrix4F m;
    p.getFullTransform4(m);
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out[r * 4 + c] = double(m(r, c));
    return true;
}

// Setting the rotation: the packed primitive's local transform is a 3x3.
// If this HDK has no setLocalTransform(UT_Matrix3D), packed rotation cannot be
// written back; the solver then treats packed points like plain ones
// (rotation in `orient`) instead of fighting the primitive every step.
template <class P, class = void>
struct CanSetLocal3 : std::false_type {};
template <class P>
struct CanSetLocal3<P, std::void_t<decltype(std::declval<P &>().setLocalTransform(
                           std::declval<const UT_Matrix3D &>()))>> : std::true_type {};
constexpr bool kPackedRotationWritable = CanSetLocal3<GU_PrimPacked>::value;

template <class P>
void setLocal3(P &p, const double m[9], std::true_type)
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
template <class P>
void setLocal3(P &, const double *, std::false_type) {}

// Below this many points a parallel pass costs more than it saves.
constexpr size_t kMinParallelPoints = 2048;

// Runs f(i) for i in [0, n), threaded on Houdini's TBB pool when n is large.
// f must only read shared data, or write to element i of its own output; the
// serial passes around each call are where the Box3D world, the mirror and the
// `seen` set are touched.
template <class F>
void
parallelForEach(size_t n, const F &f)
{
    if (n < kMinParallelPoints)
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

// Row-vector rotation matrix for unit quaternion q (x, y, z, w), taken from the
// HDK (UT_QuaternionD::getRotationMatrix). The HDK reference does not state the
// matrix convention, so the first call checks the result against the Houdini-
// free b3dop::quatToMatrix3 (which is unit-tested and defined as row-vector).
// If they disagree (say the HDK matrix is transposed) it is logged once and the
// b3dop one is used from then on, so a convention surprise cannot invert the
// rotation written back to the primitive.
void
rotationMatrixFromQuat(const float q[4], double out[9])
{
    double ref[9];
    b3dop::quatToMatrix3(q, ref);

    const double   qx = q[0], qy = q[1], qz = q[2], qw = q[3];
    UT_QuaternionD uq(qx, qy, qz, qw);
    UT_Matrix3D    m;
    uq.getRotationMatrix(m);
    double hdk[9];
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            hdk[r * 3 + c] = m(r, c);

    static std::once_flag once;
    static bool           hdkAgrees = true;
    std::call_once(once, [&] {
        double err = 0.0;
        for (int i = 0; i < 9; ++i)
            err = std::fmax(err, std::fabs(hdk[i] - ref[i]));
        hdkAgrees = err < 1e-5;
        if (!hdkAgrees)
            B3D_LOG("getRotationMatrix disagrees with the row-vector reference (max diff %g): "
                    "using the reference for packed write-back", err);
    });
    for (int i = 0; i < 9; ++i)
        out[i] = hdkAgrees ? hdk[i] : ref[i];
}

// A new local 3x3 for a packed primitive: keep the scale of `oldM3` (per row)
// and replace the rotation with q. For uniform scale this is exact.
void
composeLocal3(const float q[4], const double oldM3[9], double outM3[9])
{
    float  oq[4];
    double sc[3];
    b3dop::matrix3ToQuatScale(oldM3, oq, sc);       // sc stays 1 if the old matrix is degenerate
    double r[9];
    rotationMatrixFromQuat(q, r);
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
            outM3[row * 3 + col] = sc[row] * r[row * 3 + col];
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
    if (!isPackedPrim(prim))
        return pk;

    pk.prim    = static_cast<const GU_PrimPacked *>(prim);
    pk.primOff = po;
    const UT_Vector3 pos = gdp->getPos3(ptoff);
    pk.P[0] = pos.x();  pk.P[1] = pos.y();  pk.P[2] = pos.z();
    if (!fetchFull4(*pk.prim, pk.m4, 0))
    {
        pk.prim = nullptr;          // cannot read the transform: treat as plain
        return pk;
    }
    const double m3[9] = { pk.m4[0], pk.m4[1], pk.m4[2],
                           pk.m4[4], pk.m4[5], pk.m4[6],
                           pk.m4[8], pk.m4[9], pk.m4[10] };
    b3dop::matrix3ToQuatScale(m3, pk.rot, pk.scale);
    pk.rotationFromPrim = kPackedRotationWritable;
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
        PieceGroup           pieces;
    };
    std::unordered_map<const GU_Detail *, InnerEntry> inner;
    // Fragments: (shared source detail, fragment name) -> that fragment's piece.
    std::map<std::pair<const GU_Detail *, std::string>, InnerEntry> fragments;

    // Returns the pieces of a packed primitive's own geometry, or null.
    const PieceGroup *innerPieces(const GU_PrimPacked &prim)
    {
        GU_ConstDetailHandle handle = prim.getPackedDetail();
        GU_DetailHandleAutoReadLock lock(handle);
        const GU_Detail *gdp = gdpPtr(lock.getGdp());
        if (!gdp)
            return nullptr;

        // A packed fragment: the detail is the WHOLE source, shared by every
        // fragment of it, so only this fragment's own primitives are used.
        if (const GU_PackedFragment *frag = asFragment(prim))
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
            return fit->second.pieces.pieces.empty() || fit->second.pieces.pieces[0].xyz.empty()
                       ? nullptr : &fit->second.pieces;
        }

        auto it = inner.find(gdp);
        if (it == inner.end())
        {
            InnerEntry e;
            e.handle = handle;
            collectPieces(gdp, e.pieces);
            it = inner.emplace(gdp, std::move(e)).first;
        }
        return it->second.pieces.pieces.empty() ? nullptr : &it->second.pieces;
    }
};

// ---------------------------------------------------------------------------
// Point state
// ---------------------------------------------------------------------------

// Read P / orient / v / w of one point. Returns false if P is not finite.
// For a packed point the rotation comes from the primitive, not `orient`.
bool
readPointState(const GU_Detail *gdp, const PointAttrs &a, GA_Offset off,
               const PackedInfo &pk, DopBodyState &out)
{
    const UT_Vector3 p = gdp->getPos3(off);
    out.pos[0] = p.x();  out.pos[1] = p.y();  out.pos[2] = p.z();
    if (!isFinite3(out.pos))
        return false;

    out.rot[0] = out.rot[1] = out.rot[2] = 0.0f;  out.rot[3] = 1.0f;
    if (pk.prim && pk.rotationFromPrim)
    {
        for (int i = 0; i < 4; ++i) out.rot[i] = pk.rot[i];
        sanitizeQuat(out.rot);
    }
    else if (a.orient.isValid())
    {
        const UT_QuaternionF q = a.orient.get(off);
        out.rot[0] = q.x();  out.rot[1] = q.y();  out.rot[2] = q.z();  out.rot[3] = q.w();
        sanitizeQuat(out.rot);
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
describeStateDifference(const DopBodyState &a, const DopBodyState &b)
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

// Default settings for all points of an object (the Box3D Body DOP node), or
// built-in defaults if the object has none.
b3dop::BodySettings
nodeSettingsFor(const SIM_Object &obj)
{
    const SIM_Box3DBody *body = SIM_DATA_GETCONST(obj, kBodyDataName, SIM_Box3DBody);
    return body ? body->toBodySettings() : b3dop::BodySettings();
}

// `active` = 0 makes a point static. Otherwise the node's body type applies.
b3dop::BodySettings::Type
bodyTypeFor(const b3dop::BodySettings &node, bool active)
{
    return active ? node.type : b3dop::BodySettings::Type::Static;
}

bool
pointIsActive(const PointAttrs &a, GA_Offset off)
{
    return !a.active.isValid() || a.active.get(off) != 0;
}

// The point's id: the `id` attribute, else the point number.
int64_t
pointId(const GU_Detail *gdp, const PointAttrs &a, GA_Offset off)
{
    return a.id.isValid() ? int64_t(a.id.get(off)) : int64_t(gdp->pointIndex(off));
}

uint64_t
pointKey(const GU_Detail *gdp, const PointAttrs &a, int objectId, GA_Offset off)
{
    return b3dop::makeBodyKey(objectId, pointId(gdp, a, off));
}

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

// Decide what shapes a point's body gets. See README, "Convex and concave".
//
//  box / sphere : plain point  -> unit shape scaled by pscale * scale
//                 packed point -> fitted to the packed geometry's bounds
//  convex       : ONE hull of the packed geometry
//  concave      : one hull PER PIECE of the packed geometry (pieces grouped by
//                 the prim `name` attribute); several shapes on one body
//
// The packed geometry is the collision geometry, whether it is a simplified
// proxy or the full-detail mesh. A convex / concave point that is not packed
// (or whose packed geometry is empty) falls back to a box so the body still
// exists.
BuiltShapes
buildShapes(b3dop::Box3DWorld &world, ShapeSources &src, b3dop::ShapeType type,
            float pscale, const float scale[3], const PackedInfo &pk)
{
    using namespace b3dop;
    BuiltShapes out;

    const bool wantGeometry = shapeNeedsGeometry(type);

    // 1. The source: the packed primitive's own geometry (any shape type on a
    //    packed point). A plain point has none.
    const PieceGroup *group = pk.prim ? src.innerPieces(*pk.prim) : nullptr;

    // 1b. Convex / concave on a packed point: the hull of each instance of one
    //     source is a function of (source, pivot, scale), so look it up by that
    //     key and build the body-local points only on a miss.
    double S[3];
    if (group && wantGeometry && packedScale(pk.m4, S))
    {
        const bool convex = (type == ShapeType::Convex);
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

        if (convex)
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
        if (type == ShapeType::Convex)
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
             const PackedInfo &pk, const DopBodyState &st, bool active,
             std::string *fallbackNote = nullptr)
{
    b3dop::BodySettings b = node;
    b.type = bodyTypeFor(node, active);
    if (a.friction.isValid()) b.friction    = a.friction.get(off);
    if (a.bounce.isValid())   b.restitution = a.bounce.get(off);

    // Shape: `shape` string attribute, else the node's default.
    b3dop::ShapeType type = node.defaultShape;
    if (a.shape.isValid())
        b3dop::parseShapeType(a.shape.get(off), type);   // unknown -> keep default

    // pscale / scale size plain points. A packed point's size is its
    // primitive's transform, so they only matter as the fallback box there.
    const float ps = a.pscale.isValid() ? float(a.pscale.get(off)) : 1.0f;
    float sc[3] = { 1.0f, 1.0f, 1.0f };
    if (a.scale.isValid())
    {
        const UT_Vector3 s = a.scale.get(off);
        sc[0] = s.x();  sc[1] = s.y();  sc[2] = s.z();
    }

    const BuiltShapes shapes = buildShapes(world, src, type, ps, sc, pk);
    (void)bodyId;                   // only used by the debug log
    if (shapes.note)
    {
        B3D_LOG("  body id=%lld: %s", (long long)bodyId, shapes.note);
        if (fallbackNote)
            *fallbackNote = shapes.note;    // surfaced to the user as one warning per kind
    }

    // mass -> density (Box3D computes mass from density * volume; for a
    // compound the volume is the sum of the pieces).
    if (a.mass.isValid())
    {
        const float mass = float(a.mass.get(off));
        if (mass > 0.0f && std::isfinite(mass) && shapes.totalVolume > 0.0f)
            b.density = mass / shapes.totalVolume;
    }

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

// One point's result on its way from Box3D to the geometry (writeResultsBack).
struct WriteItem
{
    GA_Offset     off   = GA_INVALID_OFFSET;
    DopBodyState *entry = nullptr;          // this point's mirror entry
    DopBodyState  s;                        // what Box3D reports now
    DopBodyState  stored;                   // what the geometry holds after the write
    bool          storedOk = false;
    bool          hasM3    = false;         // packed rotation to write in the serial pass
    GA_Offset     primOff  = GA_INVALID_OFFSET;
    double        newM3[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
};

// One point as read by the parallel pass of syncObject().
struct PointSnap
{
    GA_Offset    off    = GA_INVALID_OFFSET;
    int64_t      bid    = 0;
    uint64_t     key    = 0;
    DopBodyState cur;
    bool         ok     = false;    // readPointState succeeded (P is finite)
    bool         active = true;
};

struct SyncCounts
{
    int added = 0, edited = 0, retyped = 0, failed = 0, duplicates = 0, badP = 0;
    std::unordered_map<std::string, int> fallbacks;     // shape fallback note -> bodies that used it
};

// Sync one object's points into the world. Returns true if any of its points
// is a body we will write back (dynamic).
bool
syncObject(Box3DSharedState &st, const SIM_Object &obj,
           std::unordered_set<uint64_t> &seen, SyncCounts &c)
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

    const PointAttrs a(gdp);
    const b3dop::BodySettings node = nodeSettingsFor(obj);
    b3dop::Box3DWorld &world = st.world;
    bool hasDynamic = false;

    ShapeSources sources;       // packed geometry, collected lazily per distinct source

    // ---- pass 1 (parallel): read every point's attributes -----------------
    // Reads only. Offsets are gathered serially first (cheap), so pass 1 can
    // index by position. When there is no `id` attribute the body id is the
    // point number, which equals the position in a full point-range walk; it
    // is filled here so the parallel pass never calls pointIndex().
    std::vector<PointSnap> snaps;
    snaps.reserve(size_t(gdp->getNumPoints()));
    for (GA_Iterator it(gdp->getPointRange()); !it.atEnd(); ++it)
    {
        PointSnap s;
        s.off = *it;
        s.bid = int64_t(snaps.size());
        snaps.push_back(s);
    }

    const int objectId = obj.getObjectId();
    parallelForEach(snaps.size(), [&](size_t i) {
        PointSnap &s = snaps[i];
        if (a.id.isValid())
            s.bid = int64_t(a.id.get(s.off));
        s.key = b3dop::makeBodyKey(objectId, s.bid);
        const PackedInfo pk = getPackedInfo(gdp, s.off);
        s.ok     = readPointState(gdp, a, s.off, pk, s.cur);
        s.active = pointIsActive(a, s.off);
    });

    // ---- pass 2 (serial): apply to Box3D, the mirror and `seen` -------------
    // Same order and same rules as before, so "first duplicate id wins" holds.
    for (PointSnap &s : snaps)
    {
        const GA_Offset off = s.off;
        const int64_t   bid = s.bid;
        const uint64_t  key = s.key;
        DopBodyState   &cur = s.cur;

        if (!seen.insert(key).second)
        {
            ++c.duplicates;         // two points with the same `id`: first wins
            continue;
        }

        if (!s.ok)
        {
            ++c.badP;
            seen.erase(key);        // treated as absent: body (if any) is removed
            continue;
        }

        const bool active = s.active;
        const b3dop::BodySettings::Type wantType = bodyTypeFor(node, active);

        auto mit = st.mirror.find(key);
        if (mit == st.mirror.end())
        {
            PackedInfo pk = getPackedInfo(gdp, off);    // only new bodies need it again
            readPackedPivot(pk);        // the hull is built from it
            std::string fallbackNote;
            if (!addPointBody(world, sources, node, a, off, bid, key, pk, cur, active, &fallbackNote))
            {
                ++c.failed;
                continue;
            }
            if (!fallbackNote.empty())
                ++c.fallbacks[fallbackNote];
            cur.dynamic = (wantType == b3dop::BodySettings::Type::Dynamic);
            cur.type    = wantType;
            cur.handle  = world.bodyHandle(key);
            st.mirror[key] = cur;
            ++c.added;
            hasDynamic = hasDynamic || cur.dynamic;
            continue;
        }

        // Existing body. Did `active` flip?
        if (mit->second.type != wantType)
        {
            world.setBodyType(key, wantType);
            mit->second.type    = wantType;
            mit->second.dynamic = (wantType == b3dop::BodySettings::Type::Dynamic);
            ++c.retyped;
        }

        // Did something else edit P / orient / v / w since we wrote it?
        if (describeStateDifference(cur, mit->second))
        {
            const b3dop::BodyHandle h = mit->second.handle;
            world.setTransformByHandle(h, cur.pos, cur.rot);
            world.setVelocityByHandle(h, cur.lin, cur.ang);
            cur.dynamic = mit->second.dynamic;
            cur.type    = mit->second.type;
            cur.handle  = h;
            mit->second = cur;
            ++c.edited;
        }
        hasDynamic = hasDynamic || mit->second.dynamic;
    }
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

    static PRM_Name    theSubStepsName("substeps", "Box3D Sub-Steps");
    static PRM_Default theSubStepsDefault(4);

    static PRM_Name    theThreadsName("threads", "Threads (0 = All Cores)");
    static PRM_Default theThreadsDefault(0);   // 0 = all cores

    static PRM_Name    theSleepName("enablesleep", "Allow Sleeping");
    static PRM_Default theSleepDefault(1);

    static PRM_Template theTemplates[] = {
        PRM_Template(PRM_FLT_J,  3, &theGravityName,  theGravityDefault),
        PRM_Template(PRM_INT_J,  1, &theSubStepsName, &theSubStepsDefault),
        PRM_Template(PRM_INT_J,  1, &theThreadsName,  &theThreadsDefault),
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

b3dop::WorldSettings
SIM_SolverBox3D::currentWorldSettings() const
{
    b3dop::WorldSettings s;
    const UT_Vector3 g = getGravity();
    s.gravity[0]  = g.x();
    s.gravity[1]  = g.y();
    s.gravity[2]  = g.z();
    s.subSteps    = static_cast<int>(getSubSteps());
    {
        // Box3D runs its own thread pool (not Houdini's). 0 means every core.
        int n = static_cast<int>(getThreads());
        if (n <= 0)
            n = static_cast<int>(std::thread::hardware_concurrency());
        s.workerCount = n < 1 ? 1 : (n > 32 ? 32 : n);      // Box3D's maximum is 32
    }
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
        if (!worldData->hasValidWorld())
        {
            reason = "no valid world (first step, reset, or copied from empty)";
        }
        else
        {
            std::lock_guard<std::mutex> lock(st->mutex);
            if (st->world.settings().workerCount != currentWorldSettings().workerCount)
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
    if (!worldData->hasValidWorld())
    {
        addError(objects(0), SIM_MESSAGE, "Box3D: world creation failed", UT_ERROR_ABORT);
        return SIM_SOLVER_FAIL;
    }

    Box3DSharedState *st = worldData->state();
    std::lock_guard<std::mutex> lock(st->mutex);

    std::vector<char> objectHasDynamic;
    syncFromGeometry(*worldData, objects, objectHasDynamic);
    stepWorld(*worldData, static_cast<float>(dt));
    writeResultsBack(*worldData, objects, objectHasDynamic);

    st->markStepped(now);

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

    std::unordered_set<uint64_t> seen;
    SyncCounts                   counts;
    objectHasDynamic.assign(size_t(objects.entries()), 0);

    for (exint i = 0; i < objects.entries(); ++i)
        objectHasDynamic[size_t(i)] = syncObject(*st, *objects(i), seen, counts) ? 1 : 0;

    // Bodies whose point (or whole object) is gone.
    int removed = 0;
    for (auto it = st->mirror.begin(); it != st->mirror.end();)
    {
        if (!seen.count(it->first))
        {
            st->world.removeBody(it->first);
            it = st->mirror.erase(it);
            ++removed;
        }
        else
            ++it;
    }

    // Hulls built for bodies that were then removed (or never created) are
    // no longer needed; hulls still used by a body stay cached for the next
    // body that instances the same geometry.
    st->world.pruneHulls();

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
                                  const std::vector<char> &objectHasDynamic)
{
    Box3DSharedState *st = worldData.state();

    for (exint i = 0; i < objects.entries(); ++i)
    {
        // Objects with only static points: nothing to write, and asking for
        // the non-const geometry would copy it for no reason.
        if (!objectHasDynamic[size_t(i)])
            continue;

        SIM_Object *obj = objects(i);
        SIM_GeometryCopy *geo = SIM_DATA_GET(*obj, SIM_GEOMETRY_DATANAME, SIM_GeometryCopy);
        if (!geo)
        {
            B3D_LOG("  writeBack: object id=%d Geometry is not writable (not a SIM_GeometryCopy)",
                    obj->getObjectId());
            addError(obj, SIM_MESSAGE,
                     "Box3D: this object's Geometry is not writable (not a SIM_GeometryCopy); results are not written back",
                     UT_ERROR_WARNING);
            continue;
        }

        SIM_GeometryAutoWriteLock lock(geo);
        GU_Detail *gdp = gdpPtr(lock.getGdp());
        if (!gdp)
            continue;

        const bool createdOrient = ensureResultAttribs(gdp);

        const PointAttrs a(gdp);          // bound AFTER the attributes exist
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

        // ---- pass 1 (serial): pull results out of Box3D ---------------------
        // The mirror and the Box3D world are shared state, so they are only
        // touched here and in pass 4.
        std::vector<WriteItem> items;
        int missing = 0;
        for (GA_Iterator it(gdp->getPointRange()); !it.atEnd(); ++it)
        {
            const GA_Offset off = *it;
            const uint64_t  key = pointKey(gdp, a, obj->getObjectId(), off);

            auto mit = st->mirror.find(key);
            if (mit == st->mirror.end() || !mit->second.dynamic)
                continue;

            WriteItem w;
            w.off   = off;
            w.entry = &mit->second;     // stable: nothing is inserted or erased below
            w.s     = mit->second;      // keeps handle and type
            if (!st->world.getTransformByHandle(w.s.handle, w.s.pos, w.s.rot) ||
                !st->world.getVelocityByHandle(w.s.handle, w.s.lin, w.s.ang))
            {
                ++missing;      // leave the point alone; next sync re-snapshots it
                continue;
            }
            items.push_back(w);
        }

        // ---- pass 2 (parallel): write attributes, one task per point ---------
        // Every item is a distinct point, so no two tasks write the same
        // element. Pages must be hardened first, or the writes would race on
        // copy-on-write pages that are still shared.
        GA_RWHandleV3 pOut(gdp, GA_ATTRIB_POINT, "P");
        for (GA_Attribute *attr : { pOut.isValid() ? pOut.getAttribute() : nullptr,
                                    qOut.isValid() ? qOut.getAttribute() : nullptr,
                                    vOut.isValid() ? vOut.getAttribute() : nullptr,
                                    wOut.isValid() ? wOut.getAttribute() : nullptr })
            if (attr)
                attr->hardenAllPages();

        {
            // rotationMatrixFromQuat has a one-time convention check; run it
            // here so no worker thread is the one that triggers (and logs) it.
            const float identity[4] = { 0, 0, 0, 1 };
            double      warm[9];
            rotationMatrixFromQuat(identity, warm);
        }

        parallelForEach(items.size(), [&](size_t i) {
            WriteItem       &w   = items[i];
            const GA_Offset  off = w.off;
            const DopBodyState &s = w.s;

            if (pOut.isValid()) pOut.set(off, UT_Vector3(float(s.pos[0]), float(s.pos[1]), float(s.pos[2])));
            if (qOut.isValid()) qOut.set(off, UT_QuaternionF(s.rot[0], s.rot[1], s.rot[2], s.rot[3]));
            if (vOut.isValid()) vOut.set(off, UT_Vector3(s.lin[0], s.lin[1], s.lin[2]));
            if (wOut.isValid()) wOut.set(off, UT_Vector3(s.ang[0], s.ang[1], s.ang[2]));

            // A packed primitive's rotation is NOT the point's `orient`: it is
            // the primitive's own transform. Work out the new matrix here (pure
            // math, plus reads of the primitive), but do not write it: that is
            // pass 3. Keep its scale, replace the rotation.
            PackedInfo pk = getPackedInfo(gdp, off);
            if (pk.prim && pk.rotationFromPrim)
            {
                const double oldM3[9] = { pk.m4[0], pk.m4[1], pk.m4[2],
                                          pk.m4[4], pk.m4[5], pk.m4[6],
                                          pk.m4[8], pk.m4[9], pk.m4[10] };
                composeLocal3(s.rot, oldM3, w.newM3);
                w.primOff = pk.primOff;
                w.hasM3   = true;
                // What the primitive will hold, without reading it back: the
                // rotation of the matrix pass 3 writes.
                double sc[3];
                b3dop::matrix3ToQuatScale(w.newM3, pk.rot, sc);
            }

            // Re-read what the points actually hold (P may be stored in lower
            // precision than Box3D's), so next step's edit detection compares
            // like with like and does not report our own write as an edit.
            w.storedOk = readPointState(gdp, a, off, pk, w.stored);
        });

        for (GA_Attribute *attr : { pOut.isValid() ? pOut.getAttribute() : nullptr,
                                    qOut.isValid() ? qOut.getAttribute() : nullptr,
                                    vOut.isValid() ? vOut.getAttribute() : nullptr,
                                    wOut.isValid() ? wOut.getAttribute() : nullptr })
            if (attr)
                attr->bumpDataId();

        // ---- pass 3 (serial): packed primitive rotation ----------------------
        // setLocalTransform + transformDirty stay serial: the HDK pages do not
        // say whether they touch shared state.
        for (const WriteItem &w : items)
        {
            if (!w.hasM3)
                continue;
            GU_PrimPacked *prim = static_cast<GU_PrimPacked *>(gdp->getPrimitive(w.primOff));
            setLocal3(*prim, w.newM3, std::integral_constant<bool, kPackedRotationWritable>());
        }

        // ---- pass 4 (serial): update the mirror --------------------------------
        for (WriteItem &w : items)
        {
            if (w.storedOk)
            {
                w.stored.dynamic = true;
                w.stored.type    = w.entry->type;
                w.stored.handle  = w.entry->handle;
                *w.entry = w.stored;
            }
            else
                *w.entry = w.s;     // w.s was copied from the entry: keeps handle and type
        }

        if (missing)
        {
            B3D_LOG("  writeBack: object id=%d: %d points have no Box3D body", obj->getObjectId(), missing);
            addError(obj, SIM_MESSAGE,
                     ("Box3D: " + std::to_string(missing) + " point(s) have no Box3D body; their results were not written back").c_str(),
                     UT_ERROR_WARNING);
        }
    }
}

void
SIM_SolverBox3D::publishEvents(SIM_Box3DWorld & /*worldData*/, SIM_ObjectArray & /*objects*/)
{
    // TODO: begin/end touch, hit, sensor, and sleep events as DOP data.
}
