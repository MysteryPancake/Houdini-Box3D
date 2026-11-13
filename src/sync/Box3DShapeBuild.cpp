// Box3DShapeBuild.cpp
//
// Collision source geometry (packed primitives' pieces, cached per source) and the Box3D
// shapes built from it, for points and for whole objects.
#include "Box3DSolverInternal.h"

namespace box3dsolver {

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

// The primitive's full transform, row-major.
void
fetchFull4(const GU_PrimPacked &p, double out[16])
{
    UT_Matrix4D m;
    p.getFullTransform4(m);
    box3dToArray(m, out);
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

// The source's summary, if one was recorded for it (no geometry is read).
bool
ShapeSources::summary(const GU_PrimPacked &prim, bool connected, SourceSummary &out)
{
    const GU_PackedFragment *frag = asFragment(prim);
    GU_DetailHandleAutoReadLock lock(sourceHandle(prim, frag));
    const GU_Detail *gdp = lock.getGdp();
    return gdp && SourceSummaryCache::instance().find(sourceIdentity(gdp, fragmentFirst(frag), connected), out);
}

// The primitive's pieces (by `name`, or connected sets), or null. Records the summary.
const PieceGroup *
ShapeSources::innerPieces(const GU_PrimPacked &prim, bool connected)
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
        return anyPieceNonEmpty(entry->pieces) ? &entry->pieces : nullptr;
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

// ---------------------------------------------------------------------------
// Building the Box3D shapes for one point
// ---------------------------------------------------------------------------

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

// A point's shapes (docs/agents/bodies.md, "Pieces and packed geometry"):
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

// A whole object's shapes, body-local: the geometry through its own transform `m` (object space;
// scale and shear are baked in), relative to `pivot`. Box / sphere / capsule fit the whole
// geometry, convex is one hull (or one per connected set), concave one hull per `name` piece.
BuiltShapes
buildWholeObjectShapes(b3dop::Box3DWorld &world, const GU_Detail *gdp, const double m[16], const double pivot[3],
                       b3dop::ShapeType type, bool hullPerConnected)
{
    using b3dop::ShapeType;
    BuiltShapes out;
    if (type == ShapeType::None)
        return out;

    PieceGroup g;
    const bool connected = type == ShapeType::Convex && hullPerConnected;
    if (connected)
        collectConnectedPieces(gdp, gdp->getPrimitiveRange(), g);
    if (g.pieces.empty())
        collectPieces(gdp, g);

    auto toBody = [&](const NamedPiece &np, std::vector<float> &dst) {
        for (size_t k = 0; k + 2 < np.xyz.size(); k += 3)
        {
            const double p[3] = { np.xyz[k], np.xyz[k + 1], np.xyz[k + 2] };
            double o[3];
            b3dop::xformPoint(m, p, o);
            for (int c = 0; c < 3; ++c)
                dst.push_back(float(o[c] - pivot[c]));
        }
    };
    std::vector<float> pts;
    auto addHull = [&](const std::vector<const NamedPiece *> &from) {
        Piece pc;
        if (pieceFromPoints(world, 0, 0,
                            [&](std::vector<float> &dst) {
                                dst.clear();
                                for (const NamedPiece *np : from)
                                    toBody(*np, dst);
                            },
                            pts, pc, out.note))
        {
            out.totalVolume += pc.volume;
            out.pieces.push_back(pc);
        }
    };

    std::vector<const NamedPiece *> all;
    for (const NamedPiece &np : g.pieces)
        if (!np.xyz.empty())
            all.push_back(&np);
    if (all.empty())
        return out;

    if (type == ShapeType::Concave || connected)
    {
        for (const NamedPiece *np : all)
            addHull({ np });
    }
    else if (type == ShapeType::Convex)
        addHull(all);
    else
    {
        pts.clear();
        for (const NamedPiece *np : all)
            toBody(*np, pts);
        Piece pc;
        if (b3dop::fitShapeToPoints(type, pts.data(), int(pts.size() / 3), pc.spec))
        {
            pc.volume = b3dop::shapeVolume(pc.spec);
            out.totalVolume += pc.volume;
            out.pieces.push_back(pc);
        }
    }
    return out;
}

} // namespace box3dsolver
