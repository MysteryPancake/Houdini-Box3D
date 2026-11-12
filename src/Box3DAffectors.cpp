// Box3DAffectors.cpp
#include "Box3DAffectors.h"
#include "Box3DAffectorMath.h"
#include "Box3DDebug.h"
#include "SIM_Box3DData.h"
#include "SIM_Box3DWorld.h"

#include <GA/GA_Iterator.h>
#include <GA/GA_Primitive.h>
#include <GA/GA_PrimitiveTypes.h>
#include <GU/GU_Detail.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_Names.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_Utils.h>
#include <UT/UT_Matrix4.h>
#include <UT/UT_Vector3.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

// The relationship type name SIM_Object::getConstAffectors() matches (it is the data type name:
// SideFX's own hair solver passes "SIM_RelationshipSource" the same way).
const char *const kCollideRelationship = "SIM_RelationshipCollide";

// Triangles from the object's polygons (fan-triangulated; open curves, packed primitives and the
// like are ignored). Vertices are the object's points, in point order of first use.
struct TriMesh
{
    std::vector<float>   xyz;           // object space
    std::vector<int32_t> tris;
};

void
buildMesh(const GU_Detail *gdp, bool houdiniWinding, TriMesh &out)
{
    out.xyz.clear();
    out.tris.clear();
    std::unordered_map<GA_Offset, int32_t> remap;
    remap.reserve(size_t(gdp->getNumPoints()));

    auto vertexOf = [&](GA_Offset pt) -> int32_t {
        auto it = remap.find(pt);
        if (it != remap.end())
            return it->second;
        const UT_Vector3 p = gdp->getPos3(pt);
        const int32_t idx = int32_t(out.xyz.size() / 3);
        out.xyz.push_back(p.x());
        out.xyz.push_back(p.y());
        out.xyz.push_back(p.z());
        remap.emplace(pt, idx);
        return idx;
    };

    for (GA_Iterator it(gdp->getPrimitiveRange()); !it.atEnd(); ++it)
    {
        const GA_Primitive *prim = gdp->getPrimitive(*it);
        if (!prim || prim->getTypeId() != GA_PRIMPOLY)
            continue;
        const GA_Size nv = prim->getVertexCount();
        if (nv < 3)
            continue;
        const int32_t first = vertexOf(prim->getPointOffset(0));
        for (GA_Size i = 1; i + 1 < nv; ++i)
        {
            const int32_t b = vertexOf(prim->getPointOffset(i));
            const int32_t c = vertexOf(prim->getPointOffset(i + 1));
            // Houdini polygons wind clockwise seen from their front (assumed: the Flip Static Mesh Winding
            // toggle on the Box3D Data node is there in case it is not); Box3D wants counter-clockwise
            // front faces, and its meshes are one-sided.
            out.tris.push_back(first);
            out.tris.push_back(houdiniWinding ? c : b);
            out.tris.push_back(houdiniWinding ? b : c);
        }
    }
}

void
destroyAffector(Box3DSharedState &st, Box3DAffectorBody &ab)
{
    if (ab.handle)
        st.world.destroyBody(ab.handle);
    ab.handle = 0;
}

} // namespace

void
syncAffectorBodies(Box3DSharedState &st, SIM_ObjectArray &objects, Box3DAffectorCounts &counts)
{
    // ---- gather: every affector of a solved object, once, that we do not solve ourselves ----
    std::unordered_set<int> solved;
    for (exint i = 0; i < objects.entries(); ++i)
        solved.insert(objects(i)->getObjectId());

    std::vector<const SIM_Object *> affectors;
    std::unordered_set<int>         seen;
    for (exint i = 0; i < objects.entries(); ++i)
    {
        SIM_ConstObjectArray aff;
        objects(i)->getConstAffectors(aff, kCollideRelationship);
        for (exint j = 0; j < aff.entries(); ++j)
        {
            const SIM_Object *a = aff(j);
            if (!a)
                continue;
            const int id = a->getObjectId();
            if (solved.count(id) || !seen.insert(id).second)
                continue;
            affectors.push_back(a);
        }
    }
    counts.total = int(affectors.size());

    // ---- build / update ---------------------------------------------------------------------
    TriMesh mesh;
    std::vector<float> baked;
    for (const SIM_Object *obj : affectors)
    {
        const int objectId = obj->getObjectId();
        const SIM_Geometry *geo = obj->getGeometry();
        if (!geo)
        {
            ++counts.noGeo;
            continue;
        }
        SIM_GeometryAutoReadLock lock(geo);
        const GU_Detail *gdp = lock.getGdp();
        if (!gdp)
        {
            ++counts.noGeo;
            continue;
        }

        Box3DAffectorBody &ab = st.affectors[objectId];
        ab.stamp = st.syncStamp;

        // The object's world transform (position + any scale), row-vector layout.
        UT_DMatrix4 xf;
        SIMgetGeometryTransform(xf, *obj);
        double m[16];
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                m[4 * r + c] = xf(r, c);
        bool finite = true;
        for (double d : m)
            finite = finite && std::isfinite(d);
        if (!finite)
        {
            ++counts.failed;
            continue;
        }

        double pos[3];
        float  rot[4];
        const bool rigid = b3dop::decomposeRigid(m, pos, rot);

        // Material: a Box3D Data node on the affector sets friction and bounce (bodies are always static).
        b3dop::BodySettings body;
        if (const SIM_Box3DData *node = SIM_DATA_GETCONST(*obj, kBox3DDataName, SIM_Box3DData))
            body = node->toBodySettings();
        body.type = b3dop::BodySettings::Type::Static;

        // What the shape depends on: the points and polygons, the material, and (when not rigid)
        // the whole matrix, which is baked into the vertices.
        const GA_Attribute *P = gdp->getP();
        uint64_t sig = b3dop::kHashSeed;
        sig = b3dop::mixHash(sig, P ? uint64_t(P->getDataId()) : 0);
        sig = b3dop::mixHash(sig, gdp->getNumPrimitives() > 0 ? uint64_t(gdp->getPrimitiveList().getDataId()) : 0);
        sig = b3dop::mixHash(sig, uint64_t(gdp->getNumPoints()));
        sig = b3dop::mixHash(sig, uint64_t(gdp->getNumPrimitives()));
        const float mat[3] = { body.friction, body.restitution, body.flipMeshWinding ? 1.0f : 0.0f };
        sig = b3dop::mixHash(sig, b3dop::hashFloats(mat, 3));
        sig = b3dop::mixHash(sig, rigid ? 1 : 2);
        if (!rigid)
            for (double d : m)
                sig = b3dop::mixDouble(sig, d);

        const bool unchanged = ab.poseKnown && ab.signature == sig;
        if (unchanged)
        {
            // Same shape. A moved (animated) rigid object only needs a new pose; a failed build
            // (handle 0) stays failed until the signature changes.
            if (ab.handle && rigid)
            {
                const bool samePose = std::fabs(ab.pos[0] - pos[0]) < 1e-9 && std::fabs(ab.pos[1] - pos[1]) < 1e-9 &&
                                      std::fabs(ab.pos[2] - pos[2]) < 1e-9 && std::fabs(ab.rot[0] - rot[0]) < 1e-7f &&
                                      std::fabs(ab.rot[1] - rot[1]) < 1e-7f && std::fabs(ab.rot[2] - rot[2]) < 1e-7f &&
                                      std::fabs(ab.rot[3] - rot[3]) < 1e-7f;
                if (!samePose)
                {
                    st.world.setTransformByHandle(ab.handle, pos, rot);
                    for (int i = 0; i < 3; ++i) ab.pos[i] = pos[i];
                    for (int i = 0; i < 4; ++i) ab.rot[i] = rot[i];
                    ++counts.moved;
                }
            }
            else if (!ab.handle)
                ++counts.failed;
            continue;
        }

        // (Re)build from scratch.
        destroyAffector(st, ab);
        ab.signature = sig;
        ab.poseKnown = true;

        buildMesh(gdp, !body.flipMeshWinding, mesh);
        if (mesh.tris.empty())
        {
            ++counts.failed;
            continue;
        }
        const float *verts = mesh.xyz.data();
        double bodyPos[3] = { 0, 0, 0 };
        float  bodyRot[4] = { 0, 0, 0, 1 };
        if (rigid)
        {
            for (int i = 0; i < 3; ++i) bodyPos[i] = pos[i];
            for (int i = 0; i < 4; ++i) bodyRot[i] = rot[i];
        }
        else
        {
            // Scale / shear cannot be a body transform: bake the matrix into the vertices and keep the body at
            // the origin. (Mirroring flips the winding, so flip the triangles back.)
            const size_t n = mesh.xyz.size() / 3;
            baked.resize(mesh.xyz.size());
            for (size_t i = 0; i < n; ++i)
            {
                const double p[3] = { mesh.xyz[3 * i], mesh.xyz[3 * i + 1], mesh.xyz[3 * i + 2] };
                double q[3];
                b3dop::transformPoint(m, p, q);
                for (int j = 0; j < 3; ++j)
                    baked[3 * i + size_t(j)] = float(q[j]);
            }
            const double det = m[0] * (m[5] * m[10] - m[6] * m[9]) - m[1] * (m[4] * m[10] - m[6] * m[8]) +
                               m[2] * (m[4] * m[9] - m[5] * m[8]);
            if (det < 0.0)
                for (size_t t = 0; t + 2 < mesh.tris.size(); t += 3)
                    std::swap(mesh.tris[t + 1], mesh.tris[t + 2]);
            verts = baked.data();
            ++counts.baked;
        }

        const b3dop::BodyHandle h = st.world.createBody(body, bodyPos, bodyRot, nullptr);
        if (!h)
        {
            ++counts.failed;
            continue;
        }
        if (!st.world.createMeshShape(h, body, verts, int(mesh.xyz.size() / 3), mesh.tris.data(),
                                      int(mesh.tris.size() / 3)))
        {
            st.world.destroyBody(h);
            ++counts.failed;
            continue;
        }
        ab.handle = h;
        for (int i = 0; i < 3; ++i) ab.pos[i] = bodyPos[i];
        for (int i = 0; i < 4; ++i) ab.rot[i] = bodyRot[i];
        ++counts.added;
        B3D_LOG("  affector id=%d: static mesh built, %d triangle(s)%s", objectId, int(mesh.tris.size() / 3),
                rigid ? "" : " (transform baked)");
    }

    // ---- remove: objects that are no longer affectors ---------------------------------------
    for (auto it = st.affectors.begin(); it != st.affectors.end();)
    {
        if (it->second.stamp != st.syncStamp)
        {
            if (it->second.handle)
                ++counts.removed;
            destroyAffector(st, it->second);
            it = st.affectors.erase(it);
        }
        else
            ++it;
    }
}
