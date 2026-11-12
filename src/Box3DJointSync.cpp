// Box3DJointSync.cpp
#include "Box3DJointSync.h"
#include "Box3DDebug.h"
#include "SIM_Box3DJoints.h"
#include "SIM_Box3DWorld.h"

#include <GA/GA_Handle.h>
#include <GA/GA_Iterator.h>
#include <GA/GA_Primitive.h>
#include <GU/GU_Detail.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <UT/UT_Quaternion.h>
#include <UT/UT_Vector3.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

constexpr float kDegToRad = 0.017453292519943295f;
constexpr float kRadToDeg = 57.29577951308232f;

const char *const kTypeAttrib  = "joint_type";
const char *const kBrokenGroup = "broken";

// The joint network's data name on an object: its Box3D Joints node's, else "Joints".
std::string
jointsDataName(const SIM_Object &obj, const SIM_Box3DJoints **nodeOut = nullptr)
{
    const SIM_Box3DJoints *node = SIM_DATA_GETCONST(obj, kBox3DJointsDataName, SIM_Box3DJoints);
    if (nodeOut)
        *nodeOut = node;
    return node ? node->geometryDataName() : std::string(kDefaultJointsGeometry);
}

uint64_t
networkKey(int objectId, const std::string &dataName)
{
    return b3dop::mixHash(b3dop::hashBytes(b3dop::kHashSeed, dataName.data(), dataName.size()),
                          uint64_t(uint32_t(objectId)));
}

uint64_t
idOf(const GA_Attribute *a)
{
    return a ? uint64_t(a->getDataId()) : ~0ull;
}

// Everything the joints are read from: the geometry's identity, topology, and the data ids of
// every attribute and group the sync reads. Equal: nothing to re-read.
uint64_t
networkFingerprint(const GU_Detail *gdp)
{
    using b3dop::mixHash;
    uint64_t h = mixHash(b3dop::kHashSeed, uint64_t(gdp->getUniqueId()));
    h = mixHash(h, idOf(gdp->getP()));
    h = mixHash(h, uint64_t(gdp->getPrimitiveList().getDataId()));
    h = mixHash(h, uint64_t(gdp->getNumPoints()));
    h = mixHash(h, uint64_t(gdp->getNumPrimitives()));
    h = mixHash(h, idOf(gdp->findPointAttribute("name")));
    h = mixHash(h, idOf(gdp->findPointAttribute("orient")));
    h = mixHash(h, idOf(gdp->findPrimitiveAttribute(kTypeAttrib)));
    for (const b3dop::JointParam &p : b3dop::jointParams())
        h = mixHash(h, idOf(gdp->findPrimitiveAttribute(p.name)));
    const GA_PrimitiveGroup *broken = gdp->findPrimitiveGroup(kBrokenGroup);
    return mixHash(h, broken ? uint64_t(broken->getDataId()) : ~0ull);
}

// A writable SIM_GeometryCopy for the object's data `name` (a referenced SOP geometry is replaced
// once by a copy that adopts it), or null.
SIM_GeometryCopy *
writableGeometry(SIM_Object &obj, const char *name)
{
    if (SIM_GeometryCopy *geo = SIM_DATA_GET(obj, name, SIM_GeometryCopy))
        return geo;
    if (!SIM_DATA_GETCONST(obj, name, SIM_Geometry))
        return nullptr;
    return SIM_DATA_CREATE(obj, name, SIM_GeometryCopy, SIM_DATA_RETURN_EXISTING | SIM_DATA_ADOPT_EXISTING_ON_DELETE);
}

SIM_Object *
objectById(SIM_ObjectArray &objects, int id)
{
    for (exint i = 0; i < objects.entries(); ++i)
        if (objects(i)->getObjectId() == id)
            return objects(i);
    return nullptr;
}

// Anchor names -> bodies, over the objects of this solve (in their order) and the affectors.
class Anchors
{
public:
    Anchors(const Box3DSharedState &st, SIM_ObjectArray &objects) : myState(st)
    {
        for (exint i = 0; i < objects.entries(); ++i)
        {
            const SIM_Object *obj = objects(i);
            const auto it = st.objectSync.find(obj->getObjectId());
            if (it == st.objectSync.end() || it->second.visited != st.syncStamp)
                continue;
            const UT_String &nm = obj->getName();
            myObjects.emplace_back(nm.isstring() ? std::string(nm.buffer()) : std::string(), &it->second);
        }
    }

    // Empty: the world (0). Else a piece's `name` in any object, "object/piece", an object with a
    // single body, or an affector object. False if nothing matches.
    bool resolve(const std::string &name, b3dop::BodyHandle &out) const
    {
        out = 0;
        if (name.empty())
            return true;
        const size_t slash = name.find('/');
        if (slash != std::string::npos)
        {
            const std::string objName = name.substr(0, slash), piece = name.substr(slash + 1);
            for (const auto &o : myObjects)
                if (o.first == objName)
                    return findIn(*o.second, piece, out);
            return false;
        }
        for (const auto &o : myObjects)
            if (findIn(*o.second, name, out))
                return true;
        for (const auto &o : myObjects)
            if (o.first == name && o.second->bodies.size() == 1 && o.second->bodies[0]->handle)
            {
                out = o.second->bodies[0]->handle;
                return true;
            }
        for (const auto &kv : myState.affectors)
            if (kv.second.name == name && kv.second.handle)
            {
                out = kv.second.handle;
                return true;
            }
        return false;
    }

private:
    static bool findIn(const Box3DObjectSync &os, const std::string &piece, b3dop::BodyHandle &out)
    {
        const auto it = os.byName.find(piece);
        if (it == os.byName.end() || !it->second->handle)
            return false;
        out = it->second->handle;
        return true;
    }

    const Box3DSharedState                                        &myState;
    std::vector<std::pair<std::string, const Box3DObjectSync *>> myObjects;
};

// The primitive attributes that override a setting, bound once per network.
struct SettingReader
{
    const b3dop::JointParam *param = nullptr;
    GA_ROHandleI             i;
    GA_ROHandleF             f;
    GA_ROHandleV3            v;
};

std::vector<SettingReader>
bindSettings(const GU_Detail *gdp)
{
    std::vector<SettingReader> out;
    for (const b3dop::JointParam &p : b3dop::jointParams())
    {
        if (!gdp->findPrimitiveAttribute(p.name))
            continue;
        SettingReader r;
        r.param = &p;
        switch (p.kind)
        {
            case b3dop::JointParam::Toggle:
                r.i = GA_ROHandleI(gdp, GA_ATTRIB_PRIMITIVE, p.name);
                if (!r.i.isValid()) continue;
                break;
            case b3dop::JointParam::Float:
            case b3dop::JointParam::Angle:
                r.f = GA_ROHandleF(gdp, GA_ATTRIB_PRIMITIVE, p.name);
                if (!r.f.isValid()) continue;
                break;
            default:
                r.v = GA_ROHandleV3(gdp, GA_ATTRIB_PRIMITIVE, p.name);
                if (!r.v.isValid()) continue;
                break;
        }
        out.push_back(r);
    }
    return out;
}

// The primitive's overrides on top of the defaults for its type.
void
applyOverrides(const std::vector<SettingReader> &readers, b3dop::JointType type, GA_Offset prim,
               b3dop::JointSettings &s)
{
    using b3dop::JointParam;
    for (const SettingReader &r : readers)
    {
        const JointParam &p = *r.param;
        if (!p.usedBy(type))
            continue;
        const float scale = p.isAngle(type) ? kDegToRad : 1.0f;
        switch (p.kind)
        {
            case JointParam::Toggle:
                s.*p.b = r.i.get(prim) != 0;
                break;
            case JointParam::Float:
            case JointParam::Angle:
                s.*p.f = r.f.get(prim) * scale;
                break;
            case JointParam::Vec3:
            case JointParam::Vec3Angle:
            {
                const UT_Vector3 v = r.v.get(prim);
                for (int k = 0; k < 3; ++k)
                    (s.*p.v)[k] = v(k) * scale;
                break;
            }
            case JointParam::Rotation:
            {
                const UT_Vector3 v = r.v.get(prim);
                const float deg[3] = { v(0), v(1), v(2) };
                box3dEulerToQuat(deg, s.*p.q);
                break;
            }
        }
    }
}

// What a joint is made from: its type and bodies, plus its frames when they are not live (a
// network that is not written back keeps the authored frames, so a change there is an edit).
uint64_t
jointSignature(b3dop::JointType type, b3dop::BodyHandle a, b3dop::BodyHandle b, const b3dop::JointFrame &fa,
               const b3dop::JointFrame &fb, bool withFrames)
{
    using b3dop::mixHash;
    uint64_t h = mixHash(mixHash(mixHash(b3dop::kHashSeed, uint64_t(type)), a), b);
    if (withFrames)
        for (const b3dop::JointFrame *f : { &fa, &fb })
        {
            for (int k = 0; k < 3; ++k)
                h = b3dop::mixDouble(h, f->pos[k]);
            h = b3dop::hashBytes(h, f->rot, sizeof(f->rot));
        }
    return h ? h : 1;
}

// An anchor moved away from what the solver last wrote: edited upstream (q and -q are equal).
bool
frameEdited(const b3dop::JointFrame &now, const b3dop::JointFrame &wrote)
{
    for (int k = 0; k < 3; ++k)
        if (std::fabs(now.pos[k] - wrote.pos[k]) > 1e-5 * std::max(1.0, std::fabs(wrote.pos[k])))
            return true;
    double dot = 0.0;
    for (int k = 0; k < 4; ++k)
        dot += double(now.rot[k]) * double(wrote.rot[k]);
    return std::fabs(dot) < 1.0 - 1e-6;
}

// A frame as the geometry stores it (float P), so it compares equal when read back.
b3dop::JointFrame
asStored(const b3dop::JointFrame &f)
{
    b3dop::JointFrame out = f;
    for (int k = 0; k < 3; ++k)
        out.pos[k] = double(float(f.pos[k]));
    return out;
}

struct NetworkRef
{
    SIM_Object            *carrier  = nullptr;
    std::string            dataName;
    const SIM_Box3DJoints *node     = nullptr;
    uint64_t               key      = 0;
    int                    carriers = 1;
};

void
processNetwork(Box3DSharedState &st, const NetworkRef &ref, const Anchors &anchors, Box3DJointCounts &counts)
{
    using b3dop::JointType;
    b3dop::Box3DWorld &world = st.world;

    Box3DJointNetwork &net = st.jointNetworks[ref.key];
    net.visited       = st.syncStamp;
    net.carrier       = ref.carrier->getObjectId();
    net.carriers      = ref.carriers;
    net.dataName      = ref.dataName;
    net.writeFeedback = ref.node ? ref.node->getWriteFeedback() : true;
    if (ref.carriers > 1)
        ++counts.shared;

    bool defaultsChanged = !net.defaultsKnown;
    for (int t = 0; t < b3dop::kJointTypeCount; ++t)
    {
        const b3dop::JointSettings d = ref.node ? ref.node->defaults(JointType(t))
                                                : b3dop::Box3DWorld::defaultJointSettings(JointType(t));
        if (!net.defaultsKnown || !b3dop::sameJointSettings(JointType(t), d, net.defaults[t]))
        {
            net.defaults[t] = d;
            defaultsChanged = true;
        }
    }
    net.defaultsKnown = true;

    const SIM_Geometry *geo = SIM_DATA_GETCONST(*ref.carrier, ref.dataName.c_str(), SIM_Geometry);
    if (!geo)
        return;
    SIM_GeometryAutoReadLock lock(geo);
    const GU_Detail *gdp = lock.getGdp();
    if (!gdp)
        return;

    // Unchanged geometry, bodies and defaults: the joints stay as they are (they keep liveStamp).
    const uint64_t fp = networkFingerprint(gdp);
    if (fp == net.fingerprint && net.bodyVersion == st.bodyVersion && !defaultsChanged)
        return;
    net.fingerprint = fp;
    net.bodyVersion = st.bodyVersion;
    net.liveStamp   = st.syncStamp;

    const GA_ROHandleS typeH(gdp, GA_ATTRIB_PRIMITIVE, kTypeAttrib);
    const GA_ROHandleS nameH(gdp, GA_ATTRIB_POINT, "name");
    const GA_ROHandleQ orientH(gdp, GA_ATTRIB_POINT, "orient");
    const GA_PrimitiveGroup *brokenGroup = gdp->findPrimitiveGroup(kBrokenGroup);
    const std::vector<SettingReader> readers = bindSettings(gdp);

    auto anchorName = [&](GA_Offset pt) {
        if (!nameH.isValid())
            return std::string();
        const char *n = nameH.get(pt);
        return n ? std::string(n) : std::string();
    };
    auto anchorFrame = [&](GA_Offset pt) {
        b3dop::JointFrame f;
        const UT_Vector3 p = gdp->getPos3(pt);
        f.pos[0] = p.x();  f.pos[1] = p.y();  f.pos[2] = p.z();
        if (orientH.isValid())
        {
            const UT_QuaternionF q = orientH.get(pt);
            f.rot[0] = q.x();  f.rot[1] = q.y();  f.rot[2] = q.z();  f.rot[3] = q.w();
        }
        return f;
    };

    for (GA_Iterator it(gdp->getPrimitiveRange()); !it.atEnd(); ++it)
    {
        const GA_Offset     off  = *it;
        const GA_Primitive *prim = gdp->getPrimitive(off);
        if (!prim)
            continue;
        if (prim->getVertexCount() != 2)
        {
            ++counts.notLines;
            continue;
        }
        JointType type = JointType::Weld;
        if (!typeH.isValid() || !b3dop::parseJointType(typeH.get(off), type))
        {
            ++counts.unknown;
            continue;
        }

        auto ins = st.joints.try_emplace(b3dop::mixHash(ref.key, uint64_t(int64_t(off))));
        Box3DJointEntry &e = ins.first->second;
        if (ins.second)
        {
            e.network = ref.key;
            e.prim    = off;
        }
        auto drop = [&]() {
            world.destroyJoint(e.handle);       // harmless when already gone with a body
            e.handle = 0;
        };

        // Broken (by us, or marked in the geometry): never remade.
        if (e.broken || (brokenGroup && brokenGroup->containsOffset(off)))
        {
            e.broken = true;
            drop();
            e.stamp = net.liveStamp;
            continue;
        }

        const GA_Offset   p0 = prim->getPointOffset(0), p1 = prim->getPointOffset(1);
        b3dop::BodyHandle a = 0, b = 0;
        if (!anchors.resolve(anchorName(p0), a) || !anchors.resolve(anchorName(p1), b))
        {
            ++counts.unresolved;
            drop();
            continue;                           // not stamped: removed; retried when bodies change
        }
        // P / orient are live on a written network (where the pieces are now), so a world rebuilt
        // mid-simulation makes its joints from them exactly as frame 1 does from the authored ones.
        const b3dop::JointFrame fa = anchorFrame(p0), fb = anchorFrame(p1);
        b3dop::JointSettings s = net.defaults[int(type)];
        applyOverrides(readers, type, off, s);
        const bool     live   = ref.carriers == 1;
        const uint64_t sig    = jointSignature(type, a, b, fa, fb, !live);
        const bool     edited = live && e.wrote && (frameEdited(fa, e.wroteA) || frameEdited(fb, e.wroteB));

        if (world.isJointValid(e.handle) && e.sig == sig && e.type == type && !edited)
        {
            if (!b3dop::sameJointSettings(type, s, e.settings))
            {
                world.updateJoint(e.handle, type, s);
                e.settings = s;
                ++counts.updated;
            }
        }
        else
        {
            drop();
            e.type     = type;
            e.bodyA    = a;
            e.bodyB    = b;
            e.sig      = sig;
            e.settings = s;
            e.wrote    = false;
            e.madeRestLength = -1.0f;
            if (type == JointType::Distance && !(s.restLength >= 0.0f))
            {
                const double dx = fb.pos[0] - fa.pos[0], dy = fb.pos[1] - fa.pos[1], dz = fb.pos[2] - fa.pos[2];
                e.madeRestLength = float(std::sqrt(dx * dx + dy * dy + dz * dz));
            }
            e.handle   = world.createJoint(type, a, b, fa, fb, s, &e);
            if (e.handle)
                ++counts.created;
            else
                ++counts.failed;
        }
        e.stamp = net.liveStamp;
    }
}

// After the step, for a written network: every live joint's anchors get P / orient set to where
// its frames are now. A distance joint made with restlength < 0 gets its starting distance
// written to `restlength`, so a world rebuilt mid-simulation keeps it.
template <class PrimOk>
void
writeLiveFrames(const b3dop::Box3DWorld &world, GU_Detail *gdp, const std::vector<Box3DJointEntry *> &entries,
                const PrimOk &primOk)
{
    if (entries.empty())
        return;
    auto ensure = [&](const char *name, int size, bool quat, bool &created) {
        created = false;
        if (!gdp->findPointAttribute(name))
        {
            gdp->addFloatTuple(GA_ATTRIB_POINT, name, size);
            created = true;
            if (quat)
                if (GA_Attribute *attr = gdp->findPointAttribute(name))
                    attr->setTypeInfo(GA_TYPE_QUATERNION);
        }
    };
    bool orientNew = false;
    ensure("orient", 4, true, orientNew);
    GA_RWHandleV3 pH(gdp, GA_ATTRIB_POINT, "P");
    GA_RWHandleQ  orientH(gdp, GA_ATTRIB_POINT, "orient");
    if (!pH.isValid() || !orientH.isValid())
        return;
    bool wroteRestLength = false;
    GA_RWHandleF restLengthH;

    for (Box3DJointEntry *e : entries)
    {
        if (!primOk(e->prim))
            continue;
        const GA_Primitive *prim = gdp->getPrimitive(e->prim);
        if (!prim || prim->getVertexCount() != 2)
            continue;
        const GA_Offset pts[2] = { prim->getPointOffset(0), prim->getPointOffset(1) };
        if (e->madeRestLength >= 0.0f)
        {
            if (!restLengthH.isValid())
            {
                if (!gdp->findPrimitiveAttribute("restlength"))
                    gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, "restlength", 1, GA_Defaults(-1.0f));
                restLengthH = GA_RWHandleF(gdp, GA_ATTRIB_PRIMITIVE, "restlength");
            }
            if (restLengthH.isValid())
            {
                restLengthH.set(e->prim, e->madeRestLength);
                wroteRestLength = true;
            }
            e->madeRestLength = -1.0f;
        }

        b3dop::JointFrame fa, fb;
        if (!world.jointWorldFrames(e->handle, fa, fb))
            continue;                           // broken, or never made
        e->wroteA = asStored(fa);
        e->wroteB = asStored(fb);
        e->wrote  = true;
        const b3dop::JointFrame *frames[2] = { &e->wroteA, &e->wroteB };
        for (int k = 0; k < 2; ++k)
        {
            const b3dop::JointFrame &f = *frames[k];
            pH.set(pts[k], UT_Vector3(float(f.pos[0]), float(f.pos[1]), float(f.pos[2])));
            orientH.set(pts[k], UT_QuaternionF(f.rot[0], f.rot[1], f.rot[2], f.rot[3]));
        }
    }
    pH.getAttribute()->bumpDataId();
    orientH.getAttribute()->bumpDataId();
    if (wroteRestLength)
        restLengthH.getAttribute()->bumpDataId();
}

} // namespace

bool
anyJointNetwork(SIM_ObjectArray &objects)
{
    for (exint i = 0; i < objects.entries(); ++i)
    {
        const SIM_Object *obj = objects(i);
        if (SIM_DATA_GETCONST(*obj, jointsDataName(*obj).c_str(), SIM_Geometry))
            return true;
    }
    return false;
}

void
syncJoints(Box3DSharedState &st, SIM_ObjectArray &objects, Box3DJointCounts &counts)
{
    // ---- gather: one network per distinct geometry (the same SOP on several objects is one) ----
    std::vector<NetworkRef>              refs;
    std::unordered_map<int64_t, size_t>  byDetail;
    for (exint i = 0; i < objects.entries(); ++i)
    {
        SIM_Object            *obj  = objects(i);
        const SIM_Box3DJoints *node = nullptr;
        const std::string      name = jointsDataName(*obj, &node);
        const SIM_Geometry    *geo  = SIM_DATA_GETCONST(*obj, name.c_str(), SIM_Geometry);
        if (!geo)
            continue;
        SIM_GeometryAutoReadLock lock(geo);
        const GU_Detail *gdp = lock.getGdp();
        if (!gdp)
            continue;
        const auto seen = byDetail.find(int64_t(gdp->getUniqueId()));
        if (seen != byDetail.end())
        {
            ++refs[seen->second].carriers;
            continue;
        }
        byDetail.emplace(int64_t(gdp->getUniqueId()), refs.size());
        NetworkRef ref;
        ref.carrier  = obj;
        ref.dataName = name;
        ref.node     = node;
        ref.key      = networkKey(obj->getObjectId(), name);
        refs.push_back(ref);
    }
    counts.networks = int(refs.size());

    if (!refs.empty())
    {
        const Anchors anchors(st, objects);
        for (const NetworkRef &ref : refs)
            processNetwork(st, ref, anchors, counts);
    }

    // ---- remove: joints whose primitive or network went away ----
    for (auto it = st.joints.begin(); it != st.joints.end();)
    {
        const Box3DJointEntry &e = it->second;
        const auto net = st.jointNetworks.find(e.network);
        const bool live = net != st.jointNetworks.end() && net->second.visited == st.syncStamp &&
                          e.stamp == net->second.liveStamp;
        if (live)
        {
            ++it;
            continue;
        }
        if (st.world.isJointValid(e.handle))
        {
            st.world.destroyJoint(e.handle);
            ++counts.removed;
        }
        it = st.joints.erase(it);
    }
    for (auto it = st.jointNetworks.begin(); it != st.jointNetworks.end();)
        it = (it->second.visited == st.syncStamp) ? std::next(it) : st.jointNetworks.erase(it);
}

int
jointsAfterStep(Box3DSharedState &st, SIM_ObjectArray &objects)
{
    b3dop::Box3DWorld &world = st.world;

    // ---- breaking: read the events before destroying anything ----
    std::vector<void *> events;
    world.brokenJoints(events);
    std::unordered_map<uint64_t, std::vector<Box3DJointEntry *>> brokeIn;     // network -> joints
    int broke = 0;
    for (void *p : events)
    {
        Box3DJointEntry *e = static_cast<Box3DJointEntry *>(p);
        if (!e || !e->handle)
            continue;
        world.destroyJoint(e->handle);
        e->handle = 0;
        e->broken = true;
        brokeIn[e->network].push_back(e);
        ++broke;
    }

    // ---- write-back, for networks read from one object ----
    std::unordered_map<uint64_t, std::vector<Box3DJointEntry *>> entriesOf;
    bool entriesGathered = false;
    for (auto &kv : st.jointNetworks)
    {
        Box3DJointNetwork &net = kv.second;
        const auto broken = brokeIn.find(kv.first);
        const bool anyBroken = broken != brokeIn.end();
        if (net.visited != st.syncStamp || net.carriers != 1)
            continue;
        if (!entriesGathered)
        {
            entriesGathered = true;
            for (auto &je : st.joints)
                entriesOf[je.second.network].push_back(&je.second);
        }
        const auto ents = entriesOf.find(kv.first);
        if (ents == entriesOf.end() && !anyBroken)
            continue;                           // no joints to keep live, nothing broke
        SIM_Object *obj = objectById(objects, net.carrier);
        SIM_GeometryCopy *geo = obj ? writableGeometry(*obj, net.dataName.c_str()) : nullptr;
        if (!geo)
            continue;
        static const std::vector<Box3DJointEntry *> kNone;
        const std::vector<Box3DJointEntry *> &entries = ents != entriesOf.end() ? ents->second : kNone;
        {
            SIM_GeometryAutoWriteLock lock(geo);
            GU_Detail *gdp = &lock.getGdp();
            auto primOk = [&](GA_Offset off) {
                return off >= 0 && off < gdp->getNumPrimitiveOffsets() && gdp->getPrimitive(off) != nullptr;
            };
            if (anyBroken)
            {
                GA_PrimitiveGroup *grp = gdp->findPrimitiveGroup(kBrokenGroup);
                if (!grp)
                    grp = gdp->newPrimitiveGroup(kBrokenGroup);
                if (grp)
                {
                    for (const Box3DJointEntry *e : broken->second)
                        if (primOk(e->prim))
                            grp->addOffset(e->prim);
                    grp->bumpDataId();
                }
            }
            writeLiveFrames(world, gdp, entries, primOk);
            if (net.writeFeedback)
            {
                auto vecAttr = [&](const char *name) {
                    if (!gdp->findPrimitiveAttribute(name))
                        gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, name, 3);
                    return GA_RWHandleV3(gdp, GA_ATTRIB_PRIMITIVE, name);
                };
                std::unordered_map<std::string, GA_RWHandleF> scalars;
                auto scalar = [&](const char *name) -> GA_RWHandleF & {
                    auto it = scalars.find(name);
                    if (it == scalars.end())
                    {
                        if (!gdp->findPrimitiveAttribute(name))
                            gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, name, 1);
                        it = scalars.emplace(name, GA_RWHandleF(gdp, GA_ATTRIB_PRIMITIVE, name)).first;
                    }
                    return it->second;
                };
                GA_RWHandleV3 forceH  = vecAttr("force");
                GA_RWHandleV3 torqueH = vecAttr("torque");
                GA_RWHandleF &linH    = scalar("linear_separation");
                GA_RWHandleF &angH    = scalar("angular_separation");
                for (const Box3DJointEntry *e : entries)
                {
                    b3dop::JointFeedback fb;
                    if (!primOk(e->prim) || !world.jointFeedback(e->handle, e->type, fb))
                        continue;
                    if (forceH.isValid())  forceH.set(e->prim, UT_Vector3(fb.force[0], fb.force[1], fb.force[2]));
                    if (torqueH.isValid()) torqueH.set(e->prim, UT_Vector3(fb.torque[0], fb.torque[1], fb.torque[2]));
                    if (linH.isValid())    linH.set(e->prim, fb.linearSeparation);
                    if (angH.isValid())    angH.set(e->prim, fb.angularSeparation * kRadToDeg);
                    for (int k = 0; k < fb.extraCount; ++k)
                    {
                        GA_RWHandleF &h = scalar(fb.extraName[k]);
                        if (h.isValid())
                            h.set(e->prim, fb.extraIsAngle[k] ? fb.extra[k] * kRadToDeg : fb.extra[k]);
                    }
                }
                if (forceH.isValid())  forceH.getAttribute()->bumpDataId();
                if (torqueH.isValid()) torqueH.getAttribute()->bumpDataId();
                for (auto &sc : scalars)
                    if (sc.second.isValid())
                        sc.second.getAttribute()->bumpDataId();
            }
        }
        // Our own write is not an edit: re-record the fingerprint (the lock is released).
        if (const SIM_Geometry *rgeo = SIM_DATA_GETCONST(*obj, net.dataName.c_str(), SIM_Geometry))
        {
            SIM_GeometryAutoReadLock rlock(rgeo);
            if (const GU_Detail *rgdp = rlock.getGdp())
                net.fingerprint = networkFingerprint(rgdp);
        }
    }
    return broke;
}
