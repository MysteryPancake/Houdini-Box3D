// Box3DJointSync.cpp
#include "Box3DJointSync.h"
#include "core/Box3DDebug.h"
#include "dop/SIM_Box3DConRel.h"
#include "dop/SIM_Box3DWorld.h"
#include "Box3DHoudiniUtil.h"

#include <GA/GA_Handle.h>
#include <GA/GA_Iterator.h>
#include <GA/GA_Primitive.h>
#include <GU/GU_Detail.h>
#include <SIM/SIM_ConstraintNetworkRelationship.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_Relationship.h>
#include <SIM/SIM_RelationshipArray.h>
#include <UT/UT_Quaternion.h>
#include <UT/UT_StringHolder.h>
#include <UT/UT_Vector3.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

// The Constraint Network DOP's attribute and group names.
const char *const kConstraintName     = "constraint_name";
const char *const kConstraintType     = "constraint_type";     // SideFX data only
const char *const kNextConstraintName = "next_constraint_name";
const char *const kBrokenGroup        = "broken";
// With Attach Internal Constraints to Object, the network is this data on the object itself.
const char *const kInternalGeometry   = "ConstraintGeometry";
// Ours: the length a distance joint made with a negative Rest Length started at.
const char *const kAutoRestLength     = "auto_restlength";

uint64_t
hashString(const std::string &s)
{
    return b3dop::hashBytes(b3dop::kHashSeed, s.data(), s.size());
}

std::string
toString(const UT_String &s)
{
    return s.isstring() ? std::string(s.buffer()) : std::string();
}

// The Constraint Network data a relationship carries, or null for other relationships.
const SIM_ConstraintNetworkRelationship *
networkData(const SIM_Relationship *rel)
{
    const SIM_Data *type = rel ? rel->getConstRelationshipTypeData() : nullptr;
    return type ? SIM_DATA_CASTCONST(type, SIM_ConstraintNetworkRelationship) : nullptr;
}

// Where a network was found: a relationship of `object` (by name), or, with an empty name, the
// object's ConstraintGeometry.
struct NetworkSource
{
    bool        inGroup = true;
    std::string relName;
};

const SIM_Relationship *
findRelationship(const SIM_Object &obj, const NetworkSource &src)
{
    return src.relName.empty() ? nullptr : obj.getRelationship(src.inGroup, src.relName.c_str());
}

const SIM_Geometry *
networkGeometry(const SIM_Object &obj, const NetworkSource &src)
{
    if (src.relName.empty())
        return SIM_DATA_GETCONST(obj, kInternalGeometry, SIM_Geometry);
    const SIM_ConstraintNetworkRelationship *cn = networkData(findRelationship(obj, src));
    return cn ? cn->getGeometry() : nullptr;
}

SIM_GeometryCopy *
writableNetworkGeometry(SIM_Object &obj, const NetworkSource &src)
{
    if (src.relName.empty())
        return box3dWritableGeometry(obj, kInternalGeometry);
    SIM_Relationship *rel = obj.getRelationships(src.inGroup).findRelationshipByName(src.relName.c_str());
    SIM_Data *type = rel ? rel->getRelationshipTypeData() : nullptr;
    SIM_ConstraintNetworkRelationship *cn = type ? SIM_DATA_CAST(type, SIM_ConstraintNetworkRelationship) : nullptr;
    return cn ? cn->getGeometry() : nullptr;
}

// A SideFX constraint data's options (b3dop::sideFXParams), over Houdini's defaults. Their classes
// are not in the HDK: read by name (box3dDataOption).
b3dop::SideFXSettings
readSideFXSettings(const SIM_Data &data, b3dop::SideFXType type)
{
    using b3dop::SideFXParam;
    b3dop::SideFXSettings s = b3dop::defaultSideFXSettings(type);
    for (const SideFXParam &p : b3dop::sideFXParams())
    {
        if (!p.usedBy(type))
            continue;
        if (p.kind == SideFXParam::Vec3)
        {
            UT_Vector3D v;
            if (box3dDataOption(data, p.name, v))
                for (int k = 0; k < 3; ++k)
                    (s.*p.v)[k] = float(v(k));
            continue;
        }
        fpreal64 v = 0.0;
        if (!box3dDataOption(data, p.name, v))
            continue;
        if (p.kind == SideFXParam::Float)
            s.*p.f = float(v);
        else
            s.*p.i = int(v);
    }
    return s;
}

// The Box3D constraint relationships a network's `constraint_name` can refer to, by data name.
// The Constraint Network DOP attaches the nodes wired into its second input under the network's
// relationship; the relationship, its Constraint Network data and the geometry are all searched
// (the first name found wins), and for an internal network the geometry and the object. SideFX's
// Glue / Hard / Spring / Bullet Soft / Cone Twist data count too (mapped per primitive).
std::unordered_map<std::string, Box3DJointKind>
networkKinds(const SIM_Object &obj, const NetworkSource &src)
{
    std::vector<const SIM_Data *> roots;
    if (src.relName.empty())
    {
        roots.push_back(obj.getConstNamedSubData(kInternalGeometry));
        roots.push_back(&obj);
    }
    else if (const SIM_Relationship *rel = findRelationship(obj, src))
    {
        roots.push_back(rel);
        const SIM_ConstraintNetworkRelationship *cn = networkData(rel);
        roots.push_back(cn);
        roots.push_back(cn ? cn->getGeometry() : nullptr);
    }

    std::unordered_map<std::string, Box3DJointKind> kinds;
    for (const SIM_Data *root : roots)
    {
        if (!root)
            continue;
        for (int i = 0; i < root->getNumSubData(); ++i)
        {
            const SIM_Data *sub = root->getConstSubData(i);
            if (!sub)
                continue;
            const SIM_Box3DConRel *conrel = SIM_DATA_CASTCONST(sub, SIM_Box3DConRel);
            b3dop::SideFXType sideFXType = b3dop::SideFXType::Glue;
            const bool sideFX = !conrel && b3dop::sideFXTypeFromDataType(sub->getDataType().c_str(), sideFXType);
            if (!conrel && !sideFX)
                continue;
            const UT_StringHolder &name = root->getSubDataName(i);
            if (!name.isstring() || kinds.count(std::string(name.c_str())))
                continue;
            Box3DJointKind k;
            if (conrel)
            {
                k.type     = conrel->jointType();
                k.settings = conrel->settings();
            }
            else
            {
                k.sideFX         = true;
                k.sideFXType     = sideFXType;
                k.sideFXSettings = readSideFXSettings(*sub, sideFXType);
            }
            kinds.emplace(std::string(name.c_str()), k);
        }
    }
    return kinds;
}

bool
sameKinds(const std::unordered_map<std::string, Box3DJointKind> &a,
          const std::unordered_map<std::string, Box3DJointKind> &b)
{
    if (a.size() != b.size())
        return false;
    for (const auto &kv : a)
    {
        const auto it = b.find(kv.first);
        if (it == b.end() || it->second.sideFX != kv.second.sideFX)
            return false;
        if (kv.second.sideFX)
        {
            if (it->second.sideFXType != kv.second.sideFXType ||
                !b3dop::sameSideFXSettings(kv.second.sideFXType, kv.second.sideFXSettings, it->second.sideFXSettings))
                return false;
        }
        else if (it->second.type != kv.second.type ||
                 !b3dop::sameJointSettings(kv.second.type, kv.second.settings, it->second.settings))
            return false;
    }
    return true;
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
    h = mixHash(h, idOf(gdp->findPrimitiveAttribute(kConstraintName)));
    h = mixHash(h, idOf(gdp->findPrimitiveAttribute(kAutoRestLength)));
    h = mixHash(h, idOf(gdp->findPrimitiveAttribute(kConstraintType)));
    for (const b3dop::JointParam &p : b3dop::jointParams())
        h = mixHash(h, idOf(gdp->findPrimitiveAttribute(p.name)));
    for (const b3dop::SideFXParam &p : b3dop::sideFXParams())
        h = mixHash(h, idOf(gdp->findPrimitiveAttribute(p.name)));
    const GA_PrimitiveGroup *broken = gdp->findPrimitiveGroup(kBrokenGroup);
    return mixHash(h, broken ? uint64_t(broken->getDataId()) : ~0ull);
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

// The primitive's attributes on top of its constraint data's settings, as the Constraint Network
// DOP documents: a float attribute multiplies the setting (angles too: it is a factor, not
// degrees), any other kind replaces it (vectors and Euler angles in Houdini units).
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
        if (p.kind == JointParam::Float || p.kind == JointParam::Angle)
        {
            s.*p.f *= r.f.get(prim);
            continue;
        }
        float v[3] = { 0, 0, 0 };
        if (p.kind == JointParam::Toggle)
            v[0] = float(r.i.get(prim));
        else
        {
            const UT_Vector3 a = r.v.get(prim);
            v[0] = a.x();  v[1] = a.y();  v[2] = a.z();
        }
        b3dop::setJointParam(s, p, type, v, box3dEulerToQuat);
    }
}

// The primitive attributes that modify a SideFX data's options (b3dop::sideFXParams), bound once
// per network.
struct SideFXReader
{
    const b3dop::SideFXParam *param = nullptr;
    GA_ROHandleF              f;
    GA_ROHandleI              i;
    GA_ROHandleV3             v;
};

std::vector<SideFXReader>
bindSideFX(const GU_Detail *gdp)
{
    using b3dop::SideFXParam;
    std::vector<SideFXReader> out;
    for (const SideFXParam &p : b3dop::sideFXParams())
    {
        if (!gdp->findPrimitiveAttribute(p.name))
            continue;
        SideFXReader r;
        r.param = &p;
        bool ok = false;
        switch (p.kind)
        {
            case SideFXParam::Float: r.f = GA_ROHandleF(gdp, GA_ATTRIB_PRIMITIVE, p.name);  ok = r.f.isValid(); break;
            case SideFXParam::Int:   r.i = GA_ROHandleI(gdp, GA_ATTRIB_PRIMITIVE, p.name);  ok = r.i.isValid(); break;
            case SideFXParam::Vec3:  r.v = GA_ROHandleV3(gdp, GA_ATTRIB_PRIMITIVE, p.name); ok = r.v.isValid(); break;
        }
        if (ok)
            out.push_back(r);
    }
    return out;
}

// The Constraint Network DOP's rule on SideFX data too: floats multiply, the rest replace.
void
applySideFXOverrides(const std::vector<SideFXReader> &readers, b3dop::SideFXType type, GA_Offset prim,
                     b3dop::SideFXSettings &s)
{
    using b3dop::SideFXParam;
    for (const SideFXReader &r : readers)
    {
        const SideFXParam &p = *r.param;
        if (!p.usedBy(type))
            continue;
        switch (p.kind)
        {
            case SideFXParam::Float: s.*p.f *= r.f.get(prim); break;
            case SideFXParam::Int:   s.*p.i = r.i.get(prim);  break;
            case SideFXParam::Vec3:
            {
                const UT_Vector3 a = r.v.get(prim);
                (s.*p.v)[0] = a.x();  (s.*p.v)[1] = a.y();  (s.*p.v)[2] = a.z();
                break;
            }
        }
    }
}

// A body as the SideFX mapping needs it: Spring converts its stiffness from the masses.
b3dop::SideFXBody
sideFXBody(const b3dop::Box3DWorld &world, b3dop::BodyHandle h)
{
    b3dop::SideFXBody body;
    if (!h || world.typeByHandle(h) != b3dop::BodySettings::Type::Dynamic)
        return body;                            // the world, static or kinematic
    float center[3], inertia[3];
    if (world.getMassDataByHandle(h, body.mass, center, inertia))
    {
        body.dynamic = body.mass > 0.0f;
        body.inertia = (inertia[0] + inertia[1] + inertia[2]) / 3.0f;
    }
    return body;
}

// What decides where a SideFX joint's frames sit (not the anchors' positions: those are live).
uint64_t
sideFXFrameSignature(const b3dop::SideFXJoint &m)
{
    uint64_t h = b3dop::hashBytes(b3dop::kHashSeed, m.rotA, sizeof(m.rotA));
    h = b3dop::hashBytes(h, m.rotB, sizeof(m.rotB));
    h = b3dop::hashBytes(h, &m.separation, sizeof(m.separation));
    return b3dop::mixHash(h, m.holdSeparation ? 1 : 0);
}

// A joint frame from its anchor and offset, and back. Identity offsets (Box3D nodes) pass the
// frame through untouched, bit for bit.
b3dop::JointFrame
jointFrameOf(const b3dop::JointFrame &anchor, const b3dop::JointFrame &adj)
{
    return b3dop::isIdentityOffset(adj) ? anchor : b3dop::applyFrameOffset(anchor, adj);
}

b3dop::JointFrame
anchorFrameOf(const b3dop::JointFrame &joint, const b3dop::JointFrame &adj)
{
    return b3dop::isIdentityOffset(adj) ? joint : b3dop::removeFrameOffset(joint, adj);
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
    return b3dop::quatsDiffer(now.rot, wrote.rot);
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

// What processNetwork() reads from a network's geometry, bound once.
struct NetworkReader
{
    const GU_Detail         *gdp;
    const GA_ROHandleS       conName, conType, name;
    const GA_ROHandleF       autoRest;
    const GA_ROHandleQ       orient, localQ;
    const GA_ROHandleV3      localP;
    const GA_PrimitiveGroup *broken;

    explicit NetworkReader(const GU_Detail *g)
        : gdp(g),
          conName(g, GA_ATTRIB_PRIMITIVE, kConstraintName),
          conType(g, GA_ATTRIB_PRIMITIVE, kConstraintType),
          name(g, GA_ATTRIB_POINT, "name"),
          autoRest(g, GA_ATTRIB_PRIMITIVE, kAutoRestLength),
          orient(g, GA_ATTRIB_POINT, "orient"),
          localQ(g, GA_ATTRIB_POINT, "local_orient"),
          localP(g, GA_ATTRIB_POINT, "local_P"),
          broken(g->findPrimitiveGroup(kBrokenGroup))
    {
    }

    std::string anchorName(GA_Offset pt) const
    {
        const char *n = name.isValid() ? static_cast<const char *>(name.get(pt)) : nullptr;
        return n ? std::string(n) : std::string();
    }

    // The anchor's P / orient: its frame in world space.
    b3dop::JointFrame frame(GA_Offset pt) const
    {
        b3dop::JointFrame f;
        const UT_Vector3 p = gdp->getPos3(pt);
        f.pos[0] = p.x();  f.pos[1] = p.y();  f.pos[2] = p.z();
        if (orient.isValid())
        {
            const UT_QuaternionF q = orient.get(pt);
            f.rot[0] = q.x();  f.rot[1] = q.y();  f.rot[2] = q.z();  f.rot[3] = q.w();
        }
        return f;
    }

    // The primitive's constraint_type (SideFX data only; missing: all).
    b3dop::ConstraintMode mode(GA_Offset prim) const
    {
        const char *t = conType.isValid() ? static_cast<const char *>(conType.get(prim)) : nullptr;
        return b3dop::constraintModeFromString(t);
    }

    // The primitive's constraint data (by `constraint_name`), or null.
    const Box3DJointKind *kind(GA_Offset prim, const std::unordered_map<std::string, Box3DJointKind> &kinds) const
    {
        const char *n = conName.isValid() ? static_cast<const char *>(conName.get(prim)) : nullptr;
        if (!n || !*n)
            return nullptr;
        const auto it = kinds.find(std::string(n));
        return it == kinds.end() ? nullptr : &it->second;
    }

    // The anchor's saved local frame (local_P / local_orient), if it has one (a zero quaternion: none).
    bool savedLocal(GA_Offset pt, b3dop::JointFrame &out) const
    {
        if (!localP.isValid() || !localQ.isValid())
            return false;
        const UT_Vector3     p = localP.get(pt);
        const UT_QuaternionF q = localQ.get(pt);
        if ((q.x() == 0.0f && q.y() == 0.0f && q.z() == 0.0f && q.w() == 0.0f) ||
            !std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z()))
            return false;
        out.pos[0] = p.x();  out.pos[1] = p.y();  out.pos[2] = p.z();
        out.rot[0] = q.x();  out.rot[1] = q.y();  out.rot[2] = q.z();  out.rot[3] = q.w();
        return true;
    }
};

// A saved local frame still describes its anchor: through the body's pose now it lands on the
// anchor's P / orient (written together after the same step). A renamed, moved or new anchor
// does not, and is made from P / orient instead.
bool
localFrameAgrees(const b3dop::Box3DWorld &world, b3dop::BodyHandle body, const b3dop::JointFrame &local,
                 const b3dop::JointFrame &live)
{
    b3dop::JointFrame w;
    if (!world.localToWorld(body, local, w))
        return false;
    for (int k = 0; k < 3; ++k)
        if (std::fabs(w.pos[k] - live.pos[k]) > 1e-5 * std::max(1.0, std::fabs(live.pos[k])))
            return false;
    return !b3dop::quatsDiffer(w.rot, live.rot);
}

struct NetworkRef
{
    SIM_Object   *object = nullptr;         // an object of this solve the network was found on
    NetworkSource source;
    uint64_t      key    = 0;
};

void
processNetwork(Box3DSharedState &st, const NetworkRef &ref, const Anchors &anchors, float timeStep,
               Box3DJointCounts &counts)
{
    using b3dop::JointType;
    b3dop::Box3DWorld &world = st.world;

    Box3DJointNetwork &net = st.jointNetworks[ref.key];
    net.visited  = st.syncStamp;
    net.objectId = ref.object->getObjectId();
    net.relName  = ref.source.relName;
    net.inGroup  = ref.source.inGroup;

    // The constraint data the network can name; any change re-reads every primitive.
    std::unordered_map<std::string, Box3DJointKind> kinds = networkKinds(*ref.object, ref.source);
    const bool kindsChanged = !net.kindsKnown || !sameKinds(kinds, net.kinds);
    if (kindsChanged)
        net.kinds = std::move(kinds);
    net.kindsKnown = true;

    const SIM_Geometry *geo = networkGeometry(*ref.object, ref.source);
    if (!geo)
        return;
    SIM_GeometryAutoReadLock lock(geo);
    const GU_Detail *gdp = lock.getGdp();
    if (!gdp)
        return;

    // Unchanged geometry, bodies and constraint data: the joints stay as they are (they keep liveStamp).
    const uint64_t fp = networkFingerprint(gdp);
    if (fp == net.fingerprint && net.bodyVersion == st.bodyVersion && !kindsChanged)
        return;
    net.fingerprint = fp;
    net.bodyVersion = st.bodyVersion;
    net.liveStamp   = st.syncStamp;

    const NetworkReader              rd(gdp);
    const std::vector<SettingReader> readers   = bindSettings(gdp);
    const std::vector<SideFXReader>  sfReaders = bindSideFX(gdp);

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
        const Box3DJointKind *kind = rd.kind(off, net.kinds);
        if (!kind)
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
        if (e.broken || (rd.broken && rd.broken->containsOffset(off)))
        {
            e.broken = true;
            drop();
            e.stamp = net.liveStamp;
            continue;
        }

        const GA_Offset   p0 = prim->getPointOffset(0), p1 = prim->getPointOffset(1);
        b3dop::BodyHandle a = 0, b = 0;
        if (!anchors.resolve(rd.anchorName(p0), a) || !anchors.resolve(rd.anchorName(p1), b))
        {
            ++counts.unresolved;
            drop();
            continue;                           // not stamped: removed; retried when bodies change
        }
        // P / orient are live on a written network (where the pieces are now), so a world rebuilt
        // mid-simulation makes its joints from them exactly as frame 1 does from the authored ones.
        const b3dop::JointFrame fa = rd.frame(p0), fb = rd.frame(p1);
        JointType            type;
        b3dop::JointSettings s;
        b3dop::JointFrame    adjA, adjB;        // the frames relative to the anchors: identity but for SideFX data
        uint64_t             frameSig = 0;
        if (kind->sideFX)
        {
            // SideFX data: the primitive's overrides on its options, then mapped onto a Box3D joint.
            b3dop::SideFXSettings sf = kind->sideFXSettings;
            applySideFXOverrides(sfReaders, kind->sideFXType, off, sf);
            b3dop::SideFXContext ctx;
            ctx.timeStep = timeStep;
            ctx.a        = sideFXBody(world, a);
            ctx.b        = sideFXBody(world, b);
            const b3dop::SideFXJoint m = b3dop::mapSideFX(kind->sideFXType, sf, rd.mode(off), ctx);
            type     = m.type;
            s        = m.settings;
            frameSig = sideFXFrameSignature(m);
            b3dop::sideFXFrameOffsets(m, fa, fb, adjA, adjB);
        }
        else
        {
            type = kind->type;
            s    = kind->settings;
            applyOverrides(readers, type, off, s);
            // A distance joint that measured its own rest length keeps it (see writeLiveFrames).
            if (type == JointType::Distance && !(s.restLength >= 0.0f) && rd.autoRest.isValid())
            {
                const float made = rd.autoRest.get(off);
                if (made >= 0.0f)
                    s.restLength = made;
            }
        }
        const bool     live   = net.writable;
        uint64_t       sig    = jointSignature(type, a, b, fa, fb, !live);
        if (frameSig)
            sig = b3dop::mixHash(sig, frameSig) | 1;
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
            e.adjA     = adjA;
            e.adjB     = adjB;
            e.madeRestLength = -1.0f;
            if (type == JointType::Distance && !(s.restLength >= 0.0f))
            {
                const double dx = fb.pos[0] - fa.pos[0], dy = fb.pos[1] - fa.pos[1], dz = fb.pos[2] - fa.pos[2];
                e.madeRestLength = float(std::sqrt(dx * dx + dy * dy + dz * dz));
            }
            // Exactly as before from the saved local frames when they still apply (a world rebuilt
            // mid-simulation); else from P / orient, and the local frames are saved after the step.
            // The local frames are the joint's (with any SideFX offset), so they are compared with
            // the frames the anchors give now.
            const b3dop::JointFrame ja = jointFrameOf(fa, adjA), jb = jointFrameOf(fb, adjB);
            b3dop::JointFrame la, lb;
            const bool useLocal = !edited && rd.savedLocal(p0, la) && rd.savedLocal(p1, lb) && localFrameAgrees(world, a, la, ja) &&
                                  localFrameAgrees(world, b, lb, jb);
            e.handle     = useLocal ? world.createJoint(type, a, b, la, lb, s, &e, true)
                                    : world.createJoint(type, a, b, ja, jb, s, &e);
            e.writeLocal = !useLocal;
            if (e.handle)
                ++counts.created;
            else
                ++counts.failed;
        }
        e.stamp = net.liveStamp;
    }
}

// After the step, for a written network: every live joint's anchors get P / orient set to where
// its frames are now, and joints made from P / orient save their local frames to local_P /
// local_orient (relative to their pieces: exact on a rebuild, wherever the pieces are). A distance
// joint made with a negative rest length gets its starting distance written to `auto_restlength`
// (not `restlength`, which multiplies the node's value), so a world rebuilt mid-simulation keeps it.
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
    // A new quaternion attribute is 0,0,0,0: start every point at identity, as the solver's
    // ensureResultAttribs does.
    if (orientNew)
        for (GA_Iterator it(gdp->getPointRange()); !it.atEnd(); ++it)
            orientH.set(*it, UT_QuaternionF(0.0f, 0.0f, 0.0f, 1.0f));
    bool wroteRestLength = false, wroteLocal = false;
    GA_RWHandleF restLengthH;
    GA_RWHandleV3 localPH;
    GA_RWHandleQ  localQH;

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
                if (!gdp->findPrimitiveAttribute(kAutoRestLength))
                    gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, kAutoRestLength, 1, GA_Defaults(-1.0f));
                restLengthH = GA_RWHandleF(gdp, GA_ATTRIB_PRIMITIVE, kAutoRestLength);
            }
            if (restLengthH.isValid())
            {
                restLengthH.set(e->prim, e->madeRestLength);
                wroteRestLength = true;
            }
            e->madeRestLength = -1.0f;
        }

        if (e->writeLocal)
        {
            b3dop::JointFrame la, lb;
            if (world.jointLocalFrames(e->handle, la, lb))
            {
                if (!localPH.isValid())
                {
                    bool created = false;
                    ensure("local_P", 3, false, created);
                    ensure("local_orient", 4, true, created);
                    localPH = GA_RWHandleV3(gdp, GA_ATTRIB_POINT, "local_P");
                    localQH = GA_RWHandleQ(gdp, GA_ATTRIB_POINT, "local_orient");
                }
                if (localPH.isValid() && localQH.isValid())
                {
                    const b3dop::JointFrame *frames[2] = { &la, &lb };
                    for (int k = 0; k < 2; ++k)
                    {
                        const b3dop::JointFrame &f = *frames[k];
                        localPH.set(pts[k], UT_Vector3(float(f.pos[0]), float(f.pos[1]), float(f.pos[2])));
                        localQH.set(pts[k], UT_QuaternionF(f.rot[0], f.rot[1], f.rot[2], f.rot[3]));
                    }
                    wroteLocal = true;
                }
            }
            e->writeLocal = false;
        }

        b3dop::JointFrame fa, fb;
        if (world.jointWorldFrames(e->handle, fa, fb))
        {
            // The anchors, not the frames (SideFX data may offset the frames from them).
            e->wroteA = asStored(anchorFrameOf(fa, e->adjA));
            e->wroteB = asStored(anchorFrameOf(fb, e->adjB));
            e->wrote  = true;
        }
        else if (!e->switchFrames)
            continue;                           // broken, or never made
        e->switchFrames = false;                // (switched: wroteA / wroteB are set already)
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
    if (wroteLocal)
    {
        localPH.getAttribute()->bumpDataId();
        localQH.getAttribute()->bumpDataId();
    }
}

} // namespace

bool
anyJointNetwork(SIM_ObjectArray &objects)
{
    for (exint i = 0; i < objects.entries(); ++i)
    {
        const SIM_Object *obj = objects(i);
        if (obj->getConstNamedSubData(kInternalGeometry))
            return true;
        for (const bool inGroup : { true, false })
            for (int r = 0; r < obj->getNumRelationships(inGroup); ++r)
                if (networkData(obj->getRelationship(inGroup, r)))
                    return true;
    }
    return false;
}

void
syncJoints(Box3DSharedState &st, SIM_ObjectArray &objects, float timeStep, Box3DJointCounts &counts)
{
    // ---- gather: each Constraint Network relationship once (it is on every object it
    // constrains), and each object's internal ConstraintGeometry ----
    std::vector<NetworkRef>         refs;
    std::unordered_set<std::string> seenRels;
    for (exint i = 0; i < objects.entries(); ++i)
    {
        SIM_Object *obj = objects(i);
        for (const bool inGroup : { true, false })
            for (int r = 0; r < obj->getNumRelationships(inGroup); ++r)
            {
                const SIM_Relationship *rel = obj->getRelationship(inGroup, r);
                if (!networkData(rel))
                    continue;
                NetworkRef ref;
                ref.object          = obj;
                ref.source.inGroup  = inGroup;
                ref.source.relName  = toString(rel->getName());
                if (ref.source.relName.empty() || !seenRels.insert(ref.source.relName).second)
                    continue;
                ref.key = b3dop::mixHash(hashString(ref.source.relName), 1);
                refs.push_back(ref);
            }
        if (obj->getConstNamedSubData(kInternalGeometry))
        {
            NetworkRef ref;
            ref.object = obj;
            ref.key    = b3dop::mixHash(uint64_t(uint32_t(obj->getObjectId())), 2);
            refs.push_back(ref);
        }
    }
    counts.networks = int(refs.size());

    if (!refs.empty())
    {
        const Anchors anchors(st, objects);
        for (const NetworkRef &ref : refs)
            processNetwork(st, ref, anchors, timeStep, counts);
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
        e->brokeLocalKnown = world.jointLocalFrames(e->handle, e->brokeLocalA, e->brokeLocalB);
        world.destroyJoint(e->handle);
        e->handle = 0;
        e->broken = true;
        brokeIn[e->network].push_back(e);
        ++broke;
    }

    // ---- write-back: broken group / next_constraint_name, live frames, feedback ----
    std::unordered_map<uint64_t, std::vector<Box3DJointEntry *>> entriesOf;
    bool entriesGathered = false;
    for (auto &kv : st.jointNetworks)
    {
        Box3DJointNetwork &net = kv.second;
        const auto broken = brokeIn.find(kv.first);
        const bool anyBroken = broken != brokeIn.end();
        if (net.visited != st.syncStamp)
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
        SIM_Object *obj = box3dObjectById(objects, net.objectId);
        NetworkSource src;
        src.inGroup = net.inGroup;
        src.relName = net.relName;
        SIM_GeometryCopy *geo = obj ? writableNetworkGeometry(*obj, src) : nullptr;
        net.writable = geo != nullptr;
        if (!geo)
            continue;
        static const std::vector<Box3DJointEntry *> kNone;
        const std::vector<Box3DJointEntry *> &entries = ents != entriesOf.end() ? ents->second : kNone;
        bool switched = false;
        {
            SIM_GeometryAutoWriteLock lock(geo);
            GU_Detail *gdp = &lock.getGdp();
            auto primOk = [&](GA_Offset off) {
                return off >= 0 && off < gdp->getNumPrimitiveOffsets() && gdp->getPrimitive(off) != nullptr;
            };
            if (anyBroken)
            {
                // A primitive with a next_constraint_name switches to it (its joint is remade as
                // that constraint at the next step, from where the pieces are now) and the
                // attribute is cleared; the rest go into the `broken` group.
                GA_RWHandleS conNameH(gdp, GA_ATTRIB_PRIMITIVE, kConstraintName);
                GA_RWHandleS nextH(gdp, GA_ATTRIB_PRIMITIVE, kNextConstraintName);
                GA_PrimitiveGroup *grp = nullptr;
                for (Box3DJointEntry *e : broken->second)
                {
                    if (!primOk(e->prim))
                        continue;
                    UT_StringHolder nextName;
                    if (nextH.isValid() && conNameH.isValid())
                        nextName = nextH.get(e->prim);
                    if (nextName.isstring())
                    {
                        conNameH.set(e->prim, nextName);
                        nextH.set(e->prim, UT_StringHolder());
                        e->broken = false;
                        switched  = true;
                        // The anchors go where the broken joint's frames are now (below).
                        b3dop::JointFrame fa, fb;
                        if (e->brokeLocalKnown && world.localToWorld(e->bodyA, e->brokeLocalA, fa) &&
                            world.localToWorld(e->bodyB, e->brokeLocalB, fb))
                        {
                            e->wroteA       = asStored(anchorFrameOf(fa, e->adjA));
                            e->wroteB       = asStored(anchorFrameOf(fb, e->adjB));
                            e->wrote        = true;
                            e->switchFrames = true;
                        }
                        continue;
                    }
                    if (!grp)
                    {
                        grp = gdp->findPrimitiveGroup(kBrokenGroup);
                        if (!grp)
                            grp = gdp->newPrimitiveGroup(kBrokenGroup);
                    }
                    if (grp)
                        grp->addOffset(e->prim);
                }
                if (grp)
                    grp->bumpDataId();
                if (switched)
                {
                    conNameH.getAttribute()->bumpDataId();
                    nextH.getAttribute()->bumpDataId();
                }
            }
            writeLiveFrames(world, gdp, entries, primOk);

            // Feedback, under the Constraint Network DOP's names: force / distance for the linear
            // part, torque / angle (radians) for the angular part, plus the per-type readings.
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
            auto length = [](const float *v) {
                return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            };
            GA_RWHandleF &forceH  = scalar("force");
            GA_RWHandleF &distH   = scalar("distance");
            GA_RWHandleF &torqueH = scalar("torque");
            GA_RWHandleF &angleH  = scalar("angle");
            for (const Box3DJointEntry *e : entries)
            {
                b3dop::JointFeedback fb;
                if (!primOk(e->prim) || !world.jointFeedback(e->handle, e->type, fb))
                    continue;
                if (forceH.isValid())  forceH.set(e->prim, length(fb.force));
                if (distH.isValid())   distH.set(e->prim, fb.linearSeparation);
                if (torqueH.isValid()) torqueH.set(e->prim, length(fb.torque));
                if (angleH.isValid())  angleH.set(e->prim, fb.angularSeparation);
                for (int k = 0; k < fb.extraCount; ++k)
                {
                    GA_RWHandleF &h = scalar(fb.extraName[k]);
                    if (h.isValid())
                        h.set(e->prim, fb.extraIsAngle[k] ? fb.extra[k] * b3dop::kRadToDeg : fb.extra[k]);
                }
            }
            for (auto &sc : scalars)
                if (sc.second.isValid())
                    sc.second.getAttribute()->bumpDataId();
        }
        // Our own write is not an edit: re-record the fingerprint (the lock is released). A
        // switched constraint_name is ours too, but must be read: force a re-read.
        if (const SIM_Geometry *rgeo = networkGeometry(*obj, src))
        {
            SIM_GeometryAutoReadLock rlock(rgeo);
            if (const GU_Detail *rgdp = rlock.getGdp())
                net.fingerprint = networkFingerprint(rgdp);
        }
        if (switched)
            net.fingerprint = 0;
    }
    return broke;
}
