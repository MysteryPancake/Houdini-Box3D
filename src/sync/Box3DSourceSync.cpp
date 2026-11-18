// Box3DSourceSync.cpp
//
// Animated bodies follow their source SOP, as RBD Bullet Solver's "Updating
// Animated and Deforming Objects". An object made by the Box3D Packed Object
// (or RBD Packed Object) carries SourceObject data whose `objpath` is the SOP
// it was made from. Before the sync, each kinematic body (a point with
// `animated`, or every point of a Kinematic Box3D Data) is moved to the pose
// its piece has in that SOP at the step's end time: P and `orient` are written
// onto the Geometry the way an edit would be, and the sync turns the change
// into a kinematic target. Pieces are matched by their `name`.
#include "sync/Box3DSolverInternal.h"

#include <GA/GA_Handle.h>
#include <GU/GU_Detail.h>
#include <OP/OP_Context.h>
#include <OP/OP_Director.h>
#include <OP/OP_Node.h>
#include <SOP/SOP_Node.h>
#include <UT/UT_StringHolder.h>

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

namespace box3dsolver {

namespace {

constexpr const char *kSourceDataName = "SourceObject";

// A piece to move: its point in the object's Geometry and the pose it has now.
struct Follower {
  GA_Offset off;
  std::string name;
  float rest[4]; // the rotation of its restxform
};

// The source SOP of `obj`, or null; `useTransform` is whether the SOP's object
// transform applies (the data's `usetransform`, written by the Box3D Packed
// Object).
SOP_Node *findSourceSop(const SIM_Object &obj, bool &useTransform) {
  const SIM_Data *source = obj.getConstNamedSubData(kSourceDataName);
  if (!source)
    return nullptr;
  UT_StringHolder path;
  if (!box3dDataOption(*source, "objpath", path) || !path.isstring())
    return nullptr;
  fpreal64 use = 0.0;
  useTransform = box3dDataOption(*source, "usetransform", use) && use != 0.0;
  OP_Node *node = OPgetDirector()->findNode(path.c_str());
  return node ? node->castToSOPNode() : nullptr;
}

// The pose a source piece has: its position and rotation through `xf` (the
// SOP object's world transform, or identity).
bool sourcePose(const GU_Detail *src, GA_Offset off, const UT_Matrix4D &xf,
                double pos[3], float rot[4]) {
  const PackedInfo pk = getPackedInfo(src, off);
  if (!pk.prim)
    return false;
  UT_Matrix4D m;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      m(r, c) = pk.m4[r * 4 + c];
  m *= xf;
  UT_Vector3D p(pk.P[0], pk.P[1], pk.P[2]);
  p *= xf;
  pos[0] = p.x();
  pos[1] = p.y();
  pos[2] = p.z();
  double m3[9], scale[3];
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      m3[r * 3 + c] = m(r, c);
  return splitPackedMatrix(m3, rot, scale);
}

// The poses the pieces in `names` have in `sop` at global time `t`, by name.
void readSourcePoses(SOP_Node &sop, double t, bool useTransform,
                     const std::unordered_map<std::string, bool> &names,
                     std::unordered_map<std::string, Box3DSourcePose> &out) {
  OP_Context context(t);
  const GU_Detail *src = sop.getCookedGeo(context);
  if (!src)
    return;
  UT_Matrix4D xf(1.0);
  if (useTransform && sop.getCreator())
    sop.getCreator()->getWorldTransform(xf, context);
  GA_ROHandleS srcNames(src, GA_ATTRIB_POINT, "name");
  if (!srcNames.isValid())
    return;
  GA_Offset off;
  GA_FOR_ALL_PTOFF(src, off) {
    const std::string name = srcNames.get(off).c_str();
    if (names.find(name) == names.end())
      continue;
    Box3DSourcePose pose;
    if (sourcePose(src, off, xf, pose.pos, pose.rot))
      out[name] = pose;
  }
}

bool samePose(const Box3DSourcePose &a, const Box3DSourcePose &b) {
  for (int i = 0; i < 3; ++i)
    if (std::fabs(a.pos[i] - b.pos[i]) > 1e-6)
      return false;
  return !b3dop::quatsDiffer(a.rot, b.rot, 1e-6);
}

} // namespace

void followSourceSops(Box3DSharedState &st, SIM_ObjectArray &objects,
                      double now, double dt) {
  for (exint i = 0; i < objects.entries(); ++i) {
    SIM_Object &obj = *objects(i);
    bool useTransform = false;
    SOP_Node *sop = findSourceSop(obj, useTransform);
    if (!sop)
      continue;

    // The pieces that move: with `animated`, or all of a Kinematic Box3D Data.
    const SIM_Box3DData *dataNode =
        SIM_DATA_GETCONST(obj, kBox3DDataName, SIM_Box3DData);
    const bool allKinematic =
        dataNode && static_cast<int>(dataNode->getBodyType()) == 1;
    std::vector<Follower> followers;
    {
      const SIM_Geometry *geo =
          SIM_DATA_GETCONST(obj, SIM_GEOMETRY_DATANAME, SIM_Geometry);
      if (!geo)
        continue;
      SIM_GeometryAutoReadLock lock(geo);
      const GU_Detail *gdp = lock.getGdp();
      if (!gdp)
        continue;
      GA_ROHandleS names(gdp, GA_ATTRIB_POINT, "name");
      GA_ROHandleI animated(gdp, GA_ATTRIB_POINT, "animated");
      GA_ROHandleM4 restxform(gdp, GA_ATTRIB_POINT, "restxform");
      GA_ROHandleQ orient(gdp, GA_ATTRIB_POINT, "orient");
      if (!names.isValid() || !restxform.isValid() || !orient.isValid() ||
          (!allKinematic && !animated.isValid()))
        continue;
      GA_Offset off;
      GA_FOR_ALL_PTOFF(gdp, off) {
        if (!allKinematic && animated.get(off) == 0)
          continue;
        Follower f;
        f.off = off;
        f.name = names.get(off).c_str();
        if (!restRotation(restxform.get(off), f.rest))
          continue;
        followers.push_back(std::move(f));
      }
    }
    if (followers.empty())
      continue;
    std::unordered_map<std::string, bool> wanted;
    for (const Follower &f : followers)
      wanted.emplace(f.name, true);

    // What each piece had when last read; pieces seen for the first time take
    // the SOP a step earlier.
    const SIM_Engine &engine = obj.getEngine();
    auto &known = st.sourcePoses[obj.getObjectId()];
    std::unordered_map<std::string, bool> unseen;
    for (const Follower &f : followers)
      if (known.find(f.name) == known.end())
        unseen.emplace(f.name, true);
    if (!unseen.empty()) {
      std::unordered_map<std::string, Box3DSourcePose> before;
      readSourcePoses(*sop, engine.getGlobalTime(SIM_Time(now - dt)),
                      useTransform, unseen, before);
      for (const auto &kv : before)
        known[kv.first] = kv.second;
    }
    std::unordered_map<std::string, Box3DSourcePose> current;
    readSourcePoses(*sop, engine.getGlobalTime(SIM_Time(now)), useTransform,
                    wanted, current);

    std::vector<std::pair<const Follower *, const Box3DSourcePose *>> moves;
    for (const Follower &f : followers) {
      auto it = current.find(f.name);
      if (it == current.end())
        continue;
      auto was = known.find(f.name);
      const bool moved =
          was != known.end() && !samePose(was->second, it->second);
      known[f.name] = it->second;
      if (moved)
        moves.emplace_back(&f, &it->second);
    }
    if (moves.empty())
      continue;

    SIM_GeometryCopy *geoCopy =
        box3dWritableGeometry(obj, SIM_GEOMETRY_DATANAME);
    if (!geoCopy)
      continue;
    SIM_GeometryAutoWriteLock lock(geoCopy);
    GU_Detail *gdp = &lock.getGdp();
    GA_RWHandleQ orient(gdp, GA_ATTRIB_POINT, "orient");
    if (!orient.isValid())
      continue;
    for (const auto &mv : moves) {
      const Follower &f = *mv.first;
      const Box3DSourcePose &pose = *mv.second;
      float since[4];
      box3dAbsoluteToRest(f.rest, pose.rot, since);
      b3dop::sanitizeQuat(since);
      gdp->setPos3(f.off, UT_Vector3(static_cast<float>(pose.pos[0]),
                                     static_cast<float>(pose.pos[1]),
                                     static_cast<float>(pose.pos[2])));
      orient.set(f.off, UT_QuaternionF(since[0], since[1], since[2], since[3]));
    }
    if (GA_Attribute *p = gdp->getP())
      p->bumpDataId();
    orient.getAttribute()->bumpDataId();
  }
}

} // namespace box3dsolver
