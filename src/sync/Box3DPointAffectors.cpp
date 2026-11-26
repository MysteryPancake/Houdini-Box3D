// Box3DPointAffectors.cpp
//
// An affector with points and no primitives (POP particles, as RBD Bullet
// Solver takes them): every point is a sphere of radius `pscale` at its
// position, followed as a kinematic body so a moving particle pushes what it
// touches. Particles are told apart by `id` (else the point number); a body is
// made when a particle is first seen, moved by a target pose each step and
// destroyed when the particle is gone.
#include "Box3DAffectors.h"
#include "core/Box3DDebug.h"
#include "core/Box3DMath.h"
#include "dop/SIM_Box3DWorld.h"

#include <GA/GA_Handle.h>
#include <GA/GA_Iterator.h>
#include <GU/GU_Detail.h>

#include <algorithm>
#include <cmath>

namespace {

// Box3D has no sphere of no size; a particle without `pscale` is a point to
// Bullet, so it gets a sphere of this radius.
constexpr float kMinPointRadius = 0.005f;

} // namespace

void clearPointSpheres(Box3DSharedState &st, Box3DAffectorBody &ab) {
  for (auto &kv : ab.spheres)
    if (kv.second.handle)
      st.world.destroyBody(kv.second.handle);
  ab.spheres.clear();
}

void syncPointSpheres(Box3DSharedState &st, Box3DAffectorBody &ab,
                      const GU_Detail *gdp, const double m[16],
                      const b3dop::BodySettings &material,
                      Box3DAffectorCounts &counts) {
  const uint64_t stamp = ++ab.sphereStamp;
  b3dop::BodySettings body = material;
  body.type = b3dop::BodySettings::Type::Kinematic;

  const GA_ROHandleF pscale(gdp, GA_ATTRIB_POINT, "pscale");
  const GA_ROHandleID ids(gdp, GA_ATTRIB_POINT, "id");
  const GA_ROHandleI dead(gdp, GA_ATTRIB_POINT, "dead");
  const float rot[4] = {0.0f, 0.0f, 0.0f, 1.0f};
  const float zero[3] = {0.0f, 0.0f, 0.0f};

  GA_Offset off;
  GA_FOR_ALL_PTOFF(gdp, off) {
    if (dead.isValid() && dead.get(off) != 0)
      continue;
    const int64_t key = ids.isValid()
                            ? static_cast<int64_t>(ids.get(off))
                            : static_cast<int64_t>(gdp->pointIndex(off));
    const UT_Vector3 p = gdp->getPos3(off);
    const double local[3] = {p.x(), p.y(), p.z()};
    double pos[3];
    b3dop::xformPoint(m, local, pos);
    if (!b3dop::finite3(pos))
      continue;
    float radius = pscale.isValid() ? pscale.get(off) : 0.0f;
    if (!(radius >= kMinPointRadius) || !std::isfinite(radius))
      radius = kMinPointRadius;

    Box3DPointSphere &s = ab.spheres[key];
    s.stamp = stamp;
    if (s.handle &&
        std::fabs(s.radius - radius) > 1e-4f * std::max(s.radius, radius)) {
      st.world.destroyBody(s.handle); // another size: a new sphere
      s.handle = 0;
    }
    if (!s.handle) {
      const b3dop::BodyHandle h = st.world.createBody(body, pos, rot, nullptr);
      b3dop::ShapeSpec spec;
      spec.type = b3dop::ShapeType::Sphere;
      spec.radius = radius;
      if (!h || !st.world.createShape(h, body, spec)) {
        if (h)
          st.world.destroyBody(h);
        ++counts.failedSpheres;
        continue;
      }
      s.handle = h;
      s.radius = radius;
      b3dop::copy3(s.pos, pos);
      s.driven = false;
      ++counts.spheres;
      continue;
    }
    const bool moved = std::fabs(s.pos[0] - pos[0]) > 1e-9 ||
                       std::fabs(s.pos[1] - pos[1]) > 1e-9 ||
                       std::fabs(s.pos[2] - pos[2]) > 1e-9;
    if (moved) {
      if (st.stepDt > 0.0f)
        st.world.setTargetTransformByHandle(s.handle, pos, rot, st.stepDt);
      else
        st.world.setTransformByHandle(s.handle, pos, rot);
      b3dop::copy3(s.pos, pos);
      s.driven = st.stepDt > 0.0f;
      ++counts.moved;
    } else if (s.driven) {
      st.world.setVelocityByHandle(s.handle, zero, zero); // it stopped
      s.driven = false;
    }
  }

  // Particles that are gone.
  for (auto it = ab.spheres.begin(); it != ab.spheres.end();) {
    if (it->second.stamp != stamp) {
      if (it->second.handle)
        st.world.destroyBody(it->second.handle);
      ++counts.removed;
      it = ab.spheres.erase(it);
    } else
      ++it;
  }
}
