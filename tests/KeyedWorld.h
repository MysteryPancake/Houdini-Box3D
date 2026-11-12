// KeyedWorld.h
//
// Tests only: Box3DWorld with bodies addressed by caller-chosen ids, built on the public handle
// API. Shape calls use the BodySettings given to addBody (density included, see setBodyDensity).
#pragma once

#include "core/Box3DWorld.h"

#include <memory>
#include <unordered_map>
#include <utility>

namespace b3dop {

class KeyedWorld : public Box3DWorld
{
public:
    bool create(const WorldSettings &s, std::shared_ptr<TaskRunner> tasks = nullptr)
    {
        myBodies.clear();
        return Box3DWorld::create(s, std::move(tasks));
    }
    void destroy()
    {
        myBodies.clear();
        Box3DWorld::destroy();
    }

    // ---- bodies --------------------------------------------------------
    bool addBody(uint64_t id, const BodySettings &body, const double position[3], const float rotationXYZW[4])
    {
        if (!isValid() || hasBody(id))
            return false;
        const BodyHandle h = createBody(body, position, rotationXYZW);
        if (!h)
            return false;
        myBodies[id] = Entry{ h, body };
        return true;
    }
    void removeBody(uint64_t id)
    {
        auto it = myBodies.find(id);
        if (it == myBodies.end())
            return;
        destroyBody(it->second.handle);
        myBodies.erase(it);
    }
    bool       hasBody(uint64_t id) const { return myBodies.count(id) != 0; }
    BodyHandle bodyHandle(uint64_t id) const       // 0 if there is no such body
    {
        auto it = myBodies.find(id);
        return it == myBodies.end() ? 0 : it->second.handle;
    }

    bool setBodyType(uint64_t id, BodySettings::Type type)
    {
        Entry *e = find(id);
        if (!e || !setTypeByHandle(e->handle, type))
            return false;
        e->settings.type = type;
        return true;
    }
    bool getBodyType(uint64_t id, BodySettings::Type &out) const
    {
        const BodyHandle h = bodyHandle(id);
        if (h)
            out = typeByHandle(h);
        return h != 0;
    }
    bool setBodyDensity(uint64_t id, float density)     // for shapes added after the call
    {
        Entry *e = find(id);
        if (e)
            e->settings.density = density;
        return e != nullptr;
    }

    // ---- shapes (mass updated by each add) ------------------------------
    bool addShape(uint64_t id, const ShapeSpec &shape)
    {
        Entry *e = find(id);
        return e && createShape(e->handle, e->settings, shape, true);
    }
    bool addBoxShape(uint64_t id, const float halfExtents[3], const float center[3] = nullptr,
                     const float rotationXYZW[4] = nullptr)
    {
        ShapeSpec s = spec(ShapeType::Box, center, rotationXYZW);
        for (int i = 0; i < 3; ++i) s.halfExtents[i] = halfExtents[i];
        return addShape(id, s);
    }
    bool addSphereShape(uint64_t id, float radius, const float center[3] = nullptr)
    {
        ShapeSpec s = spec(ShapeType::Sphere, center, nullptr);
        s.radius = radius;
        return addShape(id, s);
    }
    bool addCapsuleShape(uint64_t id, float radius, float halfLength, const float center[3] = nullptr,
                         const float rotationXYZW[4] = nullptr)
    {
        ShapeSpec s = spec(ShapeType::Capsule, center, rotationXYZW);
        s.radius     = radius;
        s.halfLength = halfLength;
        return addShape(id, s);
    }
    bool addHullShape(uint64_t id, HullId hull)
    {
        Entry *e = find(id);
        return e && createHullShape(e->handle, e->settings, hull, true);
    }
    int   shapeCount(uint64_t id) const { return shapeCountByHandle(bodyHandle(id)); }
    float bodyMass(uint64_t id) const { return massByHandle(bodyHandle(id)); }

    // ---- pose and velocity ----------------------------------------------
    bool getTransform(uint64_t id, double outPos[3], float outRotXYZW[4]) const
    {
        return getTransformByHandle(bodyHandle(id), outPos, outRotXYZW);
    }
    bool getVelocity(uint64_t id, float outLinear[3], float outAngular[3]) const
    {
        return getVelocityByHandle(bodyHandle(id), outLinear, outAngular);
    }
    bool setVelocity(uint64_t id, const float linear[3], const float angular[3])
    {
        return setVelocityByHandle(bodyHandle(id), linear, angular);
    }
    bool setTransform(uint64_t id, const double pos[3], const float rotXYZW[4])
    {
        return setTransformByHandle(bodyHandle(id), pos, rotXYZW);
    }

private:
    struct Entry
    {
        BodyHandle   handle = 0;
        BodySettings settings;
    };

    Entry *find(uint64_t id)
    {
        auto it = myBodies.find(id);
        return it == myBodies.end() ? nullptr : &it->second;
    }
    static ShapeSpec spec(ShapeType type, const float center[3], const float rot[4])
    {
        ShapeSpec s;
        s.type = type;
        for (int i = 0; i < 3 && center; ++i) s.center[i] = center[i];
        for (int i = 0; i < 4 && rot; ++i) s.rot[i] = rot[i];
        return s;
    }

    std::unordered_map<uint64_t, Entry> myBodies;
};

} // namespace b3dop
