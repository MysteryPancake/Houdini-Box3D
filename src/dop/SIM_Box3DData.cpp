// SIM_Box3DData.cpp
#include "SIM_Box3DData.h"

#include "SIM_Box3DParms.h"
#include "sync/Box3DHoudiniUtil.h"

#include <SIM/SIM_DopDescription.h>
#include <UT/UT_Quaternion.h>
#include <UT/UT_Vector3.h>
#include <UT/UT_XformOrder.h>

#include <cmath>

SIM_Box3DData::SIM_Box3DData(const SIM_DataFactory *factory)
    : BaseClass(factory), SIM_OptionsUser(this) {}

SIM_Box3DData::~SIM_Box3DData() {}

const SIM_DopDescription *SIM_Box3DData::getDescription() {
  // Parameter names match the point attributes that override them. Box3D
  // asserts on negative values, so ranges have a hard minimum of 0 (larger
  // values than the slider can be typed).
  using R = Box3DParms::Range;
  // Bullet-like defaults, shared with objects that have no Box3D Data.
  static const b3dop::BodySettings kDefaults = b3dop::nodeBodyDefaults();
  static Box3DParms p;
  static const PRM_Template *templates =
      p.folder("Shape")
          .menu("defaultshape", "Default Shape",
                static_cast<int>(b3dop::ShapeType::Convex),
                {"Box", "Sphere", "Convex Hull",
                 "Concave / Compound (Hull per Piece)", "Capsule", "None",
                 "Plane", "Cylinder"},
                "Collision shape for points without a bullet_georep attribute. "
                "Concave builds one convex hull per piece. A plane needs a "
                "static piece.")
          // Off: box / sphere / capsule from bullet_primS / radius / length /
          // primT / primR.
          .toggle(
              "autofit", "Auto Fit Box / Sphere / Capsule to Geometry", true,
              "Fit Box, Sphere and Capsule shapes to each piece's geometry. "
              "Off uses pscale / scale and the bullet_prim* attributes.")
          .toggle("hullperconnected",
                  "Create Convex Hull per Set of Connected Primitives", false,
                  "Build one convex hull per set of connected primitives, "
                  "instead of one hull around the whole piece.")
          // The hand-set shape: left at its default, pscale / scale size it.
          .vector("bullet_primS", "Box Size", {1, 1, 1},
                  "Half the box's size on each axis. Needs Auto Fit off. "
                  "Overridden by the bullet_primS attribute.",
                  PRM_XYZ_J, "{ autofit == 1 }")
          .number("bullet_radius", "Radius", 1.0, R{0, 5},
                  "Sphere or capsule radius. Needs Auto Fit off. Overridden by "
                  "the bullet_radius attribute.",
                  "{ autofit == 1 }")
          .number("bullet_length", "Length", 1.0, R{0, 5},
                  "Capsule length along Y, caps not included. Needs Auto Fit "
                  "off. Overridden by the bullet_length attribute.",
                  "{ autofit == 1 }")
          .vector("bullet_primT", "Offset", {0, 0, 0},
                  "Moves the shape from the body's centre. Needs Auto Fit off. "
                  "Overridden by the bullet_primT attribute.",
                  PRM_XYZ_J, "{ autofit == 1 }")
          .vector("bullet_primR", "Rotation", {0, 0, 0},
                  "Turns the shape, as XYZ angles in degrees. Needs Auto Fit "
                  "off. Overridden by the bullet_primR attribute.",
                  PRM_XYZ_J, "{ autofit == 1 }")
          .folder("Body")
          .menu("bodytype", "Body Type", 2, {"Static", "Kinematic", "Dynamic"},
                "Static never moves, Kinematic follows its animation, Dynamic "
                "is fully simulated. Points with active = 0 are static.")
          .toggle("computemass", "Compute Mass", kDefaults.props.computeMass,
                  "Work out mass from Density and shape volume instead of "
                  "using Mass. A computemass point attribute wins; RBD Packed "
                  "Object always writes one.")
          .number("mass", "Mass", 1.0, R{0, 10},
                  "Mass of each body (when Compute Mass is off). A mass point "
                  "attribute wins; RBD Packed Object always writes one: set it "
                  "there.",
                  "{ computemass == 1 }")
          .number("density", "Density", kDefaults.props.density, R{0, 2000},
                  "Mass per unit volume, with Compute Mass on (1000 is about "
                  "water). A density attribute wins; RBD Packed Object always "
                  "writes one.",
                  "{ computemass == 0 }")
          .toggle("computecom", "Compute Center of Mass",
                  kDefaults.props.computeCom,
                  "Balance and spin each body about the centre of mass of its "
                  "shape. Off: about the point. A computecom point attribute "
                  "wins.")
          .number("inertialtensorstiffness", "Rotational Stiffness", 1.0,
                  R{0.01, 10},
                  "How hard bodies are to spin: above 1 resists spinning, "
                  "below 1 spins easily. Overridden by the "
                  "inertialtensorstiffness attribute.")
          .number("gravityscale", "Gravity Scale", 1.0,
                  "Multiplies the solver's Gravity: 0 floats, negative falls "
                  "upward. Overridden by the gravityscale attribute.")
          .folder("Material")
          .number(
              "bounce", "Bounce", kDefaults.props.restitution, R{0, 1},
              "Speed kept after a collision: 0 doesn't bounce, 1 full speed. A "
              "bounce attribute wins; RBD Packed Object always writes one.")
          .number("friction", "Friction", kDefaults.props.friction, R{0, 1},
                  "How much surfaces resist sliding; 0 is ice-slick. A "
                  "friction attribute wins; RBD Packed Object always writes "
                  "one: set it there.")
          .number("rollingresistance", "Rolling Resistance", 0.0, R{0, 1},
                  "How much round bodies resist rolling; 0 rolls freely. "
                  "Overridden by the rollingresistance attribute.")
          .vector(
              "tangentvelocity", "Conveyor Velocity", {0, 0, 0},
              "Conveyor belt: touching bodies are dragged at this velocity, in "
              "the body's own axes. "
              "Needs friction. Overridden by the tangentvelocity attribute.",
              PRM_XYZ_J)
          .number("lineardamping", "Linear Damping", 0.0, R{0, 10},
                  "Slows movement like air drag; 0 is none. Overridden by the "
                  "lineardamping attribute.")
          .number("angulardamping", "Angular Damping", 0.0, R{0, 10},
                  "Slows spinning; 0 is none. Overridden by the angulardamping "
                  "attribute.")
          .folder("CCD & Contacts")
          .number("ccdthreshold", "CCD Threshold", 0.5, R{0, 1},
                  "Continuous collision starts when a body moves more than "
                  "this fraction of its thinnest size per step. "
                  "Lower stops tunnelling. Overridden by ccdthreshold.")
          // Box3D's "bullet" flag (Houdini's `isbullet` means something else).
          .toggle("fastobject", "Fast Object", false,
                  "Also run continuous collision against moving bodies, so "
                  "fast bodies can't pass through them. Needs the solver's "
                  "Continuous Collision.")
          .toggle("contactrecycling", "Contact Recycling", true,
                  "Reuse contacts between steps (within the solver's Contact "
                  "Recycle Distance). Turn off if thin or fast bodies show "
                  "ghost collisions.")
          .toggle(
              "addimpact", "Add Impact Data", false,
              "Record impacts in Impacts data on the object. Costs extra time "
              "and memory. Filter by speed with the solver's Impact Min Speed.")
          .folder("Sleep")
          .toggle("enablesleep", "Allow Sleeping", true,
                  "Let resting bodies stop simulating until touched. Needs the "
                  "solver's Allow Sleeping. Overridden by the "
                  "enablesleep attribute.")
          .toggle(
              "startasleep", "Start Asleep", false,
              "Bodies begin asleep until touched. Needs Allow Sleeping. "
              "Overridden by the awake attribute (awake = 0 starts asleep).",
              "{ enablesleep == 0 }")
          // Box3D: one threshold, fixed half second to sleep.
          .number("sleepthreshold", "Sleep Threshold", 0.05, R{0, 1},
                  "A body falls asleep once its speed stays below this for "
                  "half a second. Overridden by the sleepthreshold attribute.",
                  "{ enablesleep == 0 }")
          .templates();

  static SIM_DopDescription theDopDescription(
      true,           // true: also creates a DOP node that applies this data
      "box3d_data",   // internal node name
      "Box3D Data",   // node label
      kBox3DDataName, // data name on the object
      classname(), templates);

  return &theDopDescription;
}

// The menus list the enums in order.
static_assert(static_cast<int>(b3dop::BodySettings::Type::Static) == 0 &&
                  static_cast<int>(b3dop::BodySettings::Type::Kinematic) == 1 &&
                  static_cast<int>(b3dop::BodySettings::Type::Dynamic) == 2,
              "bodytype menu order");
static_assert(static_cast<int>(b3dop::ShapeType::Box) == 0 &&
                  static_cast<int>(b3dop::ShapeType::Sphere) == 1 &&
                  static_cast<int>(b3dop::ShapeType::Convex) == 2 &&
                  static_cast<int>(b3dop::ShapeType::Concave) == 3 &&
                  static_cast<int>(b3dop::ShapeType::Capsule) == 4 &&
                  static_cast<int>(b3dop::ShapeType::None) == 5 &&
                  static_cast<int>(b3dop::ShapeType::Plane) == 6 &&
                  static_cast<int>(b3dop::ShapeType::Cylinder) == 7,
              "defaultshape menu order");

b3dop::BodySettings SIM_Box3DData::toBodySettings() const {
  b3dop::BodySettings s;

  // Out-of-range menu values (set from a script) fall back to the defaults.
  const int bodyType = static_cast<int>(getBodyType());
  if (bodyType >= 0 && bodyType <= 2)
    s.type = static_cast<b3dop::BodySettings::Type>(bodyType);
  const int shape = static_cast<int>(getDefaultShape());
  if (shape >= 0 && shape <= 7)
    s.defaultShape = static_cast<b3dop::ShapeType>(shape);

  b3dop::BodyProps &p = s.props;
  p.mass = static_cast<float>(getMass());
  p.computeMass = getComputeMass();
  p.computeCom = getComputeCom();
  p.density = static_cast<float>(getDensity());
  p.friction = static_cast<float>(getFriction());
  p.restitution = static_cast<float>(getBounce());
  p.rollingResistance = static_cast<float>(getRollingResistance());
  {
    const UT_Vector3 t = getTangentVelocity();
    for (unsigned i = 0; i < 3; ++i)
      p.tangentVelocity[i] = static_cast<float>(t(i));
  }
  p.linearDamping = static_cast<float>(getLinearDamping());
  p.angularDamping = static_cast<float>(getAngularDamping());
  p.gravityScale = static_cast<float>(getGravityScale());
  p.enableSleep = getEnableSleep();
  s.startAwake = !getStartAsleep();
  p.fastObject = getFastObject();
  p.contactRecycling = getContactRecycling();
  p.sleepThreshold = static_cast<float>(getSleepThreshold());
  p.ccdThreshold = static_cast<float>(getCcdThreshold());
  s.hullPerConnectedSet = getHullPerConnected();
  s.autoFit = getAutoFit();

  const float stiffness = static_cast<float>(getInertiaStiffness());
  if (stiffness > 0.0f && std::isfinite(stiffness))
    p.inertiaScale = stiffness;

  // The hand-set shape: each value that is not its default.
  const b3dop::PrimOverrides unset;
  b3dop::PrimOverrides &o = s.prim;
  const UT_Vector3 size = getPrimSize();
  for (unsigned i = 0; i < 3; ++i)
    o.size[i] = static_cast<float>(size(i));
  o.hasSize = o.size[0] != unset.size[0] || o.size[1] != unset.size[1] ||
              o.size[2] != unset.size[2];
  const UT_Vector3 offset = getPrimOffset();
  for (unsigned i = 0; i < 3; ++i)
    o.pos[i] = static_cast<float>(offset(i));
  o.hasPos = o.pos[0] != unset.pos[0] || o.pos[1] != unset.pos[1] ||
             o.pos[2] != unset.pos[2];
  const UT_Vector3 rotation = getPrimRotation();
  const float degrees[3] = {static_cast<float>(rotation(0)),
                            static_cast<float>(rotation(1)),
                            static_cast<float>(rotation(2))};
  o.hasRot = degrees[0] != 0.0f || degrees[1] != 0.0f || degrees[2] != 0.0f;
  if (o.hasRot)
    box3dEulerToQuat(degrees, o.rot);
  o.radius = static_cast<float>(getPrimRadius());
  o.hasRadius = o.radius != unset.radius;
  o.length = static_cast<float>(getPrimLength());
  o.hasLength = o.length != unset.length;
  return s;
}
