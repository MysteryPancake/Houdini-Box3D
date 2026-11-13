// SIM_Box3DConRel.h
//
// Box3D constraint relationships: one DOP node per Box3D joint type ("Box3D
// Revolute Constraint Relationship", ...), used like SideFX's Glue / Hard /
// Spring Constraint Relationship. Wire one into the second input of a
// Constraint Network DOP; primitives whose `constraint_name` equals its Data
// Name become joints of that type, with the node's parameters as their
// settings. Primitive attributes of the same names modify them (floats
// multiply, everything else replaces), as the Constraint Network DOP documents.
// Parameters are generated from b3dop::jointParams(), so the nodes, the
// attributes and the core always agree.
#pragma once

#include <SIM/SIM_ConRel.h>
#include <SIM/SIM_DataUtils.h>
#include <SIM/SIM_OptionsUser.h>
#include <UT/UT_StringHolder.h>

#include "core/Box3DJoints.h"

class SIM_DopDescription;

// The base of the nine node types. Never created itself.
class SIM_Box3DConRel : public SIM_ConRel, public SIM_OptionsUser {
public:
  // The Box3D joint type this node makes.
  virtual b3dop::JointType jointType() const = 0;

  // The node's settings, in Box3D units (Box3D's defaults for anything the type
  // does not use).
  b3dop::JointSettings settings() const;

protected:
  explicit SIM_Box3DConRel(const SIM_DataFactory *factory);
  ~SIM_Box3DConRel() override;

  // The DOP description (node, parameters, default Data Name) for joint type
  // `t`.
  static const SIM_DopDescription *describe(b3dop::JointType t,
                                            const UT_StringHolder &classname);

private:
  DECLARE_STANDARD_GETCASTTOTYPE();
  DECLARE_CLASSNAME(SIM_Box3DConRel, SIM_ConRel);
};

// One subclass per joint type. Internal node name box3d_<type>conrel, default
// Data Name <type> (the value `constraint_name` uses).
#define BOX3D_DECLARE_CONREL(ClassName, Type, Label)                           \
  class ClassName : public SIM_Box3DConRel {                                   \
  public:                                                                      \
    b3dop::JointType jointType() const override { return Type; }               \
                                                                               \
  protected:                                                                   \
    explicit ClassName(const SIM_DataFactory *factory) : BaseClass(factory) {} \
    ~ClassName() override {}                                                   \
                                                                               \
  private:                                                                     \
    static const SIM_DopDescription *getDescription() {                        \
      return describe(Type, classname());                                      \
    }                                                                          \
    DECLARE_STANDARD_GETCASTTOTYPE();                                          \
    DECLARE_DATAFACTORY(ClassName, SIM_Box3DConRel, Label, getDescription());  \
  }

BOX3D_DECLARE_CONREL(SIM_Box3DConRelWeld, b3dop::JointType::Weld,
                     "Box3D Weld Constraint Relationship");
BOX3D_DECLARE_CONREL(SIM_Box3DConRelSpherical, b3dop::JointType::Spherical,
                     "Box3D Spherical Constraint Relationship");
BOX3D_DECLARE_CONREL(SIM_Box3DConRelRevolute, b3dop::JointType::Revolute,
                     "Box3D Revolute Constraint Relationship");
BOX3D_DECLARE_CONREL(SIM_Box3DConRelPrismatic, b3dop::JointType::Prismatic,
                     "Box3D Prismatic Constraint Relationship");
BOX3D_DECLARE_CONREL(SIM_Box3DConRelDistance, b3dop::JointType::Distance,
                     "Box3D Distance Constraint Relationship");
BOX3D_DECLARE_CONREL(SIM_Box3DConRelMotor, b3dop::JointType::Motor,
                     "Box3D Motor Constraint Relationship");
BOX3D_DECLARE_CONREL(SIM_Box3DConRelParallel, b3dop::JointType::Parallel,
                     "Box3D Parallel Constraint Relationship");
BOX3D_DECLARE_CONREL(SIM_Box3DConRelWheel, b3dop::JointType::Wheel,
                     "Box3D Wheel Constraint Relationship");
BOX3D_DECLARE_CONREL(SIM_Box3DConRelFilter, b3dop::JointType::Filter,
                     "Box3D Filter Constraint Relationship");
