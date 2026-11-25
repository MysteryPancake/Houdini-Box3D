// SIM_Box3DRegister.cpp
//
// initializeSIM(): Houdini registers our SIM_Data classes when it loads the
// plugin.

#include "SIM_Box3DConRel.h"
#include "SIM_Box3DData.h"
#include "SIM_Box3DWorld.h"
#include "SIM_SolverBox3D.h"

// Houdini's version-check entry points. Include in exactly one .cpp of the
// plugin: this one.
#include <UT/UT_DSOVersion.h>

#include <SIM/SIM_DataFactory.h>
#include <SIM/SIM_DataHint.h>
#include <SIM/SIM_DataUtils.h> // IMPLEMENT_DATAFACTORY
// Declared extern "C" and exported by the HDK headers.
void initializeSIM(void *) {
  IMPLEMENT_DATAFACTORY(SIM_Box3DWorld);
  IMPLEMENT_DATAFACTORY(SIM_Box3DData);
  IMPLEMENT_DATAFACTORY(SIM_Box3DConRelWeld);
  IMPLEMENT_DATAFACTORY(SIM_Box3DConRelSpherical);
  IMPLEMENT_DATAFACTORY(SIM_Box3DConRelRevolute);
  IMPLEMENT_DATAFACTORY(SIM_Box3DConRelPrismatic);
  IMPLEMENT_DATAFACTORY(SIM_Box3DConRelDistance);
  IMPLEMENT_DATAFACTORY(SIM_Box3DConRelMotor);
  IMPLEMENT_DATAFACTORY(SIM_Box3DConRelParallel);
  IMPLEMENT_DATAFACTORY(SIM_Box3DConRelWheel);
  IMPLEMENT_DATAFACTORY(SIM_Box3DConRelFilter);
  IMPLEMENT_DATAFACTORY(SIM_Box3DConRelMotionLock);
  IMPLEMENT_DATAFACTORY(SIM_SolverBox3D);

  // Box3D Data sits under an object's Geometry (kBox3DDataName), as Bullet Data
  // does. Houdini checks every data against the children its parent is hinted
  // to have, and warns "Unexpected subdata found" on the Geometry and "attached
  // to an unexpected parent data" on the data node for any it has no hint for.
  // The Geometry is a SIM_SopGeometry in a Packed Object and a SIM_GeometryCopy
  // in the Box3D Solver SOP (its pieces are copied).
  SIM_DataHint::addHint("SIM_SopGeometry", "SIM_Box3DData", "Box3DData", 1);
  SIM_DataHint::addHint("SIM_GeometryCopy", "SIM_Box3DData", "Box3DData", 1);
}
