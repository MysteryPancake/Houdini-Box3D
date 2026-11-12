// SIM_Box3DRegister.cpp
//
// initializeSIM(): Houdini registers our SIM_Data classes when it loads the plugin.

#include "SIM_Box3DWorld.h"
#include "SIM_Box3DData.h"
#include "SIM_SolverBox3D.h"

// Houdini's version-check entry points. Include in exactly one .cpp of the plugin: this one.
#include <UT/UT_DSOVersion.h>

#include <SIM/SIM_DataFactory.h>
#include <SIM/SIM_DataUtils.h>      // IMPLEMENT_DATAFACTORY

// Declared extern "C" and exported by the HDK headers.
void
initializeSIM(void *)
{
    IMPLEMENT_DATAFACTORY(SIM_Box3DWorld);
    IMPLEMENT_DATAFACTORY(SIM_Box3DData);
    IMPLEMENT_DATAFACTORY(SIM_SolverBox3D);
}
