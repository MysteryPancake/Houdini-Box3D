// SIM_Box3DRegister.cpp
//
// Houdini calls initializeSIM() when it loads the DLL. Every SIM_Data
// subclass has to be registered here.

#include "SIM_Box3DWorld.h"
#include "SIM_Box3DBody.h"
#include "SIM_SolverBox3D.h"

// Mandatory for every Houdini DSO (the HDK DOP sample includes it too).
// It defines the version-check entry points Houdini uses to confirm the DLL
// was built against the running Houdini. Include it in EXACTLY ONE .cpp of
// the plugin; this file is the one, so don't add it anywhere else.
#include <UT/UT_DSOVersion.h>

#include <SIM/SIM_DataFactory.h>
#include <SIM/SIM_DataUtils.h>      // IMPLEMENT_DATAFACTORY

// initializeSIM is declared extern "C" with export visibility by the HDK
// headers (as in the HDK samples), so it is only defined here.
void
initializeSIM(void *)
{
    IMPLEMENT_DATAFACTORY(SIM_Box3DWorld);
    IMPLEMENT_DATAFACTORY(SIM_Box3DBody);
    IMPLEMENT_DATAFACTORY(SIM_SolverBox3D);
}
