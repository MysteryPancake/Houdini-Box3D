// SIM_Box3DRegister.cpp
//
// initializeSIM(): Houdini registers our SIM_Data classes when it loads the plugin.

#include "SIM_Box3DWorld.h"
#include "SIM_Box3DData.h"
#include "SIM_Box3DJoints.h"
#include "SIM_SolverBox3D.h"

// Houdini's version-check entry points. Include in exactly one .cpp of the plugin: this one.
#include <UT/UT_DSOVersion.h>

#include "Box3DShapes.h"

#include <SIM/SIM_DataFactory.h>
#include <SIM/SIM_DataUtils.h>      // IMPLEMENT_DATAFACTORY
#include <UT/UT_String.h>

#include <string>

namespace {

// collisionignore / activationignore matching with Houdini's own pattern rules.
bool
houdiniPatternMatch(const std::string &patterns, const std::string &name)
{
    if (patterns.empty() || name.empty())
        return false;
    return UT_String(name.c_str()).multiMatch(patterns.c_str());
}

}

// Declared extern "C" and exported by the HDK headers.
void
initializeSIM(void *)
{
    b3dop::patternMatcher() = &houdiniPatternMatch;

    IMPLEMENT_DATAFACTORY(SIM_Box3DWorld);
    IMPLEMENT_DATAFACTORY(SIM_Box3DData);
    IMPLEMENT_DATAFACTORY(SIM_Box3DJoints);
    IMPLEMENT_DATAFACTORY(SIM_SolverBox3D);
}
