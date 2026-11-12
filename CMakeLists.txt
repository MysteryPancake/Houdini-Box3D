cmake_minimum_required(VERSION 3.22)
project(Box3DDOP LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
# Step 1 for a new machine: configure with -DBOX3DDOP_CORE_ONLY=ON. That builds
# only Box3D + the Houdini-free wrapper + a smoke test, so you can prove the
# Box3D half works before involving the HDK.
option(BOX3DDOP_CORE_ONLY "Build only the Houdini-free core and smoke test" OFF)

# Debug logging in the Houdini plugin (see src/Box3DDebug.h). On by default;
# turn OFF for a quiet build.
option(BOX3DDOP_DEBUG "Compile debug logging into the Houdini plugin" ON)

# Where the Box3D source tree lives (the folder containing its CMakeLists.txt,
# i.e. the INNER box3d folder if your zip unpacks as box3d/box3d/...).
set(BOX3D_DIR "${CMAKE_CURRENT_SOURCE_DIR}/extern/box3d" CACHE PATH "Path to Box3D source")

# ---------------------------------------------------------------------------
# Box3D, built as a static library and linked into our plugin DLL.
# Because we are not the top-level project, Box3D skips its samples/tests.
# ---------------------------------------------------------------------------
set(BUILD_SHARED_LIBS OFF)
add_subdirectory("${BOX3D_DIR}" "${CMAKE_BINARY_DIR}/box3d" EXCLUDE_FROM_ALL)
set_target_properties(box3d PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# Core: plain C++ wrapper around Box3D. No Houdini headers in here.
# ---------------------------------------------------------------------------
add_library(b3dop_core STATIC
    src/Box3DShapes.h
    src/Box3DWorld.h
    src/Box3DWorld.cpp
)
target_include_directories(b3dop_core PUBLIC src)
# PRIVATE: Box3D's headers/defines stay out of the Houdini plugin's translation
# units. It's a static lib, so CMake still links it into anything using b3dop_core.
target_link_libraries(b3dop_core PRIVATE box3d::box3d)
set_target_properties(b3dop_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(b3dop_core_smoke tests/core_smoke.cpp)
target_link_libraries(b3dop_core_smoke PRIVATE b3dop_core)

add_executable(b3dop_core_physics tests/core_physics.cpp)
target_link_libraries(b3dop_core_physics PRIVATE b3dop_core)

add_executable(b3dop_core_threads tests/core_threads.cpp)
target_link_libraries(b3dop_core_threads PRIVATE b3dop_core)

if(BOX3DDOP_CORE_ONLY)
    return()
endif()

# ---------------------------------------------------------------------------
# Houdini plugin. Run cmake from a shell with the Houdini environment set so
# HFS is set (Windows: "Houdini Command Line Tools"; Linux: source houdini_setup),
# or pass -DCMAKE_PREFIX_PATH=<HFS>/toolkit/cmake
# ---------------------------------------------------------------------------
list(APPEND CMAKE_PREFIX_PATH "$ENV{HFS}/toolkit/cmake")
find_package(Houdini REQUIRED)

add_library(Box3DDOP SHARED
    src/SIM_Box3DWorld.h
    src/SIM_Box3DWorld.cpp
    src/SIM_Box3DData.h
    src/SIM_Box3DData.cpp
    src/SIM_SolverBox3D.h
    src/SIM_SolverBox3D.cpp
    src/SIM_Box3DRegister.cpp
    src/Box3DDebug.h
)
target_link_libraries(Box3DDOP PRIVATE Houdini b3dop_core)
if(BOX3DDOP_DEBUG)
    target_compile_definitions(Box3DDOP PRIVATE BOX3DDOP_DEBUG=1)
endif()

# Output name, and copies the plugin into the houdiniX.Y/dso folder (Documents on
# Windows, the home directory on Linux)
houdini_configure_target(Box3DDOP)
