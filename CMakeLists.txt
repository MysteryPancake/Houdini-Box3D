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

# Debug logging in the Houdini plugin (see src/Box3DDebug.h). Off by default;
# configure with -DBOX3DDOP_DEBUG=ON to log to Houdini's console.
option(BOX3DDOP_DEBUG "Compile debug logging into the Houdini plugin" OFF)

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

# How many Box3D worlds may exist at once (Box3D's own default is 128). Every rebuild makes a
# new world and the old one lives on while cached frames use it, so long sessions and several
# DOP networks add up. Each slot is a ~5 KB static array entry. Box3D caps it below 65535.
set(BOX3DDOP_MAX_WORLDS 1024 CACHE STRING "Maximum number of Box3D worlds alive at once")
target_compile_definitions(box3d PUBLIC B3_MAX_WORLDS=${BOX3DDOP_MAX_WORLDS})

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
# The core's own console output (Box3D asserts and log messages) is compiled in only with the debug
# option; otherwise nothing is ever printed (on Windows that would open a console window).
if(BOX3DDOP_DEBUG)
    target_compile_definitions(b3dop_core PRIVATE BOX3DDOP_DEBUG=1)
endif()

add_executable(b3dop_core_smoke tests/core_smoke.cpp)
target_link_libraries(b3dop_core_smoke PRIVATE b3dop_core)

add_executable(b3dop_core_physics tests/core_physics.cpp)
target_link_libraries(b3dop_core_physics PRIVATE b3dop_core)

add_executable(b3dop_core_mesh tests/core_mesh.cpp)
target_link_libraries(b3dop_core_mesh PRIVATE b3dop_core)

add_executable(b3dop_core_threads tests/core_threads.cpp)
target_link_libraries(b3dop_core_threads PRIVATE b3dop_core)

add_executable(b3dop_core_taskrunner tests/core_taskrunner.cpp)
target_link_libraries(b3dop_core_taskrunner PRIVATE b3dop_core)
find_package(Threads REQUIRED)
target_link_libraries(b3dop_core_taskrunner PRIVATE Threads::Threads)

if(BOX3DDOP_CORE_ONLY)
    return()
endif()

# ---------------------------------------------------------------------------
# Houdini plugin. Run cmake from a shell with the Houdini environment set so
# HFS is set (Windows: "Houdini Command Line Tools"; Linux: source houdini_setup),
# or pass -DCMAKE_PREFIX_PATH=<HFS>/toolkit/cmake
# ---------------------------------------------------------------------------
# You can also skip HFS and pass -DHoudini_ROOT=<HFS>/toolkit/cmake: find_package
# checks <Package>_ROOT on its own (policy CMP0074, CMake 3.12+).
if(DEFINED ENV{HFS})
    list(APPEND CMAKE_PREFIX_PATH "$ENV{HFS}/toolkit/cmake")
endif()
find_package(Houdini REQUIRED)

add_library(Box3DDOP SHARED
    src/SIM_Box3DWorld.h
    src/SIM_Box3DWorld.cpp
    src/SIM_Box3DData.h
    src/SIM_Box3DData.cpp
    src/SIM_SolverBox3D.h
    src/SIM_SolverBox3D.cpp
    src/Box3DAffectors.h
    src/Box3DAffectors.cpp
    src/Box3DAffectorMath.h
    src/SIM_Box3DRegister.cpp
    src/Box3DDebug.h
    src/Box3DHoudiniTasks.h
)
target_link_libraries(Box3DDOP PRIVATE Houdini b3dop_core)
if(BOX3DDOP_DEBUG)
    target_compile_definitions(Box3DDOP PRIVATE BOX3DDOP_DEBUG=1)
endif()

# Output name, and copies the plugin into the houdiniX.Y/dso folder (Documents on
# Windows, the home directory on Linux)
houdini_configure_target(Box3DDOP)

# Node icons: Houdini picks up config/Icons/DOP_<node name>.svg for each node type.
# houdini_configure_target builds the plugin straight into houdiniX.Y/dso, so the
# icons go next to it in houdiniX.Y/config/Icons.
set(BOX3DDOP_NODES box3d_solver box3d_data)       # internal node names
set(BOX3DDOP_ICON_DIR "$<TARGET_FILE_DIR:Box3DDOP>/../config/Icons")
set(BOX3DDOP_ICON_COPIES "")
foreach(node IN LISTS BOX3DDOP_NODES)
    list(APPEND BOX3DDOP_ICON_COPIES
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/icons/box3d.svg" "${BOX3DDOP_ICON_DIR}/DOP_${node}.svg")
endforeach()
add_custom_command(TARGET Box3DDOP POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${BOX3DDOP_ICON_DIR}"
    ${BOX3DDOP_ICON_COPIES}
    VERBATIM)

# ---------------------------------------------------------------------------
# Houdini package. `cmake --build build --target package` (or `cpack -G ZIP` in the build folder)
# makes a zip to extract into houdiniX.Y/packages: Box3D.json plus the Box3D/ folder with
# dso/ and config/Icons/. The plugin only loads in the Houdini version it was built against,
# so the JSON is enabled for that version only.
# ---------------------------------------------------------------------------
string(REGEX MATCH "^[0-9]+\\.[0-9]+" BOX3DDOP_HOUDINI_VERSION "${Houdini_VERSION}")
if(NOT BOX3DDOP_HOUDINI_VERSION)
    message(WARNING "Houdini version unknown: the package JSON will not be limited to one version")
    set(BOX3DDOP_HOUDINI_VERSION "unknown")
    set(BOX3DDOP_PKG_ENABLE "true")
else()
    set(BOX3DDOP_PKG_ENABLE "\"houdini_version == '${BOX3DDOP_HOUDINI_VERSION}'\"")
endif()
configure_file(package/Box3D.json.in "${CMAKE_BINARY_DIR}/package/Box3D.json" @ONLY)

install(TARGETS Box3DDOP RUNTIME DESTINATION Box3D/dso LIBRARY DESTINATION Box3D/dso)
foreach(node IN LISTS BOX3DDOP_NODES)
    install(FILES icons/box3d.svg DESTINATION Box3D/config/Icons RENAME DOP_${node}.svg)
endforeach()
install(FILES "${CMAKE_BINARY_DIR}/package/Box3D.json" DESTINATION .)

set(CPACK_GENERATOR ZIP)
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_PACKAGE_FILE_NAME "Box3D-Houdini-${BOX3DDOP_HOUDINI_VERSION}-${CMAKE_SYSTEM_NAME}")
include(CPack)
