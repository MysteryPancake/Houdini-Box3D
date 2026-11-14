cmake_minimum_required(VERSION 3.22)
project(Box3DDOP LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# compile_commands.json in the build folder, for clang-tidy and IDEs.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
# Step 1 for a new machine: configure with -DBOX3DDOP_CORE_ONLY=ON. That builds
# only Box3D + the Houdini-free wrapper + a smoke test, so you can prove the
# Box3D half works before involving the HDK.
option(BOX3DDOP_CORE_ONLY "Build only the Houdini-free core and smoke test" OFF)

# Debug logging in the Houdini plugin (see src/core/Box3DDebug.h). Off by default;
# configure with -DBOX3DDOP_DEBUG=ON to log to Houdini's console.
option(BOX3DDOP_DEBUG "Compile debug logging into the Houdini plugin" OFF)

# Compiler warnings on our own targets (never Box3D's). CI turns warnings into errors.
option(BOX3DDOP_WARNINGS_AS_ERRORS "Treat compiler warnings in our code as errors" OFF)

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

# Warning flags for one of our targets. Headers of imported targets (Houdini) are system
# includes, so only our own code is checked.
function(b3dop_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4)
        if(BOX3DDOP_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE /WX)
        endif()
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wshadow -Wconversion)
        if(BOX3DDOP_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE -Werror)
        endif()
    endif()
endfunction()

# ---------------------------------------------------------------------------
# Formatting: `cmake --build build --target format` rewrites our sources with
# clang-format (LLVM style, .clang-format); `format-check` only reports.
# ---------------------------------------------------------------------------
find_program(CLANG_FORMAT_EXE NAMES clang-format)
if(CLANG_FORMAT_EXE)
    file(GLOB_RECURSE B3DOP_FORMAT_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.h")
    add_custom_target(format
        COMMAND ${CLANG_FORMAT_EXE} -i --style=file ${B3DOP_FORMAT_SOURCES}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} VERBATIM)
    add_custom_target(format-check
        COMMAND ${CLANG_FORMAT_EXE} --dry-run -Werror --style=file ${B3DOP_FORMAT_SOURCES}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} VERBATIM)
endif()

# How many Box3D worlds may exist at once (Box3D's own default is 128). Every rebuild makes a
# new world and the old one lives on while cached frames use it, so long sessions and several
# DOP networks add up. Each slot is a ~5 KB static array entry. Box3D caps it below 65535.
set(BOX3DDOP_MAX_WORLDS 1024 CACHE STRING "Maximum number of Box3D worlds alive at once")
target_compile_definitions(box3d PUBLIC B3_MAX_WORLDS=${BOX3DDOP_MAX_WORLDS})

# ---------------------------------------------------------------------------
# Core: plain C++ wrapper around Box3D. No Houdini headers in here.
# ---------------------------------------------------------------------------
add_library(b3dop_core STATIC
    src/core/Box3DShapes.h
    src/core/Box3DJoints.h
    src/core/Box3DSideFX.h
    src/core/Box3DSideFX.cpp
    src/core/Box3DRbdObject.h
    src/core/Box3DCpu.h
    src/core/Box3DCpu.cpp
    src/core/Box3DWorld.h
    src/core/Box3DWorld.cpp
    src/core/Box3DAffectorMath.h
    src/core/Box3DDebug.h
)
# Includes are written from src/ ("core/Box3DWorld.h"), so src/ is the include root.
target_include_directories(b3dop_core PUBLIC src)
# PRIVATE: Box3D's headers/defines stay out of the Houdini plugin's translation
# units. It's a static lib, so CMake still links it into anything using b3dop_core.
target_link_libraries(b3dop_core PRIVATE box3d::box3d)
b3dop_warnings(b3dop_core)
set_target_properties(b3dop_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
# The core's own console output (Box3D asserts and log messages) is compiled in only with the debug
# option; otherwise nothing is ever printed (on Windows that would open a console window).
if(BOX3DDOP_DEBUG)
    target_compile_definitions(b3dop_core PRIVATE BOX3DDOP_DEBUG=1)
endif()

# ---------------------------------------------------------------------------
# Core tests. Each tests/core_<name>.cpp becomes the executable b3dop_core_<name> and a CTest test
# of the same name (run with `ctest --test-dir build`, add -C Release on Visual Studio). A test passes
# when its main() returns 0. Extra arguments are passed to the test as command-line arguments.
# ---------------------------------------------------------------------------
enable_testing()
find_package(Threads REQUIRED)

function(b3dop_add_test name)
    set(target b3dop_core_${name})
    add_executable(${target} tests/core_${name}.cpp)
    target_link_libraries(${target} PRIVATE b3dop_core Threads::Threads)
    b3dop_warnings(${target})
    add_test(NAME ${target} COMMAND ${target} ${ARGN})
endfunction()

b3dop_add_test(smoke)
b3dop_add_test(physics)
b3dop_add_test(mesh)
b3dop_add_test(heightfield)
b3dop_add_test(joints)
b3dop_add_test(sidefx)
b3dop_add_test(rbdobject)
b3dop_add_test(threads)
b3dop_add_test(taskrunner)

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
    # dop/: the DOP data types (nodes) and their registration
    src/dop/SIM_Box3DWorld.h
    src/dop/SIM_Box3DWorld.cpp
    src/dop/SIM_Box3DData.h
    src/dop/SIM_Box3DData.cpp
    src/dop/SIM_Box3DConRel.h
    src/dop/SIM_Box3DConRel.cpp
    src/dop/SIM_Box3DParms.h
    src/dop/SIM_SolverBox3D.h
    src/dop/SIM_SolverBox3D.cpp
    src/dop/SIM_Box3DRegister.cpp
    # sync/: Houdini data -> Box3D world, the solver's parts (Box3DSolverInternal.h)
    src/sync/Box3DState.h
    src/sync/Box3DAttribNames.h
    src/sync/Box3DSolverInternal.h
    src/sync/Box3DShapeBuild.cpp
    src/sync/Box3DPointSync.cpp
    src/sync/Box3DWholeObject.cpp
    src/sync/Box3DPostStep.cpp
    src/sync/Box3DForces.cpp
    src/sync/Box3DGuide.cpp
    src/sync/Box3DJointSync.h
    src/sync/Box3DJointSync.cpp
    src/sync/Box3DAffectors.h
    src/sync/Box3DAffectors.cpp
    src/sync/Box3DHoudiniTasks.h
    src/sync/Box3DHoudiniUtil.h
)
target_link_libraries(Box3DDOP PRIVATE Houdini b3dop_core)
b3dop_warnings(Box3DDOP)
if(BOX3DDOP_DEBUG)
    target_compile_definitions(Box3DDOP PRIVATE BOX3DDOP_DEBUG=1)
endif()

# Output name, and copies the plugin into the houdiniX.Y/dso folder (Documents on
# Windows, the home directory on Linux)
houdini_configure_target(Box3DDOP)

# Node icons: Houdini picks up config/Icons/DOP_<node name>.svg for each node type.
# houdini_configure_target builds the plugin straight into houdiniX.Y/dso, so the
# icons go next to it in houdiniX.Y/config/Icons.
set(BOX3DDOP_NODES box3d_solver box3d_data                       # internal node names
    box3d_weldconrel box3d_sphericalconrel box3d_revoluteconrel box3d_prismaticconrel box3d_distanceconrel
    box3d_motorconrel box3d_parallelconrel box3d_wheelconrel box3d_filterconrel
    box3d_packedobject)                                          # the asset below
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

# The Box3D Packed Object digital asset (tools/make_packed_object.py): built with hython after the
# plugin, into houdiniX.Y/otls next to dso/. It copies Box3D Data's parameters from the plugin just
# built, so it is remade every time the plugin is. Problems print a warning and never fail the build.
option(BOX3DDOP_MAKE_HDA "Build the Box3D Packed Object asset with hython after the plugin" ON)
if(BOX3DDOP_MAKE_HDA)
    find_program(BOX3DDOP_HYTHON NAMES hython
        HINTS "${Houdini_DIR}/../../bin" "$ENV{HFS}/bin" NO_DEFAULT_PATH)
    if(BOX3DDOP_HYTHON)
        set(BOX3DDOP_HDA "$<TARGET_FILE_DIR:Box3DDOP>/../otls/box3d_packedobject.hda")
        add_custom_command(TARGET Box3DDOP POST_BUILD
            COMMAND "${BOX3DDOP_HYTHON}" "${CMAKE_CURRENT_SOURCE_DIR}/tools/make_packed_object.py" "${BOX3DDOP_HDA}"
            VERBATIM)
    else()
        message(WARNING "hython not found next to the Houdini toolkit: the Box3D Packed Object asset "
                        "will not be built (run tools/make_packed_object.py with hython yourself)")
    endif()
endif()

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
if(BOX3DDOP_MAKE_HDA AND BOX3DDOP_HYTHON)
    # The asset the post-build step made (absent if it failed: OPTIONAL).
    install(FILES "$<TARGET_FILE_DIR:Box3DDOP>/../otls/box3d_packedobject.hda" DESTINATION Box3D/otls OPTIONAL)
endif()

set(CPACK_GENERATOR ZIP)
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_PACKAGE_FILE_NAME "Box3D-Houdini-${BOX3DDOP_HOUDINI_VERSION}-${CMAKE_SYSTEM_NAME}")
include(CPack)
