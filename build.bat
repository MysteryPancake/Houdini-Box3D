@echo off
setlocal enabledelayedexpansion

rem --- Houdini install (edit this if you upgrade Houdini) -------------------
set "HFS=C:\Program Files\Side Effects Software\Houdini 22.0.479"

if not exist "%HFS%\toolkit\cmake" (
    echo ERROR: Houdini toolkit not found at "%HFS%\toolkit\cmake"
    echo Edit HFS at the top of build.bat.
    exit /b 1
)

cd /d "%~dp0"

rem --- Make sure the Box3D submodule is present ----------------------------
if not exist "extern\box3d\CMakeLists.txt" (
    echo Fetching Box3D submodule...
    git submodule update --init --recursive
)

rem --- Wipe build\ if this folder was moved or renamed ---------------------
set "MARKER=build\.configured_from"
if exist "%MARKER%" (
    set "OLDDIR="
    for /f "usebackq delims=" %%A in ("%MARKER%") do set "OLDDIR=%%A"
    if /i not "!OLDDIR!"=="%CD%" (
        echo Folder changed from "!OLDDIR!", resetting build folder...
        rmdir /s /q build
    )
)

rem --- Configure (first time, or after the build folder was reset) ---------
if not exist build\CMakeCache.txt (
    cmake -S . -B build -G "Visual Studio 17 2022" -A x64
    if not "%errorlevel%"=="0" goto :fail
    >"%MARKER%" echo %CD%
)

rem --- Build ---------------------------------------------------------------
cmake --build build --config Release
if not "%errorlevel%"=="0" goto :fail

echo.
echo BUILD OK. Close Houdini before rebuilding; the DLL is copied to Documents\houdini22.0\dso.
exit /b 0

:fail
echo.
echo BUILD FAILED
exit /b 1
