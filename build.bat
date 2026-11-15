@echo off
setlocal enabledelayedexpansion

rem --- Find Houdini: HFS, else the registry, else the newest install (tools\find_houdini.bat) ---
call "%~dp0tools\find_houdini.bat"
if errorlevel 1 exit /b 1
echo Using Houdini: !HFS!

cd /d "%~dp0"

rem --- Debug logging (prints to Houdini's console): off unless you run "build debug" ---
set "DBG=OFF"
if /i "%~1"=="debug" set "DBG=ON"
echo Debug logging: !DBG!

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

rem --- Configure (every build, so BOX3DDOP_DEBUG never keeps a stale cached value) ---
if exist build\CMakeCache.txt goto :reconfigure
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DBOX3DDOP_DEBUG=!DBG!
if not "%errorlevel%"=="0" goto :fail
>"%MARKER%" echo %CD%
goto :build

:reconfigure
cmake -S . -B build -DBOX3DDOP_DEBUG=!DBG!
if not "%errorlevel%"=="0" goto :fail

:build
rem --- Build ---------------------------------------------------------------
cmake --build build --config Release
if not "%errorlevel%"=="0" goto :fail

echo.
echo BUILD OK. Close Houdini before rebuilding; the DLL is copied to Documents\houdiniX.Y\dso.
exit /b 0

:fail
echo.
echo BUILD FAILED
exit /b 1
