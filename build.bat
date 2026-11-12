@echo off
setlocal enabledelayedexpansion

rem --- Find Houdini (no hardcoded path) -------------------------------------
rem  1. %HFS% if already set (Houdini "Command Line Tools" shell, or set it yourself)
rem  2. The registry: the installer records the active version under
rem     HKLM\SOFTWARE\Side Effects Software\ActiveVersion and that version's
rem     folder under ...\Houdini <version>\InstallPath
rem  3. The highest-numbered Houdini folder in %ProgramFiles%\Side Effects Software
if defined HFS if exist "!HFS!\toolkit\cmake" goto :have_hfs
set "HFS="

set "HVER="
for /f "tokens=2,*" %%A in ('reg query "HKLM\SOFTWARE\Side Effects Software" /v ActiveVersion 2^>nul ^| findstr /i "ActiveVersion"') do set "HVER=%%B"
if not defined HVER goto :scan_folders
for /f "tokens=2,*" %%A in ('reg query "HKLM\SOFTWARE\Side Effects Software\Houdini !HVER!" /v InstallPath 2^>nul ^| findstr /i "InstallPath"') do set "HFS=%%B"
if defined HFS if exist "!HFS!\toolkit\cmake" goto :have_hfs
set "HFS="

:scan_folders
rem Sort by version number, not alphabetically, so 22.0.1000 beats 22.0.479.
for /f "delims=" %%D in ('powershell -NoProfile -Command "Get-ChildItem -Directory -Path (Join-Path $env:ProgramFiles 'Side Effects Software') -Filter 'Houdini *' -EA SilentlyContinue | Where-Object { $_.Name -match '^Houdini (\d+\.\d+\.\d+)$' -and (Test-Path (Join-Path $_.FullName 'toolkit\cmake')) } | Sort-Object { [version]($_.Name -replace '^Houdini ','') } | Select-Object -Last 1 -ExpandProperty FullName"') do set "HFS=%%D"
if defined HFS goto :have_hfs

echo ERROR: Could not find a Houdini install with the HDK ^(toolkit\cmake^).
echo Run this from Houdini's "Command Line Tools" shell, or set HFS first, e.g.
echo     set "HFS=D:\Houdini\Houdini 22.0.479"
exit /b 1

:have_hfs
echo Using Houdini: !HFS!

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
echo BUILD OK. Close Houdini before rebuilding; the DLL is copied to Documents\houdiniX.Y\dso.
exit /b 0

:fail
echo.
echo BUILD FAILED
exit /b 1
