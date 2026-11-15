@echo off
setlocal enabledelayedexpansion

rem Builds the example scenes (examples\make_examples.py) with hython, into examples\scenes.
rem   make_examples.bat                   every scene
rem   make_examples.bat building shapes   some scenes
rem   make_examples.bat --check           also simulate each scene and print timings
rem Build the plugin first (build.bat): the scenes need it loaded.
rem Finds Houdini the same way as build.bat.

rem --- Find Houdini: HFS, else the registry, else the newest install (tools\find_houdini.bat) ---
call "%~dp0tools\find_houdini.bat"
if errorlevel 1 exit /b 1
set "HYTHON=!HFS!\bin\hython.exe"
if not exist "!HYTHON!" (
    echo ERROR: hython not found at "!HYTHON!".
    exit /b 1
)
echo Using Houdini: !HFS!

cd /d "%~dp0"
"!HYTHON!" "examples\make_examples.py" %*
exit /b %errorlevel%
