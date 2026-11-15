@echo off
setlocal enabledelayedexpansion

rem Runs the Houdini checks (tests\houdini\run_tests.py) with hython.
rem   test.bat                    every test
rem   test.bat pscale network     some tests
rem   test.bat --list             list the tests
rem   test.bat --failed           only what failed (or never ran) last time; add --info for INFO too
rem Build the plugin first (build.bat): the checks load it from the dso folder.
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
"!HYTHON!" "tests\houdini\run_tests.py" %*
exit /b %errorlevel%
