@echo off
rem Finds the Houdini install and sets HFS in the calling script's environment.
rem Used by build.bat, test.bat and make_examples.bat:  call "%~dp0tools\find_houdini.bat"
rem Exits with errorlevel 1 (and a message) when no install with the HDK is found.
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
endlocal & set "HFS=%HFS%"
exit /b 0
