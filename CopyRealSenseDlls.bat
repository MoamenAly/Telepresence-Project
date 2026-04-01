@echo off
setlocal enabledelayedexpansion

set "DEST=%~dp0Assets\Plugins"

echo ============================================================
echo  Intel RealSense DLL Copy Helper
echo  Destination: %DEST%
echo ============================================================
echo.

if not exist "%DEST%" (
    mkdir "%DEST%"
    echo Created %DEST%
)

set "FOUND="

rem --- Try Intel RealSense SDK 2.0 default install (x64) ---
set "SDK_DIR=C:\Program Files (x86)\Intel RealSense SDK 2.0\bin\x64"
if exist "%SDK_DIR%\realsense2.dll" (
    set "FOUND=%SDK_DIR%"
    goto :docopy
)

rem --- Try Program Files variant ---
set "SDK_DIR=C:\Program Files\Intel RealSense SDK 2.0\bin\x64"
if exist "%SDK_DIR%\realsense2.dll" (
    set "FOUND=%SDK_DIR%"
    goto :docopy
)

rem --- Try librealsense build output ---
set "SDK_DIR=%USERPROFILE%\source\repos\librealsense\build\Release"
if exist "%SDK_DIR%\realsense2.dll" (
    set "FOUND=%SDK_DIR%"
    goto :docopy
)

rem --- Try nuget cache (common with Unity wrapper) ---
for /d %%G in ("%USERPROFILE%\.nuget\packages\intel.realsense\*") do (
    if exist "%%G\lib\net45\Intel.RealSense.dll" (
        set "NUGET_MANAGED=%%G\lib\net45"
    )
    if exist "%%G\runtimes\win-x64\native\realsense2.dll" (
        set "NUGET_NATIVE=%%G\runtimes\win-x64\native"
    )
)
if defined NUGET_MANAGED if defined NUGET_NATIVE (
    echo Found DLLs in NuGet cache.
    copy /Y "%NUGET_MANAGED%\Intel.RealSense.dll" "%DEST%\" >nul
    copy /Y "%NUGET_NATIVE%\realsense2.dll" "%DEST%\" >nul
    goto :verify
)

rem --- Nothing found automatically ---
echo.
echo ERROR: Could not locate Intel RealSense DLLs automatically.
echo.
echo Please copy these two files manually into:
echo   %DEST%
echo.
echo Required files:
echo   1. realsense2.dll        (native C library)
echo   2. Intel.RealSense.dll   (managed C# wrapper)
echo.
echo Typical sources:
echo   - Intel RealSense SDK 2.0 install folder\bin\x64\
echo   - librealsense GitHub release assets (Unity package)
echo   - NuGet package Intel.RealSense
echo.
pause
exit /b 1

:docopy
echo Found DLLs in: %FOUND%
copy /Y "%FOUND%\realsense2.dll" "%DEST%\" >nul

if exist "%FOUND%\Intel.RealSense.dll" (
    copy /Y "%FOUND%\Intel.RealSense.dll" "%DEST%\" >nul
) else (
    echo WARNING: Intel.RealSense.dll not found alongside realsense2.dll.
    echo          The managed wrapper may be in a different location (NuGet, Unity package).
)

:verify
echo.
set "OK=1"
if exist "%DEST%\realsense2.dll" (
    echo [OK] realsense2.dll
) else (
    echo [MISSING] realsense2.dll
    set "OK="
)
if exist "%DEST%\Intel.RealSense.dll" (
    echo [OK] Intel.RealSense.dll
) else (
    echo [MISSING] Intel.RealSense.dll
    set "OK="
)

echo.
if defined OK (
    echo SUCCESS: Both DLLs are in place. Open Unity and let it reimport.
) else (
    echo WARNING: One or more DLLs still missing. See messages above.
)
echo.
pause
