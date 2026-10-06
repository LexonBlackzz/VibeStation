@echo off
rem VibeStation PS2 Lab - MSVC + Ninja profile-guided (PGO) build
rem
rem   build-ps2-pgo.cmd <path\to\4MiB-ROM0-bios.bin> [training EE instructions]
rem
rem Builds an instrumented trace, runs it on your BIOS, then rebuilds the trace
rem and the app with that profile. Training defaults to 450M instructions so
rem it covers the steady-state boot animation (GS rasterizer + EE interpreter),
rem not just the first visible frame. Re-run after core code changes.
rem Output: build-ps2-pgo\VibeStationPS2Lab.exe

if "%~1"=="" (
    echo usage: build-ps2-pgo.cmd ^<bios.bin^> [training EE instructions]
    exit /b 1
)
set "BIOS=%~f1"
set "TRAIN=%~2"
if "%TRAIN%"=="" set "TRAIN=450000000"

set "PATH=C:\Program Files (x86)\Microsoft Visual Studio\Installer;%PATH%"
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>nul
where cl.exe >nul 2>nul
if %errorlevel% neq 0 (
    echo MSVC x64 compiler environment is unavailable.
    exit /b 1
)

set "BUILD=build-ps2-pgo"
set "PGD=%CD:\=/%/%BUILD%/vibestation_ps2_bios_trace.pgd"

echo [1/3] Building instrumented trace...
cmake --fresh -G Ninja -S experimental/ps2 -B %BUILD% ^
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=cl ^
    -DVIBESTATION_PS2_ENABLE_UI=OFF -DBUILD_TESTING=OFF ^
    "-DCMAKE_EXE_LINKER_FLAGS_RELEASE=/INCREMENTAL:NO /GENPROFILE"
if %errorlevel% neq 0 exit /b %errorlevel%
cmake --build %BUILD% --target vibestation_ps2_bios_trace --parallel 4
if %errorlevel% neq 0 exit /b %errorlevel%

rem pgort140.dll is only needed to run the instrumented binary.
copy /y "%VCToolsInstallDir%bin\Hostx64\x64\pgort140.dll" %BUILD%\ >nul
if %errorlevel% neq 0 (
    echo pgort140.dll not found; install the MSVC x64 PGO tools.
    exit /b 1
)

echo [2/3] Training on %TRAIN% EE instructions (slow, ~1-2 minutes)...
%BUILD%\vibestation_ps2_bios_trace.exe "%BIOS%" %TRAIN% --gs-thread >nul
if %errorlevel% neq 0 (
    echo PGO training run failed.
    exit /b %errorlevel%
)

echo [3/3] Building optimized app and trace...
cmake -G Ninja -S experimental/ps2 -B %BUILD% ^
    -DVIBESTATION_PS2_ENABLE_UI=ON ^
    "-DCMAKE_EXE_LINKER_FLAGS_RELEASE=/INCREMENTAL:NO /USEPROFILE:PGD=%PGD%"
if %errorlevel% neq 0 exit /b %errorlevel%
cmake --build %BUILD% --target vibestation_ps2_bios_trace VibeStationPS2Lab --parallel 4
if %errorlevel% neq 0 exit /b %errorlevel%

echo.
echo Build succeeded!
echo Executable: %BUILD%\VibeStationPS2Lab.exe
