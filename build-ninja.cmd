@echo off
rem PERSONAL BUILD HELPER: tailored to Lexon's local Visual Studio installation.
rem This is not a portable or supported project build entry point.
set "PATH=C:\Program Files (x86)\Microsoft Visual Studio\Installer;%PATH%"
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>nul
where cl.exe >nul 2>nul
if %errorlevel% neq 0 (
    echo MSVC x64 compiler environment is unavailable.
    exit /b 1
)

set "BUILD_JOBS=%VIBESTATION_BUILD_JOBS%"
if not defined BUILD_JOBS set "BUILD_JOBS=6"

set "CMAKE_FRESH_ARG="
set "CMAKE_COMPILER_ARGS=-DCMAKE_C_COMPILER=cl -DCMAKE_CXX_COMPILER=cl"
if not exist build-ninja\CMakeCache.txt goto configure
if not exist build-ninja\build.ninja goto configure
%SystemRoot%\System32\findstr.exe /I /L /C:"cl.exe" build-ninja\CMakeCache.txt >nul 2>nul && goto build
%SystemRoot%\System32\findstr.exe /I /L /C:"CMAKE_CXX_COMPILER:STRING=cl" build-ninja\CMakeCache.txt >nul 2>nul && goto build
echo Existing build-ninja cache uses a different compiler; refreshing it for MSVC x64.
set "CMAKE_FRESH_ARG=--fresh"

:configure
echo Configuring CMake with Ninja...
cmake %CMAKE_FRESH_ARG% -G Ninja -S . -B build-ninja -DCMAKE_BUILD_TYPE=Release %CMAKE_COMPILER_ARGS% -DVIBESTATION_ENABLE_X64_JIT=ON -DVIBESTATION_ENABLE_IPO=OFF
if %errorlevel% neq 0 (
    echo CMake configuration failed.
    exit /b %errorlevel%
)

:build
echo Building VibeStation with Ninja using %BUILD_JOBS% parallel jobs...
cmake --build build-ninja --config Release --parallel %BUILD_JOBS%
if %errorlevel% neq 0 (
    echo Build failed.
    exit /b %errorlevel%
)
echo Build succeeded! The executable is located at build-ninja\VibeStation.exe
