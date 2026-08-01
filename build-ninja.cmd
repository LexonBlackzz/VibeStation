@echo off
rem PERSONAL BUILD HELPER: tailored to Lexon's local Visual Studio installation.
rem This is not a portable or supported project build entry point.
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
where cl.exe >nul 2>nul
if %errorlevel% neq 0 (
    echo MSVC x64 compiler environment is unavailable.
    exit /b 1
)

set "CMAKE_FRESH_ARG="
if exist build-ninja\CMakeCache.txt (
    findstr /I "cl.exe" build-ninja\CMakeCache.txt >nul || (
        echo Existing build-ninja cache uses a different compiler; refreshing it for MSVC x64.
        set "CMAKE_FRESH_ARG=--fresh"
    )
)
echo Configuring CMake with Ninja...
cmake %CMAKE_FRESH_ARG% -G Ninja -S . -B build-ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=cl -DCMAKE_CXX_COMPILER=cl -DVIBESTATION_ENABLE_X64_JIT=ON -DVIBESTATION_ENABLE_IPO=OFF
if %errorlevel% neq 0 (
    echo CMake configuration failed.
    exit /b %errorlevel%
)
echo Building VibeStation with Ninja...
cmake --build build-ninja --config Release --parallel 4
if %errorlevel% neq 0 (
    echo Build failed.
    exit /b %errorlevel%
)
echo Build succeeded! The executable is located at build-ninja\VibeStation.exe
