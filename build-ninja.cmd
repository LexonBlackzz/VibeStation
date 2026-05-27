@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
echo Configuring CMake with Ninja...
cmake -G Ninja -S . -B build-ninja -DCMAKE_BUILD_TYPE=Release
if %errorlevel% neq 0 (
    echo CMake configuration failed.
    exit /b %errorlevel%
)
echo Building VibeStation with Ninja...
cmake --build build-ninja --config Release
if %errorlevel% neq 0 (
    echo Build failed.
    exit /b %errorlevel%
)
echo Build succeeded! The executable is located at build-ninja\VibeStation.exe
