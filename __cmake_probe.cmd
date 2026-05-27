@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
cmake -S . -B build-vs -G "Visual Studio 18 2026" -A x64 --debug-output > build-vs-configure.log 2>&1
exit /b %errorlevel%
