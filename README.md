# VibeStation
This is a fully vibe-coded PS1 emulator for fun. It is written in C++.

## Things implemented:
BIOS loading ✅  
Game loading ⚠️❌  
Bindings ✅⚠️  
CD-ROM ⚠️  
Graphics/VRAM ⚠️  
Audio ✅  
CPU/RAM/DMA ✅  
Main UI ✅  
✅ - Implemented  
✅⚠️ - Implemented, has issues  
⚠️ - Implemented, Has big issues  
❌ - Not implemented/Doesn't work   

## Build requirements
Atleast VS 2022 (tested with vs 2022, 2026)  
CMake 3.20+ (tested with CMake 4.2.2)  
Access to internet (to download dependencies)  

# Build
Run `cmake -S . -B build-vs -G "Visual Studio 17 2022" -A x64`  
and  
`cmake --build build-vs --config Release` (or compile inside Visual Studio)    
  
    
#### ⚠️ The program does ***not*** provide any kind of BIOS files and does not condone piracy. ***All*** BIOS files used must be legally obtained or extracted from your owned console.

### ⚠️ MANY THINGS ARE EXPERIMENTAL, NOT IMPLEMENTED AND/OR NOT WORKING!
