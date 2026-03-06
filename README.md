# VibeStation
This is a fully vibe-coded PS1 emulator for fun. It is written in C++.

## Things implemented:
BIOS loading ✅  
Game loading 🚩  
Bindings ⚠️  
CD-ROM ⚠️  
Graphics/VRAM ✅⚠️  
Audio ✅  
CPU/RAM/DMA ✅   
Main UI ✅  
Real-time corrupter/BIOS corruption ✅

✅ - Implemented  
⚠️ - Implemented, buggy  
🚩 - Incomplete  
❌ - Not implemented/Doesn't work   


# Introducing Grim Reaper
<img width="607" height="132" alt="VibeStation_80u8zFqD9g" src="https://github.com/user-attachments/assets/d38f38f6-4fb9-4fb4-b911-f24bad1c29f8" />  

*Grim Reaper* is a built-in tool specifically made for **corruptions**.  Grim Reaper uses Random Strike to do all of its corruptions (essentially randomly hitting any byte).  
### Modes:  
**RAM Reaper**

<sup> You can call this cartridge tilting if you want  

<img width="607" height="299" alt="image" src="https://github.com/user-attachments/assets/c37070a0-16c3-453b-b3dd-d2b831e8cc3a" />

*RAM Reaper* does exactly what it says. It corrupts the memory of the PS1. It can corrupt the main memory, video memory, sound memory or all of them at once. It uses a random or custom seed for the corruption.  
RAM Reaper will hit any byte in your hex range randomly every frame.  
⚠️ Main RAM is very delicate and with high parameters it *will* hang the emulator, you will need to restart the BIOS.  
✅ VRAM and SPU RAM is safe to hit at any parameters if you want fun glitches.  


**Batch Corruption**

<img width="607" height="156" alt="image" src="https://github.com/user-attachments/assets/8ecd6130-340f-4550-91ec-1b3b66a4eda5" />

Essentially, it is "Vinesauce ROM Corruptor". It corrupts your selected BIOS and boots it up.

<img width="524" height="67" alt="image" src="https://github.com/user-attachments/assets/a5748dcc-c799-4130-a226-37bd07bebc16" />  

Intro/Bootmenu hits the sensitive areas of the BIOS (0x18000-0x63FFF). 0.01% is a safe parameter for a successful boot sequence.  

<img width="543" height="46" alt="image" src="https://github.com/user-attachments/assets/060ecf4c-c7e2-4e06-bebf-38879bb6ee73" />

Character Sets is a fun corruption that corrupts the characters of the BIOS. The corruption is visible at the PlayStation logo text.

<img width="547" height="47" alt="image" src="https://github.com/user-attachments/assets/3ef16cd7-3206-408c-bdb9-49e7069e584a" />

End will usually not affect your BIOS, but it can prevent game loading.

## Build requirements
Atleast VS 2022 (tested with vs 2022, 2026)  
CMake 3.20+ (tested with CMake 4.2.2)  
Access to internet (to download dependencies)  

# Build
Run  
`cmake -S . -B build-vs -G "Visual Studio 17 2022" -A x64`  
and  
`cmake --build build-vs --config Release`  
(or compile inside Visual Studio)    
  
    
#### ⚠️ The program does ***not*** provide any kind of BIOS files and does not condone piracy. ***All*** BIOS files used must be legally obtained or extracted from your owned console.

### ⚠️ MANY THINGS ARE EXPERIMENTAL, NOT IMPLEMENTED AND/OR NOT WORKING!
