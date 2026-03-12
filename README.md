# VibeStation
This is a fully vibe-coded PS1 emulator for fun. It is written in C++.

## Things implemented:
BIOS loading ✅  
Game loading 🚩  
Bindings ✅  
CD-ROM ⚠️  
Graphics/VRAM ✅⚠️  
Audio ✅  
CPU/RAM/DMA ✅   
Main UI ✅  
MDEC 🚩  
Real-time corrupter/BIOS corruption ✅   
Memory Cards ✅

✅ - Implemented  
⚠️ - Implemented, buggy  
🚩 - Incomplete  
❌ - Not implemented/Doesn't work   


# Introducing Grim Reaper
<img width="607" height="132" alt="VibeStation_80u8zFqD9g" src="https://github.com/user-attachments/assets/d38f38f6-4fb9-4fb4-b911-f24bad1c29f8" />  

*Grim Reaper* is a built-in tool specifically made for **corruptions**.  Grim Reaper uses Random Strike to do all of its corruptions (essentially randomly hitting any byte).  
### Modes:  
**BIOS corruptor**

Essentially, it almost works the same way as "Vinesauce ROM Corruptor". It corrupts your selected BIOS and boots it up.

<img width="524" height="67" alt="image" src="https://github.com/user-attachments/assets/a5748dcc-c799-4130-a226-37bd07bebc16" />  

Intro/Bootmenu hits the sensitive areas of the BIOS (0x18000-0x63FFF). 0.01% is a safe parameter for a successful boot sequence.  

<img width="543" height="46" alt="image" src="https://github.com/user-attachments/assets/060ecf4c-c7e2-4e06-bebf-38879bb6ee73" />

Character Sets is a fun corruption that corrupts the characters of the BIOS. The corruption is visible at the PlayStation logo text.

<img width="547" height="47" alt="image" src="https://github.com/user-attachments/assets/3ef16cd7-3206-408c-bdb9-49e7069e584a" />

End will usually not affect your BIOS, but it can prevent game loading.

**Batch Corruption**

<img width="607" height="156" alt="image" src="https://github.com/user-attachments/assets/8ecd6130-340f-4550-91ec-1b3b66a4eda5" />

Allows for corrupting multiple parts of the BIOS. Same principle is applied when corrupting the intro, char sets and more.

**RAM Reaper**

<sup> You can call this cartridge tilting if you want  

<img width="607" height="299" alt="image" src="https://github.com/user-attachments/assets/c37070a0-16c3-453b-b3dd-d2b831e8cc3a" />

*RAM Reaper* does exactly what it says. It corrupts the memory of the PS1. It can corrupt the main memory, video memory, sound memory or all of them at once. It uses a random or custom seed for the corruption. 

RAM Reaper will hit any byte in your hex range randomly every frame.  
⚠️ Main RAM is very delicate and with high parameters it *will* hang the emulator, you will need to restart the BIOS.  
✅ VRAM and SPU RAM is safe to hit at any parameters if you want fun glitches.  

**GPU Reaper**

<img width="692" height="200" alt="image" src="https://github.com/user-attachments/assets/5c91163d-fed5-4ca7-97f5-062e0f447c5b" />


Corrupts or warps draw offsets, draw areas, textures, and optional display state, so you get broken polygons and unstable rendering rather than only VRAM trashing from RAM reaper. Produces funny results, and is stable with any value set.

**Sound Reaper**

<img width="662" height="216" alt="image" src="https://github.com/user-attachments/assets/0dd2639d-a150-4ff5-a991-0c06350c11ef" />


Unlike targetting SPU RAM in RAM reaper, this one targets the reverb, delay, whatever SPU effects is implemented, resulting in hilarious audio corruptions.  
You can also save a certain sound sample from a specific voice channel and apply it to all channels.  

*All reapers include a preset saver/list with their seeds, so you can share your funny results with anyone having this emulator.*

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

### ⚠️ As of v0.4.5a, Discord Social SDK for RPC will be implemented, however the source code will NOT include the SDK, you must download it yourself on Discord's Developer Portal.
To build the Discord RPC:   
Run   
`cmake -S . -B build-vs -DVIBESTATION_DISCORD_SDK_ROOT="C:/path/to/discord_social_sdk"`

*C:/path/to/discord_social_sdk should be replaced with where you kept the discord_social_sdk folder*


#### ⚠️ The program does ***not*** provide any kind of BIOS files and does not condone piracy. ***All*** BIOS files used must be legally obtained or extracted from your owned console.

### ⚠️ MANY THINGS ARE EXPERIMENTAL, NOT IMPLEMENTED AND/OR NOT WORKING!
