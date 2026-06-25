<img width="1500" height="500" alt="VibeStation banner" src="https://github.com/user-attachments/assets/a1dd0bf5-1405-4267-895d-f624cf647bce" />

VibeStation is an experimental PS1 emulator with a built-in game corruptor, written in C++.

The emulator is built for fun, experimentation, debugging, and breaking games in interesting ways. Its main unique feature is Grim Reaper, a set of built-in corruption tools for BIOS, RAM, GPU, and SPU/audio corruption.

## Feature Status:
BIOS loading ✅  
Game loading ✅  
Bindings ✅  
CD-ROM ✅  
Graphics/VRAM ✅⚠️  
Audio ✅  
CPU/RAM/DMA ✅   
Main UI ✅  
MDEC ⚠️ (For GT2 and RR4 - ✅)  
Real-time corrupter/BIOS corruption ✅   
Memory Cards ✅

✅ - Implemented  
⚠️ - Implemented, buggy  
🚩 - Incomplete  
❌ - Not implemented/Doesn't work   


# Grim Reaper
<img width="607" height="132" alt="Grim Reaper Panel" src="https://github.com/user-attachments/assets/d38f38f6-4fb9-4fb4-b911-f24bad1c29f8" />  

*Grim Reaper* is VibeStation's built-in toolset for **corruptions**.  
Grim Reaper uses a "Random Strike" system to do all of its corruptions (essentially randomly hitting any byte). The goal is to create funny, broken, unstable, or interesting results directly inside the emulator without needing a separate external corruptor. 
### Modes:  
**BIOS corruptor**

Essentially, it almost works the same way as "Vinesauce ROM Corruptor". It corrupts your selected BIOS and boots it up.


<img width="524" height="67" alt="Intro Bootmenu Slider" src="https://github.com/user-attachments/assets/a5748dcc-c799-4130-a226-37bd07bebc16" />  

Intro/Bootmenu hits the sensitive areas of the BIOS (0x18000-0x63FFF). `0.01%` is a safe parameter for a successful boot sequence.  

<img width="543" height="46" alt="Character Sets Slider" src="https://github.com/user-attachments/assets/060ecf4c-c7e2-4e06-bebf-38879bb6ee73" />

Character Set corruption targets BIOS text/character data. The result is visible on the PlayStation logo text.

<img width="547" height="47" alt="End Slider" src="https://github.com/user-attachments/assets/3ef16cd7-3206-408c-bdb9-49e7069e584a" />

End-region corruption usually does not visibly affect the BIOS, but it can prevent games from loading.

**Batch Corruption**

<img width="607" height="156" alt="image" src="https://github.com/user-attachments/assets/8ecd6130-340f-4550-91ec-1b3b66a4eda5" />

Batch Corruption allows multiple BIOS regions to be corrupted at once. The same basic corruption is used for the intro, character sets, and end region areas.  

**RAM Reaper**

<sup> You can call this cartridge tilting if you want  

<img width="607" height="299" alt="RAM Reaper Panel" src="https://github.com/user-attachments/assets/c37070a0-16c3-453b-b3dd-d2b831e8cc3a" />

*RAM Reaper* corrupts PS1 memory in real-time. 
It can target:  
* Main RAM
* VRAM
* Sound RAM
* All of them at once.
  
It uses a random or custom seed for the corruption. 

RAM Reaper will hit any byte in your hex range randomly every frame.  
> ⚠️ Main RAM is very delicate and with high parameters it *will* hang the emulator, you will need to restart the BIOS.  
> ✅ VRAM and SPU RAM is safe to hit at any parameters if you want fun glitches.  

**GPU Reaper**

<img width="692" height="200" alt="GPU Reaper Panel" src="https://github.com/user-attachments/assets/5c91163d-fed5-4ca7-97f5-062e0f447c5b" />


GPU Reaper corrupts or warps draw offsets, draw areas, textures, and optional display state, so you get broken polygons and unstable rendering rather than only VRAM trashing from RAM reaper. Produces funny results, and is stable with any value set.

**Sound Reaper**

<img width="662" height="216" alt="image" src="https://github.com/user-attachments/assets/0dd2639d-a150-4ff5-a991-0c06350c11ef" />

Sound Reaper targets SPU/audio behavior rather than simply corrupting SPU RAM.
It targets the reverb, delay, and other SPU effects that are implemented, resulting in broken or unusual audio output.   
It also includes a feature for saving a certain sound sample from a specific voice channel and applying it to all channels.  

**Presets**
All reapers include a preset saver/list with their seeds, making it possible to save share your results with other VibeStation emulator users.

## Build requirements
Atleast Visual Studio 2022 (tested with VS 2022, 2026)  
CMake 3.20+ (tested with CMake 4.2.2)  
Access to internet (to download dependencies)  

# Build
Generate Visual Studio build files:  
`cmake -S . -B build-vs -G "Visual Studio 17 2022" -A x64`  
Build the release configuration:  
`cmake --build build-vs --config Release`  
You can also build the project inside Visual Studio.

# Discord RPC Support
As of `v0.4.5a`, Discord RPC support is included, but the repository does **not** ship with the Discord Social SDK.  
If you want working Discord RPC when building from source, download the SDK from Discord's Developer Portal and build with it yourself. This feature is entirely optional to you.  
Provide its path when configuring the project:  
`cmake -S . -B build-vs -DVIBESTATION_DISCORD_SDK_ROOT="C:/path/to/discord_social_sdk"`

Replace C:/path/to/discord_social_sdk with the location of your Discord Social SDK folder.
If the SDK is not provided, the project will still build with a stub fallback, but Discord RPC will not function.

#BIOS Disclaimer  
VibeStation does ***not*** provide any kind of BIOS files. You must legally obtain or extract BIOS files from hardware you own. This project does not condone piracy.

⚠️ VibeStation is still experimental. Many features are incomplete, inaccurate, buggy, or unstable. Compatibility is not guaranteed.
