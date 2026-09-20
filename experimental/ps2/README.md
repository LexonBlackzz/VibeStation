# VibeStation PS2 Experimental Core

This directory is an isolated PlayStation 2 research implementation for VibeStation.

The PS2 code intentionally does **not** include or depend on the existing PS1 core. During the early implementation phase, duplication is preferred over coupling the two emulators prematurely.

## Build

From the repository root:

```bash
cmake -S experimental/ps2 -B build-ps2
cmake --build build-ps2
ctest --test-dir build-ps2 --output-on-failure
```

The standalone graphical executable is `VibeStationPS2Lab`.

To build only the headless core/tests without SDL/ImGui dependencies:

```bash
cmake -S experimental/ps2 -B build-ps2-headless -DVIBESTATION_PS2_ENABLE_UI=OFF
cmake --build build-ps2-headless
ctest --test-dir build-ps2-headless --output-on-failure
```

## BIOS startup

VibeStation does not include a PlayStation 2 BIOS. Supply a 4 MiB BIOS image dumped from hardware you own.

The current BIOS path implements:

- 4 MiB ROM0 loading and validation;
- ROMVER discovery from the BIOS ROMDIR;
- physical ROM mapping at `0x1FC00000`;
- cached ROM alias at `0x9FC00000`;
- uncached ROM alias at `0xBFC00000`;
- read-only BIOS bus behavior;
- EE reset-vector startup at `0xBFC00000`;
- reset-instruction fetch validation.

On Windows, use **File > Load BIOS...** or **Ctrl+B**. A manual BIOS path field is also available under **View > System**.

"Start BIOS" currently establishes the real reset state and verifies instruction fetch. It does not execute the BIOS yet; the R5900 interpreter and COP0 implementation are the next execution milestone.

## Current scope

The experimental build currently contains:

- a 32 MiB EE RAM implementation;
- a minimal EE bus with RAM and ROM0 aliases;
- a deterministic event scheduler;
- an EE CPU state container;
- PS2 BIOS loading/reset startup;
- a `Ps2System` composition root;
- headless smoke tests;
- a standalone SDL/OpenGL/ImGui VibeStation-style UI;
- PS2 System, EE Debug, Scheduler, Settings, and About panels.

ELF loading, R5900 instruction execution, GS rendering, IOP, SPU2, and corruption support are not implemented yet.

## UI isolation

The PS2 build mirrors the VibeStation visual language, including the existing theme system, but maintains a separate PS2 application class and a separate ImGui settings file:

```text
vibestation_ps2_imgui.ini
```

It does not instantiate or link the PS1 `System`, GPU, SPU, renderer, EmuRunner, memory-card UI, or corruption runtime.

## Isolation rule

Until the PS2 core reaches a stable execution milestone:

- do not add PS2 conditionals to PS1 source files;
- do not reuse `src/core/gpu.*`, `src/core/system.*`, or the PS1 CPU/SPU directly;
- do not require the repository root `CMakeLists.txt` to build this project.

Shared abstractions should be extracted only after both implementations make the common boundary obvious.
