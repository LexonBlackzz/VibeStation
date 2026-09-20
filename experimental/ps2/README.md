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

The standalone graphical executable is `VibeStationPS2Lab`. A headless
`vibestation_ps2_bios_trace` executable is also built; pass it a BIOS path
and an optional EE instruction budget to print the exact EE/IOP boundary
without starting the UI.

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
- IOP reset-vector startup at `0xBFC00000`;
- reset-instruction fetch validation for both processors.

On Windows, use **File > Load BIOS...** or **Ctrl+B**. A manual BIOS path field is also available under **View > System**.

"Start BIOS" now begins concurrent EE and IOP BIOS execution. The EE starts at its R5900 reset state while an isolated R3000A IOP interpreter starts from the same ROM reset vector with its own COP0 state. During this early milestone the system advances one IOP instruction for every eight EE instructions, matching the normal PS2 startup clock relationship at instruction granularity. Unsupported instructions or device accesses halt with an exact processor/PC/opcode/reason instead of being silently ignored.

## Current scope

The experimental build currently contains:

- a 32 MiB EE RAM implementation;
- a minimal EE bus with RAM and ROM0 aliases;
- a deterministic event scheduler;
- an EE CPU state container;
- PS2 BIOS loading/reset startup;
- an expanding R5900 interpreter with COP0/COP1 startup state;
- normal and branch-likely delay-slot execution;
- HI/LO and HI1/LO1 multiply/divide paths used by the BIOS;
- 16 KiB EE scratchpad;
- early EE SIO, SBUS, RDRAM controller, DMAC, and Timer0 behavior, including BUSCLK divisors and the HBlank clock source used by BIOS timing calibration;
- a 2 MiB IOP RAM implementation, mirrored through the IOP's first 8 MiB and shared with the EE at physical `0x1C000000`;
- an isolated R3000A IOP interpreter with COP0 reset state, branch delay slots, load delay handling, exceptions, external interrupt sampling, unaligned word merges, and the MIPS-I startup instruction set;
- an IOP bus with BIOS, cache-control, hardware-register, explicit I_STAT/I_MASK/I_CTRL interrupt-controller semantics, partial SIF/SBUS, and early CDVD mappings;
- an IOP hardware-register window used by early BIOS probing;
- early CDVD byte-port state for N-READY/status/interrupts, N-command parameters, S-command parameters/results, deterministic RTC reads, mecacon version/tray queries, basic reset/NOP handling, and CDVD command-complete delivery on IOP IRQ2;
- GS privileged-register backing used during early display initialization;
- run/pause/single-step UI controls with explicit halt diagnostics;
- a `Ps2System` composition root;
- headless smoke tests;
- a standalone SDL/OpenGL/ImGui VibeStation-style UI;
- PS2 System, EE Debug, IOP Debug, Scheduler, Settings, and About panels.

The previous IOP-RAM handoff halt is now removed: EE accesses in the `0x1C000000` physical window and IOP accesses to their low-RAM mirrors refer to the same 2 MiB backing store. The BIOS timing calibration path now sees Timer0's external HBlank source instead of the old placeholder /16 clock. CDVD is intentionally still a protocol scaffold rather than a disc engine: register-level bootstrap commands work, while real seek/read media commands remain unimplemented. The next fidelity milestones are IOP timers/INTC/DMAC, stronger SIF synchronization, full CDVD command/media timing, SPU2-facing IOP hardware, and replacing the current instruction-granularity 8:1 startup interleave with event/cycle scheduling. Large parts of the R5900 instruction set, GS rendering, SPU2, ELF loading, and corruption support also remain incomplete.

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
