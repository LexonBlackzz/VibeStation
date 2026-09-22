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

On Windows, opening `VibeStationPS2Lab.exe` normally creates a persistent
graphical window. If `scph39001.bin` is present in the current user's
Downloads folder, the app loads and starts it automatically. Otherwise use
the window's **Load BIOS** and **Start BIOS** buttons, or launch with
`--bios <path>`. A progress readout is displayed until the first GS pixels
appear. The reported EE instructions per second measure host interpreter
throughput, not the PS2's 294.912 MHz EE clock; hardware-accurate cycle
accounting and real-time pacing are not implemented yet.

Release builds use link-time optimization for the PS2 interpreter, and the
idle EE path skips DMA engines whose channels are not running. On the Windows
development machine, the same retail BIOS reached the 215-million-instruction
UI capture in 58 seconds after these changes, versus roughly two minutes
before. This is a measured host-runtime improvement, not a change to the
emulated EE clock or a guarantee of full-speed emulation on other machines.

## Experimental EE recompiler

An opt-in x64 EE JIT now emits native code for a first set of side-effect-free
register instructions. Run `VibeStationPS2Lab --ee-jit` or add `--ee-jit` to
the headless trace command to try it. The normal graphical launch still uses
the interpreter. All memory, branch, COP, VU, and device operations fall back
to the interpreter, which also continues to own fetch, interrupts, PC, and
timing. On the Windows development machine, the first 20 million BIOS
instructions included 10.9 million JIT-executed register instructions; a
warm headless trace took 2.97 seconds with JIT versus 3.48 seconds without.
The full UI startup improved only from 57.9 to 56.9 seconds, with identical
captured pixels. Larger gains require multi-instruction block compilation.

## Verified retail BIOS startup visual

On September 22, 2026, a legally dumped SCPH-39001 ROM0 produced the
recognizable PS2 startup cloud and floating-block scene in both the headless
trace and the `VibeStationPS2Lab` window. The first faint display pixels
appear around 211 million EE instructions; the scene is recognizable by
215 million. The VU0 macro/Q instruction correction in commit `9be2abd`
made the first frame possible. The later scanout fix presents the BIOS
framebuffer as opaque in the UI while retaining GS alpha for circuit merging.

To reproduce the UI verification with your own legally dumped BIOS:

```text
VibeStationPS2Lab --bios <bios-path> --capture-visible <window.ppm> --capture-after-ee 215000000
```

This runs the real graphical executable, captures its composed OpenGL window
as a PPM image, and exits after a visible frame at or beyond the requested
EE instruction count. For a headless framebuffer dump, pass an optional
PPM path after the trace budget:

```text
vibestation_ps2_bios_trace <bios-path> 215000000 <frame.ppm>
```

The startup scene is still dark and approximate; this is an experimental GS
renderer, not a fully accurate PS2. The later Sony/PlayStation 2 logo sequence
and BIOS chime have not been verified. SPU2 audio synthesis/output is not yet
implemented.

The Release UI boot path skips redundant EE-to-VU0 state copies while VIF0
DMA is idle, and uses longer, unsynchronized host frames only until the first
visible BIOS pixels. On the same Windows host, the 215-million-instruction
window capture fell from about 58 to 37 seconds, with byte-identical output.
This is still an experimental, instruction-driven bootstrap rather than a
full-speed PS2 emulator.

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
- an expanding R5900 interpreter with COP0/COP1 startup state and 128-bit LQ/SQ transfers;
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
