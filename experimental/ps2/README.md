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

To measure sustained BIOS animation throughput rather than only time to the
first picture, run the trace beyond 212 million EE cycles with `--profile`:

```text
vibestation_ps2_bios_trace <bios-path> 260000000 --profile
```

The graphical app uses a separate GS raster worker so EE/IOP execution can
overlap ordered software draw commands. Add `--gs-thread` to the trace to
measure the same path; omit it for a single-threaded comparison. Draws are
drained before VRAM transfers and display reads. On the development machine,
the 260-million-instruction PGO trace improved from 2.05 to 3.11 emulated
fields/s with the same BIOS display hash and GS raster counters. This is
still far short of real-time animation.
The UI also stops rescanning VRAM on every host repaint after the first
visible frame; in a 260-million-instruction window capture, this reduced
wall time from 10.39 to 8.88 seconds with the threaded GS build.

`PROFILE_FIELD_RATE` counts emulated NTSC video fields per host second after
the first visible picture. `PROFILE_RUN_MS` and `PROFILE_DISPLAY_MS` separate
core execution from display scanout. A 30 FPS interlaced target requires
roughly 60 fields per second; a fast UI repaint alone does not imply the
emulation is running at that rate.

### Windows profile-guided fast boot

For the fastest BIOS startup on an MSVC x64 build, train a profile with your
own 4 MiB ROM0 BIOS and then build the graphical executable:

```powershell
.\experimental\ps2\scripts\build-pgo.ps1 -BiosPath 'C:\path\to\your\bios.bin'
.\build-ps2-pgo\Release\VibeStationPS2Lab.exe --bios 'C:\path\to\your\bios.bin'
```

The script first runs an instrumented 212-million-instruction BIOS trace,
then uses that profile to optimize the shared PS2 core in both the trace and
the app. Training takes longer than a normal boot and is needed again after
core code changes. The BIOS and generated profile stay local; neither is
included in the repository. An installed Visual Studio MSVC x64 toolchain is
required. On the development machine, the optimized first visible headless
frame took about 4.7 seconds, and the graphical capture reached it in about
4.9 seconds from process entry. Results depend on BIOS, hardware, and host
load. While the screen is blank, the UI prioritizes uninterrupted emulation
and may not respond to input until the first visible frame.

On Windows, opening `VibeStationPS2Lab.exe` without arguments creates a
persistent graphical window and does not load or start a BIOS automatically.
Use the window's **Load BIOS** and **Start BIOS** buttons to boot manually,
or launch with `--bios <path>` to explicitly boot on startup. A progress
readout is displayed until the first GS pixels appear. The reported EE
instructions per second measure host interpreter
throughput, not the PS2's 294.912 MHz EE clock; hardware-accurate cycle
accounting and real-time pacing are not implemented yet.

Release builds use link-time optimization for the PS2 interpreter, and the
idle EE path skips DMA engines whose channels are not running. On the Windows
development machine, the same retail BIOS reached the 215-million-instruction
UI capture in 58 seconds after these changes, versus roughly two minutes
before. This is a measured host-runtime improvement, not a change to the
emulated EE clock or a guarantee of full-speed emulation on other machines.

## EE execution backends

The cached/predecoded interpreter remains the default and is the correctness
baseline. The original `EeJit` is retained behind `--ee-jit` for historical
comparison only.

A separate second-generation x64 backend is available with
`--ee-dynarec`. It does not extend the original JIT. It builds
multi-instruction RAM-resident basic blocks (up to 64 guest instructions),
keeps a scored hot set of guest GPRs plus HI/LO in host registers for each
block, uses dirty-register writeback, and performs guarded main-RAM fastmem.
Ordinary RAM stores carry code-page generation barriers, including wide
LQ/SQ and VU/FPU transfers, so self-modifying code exits before stale native
instructions can run.

The native integer/control subset includes ordinary and likely MIPS branches,
REGIMM link/likely variants, J/JAL/JR/JALR, delay-slot annul semantics,
variable and immediate 32/64-bit shifts, conditional moves, SA helpers,
MTSAB/MTSAH, and the common scalar integer operations. COP1 register/control
moves and BC1F/BC1T/BC1FL/BC1TL remain inside native blocks. Select-0 COP0
reads/writes are supported; MTC0 Count is isolated to preserve its exact
Count-retirement ordering, while state-changing MTC0 operations plus EI/DI
are precise exits so the system immediately resamples interrupts and devices.

Compiled successor blocks are cached and followed inside one dynarec dispatch,
avoiding a return through the main PS2 run loop at every basic block. The
system still owns timing: video transitions, SIF completion, EE timers, COP0
Compare and the exact EE/IOP 8:1 boundary define the maximum retirement
deadline passed to the backend. Deadline-capped block formation prevents a
native block from crossing one of those boundaries. In dynarec mode repeated
exact IOP sub-deadlines stay inside the dynarec-only quiet super-dispatch; the
default interpreter continues using its proven normal path.

The trace reports native coverage, block/dispatch efficiency, link hit/miss
counts, guard/deadline/COP0 exits, fastmem activity, register-cache use, cache
flushes, and a top unsupported-opcode histogram. These counters are intended
to drive later ISA expansion rather than adding speculative emitters.

The two native modes are mutually exclusive:

```text
vibestation_ps2_bios_trace <bios> 400000000 --profile --gs-thread --ee-dynarec
VibeStationPS2Lab --bios <bios> --ee-dynarec
```

Use the repeatable Windows comparison harness to validate both performance
and output:

```powershell
.\experimental\ps2\scripts\benchmark-ee-dynarec.ps1 `
    -BiosPath 'C:\path\to\your\bios.bin'
```

It runs three 400M Release traces for the cached interpreter and the
second-generation dynarec, reports median field rate/run time, and fails if
the dynarec changes the interpreter's final display hash or raster-pixel
count. The dynarec remains opt-in until that comparison demonstrates a
reliable sustained BIOS win.

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

The startup scene is still approximate; this is an experimental GS renderer,
not a fully accurate PS2. SPU2 synthesis, headless PCM/WAV capture, and SDL
audio playback are implemented for the BIOS path, but timing/mixing fidelity
remains experimental and should still be validated alongside visual output
after performance changes.

The Release UI boot path skips redundant EE-to-VU0 state copies while VIF0
DMA is idle, and uses longer, unsynchronized host frames only until the first
visible BIOS pixels. On the same Windows host, the 215-million-instruction
window capture fell from about 58 to 37 seconds, with byte-identical output.
Matching the EE/IOP SIF DMA channels before servicing them then cut that
capture to about 18 seconds; a 225-million-instruction capture fell from
about 40 to 22 seconds on the same host. The visible images remained
byte-identical. These are host-specific measurements, not real-time speed.
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

The previous IOP-RAM handoff halt is now removed: EE accesses in the `0x1C000000` physical window and IOP accesses to their low-RAM mirrors refer to the same 2 MiB backing store. The BIOS timing calibration path now sees Timer0's external HBlank source instead of the old placeholder /16 clock. CDVD is intentionally still a protocol scaffold rather than a disc engine: register-level bootstrap commands work, while real seek/read media commands remain unimplemented. The next fidelity milestones are stronger IOP timer/INTC/DMAC coverage,
stronger SIF synchronization, full CDVD command/media timing, more accurate
SPU2 mixing/timing, and further replacement of instruction-granularity
interleave with event/cycle scheduling. Large parts of the R5900/VU/GS
behavior, ELF loading, game execution, and corruption support also remain
incomplete.

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
