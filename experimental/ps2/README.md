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

The standalone executable is `VibeStationPS2Lab`.

## Current scope

The initial scaffold contains:

- a 32 MiB EE RAM implementation;
- a minimal EE bus with direct, KSEG0, and KSEG1 RAM aliases;
- a deterministic event scheduler;
- an EE CPU state container;
- a `Ps2System` composition root;
- headless smoke tests.

There is no R5900 instruction execution, BIOS boot, GS rendering, IOP, SPU2, or corruption support yet.

## Isolation rule

Until the PS2 core reaches a stable execution milestone:

- do not add PS2 conditionals to PS1 source files;
- do not reuse `src/core/gpu.*`, `src/core/system.*`, or the PS1 CPU/SPU directly;
- do not require the repository root `CMakeLists.txt` to build this project.

Shared abstractions should be extracted only after both implementations make the common boundary obvious.
