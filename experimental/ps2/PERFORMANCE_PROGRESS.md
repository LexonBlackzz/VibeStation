# PS2 BIOS performance progress

Updated: 2026-09-25. This is the working log for the ongoing BIOS startup
performance goal. All implementation changes remain under `experimental/ps2`.

## Current result

- The 400 million EE instruction Release trace starts the visible BIOS scene,
  renders 90,089,261 pixels, and retains display hash
  `0xA60E661111C0A023`.
- The latest three-run median is **5.11551 visible fields/s** and **11,927 ms**
  total wall time. The initial three-run cached-interpreter baseline in this
  effort was **4.77825 fields/s** and **12,884 ms**; the user's earlier UI
  observation was 4.44261 fields/s. These rates remain far below real time.
- Full Release build and all six PS2 CTest suites pass. The graphical BIOS
  capture reached EE=260,243,456 with SDL dummy audio in 7,814 ms.

## Completed work

1. **UI execution and pacing** (`c60ac99`): moved guest execution to a
   bounded worker, released the core lock before window swaps, removed the
   500,000 instruction per UI frame cap, and paced visible video without
   accumulating unlimited catch-up time.
2. **Measurement** (`1e3c76e`): added opt-in subsystem/thread timing and a
   repeatable three-run 400M Release benchmark. The profile includes GS worker
   time, queue/flush waits, EE/IOP time, and final GS drain.
3. **EE/IOP execution** (`2f353c2`): when EE instructions are isolated from
   IOP-visible work and EE interrupts are masked, quiet EE execution advances
   to device deadlines before IOP catch-up at the 8:1 clock ratio. This cut
   quiet batches by about 2.6 million in the 400M trace. The opt-in native EE
   backend caches one repeated RAM base register per block; a targeted test
   checks in-block updates and guard exits. The native backend remains off by
   default because sustained BIOS execution is still slower than the cached
   interpreter.
4. **GS rendering** (`195ce45`): profiled draw state by primitive and texture
   format, then selected a common fixed additive blend equation once per draw.
   At 260M instructions, that equation covers about 8.6M pixels. Its targeted
   test and the BIOS display hash pass. A timed 400M trace measured GS worker
   activity at about 5.02 s, versus about 5.20 s before this change.
5. **Startup audio** (`e086a38`): confirmed headless SPU2 output contains
   65,104 stereo frames and 60,006 nonzero samples through 400M instructions.
   The UI now resamples playback to measured guest speed and keeps roughly
   63 ms of host output queued, preventing the 48 kHz device from repeatedly
   outrunning slow emulation. Headless WAV capture retains the original guest
   PCM. The Release UI capture exercised the SDL queue with dummy audio.

## Reverted or disabled experiments

- Quiet EE superbatch trials were slower (about 13.9–14.1 s at 400M) and
  remain disabled.
- Precomputing wrapped FST sprite texture coordinates preserved the display
  hash but did not improve matched 400M runs (12,712 ms versus 12,691 ms), so
  it was reverted.
- The experimental EE JIT remains opt-in. The 400M JIT trace retired about
  187.6M native instructions across 41.8M block calls with 4.45M guard exits;
  its approximately 14.5–14.9 s wall time loses to the cached interpreter.

## Next work

Continue structural EE/IOP and GS performance work, measuring each change in
the same 400M Release trace and checking both display hash and raster pixel
count. The five implementation stages are present, but BIOS animation is
still much too slow for real-time playback.

## Action log

- 2026-09-25: Created this log after completing the five ordered stages;
  recorded baseline, measurements, commits, validation, and reverted trials.
