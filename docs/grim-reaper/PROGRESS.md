# Grim Reaper 2.0 — progress log

Read this first in every new session, then `DESIGN.md`, then
`PROJECT_BRIEF.md` (the code map). The conventions file is the repo-root
`AGENTS.md`. It is gitignored and local to Lexon's machine.

## 1. Status (2026-09-30)

**Phase 1 (deterministic headless runner, telemetry, liveness gates) is done.
Phase 2 has not started.**

Definition of done:

- [x] `PROJECT_BRIEF.md` written
- [x] Headless stock-BIOS run completes faster than real time, writes
      telemetry and reports the speed factor (1.7–3.1× with the
      interpreter; see §3)
- [x] Determinism check passes on the stock BIOS: 6 runs × 1200 frames
      (2 sequential, 2 parallel threads, 2 parallel child processes), with
      bit-identical per-frame telemetry
- [x] Stock BIOS classified `alive`, both at 1500 frames and at 3600 frames
      (60 s)
- [x] Hand-made broken BIOS images are classified dead with the right
      reason: branch-to-self at reset gives `coverage_stall`, a SYSCALL/RFE
      loop gives `exception_loop`. The images are built in test code from
      the real BIOS; no BIOS data is committed. The muted-SPU test now
      expects `alive` with `silent=true`, per Lexon's answer to question 2
      (§5), and `dead_audio` is tested on synthetic telemetry.
- [x] Normal emulation is unchanged. A 2000-frame GT2 replay gives identical
      `BOOT_STATE_HASH` on every field, baseline exe vs new exe, for both the
      interpreter and the recompiler. Telemetry-off cost: nothing measurable
      (see §3).
- [x] Existing tests pass: `--cpu-backend-compare-test`, `--gpu-self-test`,
      `--scheduler-self-test <bios>`.
- [x] This file

Committed on branch `grim-reaper/phase-1`, which was branched from
`perf/event-scheduler` at `86abd98`. Every phase gets its own branch
(`grim-reaper/phase-N`); branch the next one from this one.

## 2. What changed

New files:

| File | Contents |
|---|---|
| `src/core/grim_eval.{h,cpp}` | `GrimTelemetry` (recorder), `GrimFrameTelemetry`, `GrimLivenessConfig` (all thresholds, commented), `GrimLivenessTracker` / `grim_evaluate_liveness()` (gates), `GrimEvalConfig` / `run_grim_eval()` (headless runner), JSONL writers. UI-independent library. |
| `src/platform/grim_eval_runner.{h,cpp}` | CLI modes `--grim-eval`, `--grim-determinism-test`, `--grim-self-test`, plus the gate unit tests and broken-BIOS tests |
| `docs/grim-reaper/PROJECT_BRIEF.md`, `PROGRESS.md` | docs (`DESIGN.md` was supplied by Lexon) |

Small hooks in existing files. All of them only observe; none changes
emulation:

- `cpu.h/.cpp`: `Cpu::set_telemetry()`. `Cpu::run_slice()` has a separate
  interpreter loop, used only when telemetry is attached, that records each
  executed PC. `Cpu::exception()` reports cause+EPC. `step()` itself is
  untouched: a per-instruction branch in `step()` measured +2% and was
  removed.
- `dma.h/.cpp`: cumulative per-channel `debug_completed_transfers()` and
  `debug_moved_words()`. They are not part of save states.
- `gpu.cpp`, `system.h`: always-on per-frame counters
  `ProfilingStats::gpu_fill_commands` and `gpu_gp1_commands`. Fill is also
  still counted in `rect`.
- `spu.h`: `Spu::active_voice_mask()`.
- `bios.h/.cpp`: `Bios::patch32()` writes into the loaded image (the bus
  stays read-only). `restore_original_image()` undoes it, and `reset()`
  calls that, so patches go in after `reset()`.
- `system.h`: `System::bios_mut()` and `System::dma()`.
- `main.cpp`: mode dispatch. `CMakeLists.txt`: the two new sources.

Entry point choice: **CLI modes backed by a library function.**
`run_grim_eval(const GrimEvalConfig&)` is the reusable API for later phases
and the background search. `--grim-eval` is its child-process-friendly
wrapper: a clean command line, results written to a file, and the verdict in
the exit code.

```
VibeStation.exe --grim-eval [bios] <frames> <out.jsonl> [--scenario nodisc]
    [--max-cycles N] [--max-instructions N] [--watchdog-seconds S] [--no-stop-on-death]
  exit 0 = alive, 2 = dead, 1 = error. One stdout line: GRIM_EVAL_RESULT verdict=... reason=...
  silent=0|1 end=frames|death|cycle_cap|instruction_cap|watchdog speed=...x run_hash=...
VibeStation.exe --grim-determinism-test [bios] [frames=600] [threads=2] [processes=2]
VibeStation.exe --grim-self-test [bios] [frames=1500]
```

`[bios]` may be left out when the environment variable `VIBESTATION_BIOS`
holds the path. A leading numeric argument is taken as `frames`.

All three force the interpreter and drop the log level from Info to Error, so
broken machines don't flood the log.

Runner behaviour: it creates a fresh `System`, calls `load_bios` and
`reset()`, applies ROM patches, attaches telemetry and enables SPU capture.
The capture buffer is the null sink: samples never reach SDL/WASAPI, and
headless runs never initialise SDL audio. It then runs
`run_frame(true, false)` in a loop with no host pacing. After each frame it
records telemetry, feeds the liveness tracker, and stops at the first of:
the frame count, death (by default), the cycle cap, the instruction cap, or
the wall-clock watchdog. `end_reason` says which one. The SPU was already
driven by emulated cycles, so audio did not need any decoupling.

Telemetry JSONL: one object per frame with a fixed field order, then a
`{"summary":{…}}` line. Per-frame lines contain no host time, so they diff
cleanly between runs. Only the summary has `wall_s` and `speed`. Fields:
`cycles`, `instr`, `new_pcs` (instruction words executed for the first time;
KSEG mirrors and RAM mirrors map to one physical word), `coverage`, `exc[16]`
(by cause), `exc_repeat` (non-IRQ exceptions with the same cause+EPC as the
previous one), `gp0{poly,line,rect,fill,xfer,other}`, `gp1`, `dma_n[7]`,
`dma_w[7]`, `key_on`, `voices` (active voice mask), `fb` (hash of the
displayed area), `display`, `lit` (non-black displayed pixels), `audio_n`, `audio_hash`, `rms`, `zcr`, `cpu_hash`
(GPRs, PC, HI/LO, SR, Cause, EPC, BadVAddr). `run_hash` in the summary is an
FNV hash over all the per-frame lines.

Liveness gates (`GrimLivenessConfig`, defaults in frames at ~60 fps):

| Reason | Trips when |
|---|---|
| `exception_loop` | 60 consecutive frames with ≥ 8 non-IRQ exceptions each, ≥ 90% repeating the previous cause+EPC, and no new code |
| `coverage_stall` | 300 frames with no new code, no display change, no audio movement and no GP0/DMA/key-on work |
| `frozen_frame` | 600 frames with an unchanged display, no audio movement and no GP0 commands, even while the CPU runs (a runaway CPU executing data) |
| `dead_audio` | over the whole run (≥ 300 frames): the run is *silent* **and** never showed an image (no frame with the display on and ≥ 64 lit pixels). Silent means fewer than 10 "moving" audio frames. A frame moves when RMS ≥ 16, zero-crossing rate > 0 (not DC), and RMS or ZCR changes by > 5% vs the previous frame (not one fixed tone). |

**Silence alone is not death.** A silent machine that draws an image is
`alive` with `silent=true` in the summary and in the `GRIM_EVAL_RESULT`
line. Such "duds" stay in the pool, and a later phase can filter or score
them by that flag.

Priority when several trip in the same frame: exception_loop, then
coverage_stall, then frozen_frame. `dead_audio` is decided at the end of the
run. Stock-BIOS margins over 60 s: longest stall run 40/300, longest frozen
run 81/600, 466 moving audio frames (10 needed).

## 3. Test checklist for Lexon

Run from the repo root after `build-ninja.cmd`, with
`set VIBESTATION_BIOS=D:\Misc\pSXfin_1_13-1220\bios\scph1001_original.bin`
(or pass the path as the first argument).

1. **Self-test**: `build-ninja\VibeStation.exe --grim-self-test`
   - Expect 18 `GRIM_TEST PASS` lines: 12 synthetic tests, `bios_stock_alive`,
     `bios_infinite_loop` (coverage_stall at frame 300), `bios_exception_loop`
     (exception_loop at 60), `bios_muted_audio_still_alive` (alive, silent)
     and `bios_stock_not_silent`. Also expect
     one `GRIM_TEST INFO bios_stock_speed ... speed=…x` line and a final
     `GRIM_SELF_TEST PASS failures=0`, exit code 0.
   - It takes about 20–40 s.
2. **Headless eval**: `build-ninja\VibeStation.exe --grim-eval 1800 stock.jsonl`
   - Expect `GRIM_EVAL_RESULT verdict=alive reason=alive death_frame=-1
     silent=0 end=frames frames=1800 emulated_s=30.03 … speed=…
     run_hash=0x432E585CF1535F9C`. `stock.jsonl` should have 1801 lines,
     and exit code is 0.
   - Speed was 1.5–3.1× here. The machine flips between a fast and a slow
     state, and 1800 frames took 10–20 s.
3. **Determinism**: `build-ninja\VibeStation.exe --grim-determinism-test 1200 2 2`
   - Expect `GRIM_DETERMINISM_PASS runs=6 frames=1200 run_hash=0x67E891B108DE7168
     verdict=alive reason=alive`, taking about 50 s. The hash should match on
     your machine too, as long as the BIOS file is identical.
   - A failure prints `GRIM_DETERMINISM_FAIL run=<label> frame=<n> field=<json key>`.
4. **Existing tests**: `--cpu-backend-compare-test`, `--gpu-self-test` and
   `--scheduler-self-test <bios>` should all exit 0, as before.
5. **GUI and the existing Grim Reaper** (I could not test this):
   - Boot the BIOS and a game in both CPU modes.
   - Use BIOS "reap and reboot" and turn on the RAM/GPU/Sound reapers.
   - Everything should behave exactly as before, with the same performance.

Measured telemetry-off cost (lean `--cpu-benchmark`, alternating
baseline/new exe, same session):

| Load | Baseline | New | Result |
|---|---|---|---|
| Interpreter, BIOS menu 600+600 | min 3162 ms | min 3119 ms | no change (noise ±3%) |
| Interpreter, GT2 1500+600 | median 7971 ms | median 7817 ms | no change |
| Recompiler, GT2 1500+600 | median 2414 ms | median 2390 ms | no change |

An earlier version with the hook inside `Cpu::step()` measured +2.0% on the
interpreter (8/8 pairs). That was fixed by moving the hook into the
telemetry-only loop in `run_slice`.

## 4. Known issues and untested areas

- **The watchdog is checked between frames only.** A host hang inside
  `run_frame` (an emulator bug) can only be stopped by killing the process,
  so the background search must run `--grim-eval` as a child process with a
  timeout. `--grim-determinism-test` waits on its children via
  `std::system` with no timeout. True crash and hang isolation (Job Objects /
  kill on timeout, saving the genome of every host crash) is future work.
- **Threads share process globals.** These are diagnostic statics,
  `g_diag_current_pc`, the perimeter-trace arrays that `Cpu::reset()` writes,
  log counters and the global `g_*` config. They never feed back into
  emulated state, and parallel threads were bit-identical, but they are
  formally data races. **Use processes for the search.**
- Telemetry only works under the interpreter. Dynarec evaluation is future
  work, and it would need native-code hooks (see `recompiler-native-only-goal.md`).
  `System::step()` and `run_frame`'s one-instruction fallback `cpu_.step()`
  bypass the coverage hook, but they don't occur in normal runs.
- "New basic blocks" is measured as newly executed **instruction words**
  (a bitmap over RAM, BIOS and scratchpad; other space shares one slot).
- In interlaced display modes, `fb` hashes the presented field, and the
  result depends on `g_deinterlace_mode`. A dead machine left in an
  interlaced mode could toggle between two hashes and never trip
  `coverage_stall`/`frozen_frame`. This is untested, because the BIOS runs
  non-interlaced.
- **frozen_frame limitation:** a machine stuck *redrawing* one picture (a
  frozen logo in a draw loop) is classified alive. It cannot be told apart
  from the idle Shell without input or a comparison against a clean-run
  reference. Lexon accepted this: duds are fine, and he tests by hand.
- Only tested with SCPH-1001 (NTSC U/C). PAL BIOSes are untested; the frame
  thresholds are in frames, so they last 20% longer in seconds on PAL.
- Audio metrics are basic: RMS, zero crossings of the L+R mix, and the FNV
  hash. Spectral measures come in Phase 5.
- Runaway-CPU detection (execution inside data regions) needs the Phase 3
  region map and is not implemented.
- The existing runtime reapers seed from `std::random_device` unless a custom
  seed is set, so they are not reproducible by default. They are untouched
  and unused by evaluation.

## 5. Answered questions (2026-09-30)

1. Branch and commit: every phase gets its own branch, and committing is
   allowed.
2. A silent machine that draws an image is not dead ("there can still be
   duds"). Implemented as `silent=true` on an `alive` verdict.
3. The frozen-picture redraw case being alive is fine; Lexon tests it by
   hand. No scripted input for now.
4. Interpreter speed (2–3× real time per process) is enough for now.
   Performance comes later.
5. `VIBESTATION_BIOS` was added as a fallback for the BIOS path.

No open questions.

## 6. Next step: proposed Phase 2 brief (interface genes)

Goal: SPU-register and GP0 interface genes with temporal triggers,
evaluated by the Phase 1 runner and reproducible from a genome file.

- **Genome type** (`src/core/grim_genome.{h,cpp}`): a list of typed genes
  `{type, target, params, trigger}`, serialised as text or JSON. Use a
  seeded splitmix64 inside each gene. **Do not use `std::random_device` or
  `std::uniform_*_distribution`**: their output differs between MSVC and
  libstdc++, so genomes would not reproduce across builds and CI. The
  trigger is `{start_frame, end_frame, ramp (rot), intermittent pattern}`,
  evaluated against emulated time (`boot_diag().frame_counter` or
  `cpu().cycle_count()`), never host time.
- **SPU register filter**: one choke point in `System::write16`/`write32`
  for io `0xC00`–`0xFFF`, just before `spu_.write16(offset, value)`. Map
  the offset to a voice and register (pitch `+4`, ADSR `+8/+A`, volume
  `+0/+2`, start `+6`, repeat `+E`, KON/KOFF `0x188`–`0x18E`, noise
  `0x194`, PMON `0x190`, reverb `0x1C0`+, work-area base `0x1A2`). Filters
  transform the value, drop the write, or duplicate it. Delayed key-ons need
  a small queue flushed at emulated cycles. Keep the existing
  `Spu::corrupt_runtime_state` sound reaper working untouched.
- **GP0 filter**: `Gpu::gp0()` already has a whole-command hook,
  `apply_reaper_to_gp0_command()`, run after the words are buffered in
  `gp0_buffer_` and before dispatch. Add the genome filter next to it, and
  only apply transforms that keep the word count (vertex jitter/swap,
  colour ops, semi-transparency/raw-texture bits, texpage/CLUT fields,
  draw-offset/area drift through `E3`–`E5`). Also cover
  `handle_polyline_word` for polylines. VRAM-transfer data words
  (`consume_vram_write_word`) are out of scope.
- **Plumbing**: add `GrimEvalConfig::genome`, `--grim-eval --genome <file>`
  and a genome-hash field in the summary. Pass the genome to `System`
  through one pointer (`nullptr` = off, like `Cpu::set_telemetry`) so
  normal play costs one branch per register write or GP0 command. Measure
  the cost the same way as in §3.
- **Tests**: a determinism test with a non-trivial genome (same genome gives
  identical telemetry across processes). Unit tests of each filter on
  synthetic register writes and GP0 packets. One end-to-end check that a
  pitch gene changes `audio_hash` but not `fb` during the intro, and a GP0
  colour gene changes `fb` but not `audio_hash`.
- **Out of scope for Phase 2**: fingerprints and archive (Phase 5), ROM
  genes (Phase 3+), UI.
