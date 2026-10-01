# VibeStation — project brief for Grim Reaper 2.0 sessions

A factual map of the code as of 2026-09-30 (branch `perf/event-scheduler`).
Prefer the symbol names over line numbers, since line numbers drift. The
conventions file is the repo-root `AGENTS.md`. It is gitignored and local to
Lexon's machine, so read it if you can.

## Build and test

- Build (Windows, owner's machine): `build-ninja.cmd` from the repo root.
  It uses MSVC and Ninja, builds Release with the x64 JIT on and IPO off, and
  writes `build-ninja\VibeStation.exe`. It is a single CMake target: sources
  are listed explicitly in `CMakeLists.txt` (`VIBESTATION_SOURCES`, no glob),
  so **new .cpp files must be added there**. Every build rewrites
  `resources/build_number.txt`. Never commit that file.
- CI (`.github/workflows/`): Linux GCC with LTO runs `--cpu-backend-compare-test`
  and `--gpu-self-test`. Windows MSVC builds only.
- Tests are CLI modes of the main exe, with no test framework. Exit code 0
  means pass. Existing modes: `--cpu-backend-compare-test`, `--gpu-self-test`,
  `--scheduler-self-test [bios]`, `--boot-disc-test`, `--frame-test`,
  `--cpu-benchmark`, `--spu-audio-test`, `--bios-test`. The mode dispatch
  lives in `main()` in `src/main.cpp`: options are parsed first and the rest
  goes into `passthrough`. Tests take the BIOS as a path argument, for example
  `D:\Misc\pSXfin_1_13-1220\bios\scph1001_original.bin` on Lexon's machine.
- Grim Reaper 2.0 adds `--grim-eval`, `--grim-determinism-test`,
  `--grim-self-test` (Phase 1) and `--grim-gene-test`, `--grim-random-genome`,
  `--grim-explore` (Phase 2). See `PROGRESS.md`.

## Main loop and timing

- `System::run_frame()` (`src/core/system.cpp`) runs one video frame. The
  frame length comes from `target_fps()` (PAL/NTSC, interlace) and
  `CPU_CLOCK_HZ`. Each scanline is an absolute cycle edge. Inside a scanline
  the CPU runs up to the earliest of the scanline end and
  `next_device_deadline()`, which covers timers, CD, MDEC, SIO, SPU and DMA.
  Devices sync lazily, either on MMIO through `System::IoScope` or at their
  own deadline through `service_device_events()`. VBlank (`gpu_.vblank()` and
  `irq_.request(VBlank)`) fires at the start of scanline 240 (NTSC) or
  288 (PAL). At the end of a frame, `update_display_diag()` stores a display
  hash in `boot_diag_` when `sample_display_diag` is true.
- **Nothing in the core reads the host clock for emulation.** The
  `std::chrono` calls in `run_frame`, `Gpu::gp0`, `Spu::tick` and
  `DmaController::tick` only fill `ProfilingStats` (ms fields).
- GUI pacing: the `EmuRunner` worker thread (`src/ui/emu_runner.cpp`,
  `worker_main`) calls `run_frame` and then sleeps on `steady_clock` until
  `1/target_fps/speed` has passed. Audio does **not** drive pacing. The SDL
  audio callback only drains a ring buffer.
- Headless modes (`--frame-test`, `--boot-disc-test`) build a `System` with
  `make_unique<System>()`, call `load_bios()` and `reset()`, then loop over
  `run_frame()`. They never call `SDL_Init`.

## CPU (`src/core/cpu.{h,cpp}`)

- Interpreter: `Cpu::step()` handles one instruction. It samples the IRQ
  line (`sys_->irq_pending()` → Cause.IP2, then `check_irq()`), fetches with
  `fetch32()`, runs `execute()`, then `advance_load_delay()`, then commits
  `instruction_cycles() + cycle_penalty_`. `Cpu::run_slice()` loops over
  `step()` and returns early at SIO timing boundaries. When a
  `GrimTelemetry` is attached, it uses a separate copy of that loop which
  records executed PCs. Keep per-instruction hooks out of `step()`: one
  branch there cost +2%.
- Recompiler: when the mode is Recompiler, `Cpu::run_slice()` forwards to
  `CpuRecompilerBackend::run_slice` (`src/core/cpu_recompiler.cpp`, Xbyak
  x64). The global mode comes from `effective_cpu_execution_mode()` in
  `types.h`. A CLI `--cpu` override sets `g_cpu_execution_mode_cli_*`. The
  default is the Interpreter, and headless modes never load the ini file.
- Exceptions: `Cpu::exception(Exception cause)` sets SR, Cause and EPC
  (plus BD and TAR) and jumps to `0xBFC00180` when BEV is set, otherwise to
  `0x80000080`. Causes are listed in `enum class Exception` (`cpu.h`).
  AdEL/AdES/ReservedInst paths log heavily at Warn level (AdES logs on every
  occurrence), so evaluation runs set the log level to Error.
  Interrupt entry is `exception(Exception::Interrupt)` from `step()`.
- Fetch goes through `fetch32()`: the I-cache when `instruction_cacheable()`
  is true (KUSEG/KSEG0), otherwise `System::read32_instruction()`, which
  covers the RAM, BIOS and scratchpad fast paths. The BIOS runs uncached from
  `0xBFC0xxxx`. Loads and stores go through `load8/16/32` and `store8/16/32`
  into `System::read*/write*`. Stores with SR.IsC (bit 16) invalidate I-cache
  lines instead of writing memory.
- **The dynarec bypasses these paths.** Native blocks access RAM and the
  scratchpad directly (`jit_main_ram_data_mut`, `jit_scratchpad_data_mut`),
  and do not call `step()` or `exception()` for ordinary opcodes. Only MMIO
  goes through `v4_bus_read*/write*`, bracketed by
  `jit_begin_bus_access`/`jit_end_bus_access`. Per-instruction hooks
  (coverage) therefore see nothing under the recompiler.

## Memory map (`System::read*/write*`, `psx::mask_address` in `types.h`)

- `mask_address`: KUSEG and KSEG0 (`0x80000000`) and KSEG1 (`0xA0000000`) all
  map to the same physical address (`& 0x1FFFFFFF`). KSEG2 passes through
  unmasked (cache control `0xFFFE0130`).
- RAM is 2 MB (`Ram`, `src/core/ram.cpp`). Physical addresses below the
  window set by the RAM_SIZE register (`0x1F801060`, usually 8 MB) mirror
  the 2 MB with `& (2MB-1)` (`map_main_ram_address`).
- Scratchpad: `0x1F800000`–`0x1F800FFF`, 1 KB mirrored, stored in `Ram`
  (`scratch_read*`).
- I/O: `0x1F801000`–`0x1F802FFF`. BIOS ROM: `0x1FC00000` + `Bios::mapped_size()`.
- BIOS load: `System::load_bios(path)` calls `Bios::load` (`src/core/bios.cpp`),
  which reads the file into `data_` and keeps `original_data_`.
  `System::reset()` calls `bios_.restore_original_image()` first, so in-memory
  ROM patches must be applied *after* `reset()`, through `Bios::write32`
  (`System::bios_mut()`).

## GPU (`src/core/gpu.{h,cpp}`)

- GP0 writes arrive at `System::write32` (io `0x810`) and go to
  `System::gpu_gp0()`. DMA2 goes through `System::gpu_gp0_dma()`. Both call
  `Gpu::gp0()`, which buffers words by `gp0_command_length()` and dispatches
  whole commands in a `switch`. The existing Grim Reaper hook
  `apply_reaper_to_gp0_command()` runs just before dispatch. GP1 writes go
  from io `0x814` to `Gpu::gp1()`. 8/16-bit writes are assembled through
  `gpu_gp0_shadow_`/`gpu_gp1_shadow_`.
- Command classes are always counted, per frame, in
  `System::ProfilingStats` (`gpu_flat/gouraud/textured/gouraud_textured/rect/line/transfer/other_commands`),
  with the bucket chosen by `gpu_profile_bucket_for_opcode()`. Fill (`0x02`)
  counts as `rect`. The stats are reset at the start of `run_frame`.
- VRAM: `Gpu::vram_` is a `std::array<u16, 1024*512>`, read through
  `Gpu::vram()`. The display area comes from `display_` (`DisplayMode`
  x/y start, hres/vres, 24-bit, enabled) and `calculate_crtc_rect()`.
  `Gpu::build_display_rgba(nullptr)` returns `DisplaySampleInfo.hash`, an
  FNV-1a hash of the displayed pixels (the FNV offset value when the display
  is disabled).

## SPU (`src/core/spu.{h,cpp}`)

- Register writes: `System::write16/32` for io `0xC00`–`0xFFF` calls
  `sync_spu_to_cpu()` and then `Spu::write16`. Key-on is counted in
  `AudioDiag::key_on_events`. Voices are `voices_[24]` (`VoiceState::phase`).
- Samples: `Spu::tick(cycles)` produces one stereo frame every 768 CPU
  cycles using integer accumulation, so it is additive. The SPU is driven by
  **emulated cycles** (`System::sync_spu`), not by the audio callback.
  `tick` fills `mix_buffer_` and calls `enqueue_ring_buffer()`. If
  `capture_enabled_` is set, the samples are appended to `capture_samples_`
  (capped at 180 s) and nothing is sent to the host. Otherwise, when a device
  is open, they go to `AudioRingBuffer`, and SDL's callback
  (`sdl_audio_callback`) pulls them. On Windows SDL uses WASAPI.
- `Spu::init()` only opens an SDL audio device if SDL audio was initialised,
  so headless runs have no device.
- `run_frame(…, skip_spu_for_turbo=true)` skips SPU sync (turbo), so
  evaluation must leave it false.

## DMA (`src/core/dma.{h,cpp}`)

- `DmaController::tick()` (called from `System::service_dma`) calls
  `execute_dma(ch)` for active, enabled and requesting channels.
  Immediate/Block modes go to `dma_block(ch, max_words)`, where the per-word
  switch covers 0 MDEC in, 1 MDEC out, 2 GPU, 3 CD, 4 SPU and 6 OTC.
  Linked-list mode (GPU only) goes to `dma_linked_list()`.
  `transfer_complete(ch)` clears CHCR and raises DICR.

## Existing Grim Reaper (must keep working)

- BIOS corruption: `App::reap_and_reboot_bios()` and `..._batch()`
  (`src/ui/panels/grim_reaper_actions.cpp`) randomize bytes in a chosen
  range, write `<bios>_grim_<range>.bin` next to the original, and
  `load_bios()` it. The UI lives in `grim_reaper_panel.cpp` and
  `src/ui/definitive/definitive_grim_reaper.cpp`.
- Runtime reapers: RAM, GPU and Sound. The UI syncs config
  (`grim_reaper_runtime.cpp`) into `System::set_*_reaper_config`, and
  `System::apply_{ram,gpu,sound}_reaper_for_frame()` runs at the top of every
  `run_frame`. They mutate RAM/VRAM/SPU RAM through
  `Gpu::set_reaper_pulse`, which feeds `apply_reaper_to_gp0_command`, and
  through `Spu::corrupt_runtime_state`.

## Nondeterminism sources spotted

- Runtime reapers seed from `std::random_device` unless `use_custom_seed`
  is set. BIOS reaping does the same. They are off by default.
- Process-global mutable state that emulation code writes: `g_diag_current_pc`
  and the `static prev_pc_for_diag` in `Cpu::step()`, the perimeter-trace
  arrays in `cpu.cpp` (cleared in `Cpu::reset()`), and many function-local
  `static` log counters. None of them feed back into emulated state, but they
  are data races when several `System`s run on separate threads.
- Config is global (`g_*` in `types.h`: CPU mode, deinterlace mode, fast
  modes, log level). Deinterlace mode changes the display hash in interlaced
  modes.
- `std::unordered_map` appears only in CUE parsing (`cdrom.cpp`) and the JIT.
- Host time is used only by profiling timers, log timestamps
  (`g_log_timestamp`) and `input_recorder` file names.
- Memory cards are loaded only from explicit paths (`--boot-card0/1`), so
  no-disc headless runs touch no host files.
- In the GUI, pad input arrives on the UI thread's timing and turbo/speed
  are host-driven. Headless runs have neither.

## Surprises

- `Cpu::exception()` calls `dump_perimeter_trace("perimeter_crash_dump.txt")`
  on AdEL/AdES, but it only writes when the diagnostic trace captured
  something (`cpu_diag_enabled()`).
- `System` is large (it holds a heap vector of RAM-word write provenance and
  diagnostic histories), and every `Cpu::init` allocates a
  `CpuRecompilerBackend` even in interpreter mode.
- `System` has always-on per-frame work counters (`ProfilingStats`) and a
  large `BootDiagnostics` struct. Reuse them before adding new hooks.
- Several source files have CRLF or mixed line endings. Check with
  `git diff --numstat --ignore-cr-at-eol`.

## Interface genes (Phase 2, `src/core/grim_genome.{h,cpp}`)

- `GrimGenome` is a versioned list of `GrimGene {type, target, seed, trigger,
  params}`. Parsing (nlohmann/json, already a dependency) is strict; the
  serializer is hand-written so the field order is fixed. `GrimGenomeRuntime`
  holds the per-gene splitmix64 streams, the SPU delay queue and a shadow of the
  last value written to each SPU register. Everything is integer math.
- `System` holds `GrimGenomeRuntime *grim_` and passes the same pointer to `Gpu`
  (`System::set_grim_genome`). `nullptr` = off = one branch per SPU register
  write (two sites in `System::write16/write32`, which split 32-bit writes
  first) and per buffered GP0 command (`Gpu::gp0`, before
  `apply_reaper_to_gp0_command`) or polyline word (`Gpu::handle_polyline_word`).
- SPU: `System::grim_write_spu16` -> `GrimGenomeRuntime::filter_spu_write`
  (0..8 output writes per input). Delayed writes wait in the runtime's queue;
  `System::sync_spu` ticks the SPU to each due cycle and writes them there (the
  SPU is additive, so this does not depend on where sync points fall).
  `Spu::read16` returns what the SPU was given, i.e. the transformed value.
- Time: `boot_diag_.frame_counter` is emulated (only `run_frame` increments it,
  `System::reset` zeroes it). `run_frame` passes it to `begin_frame`.
- The recompiler reaches all of this through `v4_bus_write16/32` ->
  `System::write16/32`, and GP0 through the same `Gpu::gp0`.
- Runner side: `GrimEvalConfig::{use_genome, genome, dump_wav_path,
  dump_frames_dir, dump_frames_every}`; `src/platform/grim_process.{h,cpp}`
  (CreateProcess + Job Object / fork + process group with hard timeout);
  `grim_explore` and `grim_random_genome` CLIs in `grim_eval_runner.cpp`;
  `src/platform/grim_gene_test.cpp` (`--grim-gene-test`).
- GUI: `--genome <file>` before any other mode word; `main()` loads it via
  `grim_gui_genome_load`, `App::init_runtime` calls
  `system_->set_grim_genome(grim_gui_genome())`. `System::reset()` rewinds the
  genome, so "reap and reboot" replays it from frame 0.

## Boot map and ROM genes (Phase 3, `src/core/grim_map.*`, `grim_rom.*`)

- `GrimBootMapper` (discovery only) is fed from the telemetry loop in
  `Cpu::run_slice` (`begin_instruction` before `step()`, `commit_instruction`
  after, skipped when the step entered an interrupt) and from
  `DmaController::dma_block` / `dma_linked_list` (`note_dma`, per block/packet).
  Attach with `System::set_grim_boot_mapper()`; it also needs telemetry attached.
  `GrimEvalConfig::boot_map_out` does this inside `run_grim_eval`.
  Nothing is added to `Cpu::step()`, the dynarec or normal play.
- It keeps a byte tag (ROM offset + 1) for every RAM byte, four byte tags per
  register, and per ROM word the exec/read/use flags, consumer and first-touch
  cycles. Rules (IsC stores, moves, load delay, DMA) are in PROGRESS.md 2.2.
  `GrimBootMap` = result; `grim_map_save/load` write `<path>` + `<path>.words`.
- `GrimRomContext` (stock image words + map) feeds `grim_rom_generate`; mutation
  kinds are `GrimRomMut`. Genes of type `rom_code` (genome v2) carry resolved
  patches. `GrimGenomeRuntime::apply_rom(Bios&)` verifies the BIOS hash and
  original words and patches through `Bios::patch32`; `System::reset()`,
  `boot_disc()` (after the fast-boot patch) and `set_grim_genome()` call it via
  `grim_apply_rom_genes()`; `System::grim_rom_error()` holds a failure message.
- `Bios::image_hash()` is FNV-1a-64 of the whole (512 KiB padded) stock image;
  `Bios::original_word()` reads the stock image.
- CLI: `--grim-map`, `--grim-map-summary`, `--grim-describe-genome`,
  `--grim-map-test` (`src/platform/grim_map_runner.*`, `grim_map_test.cpp`);
  `--map/--mix/...` options on `--grim-random-genome` and `--grim-explore`.

## ADPCM samples (Phase 4, `src/core/grim_sample.*`)

- `grim_scan_adpcm()` takes stock ROM words only. It scans both eight-byte address
  lanes at a 16-byte block stride, requires eight consecutive plausible blocks,
  nonzero payloads and a loop-end terminator. The raw candidate list is scored by
  `grim_sample_score()` against map SPU-consumer words; map data is never a scanner
  input. `grim_sample_annotate()` then refines boundaries from resolved voice starts.
- `GrimBootMapper` shadows SPU RAM byte tags and transfer addressing during discovery
  only. Its existing instruction/DMA entry points record start/repeat register writes
  and key-on requests in `GrimBootMap::spu_sample_uses`. There are no new normal-play
  hooks. Optional JSON events keep older maps loadable; map hashes with events differ.
- `GrimSampleContext` holds stock words, scanner candidates and annotated samples.
  `grim_sample_generate()` emits `spu_sample` genes with resolved word patches and
  the target block lane (`block_phase`, 0 or 8). Ten kinds cover filter/shift,
  loop-start/end edits, block payload shuffle/repeat/reverse/transplant, and nibble
  noise. Payload operators retain both header bytes; transplants retain target length.
  Shift genes tag values 13–15 with `emulator_shift=1`.
- Genome version stays 2. `GrimGenomeRuntime::apply_rom()` verifies the BIOS hash and
  every original word across both ROM families before writing anything. The existing
  GUI `--genome` path applies sample genes on boot/reset in either CPU mode.
- `--grim-samples [bios] out.json [--map file.json]` reports ROM ranges, blocks, loop
  points, voices and cycles. `--grim-sample-test [bios]` checks scanner fixtures,
  gene invariants, small-edit audio liveness and per-kind clean-audio differences.
  `--out-dir dir` preserves test genomes/WAVs and `audio_survival.csv` outside the repo.
- Generation/explore support `--mix samples|all`, `--sample-kind`, `--sample-count`,
  `--sample-index`, `--sample-genes`, `--sample-magnitude`. `both` retains Phase 3's
  interface+code meaning. Explore compares child audio hashes with a clean baseline
  and writes `sample_survival.csv`; use one sample gene for unambiguous attribution.
- The optional dormant view is derived from unused words with read flags. It adds
  counts/ranges to map summaries without changing the existing serialized classes.

## Audible sample windows (Phase 4.1)

- Mapper `PitchWrite` observes voice +4 writes and records pitch on each key-on.
  `grim_sample_annotate()` computes the median positive key-on duration per block,
  including the exact reduced rational milliseconds. Old maps/unplayed samples use
  base pitch. `--grim-map --shell-input` is an explicit discovery-only controller
  script; `GrimEvalConfig::scripted_buttons` does not affect normal play.
- `GrimGene::sample_sizing` is optional v2 generation metadata. Serialized
  `sizing` selects milliseconds or fraction_permille; absent means old count/RNG
  semantics. `grim_sample_window_blocks()` rounds up to aligned contiguous blocks.
  Random sample generation defaults to 100 ms; --sample-count selects legacy sizing,
  --sample-ms/--sample-fraction select new sizing, --sample-donor pins transplants.
- `src/core/grim_audibility.*`: `grim_compare_audio()` compares matched stereo PCM
  windows against a clean reference. Integer energies/counts decide a provisional
  audible verdict; `grim_audibility_json()` reports residual dB/exposure, stereo
  peaks, RMS and clipping. `grim_read_wav()` validates PCM review inputs.
- `GrimEvalConfig::audio_out` captures evaluation-only PCM without changing frame
  telemetry or run hashes. `src/platform/grim_audibility_runner.*` provides
  --grim-audibility and --grim-audio-compare. Existing --grim-explore now writes
  audibility.json and distinguishes survived+audible from survived+inaudible.
- `--grim-audibility-test` exercises labelled fixtures, repeat/backend equality,
  short/quiet/stereo/clipping controls and WAV transport. Human verdicts live in
  docs/grim-reaper/audio_labels.json; null means pending listening and is skipped.
  New resolved genomes/WAVs remain outside Git. PROGRESS §1–9 records results;
  Appendix D retains Phase 4's older hash-change measurements.
