# Grim Reaper 2.0 — progress log

Read this first in every new session, then `DESIGN.md`, then
`PROJECT_BRIEF.md` (the code map). The conventions file is the repo-root
`AGENTS.md`. It is gitignored and local to Lexon's machine.

## 1. Phase 4 status (2026-09-30)

**Phase 4 (ADPCM scanner, sample provenance and ROM sample genes) is done
on `grim-reaper/phase-4`, branched from Phase 3 `cf006dc`.** Phase 3's full record
is retained in Appendix C below. Phases 1–3 retain their behaviour. Every phase
gets its own branch; start Phase 5 from this one.

- [x] ROM-only scanner developed and rules frozen before opening map files
- [x] Scanner scored against SPU-consumer words; honest raw precision/recall (§3)
- [x] Discovery-only SPU-RAM provenance; voice start/repeat/key-on records (§3.2)
- [x] `spu_sample` genes, all ten kinds, genome v2, hash/original verification
- [x] Block alignment and header invariants; existing gene loops cover all types
- [x] Audio **survival and change** table for every kind, plus unplayed controls
- [x] Stock hash unchanged; full 5738-frame GT2 parity in both backends (§6)
- [x] All suites pass with MSVC, plus new CLI and exploration checks (§5)
- [x] Alternating lean benchmarks: no repeatable overhead (§6)
- [x] Four verified listening genomes, checklist (§7), project brief and Phase 5 brief

Nothing was added to `Cpu::step()`, the recompiler, `System`'s device paths,
`Spu`, the scheduler or DMA. SPU provenance lives inside the mapper's existing
telemetry/DMA callbacks. Builds did not encounter a locked executable; nothing
was renamed or killed. Tests, BIOS-derived maps, telemetry, WAVs and baseline
executables live outside the tracked tree in `D:\Projects\github\vs-runs\phase4`
or the gitignored maps directory. Pre-existing root artifacts were left alone.

## 2. What changed in Phase 4

| File | Contents |
|---|---|
| `src/core/grim_sample.{h,cpp}` | Byte-only scanner, score, provenance boundary annotation, sample context, ten mutations, deterministic generation, summary/description |
| `src/platform/grim_sample_runner.{h,cpp}` | `--grim-samples`, JSON ranges/blocks/loop points/voice uses and raw score |
| `src/platform/grim_sample_test.cpp` | `--grim-sample-test`: scanner fixtures, 256 mutations per kind on both lanes, audio survival/change and backend parity |
| `src/core/grim_map.{h,cpp}` | SPU RAM tag shadow, register/FIFO/DMA addressing, optional sample-use JSON records, dormant view |
| `src/core/grim_genome.{h,cpp}` | `SpuSample`, strict schema/patch checks, common ROM application and random generation |
| `src/platform/grim_eval_runner.cpp` | Sample generation options, explore clean-audio comparison, sample survival CSV |
| `src/platform/grim_gene_test.cpp`, `grim_map_test.cpp` | All-type loops, both ROM families' atomic failure tests, synthetic/unit SPU provenance checks |

Small edits: `grim_rom.{h,cpp}` and `grim_map_runner.cpp` for sample descriptions;
`main.cpp` for modes; `CMakeLists.txt` for sources; four sample genome files and
this record/`PROJECT_BRIEF.md`. No geometry/texture/font genes or archive code.

## 3. Scanner and sound bank

### 3.1 Frozen scanner rules and score

Rules were written to external `phase4/scanner-design.txt` before map inspection.
`grim_scan_adpcm()` takes **ROM words only**, with no map parameter:

- Scan both eight-byte address lanes (0 and 8), with a 16-byte stride; every
  accepted run keeps one lane. Header bit 7 clear, filter <=4, shift <=12,
  flag byte <=7. Invalid blocks and loop-end bit 0 delimit runs.
- Require at least **8 blocks** (224 decoded samples) and two blocks with
  nonzero payload, closed by a loop-end. Incomplete runs are rejected.
- Trim a consecutive all-zero prefix to at most one predictor-reset block.
  A silent block with a nonzero header remains part of the run.
- For overlapping candidates from the two lanes, prefer the longer run, then
  the earlier offset. This resolves alignment ambiguity without consulting use.

Stock SCPH-1001 (`bios_hash=0x32B1A0FA4DB70C8F`): **8 candidates / 44,976 bytes**.
Word-union score against the map's 10,800 SPU-consumer words:

| True positive | False positive | False negative | Precision | Recall |
|---:|---:|---:|---:|---:|
| 10,776 | 468 | 24 | **95.8378%** | **99.7778%** |

The test thresholds are precision >=95% and recall >=99%. Eight blocks suppress
isolated random header matches; the score still admits small structured-data
decoys, while recall requires practically the whole bank. Thresholds are a
regression guard for this BIOS, not a claim about other BIOS revisions. The map
also counts SPU register/table values, so its words are a practical answer key
for consumed ROM data rather than a perfect ADPCM-only segmentation.

Raw ranges, unchanged after scoring: `0x116F0–11770`, `0x30860–308F0`,
`0x3D320–3D410`, `0x4C9C0–4CAB0`, `0x4CBF0–4CC90`, `0x51E20–55680`,
`0x55680–57320`, `0x57320–5CA40` (ends exclusive). The first five are small
false positives outside the sound bank. Map confirmation is an annotation and
generation-selection pass, never input to the scanner or its score.

### 3.2 Samples and voice provenance

During discovery, every SPU RAM byte has a ROM-origin tag (2 MiB on the heap).
The mapper shadows transfer address `+0x1A6`, FIFO `+0x1A8`, and DMA ch4 in both
directions, including wrap. It resolves an origin only when **all 16 bytes** of
the addressed block have consecutive tags. Voice `+6` start / `+E` repeat
writes are recorded, then resolved again at key-on if an upload came later.
Zero repeat uses VibeStation's start-address fallback. Byte stores do not
advance the shadow FIFO; halfword/word and SWL/SWR paths follow the existing bus.

The 1800-frame clean map has **192/192 resolved events, 64/64 resolved key-on
starts**. Optional events change its map hash to `0xD3EDDE89438F1971`; old Phase 3
maps remain loadable with their old hashes, but lack voice metadata. Regenerate
with `--grim-map` to get the events. Map word classification remains unchanged.

Actual key-on starts refine the long byte run's first boundary:

| Index | ROM range | Blocks | Voices / first–last request |
|---:|---|---:|---|
| 5 | `0x521E0–0x55680` | 842 | 4–17, 20–21; 3.288–5.791 s |
| 6 | `0x55680–0x57320` | 458 | Uploaded to SPU RAM; no clean-boot voice use |
| 7 | `0x57320–0x5CA40` | 1394 | 0–3, 6–7, 18–19; 1.368–4.623 s |

`--grim-samples` lists headers' loop starts/ends, resolved repeat points, voice
IDs, SPU addresses and exact emulated cycles for each event. The JSON keeps the
raw scanner candidates separately from refined samples and unresolved events.
Map-less generation weights candidates by block count; with a map it prefers
SPU-confirmed candidates. The unplayed bank remains eligible (and can be a dud).

### 3.3 Optional unused / dormant view

`GrimBootMap::dormant_words()` and `grim_map_dormant_regions()` separate unused
words with read/copy flags from untouched words, without changing the serialized
class enum or old hashes. Stock result: **81,836 dormant words = 319.7 KiB**, and
14,644 untouched words = 57.2 KiB. The previous "81,836 words / 416 KB" wording
mixed distinct-word size with copy volume; 426,220 bytes were moved by the main
byte-copy routine, but that is not the size of the dormant class.

## 4. Sample genes and reproducibility

`spu_sample` is a ROM gene in **genome version 2**. It uses the same
`[offset, original_word, mutated_word, delay_slot]` patches as `rom_code`, with
delay-slot always 0. Parameters, in schema order:
`kind`, `count`, `magnitude`, `sample`, `donor`, `emulator_shift`, `block_phase`.
The last is 0 or 8, recording the target's 16-byte block lane. Indices are
generation metadata; resolved patches are authoritative when a map refines or
reorders samples. Descriptions locate ranges from patch offsets.

| Kind | Change / invariant |
|---|---|
| 0 `filter_swap` | Change filter 0–4; keep shift, flags and payload |
| 1 `shift_change` | Change shift nibble; keep filter, flags and payload |
| 2 `loop_start_move` | Clear old start bits and choose a new block's start bit |
| 3 `loop_end_remove` | Clear end bits; playback may enter adjacent data |
| 4 `loop_end_early` | Set an end bit before the final block |
| 5 `block_shuffle` | Shuffle a contiguous window of payloads |
| 6 `block_repeat` | Copy one source block's payload to selected blocks |
| 7 `block_reverse` | Reverse payload order within a contiguous window |
| 8 `transplant` | Copy another sample's payload blocks, wrapping the donor; retain target length |
| 9 `nibble_noise` | Flip selected payload nibbles; keep both header bytes |

Only explicit header genes change headers. All block operators move **14-byte
payloads**, retaining each target block's two header bytes. `count` is the
requested block count: shuffle/reverse need at least two blocks, and loop edits
may change existing flags throughout the sample. Generation retries no-ops;
an entirely ineffective CLI genome is an error. Shift 13–15 is allowed with
`emulator_shift=1`, and descriptions label it **VibeStation-defined, not
hardware-verified**. Small magnitude can wrap a shift into that range.

Application first checks the BIOS hash and **every original word across both
ROM families**, then writes all patches; a mismatch writes nothing. Overlapping
patches retain the existing ordered, last-word-write semantics. Version 1 text,
hashes and interface random streams remain unchanged. GUI boot/reset and
headless evaluation share the existing ROM application path.

## 5. Tests and audio results

All exit 0: `--cpu-backend-compare-test`, `--gpu-self-test`,
`--scheduler-self-test <bios>`, `--grim-self-test` (18 PASS), `--grim-gene-test`
(129 PASS), `--grim-map-test` (70 PASS), `--grim-sample-test` (75 PASS).
The sample test uses 600 frames and three runs per kind by default.

Sample fixtures cover both lanes, short/unterminated/invalid/silent runs and
the zero-prefix edge. Each kind gets **256 deterministic mutation checks**,
alternating lane-0/lane-8 samples, with JSON round trips, original words, range,
alignment, exact header masks, payload-source checks and reserved-shift tags.
Map tests cover FIFO/DMA wrap, deferred resolution, load delay, byte stores,
partial overwrites, high voices, optional-event round trip/merge and a real
synthetic interpreter FIFO boot. Existing gene loops now run to `Count`; static
families receive BIOS transport/atomic failure checks within those loops.

Audio liveness uses the Phase 1 gate: at least ten frames with RMS >=16,
positive zero crossings, and changing RMS or crossing rate (>5%). This checks
against silent/DC/one steady tone; it is not a perceptual quality judgment.
Changed means **at least one per-frame audio hash differs from clean boot**.
For each kind, two runs target the two played samples, one targets uploaded but
unplayed sample 6 (single requested block, magnitude 1; shuffle/reverse use 2).

| Kind | Runs | Survived + changed | Survived + identical | Audio failed | Dead |
|---|---:|---:|---:|---:|---:|
| filter_swap | 3 | 2 | 1 | 0 | 0 |
| shift_change | 3 | 2 | 1 | 0 | 0 |
| loop_start_move | 3 | 2 | 1 | 0 | 0 |
| loop_end_remove | 3 | 2 | 1 | 0 | 0 |
| loop_end_early | 3 | 2 | 1 | 0 | 0 |
| block_shuffle | 3 | 2 | 1 | 0 | 0 |
| block_repeat | 3 | 2 | 1 | 0 | 0 |
| block_reverse | 3 | 2 | 1 | 0 | 0 |
| transplant | 3 | 2 | 1 | 0 | 0 |
| nibble_noise | 3 | 2 | 1 | 0 | 0 |

This is targeted coverage, **not a random survival-rate estimate**. An earlier
three-played-target run had 3 changed / 0 identical for every kind. Every
unplayed-bank edit above is explicitly checked for audio identity. A separate
synthetic bank installed in untouched ROM also keeps the entire clean run hash
identical when its payload is edited. A composed genome containing all ten
sample kinds matches interpreter/recompiler audio, framebuffer, cycles,
key-on/voice masks and hit counts over 300 frames.

CLI checks: environment/default BIOS and explicit `--bios` sample reports are
byte-identical; flag-first `--grim-sample-test --frames 300 --seeds 1` passes.
`--grim-explore 100 4 ... 600 --mix samples --sample-genes 1 --sample-count 1`
with the new map: 4 alive, non-silent and audio-changed, zero unresolved comparisons.
Explore writes `audio_changed` to its table and `sample_survival.csv`; attribution
uses the first sample gene, so use one gene when measuring a kind.

## 6. Measured cost and parity

- Stock no-genome 1800 frames: **`run_hash=0x432E585CF1535F9C`**, unchanged.
- Full GT2 regression movie, **5738/5738 frames**: Phase 3 executable versus
  Phase 4 gives identical `BOOT_STATE_HASH` lines on **every field**, separately
  for interpreter and recompiler (including CPU/debug, device and system hashes).
- Mapper/no-mapper telemetry is identical; map determinism passes across threads
  and child processes. Sample mutations' deterministic JSON and backend checks
  are described in §5.
- Benchmarks: lean alternating Phase 3/delivery exes, reversing which executable
  runs first in each pair. The first batch had six pairs per load/backend;
  another twelve pairs per recompiler load checked its initial ~1% difference.

| Load (warmup + measured) | Pairs | Phase 3 median ms | Phase 4 median ms | Median paired change |
|---|---:|---:|---:|---:|
| Interpreter BIOS (600+600) | 6 | 6408 | 6321 | -1.6% |
| Interpreter GT2 (1500+600) | 6 | 6000 | 4027 | -3.3% |
| Recompiler BIOS (900+600), first batch | 6 | 1494 | 1509 | +1.0% |
| Recompiler GT2 (1500+600), first batch | 6 | 2373 | 2393 | +0.9% |
| Recompiler BIOS, additional batch | 12 | 1487 | 1485 | -0.3% |
| Recompiler GT2, additional batch | 12 | 5018 | 5017 | -1.1% |

No repeatable added cost. The machine still switches between fast/slow states:
the first interpreter GT2 batch contains 8 s and 3.6 s runs, and one recompiler
pair switched from 2.5 s to 5.1 s. All pairs are retained in external results;
the large median gap is not claimed as a speedup. The additional recompiler
batch resolves the small initial positive differences as session noise.

## 7. Commands and listening checklist for Lexon

Set `VIBESTATION_BIOS=D:\Misc\pSXfin_1_13-1220\bios\scph1001_original.bin`.
From the repo root after `build-ninja.cmd`:

```text
build-ninja\VibeStation.exe --grim-map 1800 docs\grim-reaper\maps\scph1001_phase4_nodisc_1800.json
build-ninja\VibeStation.exe --grim-samples docs\grim-reaper\maps\scph1001_samples.json --map docs\grim-reaper\maps\scph1001_phase4_nodisc_1800.json
build-ninja\VibeStation.exe --grim-sample-test --out-dir D:\Projects\github\vs-runs\phase4\review
build-ninja\VibeStation.exe --grim-random-genome 401 sample.json --mix samples --map docs\grim-reaper\maps\scph1001_phase4_nodisc_1800.json --sample-kind filter_swap --sample-genes 1 --sample-count 32
build-ninja\VibeStation.exe --grim-describe-genome sample.json docs\grim-reaper\maps\scph1001_phase4_nodisc_1800.json
```

Keep map JSON and its `.words` sidecar together. Sample reports derive from BIOS
bytes too; store them under the ignored maps directory or outside the repository.

**I did not hear the audio or inspect GUI output.** These four committed genomes
each ran 600 frames, stayed alive/non-silent, and changed audio hashes versus the
clean boot. Listening expectations below are inferred from the operators:

| Genome in `docs/grim-reaper/genomes/` | What to check by ear | Changed frames / first change |
|---|---|---|
| `sample_filter_crunch.json` (seed 401) | Opening chime develops predictor crunch/ringing around 1.4 s; it should still progress into the Shell | 471 / frame 83 |
| `sample_shift_steps.json` (402) | Opening chime has gain jumps/dropouts around 1.4 s; includes tagged emulator-defined shifts 13–15 | 471 / frame 82 |
| `sample_payload_reverse.json` (403) | Brief grain/click/warble in the later chime around 3.8 s, from reversed predictor-dependent payload order | 326 / frame 228 |
| `sample_early_loop_end.json` (413) | Opening sample ends/loops after block 52; listen for short repeating or buzzy sustain | 469 / frame 85 |

Boot each GUI genome with the original BIOS and no disc:

```text
build-ninja\VibeStation.exe --genome docs\grim-reaper\genomes\sample_filter_crunch.json
build-ninja\VibeStation.exe --genome docs\grim-reaper\genomes\sample_shift_steps.json
build-ninja\VibeStation.exe --genome docs\grim-reaper\genomes\sample_payload_reverse.json
build-ninja\VibeStation.exe --genome docs\grim-reaper\genomes\sample_early_loop_end.json
```

Repeat with `--cpu recompiler` before `--genome`. Listen from reset through the
Shell, checking that the sound changes are interesting rather than merely loud,
and that the machine still progresses. Captured WAVs are external under
`vs-runs/phase4/listening`; a wrong BIOS is rejected by the existing hash check.

## 8. Limits and remaining work

- Only SCPH-1001/no-disc is scored and listened-to-by-proxy here. Other BIOSes,
  disc audio and menu interactions need their own maps and listening checks.
- Address events record CPU write requests, before the sample-clock key-on
  latch. The mapper does not clear tags for SPU-internal capture/reverb writes;
  the clean-boot bank is outside those buffers. It is not a playback decoder trace.
- Map-less scanner generation can hit the small false positives. A map filters
  automatic targets/donors to confirmed SPU data; explicit indices can still
  select a decoy. A transported bank can be unplayed, and a valid edit can be
  inaudible because of playback position, loop termination or small magnitude.
- Existing save-state/genome, telemetry globals and watchdog limits remain
  as recorded in earlier phases. No new background search or fingerprint system.
- MSVC build/tests only here; GCC/POSIX paths were reviewed but not built.
- Dormant is a derived summary view, not a new serialized class or gene target.

## 9. Proposed Phase 5 brief (fingerprints, archive, background search)

Goal: turn the surviving, distinct corruptions into a reproducible library of
broken PlayStations (DESIGN Layer 3 fingerprinting and Layer 4 quality-diversity
search). Start `grim-reaper/phase-5` from this Phase 4 branch.

- **Fingerprints/descriptors.** Compute stable, versioned audio/visual/behaviour
  descriptors from captured outputs: audio spectral/temporal change, silence/DC/
  stationary-tone evidence, stereo features; visual change/colour/structure;
  Shell/disc progress, coverage and exceptions. Compare with the clean scenario.
  Define quantization and hash inputs so MSVC/GCC and repeated runs agree. Keep
  fingerprint work in evaluation, never normal play; distinguish live-but-identical
  duds from useful survivors. Test descriptor sensitivity with Phase 2/3/4 fixtures.
- **Archive.** A deterministic MAP-Elites grid with documented descriptor axes,
  per-cell quality/replacement rules and novelty against the **whole archive**.
  Persist genome v2, BIOS hash, scenario, descriptor/fingerprint versions,
  thumbnails/audio review paths and outcomes. Keep BIOS-derived data/artifacts
  outside Git; validate compatibility and recover atomically after interruption.
- **Search.** Process-isolated staged evaluation (kill check → survival → longer
  run), bounded CPU budget, hard timeouts and saved host-crash genomes. Breed
  typed genes from archive parents, vary family/count/intensity, and record
  reproducible seeds/parents. Start with CLI archive/search/query modes before
  hooking New Corruption to an archive choice. An empty archive is a clear state.
- **Tests/results.** Same genome → same fingerprint/cell; near-identical runs
  deduplicate; known different examples remain distinguishable; deterministic
  replacement/reload; timeout/crash recovery; report survival-and-change yield,
  occupied cells and evaluation throughput across a fixed seed batch. Recheck
  stock hash, full GT2 parity, all older suites and alternating lean cost.
- **Out of scope:** new geometry/texture/font ROM genes (Phase 6), probe mapping
  (7), full gallery/breeding/inspector/sharing UX (8), CPU/scheduler redesign.

---

# Appendix C — Phase 3 reference (unchanged)

The sections below are the completed Phase 3 record. Phase 4 status above is current.

## 1. Status (2026-09-30)

**Phase 3 (clean-boot BIOS map, provenance, ROM code genes) is done and committed
on branch `grim-reaper/phase-3`, branched from `grim-reaper/phase-2` (`5d25b0b`).**
Phases 1 and 2 are unchanged in behaviour. Phase 4 has not been started. Every
phase gets its own branch (`grim-reaper/phase-N`); branch the next one from this one.
Phase 1 and Phase 2 references are kept in Appendix A and B.

Definition of done for Phase 3:

- [x] Branch `grim-reaper/phase-3` from `grim-reaper/phase-2`
- [x] Copy routines measured and documented (§3); per-byte tags chosen and justified
- [x] `GrimBootMap`, `--grim-map`, `--grim-map-summary`; the map is deterministic
      (threads and processes, byte-identical files); provenance coverage reported
      (100.0%, 16355 of 16355 executed RAM words)
- [x] ROM code gene family (`rom_code`, genome version 2) with all mutation kinds,
      original-word verification and a BIOS-hash check
- [x] ROM genes applied in headless eval, explore and GUI `--genome` boot
- [x] Explore survival table by first-execution time and mutation kind (+ CSV);
      `--grim-describe-genome`
- [x] All tests in the brief pass: `--grim-map-test` (new) plus every older suite
- [x] Normal play unchanged: no-genome `run_hash` is still `0x432E585CF1535F9C`,
      the GT2 replay hashes are identical to Phase 2, no measurable cost (§6)
- [x] This file and `PROJECT_BRIEF.md`

Things I want you to look at, because you asked me to ask first about the CPU,
DMA and scheduler paths:

1. `DmaController::dma_block` and `dma_linked_list` each gained one
   `if (boot_mapper_ != nullptr)` call per block/packet (not per word).
2. `Cpu::run_slice`'s existing telemetry loop gained the mapper calls. `Cpu::step()`,
   the dynarec and the scheduler are untouched.
3. `System::reset()` and `System::boot_disc()` call `grim_apply_rom_genes()`, which
   returns immediately when no genome has ROM genes.
4. I renamed your running `build-ninja\VibeStation.exe` to
   `VibeStation.running-old.exe`: the exe was running and locked, so the build
   could not overwrite it. You can delete the old file once that instance is closed.

## 2. What changed in Phase 3

| File | Contents |
|---|---|
| `src/core/grim_map.{h,cpp}` | `GrimBootMapper` (the discovery tracker), `GrimBootMap` (result, classes, regions, JSON + binary files, summary), copy statistics |
| `src/core/grim_rom.{h,cpp}` | MIPS validity/decoding/disassembly, the nine mutation kinds, `GrimRomContext` (stock image + map), the weighted ROM gene generator, `--grim-describe-genome` text |
| `src/platform/grim_map_runner.{h,cpp}` | `--grim-map`, `--grim-map-summary`, `--grim-describe-genome` |
| `src/platform/grim_map_test.cpp` | `--grim-map-test` |

Small edits elsewhere: `cpu.h/.cpp` (mapper pointer, telemetry-loop calls),
`dma.h/.cpp` (mapper pointer, two hook calls), `system.h/.cpp`
(`set_grim_boot_mapper`, ROM genes applied at reset/after fast-boot patch/when a
genome is set, `grim_rom_error()`), `bios.h/.cpp` (`image_hash()`, `original_word()`),
`grim_genome.h/.cpp` (gene type `rom_code`, genome v2, `apply_rom`, random ROM
genes), `grim_eval.h/.cpp` (`boot_map_out`, `copy_stats_out`, `rom_gene_mismatch`),
`grim_eval_runner.cpp` (`--map`, `--mix`, ROM options and survival tables for
`--grim-random-genome` / `--grim-explore`), `main.cpp`, `CMakeLists.txt`,
`.gitignore` (`docs/grim-reaper/maps/`, because maps derive from BIOS data).
`grim_gene_test.cpp`: two loops that ran over every gene type now stop before
`RomCode`.

### 2.1 The map

Scenario: **"no disc", boot into the Shell, 1800 frames (30 s emulated)**. The
last never-before-executed instruction runs at frame 681 (11.4 s); the Shell then
only idles. 1800 is 2.6x that. The classification of a 1800- and a 3600-frame run
is identical. Interpreter speed is ~4x real time, so a map takes about 15 s.

Per ROM word the map stores: flags (executed directly / executed from RAM / read
directly / read via a RAM copy / used / partial), first-execution and
first-read cycle, consumer mask, class.

- `code`: executed, directly or from RAM via provenance. Code that also went to a
  peripheral would be `unknown` (none seen).
- `data`: loaded and **used**: consumed by an ALU op, branch or jump, a device
  register, the GTE, or a DMA to a peripheral.
- `unused`: never touched, **or loaded only to be moved**. A copy loop's load/store
  and a DMA to RAM are transport, not use. The bulk ROM copy would otherwise label
  81,836 words (416 KB of Shell code, fonts, unused data) as `data`. The summary
  says how many `unused` words were copied and never used. "Unused" means unused *in
  this scenario*: a disc boot or the memory card/CD player screens may use them.
- `unknown`: a word of an executed RAM word whose four bytes came from different
  ROM words (partial provenance), or code that also reached a peripheral.
- Consumer: `spu` (SPU registers/FIFO/SPU DMA), `gpu` (GP0/GP1/GPU DMA), `gte`
  (lwc2/mtc2/ctc2), `mdec`; `cpu` if none.

Stock SCPH-1001 result (BIOS hash `0x32B1A0FA4DB70C8F`, map hash
`0x4EABEA109DBD0DEA`): code 17,780 words (13.6%), data 16,812 (12.8%), unused 96,480
(73.6%), unknown 0. 10,800 data words reach the SPU (sound bank), 4,633 the GPU.

Files: `<out.json>` (header, provenance, counts, merged regions) and
`<out.json>.words` (20 bytes per word: binary). The JSON does not name the
sidecar; load looks for `<path>.words`, so keep them together. Both are
byte-reproducible.

### 2.2 Provenance and the copy routines (measured)

Every RAM byte (and scratchpad byte) carries a `u32` tag: ROM byte offset + 1, 0
= none. 8 MiB on the heap, discovery only. Registers carry four byte tags.

Measured on the stock BIOS (`--grim-map` prints this as `GRIM_MAP_COPY`):

| Copy routine | What | Volume |
|---|---|---|
| `0xBFC02B68` `lbu` / `0xBFC02B7C` `sb` | **byte loop, ROM to RAM**: the kernel/Shell image | 426,220 bytes (over 99% of all ROM bytes loaded by the CPU directly) |
| `0xBFC00434` `lw` / `0xBFC00448` `sw` | word loop, ROM to RAM | 35,824 bytes |
| `0x8003D8A8` `sw`, `0x00001B6C`/`1BA0`, `0x80050C80`/`D00` `sw` | word copies RAM to RAM (relocation, buffers) | tens of thousands of bytes |
| DMA | RAM to SPU (157,036 words with ROM origin in 1800 frames) and 3.18 M words from CD/MDEC/OTC to RAM | |
| I/O stores | 21,987 tagged stores to SPU/GPU registers | |

So **byte tags are required**: the main copy is a byte loop. Word tags would lose
it. There is no decompression: provenance coverage is **100.0%** (16,355
distinct executed RAM words, all with a known ROM origin). Hence ROM genes can
reach every word of the kernel and the Shell the map calls `code`.

Tag rules: loads take byte tags from memory (ROM loads tag themselves; `lwl/lwr`
merge), stores write them; exact register copies (`move`, `sll x,0`,
`addiu/ori x,y,0`) keep them, any other ALU op clears them. A load lands after
the next instruction (load delay). **Stores with SR.IsC set do not touch tags**
(they go to the I-cache). DMA RAM to device consumes tags; DMA device to RAM
clears them. Last writer wins; the ROM origin is read at execution time.

### 2.3 ROM genes

Genome **version 2**: `{"version":2,"bios_hash":"0x...","genes":[...]}`; a ROM gene is

```json
{"type":"rom_code","seed":1,"params":{"kind":2,"count":3,"early_ms":200,"curve":2},"patches":[[102916,268435459,335544323,0]]}
```

`patches` = `[rom_offset, original_word, mutated_word, in_delay_slot]`. Version 1
files parse and serialize exactly as before (Phase 2 sample hashes unchanged); a
`rom_code` gene in a v1 file, a v2 file without `bios_hash`, unaligned offsets,
out-of-range params or unknown fields are errors. A genome without ROM genes is
still written as version 1.

Applying (`GrimGenomeRuntime::apply_rom`): checks the BIOS image hash, then every
`original` against the **stock** image, then patches through `Bios::patch32()`.
Any mismatch is an error and nothing is patched (headless: `end=rom_gene_mismatch`,
`status=error`; GUI: `GRIM: ROM genes NOT applied: ...` on the console). It happens
at `System::reset()` (so reboots and "reap and reboot" replay it), after the
fast-boot patch, and when a genome is set. The BIOS copies patched code to RAM
itself.

Generation (`grim_rom_generate`): candidates are `code` words first executed at or
after `early_ms` (default 200 ms: the first 2,043 code words run in the first
7 ms of reset and kernel init, then nothing new until 133 ms) and applicable to the
kind. Weight = 16 + (later first execution)^curve (0 uniform .. 3 cubic), by
rejection sampling in integers. Kinds (`kind` param): 0 imm_perturb, 1 reg_subst
(never `$sp $ra $k0 $k1`, and `$gp`: the BIOS does use it as a load/store base,
so it is avoided too), 2 branch_invert (BEQ/BNE, BLEZ/BGTZ, BLTZ/BGEZ, not the AL
forms), 3 branch_nudge (+-1..4 words, target inside `code`), 4 alu_subst,
5 ls_width (alignment of the offset checked for the new width), 6 nop,
7 lui_ori_const (LUI with ORI/ADDIU partner within 3 words, re-encoded),
8 call_swap (off by default; `--call-swap`). Every result passes
`grim_mips_valid`. Delay-slot words are allowed and flagged.

CLI additions (`--grim-random-genome`, `--grim-explore`): `--mix interface|rom|both`,
`--map file`, `--rom-patches N` (max per gene), `--rom-genes N` (max genes),
`--early-ms N`, `--curve 0-3`, `--call-swap`. The ROM part uses its own random
stream, so the interface genes of a seed do not change. Explore prints
`GRIM_SURVIVAL` tables and writes `survival.csv`.

### 2.4 Disc scenario (added after the first Phase 3 commit)

- `--grim-map <bios> <frames> <out.json> --disc <game.cue>` maps a boot with the disc
  inserted (scenario `disc`). `--grim-map-merge a.json b.json out.json` unions two
  maps of the same BIOS (flags and consumers OR-ed, first-touch minimum).
  Crash Bandicoot (USA), 1800 frames: code 20,047 words (no-disc: 17,780); merged
  no-disc + disc: code **24,565**, data 19,531, unused 86,976. Provenance in a disc
  map is ~51% because the game's own code runs from RAM with no ROM origin; that is
  expected. Make both maps and merge them, then pass the merged map with `--map`.
- `--grim-eval ... --disc game.cue` and `--grim-explore ... --disc game.cue` boot with
  the disc. The result line has `cd_words=` (CD DMA words read). In explore, a machine
  that is alive but read under 512 words is reported as `disc_not_read` (it stays a
  survivor, and has its own column in the tables/CSV).
- Check: 3 seeds on the merged map with the disc: 3 alive, `cd_words` 103k-117k.
  A 60-seed run (single-patch, uniform): 58 alive, 2 dead-by-gate... plus 4 alive but
  `disc_not_read` (first_exec 1.4-6.5 s quintiles), so game loading breaks
  independently of liveness. Late-code survival again varied (80-100%), not monotonic.
- Not yet covered: the disc tests in `--grim-map-test` (only manual checks so far), and
  only one game was used to build the disc map.

## 3. Test checklist for Lexon

From the repo root after `build-ninja.cmd`, with
`set VIBESTATION_BIOS=D:\Misc\pSXfin_1_13-1220\bios\scph1001_original.bin`.

1. **Phase 3 tests** (about 3 minutes): `build-ninja\VibeStation.exe --grim-map-test`
   expects 60 `GRIM_MAP_TEST PASS` lines and `GRIM_MAP_TEST_RESULT PASS failures=0`.
2. **Make the map**: `mkdir docs\grim-reaper\maps` then
   `build-ninja\VibeStation.exe --grim-map 1800 docs\grim-reaper\maps\scph1001.json`
   expects `GRIM_MAP_RESULT status=ok ... code=17780 data=16812 unused=96480
   unknown=0 provenance=100.0%` and `map_hash=0x4EABEA109DBD0DEA` (same on your
   machine if the BIOS file is identical), then the `GRIM_MAP_COPY` report.
3. **Read it**: `build-ninja\VibeStation.exe --grim-map-summary docs\grim-reaper\maps\scph1001.json`
   (option `--min-bytes 0` lists every region). Compare with what you know: the
   32 KB block table should show the boot code below 0x10000, kernel + Shell code
   in 0x10000-0x48000 (all run from RAM), and the sound bank / GPU data at
   0x48000-0x60000.
4. **Older suites, all exit 0**: `--cpu-backend-compare-test`, `--gpu-self-test`,
   `--scheduler-self-test <bios>`, `--grim-self-test` (`failures=0`),
   `--grim-gene-test` (`failures=0`). Stock hash: `--grim-eval 1800 stock.jsonl`
   still prints `run_hash=0x432E585CF1535F9C`.
5. **A ROM genome**:
   `build-ninja\VibeStation.exe --grim-random-genome 5 rom5.json --mix rom --map docs\grim-reaper\maps\scph1001.json`
   prints `GRIM_GENOME seed=5 genes=3 hash=0x304A87B0447790D4`; then
   `--grim-describe-genome rom5.json docs\grim-reaper\maps\scph1001.json` lists
   every patch as disassembly before and after, with region, first execution time
   and "runs from RAM"/"delay slot"; `--grim-eval 900 rom5.jsonl --genome rom5.json`
   gives `verdict=alive` (and `run_hash=0x43B02E4594AC8DE6`).
6. **Survival statistics**:
   `--grim-explore 100 100 ex_out 900 --mix rom --map docs\grim-reaper\maps\scph1001.json --rom-patches 1 --rom-genes 1 --early-ms 0 --curve 0`
   (about 8 minutes) ends with the two `GRIM_SURVIVAL` tables and
   `ex_out\survival.csv`. My run: 88 of 100 alive; see §5.
7. **Watch and hear survivors** (I could not; no display or audio here). The
   interesting ones are the dead-adjacent ones that survive: mutations in the
   Shell's early code. Try a whole-genome of early code but not init:
   `--grim-explore 1 40 ex_early 900 --mix both --map ... --early-ms 10 --curve 0 --rom-patches 12`
   then open a survivor in the GUI, in both CPU modes:
   `build-ninja\VibeStation.exe --genome ex_early\survivors\seed_N\genome.json` and
   `build-ninja\VibeStation.exe --cpu recompiler --genome ex_early\survivors\seed_N\genome.json`,
   boot the BIOS (no disc). Console shows `GRIM: genome ... loaded`; a wrong BIOS
   prints `GRIM: ROM genes NOT applied`. Survivors keep `telemetry.jsonl`,
   `audio.wav` and `frames\*.bmp` for looking and listening first.

## 4. What this phase learned about the BIOS (useful for Phase 4)

- SCPH-1001 `0x32B1A0FA`: executed-from-ROM boot code is in 0x0-0x10000
  (1,352 code words in 0x0-0x8000, first executed at 0 ms and 4-7 ms).
  Kernel and Shell are **copied whole** (byte loop at `0xBFC02B68`) from 0x10000
  up and execute from RAM: code 0x10000-0x48000.
- The sound bank and GPU data sit above it: data at 0x40000-0x60000, 10,800 words
  (43 KB) consumed by the SPU (RAM to SPU by `sh`/FIFO stores and DMA ch4,
  21,987 tagged I/O stores, 157,036 DMA words), 4,633 words reach the GPU.
  Consumer `spu` in the map is the confirmed list of ADPCM words for Phase 4;
  `first_read` gives when each is first loaded.
- No decompression of code. Nothing executes without a known origin.
- 0x60000-0x80000 (font, etc.) is mostly `unused` here; the copy loop moves it to
  RAM but the Shell never uses it in this scenario.
- Late code is not safe per se: see §5.

## 5. Results and the design hypothesis

100 single-patch genomes, `--early-ms 0 --curve 0` (uniform over executed code),
900 frames, quintiles of first execution:

| first exec | n | alive |
|---|---|---|
| 0-884 ms | 28 | 75% (4 stall, 3 exception loop) |
| 884-1369 ms | 20 | 95% |
| 1369-3256 ms | 16 | 100% |
| 3256-6544 ms | 22 | 91% |
| >= 6544 ms | 14 | 86% |

By kind: branch_invert 10/10, alu_subst 10/10, reg_subst 14/15, nop 15/17, imm 7/8,
ls_width 12/14, branch_nudge 11/14, lui_ori 9/12. So **the early code is the most
fragile, but the gradient is weak and not monotonic** (n is small, and a single
word rarely kills). Hand tests: NOPing all 186 code words below 0x45C kills the
machine (exception loop at frame 60); +1 on six late `addiu` immediates keeps 4
of 6 alive (2 coverage_stall), 5 of 6 change the run. Mutated late code can
die, so the early-window/curve defaults are a bias, not a guarantee. A fairer test
needs many more seeds (Phase 5 infrastructure).

## 6. Measured cost and parity

- Stock no-genome `run_hash` `0x432E585CF1535F9C`, unchanged; a run with the mapper
  attached has the identical telemetry (test `mapper_does_not_change_emulation`).
- GT2 replay, 2000 frames, `VIBESTATION_STATE_HASH_*`: Phase 2 exe and Phase 3 exe
  give identical `BOOT_STATE_HASH` lines (2000 of 2000, every field) for the
  interpreter and for the recompiler.
- Lean `--cpu-benchmark`, alternating exes, 5 pairs: interpreter BIOS menu
  600+600 p2 median 6119 ms vs p3 6153 ms; recompiler 900+600 p2 2509 vs p3 2501 ms.
  No change.
- Interpreter vs recompiler with ROM genes (300 frames, composed genome): per-frame
  fb/audio/cycles/key-on equal (`rom_genes_match_between_interpreter_and_recompiler`).

## 7. Known issues and untested areas

- Discovery is interpreter-only and one scenario (no disc). A disc boot or memory
  card screens would mark more words `code`/`data`. Each map is keyed by BIOS hash;
  only SCPH-1001 was tested.
- `first_touch` of an `unused` region that was copied is the time of the copy.
- Use tracking is conservative: a byte-swap copy (ALU on tagged bytes) would count
  as use. None seen in this BIOS.
- Register tags ignore load-delay forwarding into `lwl/lwr` and the interrupt edge
  case (a load in flight when an interrupt is taken).
- The explore table attributes a multi-gene genome to its earliest patch; use
  `--rom-genes 1 --rom-patches 1` for clean attribution. In `--mix both` the
  interface genes confound it.
- ROM genes need the original BIOS file: "reap and reboot" of a corrupted BIOS
  file refuses them (loudly). `--boot-disc-test`/`--frame-test` ignore `--genome`
  (your Phase 2 answer). Genome state is not in save states (future work).
- POSIX `grim_process.cpp` and the new code were built only with MSVC here.
- `build_number.txt` is modified by every build; never commit it. Untracked
  `boot_disc_test.log` and `perimeter_crash_dump.txt` in the repo root are
  leftovers of test runs, not part of the commit.
- `--grim-random-genome`/`--grim-describe-genome` print BIOS load info lines.

## 8. Open questions

1. Do you want a second scenario (disc inserted) so disc-boot and memory-card
   code becomes `code` too? It changes which words ROM genes may touch.
2. The early window default is 200 ms and the curve 2. Given §5 (weak gradient),
   do you prefer a flatter default (curve 1) so ROM genes are more varied in
   Phase 5, at the price of more dead machines?
3. Should `unused`-but-copied words (416 KB of Shell code and data that is copied
   to RAM but never used here) become targets for probe mapping in Phase 7?

## 9. Proposed Phase 4 brief (ADPCM scanner, sample genes)

Goal: find the sound bank in ROM and add `spu_sample` genes (DESIGN 1.3, 2.2).

- **Scanner** over the ROM image for runs of aligned 16-byte ADPCM blocks (filter
  <= 4, shift <= 12, plausible flags, loop-end flag closing a sample).
  **Confirm with the map**: words with consumer `spu` (10,800 words, mostly
  0x40000-0x60000 on SCPH-1001) are the ground truth; report scanner precision/recall
  against them and the region list, and start sample boundaries from SPU
  start-address register writes (trace `Spu::write16` voice `+6` and DMA ch4
  addresses) mapped back through provenance (`note_dma` already has the RAM
  addresses and tags).
- **Sample genes** as a new family next to `rom_code`: filter swap, shift change,
  loop flag edits, block shuffle/repeat/reverse, transplant, nibble noise
  (headers kept). Same patch representation (`rom_code`-style resolved
  `(offset, original, mutated)` list with a new `type`), same hash check and
  original-word verification, genome version stays 2.
- Tests: scanner against the map's SPU words; every sample gene keeps block
  alignment; the boot chime still starts (audio liveness) for small edits; a gene
  on an unplayed sample is silent-identical (`audio_hash` equal).
- Out of scope: fingerprints/archive (Phase 5), geometry/texture/font genes (6).

---

# Appendix B — Phase 2 reference (unchanged)

The sections below are the Phase 2 record; its instructions still apply.

## 1. Status (2026-09-30)

**Phase 1 (deterministic headless runner, telemetry, liveness gates) is done and
unchanged. Phase 2 (interface genes) is done and committed on branch
`grim-reaper/phase-2`, branched from `grim-reaper/phase-1` at `ccdacf2`.**
Phase 3 has not been started. Every phase gets its own branch
(`grim-reaper/phase-N`); branch the next one from this one.

Definition of done for Phase 2:

- [x] Branch `grim-reaper/phase-2` created from `grim-reaper/phase-1`
- [x] Genome type, JSON format, generator, triggers, genome hash
      (`src/core/grim_genome.{h,cpp}`)
- [x] SPU register filter with all eight gene families (§2.3)
- [x] GP0 filter with all transforms (§2.4); the word count is always preserved
- [x] `--genome` for headless eval (`--grim-eval`), for GUI boot, and for
      `--grim-determinism-test`; `--dump-wav`, `--dump-frames`
- [x] `--grim-random-genome` and `--grim-explore` (child process, Job Object
      with kill-on-close on Windows, hard timeout, host crashes saved)
- [x] All tests in `--grim-gene-test` pass (96 checks); the Phase 1 tests
      and the existing tests still pass
- [x] Genome-off cost measured: nothing measurable (§3)
- [x] Normal emulation unchanged: a 2000-frame GT2 replay gives byte-identical
      `BOOT_STATE_HASH` lines (every field, including `spu` and `gpu`) for the
      Phase 1 exe and the Phase 2 exe, for both the interpreter and the
      recompiler
- [x] `PROGRESS.md` and `PROJECT_BRIEF.md` updated

Three things I want you to look at, because you asked me to ask first about
them: (1) `System::sync_spu` has a new loop that applies delayed SPU writes at
their due cycle. It does nothing without a genome, but it is in the device
sync path. I chose it over a new scheduler event (§2.5). (2) `System::run_frame`
gained one `if (grim_ != nullptr)` call at the top. (3) I did not touch the CPU
hot path, the dynarec or threading.

## 2. What changed in Phase 2

New files:

| File | Contents |
|---|---|
| `src/core/grim_genome.{h,cpp}` | `GrimRng` (splitmix64), `GrimGenome`/`GrimGene`/`GrimTrigger`, strict JSON parser and canonical serializer, genome hash, `grim_random_genome()`, `GrimGenomeRuntime` (the SPU and GP0 filters, the delayed-write queue, per-gene hit counters) |
| `src/platform/grim_process.{h,cpp}` | `grim_run_child()`: child process with a hard timeout. Windows: `CreateProcess` suspended, `Job Object` with `KILL_ON_JOB_CLOSE`, resume, `TerminateJobObject` on timeout, crash dialogs suppressed. POSIX: `fork`/`execv` in its own process group, `SIGKILL` of the group on timeout. |
| `src/platform/grim_gene_test.cpp` | `--grim-gene-test` |
| `docs/grim-reaper/genomes/*.json` | 15 hand-written genomes to try by ear and by eye (§3) |

Edited files (all small):

- `system.h/.cpp`: `System::set_grim_genome()`, `grim_write_spu16()`; the two
  SPU write sites (`write16`, and `write32` after it splits into two halves)
  call the filter when a genome is set; `sync_spu()` applies delayed writes;
  `run_frame()` passes the frame counter; `reset()` rewinds the genome.
- `gpu.h/.cpp`: one pointer. `Gpu::gp0()` calls `filter_gp0()` on the whole
  buffered command right before `apply_reaper_to_gp0_command()` and then
  refreshes `gp0_command_` (bits 24/25 feed the semi-transparency and
  raw-texture decisions). `handle_polyline_word()` filters polyline tail words.
- `grim_eval.{h,cpp}`: `GrimEvalConfig::{use_genome, genome, dump_wav_path,
  dump_frames_dir, dump_frames_every, native_cpu, test_hang}`;
  `GrimEvalResult::{genome_hash, gene_hits}`; both are in the summary line as
  `genome_hash` and `gene_hits`. `grim_write_wav`, `grim_write_bmp`.
- `grim_eval_runner.{h,cpp}`: `--genome`, `--dump-wav`, `--dump-frames`,
  `--test-hang` on `--grim-eval`; `--genome` on `--grim-determinism-test`;
  `--grim-random-genome`; `--grim-explore`. The result line gained
  `genome=0x<hash>`.
- `main.cpp`: mode dispatch, and the GUI flag `--genome <file>`.
  `ui/app.cpp`: `App::init_runtime` applies it to the System it creates.
- `CMakeLists.txt`: three new sources. `PROJECT_BRIEF.md`: new section.

Nothing else changed. With no genome the emulator takes the exact same paths as
Phase 1 apart from the `nullptr` checks.

### 2.1 Genome file format

```json
{"version":1,"genes":[
{"type":"spu_pitch","target":16777215,"seed":101,"trigger":{"kind":"rot","start_frame":0,"end_frame":900},"params":{"mode":1,"mul_q8":384,"offset":-700,"scale":2741,"depth":64,"period":120}}
]}
```

- Fixed field order: `type, target, seed, trigger, params`. The serializer
  writes one gene per line, and the trigger and params objects in schema
  order. `grim_genome_hash` is FNV-1a-64 over that text, so the hash does not
  depend on how a file was formatted.
- Parsing is strict. Unknown gene types, unknown or missing fields, values out
  of range, floats where an integer is required, a bad trigger, a `target` of 0
  or outside the type's mask are errors, with a message. Nothing is skipped.
  Every parameter is an integer (no floats anywhere, on purpose).
- `seed` is the gene's splitmix64 start state. The stream is advanced once per
  event the gene handles (an event = a register write or a GP0 command that
  matches the gene's register/opcode class and target), whether or not the
  trigger is currently on. Per-bit or per-vertex decisions come from a stateless
  mix of that one draw, so they do not advance the stream.
- Time is `boot_diag_.frame_counter`: it is incremented only in `run_frame()`
  and zeroed by `System::reset()`. It never reads host time.
- Trigger `kind`:
  - `always`
  - `window` (`start_frame`, `end_frame`): on for `start <= frame < end`
  - `rot` (`start_frame`, `end_frame`): magnitude ramps 0 to 1 across the
    range, then holds at 1. Every gene scales its effect by the magnitude
    (offsets, jitter and drift scale linearly, XOR masks keep a growing
    fraction of their bits, discrete choices apply with that probability).
  - `intermittent` (`start_frame`, `end_frame` (0 = forever), `period`, `duty`,
    `probability`): with `period > 0`, on for the first `duty` frames of every
    `period` (counted from `start_frame`); with `period 0`, on with
    `probability` permille per event, drawn from the gene's RNG.
- `target`: SPU genes take a voice mask (bits 0-23; bit 24 also selects the
  main volume registers for `spu_volume`). GPU genes take primitive classes
  (1 polygons, 2 lines and polylines, 4 rectangles); `gpu_state` takes one bit
  per command (bit n = E(n+1)); `gpu_fill` takes 1.

### 2.2 Gene reference

| Type | Acts on | Params (range, default) |
|---|---|---|
| `spu_pitch` | voice `+4` | `mode` 0 multiply, 1 offset, 2 quantize to a scale, 3 wobble. `mul_q8` (16-1024, 384; 256 = x1). `offset` (+-4096, 256; pitch units, 0x1000 = the sample's own rate). `scale` (1-4095, 0xAB5): 12-bit semitone mask, bit k allows semitone k. `depth` (0-1024, 64) and `period` (frames, 1-600, 120): triangle wobble, depth is a q10 fraction of the pitch. Result clamped to 0-0x3FFF. |
| `spu_adsr` | voice `+8`, `+A` | `mode` 0 infinite sustain (slowest decreasing sustain, level F), 1 instant release, 2 attack, 3 decay, 4 sustain level, 5 release (2-5 XOR random bits inside that field only). |
| `spu_volume` | voice `+0`, `+2`; main `0x180/0x182` when target bit 24 is set | `mode` 0 swap L/R (the write is redirected to the other register), 1 invert sign, 2 clamp (`clamp` 0-16383, 4096), 3 sweep (`depth`, `period`: the magnitude follows a triangle wave). Sweep-mode volume words (bit 15) are left alone by modes 1-3. |
| `spu_address` | voice `+6` start, `+E` repeat | `which` 0 start, 1 repeat, 2 both. `blocks` (+-4096, 16): offset in 8-byte units, so the result stays 8-byte aligned and wraps inside the 512 KiB. `random` 1 = a random offset in +-blocks per write. |
| `spu_keyon` | KON `0x188/0x18A`, KOFF `0x18C/0x18E` | `mode` 0 drop, 1 duplicate (the copy arrives `delay` later), 2 delay. `keys` 0 key-on, 1 key-off, 2 both. `prob` (permille per voice bit, 300), `delay` (SPU samples of 768 CPU cycles, 1-8192, 200). |
| `spu_noise` | NON `0x194/0x196` | `piggyback` 1: besides forcing target bits on when the game writes NON, also write NON with the key-on's voice bits just before every matching key-on, so it works even if the game never touches NON. |
| `spu_pmon` | PMON `0x190/0x192` | Same, for pitch modulation. Voice 0 is skipped (it has no neighbour). |
| `spu_reverb` | `0x1C0-0x1FE` config, `0x1A2` work base, EON `0x198/0x19A`, SPUCNT `0x1AA` | `cfg` (permille of config writes that get random bits flipped, 200), `base` (+-4096, 0; moves the work-area base, 8-byte units), `eon` 1 (force EON bits on target voices, also on every matching key-on), `spucnt` 1 (force the reverb-enable bit 7). For the erosion effect: `eon 1 spucnt 1 base` non-zero. |
| `gpu_vertex` | vertex words | `mode` 0 fixed offset (`dx`, `dy`), 1 noise (`amp` 0-1023), 2 drift (`dx`,`dy` = pixels per 60 frames), 3 snap to `grid`, 4 swap two vertices' coordinates (`prob` permille). `wrap` 0 clamp to -1024..1023, 1 wrap as signed 11 bit. |
| `gpu_color` | color words | `mode` 0 channel permute (`perm` 0-5, 6 = random per event), 1 invert, 2 gradient shuffle (rotate colors among the vertices of a gouraud primitive), 3 XOR `tint`. The top byte of the word (opcode) is kept. |
| `gpu_flags` | bit 25 on any primitive, bit 24 on textured ones | `semi`, `raw`: 0 keep, 1 set, 2 clear, 3 toggle. `prob` (permille per primitive). |
| `gpu_texparam` | CLUT (upper half of UV word 0), texpage (upper half of UV word 1) | `which` 0 CLUT, 1 texpage, 2 both. `clut_mask`, `tp_mask`: bits XORed. `prob`. Rectangles have a CLUT only. |
| `gpu_state` | E1..E6 | `amp` (pixels of random jitter for E3/E4/E5), `drift` (pixels per 60 frames), `e1_mask`, `e2_mask` (texture window, a strong effect), `e6_mask`: XORed. |
| `gpu_fill` | `0x02` | `color_mask` XORed into the color, `amp` perturbs the rectangle. |

GP0 rules, enforced twice (in each gene, then by a final guard in
`filter_gp0()`): the word count never changes; bits 26-31 of the first word are
never touched (they decide the length); for state, fill and transfer commands
the whole opcode byte is kept; bit 24 moves only on textured primitives; the
fuzz test runs every gene over all 256 opcodes and checks these plus "only the
words this gene owns change". Polyline terminators are passed through and are
never created.

### 2.3 SPU register filter semantics

- The choke point is `System::write16` / `System::write32` for io
  `0xC00-0xFFF`. `write32` splits into two 16-bit writes first and filters each,
  so KON/KOFF halves (`0x188/0x18A`, `0x18C/0x18E`) and every other pair are
  seen consistently. The recompiler reaches the same code through
  `v4_bus_write16/32`.
- **Readback:** a transformed write is what the hardware has. `Spu::write16`
  stores the value it is given, and `Spu::read16` returns that, so a game that
  reads a register back gets the transformed value, never the original. A
  dropped write never happened. A redirected write (L/R swap) lands in the other
  register. KON/KOFF read as 0 regardless, as before.
- `noise`, `pmon` and `reverb` keep a shadow of the last value the SPU was given
  for each register, to build the extra writes described above.
- The existing `Spu::corrupt_runtime_state` sound reaper is untouched.

### 2.4 GP0 filter semantics

`Gpu::gp0()` buffers a command, then calls the genome, then the old reaper, then
dispatches. Vertex words are `x` bits 0-10 and `y` bits 16-26 (signed 11 bit);
only those bits are rewritten. Primitives that end up too large are skipped by
the GPU, which is a valid effect. VRAM-transfer data words are out of scope.

### 2.5 Delayed key-ons

`spu_keyon` `delay` and `duplicate` put a write in a sorted queue (at most 512
entries, later ones are dropped) with the due cycle `device_cpu_cycle() + delay
* 768`. `System::sync_spu()` ticks the SPU to each due cycle, calls
`spu_.write16`, and carries on. The SPU is additive, so the result does not
depend on where the sync points fall. I did not add a scheduler event. Late
writes bypass the genome (they are not filtered a second time).

## 3. Test checklist for Lexon

Run from the repo root after `build-ninja.cmd`, with
`set VIBESTATION_BIOS=D:\Misc\pSXfin_1_13-1220\bios\scph1001_original.bin`
(or pass the BIOS path where a `[bios]` argument is shown).

1. **Gene tests**: `build-ninja\VibeStation.exe --grim-gene-test`
   - Expect 96 `GRIM_GENE_TEST PASS` lines and a last line
     `GRIM_GENE_TEST_RESULT PASS failures=0`, exit code 0, in about 70 s. It
     prints a few `INFO` lines and, from the determinism and explore tests, the
     `GRIM_DETERMINISM_PASS` line and two `GRIM_EXPLORE_SUMMARY` lines. One of
     those is meant to say `timeout=1` (the deliberately hung child is killed
     after 3 s).
   - Without a BIOS it skips the end-to-end, recompiler, determinism and explore
     tests.
2. **Phase 1 tests**, unchanged: `--grim-self-test` (17 `GRIM_TEST PASS` lines; Phase 1's note said 18, the 18th PASS is the last line,
   `GRIM_SELF_TEST PASS failures=0`), `--cpu-backend-compare-test`,
   `--gpu-self-test`, `--scheduler-self-test <bios>`. All exit 0.
3. **Stock BIOS is unchanged**: `--grim-eval 1800 stock.jsonl` should still print
   `run_hash=0x432E585CF1535F9C` (the Phase 1 value) and now also
   `genome=0x0000000000000000`.
4. **One genome, headless, with review outputs**:
   `build-ninja\VibeStation.exe --grim-eval 900 out.jsonl --genome docs\grim-reaper\genomes\mix_everything.json --dump-wav mix.wav --dump-frames mixframes 150`
   - Expect `verdict=alive ... genome=0xCEAE5586B9E3907F` and
     `run_hash=0xF7F9283AB21B38B4`, taking about 17 s here. The last line of
     `out.jsonl` has `"gene_hits":[40,14,15,377,8,10198,2129,176,3753]`: how many
     events each of the 9 genes really changed. `mix.wav` is 16-bit stereo,
     44.1 kHz; `mixframes\frame_00000.bmp` ... `frame_00750.bmp` are the
     displayed area.
   - `gene_hits` of 0 for a gene means the game never did what that gene acts on
     (for example, no key-on to drop).
5. **Random genomes**: `--grim-random-genome 1 g.json` prints
   `GRIM_GENOME seed=1 genes=<n> hash=0x8D86B610CE07F95B` (this hash must be the
   same on every compiler). `--grim-random-genome 7 g.json 3` writes exactly 3
   genes.
6. **Exploration**: `build-ninja\VibeStation.exe --grim-explore 1 12 explore_out 900`
   - Each seed prints one `GRIM_EXPLORE seed=.. verdict=.. reason=.. silent=..
     genome=..` line, then a table (also `explore_out\summary.tsv`) and
     `GRIM_EXPLORE_SUMMARY`. Survivors are in `explore_out\survivors\seed_N\`
     (`genome.json`, `telemetry.jsonl`, `audio.wav`, `frames\*.bmp`), hangs and
     host crashes in `explore_out\crashes\`, dead machines' genomes in
     `explore_out\dead\`. In my run of 48 seeds all 48 survived: the generator is
     conservative, so expect mostly survivors. Options: `--timeout S` (default
     300), `--families spu|gpu|both`, `--genes N` (max genes).
7. **Watch and listen in the GUI**, in both CPU modes. Put `--genome` first:
   - `build-ninja\VibeStation.exe --genome docs\grim-reaper\genomes\<file>.json`
   - `build-ninja\VibeStation.exe --cpu recompiler --genome docs\grim-reaper\genomes\<file>.json`
   - The console prints `GRIM: genome ... loaded: N genes, hash=0x...`. Then boot
     the BIOS or a disc as usual. The genome restarts from frame 0 on every boot
     and on every "reap and reboot" (it is rewound in `System::reset()`).
   - I could **not** listen or watch live (no audio device, no display access in
     this session). I checked audio through the WAV and hash telemetry and video
     through dumped frames (I did look at several BMPs), and I started the GUI
     with a genome in both CPU modes for 12 s each: it loaded and stayed up. The
     interpreter/recompiler parity is tested headlessly (see below).
   - What to expect on the stock BIOS (intro logo, then the Shell). The time
     axis is emulated frames, 60 per second:

   | Genome | What it should do |
   |---|---|
   | `audio_pitch_octave_down` | The startup chime and every later note play an octave low (x0.5). |
   | `audio_pitch_drift` | Notes started later come out progressively lower over the first 15 s (only notes whose pitch is written after that time). |
   | `audio_pitch_scale_snap` | Pitches snap to only three semitones (2, 5 and 7 above each sample's own pitch, mask `0x0A4`): the chime should sound wrongly tuned. |
   | `audio_sustain_forever` | Notes do not decay. Expect a droning wash. |
   | `audio_stutter_keyon` | Some key-ons arrive 57 ms late, some notes are late or missing. Few events in the intro (10 in 900 frames), so it is more audible in the Shell and in games. |
   | `audio_stereo_flip_invert` | Left and right are swapped; every 4 s for 2 s the volumes are sign-inverted (hollow, phasey sound on headphones). |
   | `audio_reverb_erosion` | Reverb is forced on and its work area is moved over neighbouring data, ramping in over 25 s: the sound should get progressively muddier. |
   | `audio_wrong_samples` | Sample start addresses are shifted randomly (up to 1.6 KB), ramping in over 10 s: notes play the wrong bits of sample RAM (bleeps, noise). |
   | `video_jitter_rot` | Vertices jitter by up to 14 px, ramping in over 20 s. By ~12 s the menu artwork is visibly shattered (I looked at frame 750). |
   | `video_mosaic_snap` | All geometry snaps to a 16 px grid: the Shell's octagons turn blocky. |
   | `video_color_shuffle` | RGB channels are permuted randomly for 1 s out of every 2 s. |
   | `video_invert_flicker` | 20% of primitives are drawn with inverted colors. |
   | `video_texture_scramble` | CLUT and texpage bits of textured primitives are scrambled, and the E1/E2/E6 state commands get random bits flipped (texture window and draw mode; ramping over 15 s): the menu textures turn into blocks (I looked at frame 750). |
   | `video_drawarea_drift` | The draw area and draw offset (E3-E5) jitter and drift: the whole picture slides and gets cropped by a moving border. |
   | `mix_everything` | 9 genes, SPU and GPU, with rot/window/intermittent triggers. |

   Tell me which of these you find interesting; the parameter names in §2.2 are
   what to tweak. Then try one on a game (`--genome` then the disc as usual).

### Measured cost (genome off)

Lean `--cpu-benchmark`, alternating the Phase 1 exe and the Phase 2 exe in one
session (`VIBESTATION_BENCH_LEAN=1`, same method as Phase 1). The machine drifts
between a fast and a slow state (3.1 s vs 6.7 s for the same interpreter run),
so only medians and same-state pairs mean anything:

| Load | Phase 1 exe | Phase 2 exe | Result |
|---|---|---|---|
| Interpreter, BIOS menu 600+600, 12 pairs | median 6993 ms (min 6357) | median 7024 ms (min 6222) | no change |
| Recompiler, BIOS menu 900+600, 10 pairs | median 1799 ms (min 1623) | median 1803 ms (min 1621) | no change |
| Recompiler, GT2 1500+600, 6 pairs | 5.5 s typical | 5.3 s typical | no change |
| Interpreter, GT2 1500+600, 6 pairs each order | median 9314 ms (new first) / 8388 ms (baseline first) | median 9090 ms (new first) / 5901 ms (baseline first) | no change; an earlier batch with the baseline first looked 3-12% slower for the new exe in 4 of 4 pairs, and it reversed when the new exe ran first: that was order/machine state |

The cost of the hooks when off is a predictable `nullptr` test per SPU register
write, per GP0 command, per polyline word, per `sync_spu` and per frame.

### Parity checks

- GT2 replay, 2000 frames (`VIBESTATION_STATE_HASH_*`, same command as
  `AGENTS.md` §6, limited to 2000 frames): Phase 1 exe and Phase 2 exe give
  identical `BOOT_STATE_HASH` lines (all fields, 2000 of 2000 frames) for the
  interpreter and for the recompiler.
- With a genome, interpreter vs recompiler (`e2e_genes_match_between_interpreter_and_recompiler`):
  9-gene genome, 400 frames of the stock BIOS. `fb_hash`, `audio_hash`, cycles,
  GP0 counters, key-on counts, active voices and every gene's hit count match in
  every frame (3206 hits). This runs through `run_grim_eval` with
  `native_cpu = true`, which skips the executed-PC hooks.

## 4. Known issues and untested areas

New in Phase 2:

- **Genes act on writes and commands, not on state.** A `spu_pitch` gene only
  changes a note when the game writes that voice's pitch register. A note held
  from before the gene's window keeps its old pitch. `spu_noise`, `spu_pmon` and
  `spu_reverb` (`eon`) piggyback on key-ons for this reason. Timing-based
  effects are therefore coarse in the BIOS intro, which writes few registers.
- **Genome state is not part of save states or rewind.** Loading a state
  restores `frame_counter` but not the gene RNG positions or the delay queue, so
  playback after a load is not reproducible. Boots and resets are.
- **Not listened to.** Audio effects are verified by hash and WAV difference and
  by unit tests on the register values, not by ear.
- **POSIX `grim_process.cpp` was not compiled here.** It is written from the API
  and should be built by the Linux CI. `--grim-explore` was run on Windows only.
- **Headless evaluation is interpreter-only** (as in Phase 1, for the coverage
  telemetry). `native_cpu` exists for parity checks. Explored genomes run under
  the interpreter; the GUI works in both modes.
- **`--genome` belongs to a `--grim-*` mode when one appears earlier on the
  command line;** otherwise it is the GUI flag. `--boot-disc-test`,
  `--frame-test` and `--cpu-benchmark` accept the flag on the command line but
  ignore it (open question 1).
- A host crash inside `--grim-eval` is only caught by `--grim-explore`'s child
  process. `--grim-determinism-test` still waits without a timeout.
- Random genomes are conservative: 48 of 48 survived 900 frames. Harsher
  settings (or different weighting) will find more dead machines; that is a
  tuning question for Phase 5.
- BMP frame dumps use the interpreter's presentation path
  (`build_display_rgba`, no interlace handling beyond what it does).
- The delay queue is capped at 512 entries and silently drops later ones.

Still open from Phase 1: see Appendix A (watchdog only between frames, threads
share diagnostic globals, interlaced display hash, redraw loops count as alive,
only SCPH-1001 tested, coverage only under the interpreter).

## 5. Open questions

1. **Should `--genome` also apply to `--boot-disc-test` / `--frame-test` /
   `--cpu-benchmark`?** It is a two-line change per mode (they create their own
   `System`). Not needed for Phase 2, but it would let CI compare backends with
   genomes on real games. Tell me if you want it.
2. **Sanity of random-genome survival.** All 48 explored seeds survived. Do you
   want the generator biased toward harsher settings by default, with a
   `--survivable` switch for the current behaviour?
3. **Save states.** Should genome state be serialized with snapshots (so rewind
   and load-state replay a genome exactly)? It touches the snapshot format.

## 6. Next step: proposed Phase 3 brief (clean-boot trace, ROM code genes)

Goal: let VibeStation map the stock BIOS by itself (DESIGN.md Layer 1, §1.1-1.2)
and add the first ROM genes (DESIGN.md §2.1, structural MIPS code genes), so a
genome can describe "the machine was born broken" and not only "the interface is
broken".

- **Clean-boot trace** (interpreter only, discovery runs only). Extend
  `GrimTelemetry` (its exec bitmap already exists) into a `GrimBootMap` with, per
  ROM word (BIOS image, 512 KiB): *executed* (fetched, directly or after being
  copied to RAM), *read as data*, *first-touch cycle*, and a *consumer* class
  (SPU RAM, VRAM/GP0, GTE, CPU only). Sources: `Cpu::run_slice`'s telemetry loop
  (fetches, already hooked), `Cpu::load8/16/32` for data reads, DMA channel
  hooks in `dma_block`/`dma_linked_list` for SPU/GPU sinks, and `Spu`/`Gpu`
  entry points. Keep it out of `Cpu::step()` (one branch there cost +2%): use the
  telemetry loop, and add a separate load hook the same way.
- **Provenance for code copied to RAM.** The kernel and shell run from RAM. Keep
  a shadow array of RAM words (2 MiB / 4 = 512K entries, heap-allocated, not a
  member array; see `AGENTS.md` §9) holding "ROM offset this word came from".
  Propagate through: word loads from ROM (tag the destination register in a
  32-entry register-tag array), stores (tag the RAM word from the register), and
  DMA (copy tags). ALU ops: clear the tag, or keep the first operand's, as in
  DESIGN §1.2. Executed RAM words then map back to ROM offsets. The existing
  `RamWordWriteProvenance` vector in `System` is unrelated (write provenance
  diagnostics) but shows where the store hook lives.
- **Outputs.** A JSON/binary map keyed by the ROM hash: region list with class
  (`code`, `data`, `unused`, `unknown`), first-touch time and consumer. A CLI
  `--grim-map <bios> <frames> <out>` and a check that the map reproduces across
  runs (same determinism test style as Phase 1).
- **Structural code genes** (`src/core/grim_genome` gets a ROM gene family; the
  genome JSON version stays 1 with new `type` values, or bumps to 2 if the target
  needs a region reference): operate on words the map labels `code`, weighted by
  first-touch time (later touch = safer). Start with the cheap, always-valid
  ones from DESIGN §2.1: immediate perturbation, register substitution (avoiding
  `$sp/$ra/$k0/$k1`), branch inversion (`BEQ<->BNE`, `BLEZ<->BGTZ`, REGIMM
  `BLTZ<->BGEZ`), branch offset nudge, ALU substitution sets, load/store width
  change (alignment kept), selective NOP. Invariant: every mutated word still
  decodes. Apply through `Bios::patch32()` after `reset()` (the Phase 1 hook);
  a ROM gene is a list of `(offset, word)` pairs plus its generating parameters.
- **Tests.** Map determinism; every mutated word decodes (fuzz over the map);
  a NOP-all-init-code gene classifies as `coverage_stall`; a late-code
  perturbation stays alive; ROM genes and interface genes compose in one genome
  and keep the genome hash stable.
- **Out of scope for Phase 3:** ADPCM scanner and sample genes (Phase 4),
  fingerprints and archive (Phase 5), ROM geometry/texture genes and the
  provenance-driven asset scanners (Phase 6).

## Appendix A — Phase 1 reference (unchanged)

The text below is the Phase 1 record. Its instructions still apply.

### A1. Phase 1 status


**Phase 1 (deterministic headless runner, telemetry, liveness gates) is done.**

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

### A2. What changed in Phase 1


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

### A3. Phase 1 test checklist


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

### A4. Phase 1 known issues and untested areas


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

### A5. Answered questions (2026-09-30)


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
