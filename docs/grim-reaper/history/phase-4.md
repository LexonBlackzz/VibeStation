# Grim Reaper 2.0 — Phase 4 record

Moved out of PROGRESS.md unchanged; it was an appendix there. Appendix references map to
the files in this folder: A = phase-1.md, B = phase-2.md, C = phase-3.md, D = phase-4.md,
E = phase-4.1.md. Current status is in ../PROGRESS.md.

# Appendix D — Phase 4 reference

The following record describes Phase 4 as delivered. Its hash-change counts and
listening predictions are historical; Phase 4.1 above records Lexon's verdicts
and replaces hash change with a calibrated audibility gate.

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
