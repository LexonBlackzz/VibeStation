# Grim Reaper 2.0 — Phase 3 record

Moved out of PROGRESS.md unchanged; it was an appendix there. Appendix references map to
the files in this folder: A = phase-1.md, B = phase-2.md, C = phase-3.md, D = phase-4.md,
E = phase-4.1.md. Current status is in ../PROGRESS.md.

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
