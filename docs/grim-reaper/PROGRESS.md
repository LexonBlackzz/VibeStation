# Grim Reaper 2.0 — progress log

Read this first in every new session, then `DESIGN.md`, then
`PROJECT_BRIEF.md` (the code map). The conventions file is the repo-root
`AGENTS.md`. It is gitignored and local to Lexon's machine.

## FMV corruption (2026-10-07)

Movies corrupted inside the MDEC as they decode (`src/core/grim_fmv.{h,cpp}`, one engine for both
front ends; the MDEC calls it through `GrimFmvHook`, which `System::FmvDispatch` implements).

- **Targets** (bit mask): 1 coefficient levels, 2 quantisation tables (as uploaded; uses strength
  only), 4 whole 8x8 blocks (swap, copy, flatten, invert, roll), 8 output pixels, 16 the movie's
  XA audio (per decoded sector: stutter, skip back into the previous sector, reverse, crush,
  dropout, blast, static). Runs, the 0xFE00 end code, output sizes and sample counts are never
  changed, so a corrupted movie never stalls the game. Audio is only touched while the MDEC has
  decoded a macroblock in the last 30 frames (`System::grim_fmv_xa_audio`), so game music and
  voices outside movies stay clean.
- **Classic:** "FMV Reaper" style (macroblock share, strength, targets, seed). Off in
  `stop_all_corruption()`.
- **Gene `mdec_fmv`** (genome v1): target = the mask above, params `rate` (permille of
  macroblocks, default 150) and `strength` (0..1024, default 256), both scaled by the trigger.
- **FMV family** (green, bit 32) in New Corruption. `GrimRandomParams::fmv` is off by default, so
  every earlier seed of `grim_random_genome` draws as before; pulls with the FMV family on differ.
- Tests: `fmv_*` in `--grim-gene-test`, `pull_fmv_family_makes_fmv_genes` in `--grim-pull-test`.
  Checked on A-Train's intro with `--genome` + `--frame-test`: broken picture, same frame timing.

## Follow-up after the v0.6.0 merge (2026-10-06)

`grim-reaper/phase-5.1` now contains main (v0.6.0: VibeStation 2 / PS2 lab, new launcher). The
Grim Reaper 2.0 page stays on the VibeStation 1 menu only; VS2 keeps its own placeholder.
Older phase records moved to `history/` (Appendix A-E = phase-1 .. phase-4.1).

- **Families are domains.** Audio (sound bank samples + SPU filters), Visual (GP0 filters; ROM
  visual genes join in Phase 6), Code, Hardware. The old Interface toggle is gone (its genes are
  split by what they break; the line still says IFACE). Visual works today.
- **Clean machine.** A pull ends at any other boot, Stop, BIOS change or Classic reaper
  (`App::stop_all_corruption()`), and the panel has a Clean machine button. Before, a game
  booted from the menu ran on the corrupted BIOS with the genes and death watch still attached.
  To corrupt a game: load it, then pull.
- **Share codes.** Copy code gives a `VSGRIM1:` code (zlib + base64url, about half the JSON);
  Paste code boots one (plain JSON also accepted). Decoding is capped at 4 MB.
- **Culprit in RAM.** An exception loop in a RAM copy of patched BIOS code now names the gene
  (the word and its neighbours must match the patched image). Before, only ROM addresses did.
- **Fixes:** Mercy stopped rerolling for good after 25 rerolls in a session (counter never
  reset); trigger times in the gene list assumed 60 fps (now the machine's real rate, so PAL is
  right); Rot label now says it covers every gene except BIOS patches.
- **Hardware survival bias.** Maps record which RAM ran code (`ram_exec`, optional, old maps keep
  their hash; the panel re-maps a BIOS once if its cached map lacks it). Non-critical RAM fault
  genes avoid it with chance 100% up to intensity 20, falling to 0% at 100. Generated RAM genes
  on code: 0% at intensity 10, 21% at 100.
- **CI:** Linux runs `--grim-gene-test`, `--grim-audibility-test --unit-only` and
  `--grim-pull-test` without a BIOS (51 checks).

Second pass the same day (looked at on screen this time, via `--open-grim-reaper`):

- **Panel redesign.** Machine card first (alive: ID, time, family dots; dead: cause, detail,
  likely culprit with "Revive without it"; idle: hint). New Corruption with Revive beside it, then
  Keep / Clean / Copy code / Paste code as real buttons. Colours come from the user's theme (ImGui
  style; the primary button follows the theme accent); only family and alive/dead colours are fixed.
  The Recipe (intensity with risk-band track, families, Rot and Mercy switches) folds away while a
  machine runs. History chips show machine ID and lifetime. Back button kept (host page).
- **Per-gene switches.** The machine keeps its whole genome plus a mask. Revive boots the genes
  that are on; a changed mask becomes a new pull whose note says what it came from
  ("#4BC4 without 1 gene"). The dead card's culprit button switches that gene off and revives.
- **Classic engines** (`src/core/grim_classic.*`): Random, Add, Subtract, Replace X with Y, Shift
  left/right, Rotate left/right, XOR, AND, OR, Invert, Set, for the BIOS (single and batch) and RAM
  reapers. BIOS hits land by random strike (as before) or every Nth byte. The Random engine with a
  random strike draws exactly like the old reaper, so old seeds and presets reproduce. Presets save
  `engine`, `engine_value`, `engine_match`, `every`. The custom hex range is shown inline.

Third pass: Classic tricks and the Disc Reaper.

- **Bug fixed:** the RAM reaper's engine never reached the emulator thread (its settings
  cross threads as atomics and the new field had none), so it always ran Random. Covered now by
  `ram_reaper_config_keeps_engine_and_tricks`.
- **Pipe engine** (all byte reapers): the hit byte becomes the byte `offset` bytes away.
- **RAM reaper tricks:** every Nth byte (fixed positions, min 8; strength = chance per hit),
  one pass every N frames (bursts), Freeze (the last N hit cells are rewritten every frame,
  so the game cannot repair them; cleared on reset or when the reaper is switched off).
- **Auto-corrupt** (BIOS styles): after Corrupt & (Re)Start, corrupt again with a fresh seed and
  reboot every N seconds. Any other boot stops it.
- **Disc Reaper** (Classic style 6, `src/core/grim_disc.*`, hooked in
  `CdRom::read_raw_sector_for_lba`, the one function every data, XA and CD-audio read goes
  through). Hits depend only on seed and sector, so a sector always comes back the same way, like
  a damaged image. The ISO9660 file system (volume descriptors, path tables, every directory,
  SYSTEM.CNF) is never touched; the boot executable named by SYSTEM.CNF is protected unless
  targeted. Targets: game files (Form 1), movies and XA audio (Form 2), CD audio tracks, boot
  program. Sync, header and subheader stay intact. "Start after N s" leaves early reads clean.
  `--disc-reaper pct=..,targets=..,seed=..,engine=..` sets it from the command line (headless).
- Targets after measuring: **Movies** (STR frames found by their 0x0160/0x8001 sector header, in
  Form 1 or 2; the 32-byte sector header and the frame's 8-byte bitstream header are kept),
  **XA audio** (Form 2 + audio bit), **CD audio**, **Game files** (Form 1, with "skip code and
  packed data": >=45% of words are common MIPS opcodes, or entropy >7.6 bits/byte), **Boot
  program**. Default: Movies + XA.

Disc bootability (`--grim-eval ... --disc <cue> --live-gates`, 3600 frames, no input, 12 games;
clean = 10/12 alive, R4 and THPS2 trip the inert gate on a long silent screen even clean, at the
same frame in every run, so the ceiling is 10/12):

| Setting | Alive |
|---|---:|
| Game files + Form 2, 0.05% (~1 byte/sector), no filter | 1/12 |
| Game files + Form 2 + CD audio, 0.5% | 1/12 |
| Game files only, 0.005% | 6/12 |
| Game files, 0.05%, skip code/packed | 4/12 |
| Game files, 0.5%, skip code/packed | 3/12 |
| Movies only, 0.5% | 10/12 |
| XA audio only, 0.5% | 10/12 |

Game data on PS1 is mostly code overlays and compressed archives, so game-file corruption kills
early (17-25 s, at the first loads); movies and XA are as safe as a clean disc. Movie corruption
looks different per game: Bloody Roar's player keeps drawing blocky garbage; Silent Hill and
Breath of Fire IV freeze on the first broken frame (the game stays alive). Open question: is that
the games' decoders or our MDEC being stricter than hardware? Compare with another emulator.
Clean runs give the same run_hash as before these changes (Crash, 1800 frames).

Yield (`--grim-pull-yield`, 40 pulls per level, 900 frames, no disc, SCPH-1001, new map):

| Families | Intensity | Alive | Dead | Survived+audible | Survived+inaudible |
|---|---:|---:|---:|---:|---:|
| Hardware only | 20 | 40 | 0 | 1 | 39 |
| Hardware only | 50 | 36 | 4 | 3 | 33 |
| Hardware only | 80 | 38 | 2 | 11 | 27 |
| All four | 20 | 37 | 3 | 17 | 20 |
| All four | 50 | 33 | 7 | 27 | 6 |
| All four | 80 | 30 | 10 | 15 | 15 |

Hardware deaths at intensity 20 went from 8/40 (§0.8) to 0/40. Hardware deaths still do not
climb with intensity (lethal mostly differs by more cells, and critical memory stays at 1%);
raising `hw_critical_permille` with intensity would be the lever, a design call. The full mix
climbs 7.5% -> 17.5% -> 25%. Small samples, read the trend.

Verified: stock `run_hash=0x432E585CF1535F9C` unchanged; CPU differential, GPU, scheduler, Grim
self/gene/map/sample tests and `--grim-pull-test` on both backends pass, with identical live
verdicts apart from the known slower recompiler exception-loop detection. No `src/core` file
outside `grim_*` changed in this follow-up, and main's merge changed none, so no GT2 replay was
rerun. The panel has still not been looked at on screen by the agent.

## 0. Phase 5 status (2026-10-03)

**Phase 5 (live random New Corruption) is implemented on `grim-reaper/phase-5`, from
Phase 4.1 plus main.** Phase 4.1 is kept in `history/phase-4.1.md` (its §N numbering is unchanged).
The panel mockups are the Design canvas linked from `DESIGN.md`.

- [x] Pull generator: families, intensity with risk, rot mode, recent-pull novelty
- [x] Live death watch with readable cause of death; no per-instruction hook
- [x] Mercy (off by default), Keep, Revive, History, genome list, Kept list
- [x] Library persisted atomically; damaged file recovered
- [x] Panel in the definitive Grim Reaper page; 1.0 reapers kept as the "Classic reapers" tab
- [x] Per-BIOS boot map made once, in a child process
- [x] Tests (`--grim-pull-test`, both backends) and yield (`--grim-pull-yield`)
- [x] Stock hash, older suites, GT2 parity (see §0.6)
- [ ] The panel has **not been looked at on screen by the agent**: only its code paths ran
      (the GUI path was driven by a temporary environment hook, since removed). Lexon's eyes first.

### 0.1 What a pull is

New Corruption draws a fresh seed, generates a genome from the enabled families at the
intensity, resets the machine (which replays ROM genes and rewinds interface genes) and runs
it live. Nothing is evaluated or filtered first. Families are *gene sources*: **Audio** =
ADPCM sample genes, **Code** = ROM code genes (needs the boot map), **Interface** = SPU/GPU
runtime filters, **Visual** = reserved for the Phase 6 ROM visual genes (the button is
disabled and generates nothing). Every gene still carries its domain colour in the list.

Intensity (0-100) sets the total gene count (`1+6t/100 .. 2+9t/100`, e.g. 58 -> 4-7), the
interface risk (magnitudes pulled toward the full parameter range; it draws no extra random
numbers, so older seeds are unchanged), and the code survival biases: words first run before
`600 - 6t` ms are avoided, late-code preference falls from cubic to uniform, patches per gene
2-8, `call_swap` only from 90. Sample windows are `100 + 4t` ms. Labels: safe <20, mild <45,
risky <70, lethal. Rot mode turns every interface gene into a ramp that starts healthy (2-7 s)
and decays; it cannot rot ROM genes, and the panel says so.

Novelty: the generator builds up to four candidates from the seed and keeps the one that
overlaps least with the last eight pulls (gene type/kind/target keys). It never decides
whether a pull is shown.

### 0.2 Death watch

`GrimLiveWatch` runs the Phase 1 gates on the live machine, once per frame, on the emulator
thread. It reads what the emulator already counts (GP0/DMA/SPU work, displayed-image hash,
exceptions) and the audio through `Spu::set_audio_tap`. Thresholds (frames, ~60/s): inert 150,
frozen picture 240, exception loop 45, COP0-sampled stuck exception 100 (a running game
disc doubles the first two and the last). Measured on a clean no-disc boot: the longest idle
run is 72 frames and the longest frozen run 81, so the gates sit well above a healthy boot.

Causes of death are plain words plus detail: "Stuck in an exception loop - Reserved-instruction
exception at BFC0 2B68, repeating with no progress", "Machine went inert", "Frozen picture",
"Black screen, no sound"; an exception loop on a word a ROM gene patched also names that gene.

The recompiler takes exceptions natively, so its exception loops are caught by the COP0
Cause/EPC sample instead: **same reason, slower** (0.75 s interpreter, 1.7 s recompiler in the
test). Both backends give identical verdicts and death frames for the hang, the clean boot and
the six generated pulls (the `GRIM_PULL_LIVE` lines of the two `--grim-pull-test` runs diff clean
apart from that one time).

**Host hangs.** A corrupted GPU linked list that points back at itself used to replay a million
packets inside one DMA, minutes of host time: the emulator thread never reached the watch and
the UI would deadlock in `pause_and_wait_idle()`. Two of the first 120 yield pulls did this.
`dma_linked_list` now stops at the first revisited node (Brent cycle detection; RAM cannot change
during the atomic transfer, so a revisit means the list never ends). Real lists never revisit
(GT2 parity below). Regression test `dma_linked_list_loop_does_not_hang_the_host`. This is the
only change to a normal-play path besides the audio-tap null check (§0.6).

### 0.3 Library, Mercy, data

`GrimLibrary` keeps every pull (200 un-kept at most; kept pulls, dead ones included, are never
trimmed) in `grim_data/library.json` beside the executable, written through a temp file and
renamed; a damaged file is moved to `.corrupt`. Mercy (off) rerolls a death inside 4 s and
marks the replaced pull so History hides it; the stats count it. The boot map for a BIOS is
made once by a child process (`--grim-map`, interpreter, about 30 s) the first time the page is
opened and cached by ROM hash under `grim_data/maps/`. Until then Audio and Interface work and
Code stays off with a status line.

### 0.4 Tests

`--grim-pull-test <bios> [--backend interpreter|recompiler]` (run it once per backend): plan
bounds and monotonicity, determinism, family toggles (every mask), unavailable families, risk
bounds, round trips, rot, novelty, early-init bias, Mercy, death text, library (round trip,
trimming, corruption recovery, atomic save), the DMA loop, and live machines (clean boot alive,
hang, exception loop, recovery after a dead pull, wrong-BIOS refusal, six generated pulls).
All pass on both backends.

### 0.5 Yield (live/dead and audible, reported separately)

`--grim-pull-yield`: 40 pulls per intensity, families Audio+Code+Interface, 900 frames, no-disc
SCPH-1001, each pull in its own child process with the live gates. Audible = the Phase 4.1
residual/exposure metric against the clean boot (provisional, as before).

Cells are counts of 40 pulls; "alive" splits into survived+audible / survived+inaudible; no pull
was silent, errored or hung the host after the DMA fix.

| Intensity | Mean genes | Alive | Dead | Survived+audible | Survived+inaudible | Death causes |
|---:|---:|---:|---:|---:|---:|---|
| 20 (safe) | 2.6 | 39 | 1 | 31 | 8 | exception loop 1 |
| 50 (risky) | 5.0 | 31 | 9 | 28 | 3 | exception loop 6, inert 3 |
| 80 (lethal) | 6.9 | 24 | 16 | 13 | 11 | exception loop 12, inert 4 |

All 120: **94 alive / 26 dead (78% / 22%)**; of the 94 survivors **72 audibly changed** (77%, 31/39 at
the safe end). Death climbs with intensity as intended. Before the DMA fix, 2 of these same
pulls hung the host (both lethal-intensity code genes); they now die or live in seconds.
The metric is provisional (four human labels), so "audible" is a measured proxy, not a listening result.

Hash change is not used anywhere. Dead pulls are not scored for audibility.

### 0.6 Verification

- Stock no-genome 1800 frames: `run_hash=0x432E585CF1535F9C`, unchanged.
- Passing: CPU differential, GPU, scheduler (13 checks), Grim self, gene, map, sample (600 frames,
  3 seeds), audibility (4 labels), pull test on both backends.
- Full GT2 regression replay, 5738/5738 frames, **every BOOT_STATE_HASH field identical** between the
  pre-Phase-5 build (phase-4.1 + main) and this build, separately for the interpreter and the recompiler.
  That covers the DMA cycle detection, the audio-tap check and the light-telemetry branch.

### 0.7 Not done / limits

- Panel not seen on screen by the agent. No thumbnails, audio clip, curse readout, "More like this"
  or per-gene toggles (Phase 8). Copy code copies the genome JSON.
- Visual family generates nothing until Phase 6. Code genes need the map; mapping runs on first use.
- Death thresholds come from one BIOS' clean boot; a game disc doubles them, nothing more. A static
  loading screen with silence in some game could still read as "frozen".
- Only MSVC/Windows built. Alternating pinned benchmarks were not rerun (the watch is off in normal
  play; the normal-path costs are one null check per produced audio block and a few arithmetic ops
  per linked-list packet).
- A pull that hangs the host for a reason other than the DMA loop would still freeze the emulator
  thread. A UI-side stall detector would be the next step.

### 0.8 Faulty Hardware Simulator (Phase 5.1, batch 1)

Branch `grim-reaper/phase-5.1`, from Phase 5. A fifth gene family, **Hardware** ("HW", purple),
that fails the machine itself. Design: `DESIGN.md` §2.10. Batch 1 covers main RAM, VRAM and
sound RAM; CD, controller, memory card and clock come next.

- Three gene types: `hw_ram`, `hw_vram`, `hw_spuram` (genome v1, no map needed, 8 params each).
  Fault kinds: stuck-high/low bits, flaky cells, bursts, bad column (same bit at a stride),
  thermal decay (rows nobody rewrites drift toward 0 or FF), rowhammer (rows rewritten every
  frame damage neighbours), dead VRAM line, and a `load` sensitivity that scales a gene's rate
  with the DMA traffic of the previous frame (idle = stable).
- Applied **only at scheduler points** (frame start; stuck cells also every eighth scanline) by
  `GrimGenomeRuntime::apply_hardware` through a small `GrimHwTarget` interface that `System`
  implements. Nothing sits on the memory access path, so the recompiler's direct RAM access is
  untouched; with no hardware genes the cost is one pointer test per eighth scanline.
- Rot (the default trigger) activates cells one after another, so a machine wears in.
- **Critical memory** (kernel low 64 KB, top 16 KB for stacks, first 4 KB of sound RAM) is
  avoided unless a gene is generated as critical: 1.0% of hardware genes (measured 1.0-1.5%
  over 4000), shown as DANGEROUS in the list; `hw_critical_permille = 0` switches it off.
  Over 4000 non-critical genes applied for 60 frames: 0 stray writes into those zones.
- Panel: a fifth family button; hardware genes show title, cells, address range, trigger,
  "bus-sensitive" and DANGEROUS. Pull settings include Hardware by default.
- Tests (`--grim-pull-test`, all pass on both backends): every fault kind on a fake target,
  rewrite-and-hold, scanline vs frame ticks, rot wear-in, closed window, bus-load ratio,
  determinism, critical rate and zero strays, round trips, all 16 fault kinds generated, the
  Hardware family in pulls, and six live-machine cases that must take effect on a real BIOS.
- Yield, hardware family only (40 pulls per level, 900 frames, no disc, live gates):

| Intensity | Alive | Dead (all exception loops) | Survived+audible | Survived+inaudible |
|---:|---:|---:|---:|---:|
| 20 | 32 | 8 | 1 | 31 |
| 50 | 34 | 6 | 7 | 27 |
| 80 | 34 | 6 | 13 | 21 |

  83% of hardware pulls live. Death does **not** climb with intensity here (20/15/15%): a
  single bad bit in running Shell code is enough, and cell counts matter less than where they
  land. "Audible" only measures sound; most of these faults are visual or silent, and there
  is no visual metric yet. Small samples: read the trend, not the digits.
- Verification: stock hash `0x432E585CF1535F9C`, all older suites, and full GT2 replay
  (5738 frames, every BOOT_STATE_HASH field) identical to the Phase 5 baseline on both backends.

**Backend drift found and fixed.** Under the `hw_rowhammer_ram` live case the two CPU backends drifted
by 2 cycles at frame 212, right after a hammer write corrupted running Shell code. It looked like an I-cache
problem; it was not. The corrupted code contained encodings real programs never use, and the recompiler had
two bugs on them (reproduced in the CPU compare harness, no I-cache or RAM change needed):
1. REGIMM branch-likely forms (opcode 1 with the unused "likely" rt bit) cost 1 cycle when taken in the
   block-head emitter; the Interpreter charges 2 like any taken BcondZ (only opcodes 0x14-0x17 are 1 cycle).
2. SWC0 of a plain COP0 register: `emit_read_cop0` used RAX as its table pointer, and RAX holds the store
   address, so the store went to a garbage address.
Both fixed in `cpu_recompiler.cpp`; new compare cases cover them (odd REGIMM loops, SWC0 value/address, SW vs
SWC0 first in a block) plus four guards for code changed behind the CPU (stale cached line, refill after
alias eviction, change before first fetch, mid-line change), which already passed. All six live hardware
cases now give identical RAM and hit counts on both backends. Full GT2 replay still matches the baseline
on every field for both backends. The step trace also gained `SCHED_STEP_STALE` lines (I-cache word differs
from memory), and `--genome file.json --frame-test` runs a genome headless on either backend.

**Not done:** faults that need an access-path hook (stuck address lines, row aliasing,
read-destructive and write-stutter faults); CD/controller/memory card/clock genes; a visual
interest metric for VRAM faults; the "Bad modchip" as a gene.
