# Grim Reaper 2.0 — Design Document

## Goal

The original Grim Reaper corrupts raw bytes in BIOS/RAM/VRAM/SPU memory. Most BIOS corruptions collapse into the same few outcomes: black screen, stuck chime, single bass note, exception loop, instant crash.

Grim Reaper 2.0 changes the question from:

> "How much can I corrupt?"

to:

> "How strangely can I break this PlayStation?"

**Mostly alive, sometimes dead, rarely boring.** Every press of **New Corruption** is a pull: nobody, including the system, knows the outcome until the machine boots. Most pulls should give a different defective PS1: a broken voice, a melted logo, a rotting sound bank, a confused GPU. Some will die, and that's part of the fun. Duds are what make an insane pull feel insane. The spirit is the Vinesauce corruptor: press the button, watch, react, press again.

What 2.0 fixes is not that machines die. It's that 1.0's machines all died *the same way* and most survivors were *the same kind of broken*. Typed genes make the survivors varied. Fast death detection and a readable cause of death make the duds quick and a little entertaining, instead of tedious.

The system has four layers:

1. **Discovery** — learn what every part of the BIOS actually *is* (code, samples, geometry, fonts, tables) by watching the emulator run it.
2. **Genes** — typed mutation operators that break each kind of thing in a way that fits it.
3. **Evaluation** — deterministic headless runs that measure whether a machine is alive and what kind of broken it is.
4. **Search (optional, later)** — a quality-diversity archive (MAP-Elites) that collects survivors that behave differently from each other. The core experience does not depend on it (see Layer 4).

---

## Layer 1 — Discovery: letting VibeStation map the BIOS

We don't write a hand-crafted BIOS parser. The emulator already sees every fetch, load, DMA and register write, so it can build the map itself. Discovery runs once per BIOS (keyed by the ROM hash) and the result is cached.

### 1.1 Clean-boot trace

Boot the unmodified BIOS headless in each evaluation scenario (see 3.1) and record, per ROM word:

| Recorded | Meaning |
|---|---|
| **Executed** | Fetched as an instruction (directly from ROM, or from RAM after being copied — see provenance) |
| **Read as data** | Loaded by the CPU or sent by DMA, but never executed |
| **First-touch time** | Emulated cycle count when the word was first used |
| **Consumer** | Which subsystem the data ended up in: SPU RAM, VRAM, GPU FIFO, GTE registers, CPU only |

First-touch time matters a lot: code used in early init almost always dies when mutated, while code used late (shell, GUI) usually survives. Mutation weight can be driven by this value, which replaces any fixed "protect the first N KB" rule.

### 1.2 Provenance tracking

The kernel and shell are copied from ROM into RAM and run from there, so raw PC traces show RAM addresses. During discovery runs only (not normal play), keep a shadow tag per RAM word holding "came from ROM offset X":

- Loads from ROM tag the destination register with the ROM offset.
- Stores write the register's tag into the RAM word's shadow tag.
- ALU operations clear or merge tags (keep it simple: clear, or keep the tag of the first operand).
- DMA transfers copy tags along with data.

Now, when the SPU receives sample data, the GPU receives a vertex, or the CPU executes a RAM instruction, we know which ROM bytes it came from. This is what makes "corrupt the logo geometry *inside the BIOS*" possible.

The discovery pass is allowed to be slow. It runs once per BIOS.

### 1.3 Asset scanners

Heuristics that find candidate regions, confirmed by the trace:

- **SPU ADPCM samples.** 16-byte blocks: byte 0 = shift (low nibble) + filter (bits 4–6), byte 1 = loop flags, 14 bytes = 28 samples. Candidates are runs of aligned blocks with filter ≤ 4, shift ≤ 12, plausible flags, ending in a loop-end flag. Confirmed by tracing what reaches SPU RAM.
- **GPU geometry and command data.** Anything whose provenance lands in GP0 parameters: vertices, colors, UVs, command words.
- **Textures, CLUTs and fonts.** Data whose provenance lands in VRAM through GP0 image transfers or DMA.
- **Tables and constants.** Data read by the CPU that never reaches a peripheral: jump tables, init values, timing constants, strings.

### 1.4 Probe mapping (differential perturbation)

For regions the trace can't explain well, probe them: corrupt one small region, run briefly, and diff the telemetry against a clean run. If the audio changes and nothing else does, it's audio-relevant. If only frame N's pixels change, it feeds the visuals at that moment. If the machine dies, it's critical.

The result is an **effect map**: ROM region → which outputs it influences → how fatal it is. This also serves as the mutation weighting table.

### 1.5 Optional hand annotations

A small per-BIOS annotation file (keyed by ROM hash) can name regions ("SCE intro diamond vertices", "boot chime sample 2") once they've been identified. It is not required, since discovery works without it, but it makes the UI and debugging much nicer.

### 1.6 Region classes

The final BIOS map labels every region as one of:

`code` · `spu_sample` · `gpu_geometry` · `gpu_command` · `texture` · `clut` · `font` · `table` · `string` · `unused` · `unknown`

Each class gets its own gene family.

---

## Layer 2 — Genes: typed mutation operators

A corruption is a **genome**: an ordered list of genes. Each gene has a type, a target, parameters, and optionally a trigger time. Genes come in two families:

- **ROM genes** modify the BIOS image (persistent, "the machine was born broken").
- **Interface genes** modify what passes between CPU and hardware at runtime (GPU FIFO, SPU registers, GTE, DMA, timers). They need no knowledge of the BIOS and also work in games.

### 2.1 Code genes (structural MIPS)

Operate only on regions labeled `code`, weighted by first-touch time and effect map.

- Immediate perturbation: small ± deltas, sign flip, bit flip within the immediate field
- Register substitution: change `rs`/`rt`/`rd` to another register, avoiding `$sp`, `$ra`, `$k0`, `$k1` by default
- Branch inversion: `BEQ ↔ BNE`, `BLEZ ↔ BGTZ`, and `BLTZ ↔ BGEZ` (the last pair is encoded in the `rt` field of the REGIMM opcode)
- Branch target nudge: offset ± small
- ALU substitution within compatible sets: `ADDU ↔ SUBU ↔ XOR ↔ OR`, `SLL ↔ SRL ↔ SRA`
- Load/store width change: `LW ↔ LH ↔ LB`, `SW ↔ SH ↔ SB` (keep alignment valid)
- Load/store offset perturbation
- Selective NOP
- Duplicate or swap adjacent instructions (respect delay slots)
- Constant perturbation in `LUI`/`ORI` pairs (mutate the assembled constant, then re-encode both halves)

Invariant: every mutated word decodes to a valid instruction unless the gene is explicitly an "illegal instruction" gene.

### 2.2 Audio sample genes (`spu_sample`)

These operate on the ADPCM data itself.

- **Filter swap.** Change a block's filter index (0–4). This changes timbre and pushes predictor error through the rest of the sample.
- **Shift change.** Change the shift nibble, giving gain jumps, crushing or clipping. Values 13–15 behave unusually on hardware, so their result depends on VibeStation's implementation.
- **Loop flag edits.** Move loop start, remove loop end (sample runs into whatever follows), add early loop end (stutter).
- **Block shuffle / repeat / reverse order.** ADPCM prediction makes reversed block order sound nothing like reversed audio.
- **Block transplant.** Copy blocks from one BIOS sample into another.
- **Nibble noise.** Perturb data nibbles while leaving headers intact, for controlled grit.

### 2.3 Audio playback genes (interface: SPU registers)

These intercept writes to SPU registers with a deterministic transform.

- **Pitch:** multiply, offset, quantize to a scale, or add a slow wobble over time
- **ADSR:** mangle attack/decay/sustain/release (e.g. infinite sustain, instant release)
- **Volume:** swap L/R, invert, clamp, sweep
- **Start and repeat address:** offset into neighboring sample data
- **Key on/off:** drop, delay or duplicate note events
- **Noise mode:** enable noise on random voices
- **Pitch modulation:** enable modulation between neighboring voices (FM-like effects)
- **Reverb config:** mutate reverb registers and the work-area base. Moving the work area over sample data makes reverb output overwrite the samples over time, so the sound bank slowly erodes while it plays.

### 2.4 Geometry and GPU genes

ROM form (via provenance): perturb vertex coordinates, colors and UVs at their source bytes.

Interface form (GP0 filter):

- Vertex jitter: fixed offset, noise, gradual drift, or snap-to-grid
- Vertex swap within a primitive (folded or inverted polygons)
- Color channel swap or rotation, gradient inversion
- Toggle semi-transparency and change the blend mode
- Toggle raw-texture and dithering bits
- Drawing offset and draw area drift
- Texpage and CLUT reference swaps

**Warning:** some GP0 command bits (gouraud, textured, quad vs tri) change how many parameter words follow. Flipping them without re-encoding desyncs the whole FIFO and produces garbage. Only flip bits that preserve word count, or rebuild the command with the correct number of parameters.

### 2.5 Texture, CLUT and font genes

- Palette rotation, channel swaps, and entry shuffles on CLUTs. Palette corruption is highly survivable and very distinctive.
- Glyph shuffle and bit flips inside font glyphs
- Row and column shifts in texture data
- Transplant image data between assets

### 2.6 DMA and ordering-table genes (interface)

- Perturb GPU linked-list pointers: skip packets, reorder draw order, loop small sub-lists
- Change DMA block sizes or chop transfers

### 2.7 GTE genes (interface, where the BIOS or a game uses the GTE)

- Perturb the projection distance (`H`), giving FOV warping
- Perturb rotation matrix or translation vector entries
- Corrupt a small fraction of results (e.g. flip the sign of SZ, or offset SXY)

### 2.8 Table, constant and string genes

- Perturb numeric tables: timing values, init constants, counts
- Shuffle string bytes (Shell UI text)

### 2.9 Temporal genes

Any gene can have a **trigger**:

- **At time T:** the corruption begins partway through the boot
- **Window:** active between T1 and T2
- **Rot:** magnitude ramps up over time, so the machine starts nearly healthy and slowly decays. This fits the Grim Reaper name well.
- **Intermittent:** active on some frames only, with a deterministic pattern from the seed

### 2.10 Faulty hardware genes (the Faulty Hardware Simulator)

A fifth family: instead of breaking the BIOS or its traffic, break the *machine*. These genes model failing memory and, later, other failing parts, as faults that develop, wear in and sometimes depend on load, so a console can feel like a dying real one.

**Principles**

- **Never on the access path.** The recompiler reads and writes RAM directly, and both CPU backends must see the same memory. Faults are applied at scheduler points: stuck cells every eighth scanline, transient faults once per frame. So a stuck bit is "held" about 33 times a frame, not on every access.
- **Critical memory is almost never hit.** The kernel's low 64 KB of RAM, the top 16 KB (stacks) and the first 4 KB of sound RAM are avoided unless a gene is deliberately generated as critical, which happens to roughly one hardware gene in a hundred (shown as DANGEROUS in the genome list). Everything else, including game code and data, is fair game.
- **Wear is the point.** The default trigger is a rot ramp, so cells fail one after another as the machine ages.

**Fault archetypes (batch 1: main RAM, VRAM, sound RAM)**

| Fault | Physical analogy | What you see |
|---|---|---|
| Stuck-high / stuck-low bits | A cell stuck on one value | Values quietly wrong; code and data in the region drift |
| Flaky cells | Marginal cells that sometimes flip | Rare random glitches, worse the harder the console works |
| Bursts | A noisy bus | Short runs of garbage bytes |
| Bad column | A fractured trace | The same bit wrong at a regular stride: stripes in buffers and samples |
| Thermal decay | Missing DRAM refresh | Memory nobody rewrites slowly drifts toward 0 or FF; busy buffers stay healthy |
| Rowhammer | Neighbouring-cell leakage | Rows rewritten every frame damage the rows next to them |
| Dead VRAM line | A dead scan row | A whole row of pixels forced to black |
| Bus load | A weak transceiver | A gene's failure rate scales with the DMA traffic of the last frame (idle = stable) |

**Deliberately not done (needs an access-path hook):** stuck address lines and row aliasing, read-destructive bits, write stutter, per-access rowhammer counters. They would need a check on every memory access (or emitted code in the recompiler) and are an opt-in idea for later.

**Later batches:** CD drive (bad sectors, slow reads, the existing Bad modchip as a gene), controller (ghost presses, stick drift, dropouts), memory card (write failures), clock drift.

---

## Layer 3 — Evaluation

### 3.1 Deterministic headless runner

- No host timing leaks, no uninitialized state, no thread-order dependence. The same genome must produce bit-identical telemetry.
- Runs much faster than real time, with several instances in parallel.
- Crash isolation: evaluations can hit emulator bugs, so each run has an instruction-count or cycle limit and a host crash guard.
- **Scenarios:** "no disc" (boots into the Shell) and optionally "disc inserted" (boot sequence with a fixed test image). Some boot visuals may come from the disc rather than the BIOS; the trace shows which.

### 3.2 Staged windows

1. **Kill check** (~0.5 s emulated): reject instant deaths.
2. **Survival run** (several seconds, long enough to get past the intro into the Shell): reject delayed deaths.
3. **Long run** (only for archive candidates): catches late collapse and lets rot genes develop.

### 3.3 Liveness signals (gates)

Avoid metrics that flag healthy machines or reward garbage.

| Signal | Notes |
|---|---|
| **Coverage growth** | New basic blocks executed per window. Tiny polling loops (VSync, CD status) are *normal*. A loop is dead only if nothing that could change its exit condition ever changes. |
| **Exception loops** | Same exception cause at the same EPC repeating with no forward progress. Exceptions themselves are fine and can be interesting. |
| **Frame liveness** | Per-frame framebuffer hash. Dead = identical for a long time *and* no pending activity. |
| **Audio liveness** | Silence, DC offset, or one unchanging tone for the whole window. |
| **Runaway CPU** | Execution mostly inside regions labeled data. Usually uninteresting noise. |

### 3.4 Interest signals (the middle band)

Noise is not interesting. Reward structure that changes:

- **Visuals:** framebuffer compression ratio in a middle range (neither a solid fill nor random static); frame-to-frame diffs that are spatially coherent
- **Audio:** spectral flatness in a middle range (not a pure tone, not white noise); changes over time
- **GPU:** variety of command types and settings used

### 3.5 Behavioral fingerprint

Descriptor per survivor, each axis normalized or log-scaled:

- Histogram of GP0 command classes
- Final display mode and draw area settings
- Perceptual hashes of the framebuffer at fixed timepoints
- Audio spectral centroid and flatness over time
- Active SPU voice set (compare with Hamming distance, not Euclidean)
- Histogram of exception causes
- Basic-block coverage set (compare with Jaccard distance)
- Boot progress reached (via coverage landmarks from the clean trace)
- Genome composition (which gene families are present) — cheap and useful

---

## Layer 4 — Search: MAP-Elites archive (optional, later)

**Status:** deferred. Phases 2–4 showed that typed genes already survive at a high rate (1000/1000 sample genomes survived; about 62% were audibly changed with time-based sizing). Random generation is therefore good enough to be the core experience, and pre-filtering pulls would remove the duds that make good pulls exciting. The archive stays in the design as an optional mode ("surprise me with something I haven't seen") for users with spare CPU. It needs cheap evaluation first (snapshot forking, cached clean references, persistent workers, staged windows).

A single global score collapses diversity toward whatever maximizes it. Instead:

- Choose 2–3 coarse descriptor axes, e.g. **boot progress** × **visual character** × **audio character**, each binned.
- The archive holds at most one elite per cell (the most interesting survivor in that cell).
- Loop: pick a random elite → apply 1–3 new genes, or remove or tweak an existing one → evaluate → if it survives, place it in its cell and replace the occupant only if it's more interesting.
- Optional: CVT-MAP-Elites (centroid-based cells) using the full descriptor, if hand-picked bins feel limiting.
- Novelty check against the **whole archive**, not a short recent history, so old archetypes don't reappear.

The search runs continuously in the background. The archive is the library of broken PlayStations.

---

## Genome format and reproducibility

- A corruption is saved as a genome (list of typed genes with parameters), plus BIOS hash and scenario, not just an RNG seed. This keeps it reproducible across VibeStation versions and makes it shareable, editable and breedable.
- Compact text or base64 export for sharing ("corruption codes").
- The fingerprint and a thumbnail or short audio clip are stored alongside for browsing.

---

## UX

Mockups: https://claude.ai/artifact/9hBVYj7yV3JqBkK28co5ws

### Principles

- **Every pull is a surprise.** New Corruption generates a fresh random genome and boots it live. Nothing is pre-evaluated or filtered; the outcome is unknown to everyone until it runs.
- **Duds are fast and readable, not hidden.** A dead machine is detected within a second or two and shown with a cause of death. The next pull is one press away.
- **The user handles machines, not parameters.** Seeds, byte offsets and target regions are internal. The UI shows the machine, its genome, gene families, intensity and history.
- **The user is the curator.** Keep saves a genome; the kept set is the library. There is no automatic library in the core experience.
- **Honest about what it is.** Results reflect VibeStation's behaviour, not verified hardware behaviour (see Risks).

### Screen 1: First run (Discovery)

Shown when the loaded BIOS has no cached map (keyed by ROM hash).

- Header: "<BIOS name> hasn't been mapped yet", followed by "Runs once per BIOS, cached by ROM hash."
- **BIOS map strip.** Fills in live as regions are classified, coloured by family. Unknown regions are hatched, unused regions are dark, and dormant regions (copied but never run in the discovery scenarios) are shown separately. Shows "% classified".
- **Discovery steps** with status (done, running with progress, queued): clean-boot trace per scenario, provenance tagging, asset scan with running counts, probe mapping.
- **"Pull interface-only"** is available immediately, because interface genes need no map.
- Status bar: "Mapping", the short ROM hash, and an estimate of time left.

### Screen 2: Panel (New Corruption)

- **NEW CORRUPTION** is the primary action. It generates a random genome from the enabled families at the current intensity, applies it at reset, and boots live. Keyboard shortcut and controller combo, so it works while watching full-screen.
- **Intensity** controls gene count, gene size and **risk**:
  - Low: survival biases on (first-touch weighting, early-init protection, conservative code genes). Mostly alive, mild.
  - Middle: biases weakened.
  - High: biases off, raw and lethal. Frequent deaths, occasionally something insane.
  - The readout names both: e.g. "4–7 genes · risky".
- **Gene families**: Audio, Visual, Code, Interface toggles, plus **Rot mode** ("starts healthy, decays").
- **Death watch.** The existing liveness gates run on the live machine. On death the panel switches to a dead state:
  - Cause of death in plain words with the technical detail underneath, e.g. "Died at 1.4 s · exception loop at BFC0 2B68", "Hung waiting for VSync", "Stuck chime", "Black screen, CPU still running".
  - The genes that were active at that point.
  - Buttons: New Corruption (primary), Revive (reboot the same genome), Keep anyway.
  - Death detection is fast (target: under 2 s after the machine stops progressing) so duds cost seconds.
- **Mercy** setting: off by default. When on, deaths inside the kill window are rerolled automatically and silently counted.
- **This machine** (alive state): live view is the emulator itself; the panel shows the machine ID, auto tags, and a **curse readout** revealed after a few seconds of running: how far it is from the clean boot, from the audibility metric and later a visual one. It is computed from the result, so it never spoils a pull in advance.
- **Secondary actions:** **More like this** (mutate the current genome slightly and boot it), **Keep**, **Revive**.
- **Genome** list, readable, with a per-gene checkbox to disable genes live, and **Copy code**.
- **History** strip: the last few pulls, alive or dead, each re-bootable.

### Screen 3: Kept & History

- **Kept:** the user's library. Thumbnails, tags, cause of death for kept dead machines, notes, paste-code import, export.
- **History:** every pull this session (and optionally previous ones), newest first, including deaths. A pull you skipped past can be recovered.
- **Stats**, for fun: pulls, deaths, death causes, longest-lived machine.
- No background processes. Thumbnails and clips are captured from live play.

### Novelty without an archive

To avoid pulls that feel samey without pre-filtering, generation keeps a light record of recent pulls (gene families, kinds and targets) and nudges away from recent combinations. This affects what is generated, not whether a pull is shown.

### Colours and theming

- Every colour comes from one theme struct; nothing is hardcoded per widget, so user themes are possible later.
- Neutrals: background, surface, raised, line, text, muted text. Accent: primary button.
- Family colours reuse VibeStation's four title-screen squares: Audio blue, Visual gold, Code red, Interface teal. The same colour always means the same family (gene dots, toggles, map regions, tags).
- Status: alive teal, working gold, dead red.

### UI by implementation phase

| Phase | Available UI |
|---|---|
| 1–4 | CLI and `--genome` only |
| 5 | Panel: New Corruption live, intensity with risk, families, death watch with cause of death, Mercy, Keep, Revive, History, genome list |
| 6–7 | Visual ROM genes join the families; the map gains named and dormant regions |
| 8 | More like this, per-gene toggles, curse readout, Kept & History screen, codes, theming |
| Later | Optional archive mode and background search |

---

## Implementation phases

| Phase | Content | Relative effort |
|---|---|---|
| **1** | Deterministic headless runner, crash guard, liveness gates | Foundation; required first |
| **2** | Interface genes: SPU register filter + GP0 filter (+ temporal triggers) | Low; immediate results, works in games too |
| **3** | Clean-boot trace: exec and read maps, first-touch times, code genes | Low–moderate |
| **4** | ADPCM scanner + SPU DMA confirmation + sample genes | Moderate |
| **5** | Live random New Corruption: genome generation from families and intensity (with risk scaling), live death watch and cause of death, Mercy, Keep, Revive, history, recent-pull novelty | Moderate; makes 2.0 playable |
| **5.1** | Faulty Hardware Simulator: failing RAM, VRAM and sound RAM genes (batch 1), then CD, controller, memory card, clock | Low-moderate; runtime genes need no map |
| **6** | Provenance tracking + ROM-level geometry, texture and font genes | Higher; the biggest piece |
| **7** | Probe mapping, effect map, hand-annotation support | Moderate; improves everything above |
| **8** | UX: More like this, inspector, curse readout, Kept & History screen, sharing codes, theming | Ongoing |
| **Later** | Fingerprints + MAP-Elites archive + background search (needs fast evaluation: snapshot forking, cached clean reference, persistent workers) | Optional |

Phases 2–4 alone already produce the "amazing" audio corruptions and logo breakage through the interface layer. Phase 6 makes those corruptions part of the BIOS itself.

---

## Risks and open questions

- **Emulator vs hardware.** Deep corruption drives the machine into states real games never reach: invalid ADPCM parameters, weird GPU settings, odd COP0/GTE states. The results show *VibeStation's* behavior there, which may differ from a real PS1. That's fine for an art tool, but shouldn't be presented as hardware-accurate.
- **Grim Reaper as a fuzzer.** Crashes and hangs in the *host* are emulator bugs worth logging. Save the genome of every host crash.
- **Determinism leaks** break reproducibility of the whole archive. Verify with repeated runs of the same genome early on.
- **Evaluation cost.** Staged windows and parallel runs keep it manageable. Profile how many evaluations per second a headless instance can do before tuning window lengths.
- **Provenance granularity.** Tag propagation through ALU ops is simplified, so some data paths (computed geometry, decompressed data) may not trace back cleanly. Probe mapping covers those gaps.
- **Per-BIOS differences.** Discovery is keyed by ROM hash, so every BIOS revision gets its own map automatically.