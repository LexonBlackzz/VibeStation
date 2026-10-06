# Grim Reaper 2.0 — Phase 4.1 record

Moved out of PROGRESS.md unchanged; it was an appendix there. Appendix references map to
the files in this folder: A = phase-1.md, B = phase-2.md, C = phase-3.md, D = phase-4.md,
E = phase-4.1.md. Current status is in ../PROGRESS.md.

# Appendix E — Phase 4.1 reference

## 1. Phase 4.1 status (2026-10-01)

**Phase 4.1 is done on `grim-reaper/phase-4.1`, from Phase 4 `9a51ea4`.**
Phase 4's complete record is retained in Appendix D, followed by earlier phases.

Lexon's listening labels correct the Phase 4 proxy: filter crunch, shift steps
and early loop end are audible; payload reverse sounds identical to clean.
An audio-hash difference proves a numerical change, not an audible one.
Sample 7 is the bass; sample 5 is the chimes. Sample 6 is uploaded but unplayed
in the no-input boot. These names are listening evidence from Lexon.

- [x] Discovery pitch writes and median playing time per ADPCM block
- [x] Optional time/fraction windows, with exact legacy v2 compatibility
- [x] Deterministic residual metric calibrated against four human labels
- [x] Paired random yield: 50 one-gene genomes per kind, old and new sizing
- [x] Eight external listening genomes covering the metric's range; human labels pending
- [x] Stock hash, full GT2 parity, all suites and alternating lean benchmarks
- [x] Updated Phase 5 brief using survived AND audibly changed

Run artifacts and new resolved genomes belong outside Git in
`D:\Projects\github\vs-runs\phase4.1`. The existing four committed genomes
remain unchanged; the tracked labels file contains paths and human verdicts.
Pre-existing root artifacts and build-number churn are excluded from this work.

## 2. Pitch and duration

`PitchWrite` observes voice register +4 in the mapper, and each key-on snapshots
its current pitch. No hook was added to ordinary CPU/SPU/device paths. Map JSON
adds optional pitch; absent fields retain the old event serialization and hash.
Duration is `28 * 4096 * 1000 / (44100 * pitch)` ms per block. Use the median of
positive key-on durations, stored as a reduced rational for exact sizing; zero
pitch is counted separately as stalled playback. Missing/unplayed pitches use
base pitch 0x1000 (40/63 ms). This is an estimate: later pitch changes, envelopes,
looping and pitch modulation can change actual heard duration.

The clean 1800-frame map records 64 pitch writes and pitch on all 64 key-ons.
Forty starts use scanner samples (32 chime, eight bass); 24 initialization starts
point outside those candidates. Code/data/unused counts and stock telemetry stay
unchanged. New map hash is `0xB487523890F0676D`.

| Sample | Identity | Blocks | Median pitch | Estimated ms/block | 100 ms window |
|---:|---|---:|---:|---:|---:|
| 5 | Chimes (Lexon's identification) | 842 | 2315 | 1.123384 | 89 blocks |
| 6 | Uploaded, unplayed in no-input boot | 458 | 4096 fallback | 0.634921 | 158 blocks |
| 7 | Bass (Lexon's identification) | 1394 | 1198.5 | 2.213136 | 46 blocks |

Pitch and duration observations come from telemetry, not listening by the agent.

## 3. Time/fraction gene windows and compatibility

Genome stays **version 2**. An optional top-level sample-gene field is either
`"sizing":{"milliseconds":100}` or `"sizing":{"fraction_permille":1000}`.
It requests a contiguous window for filter, shift, shuffle, repeat, reverse,
transplant or nibble noise, rounded up to blocks and capped at target length.
A whole sample can exceed the legacy count limit of 256. Payload operations
retain target headers, transplant retains target length, and loop edits are
unchanged (parser rejects sizing on a loop gene).

Absent sizing retains Phase 4 count semantics, random draws, canonical text,
hashes and patches. `count` remains the legacy request metadata; resolved ROM
patches are authoritative. New random generation defaults to 100 ms. CLI
`--sample-ms N` and `--sample-fraction 0.001..1` choose the new modes;
`--sample-count N` explicitly selects the old mode. Describe prints the resolved
window when supplied with context. Exact old-fixture regeneration/hash checks,
all-kind alignment/header checks and >256-block whole-sample tests were added.

## 4. Audibility metric and calibration

`grim_compare_audio()` compares aligned 44.1 kHz stereo PCM over the same boot
frames. For each non-overlapping 1411-frame window (**31.995 ms**), residual is
mutated minus clean. Report `10 log10(residual energy / reference energy)`;
reference energy is floored at RMS 64 in s16 units. Count a window when energy
ratio is at least 10,000 ppm (**-20 dB**) and residual RMS is at least eight.
**Audible means at least 4410 frames / 100 ms cumulative above-threshold time.**
Integer energies, exact ratio comparisons and sample counts determine verdicts;
rounded dB/ms values are display fields. Also report longest exposure, per-channel
peaks, whole-run RMS and clean/mutated clipped sample counts. No spectral feature
was needed to separate these labels.

The threshold is **provisional, calibrated on only four human labels**. It is a
usefulness proxy, not a proof of perceptual difference or interestingness. The
tracked `audio_labels.json` holds genome path + Lexon's verdict/notes; null
verdicts are pending and skipped. Add labels there and rerun the fixture test.

| Phase 4 fixture | Lexon's verdict | Peak residual dB | Time above -20 dB | Exposure margin vs 100 ms | Metric |
|---|---|---:|---:|---:|---|
| sample_filter_crunch | Audible crackle | -3.445 | 6559.070 ms | +6459.070 ms | Audible |
| sample_shift_steps | Audible, weakly distinct | -4.962 | 5215.261 ms | +5115.261 ms | Audible |
| sample_payload_reverse | Exactly like clean | -19.278 | 31.995 ms | -68.005 ms | Inaudible |
| sample_early_loop_end | Audible bass edit | +11.667 | 7806.893 ms | +7706.893 ms | Audible |

Peak-only classification would incorrectly accept payload reverse. Combining
energy and exposure fits all four labels. Clean RMS floors 16 and 64 produced
the same four classifications. Tests verify exact PCM and metric JSON on repeated
runs and interpreter/recompiler boots for every labelled fixture.

CLI `--grim-audio-compare clean.wav mutated.wav [report.json]` compares captured
outputs. `--grim-audibility <bios> <frames> <genome|clean> <clean.wav> <report.json>`
evaluates one candidate, with optional `--native-cpu`, `--dump-wav` and
`--telemetry`. `--grim-explore` keeps hash change as a diagnostic and now writes
raw audibility JSON plus audible/inaudible yield and residual fields. PCM capture
and all comparisons are evaluation-only; per-frame telemetry/hash format stays
unchanged.

## 5. Paired random yield

**50 one-gene genomes per kind per sizing mode: 1000 completed evaluations.**
Seeds are `410000 + 1000*kind + index`, index 0–49, magnitude 1, clean no-input
boot over 600 frames. Old sizing draws one or two blocks (shuffle/reverse require
at least two). New sizing requests 100 ms contiguous windows. The same seeds and
targets are paired; transplants also hold the donor fixed via `--sample-donor`.
Loop edits are identical in the two modes. Automatic target selection retains
SPU-confirmed sample 6, so unplayed controls remain part of the yield estimate.

Each cell below is **survived+audible / survived+inaudible / audio-failed / dead**,
using liveness first and then the provisional metric, not hash difference.

| Kind | Old block sizing (50) | New 100 ms sizing (50) |
|---|---:|---:|
| filter_swap | 15 / 35 / 0 / 0 | 37 / 13 / 0 / 0 |
| shift_change | 3 / 47 / 0 / 0 | 25 / 25 / 0 / 0 |
| loop_start_move | 32 / 18 / 0 / 0 | 32 / 18 / 0 / 0 |
| loop_end_remove | 22 / 28 / 0 / 0 | 22 / 28 / 0 / 0 |
| loop_end_early | 39 / 11 / 0 / 0 | 39 / 11 / 0 / 0 |
| block_shuffle | 5 / 45 / 0 / 0 | 31 / 19 / 0 / 0 |
| block_repeat | 13 / 37 / 0 / 0 | 34 / 16 / 0 / 0 |
| block_reverse | 19 / 31 / 0 / 0 | 35 / 15 / 0 / 0 |
| transplant | 12 / 38 / 0 / 0 | 33 / 17 / 0 / 0 |
| nibble_noise | 0 / 50 / 0 / 0 | 22 / 28 / 0 / 0 |
| **All kinds** | **160 /340 / 0 /0** | **310 /190 / 0 /0** |

Window operators alone improve from **67/350 (19.1%) to 217/350 (62.0%)**.
This supports 100 ms as the starting default without enlarging loop edits. Sample 6
accounts for 92/500 targets in each mode, all metric-inaudible. Among played
samples the all-kind rate is 160/408 (39.2%) old versus 310/408 (76.0%) new.
Larger windows still produce duds in quiet/unused portions; this is measured
metric yield, not an ear-confirmed usefulness rate or a guarantee of interest.

The first pass left 13 incomplete process attempts (exit 1 with empty output),
including failures while the final replay rerun was also interrupted. Their cause
was not established. Completed artifacts were retained; the identical missing
genomes completed on retry at lower concurrency. These attempts are documented
separately rather than being assigned a fabricated guest-death/audio verdict.
One new-target mismatch caused by a legacy no-op retry was regenerated with the
legacy target fixed, and transplant donor choices were paired explicitly.
All final 1000 reports are complete. Scripts, genomes, reports and attempt logs
remain external under `vs-runs/phase4.1`.

A final CLI exploration smoke test (four 100 ms reverse genes) yielded two
survived+audible and two survived+inaudible, with zero failed audio, dead or
unresolved comparisons. Raw reports and updated yield CSV agree with the table.

## 6. Verification and cost

- Stock no-genome 1800 frames retains **run_hash=0x432E585CF1535F9C**.
- Full GT2 replay matches Phase 4 on every BOOT_STATE_HASH field in both backends,
  **5738/5738 frames** each. The final GUI executable was rebuilt and verified
  after the reader/donor review fixes; earlier transient attempts are noted above.
- All existing suites pass: CPU differential, GPU, scheduler, Grim self-test
  (17 checks), gene-test (128), map-test (80). The final full sample suite
  passes 99 checks at 600 frames/three seeds, including exact legacy fixtures and
  explicit-donor target selection. Audibility passes all four labels, exact
  repeated/backend PCM/report equality, plus 23 synthetic/WAV unit controls.
  Grim determinism passes sequential/thread/process runs. All 10 sample kinds stay
  in the existing all-type loops. The four Phase 4 genome files are unchanged.
- Final pinned lean benchmarks show no observed slowdown in any of the four
  cases. High performance power plan stayed active; affinity 0xFFF pins logical
  CPUs 0–11 (performance cores on this 13600KF). Each case uses six alternating
  old/new pairs, reversing order every pair. Timing spikes are retained.

| Backend/load | Warmup + measured frames | Median wall ms, Phase 4 /4.1 | Million emulated cycles/host s, Phase 4 /4.1 | Median paired wall change |
|---|---:|---:|---:|---:|
| Interpreter BIOS | 600 +600 | 3070.44 /3039.67 | 110.42 /111.54 | -0.55% |
| Interpreter GT2 | 1500 +600 | 3545.04 /3521.59 | 115.20 /115.97 | -0.40% |
| Recompiler BIOS | 900 +600 | 1547.13 /1493.11 | 219.13 /227.06 | -1.76% |
| Recompiler GT2 | 1500 +600 | 2320.45 /2268.18 | 176.01 /180.05 | -2.19% |

The 600-frame windows execute 339,026,688 cycles in BIOS and 408,377,646 in GT2;
all paired final CPU cycle counts match. Throughput is computed externally using
those exact window counts and wall time. Host state still varies: apparent gains
should not be treated as a reliable speedup.

Initial instrumentation/link-order builds showed a repeatable interpreter BIOS
slowdown around 3.5%, and a core-first link experiment left a roughly 1.7% GT2
slowdown. Those builds were replaced. The final build restores the original
benchmark body/source order and appends the three new translation units. COFF
comparison confirms the benchmark loop/register allocation and inspected CPU,
scheduler, bus and SPU hot instruction bodies match Phase 4 after masking linker
relocations. No normal-play hook or emulator change was needed. The final
paired results above close the cost check; experimental timings and binary
comparisons are retained outside Git.

Repeat the new suites from repo root (`<bios>` is the original BIOS path):

```text
build-ninja\VibeStation.exe --grim-audibility-test "<bios>"
build-ninja\VibeStation.exe --grim-sample-test "<bios>" --frames 600 --seeds 3
build-ninja\VibeStation.exe --grim-map-test "<bios>" 900
```

The labels test reads `docs/grim-reaper/audio_labels.json`; use `--labels path`
for another label set, or `--unit-only` for controls without a BIOS. Existing
suite/replay commands are retained in the reference appendices. The final run
records are `suites_closure.out`, `regression_closure.out` and
`bench_results_delivery.json` in the external Phase 4.1 directory.

## 7. Listening set for Lexon

**I cannot hear the output.** All eight ran 600 frames alive/non-silent. Below,
metric verdicts are measured; sound descriptions are predictions for Lexon to
check. Genome/WAV/report files are in
`D:\Projects\github\vs-runs\phase4.1\listening`. The labels file has pending
null verdicts for these exact paths; replace null with true/false and add notes.

| File (.json) | Metric; peak /exposure | What to listen for |
|---|---|---|
| chimes_whole_reverse | Audible; -5.409 dB /2495.646 ms | Whole 842-block chime payload reversal: changed ringing/warble from about 3.3 s |
| bass_long_filter | Audible; +6.923 dB /7518.934 ms | Half the bass sample has filter edits: long rough crunch from about 1.5 s |
| bass_payload_repeat | Audible; +5.430 dB /5087.279 ms | 100 ms of repeated payload: held/rough bass fragment from about 2.1 s |
| bass_nibble_grit | Audible; -9.833 dB /575.918 ms | 100 ms nibble window: subtler bass grit near 1.8 s |
| chimes_borderline_shift | Borderline audible; -16.682 dB /127.982 ms | Small gain/crush blip near 3.6 s; only 27.982 ms above exposure cutoff |
| chimes_short_shift | Borderline inaudible; -18.003 dB /95.986 ms | Brief chime blip near 3.7 s, if distinguishable; misses cutoff by 4.014 ms |
| chimes_quiet_shuffle | Inaudible; -21.362 dB / 0 ms | Check late chime grain around 4.4 s against clean; residual peak below cutoff |
| sample6_whole_noise | Inaudible; zero residual / 0 ms | Whole unplayed sample edited: no-input boot should match clean exactly |

Use the original BIOS and no disc in GUI configuration, from repo root:

```text
build-ninja\VibeStation.exe --genome D:\Projects\github\vs-runs\phase4.1\listening\chimes_whole_reverse.json
build-ninja\VibeStation.exe --genome D:\Projects\github\vs-runs\phase4.1\listening\bass_long_filter.json
build-ninja\VibeStation.exe --genome D:\Projects\github\vs-runs\phase4.1\listening\bass_payload_repeat.json
build-ninja\VibeStation.exe --genome D:\Projects\github\vs-runs\phase4.1\listening\bass_nibble_grit.json
build-ninja\VibeStation.exe --genome D:\Projects\github\vs-runs\phase4.1\listening\chimes_borderline_shift.json
build-ninja\VibeStation.exe --genome D:\Projects\github\vs-runs\phase4.1\listening\chimes_short_shift.json
build-ninja\VibeStation.exe --genome D:\Projects\github\vs-runs\phase4.1\listening\chimes_quiet_shuffle.json
build-ninja\VibeStation.exe --genome D:\Projects\github\vs-runs\phase4.1\listening\sample6_whole_noise.json
```

Add `--cpu recompiler` before `--genome` to repeat in the other backend. Compare
against a fresh clean boot, listen through the Shell, and label both whether a
change is audible and whether it is interesting. The two borderline cases are
especially useful for refining this provisional gate.

## 8. Limits and deferred work

The optional `--grim-map --shell-input` scenario waits 899 frames, then sends
seven directions, two X and two Circle presses, holding each 12 frames and releasing.
It reached additional BIOS code (20,889 code words versus 17,780; last new code
at frame 1740 versus 681), but added no SPU events/key-ons. Sample 6 remained
unplayed. This does **not confirm** that it contains Shell menu sounds. Keep that
hypothesis for a richer Phase 5 scenario. The script and pitch tracking are
explicit evaluation/discovery paths only.

The metric uses energy and exposure, without frequency masking, perceptual
loudness weighting, time alignment, or a spectral descriptor. It can misclassify
short transients or low-level changes; the 32 ms window grid quantizes exposure.
Different scenarios, timing shifts or longer boots require matched clean
references and more listening labels. Positive-pitch key-on medians estimate
window size; they do not account for later pitch writes/envelopes/modulation.

Only MSVC/Windows was built here. Integer verdict arithmetic and explicit signed
PCM decoding are portable, but GCC/POSIX build/results remain unverified. No
fingerprint/archive/search implementation or new visual ROM gene was added.
New BIOS-derived genomes/maps and raw run outputs stay outside Git. A pre-existing
root perimeter dump was refreshed by an emulated address-error diagnostic during
testing; it and the final rerun dump were relocated to the external run directory. Other pre-existing root
artifacts were left alone. No locked file was renamed or process killed.

## 9. Proposed Phase 5 brief (live random corruption UI)

Goal: make the current typed genes playable in the GUI. The latest local DESIGN
update makes New Corruption a fresh random live pull, with the user as curator;
MAP-Elites/background search is deferred. Start `grim-reaper/phase-5` from this
Phase 4.1 branch. Preserve that surprise/dud experience instead of pre-filtering
pulls with evaluation.

- **Live generation and playback.** New Corruption generates from enabled audio,
  visual, code and interface families at the chosen intensity, applies on reset
  and boots live. Intensity scales gene count, size and risk; low keeps protection/
  first-touch biases, high weakens them. Reuse optional time/fraction sample sizing,
  existing BIOS hash/original verification and GUI genome playback. A light record
  of recent kinds/targets can nudge generation toward variety without hiding pulls.
- **Death watch and actions.** Observe the live machine, report a readable cause
  of death and permit quick New Corruption, Revive, Keep and history replay.
  Mercy is optional and off by default; it may reroll early deaths when enabled.
  Keep is the user's library, including kept dead machines. Avoid a background
  search dependency; capture review artifacts from live play and store atomically.
- **Measured usefulness.** Retain live/dead and silent/non-silent separately from
  usefulness. For reporting, a useful audio survivor means survived AND audibly
  changed against its matched clean scenario using this residual/exposure metric;
  later add visibly changed evidence for visual edits. Hash difference alone is
  diagnostic. This metric describes a pull after it runs, never spoils or hides
  the outcome. Human labels and interest remain distinct from the provisional
  audible flag; extend labels/scenarios, including sample 6 interaction.
- **Tests/results.** Deterministic generation and replay/reset, risk bounds,
  family toggles, recovery after a dead pull, readable death reasons, Mercy off/on,
  Keep/history persistence and wrong-BIOS rejection. Test observer behavior and
  metric repeat/backend parity. Report live/dead and survived-and-audible yields
  separately, with raw metric config/version. Recheck stock hash, full GT2 parity,
  older suites and alternating pinned lean cost with cycles per host second.
- **Deferred/out of scope.** Fingerprints, MAP-Elites and background search wait
  for cheap evaluation (cached clean references, snapshot forking, persistent
  workers/staged windows). Geometry/texture/font ROM genes remain Phase 6; probe
  mapping Phase 7; broader breeding/inspector/sharing/theming UX Phase 8. No CPU
  or scheduler redesign. Keep BIOS-derived artifacts and raw runs outside Git.
