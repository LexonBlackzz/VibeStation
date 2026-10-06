# VibeStation PS1 recompiler: findings, fixes, and recovery plan

Date: 2026-09-28  
Baseline revision: `ae53d12c6ca8c4ebca3710dc386179d3de790d39`

## Executive conclusion

VibeStation's x64 recompiler could run almost all dynamically executed
instructions as generated host code, but it had a severe translation-cache
identity bug. After a cache flush, an old sparse dispatch entry could point to a
recycled block object now containing code for a different guest PC. The epoch
check still passed because the recycled object had the new epoch. On Gran
Turismo 2 Arcade this dispatched a request for BIOS address `BFC00E70` to a
block compiled for `BFC00DB0`, corrupting control flow.

The fix makes the guest start PC part of block identity everywhere a dispatch
entry is accepted: the C++ lookup path, revalidation path, and resident x64
dispatcher. A regression test recreates the exact recycle sequence.

After this fix, the first 300 frames of Gran Turismo 2 Arcade produce identical
CPU, GPR, GTE, COP0, PC, cycle, GPU, IRQ, timer, DMA, SIO, and MDEC state between
the interpreter and recompiler. The recompiler executes 99.557% of guest
instructions inline and spends 0.443% in opcode helpers. It is 3.20x faster than
the interpreter in CPU time and 1.67x faster across the whole core in this
workload.

Interpreter fallback was a real problem and important paths are now native, but
it is no longer the main measured limit. The largest CPU costs are very short
blocks, dispatch frequency, repeated guest-state traffic, and the lack of a
general register-allocating IR. Whole-emulator speed is also capped by the
synchronous scalar software GPU.

## Test configuration

- Build: optimized Windows x64 `Release`
- BIOS: `PSX - SCPH1001.BIN`
- BIOS MD5: `924E392ED05558FFDB115408C263DCCF`
- Game: `Gran Turismo 2 (Europe) (En,Fr,De,Es,It) (Disc 1) (Arcade Mode)`
- Game path: `F:/Games/Gran Turismo 2 (Europe) (En,Fr,De,Es,It) (Disc 1) (Arcade Mode).cue`
- Timed boot: ISO9660/`SYSTEM.CNF`/`PS-X EXE` direct load after BIOS POST 7
- Warmup: 0 frames, because checkpoint equality from executable handoff was
  part of the test

An earlier investigation used a different `scph1001.bin` whose MD5 was
`AA8A2A6DE80C8164214D98A6298B8273`. Those results were invalid and are
superseded here.

## Implemented fixes

### Correct block identity after cache resets

`reset_translations()` recycles the fixed block slab but leaves sparse dispatch
pages allocated. Before the fix, this sequence was possible:

1. Dispatch entry A points to block slot 0, compiled for guest PC A.
2. The translation cache resets and its epoch increments.
3. Slot 0 is reused for guest PC B and receives the new epoch.
4. Entry A still points to slot 0.
5. A lookup for A checks only the epoch, accepts slot 0, and executes B.

The recompiler now requires both `cache_epoch` and `start_pc` to match in
`Impl::lookup()`, `Impl::try_revalidate()`, and the resident x64 dispatch loop.
The new `v4_stale_dispatch_entry_after_backend_flush` comparison case compiles
A, flushes, reuses the slot for B, and dispatches A again.

### Native multiply, divide, and HI/LO timing

`MULT`, `MULTU`, `DIV`, and `DIVU` now have direct x64 emitters. They preserve
signed and unsigned results, divide-by-zero behavior, `INT_MIN / -1`, R3000A
load-delay visibility, and the HI/LO result-ready stalls observed by `MFHI` and
`MFLO`. The differential suite forbids instruction helpers for these cases.

### Native GP0 and GP1 word stores

Aligned stores to `1F801810` and `1F801814` use a narrow GPU MMIO bridge instead
of the complete interpreter lifecycle. Alignment, cache isolation, delayed-load
retirement, timing, and GPU side effects remain intact. A GP1 display-enable
test requires this native path.

### Deterministic direct executable loading

The loader reads the ISO9660 volume, parses `SYSTEM.CNF`, resolves and validates
the `PS-X EXE`, copies text, clears BSS, preserves the BIOS-initialized kernel,
sets PC/GP/stack, and flushes CPU translations. It waits for POST 7 so BIOS
services are ready. The same path now backs the UI's **Direct Disc Boot** option
instead of relying only on a BIOS shell patch.

This is useful for deterministic profiling and cross-region discs. It does not
replace normal CD-ROM boot validation.

### Scheduler accounting and telemetry

- CPU and DMA cycles crossing a scanline deadline carry into the next deadline
  instead of being discarded.
- Device-requested timing boundaries end the current CPU batch before devices
  advance.
- Benchmark coverage separates inline code from V4 opcode helpers.
- Results include dispatch exits, unsafe-state rejects, helper opcode counts,
  and hashes for every major component.

## Gran Turismo 2 results

### 300-frame executable benchmark

| Metric | Interpreter | Recompiler | Change |
|---|---:|---:|---:|
| Average CPU time | 15.292 ms | 4.778 ms | 3.20x faster |
| CPU p50 | 14.473 ms | 3.909 ms | 3.70x faster |
| CPU p95 | 20.097 ms | 7.223 ms | 2.78x faster |
| Average core time | 25.417 ms | 15.230 ms | 1.67x faster |
| Retired guest instructions | 92,043,028 | 92,043,028 | equal |
| Inline native coverage | 0% | 99.557% | +99.557 points |
| Opcode-helper coverage | 0% | 0.443% | 407,671 instructions |
| Final CPU cycles | 202,110,113 | 202,110,113 | equal |
| Final PC | `8007D2BC` | `8007D2BC` | equal |
| Final CPU-state hash | `CBACB9382178ECD4` | `CBACB9382178ECD4` | equal |
| Final GPU hash | `4B2102857380CF46` | `4B2102857380CF46` | equal |

The recompiler entered 40,054,088 native blocks for 92,043,028 guest
instructions: only **2.30 guest instructions per block entry**. This explains
why 99.557% inline coverage still gives only a 3.20x CPU improvement.

CPU state is exact at every 30-frame checkpoint. Main RAM can differ at later
frame boundaries and later reconverge, while CPU and display state stay equal.
Serialized CD-ROM and SPU components also differ. CD-ROM serialization includes
diagnostic counters and queued-sector state; SPU state includes timing-sensitive
audio state. These differences show that device event granularity remains
backend-dependent and must be investigated.

### 1,200-frame controller-driven run

The test repeatedly presses Start during frames 90-500 and Cross during frames
180-1,200. It moves beyond the static intro: display hashes change throughout
the run and GTE state becomes active.

| Metric | Interpreter | Recompiler | Change |
|---|---:|---:|---:|
| Average CPU time | 15.845 ms | 9.234 ms | 1.72x faster |
| Average core time | 21.277 ms | 14.794 ms | 1.44x faster |
| Inline native coverage | 0% | 96.865% | +96.865 points |
| Opcode-helper coverage | 0% | 3.135% | 10,266,945 instructions |
| Final display hash | `58EA3FF2` | `58EA3FF2` | equal |

Interpreter, decoded backend, and recompiler are exact through frame 330. At
frame 360 the interpreter and decoded backend still match, while the native JIT
ends the frame at a nearby instruction with the same total cycle count. Later
checkpoints differ by a few cycles or instructions, although every recorded
display hash remains equal through frame 1,200. This points to native block
granularity around SIO/controller events. It is the next correctness issue to
fix before claiming long-run determinism.

### Normal BIOS boot

With the North American SCPH-1001 BIOS and European GT2 disc, a 300-frame normal
boot reaches the BIOS shell and issues no game CD commands. This is consistent
with an unsupported normal boot or region path, but does not prove the exact
cause. Direct Disc Boot now uses the parsed executable path for this test.

## Remaining bottlenecks and solutions

### Blocks are too short

Cacheable straight-line blocks stop at the remaining portion of one 16-byte
guest I-cache line. They contain only one to four instructions before other
limits. Each entry repeats lookup, guards, budget checks, PC bookkeeping, and
state synchronization.

**Solution:** cover several guest I-cache lines per block, record every covered
line and generation, and revalidate them together. End blocks at real control
flow or event boundaries with a practical 32-64 instruction cap.

### There is no general intermediate representation

The compiler recognizes instruction and block shapes directly. ALU, memory,
branch, prefix, delay-slot, and special cases fragment mixed instruction streams
and prevent cross-instruction optimization.

**Solution:** decode all R3000A instructions into an IR with explicit guest
reads/writes, load delay, branch delay, exception metadata, memory effects, and
cycle costs. Optimize and lower the complete block.

### Guest registers live in memory

Generated code repeatedly reads and writes the guest GPR array.

**Solution:** add a block-local register allocator. Track constants and dirty
values, eliminate redundant loads and dead writes, pin common state, and spill
only at pressure, observable exits, or service calls.

### Memory paths remain fragmented

RAM and scratchpad have direct paths, with narrow bridges for some MMIO. Other
MMIO and code-safety checks still leave generated code frequently.

**Solution:** implement a 4 GiB x64 fast-memory view where possible, with
protected MMIO/unmapped pages and fault thunks. Keep a page-LUT fallback and a
cheap code-page bit before invalidation.

### Scheduler events are scanline-oriented

Overshoot debt prevents lost cycles, but CPU, DMA, timers, CD-ROM, SIO, and SPU
still advance at separate batch cadences. Native blocks can meet the same cycle
budget at a different instruction boundary.

**Solution:** use an absolute-cycle event queue. Give generated code a
downcounter ending at the next event. Every instruction updates it, and memory
operations that create immediate events force an exit at that instruction.

### The scalar GPU caps unlimited speed

GP0 commands rasterize synchronously on the emulation thread. The 300-frame
result shows the ceiling: a 3.20x CPU gain becomes a 1.67x whole-core gain.

**Solution:** add an ordered renderer command boundary and explicit VRAM
barriers, then a hardware renderer. Retain the reference software renderer and
add specialized integer spans, SIMD, and deterministic tiled workers.

DuckStation's public architecture shows the required scale: native recompilers,
fast-memory mappings, block linking, a video thread, hardware renderers, and a
vectorized multi-threaded software renderer. Primary references include its
[repository overview](https://github.com/stenzek/duckstation),
[core build configuration](https://github.com/stenzek/duckstation/blob/master/src/core/CMakeLists.txt),
[CPU settings](https://github.com/stenzek/duckstation/blob/master/src/core/settings.h),
[fast-memory arena](https://github.com/stenzek/duckstation/blob/master/src/core/bus.h),
and [video-thread interface](https://github.com/stenzek/duckstation/blob/master/src/core/video_thread.h).
Use these as design references; do not copy source with an incompatible license.

## Implementation plan

### Phase 0: finish determinism and measurement

1. Fix native SIO/event-boundary drift.
2. Separate architectural device state from diagnostic counters in hashes.
3. Add RAM-difference dumps with address ranges and last-writer provenance.
4. Capture a repeatable GT2 in-race input movie and save state.
5. Add two titles with different CD, GTE, and renderer workloads.

**Exit gate:** interpreter, decoded, and recompiler match architectural hashes
at every checkpoint for at least 10,000 frames.

### Phase 1: mixed-block IR and register allocation

1. Define IR for integer, COP0, memory, HI/LO, delay, and exception behavior.
2. Build mixed blocks across multiple I-cache lines.
3. Add liveness, constants, dead-write removal, and x64 register allocation.
4. Keep uncommon GTE operations as narrow calls until profiling justifies more.
5. Expose every generic instruction-helper escape in telemetry.

**Exit gate:** at least 99.5% inline coverage in-race, at least eight guest
instructions per dispatch entry, and exact differential tests.

### Phase 2: fast memory and event-driven execution

1. Add fixed-map fast memory with a LUT fallback.
2. Replace range scans with code-page protection or bitmaps.
3. Implement an absolute event downcounter shared by CPU and devices.
4. Patch direct links after compilation and unlink on invalidation.
5. Preserve tests for interrupts, exceptions, cache isolation, delays, and SMC.

**Exit gate:** below 1.0 ms CPU p95 on the target 13th-generation i5 for a fixed
in-race workload, or a verified 8x gain over this interpreter.

### Phase 3: renderer architecture

1. Define ordered GP0 commands and VRAM synchronization.
2. Add a native-resolution hardware renderer.
3. Refactor software rasterization into specialized integer span kernels.
4. Add SIMD and deterministic tile workers.
5. Require VRAM hashes and the ten GPU tests for every renderer change.

**Exit gate:** unlimited speed is no longer scalar-rasterizer-bound; the
software path scales across cores and remains pixel-accurate.

### Phase 4: continuous performance gates

Record component p50/p95/p99 times; inline/helper coverage; block lengths;
dispatch and direct-link counts; compile time; code-cache size; invalidations;
memory slow paths; rendered pixels; and architectural hashes. Use Release,
fixed renderer/resolution, VSync off, 300 warmup frames, and at least 1,200
measured frames.

| Priority | Work | Expected result | Main risk |
|---|---|---|---|
| P0 | Exact event exits from native blocks | Removes frame-boundary drift | IRQ and delay-slot timing |
| P0 | Deterministic in-race GT2 capture | Makes optimization measurable | Input and CD stability |
| P1 | Mixed-block IR and register allocator | Largest CPU gain | Precise exceptions |
| P1 | Multi-line blocks | Far fewer dispatches | I-cache invalidation |
| P1 | Event downcounter | Lower overhead and exact timing | Device integration |
| P1 | x64 fast memory | Removes bus-call overhead | Fault handling |
| P2 | Hardware renderer | Largest whole-core gain | VRAM synchronization |
| P2 | SIMD/threaded software renderer | Faster accurate fallback | Ordering and masks |

## Validation

- Optimized Windows x64 build succeeds.
- Interpreter-versus-recompiler suite passes, including stale dispatch recycle,
  native multiply/divide, HI/LO timing, and GP1 store cases.
- All ten GPU tests pass with signature `10DA976A201320C4`.
- GT2 300-frame CPU/GPU results match at 202,110,113 cycles.
- GT2 runs 1,200 controller-driven frames with matching display hashes; native
  scheduler drift begins at frame 360 and is documented.
- `git diff --check` passes.

## Interpreting the original comparison

The reported 0.24 ms versus 15 ms latency is a 62.5x difference. The reported
2,000 FPS versus roughly 2x real time is about a 33x throughput difference if
real time is 60 FPS. These measure different things and should not be combined.
Either way, VibeStation remains far behind a mature emulator.

This patch removes a game-breaking recompiler corruption and most measured
interpreter dependence. The evidence now points at dispatch/state traffic, event
scheduling, and rendering as the work that can close the remaining gap.
