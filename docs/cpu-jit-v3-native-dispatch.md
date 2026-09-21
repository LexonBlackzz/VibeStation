# CPU JIT V3 native dispatch notes

Canonical run: Spyro the Dragon (USA), 840 warmup frames, 60 measured frames,
`--cpu x64jitv3`. All rows below produced state `CDEAA474CF09AF42`,
PC `80016488`, cycles `510309192`. Times are from the local i5-13600KF and
cannot be compared directly with the earlier Xeon measurements. The host's
frequency/load changed during this session, so the later baseline control is
the more useful timing comparison.

| Commit / experiment | CPU avg ms | p50 ms | p95 ms | Native block entries | Native instructions | Helper instructions | Code bytes | State hash | PC | Cycles |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- | ---: |
| `6ab44ab` initial baseline | 5.108 | 5.083 | 5.644 | 10,027,091 | 14,552,064 | 1,629,604 | 1,258,515 | CDEAA474CF09AF42 | 80016488 | 510309192 |
| Resident loop, initial | 5.774 | 5.754 | 6.589 | 10,027,091 | 14,552,064 | 1,629,604 | 1,258,889 | CDEAA474CF09AF42 | 80016488 | 510309192 |
| Resident loop, same-session repeat | 10.269 | 10.432 | 11.518 | 10,027,091 | 14,552,064 | 1,629,604 | 1,258,889 | CDEAA474CF09AF42 | 80016488 | 510309192 |
| `6ab44ab` same-session control | 10.728 | 10.583 | 11.827 | 10,027,091 | 14,552,064 | 1,629,604 | 1,258,515 | CDEAA474CF09AF42 | 80016488 | 510309192 |
| Resident loop + PC-indexed LUT | 10.209 | 10.092 | 12.116 | 10,027,091 | 14,552,064 | 1,629,604 | 1,258,889 | CDEAA474CF09AF42 | 80016488 | 510309192 |
| Native LW continuation | 9.228 | 9.051 | 10.984 | 8,005,353 | 14,552,064 | 1,787,943 | 1,264,638 | CDEAA474CF09AF42 | 80016488 | 510309192 |
| Native LW continuation repeat | 9.160 | 9.152 | 10.420 | 8,005,353 | 14,552,064 | 1,787,943 | 1,264,638 | CDEAA474CF09AF42 | 80016488 | 510309192 |

The resident runner executes eligible ALU and control-flow blocks in one native
call. Each compiled fixed successor points to a stable LUT cell; invalidation
clears that cell before erasing block metadata. The runner tests its cycle and
instruction budget before each block and validates each target I-cache line.
Loads, stores, dynamic jumps, helpers, diagnostics, and special memory still
exit through the existing C++ path. Pending load input is consumed by the
first linked block. The host has no calls or memory operations that can change
the I-cache or request a timing boundary while a pure native chain runs.

The `native_block_entries` counter still counts guest block executions, even
when several execute during one resident call. The decrease in C++ dispatches
is visible in `cache_hits`: 8,758,348 in the baseline versus 7,124,179 with
resident dispatch, a reduction of 1,634,169 for this measured window.

The LW continuation tier executes the following instruction with the old load
destination, then commits or cancels the pending value in generated code. It
can continue through subsequent ALU instructions and branch delay slots.
Additional memory instructions still end the block. The full CPU backend
comparison suite passes with V3 included as a target.

## Linked native entry experiment

The first linked emitter tail-jumps through stable successor cells. A cell
points to generated code or the shared chain exit; invalidation resets it
before erasing metadata. Cycle, instruction, and block totals stay in host
registers until exit. The legacy resident loop is available for A/B runs with
`VIBESTATION_V3_LEGACY_RESIDENT=1`.

The canonical Spyro image was not available in this workspace. An 840/60
Crash Bandicoot (USA) run on the same host gave:

| Path | CPU avg ms | p50 ms | p95 ms | Chain entries | Linked transitions | Max blocks | State | PC | Cycles |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- | ---: |
| Legacy resident loop | 79.311 | 72.414 | 115.077 | 0 | 0 | 0 | B86BF25F6769511B | 800141DC | 509552478 |
| First linked emitter | 85.324 | 80.652 | 136.436 | 2,368,890 | 896,632 | 16 | B86BF25F6769511B | 800141DC | 509552478 |

The first emitter averaged 1.38 blocks per linked chain entry. It still used
a dynamic I-cache line loop and memory increments for branch statistics on
each block, both candidates for removal from the hot path.

Unrolling the fixed I-cache comparisons and keeping branch counters in host
registers reduced native code from 6,334,147 to 6,161,998 bytes. On the same
Crash run it measured 84.943 ms average, 80.236 ms p50, and 139.301 ms p95,
with identical state, PC, cycles, instruction counts, and chain counts. The
small average change is within the observed host variance; this alone does
not close the gap to the legacy path.

When a slice ends inside a compiled block, V3 now lazily emits a native ALU
prefix for the safe portion that fits. On the same 840/60 Crash workload,
budget helper steps fell from 234,813 to 148,758, native inline instructions
rose from 11,353,031 to 11,439,086, and the final state remained
`B86BF25F6769511B`, PC `800141DC`, cycles `509552478`. The host speed changed
between runs: a same-session control measured 41.096 ms average for the legacy
resident loop versus 41.104 ms for linked execution. The linked path has no
demonstrated timing advantage on this workload yet.

The canonical Spyro image is at
`D:\Misc\pSXfin_1_13-1220\cdimages\Spyro the Dragon (USA).cue` on this host.
At commit `8758b39`, the 840/60 run passed the exact gate: state
`CDEAA474CF09AF42`, PC `80016488`, cycles `510309192`.

| Spyro path | CPU avg ms | p50 ms | p95 ms | Native instructions | Helper instructions | Block entries | Chain entries | Linked transitions | Max blocks | Budget helpers | Code bytes |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Linked | 5.529 | 5.456 | 6.398 | 14,552,064 | 1,685,300 | 8,002,108 | 2,339,990 | 701,072 | 16 | 264,520 | 5,444,586 |
| Legacy resident control | 5.458 | 5.431 | 6.074 | 14,552,064 | 1,685,300 | 8,002,108 | 0 | 0 | 0 | 264,520 | 5,444,586 |

The linked path averages 1.30 guest blocks per C++ chain entry on this run,
and does not yet outperform the legacy resident loop on this host.

## Linked LW continuation

Safe LW blocks whose pending load retires inside the same block now calculate
their RAM or scratchpad address in generated x64, take the RAM cycle penalty,
and continue through the stable successor cell. Invalid addresses, pending
load hazards, and unavailable fast memory leave the chain. The full CPU
comparison suite passes. The Spyro gate still matches state
`CDEAA474CF09AF42`, PC `80016488`, cycles `510309192`.

| Spyro path, same build/session | CPU avg ms | p50 ms | p95 ms | Native instructions | Helper instructions | Block entries | Chain entries | Linked transitions | Blocks per chain | Max blocks | Code bytes |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Linked with LW | 4.795 | 4.828 | 5.529 | 14,552,064 | 1,685,300 | 8,002,108 | 1,485,056 | 3,092,854 | 3.08 | 16 | 5,744,221 |
| Legacy resident control | 5.572 | 5.580 | 6.357 | 14,552,064 | 1,685,300 | 8,002,108 | 0 | 0 | — | 0 | 5,444,589 |

The linked path is 13.9% faster by CPU average in this A/B run. It still
exits often at cache misses and event budgets, so longer native chains remain
the next priority.

## Pending LW across links

The linked ABI also carries an LW pending value, source PC, and source address
across block jumps and restores them to CPU state on a chain exit. The full
comparison suite and canonical Spyro state/PC/cycle gate pass. Spyro now has
3,620,702 linked transitions across 1,520,967 chain entries, or 3.38 blocks
per entry, with a maximum of 21. Disabling this extension with
`VIBESTATION_V3_NO_PENDING_LW=1` gives 3,092,854 transitions and 3.08 blocks
per entry. The host changed speed during A/B runs (pending LW measured both
4.500 and 8.099 ms), so the timing effect needs a controlled repeat.

Alternating runs pinned to logical CPU 2 gave 4.689 and 4.494 ms without
pending LW, and 4.471 and 4.267 ms with it, all with exact Spyro state, PC,
and cycles. The mean of each pair is 4.592 versus 4.369 ms (4.9% faster).

The linked emitter no longer checks its never-lowered maximum entry count
after every block; the instruction budget already bounds the chain. A final
pinned 840/60 Spyro run after that change passed the same exact gate and full
CPU comparison suite: CPU avg 4.619 ms, p50 4.673 ms, p95 5.434 ms, 14,552,064
native instructions, 1,685,300 helper instructions, 8,002,108 block entries,
1,520,967 chain entries, 3,620,702 linked transitions, 5,913,234 code bytes.
The final timing lies within the host's observed run-to-run variation.
