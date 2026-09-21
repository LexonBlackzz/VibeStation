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
