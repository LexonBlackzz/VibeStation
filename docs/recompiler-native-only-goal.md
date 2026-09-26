# Recompiler Native-Only Execution Goal

## Purpose

The PS1 interpreter remains VibeStation's independent accuracy oracle, debugger path,
and differential-test reference. It is **not** part of the Recompiler execution
architecture.

When **Recompiler (Experimental)** is selected, ordinary guest execution must not
fall back to `Cpu::step()`, `Cpu::execute()`, `Cpu::compiled_opcode_fn()`, or a
generic C++ opcode handler to decide or verify what an instruction should do.
Correctness comes from implementing the R3000A architecture correctly in the
recompiler and proving it against independent tests.

This is an architectural invariant, not merely a performance preference.

## End-state invariants

A normal recompiler gameplay frame should satisfy all of the following:

- `interpreter_fallback_steps == 0`.
- generic opcode-helper execution is zero (`jit_v4_helper_instructions == 0`).
- hot block validation/refill is resident native code; no C++ callback is used to
  compare compiled instruction bytes against the emulated I-cache.
- branch delay state and load delay state stay resident across linked blocks.
- supported exceptions are emitted explicitly by generated code rather than
  re-executing the faulting opcode in the interpreter.
- RAM/scratchpad traffic uses native fast paths where architectural rules permit.
- MMIO uses narrow device bridges or specialized generated paths, never the
  interpreter instruction lifecycle.
- COP0 state is represented directly in the recompiler context.
- COP2/GTE transfers and commands have dedicated JIT paths. A temporary narrow
  GTE-device bridge is acceptable while SIMD lowering is developed; a generic CPU
  opcode fallback is not.
- direct-linked blocks remain in the resident dispatcher until a real machine
  boundary requires leaving: compilation miss, stale translation requiring
  recompilation, device/scheduler boundary, debugger request, or fatal condition.
- compilation is allowed to run in C++ because compilation is host tooling, not
  guest instruction execution. Compiled code must not call C++ to ask whether its
  already-compiled guest semantics are correct.

Diagnostic/compare modes may deliberately run the interpreter as a separate
reference execution. That must be opt-in and must never silently contaminate the
normal Recompiler path.

## Execution architecture

The desired hot path is:

```
resident native state
  -> translated block
  -> direct native edge
  -> translated block
  -> direct native edge
  -> ...
  -> explicit scheduler/device boundary
```

Not:

```
block -> C++ validation -> dispatcher -> opcode helper -> state rebuild -> block
```

The resident context owns PC, GPR view, HI/LO, load-delay state, branch-delay
state, cycle accounting, COP0 hot state, cache epoch/generations and scheduler
budgets. State is synchronized back to `Cpu` only when execution truly leaves
the resident chain.

## Work streams

1. **Remove generic CPU opcode fallback**
   - Native MULT/MULTU/DIV/DIVU and HI/LO scoreboard.
   - Native unaligned LWL/LWR/SWL/SWR.
   - Native COP0 transfers/RFE.
   - Dedicated COP2/GTE transfers, loads/stores and commands.
   - Native syscall/break/arithmetic-overflow/AddressError exception entry.
   - Native branch-likely/remaining control forms required by PS1 software.

2. **Remove validation round-trips**
   - Refill and validate direct-mapped guest I-cache lines inside resident x64.
   - Keep page/line invalidation metadata native-readable.
   - Recompile only when bytes actually changed.
   - Never call a C++ validator merely because an I-cache generation changed.

3. **Make native state genuinely resident**
   - Stop synchronizing architectural state after short chains.
   - Hoist scheduler/IRQ checks to real boundaries.
   - Keep frequently used guest registers host-resident with a real register
     allocator and dirty-state tracking.
   - Spill only where an exception/device boundary requires architectural state.

4. **Increase block quality**
   - Form longer basic blocks across guest I-cache lines safely.
   - Add trace/superblock formation for stable hot paths.
   - Patch direct edges after translation instead of repeatedly resolving PCs.
   - Specialize constant addresses, branches and common BIOS/game loops.

5. **Memory system**
   - Fastmem for mapped RAM and scratchpad.
   - Specialized hot MMIO paths.
   - Explicit slow exits for genuine device accesses and faults.
   - Precise code invalidation without global flushes.

6. **GTE**
   - First remove the generic CPU helper boundary.
   - Then lower common GTE commands to dedicated host code/SIMD where profitable.
   - Preserve the GTE scoreboard in resident state.

7. **Measure continuously**
   - Treat helper, dispatcher, generation, budget and state-sync counters as
     architectural debt counters.
   - Keep CPU backend compare tests as the correctness gate.
   - Add targeted differential cases for every newly lowered instruction family.

## Performance milestones

These are goals on the same high-end desktop class used for current profiling;
they are not correctness trade-offs.

| Milestone | Meaning |
| --- | --- |
| stable < 2.0 ms/frame | remove remaining pathological spikes/round-trips |
| stable < 1.0 ms/frame | hot execution is predominantly resident native |
| ~0.5 ms/frame | mature linking/fastmem/register residency |
| ~0.3 ms/frame on light scenes | DuckStation-class order of magnitude |

The final target is not a benchmark hack. If a shortcut violates guest-visible
timing, cache behavior, exception state, delay-slot behavior, or game
compatibility, fix the native design rather than delegating the instruction back
to the interpreter.

## Definition of done

The native-only project is complete when representative games can boot and play
with the Recompiler while profiling reports zero interpreter fallback and zero
generic opcode-helper execution in normal operation, differential CPU tests pass,
and remaining C++ exits correspond only to legitimate host/compiler/device
boundaries rather than CPU semantic fallback.
