# Event-driven scheduler

`System::run_frame()` used to run the CPU in ~32-instruction / 128-cycle slices
and tick every device after each slice. It now runs the CPU straight to the next
*deadline* and advances devices lazily.

## Model

- Every device keeps a "synced up to" absolute CPU cycle
  (`timers_/cdrom_/mdec_synced_cycle_`, `sio_/spu_synced_cpu_cycle_`).
- **Sync-on-access:** each MMIO read/write first advances the addressed device to
  `System::device_cpu_cycle()` (`System::IoScope`). That is the start-of-
  instruction cycle, also from resident native code (`jit_begin_bus_access`,
  and the hot 16-bit timer bridge now receives the resident cycle count).
- **Deadlines:** `System::next_device_deadline()` is the earliest of: timer IRQ
  (`Timers::cycles_until_irq`), CD-ROM event (`CdRom::cycles_until_event`),
  MDEC output-ready, SIO event, SPU IRQ watch (every 768-cycle sample while
  IRQ9 is armed) and SPU DMA-mode latch, and the pending DMA service.
  `run_frame` runs `cpu_.run_slice(deadline - now, ...)`; an instruction that
  started before the deadline retires past it (unchanged overshoot rule), then
  `service_device_events()` advances only the devices that are due.
- A device write that creates a deadline inside the running slice sets the
  existing `cpu_timing_boundary_requested_` so both backends return at that
  instruction boundary (`note_device_state_changed`).
- **Exactness:** timers and CD-ROM/MDEC are advanced *event by event*, so an
  event happens at its own cycle and `advance(a)+advance(b) == advance(a+b)`
  (proved by `--scheduler-self-test`). The SPU sample accumulator is integer
  (768 CPU cycles/sample) for the same reason.
- **Edges are absolute.** Scanline/frame edges are `frame_edge_cycle_ + sum of
  scanline cycles`; an overshoot shortens the next scanline instead of stretching
  the frame. Timers are never advanced past an unapplied HBlank/VBlank edge.
- No 32-instruction cap in normal mode (fast modes only change the SPU sync
  stride), so the Recompiler chains up to a full scanline: native chain entries
  per 600 BIOS-menu frames 5.76M -> 0.62M.
- Snapshots store each device's lag behind the CPU clock, so save/restore is
  bit-exact (device state is *not* forced up to date at save time; that would
  make rewind observable).

## Guest-visible timing changes (all toward hardware)

| Area | Before | Now | Why |
|---|---|---|---|
| Timer counter reads | up to ~128 cycles stale | exact at the load's cycle | sync-on-access |
| Timer IRQ | up to 16 cycles + slice late | at the exact target/overflow cycle | deadline |
| Timer multi-hit | one hit per tick call | one IRQ per hit | events applied individually |
| Timer pulse (mode bit 7 = 0) | bit 10 restored on the next 16-cycle tick | low for `Timers::kIrqPulseCycles` = 4 cycles | "a few cycles" (nocash); explicit and independent of scheduling |
| CD-ROM / MDEC | events fire up to a slice late, in a fixed order | at their own cycle, chronologically | catch-up lands on each event |
| CD-ROM seek->read | first sector period counted the whole tick | starts when the seek ends | period is measured from the seek end |
| Frame length | nominal + accumulated per-scanline overshoot | nominal, overshoot does not accumulate | absolute edges |
| DMA start on a device request | next 16-cycle service | the boundary at the request's cycle (device event / MMIO / SPUCNT latch) | request line change is an event |
| SPU IRQ | synced every 4 scanlines | within one sample (768 cycles) while armed | IRQ watch deadline |
| Data RAM bus penalty | interpreter only below 2 MB | every mirror in the RAM_SIZE window (both backends) | mirrors are real RAM; fixed a backend divergence |

### DMA model (documented decision)

Transfers are performed atomically at the start cycle (CHCR write when the
request is already up, otherwise at the event that raises it). The CPU is
stalled for `words + ceil(words/16)` cycles, charged to the retiring
instruction (CHCR write) or committed at the boundary (scheduler service), so
devices keep running during the stall and the completion IRQ becomes visible
only when the CPU resumes. Streaming channels (MDEC in/out, 96-192 words per
slice) re-arbitrate `16` cycles after their stall ends. Not modelled: chopping,
CPU/DMA bus interleaving, per-word transfer time.

## Known limitations

- HBlank is a zero-width pulse (unchanged); timer 0 dot-clock source ticks once
  per HBlank.
- `spu_skip_sync_for_turbo_` still skips SPU ticks entirely (SPU mode latch
  never applies while turbo skips).

## Diagnostics

- `--scheduler-self-test [bios]`
- `VIBESTATION_SLICE_TRACE_FRAME=N`, `VIBESTATION_STEP_TRACE_FRAME=N`
  (+ `VIBESTATION_STEP_TRACE_FROM=<cycle>`): log scheduler slices / single-step a
  frame; diff the Interpreter and Recompiler logs to find the first instruction
  where they disagree.
