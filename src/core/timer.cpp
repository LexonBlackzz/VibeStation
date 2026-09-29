#include "timer.h"
#include "system.h"
#include <algorithm>

namespace {
u64 g_timer_trace_counter = 0;

}

void Timers::reset() {
  for (auto &t : timers_) {
    t = {};
    t.mode |= (1u << 10);
  }
  hblank_active_ = false;
  vblank_active_ = false;
  timer0_dot_cycle_remainder_ = 0;
  timer2_sysclk8_cycle_remainder_ = 0;
}

u32 Timers::read(u32 offset) const {
  int timer = (offset >> 4) & 0x3;
  if (timer >= 3) {
    LOG_WARN("Timers: Read from invalid timer %d", timer);
    return 0;
  }

  u32 reg = offset & 0xF;
  const auto &t = timers_[timer];

  switch (reg) {
  case 0x0:
    return t.counter & 0xFFFFu;
  case 0x4: {
    u32 val = t.mode;
    const_cast<Timer &>(t).mode &= ~(3u << 11); // bits 11-12 clear on read
    return val;
  }
  case 0x8:
    return t.target;
  default:
    LOG_WARN("Timers: Unhandled read timer %d reg 0x%X", timer, reg);
    return 0;
  }
}

void Timers::write(u32 offset, u32 value) {
  int timer = (offset >> 4) & 0x3;
  if (timer >= 3) {
    LOG_WARN("Timers: Write to invalid timer %d", timer);
    return;
  }

  u32 reg = offset & 0xF;
  auto &t = timers_[timer];

  switch (reg) {
  case 0x0:
    t.counter = value & 0xFFFFu;
    if (timer == 0) {
      timer0_dot_cycle_remainder_ = 0;
    } else if (timer == 2) {
      timer2_sysclk8_cycle_remainder_ = 0;
    }
    if (g_trace_timer &&
        trace_should_log(g_timer_trace_counter, g_trace_burst_timer,
                         g_trace_stride_timer)) {
      LOG_DEBUG("Timers: T%d COUNTER <= 0x%04X", timer, t.counter);
    }
    break;
  case 0x4:
    t.mode = static_cast<u16>(value & 0x3FF);
    t.mode |= (1u << 10);
    t.counter = 0;
    t.one_shot_done = false;
    t.sync_released = false;
    t.irq_pulse_cycles_left = 0;
    if (timer == 0) {
      timer0_dot_cycle_remainder_ = 0;
    } else if (timer == 2) {
      timer2_sysclk8_cycle_remainder_ = 0;
    }
    if (g_log_fmv_diagnostics) {
      static u32 timer_mode_write_log_count = 0;
      if (timer_mode_write_log_count < 64u) {
        ++timer_mode_write_log_count;
        LOG_INFO(
            "Timers: diag T%d MODE raw=0x%04X mode=0x%04X sync=%u smode=%u irq_target=%u irq_overflow=%u repeat=%u toggle=%u source=%u target=0x%04X",
            timer, static_cast<unsigned>(value & 0xFFFFu),
            static_cast<unsigned>(t.mode), t.sync_enable() ? 1u : 0u,
            static_cast<unsigned>(t.sync_mode()), t.irq_on_target() ? 1u : 0u,
            t.irq_on_overflow() ? 1u : 0u, t.irq_repeat() ? 1u : 0u,
            t.irq_toggle() ? 1u : 0u, static_cast<unsigned>(t.clock_source()),
            static_cast<unsigned>(t.target));
      }
    }
    if (g_trace_timer &&
        trace_should_log(g_timer_trace_counter, g_trace_burst_timer,
                         g_trace_stride_timer)) {
      LOG_DEBUG("Timers: T%d MODE <= 0x%04X", timer, t.mode);
    }
    break;
  case 0x8:
    t.target = static_cast<u16>(value);
    if (g_log_fmv_diagnostics) {
      static u32 timer_target_write_log_count = 0;
      if (timer_target_write_log_count < 64u) {
        ++timer_target_write_log_count;
        LOG_INFO("Timers: diag T%d TARGET raw=0x%04X mode=0x%04X source=%u",
                 timer, static_cast<unsigned>(t.target),
                 static_cast<unsigned>(t.mode),
                 static_cast<unsigned>(t.clock_source()));
      }
    }
    if (g_trace_timer &&
        trace_should_log(g_timer_trace_counter, g_trace_burst_timer,
                         g_trace_stride_timer)) {
      LOG_DEBUG("Timers: T%d TARGET <= 0x%04X", timer, t.target);
    }
    break;
  default:
    LOG_WARN("Timers: Unhandled write timer %d reg 0x%X = 0x%04X", timer, reg,
             value);
    break;
  }
}

u32 Timers::ticks_to_target(const Timer &t) {
  const u32 counter = t.counter & 0xFFFFu;
  const u32 target = t.target;
  if (counter < target) {
    return target - counter;
  }
  // At or above the compare value the counter has to wrap before it can hit
  // the target again (a counter sitting exactly on it waits a full lap).
  return (0x10000u - counter) + target;
}

u32 Timers::ticks_to_overflow(const Timer &t) {
  return 0x10000u - (t.counter & 0xFFFFu);
}

bool Timers::counts_system_clock(int index) const {
  if (index == 2) {
    return true; // sources 0/1 = sysclk, 2/3 = sysclk / 8
  }
  const u8 source = timers_[index].clock_source();
  return source == 0 || source == 2;
}

bool Timers::counts_eighths(int index) const {
  if (index != 2) {
    return false;
  }
  const u8 source = timers_[index].clock_source();
  return source == 2 || source == 3;
}

u32 Timers::cycles_for_ticks(int index, u32 ticks) const {
  if (!counts_eighths(index)) {
    return ticks;
  }
  return ticks * 8u - timer2_sysclk8_cycle_remainder_;
}

void Timers::apply_ticks_to_event(int index, u32 ticks) {
  Timer &t = timers_[index];
  const bool target_hit = ticks == ticks_to_target(t);
  const bool overflow_hit = ticks == ticks_to_overflow(t);
  handle_timer_event(t, index, target_hit, overflow_hit);
  if (target_hit && t.reset_on_target() && t.target > 0) {
    t.counter = 0;
  } else {
    t.counter = (t.counter + ticks) & 0xFFFFu;
  }
}

void Timers::add_ticks(int index, u32 ticks) {
  Timer &t = timers_[index];
  while (ticks > 0) {
    if (is_paused_by_sync(index)) {
      return;
    }
    const u32 event = std::min(ticks_to_target(t), ticks_to_overflow(t));
    if (event > ticks) {
      t.counter = (t.counter + ticks) & 0xFFFFu;
      return;
    }
    apply_ticks_to_event(index, event);
    ticks -= event;
  }
}

void Timers::advance_timer(int index, u32 cycles) {
  Timer &t = timers_[index];
  const bool system_clock = counts_system_clock(index);
  const bool eighths = counts_eighths(index);
  u32 remaining = cycles;
  while (remaining > 0) {
    u32 step = remaining;
    if (t.irq_pulse_cycles_left != 0) {
      step = std::min(step, t.irq_pulse_cycles_left);
    }
    const bool counting = system_clock && !is_paused_by_sync(index);
    bool at_event = false;
    if (counting) {
      const u32 event_ticks =
          std::min(ticks_to_target(t), ticks_to_overflow(t));
      const u32 event_cycles = cycles_for_ticks(index, event_ticks);
      if (event_cycles <= step) {
        step = event_cycles;
        at_event = true;
      }
    }

    u32 ticks = step;
    if (eighths) {
      const u32 total = timer2_sysclk8_cycle_remainder_ + step;
      ticks = total / 8u;
      timer2_sysclk8_cycle_remainder_ = total % 8u;
    } else if (index == 2) {
      timer2_sysclk8_cycle_remainder_ = 0;
    }
    remaining -= step;

    if (t.irq_pulse_cycles_left != 0) {
      t.irq_pulse_cycles_left -= step;
      if (t.irq_pulse_cycles_left == 0) {
        t.mode |= (1u << 10);
      }
    }

    if (!counting) {
      continue;
    }
    if (!at_event) {
      t.counter = (t.counter + ticks) & 0xFFFFu;
      continue;
    }
    apply_ticks_to_event(index, ticks);

    // Reset-on-target timers with no IRQ left to raise just cycle 0..target;
    // skip whole periods instead of stepping every hit.
    const bool irq_possible =
        t.irq_on_target() && (t.irq_repeat() || !t.one_shot_done);
    if (remaining > 0 && !irq_possible && t.irq_pulse_cycles_left == 0 &&
        t.reset_on_target() && t.target > 0 && t.counter == 0) {
      const u32 period = eighths ? t.target * 8u : t.target;
      if (remaining >= period) {
        t.mode |= (1u << 11); // at least one target hit is being skipped
      }
      remaining %= period;
    }
  }
}

void Timers::advance(u32 cycles) {
  if (cycles == 0) {
    return;
  }
  for (int i = 0; i < 3; ++i) {
    advance_timer(i, cycles);
  }
}

u32 Timers::cycles_until_irq() const {
  u32 best = kNoEvent;
  for (int i = 0; i < 3; ++i) {
    const Timer &t = timers_[i];
    if (!counts_system_clock(i) || is_paused_by_sync(i)) {
      continue; // clocked by HBlank/dot edges, which the scheduler owns
    }
    if (!t.irq_repeat() && t.one_shot_done) {
      continue;
    }
    u32 ticks = kNoEvent;
    if (t.irq_on_target()) {
      ticks = std::min(ticks, ticks_to_target(t));
    }
    // A reset-on-target counter below its target never reaches 0xFFFF.
    const bool overflow_reachable =
        !(t.reset_on_target() && t.target > 0 &&
          (t.counter & 0xFFFFu) < t.target);
    if (t.irq_on_overflow() && overflow_reachable) {
      ticks = std::min(ticks, ticks_to_overflow(t));
    }
    if (ticks == kNoEvent) {
      continue;
    }
    best = std::min(best, cycles_for_ticks(i, ticks));
  }
  return best;
}

void Timers::hblank_pulse() {
  hblank_active_ = true;
  process_sync_event(0, true);

  // Timer 0 source 1/3 is not fully dot-clock accurate yet; use one tick
  // per HBlank pulse to keep BIOS timing from stalling.
  const u8 t0_source = timers_[0].clock_source();
  if (t0_source == 1 || t0_source == 3) {
    add_ticks(0, 1);
  }

  // Timer 1 source 1/3 is HBlank clock.
  const u8 t1_source = timers_[1].clock_source();
  if (t1_source == 1 || t1_source == 3) {
    add_ticks(1, 1);
  }

  hblank_active_ = false;
  process_sync_event(0, false);
}

void Timers::set_vblank(bool active) {
  if (active == vblank_active_) {
    return;
  }
  vblank_active_ = active;
  process_sync_event(1, active);
}

bool Timers::is_paused_by_sync(int index) const {
  if (index < 0 || index > 2) {
    return false;
  }

  const Timer &t = timers_[index];
  if (!t.sync_enable()) {
    return false;
  }

  if (index == 2) {
    // PSX-SPX timer2 sync modes:
    //   mode 0 or 3: stop counter
    //   mode 1 or 2: free run (same as sync disabled)
    const u8 mode = t.sync_mode();
    return mode == 0 || mode == 3;
  }

  const bool blank_active = (index == 0) ? hblank_active_ : vblank_active_;
  switch (t.sync_mode()) {
  case 0: // Pause during blanking
    return blank_active;
  case 1: // Reset on blanking and run free
    return false;
  case 2: // Reset on blanking and pause outside blanking
    return !blank_active;
  case 3: // Pause until first blanking event
    return !t.sync_released;
  default:
    return false;
  }
}

void Timers::process_sync_event(int index, bool active) {
  if (index < 0 || index > 1) {
    return;
  }

  Timer &t = timers_[index];
  if (!t.sync_enable()) {
    return;
  }

  switch (t.sync_mode()) {
  case 1:
    // Modes 1 and 2 reset the counter on the blanking edge.  The edge is
    // the start of the interval, not its trailing edge; resetting on the
    // latter incorrectly includes an entire blank period in mode 1.
    if (active) {
      t.counter = 0;
    }
    break;
  case 2:
    if (active) {
      t.counter = 0;
    }
    break;
  case 3:
    // Mode 3 pauses only until the first blanking edge.  The counter is
    // released when blanking begins, not when it ends.
    if (active) {
      t.sync_released = true;
    }
    break;
  default:
    break;
  }
}

void Timers::handle_timer_event(Timer &t, int index, bool target_hit,
                                bool overflow_hit) {
  if (target_hit) {
    t.mode |= (1u << 11);
  }
  if (overflow_hit) {
    t.mode |= (1u << 12);
  }

  const bool irq_event =
      (target_hit && t.irq_on_target()) || (overflow_hit && t.irq_on_overflow());
  if (!irq_event) {
    return;
  }

  if (!t.irq_repeat() && t.one_shot_done) {
    return;
  }

  if (!t.irq_repeat()) {
    t.one_shot_done = true;
  }

  if (t.irq_toggle()) {
    t.mode ^= (1u << 10);
  } else {
    t.mode &= ~(1u << 10);
    t.irq_pulse_cycles_left = kIrqPulseCycles;
  }

  fire_irq(index);
}

void Timers::fire_irq(int index) {
  if (g_trace_timer &&
      trace_should_log(g_timer_trace_counter, g_trace_burst_timer,
                       g_trace_stride_timer)) {
    LOG_DEBUG("Timers: IRQ from timer %d", index);
  }
  extern void timer_fire_irq(System *sys, int index);
  timer_fire_irq(sys_, index);
}
