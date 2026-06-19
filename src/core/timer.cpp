#include "timer.h"
#include "system.h"

namespace {
u64 g_timer_trace_counter = 0;

bool counter_reaches_value(u32 old_counter, u32 ticks, u16 value) {
  if (ticks == 0) {
    return false;
  }

  old_counter &= 0xFFFFu;
  if (ticks >= 0x10000u) {
    return true;
  }

  const u32 target = value;
  if (old_counter < target) {
    return old_counter + ticks >= target;
  }
  if (old_counter > target) {
    return old_counter + ticks >= (0x10000u + target);
  }

  // Already sitting on the compare value; the next hit is after a full wrap.
  return false;
}
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
    t.irq_pulse_restore_pending = false;
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

void Timers::tick(u32 cycles) {
  for (int i = 0; i < 3; ++i) {
    const u8 source = timers_[i].clock_source();
    u32 ticks = 0;

    if (i == 0 && (source == 1 || source == 3)) {
      const u32 divider = (sys_ != nullptr) ? sys_->gpu_dot_clock_divider() : 8u;
      const u64 total = static_cast<u64>(timer0_dot_cycle_remainder_) + cycles;
      ticks = static_cast<u32>(total / divider);
      timer0_dot_cycle_remainder_ = static_cast<u32>(total % divider);
    } else if (i == 2) {
      // PSX-SPX timer2 clock source:
      //   0/1 = System Clock, 2/3 = System Clock / 8
      if (source == 2 || source == 3) {
        const u64 total =
            static_cast<u64>(timer2_sysclk8_cycle_remainder_) + cycles;
        ticks = static_cast<u32>(total / 8u);
        timer2_sysclk8_cycle_remainder_ = static_cast<u32>(total % 8u);
      } else {
        ticks = cycles;
        timer2_sysclk8_cycle_remainder_ = 0;
      }
    } else {
      // Timers 0/1 use the system clock on sources 0/2.
      if (source == 0 || source == 2) {
        ticks = cycles;
      }
    }

    if (ticks > 0) {
      tick_timer(i, ticks);
    }
  }
  for (auto &t : timers_) {
    if (t.irq_pulse_restore_pending) {
      t.mode |= (1u << 10);
      t.irq_pulse_restore_pending = false;
    }
  }
}

void Timers::tick_timer(int index, u32 cycles) {
  auto &t = timers_[index];
  if (cycles == 0) {
    return;
  }

  if (is_paused_by_sync(index)) {
    return;
  }

  const u32 old_counter = t.counter;
  const bool target_hit = counter_reaches_value(old_counter, cycles, t.target);
  const bool overflow_hit = (old_counter & 0xFFFFu) + cycles >= 0x10000u;

  handle_timer_event(t, index, target_hit, overflow_hit);

  if (target_hit && t.reset_on_target() && t.target > 0) {
    t.counter = (old_counter + cycles) % t.target;
  } else {
    t.counter = (old_counter + cycles) & 0xFFFFu;
  }
}

void Timers::hblank_pulse() {
  hblank_active_ = true;
  process_sync_event(0, true);

  // Keep a narrow HBlank tick for gated Timer 0 modes which only run during
  // HBlank. Free-running dot-clock mode is advanced from CPU cycles in tick().
  const u8 t0_source = timers_[0].clock_source();
  if ((t0_source == 1 || t0_source == 3) && timers_[0].sync_enable() &&
      timers_[0].sync_mode() == 2) {
    tick_timer(0, 1);
  }

  // Timer 1 source 1/3 is HBlank clock.
  const u8 t1_source = timers_[1].clock_source();
  if (t1_source == 1 || t1_source == 3) {
    tick_timer(1, 1);
  }

  for (auto &t : timers_) {
    if (t.irq_pulse_restore_pending) {
      t.mode |= (1u << 10);
      t.irq_pulse_restore_pending = false;
    }
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
    if (!active) {
      t.counter = 0;
    }
    break;
  case 2:
    if (active) {
      t.counter = 0;
    }
    break;
  case 3:
    if (!active) {
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
    t.irq_pulse_restore_pending = true;
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
