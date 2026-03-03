#include "timer.h"
#include "system.h"

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
    return t.counter;
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
    t.counter = static_cast<u16>(value);
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
    if (g_trace_timer &&
        trace_should_log(g_timer_trace_counter, g_trace_burst_timer,
                         g_trace_stride_timer)) {
      LOG_DEBUG("Timers: T%d MODE <= 0x%04X", timer, t.mode);
    }
    break;
  case 0x8:
    t.target = static_cast<u16>(value);
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

    if (i == 2) {
      // PSX-SPX timer2 clock source:
      //   0/1 = System Clock, 2/3 = System Clock / 8
      if (source == 2 || source == 3) {
        ticks = cycles / 8;
      } else {
        ticks = cycles;
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
  const u32 new_counter = old_counter + cycles;
  t.counter = static_cast<u16>(new_counter & 0xFFFFu);

  const bool target_hit =
      (t.target != 0) && (old_counter < t.target) && (new_counter >= t.target);
  const bool overflow_hit = new_counter >= 0x10000u;

  handle_timer_event(t, index, target_hit, overflow_hit);

  if (target_hit && t.reset_on_target()) {
    t.counter = 0;
  }
}

void Timers::hblank_pulse() {
  hblank_active_ = true;
  process_sync_event(0);

  // Timer 0 source 1/3 is not fully dot-clock accurate yet; use one tick
  // per HBlank pulse to keep BIOS timing from stalling.
  const u8 t0_source = timers_[0].clock_source();
  if (t0_source == 1 || t0_source == 3) {
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
}

void Timers::set_vblank(bool active) {
  if (active == vblank_active_) {
    return;
  }
  vblank_active_ = active;
  if (active) {
    process_sync_event(1);
  }
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

void Timers::process_sync_event(int index) {
  if (index < 0 || index > 1) {
    return;
  }

  Timer &t = timers_[index];
  if (!t.sync_enable()) {
    return;
  }

  switch (t.sync_mode()) {
  case 1:
  case 2:
    t.counter = 0;
    break;
  case 3:
    t.sync_released = true;
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
