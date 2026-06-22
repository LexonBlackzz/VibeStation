#pragma once
#include "types.h"

// ── Hardware Timers (Root Counters) ────────────────────────────────
// Timer 0: Pixel clock / dot clock
// Timer 1: Horizontal retrace
// Timer 2: System clock / 8

class System;

struct Timer {
  u32 counter = 0; // Current counter value
  u16 target = 0;  // Target value
  u16 mode = 0;    // Mode register
  bool one_shot_done = false;
  bool sync_released = false;
  bool irq_pulse_restore_pending = false;

  // Mode register bits:
  // 0     : Sync enable
  // 1-2   : Sync mode
  // 3     : Reset to 0 on target
  // 4     : IRQ on target
  // 5     : IRQ on 0xFFFF overflow
  // 6     : IRQ repeat
  // 7     : IRQ toggle/pulse
  // 8-9   : Clock source
  // 10    : IRQ request (0 = yes) — read only
  // 11    : Reached target — set by hw, cleared on read
  // 12    : Reached 0xFFFF — set by hw, cleared on read

  bool irq_on_target() const { return (mode >> 4) & 1; }
  bool irq_on_overflow() const { return (mode >> 5) & 1; }
  bool irq_repeat() const { return (mode >> 6) & 1; }
  bool irq_toggle() const { return (mode >> 7) & 1; }
  bool sync_enable() const { return mode & 1; }
  u8 sync_mode() const { return static_cast<u8>((mode >> 1) & 0x3); }
  u8 clock_source() const { return static_cast<u8>((mode >> 8) & 0x3); }
  bool reset_on_target() const { return (mode >> 3) & 1; }
  bool reached_target() const { return (mode >> 11) & 1; }
  bool reached_overflow() const { return (mode >> 12) & 1; }
};

class Timers {
public:
  void init(System *sys) {
    sys_ = sys;
    reset();
  }
  void reset();

  u32 read(u32 offset) const;
  void write(u32 offset, u32 value);

  // Advance timers by the given number of CPU cycles
  void tick(u32 cycles);
  void hblank_pulse();
  void set_vblank(bool active);

  // Side-effect-free diagnostic access. Unlike read(), this does not clear
  // the timer reached-target/reached-overflow flags in the mode register.
  const Timer &debug_timer(int index) const { return timers_[index]; }
  bool debug_hblank_active() const { return hblank_active_; }
  bool debug_vblank_active() const { return vblank_active_; }
  u32 debug_timer0_dot_remainder() const {
    return timer0_dot_cycle_remainder_;
  }
  u32 debug_timer2_sysclk8_remainder() const {
    return timer2_sysclk8_cycle_remainder_;
  }

private:
  System *sys_ = nullptr;
  Timer timers_[3];
  bool hblank_active_ = false;
  bool vblank_active_ = false;
  u32 timer0_dot_cycle_remainder_ = 0;
  u32 timer2_sysclk8_cycle_remainder_ = 0;

  void tick_timer(int index, u32 ticks);
  bool is_paused_by_sync(int index) const;
  void process_sync_event(int index, bool active);
  void handle_timer_event(Timer &t, int index, bool target_hit, bool overflow_hit);
  void fire_irq(int index);
};
