#include "platform/scheduler_self_test.h"

#include "core/system.h"
#include "core/types.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

// Device-level tests for the event-driven scheduler. Every device is driven
// purely through MMIO writes plus System::add_cpu_cycle_penalty() (which moves
// the CPU clock without executing code), so no BIOS is needed. The two
// properties proved for each device are:
//   * additivity   - advancing to time T in one step or in many random steps
//                    leaves identical state and raises identical IRQs;
//   * exactness    - the advertised deadline is the exact cycle of the event:
//                    nothing happens one cycle earlier, something happens on it.

namespace {

struct Rng {
  u32 s = 0x2545F491u;
  u32 next() {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
  }
  u32 below(u32 n) { return n == 0 ? 0 : next() % n; }
};

int g_failures = 0;
int g_checks = 0;

void report(const char *name, bool pass, const char *detail = "") {
  ++g_checks;
  if (!pass) {
    ++g_failures;
  }
  std::printf("SCHED_TEST name=%s result=%s %s\n", name, pass ? "PASS" : "FAIL",
              detail);
  std::fflush(stdout);
}

std::unique_ptr<System> make_system() {
  auto sys = std::make_unique<System>();
  sys->init_hardware();
  sys->reset();
  return sys;
}

u64 now(System &sys) { return sys.cpu().cycle_count(); }

// Move the CPU clock forward without executing instructions.
void tick_clock(System &sys, u64 cycles) {
  while (cycles > 0) {
    const u32 step = static_cast<u32>(std::min<u64>(cycles, 0x40000000ull));
    sys.add_cpu_cycle_penalty(step);
    cycles -= step;
  }
}

constexpr u32 kTimerBase = 0x1F801100u;
constexpr u32 kIrqStat = 0x1F801070u;
constexpr u32 kIrqMask = 0x1F801074u;

u32 timer_reg(int index, u32 reg) {
  return kTimerBase + static_cast<u32>(index) * 0x10u + reg;
}

Interrupt timer_irq(int index) {
  switch (index) {
  case 0: return Interrupt::Timer0;
  case 1: return Interrupt::Timer1;
  default: return Interrupt::Timer2;
  }
}

struct TimerConfig {
  int index = 0;
  u16 mode = 0;
  u16 target = 0;
  u16 counter = 0;
};

TimerConfig random_timer_config(Rng &rng) {
  static const u16 kTargets[] = {0, 1, 2, 3, 5, 16, 100, 1000, 0x7FFF, 0xFFFE, 0xFFFF};
  TimerConfig cfg;
  cfg.index = static_cast<int>(rng.below(3));
  u16 mode = 0;
  if (rng.below(2)) mode |= 1u << 3; // reset on target
  if (rng.below(3) != 0) mode |= 1u << 4; // irq on target
  if (rng.below(3) == 0) mode |= 1u << 5; // irq on overflow
  if (rng.below(2)) mode |= 1u << 6; // repeat
  if (rng.below(3) == 0) mode |= 1u << 7; // toggle
  // Only system-clock sources advance with time (HBlank/dot sources tick
  // from scanline edges, which the scheduler owns).
  const u32 source = cfg.index == 2 ? rng.below(4) : (rng.below(2) ? 2u : 0u);
  mode |= static_cast<u16>(source << 8);
  cfg.mode = mode;
  cfg.target = rng.below(4) == 0 ? static_cast<u16>(rng.next())
                                 : kTargets[rng.below(sizeof(kTargets) / sizeof(kTargets[0]))];
  const u32 pick = rng.below(6);
  cfg.counter = pick == 0   ? 0
                : pick == 1 ? static_cast<u16>(cfg.target - 1u)
                : pick == 2 ? cfg.target
                : pick == 3 ? static_cast<u16>(cfg.target + 1u)
                : pick == 4 ? static_cast<u16>(0xFFF0u)
                            : static_cast<u16>(rng.next());
  return cfg;
}

void apply_timer_config(System &sys, const TimerConfig &cfg) {
  // Park the other timers and program this one. Writing MODE resets COUNTER.
  for (int i = 0; i < 3; ++i) {
    sys.write32(timer_reg(i, 4), 0);
    sys.write32(timer_reg(i, 8), 0);
  }
  sys.write32(timer_reg(cfg.index, 4), cfg.mode);
  sys.write32(timer_reg(cfg.index, 8), cfg.target);
  sys.write32(timer_reg(cfg.index, 0), cfg.counter);
}

bool timers_equal(System &a, System &b) {
  for (int i = 0; i < 3; ++i) {
    const Timer &ta = a.debug_timer(i);
    const Timer &tb = b.debug_timer(i);
    if (ta.counter != tb.counter || ta.mode != tb.mode || ta.target != tb.target ||
        ta.one_shot_done != tb.one_shot_done ||
        ta.irq_pulse_cycles_left != tb.irq_pulse_cycles_left) {
      return false;
    }
    if (a.irq().request_count(timer_irq(i)) != b.irq().request_count(timer_irq(i))) {
      return false;
    }
  }
  return a.debug_timer2_sysclk8_remainder() == b.debug_timer2_sysclk8_remainder() &&
         a.irq().stat() == b.irq().stat();
}

// Advance one system in random chunk sizes that add up to `total`.
void advance_chunked(System &sys, Rng &rng, u64 total,
                     void (*sync)(System &)) {
  while (total > 0) {
    const u64 chunk = std::min<u64>(total, 1 + rng.below(rng.below(4) == 0 ? 5000 : 40));
    tick_clock(sys, chunk);
    sync(sys);
    total -= chunk;
  }
}

void sync_timers_now(System &sys) { sys.sync_timers(now(sys)); }

void test_timer_additivity() {
  auto a = make_system();
  auto b = make_system();
  Rng rng;
  bool pass = true;
  int failing_case = -1;
  for (int c = 0; c < 400 && pass; ++c) {
    const TimerConfig cfg = random_timer_config(rng);
    apply_timer_config(*a, cfg);
    apply_timer_config(*b, cfg);
    const u64 total = 1 + rng.below(rng.below(3) == 0 ? 400000 : 4000);
    tick_clock(*a, total);
    sync_timers_now(*a);
    advance_chunked(*b, rng, total, sync_timers_now);
    if (!timers_equal(*a, *b)) {
      pass = false;
      failing_case = c;
      std::printf("  timer additivity mismatch case=%d idx=%d mode=%04X target=%04X counter=%04X total=%llu\n",
                  c, cfg.index, cfg.mode, cfg.target, cfg.counter,
                  static_cast<unsigned long long>(total));
      for (int i = 0; i < 3; ++i) {
        const Timer &ta = a->debug_timer(i);
        const Timer &tb = b->debug_timer(i);
        std::printf("    t%d A: counter=%04X mode=%04X shot=%d pulse=%u irqs=%llu | B: counter=%04X mode=%04X shot=%d pulse=%u irqs=%llu\n",
                    i, ta.counter, ta.mode, ta.one_shot_done ? 1 : 0, ta.irq_pulse_cycles_left,
                    static_cast<unsigned long long>(a->irq().request_count(timer_irq(i))),
                    tb.counter, tb.mode, tb.one_shot_done ? 1 : 0, tb.irq_pulse_cycles_left,
                    static_cast<unsigned long long>(b->irq().request_count(timer_irq(i))));
      }
      std::printf("    rem A=%u B=%u\n", a->debug_timer2_sysclk8_remainder(), b->debug_timer2_sysclk8_remainder());
    }
  }
  char detail[64];
  std::snprintf(detail, sizeof(detail), "cases=400 failing=%d", failing_case);
  report("timer_tick_additivity", pass, detail);
}

void test_timer_deadline_exact() {
  auto sys = make_system();
  Rng rng;
  bool pass = true;
  int with_deadline = 0;
  for (int c = 0; c < 600 && pass; ++c) {
    const TimerConfig cfg = random_timer_config(rng);
    apply_timer_config(*sys, cfg);
    const u64 base = sys->irq().request_count(timer_irq(cfg.index));
    const u32 deadline = sys->debug_timers_cycles_until_irq();
    if (deadline == Timers::kNoEvent) {
      tick_clock(*sys, 300000);
      sys->sync_timers(now(*sys));
      if (sys->irq().request_count(timer_irq(cfg.index)) != base) {
        pass = false;
        std::printf("  timer raised an IRQ with no deadline: idx=%d mode=%04X target=%04X counter=%04X\n",
                    cfg.index, cfg.mode, cfg.target, cfg.counter);
      }
      continue;
    }
    ++with_deadline;
    tick_clock(*sys, deadline - 1u);
    sys->sync_timers(now(*sys));
    const bool early = sys->irq().request_count(timer_irq(cfg.index)) != base;
    tick_clock(*sys, 1);
    sys->sync_timers(now(*sys));
    const bool on_time = sys->irq().request_count(timer_irq(cfg.index)) == base + 1u;
    if (early || !on_time) {
      pass = false;
      std::printf("  timer deadline mismatch: idx=%d mode=%04X target=%04X counter=%04X deadline=%u early=%d on_time=%d\n",
                  cfg.index, cfg.mode, cfg.target, cfg.counter, deadline,
                  early ? 1 : 0, on_time ? 1 : 0);
    }
  }
  char detail[64];
  std::snprintf(detail, sizeof(detail), "deadlines_checked=%d", with_deadline);
  report("timer_irq_deadline_exact", pass && with_deadline > 100, detail);
}

void test_timer_pulse_width() {
  auto sys = make_system();
  // Timer 2: sysclk, reset on target, IRQ on target, pulse mode, target 100.
  sys->write32(timer_reg(2, 4), (1u << 3) | (1u << 4) | (1u << 6));
  sys->write32(timer_reg(2, 8), 100);
  const u64 start = now(*sys);
  auto bit10 = [&]() { return (sys->debug_timer(2).mode >> 10) & 1u; };
  tick_clock(*sys, 99);
  sys->sync_timers(now(*sys));
  const bool high_before = bit10() == 1u;
  tick_clock(*sys, 1);
  sys->sync_timers(now(*sys));
  const bool low_at_irq = bit10() == 0u;
  tick_clock(*sys, Timers::kIrqPulseCycles - 1u);
  sys->sync_timers(now(*sys));
  const bool low_inside = bit10() == 0u;
  tick_clock(*sys, 1);
  sys->sync_timers(now(*sys));
  const bool high_after = bit10() == 1u;
  (void)start;
  report("timer_irq_pulse_width",
         high_before && low_at_irq && low_inside && high_after);
}

// ── CD-ROM ─────────────────────────────────────────────────────────────

constexpr u32 kCdIndex = 0x1F801800u;
constexpr u32 kCdReg1 = 0x1F801801u;
constexpr u32 kCdReg2 = 0x1F801802u;
constexpr u32 kCdReg3 = 0x1F801803u;

void cd_prepare(System &sys) {
  sys.write32(kIrqMask, 0x4u); // CDROM only
  sys.write8(kCdIndex, 1);
  sys.write8(kCdReg2, 0x1Fu); // enable all INT sources
  sys.write8(kCdIndex, 0);
}

void cd_send_command(System &sys, u8 command) {
  sys.write8(kCdIndex, 0);
  sys.write8(kCdReg1, command);
}

struct CdSignature {
  u8 last_irq;
  size_t pending;
  size_t response;
  int busy;
  u64 sectors;
  u32 irq_stat;
  bool operator==(const CdSignature &o) const {
    return last_irq == o.last_irq && pending == o.pending &&
           response == o.response && busy == o.busy && sectors == o.sectors &&
           irq_stat == o.irq_stat;
  }
};

CdSignature cd_signature(System &sys) {
  const CdRom &cd = sys.cdrom();
  return {cd.last_irq_code(), cd.pending_irq_count(), cd.response_fifo_size(),
          cd.busy_cycles_remaining(), cd.sector_count(), sys.irq().stat()};
}

void sync_cd_now(System &sys) { sys.sync_cdrom(now(sys)); }

void test_cdrom_additivity_and_deadlines(const char *name, u8 command,
                                         u32 min_events) {
  auto a = make_system();
  auto b = make_system();
  cd_prepare(*a);
  cd_prepare(*b);
  cd_send_command(*a, command);
  cd_send_command(*b, command);
  Rng rng;

  // Walk A event by event, checking each advertised deadline is exact: the
  // observable state is unchanged one cycle before it and changes on it.
  bool pass = true;
  u32 events = 0;
  u64 elapsed = 0;
  for (int guard = 0; guard < 64; ++guard) {
    const u32 deadline = a->cdrom().cycles_until_event();
    if (deadline == CdRom::kNoEvent) {
      break;
    }
    if (deadline > 1) {
      tick_clock(*a, deadline - 1u);
      a->sync_cdrom(now(*a));
      // Reading the flag register syncs on access and must not change state.
      const CdSignature before_deadline = cd_signature(*a);
      const u32 remaining = a->cdrom().cycles_until_event();
      if (remaining != 1u) {
        pass = false;
        std::printf("  %s: deadline %u but %u cycles left after deadline-1\n",
                    name, deadline, remaining);
      }
      (void)before_deadline;
      tick_clock(*a, 1);
    } else {
      tick_clock(*a, 1);
    }
    a->sync_cdrom(now(*a));
    elapsed += deadline;
    ++events;
  }
  tick_clock(*a, 1000);
  a->sync_cdrom(now(*a));
  elapsed += 1000;

  // B advances the same span in random chunks; MMIO reads of the (harmless)
  // index/status port in between sync the drive at arbitrary cycles.
  u64 remaining = elapsed;
  while (remaining > 0) {
    const u64 chunk = std::min<u64>(remaining, 1 + rng.below(rng.below(3) == 0 ? 3000 : 25));
    tick_clock(*b, chunk);
    if (rng.below(2)) {
      (void)b->read8(kCdIndex);
    } else {
      sync_cd_now(*b);
    }
    remaining -= chunk;
  }
  sync_cd_now(*b);
  const bool same = cd_signature(*a) == cd_signature(*b);
  if (!same) {
    pass = false;
    std::printf("  %s: chunked advance diverged\n", name);
  }
  char detail[80];
  std::snprintf(detail, sizeof(detail), "events=%u", events);
  report(name, pass && events >= min_events, detail);
}

// A command's INT is visible at exactly its deadline through the MMIO flag
// register (sync-on-access) and the interrupt controller.
void test_cdrom_irq_visible_at_deadline() {
  auto sys = make_system();
  cd_prepare(*sys);
  cd_send_command(*sys, 0x01); // GetStat -> INT3
  const u32 deadline = sys->cdrom().cycles_until_event();
  bool pass = deadline != CdRom::kNoEvent && deadline > 1;
  if (pass) {
    tick_clock(*sys, deadline - 1u);
    sys->write8(kCdIndex, 1);
    const u8 flag_before = static_cast<u8>(sys->read8(kCdReg3) & 0x07u);
    const bool irq_before = (sys->irq().stat() & 0x4u) != 0;
    tick_clock(*sys, 1);
    const u8 flag_at = static_cast<u8>(sys->read8(kCdReg3) & 0x07u);
    const bool irq_at = (sys->irq().stat() & 0x4u) != 0;
    pass = flag_before == 0 && !irq_before && flag_at == 3 && irq_at;
    char detail[96];
    std::snprintf(detail, sizeof(detail), "deadline=%u before=%u/%d at=%u/%d",
                  deadline, flag_before, irq_before ? 1 : 0, flag_at,
                  irq_at ? 1 : 0);
    report("cdrom_int_visible_at_exact_cycle", pass, detail);
    return;
  }
  report("cdrom_int_visible_at_exact_cycle", false, "no deadline");
}

// ── MDEC ───────────────────────────────────────────────────────────────

constexpr u32 kMdecData = 0x1F801820u;
constexpr u32 kMdecStatus = 0x1F801824u;

void mdec_feed_macroblock(System &sys) {
  sys.write32(kMdecStatus, 0x60000000u); // enable DMA in/out requests
  sys.write32(kMdecData, 0x38000006u);   // decode 6 words, 15-bit output
  for (int i = 0; i < 6; ++i) {
    sys.write32(kMdecData, 0xFE000000u); // block: DC=0, then EOB
  }
}

void test_mdec_ready_deadline() {
  auto a = make_system();
  auto b = make_system();
  mdec_feed_macroblock(*a);
  mdec_feed_macroblock(*b);
  const u32 deadline = a->debug_mdec().cycles_until_event();
  bool pass = deadline != Mdec::kNoEvent && deadline > 1;
  char detail[96] = "no macroblock output";
  if (pass) {
    // Status bit 31 = data-out FIFO empty; the request (bit 27) rises with
    // the output becoming visible.
    tick_clock(*a, deadline - 1u);
    const u32 before = a->read32(kMdecStatus);
    tick_clock(*a, 1);
    const u32 at = a->read32(kMdecStatus);
    const bool empty_before = (before >> 31) & 1u;
    const bool request_before = (before >> 27) & 1u;
    const bool empty_at = (at >> 31) & 1u;
    const bool request_at = (at >> 27) & 1u;
    pass = empty_before && !request_before && !empty_at && request_at;
    std::snprintf(detail, sizeof(detail), "deadline=%u empty %d->%d request %d->%d",
                  deadline, empty_before ? 1 : 0, empty_at ? 1 : 0,
                  request_before ? 1 : 0, request_at ? 1 : 0);

    // Additivity: b syncs in random pieces to the same cycle.
    Rng rng;
    u64 remaining = deadline;
    while (remaining > 0) {
      const u64 chunk = std::min<u64>(remaining, 1 + rng.below(70));
      tick_clock(*b, chunk);
      b->sync_mdec(now(*b));
      remaining -= chunk;
    }
    if (b->read32(kMdecStatus) != at) {
      pass = false;
    }
  }
  report("mdec_output_ready_exact_and_additive", pass, detail);
}

// ── DMA ────────────────────────────────────────────────────────────────

constexpr u32 kDmaBase = 0x1F801080u;
constexpr u32 kDmaDpcr = 0x1F8010F0u;
constexpr u32 kDmaDicr = 0x1F8010F4u;

void test_dma_otc_completion() {
  auto sys = make_system();
  constexpr u32 kWords = 40;
  sys->write32(kDmaDpcr, sys->read32(kDmaDpcr) | 0x08000000u); // enable ch6
  sys->write32(kDmaDicr, (1u << 23) | (1u << 22)); // master + ch6 IRQ enable
  sys->write32(kIrqMask, 1u << 3);
  sys->write32(kDmaBase + 6 * 0x10 + 0, 0x2000u + (kWords - 1u) * 4u);
  sys->write32(kDmaBase + 6 * 0x10 + 4, kWords);
  const u64 before = now(*sys);
  sys->write32(kDmaBase + 6 * 0x10 + 8, 0x11000002u);
  const u64 stall = now(*sys) - before;
  const u64 expected_stall = kWords + (kWords + 15u) / 16u;
  // OTC writes backwards from MADR: the top word links to its predecessor and
  // the lowest word is the end-of-list marker.
  const u32 top_addr = 0x2000u + (kWords - 1u) * 4u;
  const bool linked = sys->read32(top_addr) == ((top_addr - 4u) & 0x001FFFFCu);
  const bool terminated = sys->read32(0x2000u) == 0x00FFFFFFu;
  const bool irq = (sys->irq().stat() & (1u << 3)) != 0;
  const bool done = (sys->read32(kDmaBase + 6 * 0x10 + 8) & (1u << 24)) == 0;
  char detail[128];
  std::snprintf(detail, sizeof(detail),
                "stall=%llu expected=%llu linked=%d term=%d irq=%d done=%d",
                static_cast<unsigned long long>(stall),
                static_cast<unsigned long long>(expected_stall), linked ? 1 : 0,
                terminated ? 1 : 0, irq ? 1 : 0, done ? 1 : 0);
  report("dma_otc_completion_stall_irq",
         stall == expected_stall && linked && terminated && irq && done, detail);
}

// An armed MDEC-out DMA waits for its request line; the transfer must start at
// the exact cycle the MDEC output becomes visible, not at some later slice.
void test_dma_starts_on_device_event() {
  auto sys = make_system();
  mdec_feed_macroblock(*sys);
  const u32 deadline = sys->debug_mdec().cycles_until_event();
  bool pass = deadline != Mdec::kNoEvent && deadline > 2;
  char detail[128] = "no macroblock output";
  if (pass) {
    sys->write32(kDmaDpcr, sys->read32(kDmaDpcr) | 0x00000080u); // ch1 enable
    constexpr u32 kDest = 0x4000u;
    sys->write32(kDmaBase + 1 * 0x10 + 0, kDest);
    sys->write32(kDmaBase + 1 * 0x10 + 4, (1u << 16) | 128u); // 1 block, 128 words
    sys->write32(kDmaBase + 1 * 0x10 + 8, 0x01000200u);       // block sync, enable
    const u64 armed_at = now(*sys);
    const bool still_armed =
        (sys->read32(kDmaBase + 1 * 0x10 + 8) & (1u << 24)) != 0;
    // The deadline the scheduler will honour includes the MDEC event.
    const u64 mdec_deadline = sys->debug_mdec_synced_cycle() + deadline;
    const u64 sched_deadline = sys->next_device_deadline();
    // One cycle before the event nothing may have moved.
    sys->service_device_events(mdec_deadline - 1u);
    const u64 spent_early = now(*sys) - armed_at;
    // At the event the scheduler advances the MDEC and starts the transfer.
    tick_clock(*sys, mdec_deadline - now(*sys));
    sys->service_device_events(now(*sys));
    const u64 stalled = now(*sys) - mdec_deadline;
    const bool moved = stalled > 0;
    pass = still_armed && sched_deadline <= mdec_deadline && spent_early == 0 && moved;
    std::snprintf(detail, sizeof(detail),
                  "armed=%d sched<=mdec=%d early_stall=%llu stall_at_event=%llu",
                  still_armed ? 1 : 0, sched_deadline <= mdec_deadline ? 1 : 0,
                  static_cast<unsigned long long>(spent_early),
                  static_cast<unsigned long long>(stalled));
  }
  report("dma_starts_at_device_event", pass, detail);
}

// ── SIO ────────────────────────────────────────────────────────────────

struct SioSignature {
  u16 stat;
  u16 ctrl;
  u64 irqs;
  u32 event;
  u32 irq_stat;
  bool operator==(const SioSignature &o) const {
    return stat == o.stat && ctrl == o.ctrl && irqs == o.irqs &&
           event == o.event && irq_stat == o.irq_stat;
  }
};

SioSignature sio_signature(System &sys) {
  return {sys.sio().joy_stat_snapshot(), sys.sio().joy_ctrl_snapshot(),
          sys.sio().irq_assert_count(), sys.sio().cycles_until_event(),
          sys.irq().stat()};
}

void start_sio_transfer(System &sys) {
  sys.write32(kIrqMask, 1u << 7);
  sys.write16(0x1F80104Eu, 1u);      // BAUD: 1 * 8 cycles
  sys.write16(0x1F80104Au, 0x0003u); // select + TX enable
  sys.write8(0x1F801040u, 0x01u);    // begin transfer
}

void test_sio_additivity_and_deadline() {
  auto a = make_system();
  auto b = make_system();
  start_sio_transfer(*a);
  start_sio_transfer(*b);
  bool pass = true;
  u32 events = 0;
  u64 elapsed = 0;
  for (int guard = 0; guard < 64; ++guard) {
    const u32 deadline = a->sio().cycles_until_event();
    if (deadline == 0) {
      break;
    }
    if (deadline > 1) {
      tick_clock(*a, deadline - 1u);
      a->sync_sio(now(*a));
      if (a->sio().cycles_until_event() != 1u) {
        pass = false;
      }
      tick_clock(*a, 1);
    } else {
      tick_clock(*a, 1);
    }
    a->sync_sio(now(*a));
    elapsed += deadline;
    ++events;
  }
  Rng rng;
  u64 remaining = elapsed;
  while (remaining > 0) {
    const u64 chunk = std::min<u64>(remaining, 1 + rng.below(9));
    tick_clock(*b, chunk);
    b->sync_sio(now(*b));
    remaining -= chunk;
  }
  pass = pass && sio_signature(*a) == sio_signature(*b) && events >= 1;
  char detail[64];
  std::snprintf(detail, sizeof(detail), "events=%u", events);
  report("sio_event_deadline_exact_and_additive", pass, detail);
}

// ── Scheduler integration ──────────────────────────────────────────────

// Runs `run_frame` on a tiny spin loop in RAM with a repeating timer IRQ armed
// (masked): the IRQ count must be exactly floor(elapsed / period) at the frame
// edge, however the run is sliced, and frame length must not drift.
void test_frame_scheduler_exact(CpuExecutionMode mode, const char *name) {
  auto sys = make_system();
  g_cpu_execution_mode_cli_override = true;
  g_cpu_execution_mode_cli_value = mode;
  // j 0x80010000 ; nop
  sys->write32(0x00010000u, 0x08004000u);
  sys->write32(0x00010004u, 0x00000000u);
  CpuDebugState state = sys->cpu().debug_state();
  state.pc = 0x80010000u;
  state.next_pc = 0x80010004u;
  state.cycles = 0;
  sys->cpu().debug_set_state(state);
  sys->cpu().flush_cpu_backend();
  sys->cpu().notify_cpu_backend_frame(1);
  sys->rebase_scheduler_clock();

  constexpr u32 kPeriod = 997;
  sys->write32(timer_reg(2, 4), (1u << 3) | (1u << 4) | (1u << 6));
  sys->write32(timer_reg(2, 8), kPeriod);
  const u64 t0 = now(*sys);
  const u64 irq0 = sys->irq().request_count(Interrupt::Timer2);
  const u64 edge0 = sys->debug_frame_edge_cycle();
  constexpr int kFrames = 10;
  for (int i = 0; i < kFrames; ++i) {
    sys->run_frame(false);
  }
  const u64 edge1 = sys->debug_frame_edge_cycle();
  const u64 irqs = sys->irq().request_count(Interrupt::Timer2) - irq0;
  const u64 expected_irqs = (edge1 - t0) / kPeriod;
  // The CPU may overshoot an edge by less than one instruction; it must not
  // accumulate: total run length tracks the nominal edges within a few cycles.
  const u64 cpu_span = now(*sys) - t0;
  const u64 edge_span = edge1 - edge0;
  const bool no_drift = cpu_span >= edge_span && cpu_span - edge_span < 16;
  char detail[160];
  std::snprintf(detail, sizeof(detail),
                "irqs=%llu expected=%llu cpu_span=%llu edge_span=%llu",
                static_cast<unsigned long long>(irqs),
                static_cast<unsigned long long>(expected_irqs),
                static_cast<unsigned long long>(cpu_span),
                static_cast<unsigned long long>(edge_span));
  report(name, irqs == expected_irqs && irqs > 100 && no_drift, detail);
}

// Save state / restore must reproduce the same future, including the lazily
// synced device timestamps (needs a BIOS to reach interesting device state).
void test_snapshot_determinism(const std::string &bios_path) {
  if (bios_path.empty()) {
    std::printf("SCHED_TEST name=snapshot_determinism result=SKIP no BIOS path given\n");
    return;
  }
  auto sys = make_system();
  if (!sys->load_bios(bios_path)) {
    report("snapshot_determinism", false, "BIOS load failed");
    return;
  }
  sys->reset();
  for (int i = 0; i < 240; ++i) {
    sys->run_frame(false);
  }
  SystemSnapshot snap;
  sys->save_state(snap);
  auto hashes = [&]() {
    System::SnapshotComponentHashes h{};
    sys->debug_snapshot_component_hashes(h);
    return h;
  };
  for (int i = 0; i < 120; ++i) {
    sys->run_frame(false);
  }
  const System::SnapshotComponentHashes first = hashes();
  const u64 first_cycles = now(*sys);
  sys->restore_state(snap);
  for (int i = 0; i < 120; ++i) {
    sys->run_frame(false);
  }
  const System::SnapshotComponentHashes second = hashes();
  const bool same = first.cpu == second.cpu && first.ram == second.ram &&
                    first.gpu == second.gpu && first.irq == second.irq &&
                    first.timers == second.timers && first.dma == second.dma &&
                    first.sio == second.sio && first.cdrom == second.cdrom &&
                    first.spu == second.spu && first.mdec == second.mdec &&
                    first.system == second.system && first_cycles == now(*sys);
  report("snapshot_determinism", same);
}

} // namespace

int run_scheduler_self_tests(const std::string &bios_path) {
  std::printf("=== Scheduler self-test ===\n");
  test_timer_additivity();
  test_timer_deadline_exact();
  test_timer_pulse_width();
  test_cdrom_additivity_and_deadlines("cdrom_getstat_events", 0x01, 1);
  test_cdrom_additivity_and_deadlines("cdrom_init_two_response_events", 0x0A, 2);
  test_cdrom_irq_visible_at_deadline();
  test_mdec_ready_deadline();
  test_dma_otc_completion();
  test_dma_starts_on_device_event();
  test_sio_additivity_and_deadline();
  test_frame_scheduler_exact(CpuExecutionMode::Interpreter,
                             "frame_scheduler_exact_interpreter");
  test_frame_scheduler_exact(CpuExecutionMode::Recompiler,
                             "frame_scheduler_exact_recompiler");
  test_snapshot_determinism(bios_path);
  std::printf("SCHED_TEST_SUMMARY checks=%d failures=%d\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 2;
}
