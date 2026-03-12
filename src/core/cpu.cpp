#include "cpu.h"
#include "system.h"

namespace {
inline bool cpu_diag_enabled() { return g_cpu_deep_diagnostics; }

void log_dma_context(System *sys, u32 pc, u32 instr, const u32 *gpr) {
  static u32 last_pc = 0xFFFFFFFFu;
  static u32 last_instr = 0xFFFFFFFFu;
  if (sys == nullptr) {
    return;
  }
  if (pc == last_pc && instr == last_instr) {
    return;
  }
  last_pc = pc;
  last_instr = instr;

  const auto &mdec_in = sys->dma_last_debug(0);
  const auto &mdec = sys->dma_last_debug(1);
  const auto &cd = sys->dma_last_debug(3);
  LOG_WARN(
      "CPU: ctx sp=0x%08X ra=0x%08X a0=0x%08X a1=0x%08X v0=0x%08X v1=0x%08X",
      gpr[29], gpr[31], gpr[4], gpr[5], gpr[2], gpr[3]);
  LOG_WARN(
      "CPU: last DMA0 madr=0x%08X words=%u first=0x%08X last=0x%08X "
      "bcr=0x%08X chcr=0x%08X cyc=%llu from_ram=%u",
      mdec_in.base_addr, mdec_in.transfer_words, mdec_in.first_addr,
      mdec_in.last_addr, mdec_in.block_ctrl, mdec_in.channel_ctrl,
      static_cast<unsigned long long>(mdec_in.cpu_cycle),
      mdec_in.from_ram ? 1u : 0u);
  LOG_WARN(
      "CPU: last DMA1 madr=0x%08X words=%u first=0x%08X last=0x%08X "
      "bcr=0x%08X chcr=0x%08X cyc=%llu from_ram=%u",
      mdec.base_addr, mdec.transfer_words, mdec.first_addr, mdec.last_addr,
      mdec.block_ctrl, mdec.channel_ctrl,
      static_cast<unsigned long long>(mdec.cpu_cycle), mdec.from_ram ? 1u : 0u);
  LOG_WARN(
      "CPU: last DMA3 madr=0x%08X words=%u first=0x%08X last=0x%08X "
      "bcr=0x%08X chcr=0x%08X cyc=%llu from_ram=%u",
      cd.base_addr, cd.transfer_words, cd.first_addr, cd.last_addr,
      cd.block_ctrl, cd.channel_ctrl,
      static_cast<unsigned long long>(cd.cpu_cycle), cd.from_ram ? 1u : 0u);
}

bool is_plausible_exec_addr(u32 addr) {
  if (addr >= 0x00000000u && addr < 0x00800000u) {
    return true;
  }
  if (addr >= 0x80000000u && addr < 0x80800000u) {
    return true;
  }
  if (addr >= 0xA0000000u && addr < 0xA0800000u) {
    return true;
  }
  if (addr >= 0xBFC00000u && addr < 0xBFC80000u) {
    return true;
  }
  return false;
}

bool is_low_helper_addr(u32 addr) {
  const u32 phys = addr & 0x1FFFFFFFu;
  return phys >= 0x00002400u && phys <= 0x00002450u;
}

void log_stack_window(System *sys, const char *label, u32 sp) {
  if (sys == nullptr || (sp & 3u) != 0u || !is_plausible_exec_addr(sp)) {
    return;
  }

  LOG_WARN(
      "%s sp=0x%08X [sp+10]=0x%08X [sp+14]=0x%08X [sp+18]=0x%08X [sp+1C]=0x%08X",
      label, sp, sys->read32(sp + 0x10u), sys->read32(sp + 0x14u),
      sys->read32(sp + 0x18u), sys->read32(sp + 0x1Cu));
}

void log_low_helper_call(System *sys, u32 pc, u32 ret_pc, const u32 *gpr,
                         u32 target, const char *kind) {
  static u32 last_pc = 0xFFFFFFFFu;
  static u32 last_target = 0xFFFFFFFFu;
  if (sys == nullptr) {
    return;
  }
  if (pc == last_pc && target == last_target) {
    return;
  }
  last_pc = pc;
  last_target = target;

  const u32 base = pc & 0x1FFFFFFFu;
  LOG_WARN(
      "CPU: low-helper call kind=%s pc=0x%08X target=0x%08X ret=0x%08X sp=0x%08X ra=0x%08X",
      kind, pc, target, ret_pc, gpr[29], gpr[31]);
  LOG_WARN(
      "CPU: caller code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
      base - 0x08u, sys->read32(base - 0x08u), base - 0x04u,
      sys->read32(base - 0x04u), base + 0x00u, sys->read32(base + 0x00u),
      base + 0x04u, sys->read32(base + 0x04u), base + 0x08u,
      sys->read32(base + 0x08u));
  log_stack_window(sys, "CPU: low-helper caller frame", gpr[29]);
}

void log_suspicious_jump(System *sys, u32 pc, const u32 *gpr, u32 target,
                         u32 rs_index) {
  static u32 last_pc = 0xFFFFFFFFu;
  static u32 last_target = 0xFFFFFFFFu;
  if (sys == nullptr) {
    return;
  }
  if (pc == last_pc && target == last_target) {
    return;
  }
  last_pc = pc;
  last_target = target;

  const u32 sp = gpr[29];
  u32 sp0 = 0;
  u32 sp4 = 0;
  u32 sp8 = 0;
  if ((sp & 3u) == 0u && is_plausible_exec_addr(sp)) {
    sp0 = sys->read32(sp + 0);
    sp4 = sys->read32(sp + 4);
    sp8 = sys->read32(sp + 8);
  }

  LOG_WARN(
      "CPU: suspicious jump pc=0x%08X rs=r%u target=0x%08X sp=0x%08X ra=0x%08X stack=[%08X,%08X,%08X]",
      pc, rs_index, target, sp, gpr[31], sp0, sp4, sp8);
}

void log_zero_target_jump(System *sys, u32 pc, const u32 *gpr, u32 rs_index) {
  static u32 last_pc = 0xFFFFFFFFu;
  if (sys == nullptr || pc == last_pc) {
    return;
  }
  last_pc = pc;

  const u32 base = pc & 0x1FFFFFFFu;
  LOG_WARN("CPU: zero-target jump pc=0x%08X rs=r%u sp=0x%08X ra=0x%08X", pc,
           rs_index, gpr[29], gpr[31]);
  LOG_WARN(
      "CPU: zero-target code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
      base - 0x08u, sys->read32(base - 0x08u), base - 0x04u,
      sys->read32(base - 0x04u), base + 0x00u, sys->read32(base + 0x00u),
      base + 0x04u, sys->read32(base + 0x04u), base + 0x08u,
      sys->read32(base + 0x08u));
  log_stack_window(sys, "CPU: zero-target frame", gpr[29]);
  sys->debug_log_recent_ram_writes(gpr[29], 0x40u);
  log_dma_context(sys, pc, sys->read32(base), gpr);
}

void log_zero_pc_execution(System *sys, u32 instr, const u32 *gpr, u32 sr,
                           u32 cause, u32 epc) {
  static bool logged = false;
  if (sys == nullptr || logged) {
    return;
  }
  logged = true;

  LOG_WARN(
      "CPU: executing at PC=0x00000000 instr=0x%08X sr=0x%08X cause=0x%08X epc=0x%08X sp=0x%08X ra=0x%08X",
      instr, sr, cause, epc, gpr[29], gpr[31]);
  LOG_WARN(
      "CPU: low RAM %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
      0x00000000u, sys->read32(0x00000000u), 0x00000004u,
      sys->read32(0x00000004u), 0x00000008u, sys->read32(0x00000008u),
      0x0000000Cu, sys->read32(0x0000000Cu));
  log_stack_window(sys, "CPU: pc0 frame", gpr[29]);
  log_dma_context(sys, 0x00000000u, instr, gpr);
}

void log_suspicious_ra_load(System *sys, u32 pc, const u32 *gpr, u32 addr,
                            u32 value) {
  static u32 last_pc = 0xFFFFFFFFu;
  static u32 last_value = 0xFFFFFFFFu;
  if (sys == nullptr) {
    return;
  }
  if (pc == last_pc && value == last_value) {
    return;
  }
  last_pc = pc;
  last_value = value;

  LOG_WARN(
      "CPU: suspicious ra load pc=0x%08X addr=0x%08X val=0x%08X sp=0x%08X ra=0x%08X",
      pc, addr, value, gpr[29], gpr[31]);
  sys->debug_log_recent_ram_writes(addr, 0x20u);
}

void log_suspicious_sp_write(System *sys, u32 pc, u32 old_sp, u32 new_sp,
                             u32 ra) {
  static u32 last_pc = 0xFFFFFFFFu;
  static u32 last_new_sp = 0xFFFFFFFFu;
  if (sys == nullptr) {
    return;
  }
  if (pc == last_pc && new_sp == last_new_sp) {
    return;
  }
  last_pc = pc;
  last_new_sp = new_sp;

  LOG_WARN(
      "CPU: suspicious sp write pc=0x%08X old_sp=0x%08X new_sp=0x%08X ra=0x%08X",
      pc, old_sp, new_sp, ra);

  if ((pc >= 0x00002440u && pc <= 0x00002458u) ||
      (pc >= 0x80002440u && pc <= 0x80002458u)) {
    static bool dumped = false;
    if (!dumped) {
      dumped = true;
      const u32 base = pc & 0x1FFFFFFFu;
      const u32 line0 = base - 0x08u;
      const u32 line1 = base - 0x04u;
      const u32 line2 = base + 0x00u;
      const u32 line3 = base + 0x04u;
      const u32 line4 = base + 0x08u;
      LOG_WARN(
          "CPU: nearby code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          line0, sys->read32(line0), line1, sys->read32(line1), line2,
          sys->read32(line2), line3, sys->read32(line3), line4,
          sys->read32(line4));
    }
  }
}

void log_high_mirror_sp_write(System *sys, u32 pc, u32 old_sp, u32 new_sp,
                              u32 ra) {
  static bool logged = false;
  if (sys == nullptr || logged) {
    return;
  }
  logged = true;
  LOG_WARN(
      "CPU: high-mirror sp write pc=0x%08X old_sp=0x%08X new_sp=0x%08X ra=0x%08X",
      pc, old_sp, new_sp, ra);
  const u32 base = pc & 0x1FFFFFFFu;
  LOG_WARN(
      "CPU: high-mirror code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
      base - 0x08u, sys->read32(base - 0x08u), base - 0x04u,
      sys->read32(base - 0x04u), base + 0x00u, sys->read32(base + 0x00u),
      base + 0x04u, sys->read32(base + 0x04u), base + 0x08u,
      sys->read32(base + 0x08u));
}

void log_suspicious_ra_write(System *sys, u32 pc, u32 old_ra, u32 new_ra,
                             u32 sp) {
  static u32 last_pc = 0xFFFFFFFFu;
  static u32 last_new_ra = 0xFFFFFFFFu;
  if (sys == nullptr) {
    return;
  }
  if (pc == last_pc && new_ra == last_new_ra) {
    return;
  }
  last_pc = pc;
  last_new_ra = new_ra;

  LOG_WARN(
      "CPU: suspicious ra write pc=0x%08X old_ra=0x%08X new_ra=0x%08X sp=0x%08X",
      pc, old_ra, new_ra, sp);
  if (new_ra == 0u) {
    sys->debug_log_recent_ram_writes(sp, 0x40u);
  }

  if (((old_ra & 0x1FFFFFFFu) == 0x000152ACu) ||
      ((new_ra & 0x1FFFFFFFu) == 0x000152ACu)) {
    static bool dumped_low_helper_caller = false;
    if (!dumped_low_helper_caller) {
      dumped_low_helper_caller = true;
      LOG_WARN(
          "CPU: low-helper caller code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x0001529Cu, sys->read32(0x0001529Cu), 0x000152A0u,
          sys->read32(0x000152A0u), 0x000152A4u, sys->read32(0x000152A4u),
          0x000152A8u, sys->read32(0x000152A8u), 0x000152ACu,
          sys->read32(0x000152ACu));
      LOG_WARN(
          "CPU: low-helper caller code %08X=%08X %08X=%08X %08X=%08X",
          0x000152B0u, sys->read32(0x000152B0u), 0x000152B4u,
          sys->read32(0x000152B4u), 0x000152B8u, sys->read32(0x000152B8u));
      log_stack_window(sys, "CPU: low-helper saved-ra frame", sp);
    }
  }

  if ((pc >= 0x80043920u && pc <= 0x80043940u) ||
      (pc >= 0x00043920u && pc <= 0x00043940u)) {
    static bool dumped = false;
    if (!dumped) {
      dumped = true;
      const u32 base = pc & 0x1FFFFFFFu;
      LOG_WARN(
          "CPU: ra-write code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          base - 0x08u, sys->read32(base - 0x08u), base - 0x04u,
          sys->read32(base - 0x04u), base + 0x00u, sys->read32(base + 0x00u),
          base + 0x04u, sys->read32(base + 0x04u), base + 0x08u,
          sys->read32(base + 0x08u));
    }
  }
}

const char *exception_name(Exception cause) {
  switch (cause) {
  case Exception::Interrupt:
    return "Interrupt";
  case Exception::AddrLoadErr:
    return "AddrLoadErr";
  case Exception::AddrStoreErr:
    return "AddrStoreErr";
  case Exception::BusErrInstr:
    return "BusErrInstr";
  case Exception::BusErrData:
    return "BusErrData";
  case Exception::Syscall:
    return "Syscall";
  case Exception::Break:
    return "Break";
  case Exception::ReservedInst:
    return "ReservedInst";
  case Exception::CopUnusable:
    return "CopUnusable";
  case Exception::Overflow:
    return "Overflow";
  default:
    return "Unknown";
  }
}
} // namespace

// ── Init / Reset ───────────────────────────────────────────────────

void Cpu::init(System *sys) {
  sys_ = sys;
  reset();
}

void Cpu::reset() {
  std::memset(gpr_, 0, sizeof(gpr_));
  pc_ = 0xBFC00000; // BIOS entry point
  next_pc_ = pc_ + 4;
  current_pc_ = 0;
  hi_ = 0;
  lo_ = 0;
  load_ = {};
  next_load_ = {};
  in_delay_slot_ = false;
  pending_delay_slot_ = false;
  pending_branch_taken_ = false;
  pending_branch_pc_ = 0;
  active_branch_pc_ = 0;
  exception_raised_ = false;
  // On reset, BEV must be set so exceptions vector to BIOS (0xBFC00180).
  cop0_sr_ = (1u << 22);
  cop0_cause_ = 0;
  cop0_epc_ = 0;
  cop0_badvaddr_ = 0;
  cop0_jumpdest_ = 0;
  cycles_ = 0;
  std::memset(cop0_regs_, 0, sizeof(cop0_regs_));
}

// ── Register Helpers ───────────────────────────────────────────────

void Cpu::set_reg(u32 index, u32 value) {
  if (index == 0) {
    gpr_[0] = 0;
    return;
  }

  if (cpu_diag_enabled()) {
    if (index == 29) {
      const bool entered_high_mirror =
          (value >= 0x807F0000u && value < 0x80800000u) &&
          (gpr_[29] < 0x807F0000u || gpr_[29] >= 0x80800000u);
      if (entered_high_mirror) {
        log_high_mirror_sp_write(sys_, current_pc_, gpr_[29], value, gpr_[31]);
      }

      const bool suspicious_sp =
          (value >= 0x80800000u && value < 0x81000000u) ||
          (value < 0x80000000u && value >= 0x00800000u);
      if (suspicious_sp) {
        log_suspicious_sp_write(sys_, current_pc_, gpr_[29], value, gpr_[31]);
      }
    }

    if (index == 31) {
      const bool suspicious_ra =
          (value == 0u) ||
          (!is_plausible_exec_addr(value) && value != next_pc_);
      if (suspicious_ra) {
        log_suspicious_ra_write(sys_, current_pc_, gpr_[31], value, gpr_[29]);
      }
    }
  }

  // Cancel any pending load to the same register
  if (load_.reg == index) {
    load_.reg = 0;
  }
  if (next_load_.reg == index) {
    next_load_.reg = 0;
  }
  gpr_[index] = value;
}

u32 Cpu::read_cop0_reg(u32 index) const {
  switch (index) {
  case 6:
    return cop0_jumpdest_;
  case 8:
    return cop0_badvaddr_;
  case 12:
    return cop0_sr_;
  case 13:
    return cop0_cause_;
  case 14:
    return cop0_epc_;
  case 15:
    return 0x00000002; // PRId: R3000A
  default:
    return cop0_regs_[index & 31];
  }
}

void Cpu::write_cop0_reg(u32 index, u32 value) {
  switch (index) {
  case 3:
  case 5:
  case 7:
  case 9:
  case 11:
    cop0_regs_[index] = value;
    break;
  case 6:
    cop0_jumpdest_ = value;
    break;
  case 8:
    cop0_badvaddr_ = value;
    break;
  case 12:
    cop0_sr_ = value;
    break;
  case 13:
    // Only SW interrupt pending bits are writable.
    cop0_cause_ = (cop0_cause_ & ~0x300u) | (value & 0x300u);
    break;
  case 14:
    cop0_epc_ = value;
    break;
  default:
    cop0_regs_[index & 31] = value;
    break;
  }
}

void Cpu::raise_cop_unusable(u32 cop_index) {
  cop0_cause_ = (cop0_cause_ & ~(0x3u << 28)) | ((cop_index & 0x3u) << 28);
  exception(Exception::CopUnusable);
}

void Cpu::apply_pending_load() {
  if (cpu_diag_enabled() && load_.reg == 31) {
    const bool suspicious_ra =
        (load_.value == 0u) || !is_plausible_exec_addr(load_.value);
    if (suspicious_ra) {
      log_suspicious_ra_write(sys_, current_pc_, gpr_[31], load_.value,
                              gpr_[29]);
    }
  }
  if (load_.reg != 0) {
    gpr_[load_.reg] = load_.value;
  }
  load_ = {0, 0};
}

void Cpu::schedule_load(u32 index, u32 value) {
  if (index == 0) {
    next_load_ = {0, 0};
    return;
  }
  next_load_ = {index, value};
}

void Cpu::begin_branch(bool taken, u32 target) {
  pending_delay_slot_ = true;
  pending_branch_taken_ = taken;
  pending_branch_pc_ = current_pc_;
  if (taken) {
    next_pc_ = target;
  }
}

// ── Memory Access ──────────────────────────────────────────────────

u32 Cpu::fetch32(u32 addr) {
  if (addr & 3) {
    cop0_badvaddr_ = addr;
    exception(Exception::AddrLoadErr);
    return 0;
  }
  return sys_->read32_instruction(addr);
}

u32 Cpu::load32(u32 addr) {
  if (addr & 3) {
    cop0_badvaddr_ = addr;
    exception(Exception::AddrLoadErr);
    return 0;
  }
  return sys_->read32_data(addr);
}

u16 Cpu::load16(u32 addr) {
  if (addr & 1) {
    cop0_badvaddr_ = addr;
    exception(Exception::AddrLoadErr);
    return 0;
  }
  return sys_->read16_data(addr);
}

u8 Cpu::load8(u32 addr) { return sys_->read8_data(addr); }

void Cpu::store32(u32 addr, u32 value) {
  if (addr & 3) {
    cop0_badvaddr_ = addr;
    exception(Exception::AddrStoreErr);
    return;
  }
  // Check if cache is isolated (COP0 SR bit 16)
  if (cop0_sr_ & (1u << 16)) {
    return; // Cache isolated, writes go nowhere
  }
  sys_->write32_data(addr, value);
}

void Cpu::store16(u32 addr, u16 value) {
  if (addr & 1) {
    cop0_badvaddr_ = addr;
    exception(Exception::AddrStoreErr);
    return;
  }
  if (cop0_sr_ & (1u << 16))
    return;
  sys_->write16_data(addr, value);
}

void Cpu::store8(u32 addr, u8 value) {
  if (cop0_sr_ & (1u << 16))
    return;
  sys_->write8_data(addr, value);
}

// ── Exception Handling ─────────────────────────────────────────────

void Cpu::exception(Exception cause) {
  static u32 logged_exception_count = 0;
  exception_raised_ = true;

  u32 handler;
  if (cop0_sr_ & (1u << 22)) { // BEV bit
    handler = 0xBFC00180;
  } else {
    // PS1/R3000A general exception vector when BEV=0.
    handler = 0x80000080;
  }

  // Shift the Interrupt Enable/Kernel-User mode stack in SR
  u32 mode = cop0_sr_ & 0x3F;
  cop0_sr_ &= ~0x3Fu;
  cop0_sr_ |= (mode << 2) & 0x3F;

  // Set cause register
  if (cause != Exception::CopUnusable) {
    cop0_cause_ &= ~(0x3u << 28); // CE field is only meaningful for CopUnusable.
  }
  cop0_cause_ = (cop0_cause_ & ~0x7Cu) | (static_cast<u32>(cause) << 2);

  // Set EPC
  if (in_delay_slot_) {
    cop0_epc_ = active_branch_pc_ != 0 ? active_branch_pc_ : (current_pc_ - 4);
    cop0_cause_ |= (1u << 31); // BD (Branch Delay) bit
  } else {
    cop0_epc_ = current_pc_;
    cop0_cause_ &= ~(1u << 31);
  }

  // Jump to exception handler
  if (cause != Exception::Interrupt && logged_exception_count < 32) {
    ++logged_exception_count;
    LOG_WARN(
        "CPU: exception cause=%s pc=0x%08X handler=0x%08X sr=0x%08X sp=0x%08X ra=0x%08X",
        exception_name(cause), current_pc_, handler, cop0_sr_, gpr_[29],
        gpr_[31]);
  }

  pc_ = handler;
  next_pc_ = handler + 4;
  in_delay_slot_ = false;
  pending_delay_slot_ = false;
  pending_branch_taken_ = false;
  pending_branch_pc_ = 0;
  active_branch_pc_ = 0;
}

bool Cpu::check_irq() {
  // Check if interrupts are enabled (IEc bit, bit 0 of SR)
  bool iec = cop0_sr_ & 1;
  if (!iec)
    return false;

  // Check if any interrupt is pending and unmasked
  // COP0 Cause bits 8-15 (IP) & SR bits 8-15 (IM)
  u32 pending = cop0_cause_ & cop0_sr_ & 0xFF00;
  return pending != 0;
}

// ── Main Step ──────────────────────────────────────────────────────

u32 Cpu::step() {
  static u64 trace_step_counter = 0;
  static u32 trace_last_pc = 0;
  static u32 trace_last_instr = 0;
  static u64 trace_repeat = 0;
  static u32 prev_pc_for_diag = 0;
  const bool cpu_diag = cpu_diag_enabled();
  current_pc_ = pc_;
  exception_raised_ = false;
  next_load_ = {0, 0};
  in_delay_slot_ = pending_delay_slot_;
  active_branch_pc_ = pending_branch_pc_;
  pending_delay_slot_ = false;
  pending_branch_taken_ = false;
  pending_branch_pc_ = 0;

  // Check for hardware interrupts
  if (sys_->irq_pending()) {
    cop0_cause_ |= (1u << 10); // Set IP2 (hardware interrupt line)
  } else {
    cop0_cause_ &= ~(1u << 10);
  }

  if (check_irq()) {
    exception(Exception::Interrupt);
    apply_pending_load();
    constexpr u32 irq_cycles = 2;
    cycles_ += irq_cycles;
    return irq_cycles;
  }

  // Fetch instruction
  u32 instruction = fetch32(pc_);
  if (exception_raised_) {
    apply_pending_load();
    constexpr u32 fault_cycles = 2;
    cycles_ += fault_cycles;
    return fault_cycles;
  }

  if (cpu_diag) {
    if (current_pc_ >= 0xBFC02B58u && current_pc_ <= 0xBFC02B7Cu) {
      static bool logged_bios_low_stub_loop = false;
      if (!logged_bios_low_stub_loop) {
        logged_bios_low_stub_loop = true;
        LOG_WARN(
            "CPU: bios low-stub loop prev_pc=0x%08X pc=0x%08X instr=0x%08X sp=0x%08X ra=0x%08X",
            prev_pc_for_diag, current_pc_, instruction, gpr_[29], gpr_[31]);
        LOG_WARN(
            "CPU: bios low-stub regs a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X t0=0x%08X t1=0x%08X t2=0x%08X t3=0x%08X t4=0x%08X t5=0x%08X t6=0x%08X t7=0x%08X",
            gpr_[4], gpr_[5], gpr_[6], gpr_[7], gpr_[8], gpr_[9], gpr_[10],
            gpr_[11], gpr_[12], gpr_[13], gpr_[14], gpr_[15]);
        LOG_WARN(
            "CPU: bios low-stub code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
            0x1FC02B58u, sys_->read32(0x1FC02B58u), 0x1FC02B5Cu,
            sys_->read32(0x1FC02B5Cu), 0x1FC02B60u, sys_->read32(0x1FC02B60u),
            0x1FC02B64u, sys_->read32(0x1FC02B64u), 0x1FC02B68u,
            sys_->read32(0x1FC02B68u));
        LOG_WARN(
            "CPU: bios low-stub code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
            0x1FC02B6Cu, sys_->read32(0x1FC02B6Cu), 0x1FC02B70u,
            sys_->read32(0x1FC02B70u), 0x1FC02B74u, sys_->read32(0x1FC02B74u),
            0x1FC02B78u, sys_->read32(0x1FC02B78u), 0x1FC02B7Cu,
            sys_->read32(0x1FC02B7Cu));
      }
    }
    if (current_pc_ != 0u && !is_plausible_exec_addr(current_pc_)) {
      static bool logged_implausible_pc_entry = false;
      if (!logged_implausible_pc_entry) {
        logged_implausible_pc_entry = true;
        LOG_WARN(
            "CPU: implausible pc entry prev_pc=0x%08X pc=0x%08X instr=0x%08X branch_pc=0x%08X sp=0x%08X ra=0x%08X",
            prev_pc_for_diag, current_pc_, instruction, active_branch_pc_,
            gpr_[29], gpr_[31]);
        if (prev_pc_for_diag != 0u) {
          const u32 prev_base = prev_pc_for_diag & 0x1FFFFFFFu;
          LOG_WARN(
              "CPU: prev code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
              prev_base - 0x08u, sys_->read32(prev_base - 0x08u),
              prev_base - 0x04u, sys_->read32(prev_base - 0x04u), prev_base,
              sys_->read32(prev_base), prev_base + 0x04u,
              sys_->read32(prev_base + 0x04u), prev_base + 0x08u,
              sys_->read32(prev_base + 0x08u));
          sys_->debug_log_recent_ram_writes(prev_base, 0x20u);
        }
        if (active_branch_pc_ != 0u) {
          const u32 branch_base = active_branch_pc_ & 0x1FFFFFFFu;
          LOG_WARN(
              "CPU: branch code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
              branch_base - 0x08u, sys_->read32(branch_base - 0x08u),
              branch_base - 0x04u, sys_->read32(branch_base - 0x04u),
              branch_base, sys_->read32(branch_base), branch_base + 0x04u,
              sys_->read32(branch_base + 0x04u), branch_base + 0x08u,
              sys_->read32(branch_base + 0x08u));
          sys_->debug_log_recent_ram_writes(branch_base, 0x20u);
        }
        log_stack_window(sys_, "CPU: implausible pc frame", gpr_[29]);
      }
    }
    if (current_pc_ == 0x00000000u) {
      log_zero_pc_execution(sys_, instruction, gpr_, cop0_sr_, cop0_cause_,
                            cop0_epc_);
    }
    if (current_pc_ >= 0x80015298u && current_pc_ <= 0x800152B8u) {
    static bool logged_low_helper_caller_entry = false;
    if (!logged_low_helper_caller_entry) {
      logged_low_helper_caller_entry = true;
      LOG_WARN(
          "CPU: near low-helper caller prev_pc=0x%08X pc=0x%08X instr=0x%08X sp=0x%08X ra=0x%08X",
          prev_pc_for_diag, current_pc_, instruction, gpr_[29], gpr_[31]);
      LOG_WARN(
          "CPU: caller code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x0001529Cu, sys_->read32(0x0001529Cu), 0x000152A0u,
          sys_->read32(0x000152A0u), 0x000152A4u, sys_->read32(0x000152A4u),
          0x000152A8u, sys_->read32(0x000152A8u), 0x000152ACu,
          sys_->read32(0x000152ACu));
      LOG_WARN(
          "CPU: caller code %08X=%08X %08X=%08X %08X=%08X",
          0x000152B0u, sys_->read32(0x000152B0u), 0x000152B4u,
          sys_->read32(0x000152B4u), 0x000152B8u, sys_->read32(0x000152B8u));
      log_stack_window(sys_, "CPU: caller frame", gpr_[29]);
    }
    }
    if (current_pc_ >= 0x8001EC98u && current_pc_ <= 0x8001ECC0u) {
    static bool logged_low_helper_chain_entry = false;
    if (!logged_low_helper_chain_entry) {
      logged_low_helper_chain_entry = true;
      LOG_WARN(
          "CPU: near 0x8001ECA4 chain prev_pc=0x%08X pc=0x%08X instr=0x%08X sp=0x%08X ra=0x%08X",
          prev_pc_for_diag, current_pc_, instruction, gpr_[29], gpr_[31]);
      LOG_WARN(
          "CPU: chain code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x0001EC9Cu, sys_->read32(0x0001EC9Cu), 0x0001ECA0u,
          sys_->read32(0x0001ECA0u), 0x0001ECA4u, sys_->read32(0x0001ECA4u),
          0x0001ECA8u, sys_->read32(0x0001ECA8u), 0x0001ECACu,
          sys_->read32(0x0001ECACu));
      LOG_WARN(
          "CPU: chain code %08X=%08X %08X=%08X %08X=%08X",
          0x0001ECB0u, sys_->read32(0x0001ECB0u), 0x0001ECB4u,
          sys_->read32(0x0001ECB4u), 0x0001ECB8u, sys_->read32(0x0001ECB8u));
      log_stack_window(sys_, "CPU: chain frame", gpr_[29]);
    }
    }
    if (current_pc_ >= 0x8001ECBCu && current_pc_ <= 0x8001ED10u) {
    static bool logged_low_helper_chain_tail = false;
    if (!logged_low_helper_chain_tail) {
      logged_low_helper_chain_tail = true;
      LOG_WARN(
          "CPU: near low-helper chain tail prev_pc=0x%08X pc=0x%08X instr=0x%08X sp=0x%08X ra=0x%08X",
          prev_pc_for_diag, current_pc_, instruction, gpr_[29], gpr_[31]);
      LOG_WARN(
          "CPU: chain tail %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x0001ECBCu, sys_->read32(0x0001ECBCu), 0x0001ECC0u,
          sys_->read32(0x0001ECC0u), 0x0001ECC4u, sys_->read32(0x0001ECC4u),
          0x0001ECC8u, sys_->read32(0x0001ECC8u), 0x0001ECCCu,
          sys_->read32(0x0001ECCCu));
      LOG_WARN(
          "CPU: chain tail %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x0001ECD0u, sys_->read32(0x0001ECD0u), 0x0001ECD4u,
          sys_->read32(0x0001ECD4u), 0x0001ECD8u, sys_->read32(0x0001ECD8u),
          0x0001ECDCu, sys_->read32(0x0001ECDCu), 0x0001ECE0u,
          sys_->read32(0x0001ECE0u));
      LOG_WARN(
          "CPU: chain tail %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x0001ECE4u, sys_->read32(0x0001ECE4u), 0x0001ECE8u,
          sys_->read32(0x0001ECE8u), 0x0001ECECu, sys_->read32(0x0001ECECu),
          0x0001ECF0u, sys_->read32(0x0001ECF0u));
      LOG_WARN(
          "CPU: chain tail %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x0001ECF4u, sys_->read32(0x0001ECF4u), 0x0001ECF8u,
          sys_->read32(0x0001ECF8u), 0x0001ECFCu, sys_->read32(0x0001ECFCu),
          0x0001ED00u, sys_->read32(0x0001ED00u), 0x0001ED04u,
          sys_->read32(0x0001ED04u));
      LOG_WARN(
          "CPU: chain tail %08X=%08X %08X=%08X %08X=%08X",
          0x0001ED08u, sys_->read32(0x0001ED08u), 0x0001ED0Cu,
          sys_->read32(0x0001ED0Cu), 0x0001ED10u, sys_->read32(0x0001ED10u));
      LOG_WARN(
          "CPU: chain tail regs a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X t0=0x%08X t1=0x%08X",
          gpr_[4], gpr_[5], gpr_[6], gpr_[7], gpr_[8], gpr_[9]);
      log_stack_window(sys_, "CPU: chain tail frame", gpr_[29]);
    }
    }
    if (current_pc_ >= 0x8001EE88u && current_pc_ <= 0x8001EF30u) {
    static bool logged_stack_zero_path = false;
    if (!logged_stack_zero_path) {
      logged_stack_zero_path = true;
      LOG_WARN(
          "CPU: near stack-zero helper prev_pc=0x%08X pc=0x%08X instr=0x%08X sp=0x%08X ra=0x%08X",
          prev_pc_for_diag, current_pc_, instruction, gpr_[29], gpr_[31]);
      LOG_WARN(
          "CPU: stack-zero regs a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X t0=0x%08X t1=0x%08X t2=0x%08X t3=0x%08X v0=0x%08X v1=0x%08X",
          gpr_[4], gpr_[5], gpr_[6], gpr_[7], gpr_[8], gpr_[9], gpr_[10],
          gpr_[11], gpr_[2], gpr_[3]);
      LOG_WARN(
          "CPU: stack-zero code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x0001EE88u, sys_->read32(0x0001EE88u), 0x0001EE8Cu,
          sys_->read32(0x0001EE8Cu), 0x0001EE90u, sys_->read32(0x0001EE90u),
          0x0001EE94u, sys_->read32(0x0001EE94u), 0x0001EE98u,
          sys_->read32(0x0001EE98u));
      LOG_WARN(
          "CPU: stack-zero code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x0001EE9Cu, sys_->read32(0x0001EE9Cu), 0x0001EEA0u,
          sys_->read32(0x0001EEA0u), 0x0001EEA4u, sys_->read32(0x0001EEA4u),
          0x0001EEA8u, sys_->read32(0x0001EEA8u), 0x0001EEACu,
          sys_->read32(0x0001EEACu));
      LOG_WARN(
          "CPU: stack-zero code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x0001EEB0u, sys_->read32(0x0001EEB0u), 0x0001EEB4u,
          sys_->read32(0x0001EEB4u), 0x0001EEB8u, sys_->read32(0x0001EEB8u),
          0x0001EEBCu, sys_->read32(0x0001EEBCu), 0x0001EEC0u,
          sys_->read32(0x0001EEC0u));
      LOG_WARN(
          "CPU: stack-zero code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x0001EEC4u, sys_->read32(0x0001EEC4u), 0x0001EEC8u,
          sys_->read32(0x0001EEC8u), 0x0001EECCu, sys_->read32(0x0001EECCu),
          0x0001EED0u, sys_->read32(0x0001EED0u), 0x0001EED4u,
          sys_->read32(0x0001EED4u));
      LOG_WARN(
          "CPU: stack-zero code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x0001EED8u, sys_->read32(0x0001EED8u), 0x0001EEDCu,
          sys_->read32(0x0001EEDCu), 0x0001EEE0u, sys_->read32(0x0001EEE0u),
          0x0001EEE4u, sys_->read32(0x0001EEE4u), 0x0001EEE8u,
          sys_->read32(0x0001EEE8u));
      LOG_WARN(
          "CPU: stack-zero code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x0001EEECu, sys_->read32(0x0001EEECu), 0x0001EEF0u,
          sys_->read32(0x0001EEF0u), 0x0001EEF4u, sys_->read32(0x0001EEF4u),
          0x0001EEF8u, sys_->read32(0x0001EEF8u), 0x0001EEFCu,
          sys_->read32(0x0001EEFCu));
      LOG_WARN(
          "CPU: stack-zero code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x0001EF00u, sys_->read32(0x0001EF00u), 0x0001EF04u,
          sys_->read32(0x0001EF04u), 0x0001EF08u, sys_->read32(0x0001EF08u),
          0x0001EF0Cu, sys_->read32(0x0001EF0Cu), 0x0001EF10u,
          sys_->read32(0x0001EF10u));
      LOG_WARN(
          "CPU: stack-zero code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x0001EF14u, sys_->read32(0x0001EF14u), 0x0001EF18u,
          sys_->read32(0x0001EF18u), 0x0001EF1Cu, sys_->read32(0x0001EF1Cu),
          0x0001EF20u, sys_->read32(0x0001EF20u), 0x0001EF24u,
          sys_->read32(0x0001EF24u));
      LOG_WARN(
          "CPU: stack-zero code %08X=%08X %08X=%08X %08X=%08X",
          0x0001EF28u, sys_->read32(0x0001EF28u), 0x0001EF2Cu,
          sys_->read32(0x0001EF2Cu), 0x0001EF30u, sys_->read32(0x0001EF30u));
      log_stack_window(sys_, "CPU: stack-zero frame", gpr_[29]);
      sys_->debug_log_recent_ram_writes(gpr_[4], 0x40u);
      sys_->debug_log_recent_ram_writes(gpr_[5], 0x40u);
    }
    }
    if (current_pc_ >= 0x8001D9F0u && current_pc_ <= 0x8001DA24u) {
    static bool logged_source_prep_head = false;
    if (!logged_source_prep_head) {
      logged_source_prep_head = true;
      LOG_WARN(
          "CPU: near source-prep head prev_pc=0x%08X pc=0x%08X instr=0x%08X sp=0x%08X ra=0x%08X",
          prev_pc_for_diag, current_pc_, instruction, gpr_[29], gpr_[31]);
      LOG_WARN(
          "CPU: source-prep regs a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X t0=0x%08X t1=0x%08X t2=0x%08X t3=0x%08X s0=0x%08X s1=0x%08X",
          gpr_[4], gpr_[5], gpr_[6], gpr_[7], gpr_[8], gpr_[9], gpr_[10],
          gpr_[11], gpr_[16], gpr_[17]);
      LOG_WARN(
          "CPU: source-prep code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x0001D9F0u, sys_->read32(0x0001D9F0u), 0x0001D9F4u,
          sys_->read32(0x0001D9F4u), 0x0001D9F8u, sys_->read32(0x0001D9F8u),
          0x0001D9FCu, sys_->read32(0x0001D9FCu), 0x0001DA00u,
          sys_->read32(0x0001DA00u));
      LOG_WARN(
          "CPU: source-prep code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x0001DA04u, sys_->read32(0x0001DA04u), 0x0001DA08u,
          sys_->read32(0x0001DA08u), 0x0001DA0Cu, sys_->read32(0x0001DA0Cu),
          0x0001DA10u, sys_->read32(0x0001DA10u), 0x0001DA14u,
          sys_->read32(0x0001DA14u));
      LOG_WARN(
          "CPU: source-prep ram %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x000F11E0u, sys_->read32(0x000F11E0u), 0x000F11E4u,
          sys_->read32(0x000F11E4u), 0x000F11E8u, sys_->read32(0x000F11E8u),
          0x000F11ECu, sys_->read32(0x000F11ECu));
      LOG_WARN(
          "CPU: source-prep ram %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x000F11F0u, sys_->read32(0x000F11F0u), 0x000F11F4u,
          sys_->read32(0x000F11F4u), 0x000F11F8u, sys_->read32(0x000F11F8u),
          0x000F11FCu, sys_->read32(0x000F11FCu));
      sys_->debug_log_recent_ram_writes(0x000F11E0u, 0x60u);
    }
    }
    if (current_pc_ >= 0x8001DAB8u && current_pc_ <= 0x8001DAE0u) {
    static bool logged_source_prep_tail = false;
    if (!logged_source_prep_tail) {
      logged_source_prep_tail = true;
      LOG_WARN(
          "CPU: near source-prep tail prev_pc=0x%08X pc=0x%08X instr=0x%08X sp=0x%08X ra=0x%08X",
          prev_pc_for_diag, current_pc_, instruction, gpr_[29], gpr_[31]);
      LOG_WARN(
          "CPU: source-tail regs a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X s0=0x%08X s1=0x%08X s2=0x%08X s3=0x%08X",
          gpr_[4], gpr_[5], gpr_[6], gpr_[7], gpr_[16], gpr_[17], gpr_[18],
          gpr_[19]);
      LOG_WARN(
          "CPU: source-tail code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x0001DAB8u, sys_->read32(0x0001DAB8u), 0x0001DABCu,
          sys_->read32(0x0001DABCu), 0x0001DAC0u, sys_->read32(0x0001DAC0u),
          0x0001DAC4u, sys_->read32(0x0001DAC4u), 0x0001DAC8u,
          sys_->read32(0x0001DAC8u));
      LOG_WARN(
          "CPU: source-tail code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x0001DACCu, sys_->read32(0x0001DACCu), 0x0001DAD0u,
          sys_->read32(0x0001DAD0u), 0x0001DAD4u, sys_->read32(0x0001DAD4u),
          0x0001DAD8u, sys_->read32(0x0001DAD8u), 0x0001DADCu,
          sys_->read32(0x0001DADCu));
      LOG_WARN(
          "CPU: source-tail ram %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x000F1200u, sys_->read32(0x000F1200u), 0x000F1204u,
          sys_->read32(0x000F1204u), 0x000F1240u, sys_->read32(0x000F1240u),
          0x000F1244u, sys_->read32(0x000F1244u));
      LOG_WARN(
          "CPU: source-tail ram %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x000F1248u, sys_->read32(0x000F1248u), 0x000F124Cu,
          sys_->read32(0x000F124Cu), 0x000F0A40u, sys_->read32(0x000F0A40u),
          0x000F0A44u, sys_->read32(0x000F0A44u));
      sys_->debug_log_recent_ram_writes(0x000F1240u, 0x80u);
      sys_->debug_log_recent_ram_writes(0x000F0A40u, 0x40u);
    }
    }
    if (current_pc_ >= 0x8001EF20u && current_pc_ <= 0x8001EF28u &&
        gpr_[5] >= 0x801FFF00u && gpr_[5] < 0x80200000u) {
    static bool logged_stack_zero_overrun = false;
    if (!logged_stack_zero_overrun) {
      logged_stack_zero_overrun = true;
      LOG_WARN(
          "CPU: stack-zero overrun pc=0x%08X instr=0x%08X sp=0x%08X ra=0x%08X a0=0x%08X a1=0x%08X v0=0x%08X v1=0x%08X t0=0x%08X t1=0x%08X t2=0x%08X t3=0x%08X",
          current_pc_, instruction, gpr_[29], gpr_[31], gpr_[4], gpr_[5],
          gpr_[2], gpr_[3], gpr_[8], gpr_[9], gpr_[10], gpr_[11]);
      LOG_WARN(
          "CPU: stack-zero src %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          (gpr_[4] + 0x00u) & 0x1FFFFFFFu, sys_->read32(gpr_[4] + 0x00u),
          (gpr_[4] + 0x04u) & 0x1FFFFFFFu, sys_->read32(gpr_[4] + 0x04u),
          (gpr_[4] + 0x08u) & 0x1FFFFFFFu, sys_->read32(gpr_[4] + 0x08u),
          (gpr_[4] + 0x0Cu) & 0x1FFFFFFFu, sys_->read32(gpr_[4] + 0x0Cu));
      LOG_WARN(
          "CPU: stack-zero dst %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          (gpr_[5] - 0x08u) & 0x1FFFFFFFu, sys_->read32(gpr_[5] - 0x08u),
          (gpr_[5] - 0x04u) & 0x1FFFFFFFu, sys_->read32(gpr_[5] - 0x04u),
          (gpr_[5] + 0x00u) & 0x1FFFFFFFu, sys_->read32(gpr_[5] + 0x00u),
          (gpr_[5] + 0x04u) & 0x1FFFFFFFu, sys_->read32(gpr_[5] + 0x04u));
    }
    }
    if (current_pc_ >= 0x80116CFCu && current_pc_ <= 0x80116D18u) {
    const bool came_from_outside =
        !(prev_pc_for_diag >= 0x80116CFCu && prev_pc_for_diag <= 0x80116D18u);
    const u32 dst_phys = gpr_[5] & 0x1FFFFFFFu;
    const bool dst_is_low_helper =
        (dst_phys >= 0x000023D0u && dst_phys <= 0x00002980u);
    const bool dst_is_stack_top =
        (gpr_[5] >= 0x801FFF00u && gpr_[5] < 0x80200000u);
    static bool logged_zero_helper_low = false;
    static bool logged_zero_helper_stack = false;
    const bool should_log_low = came_from_outside && dst_is_low_helper &&
                                !logged_zero_helper_low;
    const bool should_log_stack = came_from_outside && dst_is_stack_top &&
                                  !logged_zero_helper_stack;
    if (should_log_low || should_log_stack) {
      if (should_log_low) {
        logged_zero_helper_low = true;
      }
      if (should_log_stack) {
        logged_zero_helper_stack = true;
      }
      LOG_WARN(
          "CPU: zero-helper entry prev_pc=0x%08X pc=0x%08X instr=0x%08X sp=0x%08X ra=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X v0=0x%08X v1=0x%08X t0=0x%08X t1=0x%08X t2=0x%08X t3=0x%08X",
          prev_pc_for_diag, current_pc_, instruction, gpr_[29], gpr_[31],
          gpr_[4], gpr_[5], gpr_[6], gpr_[7], gpr_[2], gpr_[3], gpr_[8],
          gpr_[9], gpr_[10], gpr_[11]);
      LOG_WARN(
          "CPU: zero-helper code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x00116CFCu, sys_->read32(0x00116CFCu), 0x00116D00u,
          sys_->read32(0x00116D00u), 0x00116D04u, sys_->read32(0x00116D04u),
          0x00116D08u, sys_->read32(0x00116D08u), 0x00116D0Cu,
          sys_->read32(0x00116D0Cu), 0x00116D10u, sys_->read32(0x00116D10u));
      LOG_WARN(
          "CPU: zero-helper code %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x00116D14u, sys_->read32(0x00116D14u), 0x00116D18u,
          sys_->read32(0x00116D18u), 0x00116D1Cu, sys_->read32(0x00116D1Cu),
          0x00116D20u, sys_->read32(0x00116D20u));
      LOG_WARN(
          "CPU: zero-helper src %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          (gpr_[4] + 0x00u) & 0x1FFFFFFFu, sys_->read32(gpr_[4] + 0x00u),
          (gpr_[4] + 0x04u) & 0x1FFFFFFFu, sys_->read32(gpr_[4] + 0x04u),
          (gpr_[4] + 0x08u) & 0x1FFFFFFFu, sys_->read32(gpr_[4] + 0x08u),
          (gpr_[4] + 0x0Cu) & 0x1FFFFFFFu, sys_->read32(gpr_[4] + 0x0Cu));
      LOG_WARN(
          "CPU: zero-helper dst %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          (gpr_[5] - 0x08u) & 0x1FFFFFFFu, sys_->read32(gpr_[5] - 0x08u),
          (gpr_[5] - 0x04u) & 0x1FFFFFFFu, sys_->read32(gpr_[5] - 0x04u),
          (gpr_[5] + 0x00u) & 0x1FFFFFFFu, sys_->read32(gpr_[5] + 0x00u),
          (gpr_[5] + 0x04u) & 0x1FFFFFFFu, sys_->read32(gpr_[5] + 0x04u));
      sys_->debug_log_recent_ram_writes(gpr_[4], 0x40u);
      sys_->debug_log_recent_ram_writes(gpr_[5], 0x40u);
      if (dst_is_low_helper) {
        sys_->debug_log_recent_ram_writes(0x000023D0u, 0x80u);
        sys_->debug_log_recent_ram_writes(0x00002930u, 0x80u);
      }
    }
    }
    if (current_pc_ >= 0x000023D0u && current_pc_ <= 0x00002408u) {
    static bool logged_low_helper_leadin = false;
    if (!logged_low_helper_leadin) {
      logged_low_helper_leadin = true;
      LOG_WARN(
          "CPU: entered low-helper lead-in prev_pc=0x%08X pc=0x%08X instr=0x%08X sp=0x%08X ra=0x%08X",
          prev_pc_for_diag, current_pc_, instruction, gpr_[29], gpr_[31]);
      LOG_WARN(
          "CPU: low lead-in %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x000023D0u, sys_->read32(0x000023D0u), 0x000023D4u,
          sys_->read32(0x000023D4u), 0x000023D8u, sys_->read32(0x000023D8u),
          0x000023DCu, sys_->read32(0x000023DCu), 0x000023E0u,
          sys_->read32(0x000023E0u));
      LOG_WARN(
          "CPU: low lead-in %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x000023E4u, sys_->read32(0x000023E4u), 0x000023E8u,
          sys_->read32(0x000023E8u), 0x000023ECu, sys_->read32(0x000023ECu),
          0x000023F0u, sys_->read32(0x000023F0u), 0x000023F4u,
          sys_->read32(0x000023F4u));
      LOG_WARN(
          "CPU: low lead-in %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x000023F8u, sys_->read32(0x000023F8u), 0x000023FCu,
          sys_->read32(0x000023FCu), 0x00002400u, sys_->read32(0x00002400u),
          0x00002404u, sys_->read32(0x00002404u), 0x00002408u,
          sys_->read32(0x00002408u));
      LOG_WARN(
          "CPU: low lead-in regs a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X t6=0x%08X t7=0x%08X t8=0x%08X t9=0x%08X",
          gpr_[4], gpr_[5], gpr_[6], gpr_[7], gpr_[14], gpr_[15], gpr_[24],
          gpr_[25]);
      if (is_plausible_exec_addr(gpr_[6])) {
        LOG_WARN(
            "CPU: low lead-in a2 mem [0]=0x%08X [4]=0x%08X [8]=0x%08X [C]=0x%08X",
            sys_->read32(gpr_[6] + 0x00u), sys_->read32(gpr_[6] + 0x04u),
            sys_->read32(gpr_[6] + 0x08u), sys_->read32(gpr_[6] + 0x0Cu));
      }
      sys_->debug_log_recent_ram_writes(0x000023D0u, 0x80u);
      sys_->debug_log_recent_ram_writes(0x00002930u, 0x80u);
      log_stack_window(sys_, "CPU: low lead-in frame", gpr_[29]);
    }
    }
    if (current_pc_ >= 0x00002400u && current_pc_ <= 0x00002424u) {
    static bool logged_low_prologue_entry = false;
    if (!logged_low_prologue_entry) {
      logged_low_prologue_entry = true;
      LOG_WARN(
          "CPU: entered low-helper prologue prev_pc=0x%08X pc=0x%08X instr=0x%08X sp=0x%08X ra=0x%08X",
          prev_pc_for_diag, current_pc_, instruction, gpr_[29], gpr_[31]);
      LOG_WARN(
          "CPU: low-helper code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x00002400u, sys_->read32(0x00002400u), 0x00002404u,
          sys_->read32(0x00002404u), 0x00002408u, sys_->read32(0x00002408u),
          0x0000240Cu, sys_->read32(0x0000240Cu), 0x00002410u,
          sys_->read32(0x00002410u));
      LOG_WARN(
          "CPU: low-helper code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x00002414u, sys_->read32(0x00002414u), 0x00002418u,
          sys_->read32(0x00002418u), 0x0000241Cu, sys_->read32(0x0000241Cu),
          0x00002420u, sys_->read32(0x00002420u), 0x00002424u,
          sys_->read32(0x00002424u));
      log_stack_window(sys_, "CPU: low-helper prologue frame", gpr_[29]);
    }
    }
    if (current_pc_ >= 0x00002440u && current_pc_ <= 0x00002454u) {
    static bool logged_low_epilogue_entry = false;
    if (!logged_low_epilogue_entry) {
      logged_low_epilogue_entry = true;
      LOG_WARN(
          "CPU: entered 0x000024xx prev_pc=0x%08X pc=0x%08X instr=0x%08X sp=0x%08X ra=0x%08X",
          prev_pc_for_diag, current_pc_, instruction, gpr_[29], gpr_[31]);
      LOG_WARN(
          "CPU: low-epilogue code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x00002420u, sys_->read32(0x00002420u), 0x00002424u,
          sys_->read32(0x00002424u), 0x00002428u, sys_->read32(0x00002428u),
          0x0000242Cu, sys_->read32(0x0000242Cu), 0x00002430u,
          sys_->read32(0x00002430u), 0x00002434u, sys_->read32(0x00002434u),
          0x00002438u, sys_->read32(0x00002438u), 0x0000243Cu,
          sys_->read32(0x0000243Cu));
      LOG_WARN(
          "CPU: low-epilogue code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
          0x00002440u, sys_->read32(0x00002440u), 0x00002444u,
          sys_->read32(0x00002444u), 0x00002448u, sys_->read32(0x00002448u),
          0x0000244Cu, sys_->read32(0x0000244Cu), 0x00002450u,
          sys_->read32(0x00002450u));
    }
    }
    if (current_pc_ >= 0x00002420u && current_pc_ <= 0x00002450u) {
    static bool logged_low_func_call = false;
    const bool came_from_outside =
        !(prev_pc_for_diag >= 0x00002420u && prev_pc_for_diag <= 0x00002450u);
    if (!logged_low_func_call && came_from_outside) {
      logged_low_func_call = true;
      LOG_WARN(
          "CPU: entered low helper prev_pc=0x%08X pc=0x%08X sp=0x%08X ra=0x%08X v0=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X",
          prev_pc_for_diag, current_pc_, gpr_[29], gpr_[31], gpr_[2], gpr_[4],
          gpr_[5], gpr_[6], gpr_[7]);
    }
    }
  }

  if (g_trace_cpu) {
    ++trace_step_counter;
    const bool same_as_last =
        (trace_step_counter > 1 && current_pc_ == trace_last_pc &&
         instruction == trace_last_instr);
    if (same_as_last) {
      ++trace_repeat;
      const u32 repeat_stride = std::max<u32>(4096u, g_trace_stride_cpu);
      if ((trace_repeat % static_cast<u64>(repeat_stride)) == 0) {
        LOG_CAT_DEBUG(LogCategory::Cpu, "CPU: repeat pc=0x%08X instr=0x%08X repeats=%llu",
                  current_pc_, instruction,
                  static_cast<unsigned long long>(trace_repeat));
      }
    } else {
      if (trace_repeat > 0) {
        LOG_CAT_DEBUG(LogCategory::Cpu, "CPU: previous pc=0x%08X instr=0x%08X repeated %llu times",
                  trace_last_pc, trace_last_instr,
                  static_cast<unsigned long long>(trace_repeat));
        trace_repeat = 0;
      }
      if (trace_step_counter <= static_cast<u64>(g_trace_burst_cpu) ||
          (g_trace_stride_cpu > 0 &&
           (trace_step_counter % static_cast<u64>(g_trace_stride_cpu)) == 0)) {
        LOG_CAT_DEBUG(LogCategory::Cpu,
            "CPU: step=%llu pc=0x%08X instr=0x%08X sr=0x%08X cause=0x%08X",
            static_cast<unsigned long long>(trace_step_counter), current_pc_,
            instruction, cop0_sr_, cop0_cause_);
      }
      trace_last_pc = current_pc_;
      trace_last_instr = instruction;
    }
  }

  pc_ = next_pc_;
  next_pc_ += 4;

  // Decode and execute
  execute(instruction);

  // Commit previous cycle's delayed load after one full instruction delay.
  apply_pending_load();

  // Only arm the next delayed load when the instruction completed normally.
  if (!exception_raised_) {
    load_ = next_load_;
  } else {
    load_ = {0, 0};
  }

  const u32 consumed_cycles = instruction_cycles(instruction);
  cycles_ += consumed_cycles;
  prev_pc_for_diag = current_pc_;
  return consumed_cycles;
}

// ── Instruction Dispatch ───────────────────────────────────────────

void Cpu::execute(u32 i) {
  switch (op(i)) {
  case 0x00:
    op_special(i);
    break;
  case 0x01:
    op_bcondz(i);
    break;
  case 0x02:
    op_j(i);
    break;
  case 0x03:
    op_jal(i);
    break;
  case 0x04:
    op_beq(i);
    break;
  case 0x05:
    op_bne(i);
    break;
  case 0x06:
    op_blez(i);
    break;
  case 0x07:
    op_bgtz(i);
    break;
  case 0x14:
    op_beql(i);
    break;
  case 0x15:
    op_bnel(i);
    break;
  case 0x16:
    op_blezl(i);
    break;
  case 0x17:
    op_bgtzl(i);
    break;
  case 0x08:
    op_addi(i);
    break;
  case 0x09:
    op_addiu(i);
    break;
  case 0x0A:
    op_slti(i);
    break;
  case 0x0B:
    op_sltiu(i);
    break;
  case 0x0C:
    op_andi(i);
    break;
  case 0x0D:
    op_ori(i);
    break;
  case 0x0E:
    op_xori(i);
    break;
  case 0x0F:
    op_lui(i);
    break;
  case 0x10:
    op_cop0(i);
    break;
  case 0x11:
    op_cop1(i);
    break;
  case 0x12:
    op_cop2(i);
    break;
  case 0x13:
    op_cop3(i);
    break;
  case 0x20:
    op_lb(i);
    break;
  case 0x21:
    op_lh(i);
    break;
  case 0x22:
    op_lwl(i);
    break;
  case 0x23:
    op_lw(i);
    break;
  case 0x24:
    op_lbu(i);
    break;
  case 0x25:
    op_lhu(i);
    break;
  case 0x26:
    op_lwr(i);
    break;
  case 0x28:
    op_sb(i);
    break;
  case 0x29:
    op_sh(i);
    break;
  case 0x2A:
    op_swl(i);
    break;
  case 0x2B:
    op_sw(i);
    break;
  case 0x2E:
    op_swr(i);
    break;
  case 0x30:
    op_lwc0(i);
    break;
  case 0x31:
    op_lwc1(i);
    break;
  case 0x32:
    op_lwc2(i);
    break;
  case 0x33:
    op_lwc3(i);
    break;
  case 0x38:
    op_swc0(i);
    break;
  case 0x39:
    op_swc1(i);
    break;
  case 0x3A:
    op_swc2(i);
    break;
  case 0x3B:
    op_swc3(i);
    break;
  default:
    LOG_WARN("CPU: Unhandled opcode 0x%02X instr=0x%08X at PC=0x%08X", op(i),
             i, current_pc_);
    log_dma_context(sys_, current_pc_, i, gpr_);
    exception(Exception::ReservedInst);
    break;
  }
}

// ── SPECIAL opcodes ────────────────────────────────────────────────

void Cpu::op_special(u32 i) {
  switch (funct(i)) {
  case 0x00:
    op_sll(i);
    break;
  case 0x02:
    op_srl(i);
    break;
  case 0x03:
    op_sra(i);
    break;
  case 0x04:
    op_sllv(i);
    break;
  case 0x06:
    op_srlv(i);
    break;
  case 0x07:
    op_srav(i);
    break;
  case 0x08:
    op_jr(i);
    break;
  case 0x09:
    op_jalr(i);
    break;
  case 0x0A:
    op_movz(i);
    break;
  case 0x0B:
    op_movn(i);
    break;
  case 0x0C:
    op_syscall(i);
    break;
  case 0x0D:
    op_break(i);
    break;
  case 0x0F:
    op_sync(i);
    break;
  case 0x10:
    op_mfhi(i);
    break;
  case 0x11:
    op_mthi(i);
    break;
  case 0x12:
    op_mflo(i);
    break;
  case 0x13:
    op_mtlo(i);
    break;
  case 0x14:
    // Seen in some title startup loops; treat as benign no-op for compatibility.
    break;
  case 0x18:
    op_mult(i);
    break;
  case 0x19:
    op_multu(i);
    break;
  case 0x1A:
    op_div(i);
    break;
  case 0x1B:
    op_divu(i);
    break;
  case 0x20:
    op_add(i);
    break;
  case 0x21:
    op_addu(i);
    break;
  case 0x22:
    op_sub(i);
    break;
  case 0x23:
    op_subu(i);
    break;
  case 0x24:
    op_and(i);
    break;
  case 0x25:
    op_or(i);
    break;
  case 0x26:
    op_xor(i);
    break;
  case 0x27:
    op_nor(i);
    break;
  case 0x2A:
    op_slt(i);
    break;
  case 0x2B:
    op_sltu(i);
    break;
  default:
    if (g_experimental_unhandled_special_returns_zero) {
      // Experimental behavior: treat unknown SPECIAL as writing 0 to rd.
      set_reg(rd(i), 0);
      LOG_WARN("CPU: Experimental fallback for SPECIAL funct 0x%02X at PC=0x%08X (rd <- 0)",
               funct(i), current_pc_);
      log_dma_context(sys_, current_pc_, i, gpr_);
      break;
    }
    LOG_WARN("CPU: Unhandled SPECIAL funct 0x%02X instr=0x%08X at PC=0x%08X",
             funct(i), i, current_pc_);
    log_dma_context(sys_, current_pc_, i, gpr_);
    exception(Exception::ReservedInst);
    break;
  }
}

// ── Shift Instructions ─────────────────────────────────────────────

void Cpu::op_sll(u32 i) { set_reg(rd(i), gpr_[rt(i)] << shamt(i)); }
void Cpu::op_srl(u32 i) { set_reg(rd(i), gpr_[rt(i)] >> shamt(i)); }
void Cpu::op_sra(u32 i) {
  set_reg(rd(i), static_cast<u32>(static_cast<s32>(gpr_[rt(i)]) >> shamt(i)));
}
void Cpu::op_sllv(u32 i) {
  set_reg(rd(i), gpr_[rt(i)] << (gpr_[rs(i)] & 0x1F));
}
void Cpu::op_srlv(u32 i) {
  set_reg(rd(i), gpr_[rt(i)] >> (gpr_[rs(i)] & 0x1F));
}
void Cpu::op_srav(u32 i) {
  set_reg(rd(i), static_cast<u32>(static_cast<s32>(gpr_[rt(i)]) >>
                                  (gpr_[rs(i)] & 0x1F)));
}

// ── Jump / Branch Instructions ─────────────────────────────────────

void Cpu::op_jr(u32 i) {
  const u32 target = gpr_[rs(i)];
  if (cpu_diag_enabled()) {
    if (target == 0u) {
      log_zero_target_jump(sys_, current_pc_, gpr_, rs(i));
    }
    if (!is_plausible_exec_addr(target)) {
      log_suspicious_jump(sys_, current_pc_, gpr_, target, rs(i));
    }
    if (is_low_helper_addr(target)) {
      log_low_helper_call(sys_, current_pc_, next_pc_, gpr_, target, "jr");
    }
  }
  begin_branch(true, target);
}

void Cpu::op_jalr(u32 i) {
  const u32 target = gpr_[rs(i)];
  if (cpu_diag_enabled()) {
    if (target == 0u) {
      log_zero_target_jump(sys_, current_pc_, gpr_, rs(i));
    }
    if (!is_plausible_exec_addr(target)) {
      log_suspicious_jump(sys_, current_pc_, gpr_, target, rs(i));
    }
    if (is_low_helper_addr(target)) {
      log_low_helper_call(sys_, current_pc_, next_pc_, gpr_, target, "jalr");
    }
  }
  set_reg(rd(i), next_pc_); // Save return address
  begin_branch(true, target);
}

void Cpu::op_movz(u32 i) {
  if (gpr_[rt(i)] == 0) {
    set_reg(rd(i), gpr_[rs(i)]);
  }
}

u32 Cpu::instruction_cycles(u32 instruction) const {
  if (exception_raised_) {
    return 2;
  }

  switch (op(instruction)) {
  case 0x00:
    switch (funct(instruction)) {
    case 0x08: // JR
    case 0x09: // JALR
      return 2;
    case 0x18: // MULT
    case 0x19: // MULTU
      return 8;
    case 0x1A: // DIV
    case 0x1B: // DIVU
      return 35;
    default:
      return 1;
    }
  case 0x01: // BcondZ
  case 0x02: // J
  case 0x03: // JAL
  case 0x04: // BEQ
  case 0x05: // BNE
  case 0x06: // BLEZ
  case 0x07: // BGTZ
    return pending_branch_taken_ ? 2 : 1;
  case 0x10: // COP0
  case 0x11: // COP1
  case 0x12: // COP2 / GTE
  case 0x13: // COP3
    return 2;
  case 0x20: // LB
  case 0x21: // LH
  case 0x22: // LWL
  case 0x23: // LW
  case 0x24: // LBU
  case 0x25: // LHU
  case 0x26: // LWR
  case 0x28: // SB
  case 0x29: // SH
  case 0x2A: // SWL
  case 0x2B: // SW
  case 0x2E: // SWR
  case 0x30: // LWC0
  case 0x31: // LWC1
  case 0x32: // LWC2
  case 0x33: // LWC3
  case 0x38: // SWC0
  case 0x39: // SWC1
  case 0x3A: // SWC2
  case 0x3B: // SWC3
    return 2;
  default:
    return 1;
  }
}

void Cpu::op_movn(u32 i) {
  if (gpr_[rt(i)] != 0) {
    set_reg(rd(i), gpr_[rs(i)]);
  }
}

void Cpu::op_sync(u32 /*i*/) {
  // In-order single-core emulation: nothing to serialize.
}

void Cpu::op_j(u32 i) {
  const u32 target = (pc_ & 0xF0000000) | (imm26(i) << 2);
  if (cpu_diag_enabled() && is_low_helper_addr(target)) {
    log_low_helper_call(sys_, current_pc_, next_pc_, gpr_, target, "j");
  }
  begin_branch(true, target);
}

void Cpu::op_jal(u32 i) {
  const u32 target = (pc_ & 0xF0000000) | (imm26(i) << 2);
  if (cpu_diag_enabled() && is_low_helper_addr(target)) {
    log_low_helper_call(sys_, current_pc_, next_pc_, gpr_, target, "jal");
  }
  set_reg(31, next_pc_); // $ra
  begin_branch(true, target);
}

void Cpu::op_beq(u32 i) {
  const bool taken = gpr_[rs(i)] == gpr_[rt(i)];
  begin_branch(taken, pc_ + (simm(i) << 2));
}

void Cpu::op_bne(u32 i) {
  const bool taken = gpr_[rs(i)] != gpr_[rt(i)];
  begin_branch(taken, pc_ + (simm(i) << 2));
}

void Cpu::op_blez(u32 i) {
  const bool taken = static_cast<s32>(gpr_[rs(i)]) <= 0;
  begin_branch(taken, pc_ + (simm(i) << 2));
}

void Cpu::op_bgtz(u32 i) {
  const bool taken = static_cast<s32>(gpr_[rs(i)]) > 0;
  begin_branch(taken, pc_ + (simm(i) << 2));
}

void Cpu::op_beql(u32 i) {
  const bool taken = gpr_[rs(i)] == gpr_[rt(i)];
  if (taken) {
    begin_branch(true, pc_ + (simm(i) << 2));
  } else {
    // Branch-likely: not taken annuls the delay slot.
    pc_ = next_pc_;
    next_pc_ += 4;
  }
}

void Cpu::op_bnel(u32 i) {
  const bool taken = gpr_[rs(i)] != gpr_[rt(i)];
  if (taken) {
    begin_branch(true, pc_ + (simm(i) << 2));
  } else {
    pc_ = next_pc_;
    next_pc_ += 4;
  }
}

void Cpu::op_blezl(u32 i) {
  const bool taken = static_cast<s32>(gpr_[rs(i)]) <= 0;
  if (taken) {
    begin_branch(true, pc_ + (simm(i) << 2));
  } else {
    pc_ = next_pc_;
    next_pc_ += 4;
  }
}

void Cpu::op_bgtzl(u32 i) {
  const bool taken = static_cast<s32>(gpr_[rs(i)]) > 0;
  if (taken) {
    begin_branch(true, pc_ + (simm(i) << 2));
  } else {
    pc_ = next_pc_;
    next_pc_ += 4;
  }
}

void Cpu::op_bcondz(u32 i) {
  const u32 rt_field = rt(i);

  const bool bgez = (rt_field & 0x01u) != 0u;
  const bool likely = (rt_field & 0x02u) != 0u;
  const bool link = (rt_field & 0x10u) != 0u;

  const s32 val = static_cast<s32>(gpr_[rs(i)]);
  const bool taken = bgez ? (val >= 0) : (val < 0);

  if (likely && !taken) {
    // Branch-likely: not taken annuls the delay slot.
    pc_ = next_pc_;
    next_pc_ += 4;
    return;
  }

  // Keep broad REGIMM compatibility: legacy code may probe multiple rt forms.
  // Match existing emulator behavior by always writing RA for link variants.
  if (link) {
    set_reg(31, next_pc_);
  }
  begin_branch(taken, pc_ + (simm(i) << 2));
}

// ── Arithmetic Instructions ────────────────────────────────────────

void Cpu::op_addi(u32 i) {
  s32 s = static_cast<s32>(gpr_[rs(i)]);
  s32 imm = simm(i);
  s32 result = s + imm;
  // Check overflow
  if ((s > 0 && imm > 0 && result < 0) || (s < 0 && imm < 0 && result > 0)) {
    exception(Exception::Overflow);
    return;
  }
  set_reg(rt(i), static_cast<u32>(result));
}

void Cpu::op_addiu(u32 i) {
  set_reg(rt(i), gpr_[rs(i)] + static_cast<u32>(simm(i)));
}
void Cpu::op_slti(u32 i) {
  set_reg(rt(i), static_cast<s32>(gpr_[rs(i)]) < simm(i) ? 1 : 0);
}
void Cpu::op_sltiu(u32 i) {
  set_reg(rt(i), gpr_[rs(i)] < static_cast<u32>(simm(i)) ? 1 : 0);
}
void Cpu::op_andi(u32 i) { set_reg(rt(i), gpr_[rs(i)] & imm16(i)); }
void Cpu::op_ori(u32 i) { set_reg(rt(i), gpr_[rs(i)] | imm16(i)); }
void Cpu::op_xori(u32 i) { set_reg(rt(i), gpr_[rs(i)] ^ imm16(i)); }
void Cpu::op_lui(u32 i) { set_reg(rt(i), static_cast<u32>(imm16(i)) << 16); }

void Cpu::op_add(u32 i) {
  s32 a = static_cast<s32>(gpr_[rs(i)]);
  s32 b = static_cast<s32>(gpr_[rt(i)]);
  s32 result = a + b;
  if ((a > 0 && b > 0 && result < 0) || (a < 0 && b < 0 && result > 0)) {
    exception(Exception::Overflow);
    return;
  }
  set_reg(rd(i), static_cast<u32>(result));
}

void Cpu::op_addu(u32 i) { set_reg(rd(i), gpr_[rs(i)] + gpr_[rt(i)]); }

void Cpu::op_sub(u32 i) {
  s32 a = static_cast<s32>(gpr_[rs(i)]);
  s32 b = static_cast<s32>(gpr_[rt(i)]);
  s32 result = a - b;
  if ((a > 0 && b < 0 && result < 0) || (a < 0 && b > 0 && result > 0)) {
    exception(Exception::Overflow);
    return;
  }
  set_reg(rd(i), static_cast<u32>(result));
}

void Cpu::op_subu(u32 i) { set_reg(rd(i), gpr_[rs(i)] - gpr_[rt(i)]); }

// ── Logic Instructions ─────────────────────────────────────────────

void Cpu::op_and(u32 i) { set_reg(rd(i), gpr_[rs(i)] & gpr_[rt(i)]); }
void Cpu::op_or(u32 i) { set_reg(rd(i), gpr_[rs(i)] | gpr_[rt(i)]); }
void Cpu::op_xor(u32 i) { set_reg(rd(i), gpr_[rs(i)] ^ gpr_[rt(i)]); }
void Cpu::op_nor(u32 i) { set_reg(rd(i), ~(gpr_[rs(i)] | gpr_[rt(i)])); }

// ── Comparison Instructions ────────────────────────────────────────

void Cpu::op_slt(u32 i) {
  set_reg(rd(i), static_cast<s32>(gpr_[rs(i)]) < static_cast<s32>(gpr_[rt(i)])
                     ? 1
                     : 0);
}
void Cpu::op_sltu(u32 i) { set_reg(rd(i), gpr_[rs(i)] < gpr_[rt(i)] ? 1 : 0); }

// ── Multiply / Divide ──────────────────────────────────────────────

void Cpu::op_mult(u32 i) {
  s64 result = static_cast<s64>(static_cast<s32>(gpr_[rs(i)])) *
               static_cast<s64>(static_cast<s32>(gpr_[rt(i)]));
  lo_ = static_cast<u32>(result);
  hi_ = static_cast<u32>(result >> 32);
}

void Cpu::op_multu(u32 i) {
  u64 result = static_cast<u64>(gpr_[rs(i)]) * static_cast<u64>(gpr_[rt(i)]);
  lo_ = static_cast<u32>(result);
  hi_ = static_cast<u32>(result >> 32);
}

void Cpu::op_div(u32 i) {
  s32 n = static_cast<s32>(gpr_[rs(i)]);
  s32 d = static_cast<s32>(gpr_[rt(i)]);
  if (d == 0) {
    // Division by zero
    lo_ = (n >= 0) ? 0xFFFFFFFFu : 1u;
    hi_ = static_cast<u32>(n);
  } else if (n == static_cast<s32>(0x80000000) && d == -1) {
    // Overflow
    lo_ = 0x80000000u;
    hi_ = 0;
  } else {
    lo_ = static_cast<u32>(n / d);
    hi_ = static_cast<u32>(n % d);
  }
}

void Cpu::op_divu(u32 i) {
  u32 n = gpr_[rs(i)];
  u32 d = gpr_[rt(i)];
  if (d == 0) {
    lo_ = 0xFFFFFFFFu;
    hi_ = n;
  } else {
    lo_ = n / d;
    hi_ = n % d;
  }
}

void Cpu::op_mfhi(u32 i) { set_reg(rd(i), hi_); }
void Cpu::op_mthi(u32 i) { hi_ = gpr_[rs(i)]; }
void Cpu::op_mflo(u32 i) { set_reg(rd(i), lo_); }
void Cpu::op_mtlo(u32 i) { lo_ = gpr_[rs(i)]; }

// ── Load Instructions ──────────────────────────────────────────────

void Cpu::op_lb(u32 i) {
  u32 addr = gpr_[rs(i)] + static_cast<u32>(simm(i));
  u8 val = load8(addr);
  // Load delay: value takes effect after the next instruction
  schedule_load(rt(i), static_cast<u32>(sign_extend_8(val)));
}

void Cpu::op_lbu(u32 i) {
  u32 addr = gpr_[rs(i)] + static_cast<u32>(simm(i));
  schedule_load(rt(i), load8(addr));
}

void Cpu::op_lh(u32 i) {
  u32 addr = gpr_[rs(i)] + static_cast<u32>(simm(i));
  u16 val = load16(addr);
  if (exception_raised_)
    return;
  schedule_load(rt(i), static_cast<u32>(sign_extend_16(val)));
}

void Cpu::op_lhu(u32 i) {
  u32 addr = gpr_[rs(i)] + static_cast<u32>(simm(i));
  u16 val = load16(addr);
  if (exception_raised_)
    return;
  schedule_load(rt(i), val);
}

void Cpu::op_lw(u32 i) {
  u32 addr = gpr_[rs(i)] + static_cast<u32>(simm(i));
  u32 val = load32(addr);
  if (exception_raised_)
    return;
  if (cpu_diag_enabled() && rt(i) == 31 && !is_plausible_exec_addr(val)) {
    log_suspicious_ra_load(sys_, current_pc_, gpr_, addr, val);
  }
  schedule_load(rt(i), val);
}

void Cpu::op_lwl(u32 i) {
  u32 addr = gpr_[rs(i)] + static_cast<u32>(simm(i));
  u32 aligned = addr & ~3u;
  u32 val = load32(aligned);
  u32 cur = gpr_[rt(i)];
  // If there's a pending load for this register, use that value
  if (load_.reg == rt(i))
    cur = load_.value;

  u32 result;
  switch (addr & 3) {
  case 0:
    result = (cur & 0x00FFFFFF) | (val << 24);
    break;
  case 1:
    result = (cur & 0x0000FFFF) | (val << 16);
    break;
  case 2:
    result = (cur & 0x000000FF) | (val << 8);
    break;
  case 3:
    result = val;
    break;
  default:
    result = 0;
    break;
  }
  schedule_load(rt(i), result);
}

void Cpu::op_lwr(u32 i) {
  u32 addr = gpr_[rs(i)] + static_cast<u32>(simm(i));
  u32 aligned = addr & ~3u;
  u32 val = load32(aligned);
  u32 cur = gpr_[rt(i)];
  if (load_.reg == rt(i))
    cur = load_.value;

  u32 result;
  switch (addr & 3) {
  case 0:
    result = val;
    break;
  case 1:
    result = (cur & 0xFF000000) | (val >> 8);
    break;
  case 2:
    result = (cur & 0xFFFF0000) | (val >> 16);
    break;
  case 3:
    result = (cur & 0xFFFFFF00) | (val >> 24);
    break;
  default:
    result = 0;
    break;
  }
  schedule_load(rt(i), result);
}

// ── Store Instructions ─────────────────────────────────────────────

void Cpu::op_sb(u32 i) {
  u32 addr = gpr_[rs(i)] + static_cast<u32>(simm(i));
  const u32 phys = addr & 0x1FFFFFFFu;
  if (cpu_diag_enabled() && current_pc_ == 0xBFC02B7Cu &&
      phys >= 0x00047680u && phys < 0x000476C0u) {
    static u32 bios_low_stub_sb_log_count = 0;
    if (bios_low_stub_sb_log_count < 16u) {
      ++bios_low_stub_sb_log_count;
      LOG_WARN(
          "CPU: bios low-stub sb pc=0x%08X virt=0x%08X phys=0x%08X byte=0x%02X rs=r%u base=0x%08X rt=r%u raw=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X t0=0x%08X t1=0x%08X t2=0x%08X t3=0x%08X t4=0x%08X t5=0x%08X t6=0x%08X t7=0x%08X",
          current_pc_,
          addr, phys, static_cast<unsigned>(gpr_[rt(i)] & 0xFFu),
          static_cast<unsigned>(rs(i)), gpr_[rs(i)],
          static_cast<unsigned>(rt(i)), gpr_[rt(i)], gpr_[4], gpr_[5],
          gpr_[6], gpr_[7], gpr_[8], gpr_[9], gpr_[10], gpr_[11], gpr_[12],
          gpr_[13], gpr_[14], gpr_[15]);
    }
  }
  store8(addr, static_cast<u8>(gpr_[rt(i)]));
}

void Cpu::op_sh(u32 i) {
  u32 addr = gpr_[rs(i)] + static_cast<u32>(simm(i));
  store16(addr, static_cast<u16>(gpr_[rt(i)]));
}

void Cpu::op_sw(u32 i) {
  u32 addr = gpr_[rs(i)] + static_cast<u32>(simm(i));
  store32(addr, gpr_[rt(i)]);
}

void Cpu::op_swl(u32 i) {
  u32 addr = gpr_[rs(i)] + static_cast<u32>(simm(i));
  u32 aligned = addr & ~3u;
  u32 val = load32(aligned);
  u32 reg_val = gpr_[rt(i)];

  u32 result;
  switch (addr & 3) {
  case 0:
    result = (val & 0xFFFFFF00) | (reg_val >> 24);
    break;
  case 1:
    result = (val & 0xFFFF0000) | (reg_val >> 16);
    break;
  case 2:
    result = (val & 0xFF000000) | (reg_val >> 8);
    break;
  case 3:
    result = reg_val;
    break;
  default:
    result = 0;
    break;
  }
  store32(aligned, result);
}

void Cpu::op_swr(u32 i) {
  u32 addr = gpr_[rs(i)] + static_cast<u32>(simm(i));
  u32 aligned = addr & ~3u;
  u32 val = load32(aligned);
  u32 reg_val = gpr_[rt(i)];

  u32 result;
  switch (addr & 3) {
  case 0:
    result = reg_val;
    break;
  case 1:
    result = (val & 0x000000FF) | (reg_val << 8);
    break;
  case 2:
    result = (val & 0x0000FFFF) | (reg_val << 16);
    break;
  case 3:
    result = (val & 0x00FFFFFF) | (reg_val << 24);
    break;
  default:
    result = 0;
    break;
  }
  store32(aligned, result);
}

// ── System Instructions ────────────────────────────────────────────

void Cpu::op_syscall(u32 /*i*/) { exception(Exception::Syscall); }
void Cpu::op_break(u32 /*i*/) { exception(Exception::Break); }

// ── COP0 (System Control) ──────────────────────────────────────────

void Cpu::op_cop0(u32 i) {
  u32 sub = rs(i);
  switch (sub) {
  case 0x00: // MFC0: Move from COP0
    schedule_load(rt(i), read_cop0_reg(rd(i)));
    break;

  case 0x04: // MTC0: Move to COP0
    write_cop0_reg(rd(i), gpr_[rt(i)]);
    break;

  case 0x10: // RFE: Return from Exception
    if ((i & 0x3F) == 0x10) {
      // Pop the Interrupt Enable/Kernel-User mode stack
      u32 mode = cop0_sr_ & 0x3F;
      cop0_sr_ &= ~0x3Fu;
      cop0_sr_ |= (mode >> 2);
    }
    else {
        LOG_WARN("CPU: Unhandled COP0 CO funct 0x%02X at PC=0x%08X", i & 0x3F,
            current_pc_);
        exception(Exception::ReservedInst);
    }
    break;

  case 0x0A:
    // Compatibility nop: observed in Silent Hill startup polling loop.
    break;

  default:
    LOG_WARN("CPU: Unhandled COP0 sub-op 0x%02X at PC=0x%08X", sub,
             current_pc_);
    exception(Exception::ReservedInst);
    break;
  }
}

void Cpu::op_cop1(u32 /*i*/) { raise_cop_unusable(1); }

void Cpu::op_lwc0(u32 i) {
  const u32 addr = gpr_[rs(i)] + static_cast<u32>(simm(i));
  const u32 val = load32(addr);
  if (exception_raised_) {
    return;
  }
  write_cop0_reg(rt(i), val);
}

void Cpu::op_swc0(u32 i) {
  const u32 addr = gpr_[rs(i)] + static_cast<u32>(simm(i));
  const u32 val = read_cop0_reg(rt(i));
  store32(addr, val);
}

void Cpu::op_lwc1(u32 /*i*/) { raise_cop_unusable(1); }
void Cpu::op_swc1(u32 /*i*/) { raise_cop_unusable(1); }

// ── COP2 (GTE) ────────────────────────────────────────────────────

void Cpu::op_cop2(u32 i) {
  u32 sub = rs(i);
  switch (sub) {
  case 0x00: // MFC2: Move from COP2 data register
    schedule_load(rt(i), gte.read_data(rd(i)));
    break;
  case 0x02: // CFC2: Move from COP2 control register
    schedule_load(rt(i), gte.read_ctrl(rd(i)));
    break;
  case 0x04: // MTC2: Move to COP2 data register
    gte.write_data(rd(i), gpr_[rt(i)]);
    break;
  case 0x06: // CTC2: Move to COP2 control register
    gte.write_ctrl(rd(i), gpr_[rt(i)]);
    break;
  default:
    if (sub & 0x10) {
      // GTE command
      gte.execute(i);
    } else {
      LOG_WARN("CPU: Unhandled COP2 sub-op 0x%02X at PC=0x%08X", sub,
               current_pc_);
      exception(Exception::ReservedInst);
    }
    break;
  }
}

void Cpu::op_lwc2(u32 i) {
  u32 addr = gpr_[rs(i)] + static_cast<u32>(simm(i));
  u32 val = load32(addr);
  if (exception_raised_)
    return;
  gte.write_data(rt(i), val);
}

void Cpu::op_swc2(u32 i) {
  u32 addr = gpr_[rs(i)] + static_cast<u32>(simm(i));
  u32 val = gte.read_data(rt(i));
  store32(addr, val);
}

void Cpu::op_cop3(u32 /*i*/) { raise_cop_unusable(3); }
void Cpu::op_lwc3(u32 /*i*/) { raise_cop_unusable(3); }
void Cpu::op_swc3(u32 /*i*/) { raise_cop_unusable(3); }

