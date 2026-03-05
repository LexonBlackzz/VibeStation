#include "cpu.h"
#include "system.h"

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

  // Cancel any pending load to the same register
  if (load_.reg == index) {
    load_.reg = 0;
  }
  if (next_load_.reg == index) {
    next_load_.reg = 0;
  }
  gpr_[index] = value;
  gpr_[0] = 0; // $zero is hardwired to 0
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

void Cpu::apply_pending_load() {
  if (load_.reg != 0) {
    gpr_[load_.reg] = load_.value;
  }
  gpr_[0] = 0;
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

u32 Cpu::load32(u32 addr) {
  if (addr & 3) {
    cop0_badvaddr_ = addr;
    exception(Exception::AddrLoadErr);
    return 0;
  }
  return sys_->read32(addr);
}

u16 Cpu::load16(u32 addr) {
  if (addr & 1) {
    cop0_badvaddr_ = addr;
    exception(Exception::AddrLoadErr);
    return 0;
  }
  return sys_->read16(addr);
}

u8 Cpu::load8(u32 addr) { return sys_->read8(addr); }

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
  sys_->write32(addr, value);
}

void Cpu::store16(u32 addr, u16 value) {
  if (addr & 1) {
    cop0_badvaddr_ = addr;
    exception(Exception::AddrStoreErr);
    return;
  }
  if (cop0_sr_ & (1u << 16))
    return;
  sys_->write16(addr, value);
}

void Cpu::store8(u32 addr, u8 value) {
  if (cop0_sr_ & (1u << 16))
    return;
  sys_->write8(addr, value);
}

// ── Exception Handling ─────────────────────────────────────────────

void Cpu::exception(Exception cause) {
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

void Cpu::step() {
  static u64 trace_step_counter = 0;
  static u32 trace_last_pc = 0;
  static u32 trace_last_instr = 0;
  static u64 trace_repeat = 0;
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
    gpr_[0] = 0;
    cycles_++;
    return;
  }

  // Fetch instruction
  u32 instruction = load32(pc_);
  if (exception_raised_) {
    apply_pending_load();
    gpr_[0] = 0;
    cycles_++;
    return;
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

  // Ensure $zero stays 0
  gpr_[0] = 0;

  cycles_++;
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
  case 0x12:
    op_cop2(i);
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
  case 0x32:
    op_lwc2(i);
    break;
  case 0x38:
    op_swc0(i);
    break;
  case 0x3A:
    op_swc2(i);
    break;
  default:
    LOG_WARN("CPU: Unhandled opcode 0x%02X at PC=0x%08X", op(i), current_pc_);
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
      break;
    }
    LOG_WARN("CPU: Unhandled SPECIAL funct 0x%02X at PC=0x%08X", funct(i),
             current_pc_);
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
  begin_branch(true, gpr_[rs(i)]);
}

void Cpu::op_jalr(u32 i) {
  set_reg(rd(i), next_pc_); // Save return address
  begin_branch(true, gpr_[rs(i)]);
}

void Cpu::op_movz(u32 i) {
  if (gpr_[rt(i)] == 0) {
    set_reg(rd(i), gpr_[rs(i)]);
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
  begin_branch(true, (pc_ & 0xF0000000) | (imm26(i) << 2));
}

void Cpu::op_jal(u32 i) {
  set_reg(31, next_pc_); // $ra
  begin_branch(true, (pc_ & 0xF0000000) | (imm26(i) << 2));
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
      LOG_WARN("CPU: Unhandled COP2 sub-op 0x%02X", sub);
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

