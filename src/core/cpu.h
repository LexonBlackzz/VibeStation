#pragma once
#include "gte.h"
#include "types.h"


// ── MIPS R3000A CPU ────────────────────────────────────────────────
// 32-bit RISC processor with COP0 (System Control) and COP2 (GTE).
// Features: 32 GPRs, HI/LO for multiply/divide, load and branch delay slots.

class System;

// COP0 exception causes
enum class Exception : u32 {
  Interrupt = 0x0,
  AddrLoadErr = 0x4,
  AddrStoreErr = 0x5,
  BusErrInstr = 0x6,
  BusErrData = 0x7,
  Syscall = 0x8,
  Break = 0x9,
  ReservedInst = 0xA,
  CopUnusable = 0xB,
  Overflow = 0xC,
};

class Cpu {
public:
  void init(System *sys);
  void reset();

  // Execute one instruction and return the number of CPU cycles it consumed.
  u32 step();

  // COP2 (GTE) — publicly accessible for DMA
  Gte gte;

  // Debug access
  u32 pc() const { return pc_; }
  u32 reg(int i) const { return gpr_[i]; }
  u64 cycle_count() const { return cycles_; }

private:
  System *sys_ = nullptr;

  // ── Registers ──────────────────────────────────────────────────
  u32 gpr_[32] = {};    // General purpose registers (r0 ≡ 0)
  u32 pc_ = 0;          // Program counter
  u32 next_pc_ = 0;     // Next PC (branch delay slot)
  u32 current_pc_ = 0;  // PC of currently executing instruction
  u32 hi_ = 0, lo_ = 0; // Multiply/divide outputs

  // Load delay slot
  struct PendingLoad {
    u32 reg = 0;
    u32 value = 0;
  };
  PendingLoad load_ = {};
  PendingLoad next_load_ = {};
  bool in_delay_slot_ = false;
  bool pending_delay_slot_ = false;
  bool pending_branch_taken_ = false;
  u32 pending_branch_pc_ = 0;
  u32 active_branch_pc_ = 0;
  bool exception_raised_ = false;

  // ── COP0 Registers ────────────────────────────────────────────
  u32 cop0_sr_ = 0;        // Status Register (R12)
  u32 cop0_cause_ = 0;     // Cause Register (R13)
  u32 cop0_epc_ = 0;       // Exception PC (R14)
  u32 cop0_badvaddr_ = 0;  // Bad Virtual Address (R8)
  u32 cop0_jumpdest_ = 0;  // Jump destination for debug (R6)
  u32 cop0_regs_[32] = {}; // All 32 COP0 regs (some unused)

  u64 cycles_ = 0;

  // ── Helpers ────────────────────────────────────────────────────
  void set_reg(u32 index, u32 value);
  void apply_pending_load();
  void schedule_load(u32 index, u32 value);
  void begin_branch(bool taken, u32 target);
  u32 read_cop0_reg(u32 index) const;
  void write_cop0_reg(u32 index, u32 value);
  void raise_cop_unusable(u32 cop_index);
  u32 instruction_cycles(u32 instruction) const;

  // Memory access (through system bus)
  u32 fetch32(u32 addr);
  u32 load32(u32 addr);
  u16 load16(u32 addr);
  u8 load8(u32 addr);
  void store32(u32 addr, u32 value);
  void store16(u32 addr, u16 value);
  void store8(u32 addr, u8 value);

  // ── Instruction Decode ─────────────────────────────────────────
  void execute(u32 instruction);

  // Decode helpers (extract fields from instruction)
  static u32 op(u32 i) { return (i >> 26) & 0x3F; }
  static u32 rs(u32 i) { return (i >> 21) & 0x1F; }
  static u32 rt(u32 i) { return (i >> 16) & 0x1F; }
  static u32 rd(u32 i) { return (i >> 11) & 0x1F; }
  static u32 shamt(u32 i) { return (i >> 6) & 0x1F; }
  static u32 funct(u32 i) { return i & 0x3F; }
  static u16 imm16(u32 i) { return static_cast<u16>(i & 0xFFFF); }
  static u32 imm26(u32 i) { return i & 0x03FFFFFF; }
  static s32 simm(u32 i) { return sign_extend_16(static_cast<u16>(i)); }

  // ── Exception Handling ─────────────────────────────────────────
  void exception(Exception cause);
  bool check_irq();

  // ── Opcode Handlers — Primary ──────────────────────────────────
  void op_special(u32 i);
  void op_bcondz(u32 i);
  void op_j(u32 i);
  void op_jal(u32 i);
  void op_beq(u32 i);
  void op_bne(u32 i);
  void op_blez(u32 i);
  void op_bgtz(u32 i);
  void op_beql(u32 i);
  void op_bnel(u32 i);
  void op_blezl(u32 i);
  void op_bgtzl(u32 i);
  void op_addi(u32 i);
  void op_addiu(u32 i);
  void op_slti(u32 i);
  void op_sltiu(u32 i);
  void op_andi(u32 i);
  void op_ori(u32 i);
  void op_xori(u32 i);
  void op_lui(u32 i);
  void op_cop0(u32 i);
  void op_cop1(u32 i);
  void op_cop2(u32 i);
  void op_cop3(u32 i);
  void op_lb(u32 i);
  void op_lh(u32 i);
  void op_lwl(u32 i);
  void op_lw(u32 i);
  void op_lbu(u32 i);
  void op_lhu(u32 i);
  void op_lwr(u32 i);
  void op_sb(u32 i);
  void op_sh(u32 i);
  void op_swl(u32 i);
  void op_sw(u32 i);
  void op_swr(u32 i);
  void op_lwc0(u32 i);
  void op_lwc1(u32 i);
  void op_swc0(u32 i);
  void op_swc1(u32 i);
  void op_lwc2(u32 i);
  void op_lwc3(u32 i);
  void op_swc2(u32 i);
  void op_swc3(u32 i);

  // ── Opcode Handlers — SPECIAL (funct) ──────────────────────────
  void op_sll(u32 i);
  void op_srl(u32 i);
  void op_sra(u32 i);
  void op_sllv(u32 i);
  void op_srlv(u32 i);
  void op_srav(u32 i);
  void op_jr(u32 i);
  void op_jalr(u32 i);
  void op_movz(u32 i);
  void op_movn(u32 i);
  void op_sync(u32 i);
  void op_syscall(u32 i);
  void op_break(u32 i);
  void op_mfhi(u32 i);
  void op_mthi(u32 i);
  void op_mflo(u32 i);
  void op_mtlo(u32 i);
  void op_mult(u32 i);
  void op_multu(u32 i);
  void op_div(u32 i);
  void op_divu(u32 i);
  void op_add(u32 i);
  void op_addu(u32 i);
  void op_sub(u32 i);
  void op_subu(u32 i);
  void op_and(u32 i);
  void op_or(u32 i);
  void op_xor(u32 i);
  void op_nor(u32 i);
  void op_slt(u32 i);
  void op_sltu(u32 i);
};
