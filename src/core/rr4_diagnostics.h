#pragma once
// Ridge Racer 4 specific diagnostic logging.
// Extracted from cpu.cpp to keep game-specific debug code out of the core CPU path.

#include "types.h"

class System;

struct Rr4DiagState {
  // load32 sentinel zones
  u32 slot18_sentinel = 0;
  u32 next_sentinel = 0;
  u32 head_sentinel = 0;

  // exception zone
  u32 syscall_logs = 0;

  // step() zones
  u32 dma_setup = 0;
  u32 sector_parse = 0;
  u32 payload_decision = 0;
  u32 post_parse = 0;
  u32 late_low_loop = 0;
  u32 callback2_exec = 0;
  u32 late_low_dispatcher = 0;
  u32 late_low_callsite = 0;
  u32 low_k0_chain = 0;
  u32 low_dispatch = 0;
  u32 node_window = 0;
  u32 late_low_path = 0;
  bool late_code_dumped = false;
  u32 decomp_step = 0;
  bool lowclr_entry_logged = false;
  bool lowclr_lowdest_logged = false;
  u32 wait_log = 0;
  u32 dma_callback = 0;
  u32 bios_trampoline = 0;
  u32 callback_code = 0;
  bool callback_code_dumped = false;
  u32 callback_log = 0;
  u32 fault_window = 0;
  u32 lowclr_caller = 0;

  // step() low-runtime zone
  u32 low_runtime_log_count = 0;
  bool low_runtime_active = false;
  bool low_runtime_full_dumped = false;

  // op_lw zone
  u32 node_lw_fault = 0;

  // op_sb zones
  u32 node_sb = 0;
  u32 bridge_sb = 0;

  // op_sh zones
  u32 node_sh = 0;
  u32 bridge_sh = 0;

  // op_sw zones
  u32 low_slot_sw = 0;
  u32 low_slot_sw_late = 0;
  u32 low_vec_sw = 0;
  u32 low_slot_sw_focus = 0;
  u32 node_sw = 0;
  u32 node_sw_late = 0;
  u32 node_sw_fault = 0;
  u32 bridge_sw = 0;

  // store16 zones
  u32 lowclr_wrap = 0;
  u32 lowclr_store16 = 0;

  // prev PC tracking (updated by caller after on_step returns)
  u32 prev_pc_for_diag = 0;
};

namespace rr4_diag {
void on_load32(Rr4DiagState& st, u32 pc, u32 addr, u32 value,
               const u32* regs, u64 cycles, System& sys);
void on_exception(Rr4DiagState& st, u32 pc, u64 cycles,
                  u32 cop0_epc, const u32* regs, System& sys);
void on_step(Rr4DiagState& st, u32 pc, u32 instruction, const u32* regs,
             u64 cycles, bool cpu_diag, u32 cop0_sr, u32 cop0_cause,
             bool irq_pending, System& sys);
void on_op_lw(Rr4DiagState& st, u32 pc, u32 instruction, u32 addr,
              const u32* regs, u64 cycles, u32 cop0_sr, u32 cop0_cause,
              bool irq_pending);
void on_op_sb(Rr4DiagState& st, u32 pc, u32 instruction, u32 addr,
              const u32* regs, u32 cop0_sr, u32 cop0_cause, bool irq_pending);
void on_op_sh(Rr4DiagState& st, u32 pc, u32 instruction, u32 addr,
              const u32* regs);
void on_op_sw(Rr4DiagState& st, u32 pc, u32 instruction, u32 addr,
              u32 store_val, const u32* regs, u64 cycles, u32 cop0_sr,
              u32 cop0_cause, bool irq_pending);
void on_store16(Rr4DiagState& st, u32 pc, u32 addr, u16 value,
                const u32* regs, u64 cycles, System& sys);
} // namespace rr4_diag
