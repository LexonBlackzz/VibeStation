#pragma once
#include "gte.h"
#include "types.h"
#include <array>
#include <cstddef>


// ── MIPS R3000A CPU ────────────────────────────────────────────────
// 32-bit RISC processor with COP0 (System Control) and COP2 (GTE).
// Features: 32 GPRs, HI/LO for multiply/divide, load and branch delay slots.

class System;
class CpuOptimizedBackend;

struct CpuRunSliceResult {
  u32 cycles = 0;
  u32 instructions = 0;
};

struct CpuBackendStats {
  static constexpr size_t kDecodedOpStatsCount = 64;

  bool available = false;
  bool active = false;
  bool native_available = false;
  u32 block_count = 0;
  u32 interpreter_only_blocks = 0;
  u64 decoded_blocks = 0;
  u64 native_blocks = 0;
  u64 native_compile_attempts = 0;
  u64 native_compile_successes = 0;
  u64 native_blocks_compiled = 0;
  u64 native_branch_tail_blocks_compiled = 0;
  u64 native_memory_blocks_compiled = 0;
  u64 native_alu_blocks_compiled = 0;
  u64 native_reduced_helper_compile_attempts = 0;
  u64 native_reduced_helper_compile_successes = 0;
  u64 native_reduced_helper_blocks_compiled = 0;
  u64 native_reduced_helper_ram_load_blocks_compiled = 0;
  u64 native_reduced_helper_rejected_blocks = 0;
  u64 native_reduced_helper_rejected_instructions = 0;
  u64 native_reduced_helper_reject_stores = 0;
  u64 native_reduced_helper_reject_load_base_written = 0;
  u64 native_reduced_helper_reject_unsupported_memory = 0;
  u64 native_branch_tail_reduced_helper_candidate_blocks = 0;
  u64 native_branch_tail_reduced_helper_rejected_blocks = 0;
  u64 native_branch_tail_reduced_helper_reject_disabled = 0;
  u64 native_branch_tail_reduced_helper_reject_memory = 0;
  u64 native_branch_tail_reduced_helper_load_candidate_blocks = 0;
  u64 native_branch_tail_reduced_helper_store_candidate_blocks = 0;
  u64 native_branch_tail_reduced_helper_mixed_load_store_candidate_blocks = 0;
  u64 native_branch_tail_reduced_helper_reject_load = 0;
  u64 native_branch_tail_reduced_helper_reject_store = 0;
  u64 native_branch_tail_reduced_helper_reject_mixed_load_store = 0;
  u64 native_branch_tail_reduced_helper_reject_load_base_written = 0;
  u64 native_branch_tail_reduced_helper_reject_unsupported_load = 0;
  u64 native_branch_tail_reduced_helper_reject_body = 0;
  u64 native_branch_tail_reduced_helper_reject_delay_slot = 0;
  u64 native_branch_tail_reduced_helper_reject_delay_slot_load = 0;
  u64 native_branch_tail_reduced_helper_load_lw = 0;
  u64 native_branch_tail_reduced_helper_load_lb = 0;
  u64 native_branch_tail_reduced_helper_load_lbu = 0;
  u64 native_branch_tail_reduced_helper_load_lh = 0;
  u64 native_branch_tail_reduced_helper_load_lhu = 0;
  u64 native_branch_tail_aggressive_reduced_helper_candidate_blocks = 0;
  u64 native_branch_tail_aggressive_reduced_helper_rejected_blocks = 0;
  u64 native_branch_tail_aggressive_reduced_helper_reject_disabled = 0;
  u64 native_branch_tail_aggressive_reduced_helper_reject_memory = 0;
  u64 native_branch_tail_aggressive_reduced_helper_reject_store = 0;
  u64 native_branch_tail_aggressive_reduced_helper_reject_store_sb = 0;
  u64 native_branch_tail_aggressive_reduced_helper_reject_store_sh = 0;
  u64 native_branch_tail_aggressive_reduced_helper_reject_store_other = 0;
  u64 native_branch_tail_aggressive_reduced_helper_reject_delay_slot_memory = 0;
  u64 native_branch_tail_aggressive_reduced_helper_reject_delay_slot_load = 0;
  u64 native_branch_tail_aggressive_reduced_helper_reject_delay_slot_store = 0;
  u64 native_branch_tail_aggressive_reduced_helper_reject_body = 0;
  u64 native_branch_tail_aggressive_reduced_helper_reject_delay_slot = 0;
  u64 native_branch_tail_aggressive_reduced_helper_load_candidate_blocks = 0;
  u64 native_branch_tail_aggressive_reduced_helper_store_candidate_blocks = 0;
  u64 native_branch_tail_aggressive_reduced_helper_mixed_load_store_candidate_blocks = 0;
  u64 native_branch_tail_aggressive_reduced_helper_store_sb = 0;
  u64 native_branch_tail_aggressive_reduced_helper_store_sh = 0;
  u64 native_branch_tail_aggressive_reduced_helper_store_sw = 0;
  u64 native_compile_failures = 0;
  u64 native_rejected_unsafe_blocks = 0;
  u64 native_to_decoded_fallbacks = 0;
  u64 native_hot_threshold_skips = 0;
  u64 native_short_block_skips = 0;
  u64 native_reject_branch = 0;
  u64 native_reject_memory = 0;
  u64 native_reject_cop0 = 0;
  u64 native_reject_cop2 = 0;
  u64 native_reject_exception_unknown = 0;
  u64 native_reject_exception_risk = 0;
  u64 native_reject_fallback_instruction = 0;
  u64 native_reject_unsupported_instruction = 0;
  u64 native_reject_unsafe_state = 0;
  u64 native_reject_pc_state = 0;
  u64 native_reject_branch_delay_state = 0;
  u64 native_reject_load_delay_state = 0;
  u64 native_reject_irq_state = 0;
  u64 native_reject_invalidated_state = 0;
  u64 native_reject_other_state = 0;
  u64 native_reject_pc_mismatch = 0;
  u64 native_reject_next_pc_mismatch = 0;
  u64 native_reject_block_start_after_branch_delay = 0;
  u64 native_reject_stale_invalid_block_state = 0;
  u64 native_reject_branch_delay_in_delay_slot = 0;
  u64 native_reject_branch_delay_pending_delay_slot = 0;
  u64 native_reject_branch_delay_pending_branch_taken = 0;
  u64 native_reject_branch_delay_pending_branch_pc = 0;
  u64 native_reject_budget = 0;
  u64 native_reject_icache = 0;
  u64 native_reject_mmio = 0;
  u64 native_reject_unaligned = 0;
  u64 native_branch_tail_rejects = 0;
  u64 native_branch_tail_disabled_fallbacks = 0;
  u64 native_branch_tail_blacklisted_fallbacks = 0;
  u64 native_all_disabled_fallbacks = 0;
  u64 native_memory_disabled_fallbacks = 0;
  u64 native_load_disabled_fallbacks = 0;
  u64 native_store_disabled_fallbacks = 0;
  u64 native_mmio_disabled_fallbacks = 0;
  u64 native_ram_disabled_fallbacks = 0;
  u64 native_load_delay_disabled_fallbacks = 0;
  u64 native_mixed_memory_disabled_fallbacks = 0;
  u64 native_alu_disabled_fallbacks = 0;
  u64 native_rejected_block_count = 0;
  u64 native_rejected_block_instructions = 0;
  u64 native_rejection_profiled_blocks = 0;
  u64 native_rejection_profiled_instructions = 0;
  u64 native_rejected_memory_only_blocks = 0;
  u64 native_rejected_prefix_blocks = 0;
  u64 native_rejected_prefix_instructions = 0;
  u64 native_rejected_prefix_total_instructions = 0;
  u64 native_rejected_single_blocker_blocks = 0;
  u64 native_rejected_single_unsupported_blocker_blocks = 0;
  u64 native_rejected_shape_straight_alu = 0;
  u64 native_rejected_shape_straight_alu_ram_load = 0;
  u64 native_rejected_shape_straight_alu_ram_store = 0;
  u64 native_rejected_shape_helper_safe_memory = 0;
  u64 native_rejected_shape_branch_tail = 0;
  u64 native_rejected_shape_fallback_control = 0;
  u64 native_rejected_shape_cop0 = 0;
  u64 native_rejected_shape_cop2 = 0;
  u64 native_rejected_shape_exception_unsafe = 0;
  std::array<u64, kDecodedOpStatsCount> native_rejected_opcode_counts{};
  std::array<u64, kDecodedOpStatsCount>
      native_rejected_first_blocker_opcode_counts{};
  u64 cache_hits = 0;
  u64 cache_misses = 0;
  u64 invalidations = 0;
  u64 invalidation_queries = 0;
  u64 invalidation_fast_no_code_page_exits = 0;
  u64 invalidation_blocks_examined = 0;
  u64 invalidation_blocks_invalidated = 0;
  u64 flushes = 0;
  u64 decoded_block_entries = 0;
  u64 native_block_entries = 0;
  u64 native_branch_tail_entries = 0;
  u64 native_branch_taken = 0;
  u64 native_branch_not_taken = 0;
  u64 native_branch_tail_to_decoded_fallbacks = 0;
  u64 native_branch_tail_completed_exits = 0;
  u64 native_branch_tail_budget_exits = 0;
  u64 native_branch_tail_fallback_exits = 0;
  u64 native_branch_tail_exception_exits = 0;
  u64 native_branch_tail_bgtz_entries = 0;
  u64 native_branch_tail_blez_entries = 0;
  u64 native_branch_tail_bgtz_taken = 0;
  u64 native_branch_tail_bgtz_not_taken = 0;
  u64 native_branch_tail_blez_taken = 0;
  u64 native_branch_tail_blez_not_taken = 0;
  u64 native_branch_tail_reject_bne = 0;
  u64 native_branch_tail_reject_beq = 0;
  u64 native_branch_tail_reject_bgtz = 0;
  u64 native_branch_tail_reject_blez = 0;
  u64 native_branch_tail_reject_other_opcode = 0;
  u64 native_memory_block_entries = 0;
  u64 native_alu_block_entries = 0;
  u64 native_reduced_helper_entries = 0;
  u64 native_reduced_helper_ram_load_entries = 0;
  u64 native_branch_tail_reduced_helper_entries = 0;
  u64 native_branch_tail_reduced_helper_ram_load_entries = 0;
  u64 native_branch_tail_aggressive_reduced_helper_entries = 0;
  u64 native_branch_tail_aggressive_reduced_helper_ram_load_entries = 0;
  u64 native_branch_tail_aggressive_reduced_helper_store_entries = 0;
  u64 native_branch_tail_aggressive_reduced_helper_mixed_memory_entries = 0;
  u64 native_reduced_helper_instructions = 0;
  u64 native_branch_tail_reduced_helper_instructions = 0;
  u64 native_branch_tail_reduced_helper_ram_load_instructions = 0;
  u64 native_branch_tail_aggressive_reduced_helper_instructions = 0;
  u64 native_branch_tail_aggressive_reduced_helper_ram_load_instructions = 0;
  u64 native_branch_tail_aggressive_reduced_helper_store_instructions = 0;
  u64 native_branch_tail_reduced_helper_prepare_helpers_avoided = 0;
  u64 native_branch_tail_reduced_helper_finish_helpers_avoided = 0;
  u64 native_branch_tail_reduced_helper_branch_helpers_avoided = 0;
  u64 native_branch_tail_reduced_helper_memory_helpers_avoided = 0;
  u64 native_branch_tail_aggressive_reduced_helper_prepare_helpers_avoided = 0;
  u64 native_branch_tail_aggressive_reduced_helper_finish_helpers_avoided = 0;
  u64 native_branch_tail_aggressive_reduced_helper_branch_helpers_avoided = 0;
  u64 native_branch_tail_aggressive_reduced_helper_memory_helpers_avoided = 0;
  u64 native_branch_tail_aggressive_reduced_helper_runtime_fallbacks = 0;
  u64 native_branch_tail_aggressive_reduced_helper_load_delay_fallbacks = 0;
  u64 native_branch_tail_aggressive_reduced_helper_irq_hook_fallbacks = 0;
  u64 native_branch_tail_aggressive_reduced_helper_preflight_fallbacks = 0;
  u64 native_branch_tail_aggressive_reduced_helper_preflight_mmio = 0;
  u64 native_branch_tail_aggressive_reduced_helper_preflight_unaligned = 0;
  u64 native_branch_tail_aggressive_reduced_helper_preflight_non_ram = 0;
  u64 native_branch_tail_aggressive_reduced_helper_preflight_scratchpad = 0;
  u64 native_branch_tail_aggressive_reduced_helper_preflight_disabled = 0;
  u64 native_branch_tail_aggressive_reduced_helper_preflight_code_page = 0;
  u64 native_branch_tail_aggressive_reduced_helper_preflight_passes = 0;
  u64 native_branch_tail_aggressive_reduced_helper_preflight_instructions = 0;
  u64 native_branch_tail_aggressive_reduced_helper_preflight_direct_candidate_blocks = 0;
  u64 native_branch_tail_aggressive_reduced_helper_preflight_direct_attempts = 0;
  u64 native_branch_tail_aggressive_reduced_helper_preflight_direct_checks = 0;
  u64 native_branch_tail_aggressive_reduced_helper_preflight_direct_fallbacks = 0;
  u64 native_branch_tail_aggressive_reduced_helper_preflight_full_attempts = 0;
  u64 native_branch_tail_aggressive_reduced_helper_preflight_full_fallbacks = 0;
  u64 native_branch_tail_aggressive_reduced_helper_adaptive_repeated_failures = 0;
  u64 native_branch_tail_aggressive_reduced_helper_adaptive_disabled_blocks = 0;
  u64 native_branch_tail_aggressive_reduced_helper_adaptive_direct_entries = 0;
  u64 native_branch_tail_aggressive_reduced_helper_adaptive_preflight_attempts_avoided = 0;
  u64 native_branch_tail_aggressive_reduced_helper_adaptive_disable_mmio = 0;
  u64 native_branch_tail_aggressive_reduced_helper_adaptive_disable_non_ram = 0;
  u64 native_branch_tail_aggressive_reduced_helper_adaptive_disable_scratchpad = 0;
  u64 native_branch_tail_aggressive_reduced_helper_adaptive_disable_unaligned = 0;
  u64 native_branch_tail_aggressive_reduced_helper_adaptive_disable_code_page = 0;
  u64 native_branch_tail_aggressive_reduced_helper_helper_fallback_entries = 0;
  u64 native_branch_tail_aggressive_reduced_helper_fast_store8 = 0;
  u64 native_branch_tail_aggressive_reduced_helper_fast_store16 = 0;
  u64 native_branch_tail_aggressive_reduced_helper_fast_store32 = 0;
  u64 native_branch_tail_reduced_helper_normal_fallbacks = 0;
  u64 native_branch_tail_reduced_helper_runtime_fallbacks = 0;
  u64 native_branch_tail_reduced_helper_load_delay_fallbacks = 0;
  u64 native_branch_tail_reduced_helper_irq_hook_fallbacks = 0;
  u64 native_branch_tail_reduced_helper_ram_load_preflight_fallbacks = 0;
  u64 native_branch_tail_reduced_helper_ram_load_preflight_mmio = 0;
  u64 native_branch_tail_reduced_helper_ram_load_preflight_unaligned = 0;
  u64 native_branch_tail_reduced_helper_ram_load_preflight_non_ram = 0;
  u64 native_branch_tail_reduced_helper_ram_load_preflight_disabled = 0;
  u64 native_reduced_helper_fallbacks = 0;
  u64 native_reduced_helper_load_delay_entries = 0;
  u64 native_reduced_helper_ram_load_preflight_fallbacks = 0;
  u64 native_reduced_helper_ram_load_preflight_mmio = 0;
  u64 native_reduced_helper_ram_load_preflight_unaligned = 0;
  u64 native_reduced_helper_ram_load_preflight_non_ram = 0;
  u64 native_reduced_helper_ram_load_preflight_disabled = 0;
  u64 native_direct_helper_candidate_blocks = 0;
  u64 native_reduced_helper_candidate_blocks = 0;
  u64 native_shape_straight_alu = 0;
  u64 native_shape_straight_alu_ram_load = 0;
  u64 native_shape_straight_alu_ram_store = 0;
  u64 native_shape_helper_safe_memory = 0;
  u64 native_shape_branch_tail = 0;
  u64 native_shape_fallback_control = 0;
  u64 native_shape_cop0 = 0;
  u64 native_shape_cop2 = 0;
  u64 native_shape_exception_unsafe = 0;
  u64 compare_flag_leak_warnings = 0;
  u64 optimized_instructions = 0;
  u64 decoded_instructions = 0;
  u64 native_instructions = 0;
  u64 native_cycles = 0;
  u64 fallback_instructions = 0;
  u64 interpreter_fallback_steps = 0;
  u64 forced_interpreter_slices = 0;
  u64 forced_interpreter_instructions = 0;
  u64 forced_interpreter_trace = 0;
  u64 forced_interpreter_deep_diagnostics = 0;
  u64 forced_interpreter_fmv = 0;
  u64 forced_interpreter_backend_compare = 0;
  u64 forced_interpreter_other = 0;
  CpuForcedInterpreterReason forced_interpreter_last_reason =
      CpuForcedInterpreterReason::None;
  u64 memory_helper_calls = 0;
  u64 native_prepare_helper_calls = 0;
  u64 native_finish_helper_calls = 0;
  u64 native_branch_helper_calls = 0;
  u64 native_memory_helper_calls = 0;
  u64 native_memory_helper_load_calls = 0;
  u64 native_memory_helper_store_calls = 0;
  u64 native_memory_helper_ram_calls = 0;
  u64 native_memory_helper_scratchpad_calls = 0;
  u64 native_memory_helper_bios_calls = 0;
  u64 native_memory_helper_mmio_calls = 0;
  u64 native_memory_helper_unknown_calls = 0;
  u64 native_memory_helper_unaligned_calls = 0;
  u64 native_branch_tail_prepare_helper_calls = 0;
  u64 native_branch_tail_finish_helper_calls = 0;
  u64 native_branch_tail_delay_slot_finish_helper_calls = 0;
  u64 native_branch_tail_branch_helper_calls = 0;
  u64 native_branch_tail_memory_helper_calls = 0;
  u64 native_branch_tail_memory_helper_load_calls = 0;
  u64 native_branch_tail_memory_helper_store_calls = 0;
  u64 native_memory_fastpath_loads = 0;
  u64 native_memory_fastpath_stores = 0;
  u64 native_memory_fastpath_mmio_loads = 0;
  u64 native_memory_fastpath_load_misses = 0;
  u64 native_memory_fastpath_load_miss_disabled = 0;
  u64 native_memory_fastpath_load_miss_trace = 0;
  u64 native_memory_fastpath_load_miss_unaligned = 0;
  u64 native_memory_fastpath_load_miss_non_ram = 0;
  u64 native_branch_tail_ram_load_fastpath_loads = 0;
  u64 native_branch_tail_ram_load_fastpath_load_misses = 0;
  u64 native_branch_tail_ram_load_fastpath_load_miss_disabled = 0;
  u64 native_branch_tail_ram_load_fastpath_load_miss_trace = 0;
  u64 native_branch_tail_ram_load_fastpath_load_miss_unaligned = 0;
  u64 native_branch_tail_ram_load_fastpath_load_miss_non_ram = 0;
  u64 native_memory_exception_exits = 0;
  u64 native_memory_operand_mismatches = 0;
  u64 native_memory_shared_decoded_calls = 0;
  u64 native_branch_delay_slot_memory_helpers = 0;
  u64 native_helper_load_delay_entries = 0;
  u64 native_helper_load_delay_passes = 0;
  u64 native_helper_load_delay_fallbacks = 0;
  u64 mmio_accesses = 0;
  u64 exceptions = 0;
  u64 interrupt_exits = 0;
  u64 budget_exits = 0;
  u64 fallback_exits = 0;
  u64 pc_mismatch_exits = 0;
  u64 invalidated_exits = 0;
  u64 executed_cycles = 0;
  size_t code_bytes = 0;
  size_t native_code_bytes = 0;
};

struct CpuDebugState {
  std::array<u32, 32> gpr{};
  u32 pc = 0;
  u32 next_pc = 0;
  u32 current_pc = 0;
  u32 hi = 0;
  u32 lo = 0;
  u32 load_reg = 0;
  u32 load_value = 0;
  u32 next_load_reg = 0;
  u32 next_load_value = 0;
  bool in_delay_slot = false;
  bool pending_delay_slot = false;
  bool pending_branch_taken = false;
  u32 pending_branch_pc = 0;
  u32 active_branch_pc = 0;
  bool exception_raised = false;
  u32 cop0_sr = 0;
  u32 cop0_cause = 0;
  u32 cop0_epc = 0;
  u32 cop0_badvaddr = 0;
  u64 cycles = 0;
};

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
  Trap = 0xD,
};

class Cpu {
public:
  Cpu();
  ~Cpu();

  void init(System *sys);
  void reset();

  // Execute one instruction and return the number of CPU cycles it consumed.
  u32 step();
  CpuRunSliceResult run_slice(u32 max_cycles, u32 max_instructions);
  u32 read_instruction_for_backend(u32 addr) const;
  void notify_code_write(u32 phys_or_normalized_addr, u32 size_bytes);
  void notify_cpu_backend_frame(u32 frame_index);
  void flush_cpu_backend();
  CpuBackendStats cpu_backend_stats() const;

  // COP2 (GTE) — publicly accessible for DMA
  Gte gte;

  // Debug access
  u32 pc() const { return pc_; }
  u32 reg(int i) const { return gpr_[i]; }
  u64 cycle_count() const { return cycles_; }
  void add_cycle_penalty(u32 cycles);
  CpuDebugState debug_state() const;
  void debug_set_state(const CpuDebugState &state);
  void debug_invalidate_icache_line(u32 addr);

private:
  System *sys_ = nullptr;
  std::unique_ptr<CpuOptimizedBackend> optimized_backend_;
  friend class CpuOptimizedBackend;

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
  u32 exception_return_regs_[32] = {};
  u32 exception_return_hi_ = 0;
  u32 exception_return_lo_ = 0;
  u32 exception_return_epc_ = 0;
  u32 exception_return_sr_ = 0;
  bool exception_return_bd_ = false;
  bool exception_return_valid_ = false;

  struct ICacheLine {
    u32 tag = 0;
    std::array<u32, 4> words = {};
    bool valid = false;
  };
  std::array<ICacheLine, 256> icache_ = {};

  u64 cycles_ = 0;
  u64 gte_input_ready_cycle_ = 0;
  u64 gte_result_ready_cycle_ = 0;
  u64 muldiv_result_ready_cycle_ = 0;
  u32 cycle_penalty_ = 0;
  bool executing_step_ = false;

  // ── Helpers ────────────────────────────────────────────────────
  void set_reg(u32 index, u32 value);
  void advance_load_delay();
  void flush_load_delay();
  void schedule_load(u32 index, u32 value);
  void begin_branch(bool taken, u32 target);
  u32 read_cop0_reg(u32 index) const;
  void write_cop0_reg(u32 index, u32 value);
  void raise_cop_unusable(u32 cop_index);
  u32 cpu_data_read_penalty(u32 addr) const;
  static bool gte_data_reg_reads_result(u32 reg);
  static bool gte_ctrl_reg_reads_result(u32 reg);
  u32 gte_input_stall_cycles() const;
  u32 gte_result_stall_cycles() const;
  u32 muldiv_stall_cycles() const;
  void stall_until_muldiv_complete();
  void mark_muldiv_result_pending(u32 ticks);
  static u32 mult_result_ticks(s32 value);
  static u32 multu_result_ticks(u32 value);
  static u32 div_result_ticks();
  u32 instruction_cycles(u32 instruction) const;
  static u32 gte_command_cycles(u32 instruction);
  bool instruction_cacheable(u32 addr) const;
  void invalidate_icache_line(u32 addr);

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
  void op_trap_special(u32 i);
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
