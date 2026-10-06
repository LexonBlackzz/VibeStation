#include "platform/cpu_backend_compare_runner.h"
#include "core/gte.h"
#include "core/system.h"
#include <array>
#include <memory>
#include <string_view>
#include <vector>

namespace {
constexpr u32 kCpuComparePc = 0x80010000u;

bool run_gte_final_accumulator_regression() {
  // The first two matrix products stay within the signed 44-bit accumulator,
  // while the third crosses the positive limit. The hardware exposes the
  // final, unwrapped result shifted by SF and then truncated to 32 bits.
  // Truncating the third intermediate result before that shift produces a
  // different MAC value.
  constexpr u32 kPositiveS16Pair = 0x7FFF7FFFu;
  constexpr u32 kTranslation = 0x7FF6D83Fu;
  constexpr u32 kExpectedMac = 0x8002D80Fu;
  constexpr u32 kMvmvaShifted = 0x00080012u;

  Gte gte;
  gte.write_ctrl(0, kPositiveS16Pair);
  gte.write_ctrl(1, kPositiveS16Pair);
  gte.write_ctrl(2, kPositiveS16Pair);
  gte.write_ctrl(3, kPositiveS16Pair);
  gte.write_ctrl(4, 0x00007FFFu);
  gte.write_ctrl(5, kTranslation);
  gte.write_ctrl(6, kTranslation);
  gte.write_ctrl(7, kTranslation);
  gte.write_data(0, kPositiveS16Pair);
  gte.write_data(1, 0x00007FFFu);
  gte.execute(kMvmvaShifted);

  const u32 mac1 = gte.read_data(25);
  const u32 mac2 = gte.read_data(26);
  const u32 mac3 = gte.read_data(27);
  const bool passed =
      mac1 == kExpectedMac && mac2 == kExpectedMac && mac3 == kExpectedMac;
  LOG_INFO(
      "GTE_REGRESSION name=final_accumulator_shift result=%s mac1=0x%08X "
      "mac2=0x%08X mac3=0x%08X expected=0x%08X flags=0x%08X",
      passed ? "PASS" : "FAIL", mac1, mac2, mac3, kExpectedMac,
      gte.read_ctrl(31));
  return passed;
}

bool run_gte_register_sign_extension_regression() {
  Gte gte;
  gte.write_data(1, 0x00008001u);
  gte.write_data(3, 0x0000FFFFu);
  gte.write_data(5, 0x00007FFFu);
  gte.write_ctrl(26, 0x00008001u);

  const u32 vz0 = gte.read_data(1);
  const u32 vz1 = gte.read_data(3);
  const u32 vz2 = gte.read_data(5);
  const u32 h = gte.read_ctrl(26);
  const bool passed = vz0 == 0xFFFF8001u && vz1 == 0xFFFFFFFFu &&
                      vz2 == 0x00007FFFu && h == 0xFFFF8001u;
  LOG_INFO(
      "GTE_REGRESSION name=register_sign_extension result=%s "
      "vz0=0x%08X vz1=0x%08X vz2=0x%08X h=0x%08X",
      passed ? "PASS" : "FAIL", vz0, vz1, vz2, h);
  return passed;
}

bool run_gte_writable_mac_regression() {
  // Spyro clears MAC1-3 with MTC2 before using GPL as a vector scaler. If
  // writes to the MAC registers are ignored, GPL incorporates stale results
  // from an unrelated command and produces enormous, nondeterministic steps.
  constexpr u32 kGplUnshifted = 0x00A0003Eu;
  constexpr u32 kScale = 0x00001000u;
  constexpr u32 kIr1 = 0x00000123u;
  constexpr u32 kIr2 = 0xFFFFFF45u;
  constexpr u32 kIr3 = 0x00000007u;

  Gte gte;
  gte.write_data(8, kScale);
  gte.write_data(9, kIr1);
  gte.write_data(10, kIr2);
  gte.write_data(11, kIr3);
  gte.write_data(25, 0x12345678u);
  gte.write_data(26, 0x87654321u);
  gte.write_data(27, 0x55AA55AAu);
  gte.write_data(25, 0u);
  gte.write_data(26, 0u);
  gte.write_data(27, 0u);
  gte.execute(kGplUnshifted);

  const u32 mac1 = gte.read_data(25);
  const u32 mac2 = gte.read_data(26);
  const u32 mac3 = gte.read_data(27);
  const u32 expected_mac1 = kIr1 << 12;
  const u32 expected_mac2 = kIr2 << 12;
  const u32 expected_mac3 = kIr3 << 12;
  const bool passed = mac1 == expected_mac1 && mac2 == expected_mac2 &&
                      mac3 == expected_mac3;
  LOG_INFO(
      "GTE_REGRESSION name=writable_mac_gpl result=%s mac=(0x%08X,0x%08X,0x%08X) "
      "expected=(0x%08X,0x%08X,0x%08X)",
      passed ? "PASS" : "FAIL", mac1, mac2, mac3, expected_mac1,
      expected_mac2, expected_mac3);
  return passed;
}

struct CpuCompareMemoryWord {
  u32 addr = 0;
  u32 value = 0;
};

struct CpuCompareCodeMutation {
  u32 after_instructions = 0;
  u32 addr = 0;
  u32 value = 0;
  bool invalidate_icache_line = true;
  bool prime_sio_transfer = false;
  // Host JIT maintenance only: discard every translation at this boundary
  // without touching guest memory (addr/value are ignored).
  bool flush_backend = false;
};

struct CpuCompareNativeTierMode {
  bool all_native = true;
  bool memory_native = true;
  bool alu_native = true;
};

struct CpuCompareCase {
  const char *name = "";
  u32 start_pc = kCpuComparePc;
  std::vector<u32> program;
  std::vector<CpuCompareMemoryWord> memory;
  std::vector<u32> compare_memory_addresses;
  std::vector<CpuCompareCodeMutation> mutations;
  std::vector<u32> segment_instructions;
  std::vector<CpuCompareNativeTierMode> segment_native_tiers;
  std::array<u32, 32> initial_gpr{};
  u32 initial_load_reg = 0;
  u32 initial_load_value = 0;
  u32 initial_cop0_sr_bits = 0;
  u32 initial_irq_mask = 0;
  bool initial_irq_pending = false;
  bool prime_sio_before_run = false;
  bool prime_cdrom_irq_before_run = false;
  u32 initial_next_pc = 0;
  bool initial_pending_delay_slot = false;
  bool initial_pending_branch_taken = false;
  u32 initial_pending_branch_pc = 0;
  bool request_irq_on_branch = false;
  u32 instructions = 0;
  bool expect_final_control_state = false;
  u32 expected_pc = 0;
  u32 expected_next_pc = 0;
  u32 expected_current_pc = 0;
  u64 expected_cycles = 0;
  // Sync-on-access checks. After the run, (gpr[expect_gpr_reg] & mask) must
  // equal the CPU cycle count at the end of segment `segment_index` (the
  // start-of-instruction cycle of the next instruction), or a fixed value.
  int expect_gpr_reg = -1;
  u32 expect_gpr_mask = 0xFFFFFFFFu;
  bool expect_gpr_is_segment_cycles = false;
  size_t expect_gpr_segment = 0;
  u32 expect_gpr_value = 0;
  // Optional: (GTE FLAG & mask) must equal value after the run.
  u32 expect_gte_flags_mask = 0;
  u32 expect_gte_flags_value = 0;
  bool experimental_unknown_fallback = false;
  bool require_full_native_when_available = false;
  bool require_native_entry_when_available = false;
  bool require_v2_native_entry_when_available = false;
  bool require_v4_native_entry_when_available = false;
  bool require_v4_native_load_entry_when_available = false;
  bool require_v4_native_store_entry_when_available = false;
  bool require_v4_store_tail_block_when_available = false;
  bool require_v4_store_branch_fusion_when_available = false;
  bool require_v4_store_smc_native_when_available = false;
  bool require_v4_load_tail_block_when_available = false;
  bool require_v4_load_branch_fusion_when_available = false;
  bool require_v4_native_branch_entry_when_available = false;
  bool require_v4_pending_delay_native_when_available = false;
  bool require_v4_hot_mmio16_native_when_available = false;
  bool require_v4_mmio_native_when_available = false;
  bool require_v4_hilo_native_when_available = false;
  bool require_v4_muldiv_native_when_available = false;
  bool require_v4_cop0_native_when_available = false;
  bool require_v4_cop2_native_when_available = false;
  bool require_v4_unaligned_native_when_available = false;
  bool require_v4_exception_native_when_available = false;
  bool require_v4_entry_exception_native_when_available = false;
  bool require_v4_folded_branch_block_when_available = false;
  bool require_v4_page_local_invalidation_when_available = false;
  bool require_v4_cached_same_page_retention_when_available = false;
  bool require_v4_icache_revalidation_when_available = false;
  bool require_v4_native_icache_revalidation_when_available = false;
  bool require_v4_native_chain_when_available = false;
  bool require_v4_crossline_block_when_available = false;
  bool require_v4_constant_address_memory_when_available = false;
  bool require_v2_store_branch_entry_when_available = false;
  bool require_v2_branch_not_taken_entry_when_available = false;
  bool require_v2_helper_entry_when_available = false;
  bool require_native_memory_helper_when_available = false;
  bool require_native_memory_exception_when_available = false;
  bool require_native_helper_load_delay_entry_when_available = false;
  bool require_native_branch_tail_when_available = false;
  bool native_branch_should_be_taken = false;
  u8 native_branch_primary_op = 0xFFu;
  bool require_native_branch_delay_memory_helper_when_available = false;
  bool require_native_mmio_when_available = false;
  bool disable_branch_tail_for_x64 = false;
  bool blacklist_branch_tail_for_x64 = false;
  bool require_branch_tail_disabled_fallback_when_available = false;
  bool require_branch_tail_blacklisted_fallback_when_available = false;
  bool allow_partial_native_branch_tail = false;
  bool enable_reduced_helper_branch_tail_for_x64 = false;
  bool allow_partial_native_memory_helper = false;
  bool compare_segment_states = false;
  // Most compare cases want a logical segment to continue after a scheduler
  // timing boundary, mirroring System::run_frame(). A dedicated boundary case
  // can keep the short return visible instead.
  bool preserve_timing_boundary_short_return = false;
  u32 run_slice_cycle_budget = 100000u;
  bool disable_all_native_for_x64 = false;
  bool disable_memory_native_for_x64 = false;
  bool disable_alu_native_for_x64 = false;
  bool enable_aggressive_reduced_helper_branch_tail_for_x64 = false;
  bool enable_native_prefix_for_x64 = false;
  bool enable_aggressive_native_prefix_ram_for_x64 = false;
  bool require_all_native_disabled_fallback_when_available = false;
  bool require_memory_native_disabled_fallback_when_available = false;
  bool require_alu_native_disabled_fallback_when_available = false;
  bool require_native_memory_tier_entry_when_available = false;
  bool require_native_alu_tier_entry_when_available = false;
  bool require_native_reduced_helper_ram_load_entry_when_available = false;
  bool require_native_reduced_helper_branch_tail_entry_when_available = false;
  bool require_native_reduced_helper_branch_tail_ram_load_entry_when_available =
      false;
  bool require_native_aggressive_reduced_helper_branch_tail_entry_when_available =
      false;
  bool require_native_aggressive_reduced_helper_branch_tail_store_entry_when_available =
      false;
  bool require_native_aggressive_reduced_helper_branch_tail_mixed_entry_when_available =
      false;
  bool require_native_prefix_entry_when_available = false;
  bool require_native_prefix_bne_blocker_when_available = false;
  bool require_native_prefix_beq_blocker_when_available = false;
  bool require_native_prefix_jr_blocker_when_available = false;
  bool require_native_prefix_cop2_blocker_when_available = false;
  bool require_native_prefix_other_blocker_when_available = false;
  bool require_native_prefix_ram_load_entry_when_available = false;
  bool require_native_prefix_ram_load_aggressive_entry_when_available = false;
  bool require_native_prefix_ram_load_full_preflight_when_available = false;
  bool require_native_prefix_ram_load_preflight_non_ram_when_available = false;
  bool require_native_prefix_ram_load_adaptive_disable_when_available = false;
  bool require_native_prefix_ram_load_adaptive_direct_entry_when_available =
      false;
  bool require_native_prefix_reject_store_when_available = false;
  bool require_no_native_reduced_helper_ram_load_entry = false;
  bool require_no_native_reduced_helper_branch_tail_ram_load_entry = false;
  bool require_no_native_instruction_helpers_when_available = false;
  bool require_no_native_branch_tail_helpers_when_available = false;
  bool require_reduced_helper_preflight_mmio_when_available = false;
  bool require_reduced_helper_preflight_unaligned_when_available = false;
  bool require_reduced_helper_preflight_non_ram_when_available = false;
  bool require_reduced_helper_branch_tail_preflight_mmio_when_available =
      false;
  bool require_reduced_helper_branch_tail_preflight_unaligned_when_available =
      false;
  bool require_reduced_helper_branch_tail_preflight_non_ram_when_available =
      false;
  bool require_reduced_helper_branch_tail_reject_load_base_written_when_available =
      false;
  bool require_aggressive_reduced_helper_branch_tail_preflight_non_ram_when_available =
      false;
  bool require_aggressive_reduced_helper_branch_tail_preflight_code_page_when_available =
      false;
  bool require_aggressive_reduced_helper_branch_tail_direct_preflight_when_available =
      false;
  bool require_aggressive_reduced_helper_branch_tail_full_preflight_when_available =
      false;
  bool require_aggressive_reduced_helper_branch_tail_adaptive_disable_when_available =
      false;
  bool require_aggressive_reduced_helper_branch_tail_adaptive_direct_entry_when_available =
      false;
  bool enable_ram_load_fastpath_for_x64 = false;
  bool require_native_ram_load_fastpath_when_available = false;
  bool require_no_native_ram_load_fastpath = false;
  bool expect_x64_fallback = false;
};

struct CpuComparePeripheralState {
  u32 irq_stat = 0;
  u32 irq_mask = 0;
  u32 dma_dpcr = 0;
  u32 dma_dicr = 0;
  u32 sio_cycles_until_event = 0;
  u64 cd_sector_count = 0;
  int cd_read_lba = 0;
  int cd_active_lba = 0;
  int cd_data_index = 0;
  int cd_busy_cycles = 0;
  size_t cd_pending_irqs = 0;
  size_t cd_response_size = 0;
  u8 cd_last_irq = 0;
  bool cd_data_ready = false;
  bool cd_data_request = false;
  u64 cd_last_irq_clear_cycle = 0;
  u64 cd_io_count = 0;
  u64 first_cd_io_cycle = 0;
  u64 sio_io_count = 0;
  u64 first_sio_io_cycle = 0;
};

static bool cpu_compare_peripherals_equal(
    const CpuComparePeripheralState &a,
    const CpuComparePeripheralState &b) {
  return a.irq_stat == b.irq_stat && a.irq_mask == b.irq_mask &&
         a.dma_dpcr == b.dma_dpcr && a.dma_dicr == b.dma_dicr &&
         a.sio_cycles_until_event == b.sio_cycles_until_event &&
         a.cd_sector_count == b.cd_sector_count &&
         a.cd_read_lba == b.cd_read_lba &&
         a.cd_active_lba == b.cd_active_lba &&
         a.cd_data_index == b.cd_data_index &&
         a.cd_busy_cycles == b.cd_busy_cycles &&
         a.cd_pending_irqs == b.cd_pending_irqs &&
         a.cd_response_size == b.cd_response_size &&
         a.cd_last_irq == b.cd_last_irq &&
         a.cd_data_ready == b.cd_data_ready &&
         a.cd_data_request == b.cd_data_request &&
         a.cd_last_irq_clear_cycle == b.cd_last_irq_clear_cycle &&
         a.cd_io_count == b.cd_io_count &&
         a.first_cd_io_cycle == b.first_cd_io_cycle &&
         a.sio_io_count == b.sio_io_count &&
         a.first_sio_io_cycle == b.first_sio_io_cycle;
}

struct CpuCompareRunResult {
  CpuDebugState state{};
  CpuBackendStats stats{};
  CpuRunSliceResult run{};
  std::vector<CpuDebugState> segment_states;
  std::vector<CpuComparePeripheralState> segment_peripherals;
  CpuComparePeripheralState peripherals{};
  u32 irq_stat = 0;
  u32 irq_mask = 0;
  u32 gte_flags = 0;
  std::vector<u32> memory_values;
};

static CpuComparePeripheralState capture_cpu_compare_peripherals(
    System &sys) {
  const CdRom &cd = sys.cdrom();
  CpuComparePeripheralState out{};
  out.irq_stat = sys.irq().stat();
  out.irq_mask = sys.irq().mask();
  out.dma_dpcr = sys.debug_dma_read(0x70u);
  out.dma_dicr = sys.debug_dma_read(0x74u);
  out.sio_cycles_until_event = sys.sio().cycles_until_event();
  out.cd_sector_count = cd.sector_count();
  out.cd_read_lba = cd.current_read_lba();
  out.cd_active_lba = cd.active_data_lba();
  out.cd_data_index = cd.dma_data_index();
  out.cd_busy_cycles = cd.busy_cycles_remaining();
  out.cd_pending_irqs = cd.pending_irq_count();
  out.cd_response_size = cd.response_fifo_size();
  out.cd_last_irq = cd.last_irq_code();
  out.cd_data_ready = cd.sector_data_ready();
  out.cd_data_request = cd.sector_data_request();
  out.cd_last_irq_clear_cycle = cd.debug_last_irq_clear_cycle();
  const auto &boot = sys.boot_diag();
  out.cd_io_count = boot.cd_io_count;
  out.first_cd_io_cycle = boot.first_cd_io_cycle;
  out.sio_io_count = boot.sio_io_count;
  out.first_sio_io_cycle = boot.first_sio_io_cycle;
  return out;
}

static u32 enc_r(u32 rs, u32 rt, u32 rd, u32 shamt, u32 funct) {
  return ((rs & 31u) << 21) | ((rt & 31u) << 16) | ((rd & 31u) << 11) |
         ((shamt & 31u) << 6) | (funct & 63u);
}

static u32 enc_i(u32 op, u32 rs, u32 rt, u16 imm) {
  return ((op & 63u) << 26) | ((rs & 31u) << 21) | ((rt & 31u) << 16) |
         imm;
}

static u32 enc_j(u32 op, u32 target) {
  return ((op & 63u) << 26) | ((target >> 2) & 0x03FFFFFFu);
}

static const char *cpu_compare_mode_name(CpuExecutionMode mode) {
  switch (mode) {
  case CpuExecutionMode::DecodedBlockInterpreter:
    return "DecodedBlockInterpreter";
  case CpuExecutionMode::X64Jit:
    return "X64Jit";
  case CpuExecutionMode::X64JitV2:
    return "X64JitV2";
  case CpuExecutionMode::X64JitV3:
    return "X64JitV3";
  case CpuExecutionMode::Recompiler:
    return "Recompiler";
  case CpuExecutionMode::Interpreter:
  default:
    return "Interpreter";
  }
}

static const char *cpu_compare_outcome(CpuExecutionMode mode,
                                       const CpuBackendStats &stats) {
  if (mode == CpuExecutionMode::Interpreter) {
    return "interpreter";
  }

  if (stats.native_block_entries != 0 || stats.native_instructions != 0) {
    if (stats.decoded_instructions != 0 || stats.fallback_instructions != 0 ||
        stats.interpreter_fallback_steps != 0) {
      return "native_with_fallback";
    }
    return "native";
  }

  if (mode == CpuExecutionMode::X64Jit && !stats.native_available) {
    if (stats.decoded_instructions != 0 || stats.decoded_block_entries != 0) {
      return "x64_unavailable_decoded_fallback";
    }
    if (stats.interpreter_fallback_steps != 0 ||
        stats.fallback_instructions != 0) {
      return "x64_unavailable_interpreter_fallback";
    }
    return "x64_unavailable_no_optimized_work";
  }

  if (stats.decoded_instructions != 0 || stats.decoded_block_entries != 0) {
    if (stats.interpreter_fallback_steps != 0 ||
        stats.fallback_instructions != 0) {
      return mode == CpuExecutionMode::X64Jit
                 ? "decoded_with_interpreter_fallback"
                 : "decoded_interpreter_fallback";
    }
    return mode == CpuExecutionMode::X64Jit ? "decoded_fallback"
                                            : "decoded";
  }

  if (stats.interpreter_fallback_steps != 0 || stats.fallback_instructions != 0) {
    return mode == CpuExecutionMode::X64Jit ? "interpreter_fallback"
                                            : "fallback";
  }

  return "no_optimized_work";
}

static void log_cpu_compare_program(const CpuCompareCase &test_case) {
  for (size_t i = 0; i < test_case.program.size(); ++i) {
    LOG_ERROR("CPU_COMPARE_PROGRAM name=%s index=%u pc=0x%08X opcode=0x%08X",
              test_case.name, static_cast<unsigned>(i),
              test_case.start_pc + static_cast<u32>(i * 4u),
              test_case.program[i]);
  }
}

static void log_cpu_compare_failure_summary(
    const CpuCompareCase &test_case, CpuExecutionMode mode,
    const CpuCompareRunResult &reference, const CpuCompareRunResult &actual,
    bool state_pass, bool segment_state_pass, bool irq_state_pass,
    bool memory_state_pass, bool peripheral_state_pass,
    bool segment_peripheral_pass, bool expected_state_pass,
    bool native_check_pass, const char *native_check) {
  int first_reg = -1;
  u32 first_reg_ref = 0;
  u32 first_reg_actual = 0;
  for (u32 i = 0; i < 32u; ++i) {
    if (reference.state.gpr[i] != actual.state.gpr[i]) {
      first_reg = static_cast<int>(i);
      first_reg_ref = reference.state.gpr[i];
      first_reg_actual = actual.state.gpr[i];
      break;
    }
  }

  LOG_ERROR(
      "CPU_COMPARE_FAIL name=%s ref=Interpreter mode=%s state=%u segment=%u irq=%u mem=%u "
      "periph=%u seg_periph=%u expected=%u native=%u native_check=%s "
      "pc=%08X/%08X next=%08X/%08X current=%08X/%08X cyc=%llu/%llu "
      "cause=%08X/%08X epc=%08X/%08X bad=%08X/%08X "
      "delay=%u/%u branch_pc=%08X/%08X "
      "first_reg=%d:%08X/%08X native_instr=%llu decoded_instr=%llu "
      "fallback_instr=%llu icache_refills=%llu helper_instr=%llu "
      "dispatch=%llu missing=%llu generation=%llu budget=%llu bail=%llu",
      test_case.name, cpu_compare_mode_name(mode),
      state_pass ? 1u : 0u, segment_state_pass ? 1u : 0u,
      irq_state_pass ? 1u : 0u, memory_state_pass ? 1u : 0u,
      peripheral_state_pass ? 1u : 0u,
      segment_peripheral_pass ? 1u : 0u,
      expected_state_pass ? 1u : 0u, native_check_pass ? 1u : 0u,
      native_check,
      reference.state.pc, actual.state.pc,
      reference.state.next_pc, actual.state.next_pc,
      reference.state.current_pc, actual.state.current_pc,
      static_cast<unsigned long long>(reference.state.cycles),
      static_cast<unsigned long long>(actual.state.cycles),
      reference.state.cop0_cause, actual.state.cop0_cause,
      reference.state.cop0_epc, actual.state.cop0_epc,
      reference.state.cop0_badvaddr, actual.state.cop0_badvaddr,
      reference.state.pending_delay_slot ? 1u : 0u,
      actual.state.pending_delay_slot ? 1u : 0u,
      reference.state.pending_branch_pc, actual.state.pending_branch_pc,
      first_reg, first_reg_ref, first_reg_actual,
      static_cast<unsigned long long>(actual.stats.native_instructions),
      static_cast<unsigned long long>(actual.stats.decoded_instructions),
      static_cast<unsigned long long>(actual.stats.fallback_instructions),
      static_cast<unsigned long long>(actual.stats.recompiler_frame_icache_refills),
      static_cast<unsigned long long>(actual.stats.jit_v4_helper_instructions),
      static_cast<unsigned long long>(actual.stats.recompiler_frame_native_dispatches),
      static_cast<unsigned long long>(actual.stats.recompiler_frame_dispatch_missing_exits),
      static_cast<unsigned long long>(actual.stats.recompiler_frame_dispatch_generation_exits),
      static_cast<unsigned long long>(actual.stats.recompiler_frame_dispatch_budget_exits),
      static_cast<unsigned long long>(actual.stats.recompiler_frame_dispatch_bail_exits));
}

static bool cpu_debug_states_equal(const CpuDebugState &a,
                                   const CpuDebugState &b) {
  for (u32 i = 0; i < 32u; ++i) {
    if (a.gpr[i] != b.gpr[i]) {
      return false;
    }
  }

  return a.pc == b.pc && a.next_pc == b.next_pc &&
         a.current_pc == b.current_pc && a.hi == b.hi && a.lo == b.lo &&
         a.load_reg == b.load_reg && a.load_value == b.load_value &&
         a.next_load_reg == b.next_load_reg &&
         a.next_load_value == b.next_load_value &&
         a.in_delay_slot == b.in_delay_slot &&
         a.pending_delay_slot == b.pending_delay_slot &&
         a.pending_branch_taken == b.pending_branch_taken &&
         a.pending_branch_pc == b.pending_branch_pc &&
         a.active_branch_pc == b.active_branch_pc &&
         a.exception_raised == b.exception_raised &&
         a.cop0_sr == b.cop0_sr && a.cop0_cause == b.cop0_cause &&
         a.cop0_epc == b.cop0_epc &&
         a.cop0_badvaddr == b.cop0_badvaddr && a.cycles == b.cycles;
}

static void log_cpu_debug_state_diff(const char *name,
                                     const char *actual_mode,
                                     const CpuDebugState &reference,
                                     const CpuDebugState &actual) {
  auto field = [&](const char *field_name, auto a, auto b) {
    if (a != b) {
      LOG_ERROR("CPU_COMPARE_DIFF name=%s mode=%s field=%s reference=0x%llX actual=0x%llX",
                name, actual_mode, field_name,
                static_cast<unsigned long long>(a),
                static_cast<unsigned long long>(b));
    }
  };

  field("pc", reference.pc, actual.pc);
  field("next_pc", reference.next_pc, actual.next_pc);
  field("current_pc", reference.current_pc, actual.current_pc);
  field("hi", reference.hi, actual.hi);
  field("lo", reference.lo, actual.lo);
  field("load_reg", reference.load_reg, actual.load_reg);
  field("load_value", reference.load_value, actual.load_value);
  field("next_load_reg", reference.next_load_reg, actual.next_load_reg);
  field("next_load_value", reference.next_load_value,
        actual.next_load_value);
  field("in_delay_slot", reference.in_delay_slot ? 1u : 0u,
        actual.in_delay_slot ? 1u : 0u);
  field("pending_delay_slot", reference.pending_delay_slot ? 1u : 0u,
        actual.pending_delay_slot ? 1u : 0u);
  field("pending_branch_taken", reference.pending_branch_taken ? 1u : 0u,
        actual.pending_branch_taken ? 1u : 0u);
  field("pending_branch_pc", reference.pending_branch_pc,
        actual.pending_branch_pc);
  field("active_branch_pc", reference.active_branch_pc,
        actual.active_branch_pc);
  field("exception_raised", reference.exception_raised ? 1u : 0u,
        actual.exception_raised ? 1u : 0u);
  field("cop0_sr", reference.cop0_sr, actual.cop0_sr);
  field("cop0_cause", reference.cop0_cause, actual.cop0_cause);
  field("cop0_epc", reference.cop0_epc, actual.cop0_epc);
  field("cop0_badvaddr", reference.cop0_badvaddr, actual.cop0_badvaddr);
  field("cycles", reference.cycles, actual.cycles);

  for (u32 i = 0; i < 32u; ++i) {
    if (reference.gpr[i] != actual.gpr[i]) {
      LOG_ERROR("CPU_COMPARE_DIFF name=%s mode=%s reg=r%u reference=0x%08X actual=0x%08X",
                name, actual_mode, static_cast<unsigned>(i), reference.gpr[i],
                actual.gpr[i]);
    }
  }
}

static bool cpu_compare_expected_state_pass(const CpuCompareCase &test_case,
                                            CpuExecutionMode mode,
                                            const CpuCompareRunResult &result) {
  const CpuDebugState &state = result.state;
  bool pass = true;
  auto field = [&](const char *field_name, auto expected, auto actual) {
    if (expected == actual) {
      return;
    }
    pass = false;
    (void)field_name;
    (void)expected;
    (void)actual;
  };

  if (test_case.expect_final_control_state) {
    field("pc", test_case.expected_pc, state.pc);
    field("next_pc", test_case.expected_next_pc, state.next_pc);
    field("current_pc", test_case.expected_current_pc, state.current_pc);
    field("cycles", test_case.expected_cycles, state.cycles);
  }

  if (test_case.expect_gpr_reg >= 0) {
    const u32 actual =
        state.gpr[static_cast<size_t>(test_case.expect_gpr_reg)] &
        test_case.expect_gpr_mask;
    u32 expected = test_case.expect_gpr_value & test_case.expect_gpr_mask;
    if (test_case.expect_gpr_is_segment_cycles) {
      expected = result.segment_states.size() > test_case.expect_gpr_segment
                     ? static_cast<u32>(
                           result.segment_states[test_case.expect_gpr_segment]
                               .cycles) &
                           test_case.expect_gpr_mask
                     : 0xDEADBEEFu;
    }
    if (actual != expected) {
      LOG_ERROR(
          "CPU_COMPARE_EXPECT name=%s mode=%s r%d=0x%08X expected=0x%08X",
          test_case.name, cpu_compare_mode_name(mode), test_case.expect_gpr_reg,
          actual, expected);
    }
    field("gpr", expected, actual);
  }
  if (test_case.expect_gte_flags_mask != 0u) {
    const u32 actual = result.gte_flags & test_case.expect_gte_flags_mask;
    if (actual != test_case.expect_gte_flags_value) {
      LOG_ERROR(
          "CPU_COMPARE_EXPECT name=%s mode=%s gte_flags=0x%08X expected=0x%08X",
          test_case.name, cpu_compare_mode_name(mode), actual,
          test_case.expect_gte_flags_value);
    }
    field("gte_flags", test_case.expect_gte_flags_value, actual);
  }
  return pass;
}

static CpuCompareRunResult run_cpu_compare_case_once(
    const CpuCompareCase &test_case, CpuExecutionMode mode) {
  auto sys = std::make_unique<System>();
  sys->init_hardware();
  sys->reset();

  for (size_t i = 0; i < test_case.program.size(); ++i) {
    sys->write32((test_case.start_pc & 0x1FFFFFFFu) +
                     static_cast<u32>(i * 4u),
                 test_case.program[i]);
  }
  for (const CpuCompareMemoryWord &word : test_case.memory) {
    sys->write32(word.addr, word.value);
  }
  if (test_case.initial_irq_mask != 0u ||
      test_case.request_irq_on_branch || test_case.initial_irq_pending) {
    sys->irq().write(4u, test_case.initial_irq_mask);
  }
  if (test_case.initial_irq_pending) {
    sys->irq().request(Interrupt::VBlank);
  }

  CpuDebugState initial = sys->cpu().debug_state();
  initial.pc = test_case.start_pc;
  initial.next_pc =
      test_case.initial_next_pc != 0u ? test_case.initial_next_pc
                                     : test_case.start_pc + 4u;
  initial.current_pc = 0;
  initial.cycles = 0;
  initial.load_reg = test_case.initial_load_reg;
  initial.load_value = test_case.initial_load_value;
  initial.next_load_reg = 0;
  initial.next_load_value = 0;
  initial.in_delay_slot = false;
  initial.pending_delay_slot = test_case.initial_pending_delay_slot;
  initial.pending_branch_taken = test_case.initial_pending_branch_taken;
  initial.pending_branch_pc = test_case.initial_pending_branch_pc;
  initial.active_branch_pc = 0;
  initial.exception_raised = false;
  initial.cop0_sr |= test_case.initial_cop0_sr_bits;
  for (u32 i = 0; i < 32u; ++i) {
    initial.gpr[i] = test_case.initial_gpr[i];
  }
  initial.gpr[0] = 0;
  sys->cpu().debug_set_state(initial);
  sys->cpu().flush_cpu_backend();
  sys->cpu().notify_cpu_backend_frame(1);

  g_cpu_execution_mode_cli_override = true;
  g_cpu_execution_mode_cli_value = mode;
  g_cpu_backend_compare_irq_on_branch = test_case.request_irq_on_branch;
  g_cpu_backend_compare_allow_partial_branch_tail =
      mode == CpuExecutionMode::X64Jit &&
      test_case.allow_partial_native_branch_tail;
  g_cpu_backend_compare_allow_partial_memory_helper =
      mode == CpuExecutionMode::X64Jit &&
      test_case.allow_partial_native_memory_helper;
  g_cpu_x64_jit_branch_tail_cli_override = true;
  g_cpu_x64_jit_branch_tail_cli_value =
      !(mode == CpuExecutionMode::X64Jit &&
        test_case.disable_branch_tail_for_x64);
  g_cpu_x64_jit_branch_tail_blacklist.clear();
  if (mode == CpuExecutionMode::X64Jit &&
      test_case.blacklist_branch_tail_for_x64) {
    g_cpu_x64_jit_branch_tail_blacklist.push_back(test_case.start_pc);
  }
  g_cpu_x64_jit_all_native_cli_override = true;
  g_cpu_x64_jit_all_native_cli_value =
      !(mode == CpuExecutionMode::X64Jit &&
        test_case.disable_all_native_for_x64);
  g_cpu_x64_jit_native_memory_cli_override = true;
  g_cpu_x64_jit_native_memory_cli_value =
      !(mode == CpuExecutionMode::X64Jit &&
        test_case.disable_memory_native_for_x64);
  g_cpu_x64_jit_native_alu_cli_override = true;
  g_cpu_x64_jit_native_alu_cli_value =
      !(mode == CpuExecutionMode::X64Jit &&
        test_case.disable_alu_native_for_x64);
  g_cpu_x64_jit_ram_load_fastpath_enabled =
      mode == CpuExecutionMode::X64Jit &&
      test_case.enable_ram_load_fastpath_for_x64;
  g_cpu_x64_jit_reduced_helper_branch_tail_enabled =
      mode == CpuExecutionMode::X64Jit &&
      test_case.enable_reduced_helper_branch_tail_for_x64;
  g_cpu_x64_jit_aggressive_reduced_helper_branch_tail_enabled =
      mode == CpuExecutionMode::X64Jit &&
      test_case.enable_aggressive_reduced_helper_branch_tail_for_x64;
  g_cpu_x64_jit_native_prefix_enabled =
      mode == CpuExecutionMode::X64Jit &&
      test_case.enable_native_prefix_for_x64;
  g_cpu_x64_jit_aggressive_native_prefix_ram_enabled =
      mode == CpuExecutionMode::X64Jit &&
      test_case.enable_aggressive_native_prefix_ram_for_x64;
  if (test_case.prime_sio_before_run) {
    // Start a short PAD/SIO transfer at the exact pre-instruction CPU
    // timestamp. I-cache fetch/refill cycles for the first guest instruction
    // must not be observed by that instruction's own MMIO transaction.
    sys->write16(0x1F80104Eu, 1u);      // BAUD: 1 * 8 cycles
    sys->write16(0x1F80104Au, 0x0003u); // select + TX enable
    sys->write8(0x1F801040u, 0x01u);    // begin transfer
  }
  if (test_case.prime_cdrom_irq_before_run) {
    // Use the real CD-ROM command path to establish an active INT3. The guest
    // test can then acknowledge it from inside a resident native chain and
    // compare the device-visible acknowledgement timestamp with Interpreter.
    sys->write8(0x1F801800u, 0u);    // index 0
    sys->write8(0x1F801801u, 0x01u); // GetStat -> INT3
  }

  CpuCompareRunResult out{};
  u32 executed = 0;
  size_t segment_index = 0;
  auto run_segment = [&](u32 instruction_count) {
    if (instruction_count == 0) {
      return;
    }
    if (mode == CpuExecutionMode::X64Jit &&
        segment_index < test_case.segment_native_tiers.size()) {
      const CpuCompareNativeTierMode &tiers =
          test_case.segment_native_tiers[segment_index];
      g_cpu_x64_jit_all_native_cli_value = tiers.all_native;
      g_cpu_x64_jit_native_memory_cli_value = tiers.memory_native;
      g_cpu_x64_jit_native_alu_cli_value = tiers.alu_native;
    }
    CpuRunSliceResult segment{};
    while (segment.instructions < instruction_count &&
           segment.cycles < test_case.run_slice_cycle_budget) {
      const u32 remaining_instructions =
          instruction_count - segment.instructions;
      const u32 remaining_cycles =
          test_case.run_slice_cycle_budget - segment.cycles;
      const CpuRunSliceResult part =
          sys->cpu().run_slice(remaining_cycles, remaining_instructions);
      segment.cycles += part.cycles;
      segment.instructions += part.instructions;

      const bool timing_boundary =
          sys->cpu_timing_boundary_requested();
      if (timing_boundary &&
          !test_case.preserve_timing_boundary_short_return) {
        // System::run_frame() consumes this after resampling the device and
        // then continues with the remaining slice budget. Do the same for
        // multi-instruction compare segments so code mutations stay anchored
        // to their requested architectural instruction boundary.
        sys->consume_cpu_timing_boundary_request();
        if (part.instructions != 0u && part.cycles != 0u) {
          continue;
        }
      }
      break;
    }
    if (mode == CpuExecutionMode::Recompiler &&
        segment.instructions != instruction_count &&
        segment.cycles < test_case.run_slice_cycle_budget) {
      LOG_WARN(
          "CPU_COMPARE_SEGMENT_SHORT name=%s index=%zu requested=%u retired=%u "
          "pc=0x%08X next=0x%08X cycles=%u budget=%u boundary=%u",
          test_case.name, segment_index, instruction_count,
          segment.instructions, sys->cpu().debug_state().pc,
          sys->cpu().debug_state().next_pc, segment.cycles,
          test_case.run_slice_cycle_budget,
          sys->cpu_timing_boundary_requested() ? 1u : 0u);
    }
    out.run.cycles += segment.cycles;
    out.run.instructions += segment.instructions;
    executed += segment.instructions;
    out.segment_states.push_back(sys->cpu().debug_state());
    out.segment_peripherals.push_back(
        capture_cpu_compare_peripherals(*sys));
    ++segment_index;
  };

  if (!test_case.segment_instructions.empty()) {
    for (u32 instruction_count : test_case.segment_instructions) {
      if (executed >= test_case.instructions) {
        break;
      }
      const u32 remaining = test_case.instructions - executed;
      run_segment(std::min(instruction_count, remaining));
    }
  } else {
    for (const CpuCompareCodeMutation &mutation : test_case.mutations) {
      const u32 target = std::min(mutation.after_instructions,
                                  test_case.instructions);
      if (target > executed) {
        run_segment(target - executed);
      }
      if (mutation.flush_backend) {
        sys->cpu().flush_cpu_backend();
        continue;
      }
      sys->write32(mutation.addr, mutation.value);
      if (mutation.invalidate_icache_line) {
        sys->cpu().debug_invalidate_icache_line(mutation.addr);
      }
      if (mutation.prime_sio_transfer) {
        // Start an eight-cycle PAD/SIO transfer exactly at this CPU boundary.
        // A subsequent first-instruction MMIO access must observe this timestamp,
        // not a resident I-cache refill charged before that instruction retires.
        sys->write16(0x1F80104Eu, 1u);      // BAUD: 1 * 8 cycles
        sys->write16(0x1F80104Au, 0x0003u); // select + TX enable
        sys->write8(0x1F801040u, 0x01u);    // begin transfer
      }
    }
  }
  if (executed < test_case.instructions) {
    run_segment(test_case.instructions - executed);
  }
  out.state = sys->cpu().debug_state();
  out.gte_flags = sys->cpu().gte.read_ctrl(31);
  out.stats = sys->cpu().cpu_backend_stats();
  out.peripherals = capture_cpu_compare_peripherals(*sys);
  out.irq_stat = sys->irq().stat();
  out.irq_mask = sys->irq().mask();
  for (u32 addr : test_case.compare_memory_addresses) {
    out.memory_values.push_back(sys->read32(addr));
  }
  return out;
}

static void pad_cpu_compare_program(CpuCompareCase &test_case,
                                    u32 instruction_count = 16u) {
  while (test_case.program.size() < instruction_count) {
    test_case.program.push_back(0);
  }
  test_case.instructions = instruction_count;
}

static void append_deterministic_random_compare_cases(
    std::vector<CpuCompareCase> &cases) {
  struct RandomCaseSeed {
    const char *name;
    u32 value;
  };
  constexpr std::array<RandomCaseSeed, 4> seeds = {{
      {"random_block_seed_13579BDF", 0x13579BDFu},
      {"random_block_seed_2468ACE1", 0x2468ACE1u},
      {"random_block_seed_C001D00D", 0xC001D00Du},
      {"random_block_seed_5EED1234", 0x5EED1234u},
  }};

  for (const RandomCaseSeed &seed : seeds) {
    CpuCompareCase test{};
    test.name = seed.name;
    test.initial_gpr[1] = 0x80012000u;

    u32 random_state = seed.value;
    auto next_random = [&]() {
      random_state ^= random_state << 13u;
      random_state ^= random_state >> 17u;
      random_state ^= random_state << 5u;
      return random_state;
    };
    auto random_work_gpr = [&]() {
      return 2u + (next_random() % 14u);
    };

    for (u32 reg = 2; reg < 16u; ++reg) {
      test.initial_gpr[reg] = next_random();
    }
    for (u32 word = 0; word < 16u; ++word) {
      const u32 addr = 0x00012000u + word * 4u;
      test.memory.push_back({addr, next_random()});
      test.compare_memory_addresses.push_back(addr);
    }

    constexpr std::array<u32, 8> r_functions = {
        0x21u, 0x23u, 0x24u, 0x25u, 0x26u, 0x27u, 0x2Au, 0x2Bu,
    };
    constexpr std::array<u32, 7> immediate_ops = {
        0x09u, 0x0Au, 0x0Bu, 0x0Cu, 0x0Du, 0x0Eu, 0x0Fu,
    };
    constexpr std::array<u32, 8> memory_ops = {
        0x20u, 0x21u, 0x23u, 0x24u, 0x25u, 0x28u, 0x29u, 0x2Bu,
    };

    for (u32 instruction = 0; instruction < 16u; ++instruction) {
      const u32 family = next_random() % 4u;
      const u32 rs = random_work_gpr();
      const u32 rt = random_work_gpr();
      const u32 rd = random_work_gpr();
      if (family == 0u) {
        const u32 funct = r_functions[next_random() % r_functions.size()];
        test.program.push_back(enc_r(rs, rt, rd, 0u, funct));
      } else if (family == 1u) {
        const u32 funct = next_random() % 3u;
        const u32 shift_function =
            funct == 0u ? 0x00u : (funct == 1u ? 0x02u : 0x03u);
        test.program.push_back(
            enc_r(0u, rt, rd, next_random() & 31u, shift_function));
      } else if (family == 2u) {
        const u32 op =
            immediate_ops[next_random() % immediate_ops.size()];
        test.program.push_back(
            enc_i(op, op == 0x0Fu ? 0u : rs, rt,
                  static_cast<u16>(next_random())));
      } else {
        const u32 op = memory_ops[next_random() % memory_ops.size()];
        u32 byte_offset = next_random() & 0x3Fu;
        if (op == 0x21u || op == 0x25u || op == 0x29u) {
          byte_offset &= ~1u;
        } else if (op == 0x23u || op == 0x2Bu) {
          byte_offset &= ~3u;
        }
        test.program.push_back(
            enc_i(op, 1u, rt, static_cast<u16>(byte_offset)));
      }
    }

    test.instructions = static_cast<u32>(test.program.size());
    test.segment_instructions.assign(test.instructions, 1u);
    test.compare_segment_states = true;
    cases.push_back(std::move(test));
  }
}

static std::vector<CpuCompareCase> make_cpu_compare_cases() {
  std::vector<CpuCompareCase> cases;

  CpuCompareCase native_control{};
  native_control.name = "native_control_state_icache_cycles";
  native_control.program.assign(16u, 0u);
  native_control.instructions = 16;
  native_control.expect_final_control_state = true;
  native_control.expected_pc = kCpuComparePc + 16u * 4u;
  native_control.expected_next_pc = native_control.expected_pc + 4u;
  native_control.expected_current_pc = kCpuComparePc + 15u * 4u;
  native_control.expected_cycles = 32u;
  native_control.require_full_native_when_available = true;
  native_control.require_v4_native_entry_when_available = true;
  cases.push_back(native_control);

  CpuCompareCase all_native_disabled{};
  all_native_disabled.name = "x64_all_native_disabled_gate";
  all_native_disabled.program.assign(16u, 0u);
  all_native_disabled.instructions = 16;
  all_native_disabled.disable_all_native_for_x64 = true;
  all_native_disabled.expect_x64_fallback = true;
  all_native_disabled.require_all_native_disabled_fallback_when_available =
      true;
  cases.push_back(all_native_disabled);

  CpuCompareCase native_memory_disabled{};
  native_memory_disabled.name = "x64_native_memory_disabled_gate";
  native_memory_disabled.initial_gpr[1] = 0x80011180u;
  native_memory_disabled.memory.push_back({0x00011180u, 0x12345678u});
  native_memory_disabled.program = {enc_i(0x23, 1, 2, 0), 0};
  pad_cpu_compare_program(native_memory_disabled);
  native_memory_disabled.disable_memory_native_for_x64 = true;
  cases.push_back(native_memory_disabled);

  CpuCompareCase native_alu_disabled{};
  native_alu_disabled.name = "x64_native_alu_disabled_gate";
  native_alu_disabled.program.assign(16u, 0u);
  native_alu_disabled.instructions = 16;
  native_alu_disabled.disable_alu_native_for_x64 = true;
  cases.push_back(native_alu_disabled);

  CpuCompareCase jit_v2_native_smoke{};
  jit_v2_native_smoke.name = "jit_v2_native_register_cache_smoke";
  jit_v2_native_smoke.initial_gpr[1] = 0x12345678u;
  jit_v2_native_smoke.program = {
      enc_i(0x09, 1, 2, 7),
      enc_i(0x0E, 2, 3, 0x55AA),
      enc_r(2, 3, 4, 0, 0x21),
      enc_i(0x0C, 4, 5, 0x0FFF),
  };
  jit_v2_native_smoke.instructions = 4;
  jit_v2_native_smoke.require_v2_native_entry_when_available = true;
  cases.push_back(jit_v2_native_smoke);

  CpuCompareCase v4_multi_register_cache{};
  v4_multi_register_cache.name = "v4_multi_register_cache_interleaved";
  v4_multi_register_cache.program = {
      enc_i(0x09, 0, 1, 1),
      enc_i(0x09, 0, 2, 2),
      enc_i(0x09, 1, 3, 3),
      enc_r(2, 3, 4, 0, 0x21),
      enc_i(0x09, 0, 5, 1),
      enc_i(0x09, 0, 5, 2),
      enc_i(0x09, 5, 5, 3),
      enc_r(4, 5, 6, 0, 0x21),
  };
  v4_multi_register_cache.instructions =
      static_cast<u32>(v4_multi_register_cache.program.size());
  v4_multi_register_cache.require_full_native_when_available = true;
  v4_multi_register_cache.require_v4_native_entry_when_available = true;
  cases.push_back(v4_multi_register_cache);

  CpuCompareCase v4_crossline_block{};
  v4_crossline_block.name = "v4_promoted_crossline_native_block";
  v4_crossline_block.program = {
      enc_i(0x09, 1, 1, 1),
      enc_i(0x09, 2, 2, 2),
      enc_r(1, 2, 3, 0, 0x26),
      enc_r(3, 2, 4, 0, 0x21),
      enc_i(0x09, 6, 6, 3),
      enc_r(4, 6, 7, 0, 0x26),
      enc_r(7, 3, 8, 0, 0x21),
      enc_r(0, 8, 9, 1, 0x00),
      enc_i(0x09, 10, 10, 4),
      enc_r(9, 10, 11, 0, 0x25),
      enc_r(11, 7, 12, 0, 0x26),
      enc_r(12, 1, 13, 0, 0x21),
      enc_i(0x05, 1, 0, -13),
      enc_i(0x09, 5, 5, 1),
  };
  v4_crossline_block.instructions = 28u;
  v4_crossline_block.require_full_native_when_available = true;
  v4_crossline_block.require_v4_native_entry_when_available = true;
  v4_crossline_block.require_v4_native_chain_when_available = true;
  v4_crossline_block.require_v4_crossline_block_when_available = true;
  cases.push_back(v4_crossline_block);

  CpuCompareCase v4_constant_address_memory{};
  v4_constant_address_memory.name = "v4_constant_address_memory";
  v4_constant_address_memory.program = {
      enc_i(0x09, 0, 1, 0x1000),
      enc_i(0x09, 0, 2, 0x1234),
      enc_i(0x2B, 1, 2, 0),
      0u,
      enc_i(0x09, 0, 1, 0x1000),
      enc_i(0x23, 1, 3, 0),
      0u,
  };
  v4_constant_address_memory.instructions = 7u;
  v4_constant_address_memory.compare_memory_addresses = {0x00001000u};
  v4_constant_address_memory.require_full_native_when_available = true;
  v4_constant_address_memory.require_v4_native_entry_when_available = true;
  v4_constant_address_memory.require_v4_native_load_entry_when_available =
      true;
  v4_constant_address_memory.require_v4_native_store_entry_when_available =
      true;
  v4_constant_address_memory
      .require_v4_constant_address_memory_when_available = true;
  cases.push_back(v4_constant_address_memory);

  CpuCompareCase jit_v2_bne_not_taken{};
  jit_v2_bne_not_taken.name = "jit_v2_bne_not_taken_delay";
  jit_v2_bne_not_taken.initial_gpr[1] = 7u;
  jit_v2_bne_not_taken.initial_gpr[2] = 7u;
  jit_v2_bne_not_taken.program = {
      0u,
      enc_i(0x05, 1, 2, 2),
      enc_i(0x09, 3, 3, 1),
      0u,
  };
  jit_v2_bne_not_taken.instructions = 3;
  jit_v2_bne_not_taken.require_v2_branch_not_taken_entry_when_available =
      true;
  cases.push_back(jit_v2_bne_not_taken);

  CpuCompareCase jit_v2_scratch_store_branch{};
  jit_v2_scratch_store_branch.name =
      "native_scratchpad_sw_sw_bne_delay_loop";
  jit_v2_scratch_store_branch.initial_gpr[1] = 0x1F800000u;
  jit_v2_scratch_store_branch.initial_gpr[2] = 0x11223344u;
  jit_v2_scratch_store_branch.initial_gpr[3] = 0x55667788u;
  jit_v2_scratch_store_branch.initial_gpr[4] = 1u;
  jit_v2_scratch_store_branch.initial_gpr[5] = 0u;
  jit_v2_scratch_store_branch.program = {
      enc_i(0x2B, 1, 2, 0),
      enc_i(0x2B, 1, 3, 4),
      enc_i(0x05, 4, 5, static_cast<u16>(-3)),
      enc_i(0x09, 6, 6, 1),
  };
  jit_v2_scratch_store_branch.instructions = 8;
  jit_v2_scratch_store_branch.compare_memory_addresses = {
      0x1F800000u, 0x1F800004u,
  };
  jit_v2_scratch_store_branch.require_v2_store_branch_entry_when_available =
      true;
  jit_v2_scratch_store_branch.require_v4_native_entry_when_available = true;
  jit_v2_scratch_store_branch.require_v4_native_store_entry_when_available =
      true;
  jit_v2_scratch_store_branch.require_v4_native_branch_entry_when_available =
      true;
  cases.push_back(jit_v2_scratch_store_branch);

  CpuCompareCase jit_v2_ram_store_branch = jit_v2_scratch_store_branch;
  jit_v2_ram_store_branch.name = "jit_v2_ram_sw_sw_bne_delay_loop";
  jit_v2_ram_store_branch.initial_gpr[1] = 0x80011000u;
  jit_v2_ram_store_branch.compare_memory_addresses = {
      0x00011000u, 0x00011004u,
  };
  cases.push_back(jit_v2_ram_store_branch);

  CpuCompareCase native_mixed{};
  native_mixed.name = "native_mixed_alu_immediate";
  native_mixed.program = {
      0,
      enc_i(0x09, 0, 1, 0x0001),
      enc_i(0x09, 1, 1, 0x0001),
      enc_i(0x0F, 0, 2, 0x1234),
      enc_i(0x0D, 2, 2, 0x5678),
      enc_i(0x0C, 2, 3, 0x00FF),
      enc_i(0x0E, 3, 4, 0x00AA),
      enc_r(1, 1, 5, 0, 0x21),
      enc_r(5, 1, 6, 0, 0x23),
      enc_r(2, 4, 7, 0, 0x24),
      enc_r(2, 4, 8, 0, 0x25),
      enc_r(2, 4, 9, 0, 0x26),
      enc_r(2, 4, 10, 0, 0x27),
      enc_i(0x0A, 10, 11, 0x0000),
      enc_i(0x0B, 10, 12, 0xFFFF),
      enc_r(12, 11, 13, 0, 0x21),
  };
  native_mixed.require_full_native_when_available = true;
  native_mixed.instructions = 16;
  cases.push_back(native_mixed);

  CpuCompareCase r0_writes{};
  r0_writes.name = "native_r0_writes_ignored";
  r0_writes.program = {
      enc_i(0x09, 0, 0, 0x1234),
      enc_i(0x0F, 0, 0, 0xFFFF),
      enc_i(0x0D, 0, 0, 0xFFFF),
      enc_r(0, 0, 0, 4, 0x00),
      enc_i(0x09, 0, 1, 0x0007),
  };
  r0_writes.require_full_native_when_available = true;
  pad_cpu_compare_program(r0_writes);
  cases.push_back(r0_writes);

  CpuCompareCase addiu_overlap{};
  addiu_overlap.name = "native_addiu_positive_negative_overlap";
  addiu_overlap.initial_gpr[1] = 0x7FFFFFFFu;
  addiu_overlap.initial_gpr[2] = 0x00000010u;
  addiu_overlap.program = {
      enc_i(0x09, 0, 3, 0x7FFF),
      enc_i(0x09, 3, 4, 0x8000),
      enc_i(0x09, 1, 1, 0x0001),
      enc_i(0x0D, 2, 2, 0x0001),
      enc_r(1, 1, 1, 0, 0x21),
  };
  addiu_overlap.require_full_native_when_available = true;
  pad_cpu_compare_program(addiu_overlap);
  cases.push_back(addiu_overlap);

  CpuCompareCase wrapping_logic{};
  wrapping_logic.name = "native_wrapping_logic_lui_zero_extend";
  wrapping_logic.program = {
      enc_i(0x0F, 0, 1, 0xFFFF),
      enc_i(0x0D, 1, 1, 0xFFFF),
      enc_i(0x09, 0, 2, 0x0001),
      enc_r(1, 2, 3, 0, 0x21),
      enc_r(2, 1, 4, 0, 0x23),
      enc_i(0x0F, 0, 5, 0x00FF),
      enc_i(0x0C, 5, 6, 0xF0F0),
      enc_i(0x0D, 6, 7, 0x0F0F),
      enc_i(0x0E, 7, 8, 0xFFFF),
      enc_r(7, 8, 9, 0, 0x24),
      enc_r(7, 8, 10, 0, 0x25),
      enc_r(7, 8, 11, 0, 0x26),
      enc_r(7, 8, 12, 0, 0x27),
  };
  wrapping_logic.require_full_native_when_available = true;
  pad_cpu_compare_program(wrapping_logic);
  cases.push_back(wrapping_logic);

  CpuCompareCase comparisons{};
  comparisons.name = "native_signed_unsigned_comparisons";
  comparisons.program = {
      enc_i(0x09, 0, 1, 0xFFFF),
      enc_i(0x09, 0, 2, 0x0001),
      enc_i(0x0F, 0, 3, 0x8000),
      enc_r(1, 2, 4, 0, 0x2A),
      enc_r(1, 2, 5, 0, 0x2B),
      enc_r(3, 2, 6, 0, 0x2A),
      enc_r(3, 2, 7, 0, 0x2B),
      enc_i(0x0A, 2, 8, 0xFFFF),
      enc_i(0x0A, 1, 9, 0x0001),
      enc_i(0x0B, 2, 10, 0xFFFF),
      enc_i(0x0B, 1, 11, 0x0001),
  };
  comparisons.require_full_native_when_available = true;
  pad_cpu_compare_program(comparisons);
  cases.push_back(comparisons);

  CpuCompareCase shifts{};
  shifts.name = "native_shift_immediate_and_variable";
  shifts.program = {
      enc_i(0x0F, 0, 1, 0x8000),
      enc_i(0x0D, 1, 1, 0x0001),
      enc_r(0, 1, 2, 0, 0x00),
      enc_r(0, 1, 3, 4, 0x00),
      enc_r(0, 1, 4, 0, 0x02),
      enc_r(0, 1, 5, 4, 0x02),
      enc_r(0, 1, 6, 0, 0x03),
      enc_r(0, 1, 7, 4, 0x03),
      enc_i(0x09, 0, 8, 0x0028),
      enc_r(8, 1, 9, 0, 0x04),
      enc_r(8, 1, 10, 0, 0x06),
      enc_r(8, 1, 11, 0, 0x07),
      enc_r(8, 1, 1, 0, 0x04),
  };
  shifts.require_full_native_when_available = true;
  pad_cpu_compare_program(shifts);
  cases.push_back(shifts);

  CpuCompareCase conditional_moves{};
  conditional_moves.name = "native_movz_movn_sync";
  conditional_moves.initial_gpr[1] = 0x11111111u;
  conditional_moves.initial_gpr[2] = 0u;
  conditional_moves.initial_gpr[3] = 0x22222222u;
  conditional_moves.initial_gpr[4] = 5u;
  conditional_moves.program = {
      enc_r(1, 2, 7, 0, 0x0A),
      enc_r(3, 4, 8, 0, 0x0A),
      enc_r(3, 4, 9, 0, 0x0B),
      enc_r(1, 2, 10, 0, 0x0B),
      enc_r(0, 0, 0, 0, 0x0F),
      enc_r(7, 9, 11, 0, 0x21),
  };
  conditional_moves.require_full_native_when_available = true;
  pad_cpu_compare_program(conditional_moves);
  cases.push_back(conditional_moves);

  CpuCompareCase compat_specials{};
  compat_specials.name = "native_compat_special_noops_and_aliases";
  compat_specials.initial_gpr[1] = 0xFFFFFFFFu;
  compat_specials.initial_gpr[2] = 2u;
  compat_specials.initial_gpr[5] = 0x55555555u;
  compat_specials.initial_gpr[6] = 0x66666666u;
  compat_specials.initial_gpr[7] = 0x77777777u;
  compat_specials.initial_gpr[8] = 0x88888888u;
  compat_specials.initial_gpr[10] = 0x99999999u;
  compat_specials.program = {
      enc_r(1, 2, 3, 0, 0x2D),
      enc_r(2, 1, 4, 0, 0x2F),
      enc_r(1, 2, 5, 0, 0x14),
      enc_r(1, 2, 6, 0, 0x1C),
      enc_r(1, 2, 7, 0, 0x28),
      enc_r(1, 2, 8, 0, 0x29),
      enc_r(0, 10, 10, 0, 0x38),
      enc_r(3, 4, 9, 0, 0x21),
  };
  compat_specials.require_full_native_when_available = true;
  pad_cpu_compare_program(compat_specials);
  cases.push_back(compat_specials);

  CpuCompareCase hilo_muldiv{};
  hilo_muldiv.name = "decoded_hilo_muldiv_timing";
  hilo_muldiv.initial_gpr[1] = 0xFFFFFFF9u;
  hilo_muldiv.initial_gpr[2] = 3u;
  hilo_muldiv.program = {
      enc_r(1, 2, 0, 0, 0x18), // MULT
      enc_r(0, 0, 3, 0, 0x12), // MFLO
      enc_r(0, 0, 4, 0, 0x10), // MFHI
      enc_r(1, 2, 0, 0, 0x1A), // DIV
      enc_r(0, 0, 5, 0, 0x12), // MFLO
      enc_r(0, 0, 6, 0, 0x10), // MFHI
      enc_r(2, 1, 0, 0, 0x19), // MULTU
      enc_r(0, 0, 7, 0, 0x12), // MFLO
      enc_r(2, 1, 0, 0, 0x1B), // DIVU
      enc_r(0, 0, 8, 0, 0x12), // MFLO
      enc_r(0, 0, 9, 0, 0x10), // MFHI
      enc_r(3, 0, 0, 0, 0x11), // MTHI
      enc_r(4, 0, 0, 0, 0x13), // MTLO
  };
  pad_cpu_compare_program(hilo_muldiv);
  cases.push_back(hilo_muldiv);

  CpuCompareCase overflow_add{};
  overflow_add.name = "decoded_add_overflow";
  overflow_add.initial_gpr[1] = 0x7FFFFFFFu;
  overflow_add.initial_gpr[2] = 1u;
  overflow_add.program = {enc_r(1, 2, 3, 0, 0x20)};
  overflow_add.instructions = 1;
  overflow_add.require_v4_native_entry_when_available = true;
  overflow_add.require_v4_exception_native_when_available = true;
  cases.push_back(overflow_add);

  CpuCompareCase overflow_sub{};
  overflow_sub.name = "decoded_sub_overflow";
  overflow_sub.initial_gpr[1] = 0x80000000u;
  overflow_sub.initial_gpr[2] = 1u;
  overflow_sub.program = {enc_r(1, 2, 3, 0, 0x22)};
  overflow_sub.instructions = 1;
  overflow_sub.require_v4_native_entry_when_available = true;
  overflow_sub.require_v4_exception_native_when_available = true;
  cases.push_back(overflow_sub);

  CpuCompareCase overflow_addi{};
  overflow_addi.name = "decoded_addi_overflow";
  overflow_addi.initial_gpr[1] = 0x7FFFFFFFu;
  overflow_addi.program = {enc_i(0x08, 1, 2, 1)};
  overflow_addi.instructions = 1;
  overflow_addi.require_v4_native_entry_when_available = true;
  overflow_addi.require_v4_exception_native_when_available = true;
  cases.push_back(overflow_addi);

  CpuCompareCase v4_misaligned_lw_exception{};
  v4_misaligned_lw_exception.name = "v4_misaligned_lw_exception_native";
  v4_misaligned_lw_exception.start_pc = 0xA0010000u;
  v4_misaligned_lw_exception.initial_gpr[1] = 0x80012001u;
  v4_misaligned_lw_exception.program = {enc_i(0x23, 1, 2, 0)};
  v4_misaligned_lw_exception.instructions = 1u;
  v4_misaligned_lw_exception.require_v4_native_entry_when_available = true;
  v4_misaligned_lw_exception.require_v4_exception_native_when_available = true;
  cases.push_back(v4_misaligned_lw_exception);

  CpuCompareCase v4_misaligned_sw_exception{};
  v4_misaligned_sw_exception.name = "v4_misaligned_sw_exception_native";
  v4_misaligned_sw_exception.start_pc = 0xA0010000u;
  v4_misaligned_sw_exception.initial_gpr[1] = 0x80012001u;
  v4_misaligned_sw_exception.initial_gpr[2] = 0x12345678u;
  v4_misaligned_sw_exception.memory.push_back({0x00012000u, 0xAABBCCDDu});
  v4_misaligned_sw_exception.compare_memory_addresses.push_back(0x00012000u);
  v4_misaligned_sw_exception.program = {enc_i(0x2B, 1, 2, 0)};
  v4_misaligned_sw_exception.instructions = 1u;
  v4_misaligned_sw_exception.require_v4_native_entry_when_available = true;
  v4_misaligned_sw_exception.require_v4_exception_native_when_available = true;
  cases.push_back(v4_misaligned_sw_exception);

  CpuCompareCase unaligned_merge{};
  unaligned_merge.name = "decoded_unaligned_load_store_merge";
  unaligned_merge.initial_gpr[1] = 0x80011201u;
  unaligned_merge.initial_gpr[2] = 0xAABBCCDDu;
  unaligned_merge.memory = {
      {0x00011200u, 0x44332211u},
      {0x00011204u, 0x88776655u},
  };
  unaligned_merge.compare_memory_addresses = {0x00011200u, 0x00011204u};
  unaligned_merge.program = {
      enc_i(0x22, 1, 2, 2), // LWL
      enc_i(0x26, 1, 2, 0), // LWR
      0,
      enc_i(0x2A, 1, 2, 2), // SWL
      enc_i(0x2E, 1, 2, 0), // SWR
  };
  pad_cpu_compare_program(unaligned_merge);
  unaligned_merge.require_v4_native_entry_when_available = true;
  unaligned_merge.require_v4_native_load_entry_when_available = true;
  unaligned_merge.require_v4_native_store_entry_when_available = true;
  unaligned_merge.require_v4_unaligned_native_when_available = true;
  cases.push_back(unaligned_merge);

  CpuCompareCase v4_cop2_native{};
  v4_cop2_native.name = "v4_cop2_transfers_command_native";
  v4_cop2_native.start_pc = 0xA0010000u;
  v4_cop2_native.initial_gpr[1] = 0x44332211u;
  v4_cop2_native.initial_gpr[3] = 0x12345678u;
  v4_cop2_native.initial_gpr[8] = 0x00020001u;
  v4_cop2_native.initial_gpr[9] = 0x00040003u;
  v4_cop2_native.initial_gpr[10] = 0x00060005u;
  v4_cop2_native.program = {
      (0x12u << 26) | (4u << 21) | (1u << 16) | (6u << 11),
      (0x12u << 26) | (0u << 21) | (2u << 16) | (6u << 11),
      0u,
      (0x12u << 26) | (6u << 21) | (3u << 16) | (5u << 11),
      (0x12u << 26) | (2u << 21) | (4u << 16) | (5u << 11),
      0u,
      (0x12u << 26) | (4u << 21) | (8u << 16) | (12u << 11),
      (0x12u << 26) | (4u << 21) | (9u << 16) | (13u << 11),
      (0x12u << 26) | (4u << 21) | (10u << 16) | (14u << 11),
      (0x12u << 26) | (0x10u << 21) | 0x06u,
      (0x12u << 26) | (0u << 21) | (5u << 16) | (24u << 11),
      0u,
  };
  v4_cop2_native.instructions = 12u;
  v4_cop2_native.require_v4_native_entry_when_available = true;
  v4_cop2_native.require_v4_cop2_native_when_available = true;
  cases.push_back(v4_cop2_native);

  CpuCompareCase cop_transfers{};
  cop_transfers.name = "decoded_cop0_cop2_lwc2_swc2";
  cop_transfers.initial_gpr[1] = 0x80011300u;
  cop_transfers.initial_gpr[2] = 0x00000401u;
  cop_transfers.initial_gpr[4] = 0x12345678u;
  cop_transfers.memory = {{0x00011300u, 0x89ABCDEFu}};
  cop_transfers.compare_memory_addresses = {0x00011300u, 0x00011304u};
  cop_transfers.program = {
      (0x10u << 26) | (4u << 21) | (2u << 16) | (12u << 11), // MTC0 SR
      (0x10u << 26) | (0u << 21) | (3u << 16) | (12u << 11), // MFC0 SR
      (0x12u << 26) | (4u << 21) | (4u << 16) | (6u << 11),  // MTC2 RGB
      enc_i(0x3A, 1, 6, 4),                                  // SWC2 RGB
      enc_i(0x32, 1, 7, 0),                                  // LWC2 OTZ
      (0x12u << 26) | (0u << 21) | (5u << 16) | (7u << 11),  // MFC2 OTZ
      0,
  };
  pad_cpu_compare_program(cop_transfers);
  cop_transfers.require_v4_native_entry_when_available = true;
  cop_transfers.require_v4_native_load_entry_when_available = true;
  cop_transfers.require_v4_native_store_entry_when_available = true;
  cop_transfers.require_v4_cop2_native_when_available = true;
  cases.push_back(cop_transfers);

  CpuCompareCase cop0_memory_transfers{};
  cop0_memory_transfers.name = "v4_lwc0_swc0_native";
  cop0_memory_transfers.initial_gpr[1] = 0x80011340u;
  cop0_memory_transfers.initial_gpr[2] = 0x80011344u;
  cop0_memory_transfers.memory = {
      {0x00011340u, 0x00000300u},
      {0x00011344u, 0u},
  };
  cop0_memory_transfers.compare_memory_addresses = {
      0x00011340u, 0x00011344u,
  };
  cop0_memory_transfers.program = {
      enc_i(0x30, 1, 13, 0), // LWC0 Cause <- [r1]
      enc_i(0x38, 2, 13, 0), // SWC0 [r2] <- Cause
      0u,
  };
  cop0_memory_transfers.instructions = 3u;
  cop0_memory_transfers.require_v4_native_entry_when_available = true;
  cop0_memory_transfers.require_v4_native_load_entry_when_available = true;
  cop0_memory_transfers.require_v4_native_store_entry_when_available = true;
  cop0_memory_transfers.require_v4_cop0_native_when_available = true;
  cases.push_back(cop0_memory_transfers);

  CpuCompareCase branch_likely_not_taken{};
  branch_likely_not_taken.name = "decoded_beql_not_taken_annuls_delay";
  branch_likely_not_taken.initial_gpr[1] = 1u;
  branch_likely_not_taken.initial_gpr[2] = 2u;
  branch_likely_not_taken.program = {
      enc_i(0x14, 1, 2, 1), enc_i(0x09, 0, 3, 0x1111),
      enc_i(0x09, 0, 4, 0x2222),
  };
  branch_likely_not_taken.instructions = 2;
  cases.push_back(branch_likely_not_taken);

  CpuCompareCase branch_likely_taken{};
  branch_likely_taken.name = "decoded_beql_taken_delay";
  branch_likely_taken.initial_gpr[1] = 1u;
  branch_likely_taken.initial_gpr[2] = 1u;
  branch_likely_taken.program = {
      enc_i(0x14, 1, 2, 1), enc_i(0x09, 0, 3, 0x1111),
      enc_i(0x09, 0, 4, 0x2222),
  };
  branch_likely_taken.instructions = 3;
  cases.push_back(branch_likely_taken);

  CpuCompareCase decoded_load_then_movz_cancel{};
  decoded_load_then_movz_cancel.name =
      "decoded_load_then_native_movz_cancel";
  decoded_load_then_movz_cancel.initial_gpr[1] = 0x800114A0u;
  decoded_load_then_movz_cancel.initial_gpr[2] = 0x11111111u;
  decoded_load_then_movz_cancel.initial_gpr[3] = 7u;
  decoded_load_then_movz_cancel.initial_gpr[4] = 0u;
  decoded_load_then_movz_cancel.memory.push_back(
      {0x000114A0u, 0xDEADBEEFu});
  decoded_load_then_movz_cancel.program = {
      enc_i(0x23, 1, 2, 0),
      enc_r(3, 4, 2, 0, 0x0A),
      enc_r(2, 0, 5, 0, 0x21),
  };
  pad_cpu_compare_program(decoded_load_then_movz_cancel, 17u);
  decoded_load_then_movz_cancel.segment_instructions = {1u, 16u};
  decoded_load_then_movz_cancel.segment_native_tiers = {
      {false, true, true}, {true, true, true}};
  decoded_load_then_movz_cancel.compare_segment_states = true;
  decoded_load_then_movz_cancel
      .require_native_alu_tier_entry_when_available = true;
  cases.push_back(decoded_load_then_movz_cancel);

  CpuCompareCase decoded_load_then_movn_no_cancel{};
  decoded_load_then_movn_no_cancel.name =
      "decoded_load_then_native_movn_no_cancel";
  decoded_load_then_movn_no_cancel.initial_gpr[1] = 0x800114B0u;
  decoded_load_then_movn_no_cancel.initial_gpr[2] = 0x11111111u;
  decoded_load_then_movn_no_cancel.initial_gpr[3] = 7u;
  decoded_load_then_movn_no_cancel.initial_gpr[4] = 0u;
  decoded_load_then_movn_no_cancel.memory.push_back(
      {0x000114B0u, 0xCAFEBABEu});
  decoded_load_then_movn_no_cancel.program = {
      enc_i(0x23, 1, 2, 0),
      enc_r(3, 4, 2, 0, 0x0B),
      enc_r(2, 0, 5, 0, 0x21),
  };
  pad_cpu_compare_program(decoded_load_then_movn_no_cancel, 17u);
  decoded_load_then_movn_no_cancel.segment_instructions = {1u, 16u};
  decoded_load_then_movn_no_cancel.segment_native_tiers = {
      {false, true, true}, {true, true, true}};
  decoded_load_then_movn_no_cancel.compare_segment_states = true;
  decoded_load_then_movn_no_cancel
      .require_native_alu_tier_entry_when_available = true;
  cases.push_back(decoded_load_then_movn_no_cancel);

  CpuCompareCase decoded_load_then_clear_cancel{};
  decoded_load_then_clear_cancel.name =
      "decoded_load_then_native_clear_cancel";
  decoded_load_then_clear_cancel.initial_gpr[1] = 0x800114C0u;
  decoded_load_then_clear_cancel.initial_gpr[2] = 0x11111111u;
  decoded_load_then_clear_cancel.memory.push_back(
      {0x000114C0u, 0x1234ABCDu});
  decoded_load_then_clear_cancel.program = {
      enc_i(0x23, 1, 2, 0),
      enc_r(0, 2, 2, 0, 0x38),
      enc_r(2, 0, 5, 0, 0x21),
  };
  pad_cpu_compare_program(decoded_load_then_clear_cancel, 17u);
  decoded_load_then_clear_cancel.segment_instructions = {1u, 16u};
  decoded_load_then_clear_cancel.segment_native_tiers = {
      {false, true, true}, {true, true, true}};
  decoded_load_then_clear_cancel.compare_segment_states = true;
  decoded_load_then_clear_cancel
      .require_native_alu_tier_entry_when_available = true;
  cases.push_back(decoded_load_then_clear_cancel);

  CpuCompareCase decoded_load_then_reduced_alu_cancel{};
  decoded_load_then_reduced_alu_cancel.name =
      "decoded_load_then_reduced_alu_cancel";
  decoded_load_then_reduced_alu_cancel.initial_gpr[1] = 0x80011480u;
  decoded_load_then_reduced_alu_cancel.initial_gpr[2] = 0x11111111u;
  decoded_load_then_reduced_alu_cancel.memory.push_back(
      {0x00011480u, 0xDEADBEEFu});
  decoded_load_then_reduced_alu_cancel.program = {
      enc_i(0x23, 1, 2, 0), enc_i(0x09, 0, 2, 7),
      enc_r(2, 0, 3, 0, 0x21),
  };
  pad_cpu_compare_program(decoded_load_then_reduced_alu_cancel, 17u);
  decoded_load_then_reduced_alu_cancel.segment_instructions = {1u, 16u};
  decoded_load_then_reduced_alu_cancel.segment_native_tiers = {
      {false, true, true}, {true, true, true}};
  decoded_load_then_reduced_alu_cancel.compare_segment_states = true;
  decoded_load_then_reduced_alu_cancel
      .require_native_alu_tier_entry_when_available = true;
  cases.push_back(decoded_load_then_reduced_alu_cancel);

  CpuCompareCase decoded_load_then_reduced_alu_retire{};
  decoded_load_then_reduced_alu_retire.name =
      "decoded_load_then_reduced_alu_retire";
  decoded_load_then_reduced_alu_retire.initial_gpr[1] = 0x80011490u;
  decoded_load_then_reduced_alu_retire.initial_gpr[2] = 0x11111111u;
  decoded_load_then_reduced_alu_retire.memory.push_back(
      {0x00011490u, 0xCAFEBABEu});
  decoded_load_then_reduced_alu_retire.program = {
      enc_i(0x23, 1, 2, 0), enc_i(0x09, 0, 3, 7),
      enc_r(2, 0, 4, 0, 0x21),
  };
  pad_cpu_compare_program(decoded_load_then_reduced_alu_retire, 17u);
  decoded_load_then_reduced_alu_retire.segment_instructions = {1u, 16u};
  decoded_load_then_reduced_alu_retire.segment_native_tiers = {
      {false, true, true}, {true, true, true}};
  decoded_load_then_reduced_alu_retire.compare_segment_states = true;
  decoded_load_then_reduced_alu_retire
      .require_native_alu_tier_entry_when_available = true;
  cases.push_back(decoded_load_then_reduced_alu_retire);

  CpuCompareCase reduced_alu_then_decoded_boundary{};
  reduced_alu_then_decoded_boundary.name =
      "reduced_alu_then_decoded_boundary";
  reduced_alu_then_decoded_boundary.program.assign(17u, 0u);
  reduced_alu_then_decoded_boundary.program[0] =
      enc_i(0x09, 0, 2, 0x1234);
  reduced_alu_then_decoded_boundary.program[15] =
      enc_i(0x0E, 2, 2, 0x00FF);
  reduced_alu_then_decoded_boundary.program[16] =
      enc_r(2, 0, 3, 0, 0x21);
  reduced_alu_then_decoded_boundary.instructions = 17u;
  reduced_alu_then_decoded_boundary.segment_instructions = {16u, 1u};
  reduced_alu_then_decoded_boundary.segment_native_tiers = {
      {true, true, true}, {false, true, true}};
  reduced_alu_then_decoded_boundary.compare_segment_states = true;
  reduced_alu_then_decoded_boundary.expect_final_control_state = true;
  reduced_alu_then_decoded_boundary.expected_pc = kCpuComparePc + 68u;
  reduced_alu_then_decoded_boundary.expected_next_pc = kCpuComparePc + 72u;
  reduced_alu_then_decoded_boundary.expected_current_pc =
      kCpuComparePc + 64u;
  reduced_alu_then_decoded_boundary.expected_cycles = 37u;
  reduced_alu_then_decoded_boundary
      .require_native_alu_tier_entry_when_available = true;
  cases.push_back(reduced_alu_then_decoded_boundary);

  CpuCompareCase bne_taken{};
  bne_taken.name = "native_branch_tail_bne_taken_alu_delay";
  bne_taken.initial_gpr[1] = 1u;
  bne_taken.initial_gpr[2] = 2u;
  bne_taken.program = {
      enc_i(0x05, 1, 2, 2),
      enc_i(0x09, 0, 3, 0x0011),
      enc_i(0x09, 0, 4, 0x0022),
      enc_i(0x09, 0, 5, 0x0033),
  };
  bne_taken.instructions = 2;
  bne_taken.require_full_native_when_available = true;
  bne_taken.require_native_branch_tail_when_available = true;
  bne_taken.native_branch_should_be_taken = true;
  cases.push_back(bne_taken);

  CpuCompareCase bne_not_taken{};
  bne_not_taken.name = "native_branch_tail_bne_not_taken_alu_delay";
  bne_not_taken.initial_gpr[1] = 1u;
  bne_not_taken.initial_gpr[2] = 1u;
  bne_not_taken.program = bne_taken.program;
  bne_not_taken.instructions = 2;
  bne_not_taken.require_full_native_when_available = true;
  bne_not_taken.require_native_branch_tail_when_available = true;
  cases.push_back(bne_not_taken);

  CpuCompareCase reduced_bne_taken{};
  reduced_bne_taken.name =
      "native_reduced_helper_branch_tail_bne_taken_alu_body_delay";
  reduced_bne_taken.initial_gpr[1] = 1u;
  reduced_bne_taken.initial_gpr[2] = 2u;
  reduced_bne_taken.program = {
      enc_i(0x09, 0, 6, 0x0001),
      enc_i(0x05, 1, 2, 2),
      enc_i(0x09, 0, 3, 0x0011),
      enc_i(0x09, 0, 4, 0x0022),
      enc_i(0x09, 0, 5, 0x0033),
  };
  reduced_bne_taken.instructions = 3;
  reduced_bne_taken.enable_reduced_helper_branch_tail_for_x64 = true;
  reduced_bne_taken.require_full_native_when_available = true;
  reduced_bne_taken.require_native_branch_tail_when_available = true;
  reduced_bne_taken.require_no_native_instruction_helpers_when_available =
      true;
  reduced_bne_taken.require_no_native_branch_tail_helpers_when_available =
      true;
  reduced_bne_taken.native_branch_should_be_taken = true;
  cases.push_back(reduced_bne_taken);

  CpuCompareCase reduced_bne_lw_not_taken{};
  reduced_bne_lw_not_taken.name =
      "native_reduced_helper_branch_tail_lw_not_taken";
  reduced_bne_lw_not_taken.initial_gpr[1] = 0x80011600u;
  reduced_bne_lw_not_taken.initial_gpr[2] = 0x11111111u;
  reduced_bne_lw_not_taken.initial_gpr[3] = 0x11111111u;
  reduced_bne_lw_not_taken.memory.push_back(
      {0x00011600u, 0x22222222u});
  reduced_bne_lw_not_taken.program = {
      enc_i(0x23, 1, 2, 0),
      enc_i(0x05, 2, 3, 2),
      enc_r(2, 0, 4, 0, 0x21),
      enc_i(0x09, 0, 5, 0x0011),
      enc_i(0x09, 0, 6, 0x0022),
  };
  reduced_bne_lw_not_taken.instructions = 3;
  reduced_bne_lw_not_taken.enable_ram_load_fastpath_for_x64 = true;
  reduced_bne_lw_not_taken.enable_reduced_helper_branch_tail_for_x64 = true;
  reduced_bne_lw_not_taken.require_full_native_when_available = true;
  reduced_bne_lw_not_taken.require_native_branch_tail_when_available = true;
  reduced_bne_lw_not_taken
      .require_native_reduced_helper_branch_tail_entry_when_available = true;
  reduced_bne_lw_not_taken
      .require_native_reduced_helper_branch_tail_ram_load_entry_when_available =
      true;
  reduced_bne_lw_not_taken.require_native_ram_load_fastpath_when_available =
      true;
  reduced_bne_lw_not_taken.require_no_native_instruction_helpers_when_available =
      true;
  reduced_bne_lw_not_taken.require_no_native_branch_tail_helpers_when_available =
      true;
  cases.push_back(reduced_bne_lw_not_taken);

  CpuCompareCase reduced_bne_lw_taken_after_alu{};
  reduced_bne_lw_taken_after_alu.name =
      "native_reduced_helper_branch_tail_lw_taken_after_alu";
  reduced_bne_lw_taken_after_alu.initial_gpr[1] = 0x80011610u;
  reduced_bne_lw_taken_after_alu.initial_gpr[2] = 0x11111111u;
  reduced_bne_lw_taken_after_alu.initial_gpr[3] = 0x11111111u;
  reduced_bne_lw_taken_after_alu.memory.push_back(
      {0x00011610u, 0x22222222u});
  reduced_bne_lw_taken_after_alu.program = {
      enc_i(0x23, 1, 2, 0),
      enc_r(2, 0, 4, 0, 0x21),
      enc_i(0x05, 2, 3, 1),
      enc_r(2, 0, 5, 0, 0x21),
      enc_i(0x09, 0, 6, 0x0022),
  };
  reduced_bne_lw_taken_after_alu.instructions = 4;
  reduced_bne_lw_taken_after_alu.enable_ram_load_fastpath_for_x64 = true;
  reduced_bne_lw_taken_after_alu.enable_reduced_helper_branch_tail_for_x64 =
      true;
  reduced_bne_lw_taken_after_alu.require_full_native_when_available = true;
  reduced_bne_lw_taken_after_alu.require_native_branch_tail_when_available =
      true;
  reduced_bne_lw_taken_after_alu
      .require_native_reduced_helper_branch_tail_entry_when_available = true;
  reduced_bne_lw_taken_after_alu
      .require_native_reduced_helper_branch_tail_ram_load_entry_when_available =
      true;
  reduced_bne_lw_taken_after_alu
      .require_native_ram_load_fastpath_when_available = true;
  reduced_bne_lw_taken_after_alu
      .require_no_native_instruction_helpers_when_available = true;
  reduced_bne_lw_taken_after_alu
      .require_no_native_branch_tail_helpers_when_available = true;
  reduced_bne_lw_taken_after_alu.native_branch_should_be_taken = true;
  cases.push_back(reduced_bne_lw_taken_after_alu);

  CpuCompareCase reduced_bne_lw_r0{};
  reduced_bne_lw_r0.name = "native_reduced_helper_branch_tail_lw_r0";
  reduced_bne_lw_r0.initial_gpr[1] = 0x80011620u;
  reduced_bne_lw_r0.initial_gpr[2] = 7u;
  reduced_bne_lw_r0.initial_gpr[3] = 7u;
  reduced_bne_lw_r0.memory.push_back({0x00011620u, 0xFFFFFFFFu});
  reduced_bne_lw_r0.program = {
      enc_i(0x23, 1, 0, 0),
      enc_i(0x05, 2, 3, 1),
      enc_r(0, 0, 4, 0, 0x21),
      enc_i(0x09, 0, 5, 0x0011),
  };
  reduced_bne_lw_r0.instructions = 3;
  reduced_bne_lw_r0.enable_ram_load_fastpath_for_x64 = true;
  reduced_bne_lw_r0.enable_reduced_helper_branch_tail_for_x64 = true;
  reduced_bne_lw_r0.require_full_native_when_available = true;
  reduced_bne_lw_r0.require_native_branch_tail_when_available = true;
  reduced_bne_lw_r0
      .require_native_reduced_helper_branch_tail_entry_when_available = true;
  reduced_bne_lw_r0
      .require_native_reduced_helper_branch_tail_ram_load_entry_when_available =
      true;
  reduced_bne_lw_r0.require_native_ram_load_fastpath_when_available = true;
  reduced_bne_lw_r0.require_no_native_instruction_helpers_when_available =
      true;
  reduced_bne_lw_r0.require_no_native_branch_tail_helpers_when_available =
      true;
  cases.push_back(reduced_bne_lw_r0);

  CpuCompareCase reduced_bne_lw_base_write_reject{};
  reduced_bne_lw_base_write_reject.name =
      "native_reduced_helper_branch_tail_lw_base_write_rejected";
  reduced_bne_lw_base_write_reject.initial_gpr[1] = 0x80011630u;
  reduced_bne_lw_base_write_reject.initial_gpr[2] = 0x11111111u;
  reduced_bne_lw_base_write_reject.initial_gpr[3] = 0x11111111u;
  reduced_bne_lw_base_write_reject.memory.push_back(
      {0x00011634u, 0x22222222u});
  reduced_bne_lw_base_write_reject.program = {
      enc_i(0x09, 1, 1, 4),
      enc_i(0x23, 1, 2, 0),
      enc_i(0x05, 2, 3, 1),
      enc_r(2, 0, 4, 0, 0x21),
      enc_i(0x09, 0, 5, 0x0022),
  };
  reduced_bne_lw_base_write_reject.instructions = 4;
  reduced_bne_lw_base_write_reject.enable_ram_load_fastpath_for_x64 = true;
  reduced_bne_lw_base_write_reject
      .enable_reduced_helper_branch_tail_for_x64 = true;
  reduced_bne_lw_base_write_reject.require_full_native_when_available = true;
  reduced_bne_lw_base_write_reject.require_native_branch_tail_when_available =
      true;
  reduced_bne_lw_base_write_reject
      .require_no_native_reduced_helper_branch_tail_ram_load_entry = true;
  reduced_bne_lw_base_write_reject
      .require_reduced_helper_branch_tail_reject_load_base_written_when_available =
      true;
  reduced_bne_lw_base_write_reject
      .require_native_ram_load_fastpath_when_available = true;
  cases.push_back(reduced_bne_lw_base_write_reject);

  CpuCompareCase reduced_bne_lw_scratchpad_reject{};
  reduced_bne_lw_scratchpad_reject.name =
      "native_reduced_helper_branch_tail_lw_scratchpad_rejected";
  reduced_bne_lw_scratchpad_reject.initial_gpr[1] = 0x1F800000u;
  reduced_bne_lw_scratchpad_reject.initial_gpr[2] = 1u;
  reduced_bne_lw_scratchpad_reject.initial_gpr[3] = 1u;
  reduced_bne_lw_scratchpad_reject.memory.push_back(
      {0x1F800000u, 0x55667788u});
  reduced_bne_lw_scratchpad_reject.program = {
      enc_i(0x23, 1, 4, 0),
      enc_i(0x05, 2, 3, 1),
      enc_i(0x09, 0, 5, 0x0011),
      enc_i(0x09, 0, 6, 0x0022),
  };
  reduced_bne_lw_scratchpad_reject.instructions = 3;
  reduced_bne_lw_scratchpad_reject.enable_ram_load_fastpath_for_x64 = true;
  reduced_bne_lw_scratchpad_reject.enable_reduced_helper_branch_tail_for_x64 =
      true;
  reduced_bne_lw_scratchpad_reject
      .require_native_entry_when_available = true;
  reduced_bne_lw_scratchpad_reject
      .require_native_memory_helper_when_available = true;
  reduced_bne_lw_scratchpad_reject
      .require_no_native_reduced_helper_branch_tail_ram_load_entry = true;
  reduced_bne_lw_scratchpad_reject
      .require_reduced_helper_branch_tail_preflight_non_ram_when_available =
      true;
  reduced_bne_lw_scratchpad_reject.require_no_native_ram_load_fastpath = true;
  cases.push_back(reduced_bne_lw_scratchpad_reject);

  CpuCompareCase aggressive_bne_taken{};
  aggressive_bne_taken.name =
      "native_aggressive_reduced_helper_branch_tail_bne_taken_alu";
  aggressive_bne_taken.initial_gpr[1] = 1u;
  aggressive_bne_taken.initial_gpr[2] = 2u;
  aggressive_bne_taken.program = {
      enc_i(0x09, 0, 6, 0x0001),
      enc_i(0x05, 1, 2, 2),
      enc_i(0x09, 0, 3, 0x0011),
      enc_i(0x09, 0, 4, 0x0022),
      enc_i(0x09, 0, 5, 0x0033),
  };
  aggressive_bne_taken.instructions = 3;
  aggressive_bne_taken.enable_aggressive_reduced_helper_branch_tail_for_x64 =
      true;
  aggressive_bne_taken.require_full_native_when_available = true;
  aggressive_bne_taken.require_native_branch_tail_when_available = true;
  aggressive_bne_taken
      .require_native_aggressive_reduced_helper_branch_tail_entry_when_available =
      true;
  aggressive_bne_taken.require_no_native_instruction_helpers_when_available =
      true;
  aggressive_bne_taken.require_no_native_branch_tail_helpers_when_available =
      true;
  aggressive_bne_taken.native_branch_should_be_taken = true;
  cases.push_back(aggressive_bne_taken);

  CpuCompareCase guarded_signed_branch{};
  guarded_signed_branch.name =
      "native_aggressive_branch_tail_guarded_signed_arithmetic";
  guarded_signed_branch.initial_gpr[1] = 100u;
  guarded_signed_branch.initial_gpr[2] = 23u;
  guarded_signed_branch.program = {
      enc_r(1, 2, 3, 0, 0x20), // ADD r3,r1,r2 = 123
      enc_i(0x08, 3, 4, 0xFFFD), // ADDI r4,r3,-3 = 120
      enc_i(0x05, 4, 0, 1),      // BNE r4,r0,+1
      enc_r(4, 2, 5, 0, 0x22),   // SUB r5,r4,r2 = 97 (delay)
      enc_i(0x09, 0, 6, 0x0033),
  };
  guarded_signed_branch.instructions = 4;
  guarded_signed_branch
      .enable_aggressive_reduced_helper_branch_tail_for_x64 = true;
  guarded_signed_branch.require_full_native_when_available = true;
  guarded_signed_branch.require_native_branch_tail_when_available = true;
  guarded_signed_branch
      .require_native_aggressive_reduced_helper_branch_tail_entry_when_available =
      true;
  guarded_signed_branch
      .require_aggressive_reduced_helper_branch_tail_full_preflight_when_available =
      true;
  guarded_signed_branch.require_no_native_instruction_helpers_when_available =
      true;
  guarded_signed_branch.require_no_native_branch_tail_helpers_when_available =
      true;
  guarded_signed_branch.native_branch_should_be_taken = true;
  cases.push_back(guarded_signed_branch);

  CpuCompareCase guarded_add_overflow{};
  guarded_add_overflow.name =
      "native_aggressive_branch_tail_guarded_add_overflow_fallback";
  guarded_add_overflow.initial_gpr[1] = 0x7FFFFFFFu;
  guarded_add_overflow.initial_gpr[2] = 1u;
  guarded_add_overflow.program = {
      enc_r(1, 2, 3, 0, 0x20), // ADD overflows before branch
      enc_i(0x05, 3, 0, 1),
      0,
      0,
  };
  guarded_add_overflow.instructions = 3;
  guarded_add_overflow
      .enable_aggressive_reduced_helper_branch_tail_for_x64 = true;
  guarded_add_overflow.expect_x64_fallback = true;
  cases.push_back(guarded_add_overflow);

  CpuCompareCase guarded_delay_addi_overflow{};
  guarded_delay_addi_overflow.name =
      "native_aggressive_branch_tail_guarded_delay_addi_overflow_fallback";
  guarded_delay_addi_overflow.initial_gpr[1] = 1u;
  guarded_delay_addi_overflow.initial_gpr[2] = 0x7FFFFFFFu;
  guarded_delay_addi_overflow.program = {
      enc_i(0x05, 1, 0, 1),      // BNE taken
      enc_i(0x08, 2, 2, 1),      // ADDI overflows in delay slot
      0,
  };
  guarded_delay_addi_overflow.instructions = 2;
  guarded_delay_addi_overflow
      .enable_aggressive_reduced_helper_branch_tail_for_x64 = true;
  guarded_delay_addi_overflow.expect_x64_fallback = true;
  cases.push_back(guarded_delay_addi_overflow);

  CpuCompareCase guarded_active_load_delay{};
  guarded_active_load_delay.name =
      "native_aggressive_branch_tail_active_load_delay";
  guarded_active_load_delay.initial_gpr[1] = 0x80011720u;
  guarded_active_load_delay.initial_gpr[2] = 0x11111111u;
  guarded_active_load_delay.memory.push_back(
      {0x00011720u, 0x00000007u});
  guarded_active_load_delay.program = {
      enc_i(0x23, 1, 2, 0),       // decoded LW leaves r2 pending
      enc_r(2, 0, 3, 0, 0x21),    // ADDU sees old r2, then load commits
      enc_i(0x05, 2, 0, 2),       // BNE sees loaded r2
      enc_i(0x08, 2, 4, 1),       // ADDI delay sees loaded r2
      0,
      0,
  };
  guarded_active_load_delay.instructions = 4;
  guarded_active_load_delay.segment_instructions = {1u, 3u};
  guarded_active_load_delay.segment_native_tiers = {
      {false, true, true}, {true, true, true}};
  guarded_active_load_delay.compare_segment_states = true;
  guarded_active_load_delay.enable_ram_load_fastpath_for_x64 = true;
  guarded_active_load_delay
      .enable_aggressive_reduced_helper_branch_tail_for_x64 = true;
  guarded_active_load_delay
      .require_native_aggressive_reduced_helper_branch_tail_entry_when_available =
      true;
  guarded_active_load_delay.require_native_branch_tail_when_available = true;
  guarded_active_load_delay
      .require_aggressive_reduced_helper_branch_tail_full_preflight_when_available =
      true;
  guarded_active_load_delay.native_branch_should_be_taken = true;
  cases.push_back(guarded_active_load_delay);

  CpuCompareCase aggressive_sw{};
  aggressive_sw.name = "native_aggressive_reduced_helper_branch_tail_sw";
  aggressive_sw.initial_gpr[1] = 0x80011640u;
  aggressive_sw.initial_gpr[2] = 0xAABBCCDDu;
  aggressive_sw.initial_gpr[3] = 1u;
  aggressive_sw.initial_gpr[4] = 2u;
  aggressive_sw.memory.push_back({0x00011640u, 0u});
  aggressive_sw.compare_memory_addresses.push_back(0x00011640u);
  aggressive_sw.program = {
      enc_i(0x2B, 1, 2, 0),
      enc_i(0x05, 3, 4, 1),
      enc_i(0x09, 0, 5, 0x0011),
      enc_i(0x09, 0, 6, 0x0022),
  };
  aggressive_sw.instructions = 3;
  aggressive_sw.enable_ram_load_fastpath_for_x64 = true;
  aggressive_sw.enable_aggressive_reduced_helper_branch_tail_for_x64 = true;
  aggressive_sw.require_full_native_when_available = true;
  aggressive_sw.require_native_branch_tail_when_available = true;
  aggressive_sw
      .require_native_aggressive_reduced_helper_branch_tail_entry_when_available =
      true;
  aggressive_sw
      .require_native_aggressive_reduced_helper_branch_tail_store_entry_when_available =
      true;
  aggressive_sw.require_no_native_instruction_helpers_when_available = true;
  aggressive_sw.require_no_native_branch_tail_helpers_when_available = true;
  aggressive_sw
      .require_aggressive_reduced_helper_branch_tail_direct_preflight_when_available =
      true;
  aggressive_sw.native_branch_should_be_taken = true;
  cases.push_back(aggressive_sw);

  CpuCompareCase aggressive_sb{};
  aggressive_sb.name = "native_aggressive_reduced_helper_branch_tail_sb";
  aggressive_sb.initial_gpr[1] = 0x80011680u;
  aggressive_sb.initial_gpr[2] = 0xAABBCCDDu;
  aggressive_sb.initial_gpr[3] = 1u;
  aggressive_sb.initial_gpr[4] = 2u;
  aggressive_sb.memory.push_back({0x00011680u, 0x11223344u});
  aggressive_sb.compare_memory_addresses.push_back(0x00011680u);
  aggressive_sb.program = {
      enc_i(0x28, 1, 2, 1),
      enc_i(0x05, 3, 4, 1),
      enc_i(0x09, 0, 5, 0x0011),
      enc_i(0x09, 0, 6, 0x0022),
  };
  aggressive_sb.instructions = 3;
  aggressive_sb.enable_ram_load_fastpath_for_x64 = true;
  aggressive_sb.enable_aggressive_reduced_helper_branch_tail_for_x64 = true;
  aggressive_sb.require_full_native_when_available = true;
  aggressive_sb.require_native_branch_tail_when_available = true;
  aggressive_sb
      .require_native_aggressive_reduced_helper_branch_tail_entry_when_available =
      true;
  aggressive_sb
      .require_native_aggressive_reduced_helper_branch_tail_store_entry_when_available =
      true;
  aggressive_sb.require_no_native_instruction_helpers_when_available = true;
  aggressive_sb.require_no_native_branch_tail_helpers_when_available = true;
  aggressive_sb.native_branch_should_be_taken = true;
  cases.push_back(aggressive_sb);

  CpuCompareCase aggressive_sh{};
  aggressive_sh.name = "native_aggressive_reduced_helper_branch_tail_sh";
  aggressive_sh.initial_gpr[1] = 0x80011684u;
  aggressive_sh.initial_gpr[2] = 0xAABBCCDDu;
  aggressive_sh.initial_gpr[3] = 3u;
  aggressive_sh.initial_gpr[4] = 3u;
  aggressive_sh.memory.push_back({0x00011684u, 0x11223344u});
  aggressive_sh.compare_memory_addresses.push_back(0x00011684u);
  aggressive_sh.program = {
      enc_i(0x29, 1, 2, 2),
      enc_i(0x05, 3, 4, 1),
      enc_i(0x09, 0, 5, 0x0011),
      enc_i(0x09, 0, 6, 0x0022),
  };
  aggressive_sh.instructions = 3;
  aggressive_sh.enable_ram_load_fastpath_for_x64 = true;
  aggressive_sh.enable_aggressive_reduced_helper_branch_tail_for_x64 = true;
  aggressive_sh.require_full_native_when_available = true;
  aggressive_sh.require_native_branch_tail_when_available = true;
  aggressive_sh
      .require_native_aggressive_reduced_helper_branch_tail_entry_when_available =
      true;
  aggressive_sh
      .require_native_aggressive_reduced_helper_branch_tail_store_entry_when_available =
      true;
  aggressive_sh.require_no_native_instruction_helpers_when_available = true;
  aggressive_sh.require_no_native_branch_tail_helpers_when_available = true;
  aggressive_sh.native_branch_should_be_taken = false;
  cases.push_back(aggressive_sh);

  CpuCompareCase aggressive_sb_lbu_order{};
  aggressive_sb_lbu_order.name =
      "native_aggressive_reduced_helper_branch_tail_sb_lbu_order";
  aggressive_sb_lbu_order.initial_gpr[1] = 0x80011690u;
  aggressive_sb_lbu_order.initial_gpr[2] = 0x0000005Au;
  aggressive_sb_lbu_order.initial_gpr[3] = 0x0000005Au;
  aggressive_sb_lbu_order.memory.push_back({0x00011690u, 0u});
  aggressive_sb_lbu_order.compare_memory_addresses.push_back(0x00011690u);
  aggressive_sb_lbu_order.program = {
      enc_i(0x28, 1, 2, 0),
      enc_i(0x24, 1, 5, 0),
      enc_i(0x0D, 0, 0, 0),
      enc_i(0x05, 5, 3, 1),
      enc_i(0x09, 0, 6, 0x0011),
      enc_i(0x09, 0, 7, 0x0022),
  };
  aggressive_sb_lbu_order.instructions = 5;
  aggressive_sb_lbu_order.enable_ram_load_fastpath_for_x64 = true;
  aggressive_sb_lbu_order
      .enable_aggressive_reduced_helper_branch_tail_for_x64 = true;
  aggressive_sb_lbu_order.require_full_native_when_available = true;
  aggressive_sb_lbu_order.require_native_branch_tail_when_available = true;
  aggressive_sb_lbu_order
      .require_native_aggressive_reduced_helper_branch_tail_entry_when_available =
      true;
  aggressive_sb_lbu_order
      .require_native_aggressive_reduced_helper_branch_tail_store_entry_when_available =
      true;
  aggressive_sb_lbu_order
      .require_native_aggressive_reduced_helper_branch_tail_mixed_entry_when_available =
      true;
  aggressive_sb_lbu_order.require_native_ram_load_fastpath_when_available =
      true;
  aggressive_sb_lbu_order.require_no_native_instruction_helpers_when_available =
      true;
  aggressive_sb_lbu_order.require_no_native_branch_tail_helpers_when_available =
      true;
  aggressive_sb_lbu_order.native_branch_should_be_taken = false;
  cases.push_back(aggressive_sb_lbu_order);

  CpuCompareCase aggressive_sw_after_alu{};
  aggressive_sw_after_alu.name =
      "native_aggressive_reduced_helper_branch_tail_sw_after_alu";
  aggressive_sw_after_alu.initial_gpr[1] = 0x80011650u;
  aggressive_sw_after_alu.initial_gpr[2] = 0x10u;
  aggressive_sw_after_alu.initial_gpr[3] = 7u;
  aggressive_sw_after_alu.initial_gpr[4] = 8u;
  aggressive_sw_after_alu.memory.push_back({0x00011650u, 0u});
  aggressive_sw_after_alu.compare_memory_addresses.push_back(0x00011650u);
  aggressive_sw_after_alu.program = {
      enc_i(0x09, 2, 2, 1),
      enc_i(0x2B, 1, 2, 0),
      enc_i(0x05, 3, 4, 1),
      enc_i(0x09, 0, 5, 0x0011),
      enc_i(0x09, 0, 6, 0x0022),
  };
  aggressive_sw_after_alu.instructions = 4;
  aggressive_sw_after_alu.enable_ram_load_fastpath_for_x64 = true;
  aggressive_sw_after_alu
      .enable_aggressive_reduced_helper_branch_tail_for_x64 = true;
  aggressive_sw_after_alu.require_full_native_when_available = true;
  aggressive_sw_after_alu.require_native_branch_tail_when_available = true;
  aggressive_sw_after_alu
      .require_native_aggressive_reduced_helper_branch_tail_entry_when_available =
      true;
  aggressive_sw_after_alu
      .require_native_aggressive_reduced_helper_branch_tail_store_entry_when_available =
      true;
  aggressive_sw_after_alu.require_no_native_instruction_helpers_when_available =
      true;
  aggressive_sw_after_alu.require_no_native_branch_tail_helpers_when_available =
      true;
  aggressive_sw_after_alu.native_branch_should_be_taken = true;
  cases.push_back(aggressive_sw_after_alu);

  CpuCompareCase aggressive_sw_base_after_alu{};
  aggressive_sw_base_after_alu.name =
      "native_aggressive_reduced_helper_branch_tail_sw_base_after_alu";
  aggressive_sw_base_after_alu.initial_gpr[1] = 0x80011660u;
  aggressive_sw_base_after_alu.initial_gpr[2] = 0x12345678u;
  aggressive_sw_base_after_alu.initial_gpr[3] = 1u;
  aggressive_sw_base_after_alu.initial_gpr[4] = 2u;
  aggressive_sw_base_after_alu.memory.push_back({0x00011664u, 0u});
  aggressive_sw_base_after_alu.compare_memory_addresses.push_back(0x00011664u);
  aggressive_sw_base_after_alu.program = {
      enc_i(0x09, 1, 1, 4),
      enc_i(0x2B, 1, 2, 0),
      enc_i(0x05, 3, 4, 1),
      enc_i(0x09, 0, 5, 0x0011),
      enc_i(0x09, 0, 6, 0x0022),
  };
  aggressive_sw_base_after_alu.instructions = 4;
  aggressive_sw_base_after_alu.enable_ram_load_fastpath_for_x64 = true;
  aggressive_sw_base_after_alu
      .enable_aggressive_reduced_helper_branch_tail_for_x64 = true;
  aggressive_sw_base_after_alu.require_full_native_when_available = true;
  aggressive_sw_base_after_alu.require_native_branch_tail_when_available = true;
  aggressive_sw_base_after_alu
      .require_native_aggressive_reduced_helper_branch_tail_entry_when_available =
      true;
  aggressive_sw_base_after_alu
      .require_native_aggressive_reduced_helper_branch_tail_store_entry_when_available =
      true;
  aggressive_sw_base_after_alu
      .require_no_native_instruction_helpers_when_available = true;
  aggressive_sw_base_after_alu
      .require_no_native_branch_tail_helpers_when_available = true;
  aggressive_sw_base_after_alu
      .require_aggressive_reduced_helper_branch_tail_full_preflight_when_available =
      true;
  aggressive_sw_base_after_alu.native_branch_should_be_taken = true;
  cases.push_back(aggressive_sw_base_after_alu);

  CpuCompareCase aggressive_lw_sw_order{};
  aggressive_lw_sw_order.name =
      "native_aggressive_reduced_helper_branch_tail_lw_sw_order";
  aggressive_lw_sw_order.initial_gpr[1] = 0x80011670u;
  aggressive_lw_sw_order.initial_gpr[2] = 0x11111111u;
  aggressive_lw_sw_order.initial_gpr[3] = 0x22222222u;
  aggressive_lw_sw_order.memory.push_back({0x00011670u, 0x22222222u});
  aggressive_lw_sw_order.memory.push_back({0x00011674u, 0u});
  aggressive_lw_sw_order.compare_memory_addresses.push_back(0x00011674u);
  aggressive_lw_sw_order.program = {
      enc_i(0x23, 1, 2, 0),
      enc_i(0x2B, 1, 2, 4),
      enc_r(2, 0, 5, 0, 0x21),
      enc_i(0x05, 2, 3, 1),
      enc_i(0x09, 0, 6, 0x0011),
      enc_i(0x09, 0, 7, 0x0022),
  };
  aggressive_lw_sw_order.instructions = 5;
  aggressive_lw_sw_order.enable_ram_load_fastpath_for_x64 = true;
  aggressive_lw_sw_order
      .enable_aggressive_reduced_helper_branch_tail_for_x64 = true;
  aggressive_lw_sw_order.require_full_native_when_available = true;
  aggressive_lw_sw_order.require_native_branch_tail_when_available = true;
  aggressive_lw_sw_order
      .require_native_aggressive_reduced_helper_branch_tail_entry_when_available =
      true;
  aggressive_lw_sw_order
      .require_native_aggressive_reduced_helper_branch_tail_store_entry_when_available =
      true;
  aggressive_lw_sw_order
      .require_native_aggressive_reduced_helper_branch_tail_mixed_entry_when_available =
      true;
  aggressive_lw_sw_order.require_native_ram_load_fastpath_when_available =
      true;
  aggressive_lw_sw_order.require_no_native_instruction_helpers_when_available =
      true;
  aggressive_lw_sw_order.require_no_native_branch_tail_helpers_when_available =
      true;
  cases.push_back(aggressive_lw_sw_order);

  CpuCompareCase aggressive_sw_scratchpad_reject{};
  aggressive_sw_scratchpad_reject.name =
      "native_scratchpad_sw_branch_tail";
  aggressive_sw_scratchpad_reject.initial_gpr[1] = 0x1F800000u;
  aggressive_sw_scratchpad_reject.initial_gpr[2] = 0x55667788u;
  aggressive_sw_scratchpad_reject.initial_gpr[3] = 1u;
  aggressive_sw_scratchpad_reject.initial_gpr[4] = 2u;
  aggressive_sw_scratchpad_reject.memory.push_back({0x1F800000u, 0u});
  aggressive_sw_scratchpad_reject.compare_memory_addresses.push_back(
      0x1F800000u);
  aggressive_sw_scratchpad_reject.program = {
      enc_i(0x2B, 1, 2, 0),
      enc_i(0x05, 3, 4, 1),
      enc_i(0x09, 0, 5, 0x0011),
      enc_i(0x09, 0, 6, 0x0022),
  };
  aggressive_sw_scratchpad_reject.instructions = 3;
  aggressive_sw_scratchpad_reject.enable_ram_load_fastpath_for_x64 = true;
  aggressive_sw_scratchpad_reject
      .enable_aggressive_reduced_helper_branch_tail_for_x64 = true;
  aggressive_sw_scratchpad_reject.require_v4_native_entry_when_available = true;
  aggressive_sw_scratchpad_reject
      .require_v4_native_store_entry_when_available = true;
  aggressive_sw_scratchpad_reject
      .require_v4_native_branch_entry_when_available = true;
  aggressive_sw_scratchpad_reject
      .require_aggressive_reduced_helper_branch_tail_preflight_non_ram_when_available =
      true;
  cases.push_back(aggressive_sw_scratchpad_reject);

  CpuCompareCase aggressive_sw_scratchpad_adaptive{};
  aggressive_sw_scratchpad_adaptive.name =
      "native_scratchpad_sw_branch_loop";
  aggressive_sw_scratchpad_adaptive.initial_gpr[1] = 0x1F800000u;
  aggressive_sw_scratchpad_adaptive.initial_gpr[2] = 0x55667788u;
  aggressive_sw_scratchpad_adaptive.initial_gpr[3] = 1u;
  aggressive_sw_scratchpad_adaptive.initial_gpr[4] = 2u;
  aggressive_sw_scratchpad_adaptive.memory.push_back({0x1F800000u, 0u});
  aggressive_sw_scratchpad_adaptive.compare_memory_addresses.push_back(
      0x1F800000u);
  aggressive_sw_scratchpad_adaptive.program = {
      enc_i(0x2B, 1, 2, 0),
      enc_i(0x05, 3, 4, 0xFFFE),
      enc_i(0x00, 0, 0, 0),
  };
  aggressive_sw_scratchpad_adaptive.instructions = 15;
  aggressive_sw_scratchpad_adaptive.enable_ram_load_fastpath_for_x64 = true;
  aggressive_sw_scratchpad_adaptive
      .enable_aggressive_reduced_helper_branch_tail_for_x64 = true;
  aggressive_sw_scratchpad_adaptive.require_v4_native_entry_when_available =
      true;
  aggressive_sw_scratchpad_adaptive
      .require_v4_native_store_entry_when_available = true;
  aggressive_sw_scratchpad_adaptive
      .require_v4_native_branch_entry_when_available = true;
  aggressive_sw_scratchpad_adaptive
      .require_aggressive_reduced_helper_branch_tail_preflight_non_ram_when_available =
      true;
  aggressive_sw_scratchpad_adaptive
      .require_aggressive_reduced_helper_branch_tail_adaptive_disable_when_available =
      true;
  aggressive_sw_scratchpad_adaptive
      .require_aggressive_reduced_helper_branch_tail_adaptive_direct_entry_when_available =
      true;
  cases.push_back(aggressive_sw_scratchpad_adaptive);

  CpuCompareCase aggressive_sb_scratchpad_reject{};
  aggressive_sb_scratchpad_reject.name =
      "native_scratchpad_sb_branch_tail";
  aggressive_sb_scratchpad_reject.initial_gpr[1] = 0x1F800004u;
  aggressive_sb_scratchpad_reject.initial_gpr[2] = 0x000000AAu;
  aggressive_sb_scratchpad_reject.initial_gpr[3] = 1u;
  aggressive_sb_scratchpad_reject.initial_gpr[4] = 2u;
  aggressive_sb_scratchpad_reject.memory.push_back(
      {0x1F800004u, 0x11223344u});
  aggressive_sb_scratchpad_reject.compare_memory_addresses.push_back(
      0x1F800004u);
  aggressive_sb_scratchpad_reject.program = {
      enc_i(0x28, 1, 2, 0),
      enc_i(0x05, 3, 4, 1),
      enc_i(0x09, 0, 5, 0x0011),
      enc_i(0x09, 0, 6, 0x0022),
  };
  aggressive_sb_scratchpad_reject.instructions = 3;
  aggressive_sb_scratchpad_reject.enable_ram_load_fastpath_for_x64 = true;
  aggressive_sb_scratchpad_reject
      .enable_aggressive_reduced_helper_branch_tail_for_x64 = true;
  aggressive_sb_scratchpad_reject.require_v4_native_entry_when_available = true;
  aggressive_sb_scratchpad_reject
      .require_v4_native_store_entry_when_available = true;
  aggressive_sb_scratchpad_reject
      .require_v4_native_branch_entry_when_available = true;
  aggressive_sb_scratchpad_reject
      .require_aggressive_reduced_helper_branch_tail_preflight_non_ram_when_available =
      true;
  cases.push_back(aggressive_sb_scratchpad_reject);

  CpuCompareCase aggressive_sw_code_page_reject{};
  aggressive_sw_code_page_reject.name =
      "native_aggressive_reduced_helper_branch_tail_sw_code_page_rejected";
  aggressive_sw_code_page_reject.initial_gpr[1] = kCpuComparePc;
  aggressive_sw_code_page_reject.initial_gpr[2] = enc_i(0x09, 0, 8, 0x0077);
  aggressive_sw_code_page_reject.initial_gpr[3] = 1u;
  aggressive_sw_code_page_reject.initial_gpr[4] = 2u;
  aggressive_sw_code_page_reject.compare_memory_addresses.push_back(
      kCpuComparePc);
  aggressive_sw_code_page_reject.program = {
      enc_i(0x2B, 1, 2, 0),
      enc_i(0x05, 3, 4, 1),
      enc_i(0x09, 0, 5, 0x0011),
      enc_i(0x09, 0, 6, 0x0022),
  };
  aggressive_sw_code_page_reject.instructions = 3;
  aggressive_sw_code_page_reject.enable_ram_load_fastpath_for_x64 = true;
  aggressive_sw_code_page_reject
      .enable_aggressive_reduced_helper_branch_tail_for_x64 = true;
  aggressive_sw_code_page_reject
      .require_aggressive_reduced_helper_branch_tail_preflight_code_page_when_available =
      true;
  cases.push_back(aggressive_sw_code_page_reject);

  CpuCompareCase beq_taken{};
  beq_taken.name = "native_branch_tail_beq_taken_alu_delay";
  beq_taken.initial_gpr[1] = 7u;
  beq_taken.initial_gpr[2] = 7u;
  beq_taken.program = {
      enc_i(0x04, 1, 2, 2),
      enc_i(0x09, 0, 3, 0x0011),
      enc_i(0x09, 0, 4, 0x0022),
      enc_i(0x09, 0, 5, 0x0033),
  };
  beq_taken.instructions = 2;
  beq_taken.require_full_native_when_available = true;
  beq_taken.require_native_branch_tail_when_available = true;
  beq_taken.native_branch_should_be_taken = true;
  cases.push_back(beq_taken);

  CpuCompareCase beq_not_taken{};
  beq_not_taken.name = "native_branch_tail_beq_not_taken_alu_delay";
  beq_not_taken.initial_gpr[1] = 7u;
  beq_not_taken.initial_gpr[2] = 8u;
  beq_not_taken.program = beq_taken.program;
  beq_not_taken.instructions = 2;
  beq_not_taken.require_full_native_when_available = true;
  beq_not_taken.require_native_branch_tail_when_available = true;
  cases.push_back(beq_not_taken);

  CpuCompareCase bgtz_taken{};
  bgtz_taken.name = "native_branch_tail_bgtz_taken_alu_delay";
  bgtz_taken.initial_gpr[1] = 1u;
  bgtz_taken.program = {
      enc_i(0x07, 1, 0, 1),
      enc_i(0x09, 0, 2, 0x0031),
      0,
  };
  bgtz_taken.instructions = 2;
  bgtz_taken.require_full_native_when_available = true;
  bgtz_taken.require_native_branch_tail_when_available = true;
  bgtz_taken.native_branch_should_be_taken = true;
  bgtz_taken.native_branch_primary_op = 0x07u;
  cases.push_back(bgtz_taken);

  CpuCompareCase bgtz_not_taken{};
  bgtz_not_taken.name = "native_branch_tail_bgtz_not_taken_alu_delay";
  bgtz_not_taken.initial_gpr[1] = 0xFFFFFFFFu;
  bgtz_not_taken.program = bgtz_taken.program;
  bgtz_not_taken.instructions = 2;
  bgtz_not_taken.require_full_native_when_available = true;
  bgtz_not_taken.require_native_branch_tail_when_available = true;
  bgtz_not_taken.native_branch_primary_op = 0x07u;
  cases.push_back(bgtz_not_taken);

  CpuCompareCase blez_taken{};
  blez_taken.name = "native_branch_tail_blez_taken_alu_delay";
  blez_taken.initial_gpr[1] = 0u;
  blez_taken.program = {
      enc_i(0x06, 1, 0, 1),
      enc_i(0x09, 0, 2, 0x0032),
      0,
  };
  blez_taken.instructions = 2;
  blez_taken.require_full_native_when_available = true;
  blez_taken.require_native_branch_tail_when_available = true;
  blez_taken.native_branch_should_be_taken = true;
  blez_taken.native_branch_primary_op = 0x06u;
  cases.push_back(blez_taken);

  CpuCompareCase blez_not_taken{};
  blez_not_taken.name = "native_branch_tail_blez_not_taken_alu_delay";
  blez_not_taken.initial_gpr[1] = 1u;
  blez_not_taken.program = blez_taken.program;
  blez_not_taken.instructions = 2;
  blez_not_taken.require_full_native_when_available = true;
  blez_not_taken.require_native_branch_tail_when_available = true;
  blez_not_taken.native_branch_primary_op = 0x06u;
  cases.push_back(blez_not_taken);

  CpuCompareCase bltz_taken{};
  bltz_taken.name = "native_branch_tail_bltz_taken_alu_delay";
  bltz_taken.initial_gpr[1] = 0xFFFFFFFFu;
  bltz_taken.program = {
      enc_i(0x01, 1, 0x00, 1),
      enc_i(0x09, 0, 2, 0x0034),
      0,
  };
  bltz_taken.instructions = 2;
  bltz_taken.require_full_native_when_available = true;
  bltz_taken.require_native_branch_tail_when_available = true;
  bltz_taken.native_branch_should_be_taken = true;
  cases.push_back(bltz_taken);

  CpuCompareCase bltz_not_taken{};
  bltz_not_taken.name = "native_branch_tail_bltz_not_taken_alu_delay";
  bltz_not_taken.initial_gpr[1] = 1u;
  bltz_not_taken.program = bltz_taken.program;
  bltz_not_taken.instructions = 2;
  bltz_not_taken.require_full_native_when_available = true;
  bltz_not_taken.require_native_branch_tail_when_available = true;
  cases.push_back(bltz_not_taken);

  CpuCompareCase bgez_taken{};
  bgez_taken.name = "native_branch_tail_bgez_taken_alu_delay";
  bgez_taken.initial_gpr[1] = 0u;
  bgez_taken.program = {
      enc_i(0x01, 1, 0x01, 1),
      enc_i(0x09, 0, 2, 0x0035),
      0,
  };
  bgez_taken.instructions = 2;
  bgez_taken.require_full_native_when_available = true;
  bgez_taken.require_native_branch_tail_when_available = true;
  bgez_taken.native_branch_should_be_taken = true;
  cases.push_back(bgez_taken);

  CpuCompareCase bgez_not_taken{};
  bgez_not_taken.name = "native_branch_tail_bgez_not_taken_alu_delay";
  bgez_not_taken.initial_gpr[1] = 0xFFFFFFFFu;
  bgez_not_taken.program = bgez_taken.program;
  bgez_not_taken.instructions = 2;
  bgez_not_taken.require_full_native_when_available = true;
  bgez_not_taken.require_native_branch_tail_when_available = true;
  cases.push_back(bgez_not_taken);

  CpuCompareCase bgtz_memory_loop{};
  bgtz_memory_loop.name = "native_branch_tail_bgtz_memory_store_delay";
  bgtz_memory_loop.initial_gpr[1] = 0x80011240u;
  bgtz_memory_loop.memory.push_back({0x00011240u, 2u});
  bgtz_memory_loop.program = {
      enc_i(0x24, 1, 2, 0),
      0,
      enc_i(0x09, 2, 2, 0xFFFF),
      enc_i(0x07, 2, 0, 0xFFFD),
      enc_i(0x28, 1, 2, 1),
  };
  bgtz_memory_loop.instructions = 5;
  bgtz_memory_loop.require_full_native_when_available = true;
  bgtz_memory_loop.require_native_memory_helper_when_available = true;
  bgtz_memory_loop.require_native_branch_tail_when_available = true;
  bgtz_memory_loop.native_branch_should_be_taken = true;
  bgtz_memory_loop.native_branch_primary_op = 0x07u;
  bgtz_memory_loop
      .require_native_branch_delay_memory_helper_when_available = true;
  cases.push_back(bgtz_memory_loop);

  CpuCompareCase blez_memory_delay{};
  blez_memory_delay.name = "native_branch_tail_blez_memory_load_delay";
  blez_memory_delay.initial_gpr[1] = 0u;
  blez_memory_delay.initial_gpr[6] = 0x80011250u;
  blez_memory_delay.memory.push_back({0x00011250u, 0x55667788u});
  blez_memory_delay.program = {
      enc_i(0x06, 1, 0, 1),
      enc_i(0x23, 6, 5, 0),
      0,
  };
  blez_memory_delay.instructions = 2;
  blez_memory_delay.require_full_native_when_available = true;
  blez_memory_delay.require_native_memory_helper_when_available = true;
  blez_memory_delay.require_native_branch_tail_when_available = true;
  blez_memory_delay.native_branch_should_be_taken = true;
  blez_memory_delay.native_branch_primary_op = 0x06u;
  blez_memory_delay
      .require_native_branch_delay_memory_helper_when_available = true;
  cases.push_back(blez_memory_delay);

  CpuCompareCase beq_memory_body_delay{};
  beq_memory_body_delay.name = "native_branch_tail_beq_memory_body_delay";
  beq_memory_body_delay.initial_gpr[1] = 0x80011260u;
  beq_memory_body_delay.initial_gpr[2] = 5u;
  beq_memory_body_delay.memory.push_back({0x00011264u, 0x11223344u});
  beq_memory_body_delay.program = {
      enc_i(0x2B, 1, 2, 0),
      enc_i(0x23, 1, 4, 0),
      0,
      enc_i(0x04, 4, 2, 1),
      enc_i(0x23, 1, 5, 4),
      0,
  };
  beq_memory_body_delay.instructions = 5;
  beq_memory_body_delay.require_full_native_when_available = true;
  beq_memory_body_delay.require_native_memory_helper_when_available = true;
  beq_memory_body_delay.require_native_branch_tail_when_available = true;
  beq_memory_body_delay.native_branch_should_be_taken = true;
  beq_memory_body_delay
      .require_native_branch_delay_memory_helper_when_available = true;
  cases.push_back(beq_memory_body_delay);

  CpuCompareCase bne_memory_body{};
  bne_memory_body.name = "native_branch_tail_bne_memory_body";
  bne_memory_body.initial_gpr[1] = 0x80011100u;
  bne_memory_body.initial_gpr[2] = 0x12345678u;
  bne_memory_body.program = {
      enc_i(0x2B, 1, 2, 0),
      enc_i(0x23, 1, 3, 0),
      0,
      enc_i(0x05, 3, 0, 1),
      enc_i(0x09, 3, 4, 1),
      0,
  };
  bne_memory_body.instructions = 5;
  bne_memory_body.require_full_native_when_available = true;
  bne_memory_body.require_native_memory_helper_when_available = true;
  bne_memory_body.require_native_branch_tail_when_available = true;
  bne_memory_body.native_branch_should_be_taken = true;
  cases.push_back(bne_memory_body);

  CpuCompareCase bne_lw_delay{};
  bne_lw_delay.name = "native_branch_tail_bne_lw_delay";
  bne_lw_delay.initial_gpr[1] = 1u;
  bne_lw_delay.initial_gpr[2] = 0u;
  bne_lw_delay.initial_gpr[6] = 0x80011120u;
  bne_lw_delay.memory.push_back({0x00011120u, 0xCAFEBABEu});
  bne_lw_delay.program = {
      enc_i(0x05, 1, 2, 1),
      enc_i(0x23, 6, 5, 0),
      0,
      0,
  };
  bne_lw_delay.instructions = 2;
  bne_lw_delay.require_full_native_when_available = true;
  bne_lw_delay.require_native_memory_helper_when_available = true;
  bne_lw_delay.require_native_branch_tail_when_available = true;
  bne_lw_delay.native_branch_should_be_taken = true;
  bne_lw_delay.require_native_branch_delay_memory_helper_when_available = true;
  cases.push_back(bne_lw_delay);

  CpuCompareCase branch_tail_then_decoded_consumer{};
  branch_tail_then_decoded_consumer.name =
      "native_branch_tail_then_decoded_load_consumer";
  branch_tail_then_decoded_consumer.initial_gpr[1] = 1u;
  branch_tail_then_decoded_consumer.initial_gpr[6] = 0x80011270u;
  branch_tail_then_decoded_consumer.memory.push_back(
      {0x00011270u, 0xABCDEF01u});
  branch_tail_then_decoded_consumer.program = {
      enc_i(0x05, 1, 0, 1),
      enc_i(0x23, 6, 5, 0),
      enc_r(5, 0, 7, 0, 0x21),
  };
  branch_tail_then_decoded_consumer.instructions = 3;
  branch_tail_then_decoded_consumer.segment_instructions = {2u, 1u};
  branch_tail_then_decoded_consumer.segment_native_tiers = {
      {true, true, true}, {false, true, true}};
  branch_tail_then_decoded_consumer.compare_segment_states = true;
  branch_tail_then_decoded_consumer
      .require_native_memory_helper_when_available = true;
  branch_tail_then_decoded_consumer
      .require_native_branch_tail_when_available = true;
  branch_tail_then_decoded_consumer.native_branch_should_be_taken = true;
  branch_tail_then_decoded_consumer
      .require_native_branch_delay_memory_helper_when_available = true;
  cases.push_back(branch_tail_then_decoded_consumer);

  CpuCompareCase decoded_load_then_branch_tail{};
  decoded_load_then_branch_tail.name =
      "decoded_load_then_native_branch_tail_cancel";
  decoded_load_then_branch_tail.initial_gpr[1] = 0x80011280u;
  decoded_load_then_branch_tail.initial_gpr[2] = 0x11111111u;
  decoded_load_then_branch_tail.initial_gpr[4] = 1u;
  decoded_load_then_branch_tail.memory.push_back(
      {0x00011280u, 0xDEADBEEFu});
  decoded_load_then_branch_tail.program = {
      enc_i(0x23, 1, 2, 0),
      enc_i(0x05, 4, 0, 1),
      enc_i(0x09, 0, 2, 7),
      0,
  };
  decoded_load_then_branch_tail.instructions = 3;
  decoded_load_then_branch_tail.segment_instructions = {1u, 2u};
  decoded_load_then_branch_tail.segment_native_tiers = {
      {false, true, true}, {true, true, true}};
  decoded_load_then_branch_tail.compare_segment_states = true;
  decoded_load_then_branch_tail
      .require_native_branch_tail_when_available = true;
  decoded_load_then_branch_tail.native_branch_should_be_taken = true;
  cases.push_back(decoded_load_then_branch_tail);

  CpuCompareCase bne_delay_exception{};
  bne_delay_exception.name = "native_branch_tail_delay_memory_exception";
  bne_delay_exception.initial_gpr[1] = 1u;
  bne_delay_exception.initial_gpr[2] = 0u;
  bne_delay_exception.initial_gpr[6] = 0x80011122u;
  bne_delay_exception.program = {
      enc_i(0x05, 1, 2, 1),
      enc_i(0x23, 6, 5, 0),
      0,
  };
  bne_delay_exception.instructions = 2;
  bne_delay_exception.require_full_native_when_available = true;
  bne_delay_exception.require_native_memory_helper_when_available = true;
  bne_delay_exception.require_native_memory_exception_when_available = true;
  bne_delay_exception.require_native_branch_tail_when_available = true;
  bne_delay_exception.native_branch_should_be_taken = true;
  bne_delay_exception.require_native_branch_delay_memory_helper_when_available =
      true;
  cases.push_back(bne_delay_exception);

  CpuCompareCase bne_load_delay_crossing{};
  bne_load_delay_crossing.name = "native_branch_tail_load_delay_crossing";
  bne_load_delay_crossing.initial_gpr[1] = 0x80011130u;
  bne_load_delay_crossing.initial_gpr[2] = 0u;
  bne_load_delay_crossing.memory.push_back({0x00011130u, 0x01020304u});
  bne_load_delay_crossing.program = {
      enc_i(0x23, 1, 2, 0),
      enc_i(0x05, 2, 0, 1),
      enc_r(2, 0, 3, 0, 0x21),
      0,
  };
  bne_load_delay_crossing.instructions = 3;
  bne_load_delay_crossing.require_full_native_when_available = true;
  bne_load_delay_crossing.require_native_memory_helper_when_available = true;
  bne_load_delay_crossing.require_native_branch_tail_when_available = true;
  cases.push_back(bne_load_delay_crossing);

  CpuCompareCase bne_loop_shape{};
  bne_loop_shape.name = "native_branch_tail_atrain_loop_shape";
  bne_loop_shape.initial_gpr[1] = 0x80011140u;
  bne_loop_shape.initial_gpr[2] = 1u;
  bne_loop_shape.initial_gpr[4] = 10u;
  bne_loop_shape.program = {
      0,
      enc_r(0, 2, 2, 1, 0x00),
      enc_r(4, 2, 3, 0, 0x23),
      enc_i(0x2B, 1, 3, 0),
      enc_i(0x23, 1, 5, 0),
      0,
      enc_i(0x09, 6, 6, 1),
      enc_i(0x2B, 1, 6, 4),
      enc_i(0x23, 1, 7, 4),
      0,
      enc_i(0x0A, 6, 8, 2),
      enc_i(0x05, 8, 0, 0xFFF4),
      enc_i(0x23, 1, 9, 8),
  };
  bne_loop_shape.memory.push_back({0x00011148u, 0x0BADF00Du});
  bne_loop_shape.instructions = 26;
  bne_loop_shape.require_full_native_when_available = true;
  bne_loop_shape.require_native_memory_helper_when_available = true;
  bne_loop_shape.require_native_branch_tail_when_available = true;
  bne_loop_shape.native_branch_should_be_taken = true;
  bne_loop_shape.require_native_branch_delay_memory_helper_when_available = true;
  cases.push_back(bne_loop_shape);

  CpuCompareCase atrain_splash_poll_pair{};
  atrain_splash_poll_pair.name =
      "native_branch_tail_atrain_splash_poll_pair_segments";
  atrain_splash_poll_pair.initial_gpr[1] = 0x80011300u;
  atrain_splash_poll_pair.memory.push_back({0x00011300u, 3u});
  atrain_splash_poll_pair.program = {
      enc_i(0x09, 1, 1, 0),
      enc_i(0x23, 1, 3, 0),
      0,
      enc_i(0x09, 3, 4, 1),
      enc_i(0x2B, 1, 4, 4),
      enc_i(0x23, 1, 5, 4),
      0,
      enc_i(0x05, 5, 0, 1),
      0,
      enc_i(0x0F, 0, 6, 0x8001),
      enc_i(0x23, 6, 7, 0x1304),
      0,
      enc_r(0, 7, 8, 0, 0x2A),
      enc_i(0x05, 8, 0, 0xFFFB),
      0,
  };
  atrain_splash_poll_pair.instructions = 15;
  atrain_splash_poll_pair.segment_instructions = {9u, 6u};
  atrain_splash_poll_pair.compare_segment_states = true;
  atrain_splash_poll_pair.require_full_native_when_available = true;
  atrain_splash_poll_pair.require_native_memory_helper_when_available = true;
  atrain_splash_poll_pair.require_native_branch_tail_when_available = true;
  atrain_splash_poll_pair.native_branch_should_be_taken = true;
  atrain_splash_poll_pair.compare_memory_addresses.push_back(0x00011304u);
  cases.push_back(atrain_splash_poll_pair);

  CpuCompareCase native_prefix_bne{};
  native_prefix_bne.name = "native_prefix_bne_taken";
  native_prefix_bne.program = {
      enc_i(0x09, 0, 1, 1),
      enc_i(0x09, 1, 2, 1),
      enc_i(0x05, 1, 2, 1),
      enc_i(0x09, 0, 3, 0x0011),
      enc_i(0x09, 0, 4, 0x0022),
      enc_i(0x09, 0, 5, 0x0033),
  };
  native_prefix_bne.instructions = 5;
  native_prefix_bne.disable_branch_tail_for_x64 = true;
  native_prefix_bne.enable_native_prefix_for_x64 = true;
  native_prefix_bne.require_native_prefix_entry_when_available = true;
  native_prefix_bne.require_native_prefix_bne_blocker_when_available = true;
  cases.push_back(native_prefix_bne);

  CpuCompareCase native_prefix_beq{};
  native_prefix_beq.name = "native_prefix_beq_not_taken";
  native_prefix_beq.program = {
      enc_i(0x09, 0, 1, 3),
      enc_i(0x09, 0, 2, 4),
      enc_i(0x04, 1, 2, 1),
      enc_i(0x09, 0, 3, 0x0011),
      enc_i(0x09, 0, 4, 0x0022),
      enc_i(0x09, 0, 5, 0x0033),
  };
  native_prefix_beq.instructions = 5;
  native_prefix_beq.disable_branch_tail_for_x64 = true;
  native_prefix_beq.enable_native_prefix_for_x64 = true;
  native_prefix_beq.require_native_prefix_entry_when_available = true;
  native_prefix_beq.require_native_prefix_beq_blocker_when_available = true;
  cases.push_back(native_prefix_beq);

  CpuCompareCase native_prefix_jr{};
  native_prefix_jr.name = "native_prefix_jr";
  native_prefix_jr.initial_gpr[8] = kCpuComparePc + 0x10u;
  native_prefix_jr.program = {
      enc_i(0x09, 0, 1, 5),
      enc_i(0x09, 1, 2, 1),
      enc_r(8, 0, 0, 0, 0x08),
      enc_i(0x09, 0, 3, 0x0011),
      enc_i(0x09, 0, 4, 0x0022),
      enc_i(0x09, 0, 5, 0x0033),
  };
  native_prefix_jr.instructions = 5;
  cases.push_back(native_prefix_jr);

  CpuCompareCase native_prefix_cop2{};
  native_prefix_cop2.name = "native_prefix_cop2_blocker";
  native_prefix_cop2.program = {
      enc_i(0x09, 0, 1, 7),
      enc_i(0x09, 1, 2, 1),
      (0x12u << 26) | (0u << 21) | (2u << 16) | (0u << 11),
      enc_i(0x09, 0, 3, 3),
  };
  native_prefix_cop2.instructions = 4;
  cases.push_back(native_prefix_cop2);

  CpuCompareCase native_prefix_unsupported{};
  native_prefix_unsupported.name = "native_prefix_unsupported_blocker";
  native_prefix_unsupported.program = {
      enc_i(0x09, 0, 1, 9),
      enc_i(0x09, 1, 2, 1),
      0xFC000000u,
      enc_i(0x09, 0, 3, 3),
  };
  native_prefix_unsupported.instructions = 3;
  native_prefix_unsupported.enable_native_prefix_for_x64 = true;
  native_prefix_unsupported.require_native_prefix_entry_when_available = true;
  native_prefix_unsupported.require_native_prefix_other_blocker_when_available =
      true;
  native_prefix_unsupported.require_no_native_instruction_helpers_when_available =
      true;
  cases.push_back(native_prefix_unsupported);

  CpuCompareCase native_prefix_ram_lw_bne{};
  native_prefix_ram_lw_bne.name = "native_prefix_ram_load_lw_bne";
  native_prefix_ram_lw_bne.initial_gpr[8] = 0x80011700u;
  native_prefix_ram_lw_bne.memory.push_back({0x00011700u, 0x12345678u});
  native_prefix_ram_lw_bne.program = {
      enc_i(0x09, 0, 1, 1),
      enc_i(0x23, 8, 2, 0),
      enc_i(0x05, 1, 0, 1),
      enc_i(0x09, 0, 3, 0x0011),
      enc_i(0x09, 0, 4, 0x0022),
      enc_i(0x09, 0, 5, 0x0033),
  };
  native_prefix_ram_lw_bne.instructions = 5;
  native_prefix_ram_lw_bne.disable_branch_tail_for_x64 = true;
  native_prefix_ram_lw_bne.enable_native_prefix_for_x64 = true;
  native_prefix_ram_lw_bne.enable_ram_load_fastpath_for_x64 = true;
  native_prefix_ram_lw_bne.require_native_prefix_entry_when_available =
      true;
  native_prefix_ram_lw_bne.require_native_prefix_ram_load_entry_when_available =
      true;
  native_prefix_ram_lw_bne.require_native_prefix_bne_blocker_when_available =
      true;
  native_prefix_ram_lw_bne.require_native_ram_load_fastpath_when_available =
      true;
  cases.push_back(native_prefix_ram_lw_bne);

  CpuCompareCase native_prefix_ram_lw_beq_independent{};
  native_prefix_ram_lw_beq_independent.name =
      "native_prefix_ram_load_lw_independent_beq";
  native_prefix_ram_lw_beq_independent.initial_gpr[8] = 0x80011710u;
  native_prefix_ram_lw_beq_independent.memory.push_back(
      {0x00011710u, 0x87654321u});
  native_prefix_ram_lw_beq_independent.program = {
      enc_i(0x09, 0, 1, 3),
      enc_i(0x23, 8, 2, 0),
      enc_i(0x09, 1, 3, 1),
      enc_i(0x04, 1, 0, 1),
      enc_i(0x09, 0, 4, 0x0022),
      enc_i(0x09, 0, 5, 0x0033),
  };
  native_prefix_ram_lw_beq_independent.instructions = 5;
  native_prefix_ram_lw_beq_independent.disable_branch_tail_for_x64 = true;
  native_prefix_ram_lw_beq_independent.enable_native_prefix_for_x64 = true;
  native_prefix_ram_lw_beq_independent.enable_ram_load_fastpath_for_x64 =
      true;
  native_prefix_ram_lw_beq_independent
      .require_native_prefix_entry_when_available = true;
  native_prefix_ram_lw_beq_independent
      .require_native_prefix_ram_load_entry_when_available = true;
  native_prefix_ram_lw_beq_independent
      .require_native_prefix_beq_blocker_when_available = true;
  native_prefix_ram_lw_beq_independent
      .require_native_ram_load_fastpath_when_available = true;
  cases.push_back(native_prefix_ram_lw_beq_independent);

  CpuCompareCase native_prefix_ram_lw_r0{};
  native_prefix_ram_lw_r0.name = "native_prefix_ram_load_lw_r0";
  native_prefix_ram_lw_r0.initial_gpr[1] = 1u;
  native_prefix_ram_lw_r0.initial_gpr[8] = 0x80011720u;
  native_prefix_ram_lw_r0.memory.push_back({0x00011720u, 0xFFFFFFFFu});
  native_prefix_ram_lw_r0.program = {
      enc_i(0x09, 0, 3, 7),
      enc_i(0x23, 8, 0, 0),
      enc_i(0x05, 1, 0, 1),
      enc_i(0x09, 0, 4, 0x0011),
      enc_i(0x09, 0, 5, 0x0022),
      enc_i(0x09, 0, 6, 0x0033),
  };
  native_prefix_ram_lw_r0.instructions = 5;
  native_prefix_ram_lw_r0.disable_branch_tail_for_x64 = true;
  native_prefix_ram_lw_r0.enable_native_prefix_for_x64 = true;
  native_prefix_ram_lw_r0.enable_ram_load_fastpath_for_x64 = true;
  native_prefix_ram_lw_r0.require_native_prefix_entry_when_available =
      true;
  native_prefix_ram_lw_r0.require_native_prefix_ram_load_entry_when_available =
      true;
  native_prefix_ram_lw_r0.require_native_prefix_bne_blocker_when_available =
      true;
  native_prefix_ram_lw_r0.require_native_ram_load_fastpath_when_available =
      true;
  cases.push_back(native_prefix_ram_lw_r0);

  CpuCompareCase native_prefix_ram_lw_consumed{};
  native_prefix_ram_lw_consumed.name =
      "native_prefix_ram_load_delay_consumed";
  native_prefix_ram_lw_consumed.initial_gpr[2] = 0x10u;
  native_prefix_ram_lw_consumed.initial_gpr[8] = 0x80011730u;
  native_prefix_ram_lw_consumed.memory.push_back(
      {0x00011730u, 0x22222222u});
  native_prefix_ram_lw_consumed.program = {
      enc_i(0x09, 0, 1, 1),
      enc_i(0x23, 8, 2, 0),
      enc_i(0x09, 2, 3, 1),
      enc_i(0x05, 1, 0, 1),
      enc_i(0x09, 0, 4, 0x0011),
      enc_i(0x09, 0, 5, 0x0022),
  };
  native_prefix_ram_lw_consumed.instructions = 5;
  native_prefix_ram_lw_consumed.disable_branch_tail_for_x64 = true;
  native_prefix_ram_lw_consumed.enable_native_prefix_for_x64 = true;
  native_prefix_ram_lw_consumed.enable_ram_load_fastpath_for_x64 = true;
  native_prefix_ram_lw_consumed.require_native_prefix_entry_when_available =
      true;
  native_prefix_ram_lw_consumed
      .require_native_prefix_ram_load_entry_when_available = true;
  native_prefix_ram_lw_consumed
      .require_native_prefix_bne_blocker_when_available = true;
  native_prefix_ram_lw_consumed
      .require_native_ram_load_fastpath_when_available = true;
  cases.push_back(native_prefix_ram_lw_consumed);

  CpuCompareCase native_prefix_ram_lw_pending_bne{};
  native_prefix_ram_lw_pending_bne.name =
      "native_prefix_ram_load_pending_at_bne";
  native_prefix_ram_lw_pending_bne.initial_gpr[2] = 0u;
  native_prefix_ram_lw_pending_bne.initial_gpr[8] = 0x80011740u;
  native_prefix_ram_lw_pending_bne.memory.push_back(
      {0x00011740u, 1u});
  native_prefix_ram_lw_pending_bne.program = {
      enc_i(0x09, 0, 1, 1),
      enc_i(0x23, 8, 2, 0),
      enc_i(0x05, 2, 0, 1),
      enc_i(0x09, 0, 3, 0x0011),
      enc_i(0x09, 0, 4, 0x0022),
      enc_i(0x09, 0, 5, 0x0033),
  };
  native_prefix_ram_lw_pending_bne.instructions = 5;
  native_prefix_ram_lw_pending_bne.disable_branch_tail_for_x64 = true;
  native_prefix_ram_lw_pending_bne.enable_native_prefix_for_x64 = true;
  native_prefix_ram_lw_pending_bne.enable_ram_load_fastpath_for_x64 =
      true;
  native_prefix_ram_lw_pending_bne
      .require_native_prefix_entry_when_available = true;
  native_prefix_ram_lw_pending_bne
      .require_native_prefix_ram_load_entry_when_available = true;
  native_prefix_ram_lw_pending_bne
      .require_native_prefix_bne_blocker_when_available = true;
  native_prefix_ram_lw_pending_bne
      .require_native_ram_load_fastpath_when_available = true;
  cases.push_back(native_prefix_ram_lw_pending_bne);

  CpuCompareCase native_prefix_ram_lw_scratchpad{};
  native_prefix_ram_lw_scratchpad.name =
      "native_prefix_ram_load_scratchpad_fallback";
  native_prefix_ram_lw_scratchpad.initial_gpr[1] = 1u;
  native_prefix_ram_lw_scratchpad.initial_gpr[8] = 0x1F800000u;
  native_prefix_ram_lw_scratchpad.memory.push_back(
      {0x1F800000u, 0x44556677u});
  native_prefix_ram_lw_scratchpad.program = {
      enc_i(0x09, 0, 3, 7),
      enc_i(0x23, 8, 2, 0),
      enc_i(0x05, 1, 0, 1),
      enc_i(0x09, 0, 4, 0x0011),
      enc_i(0x09, 0, 5, 0x0022),
  };
  native_prefix_ram_lw_scratchpad.instructions = 5;
  native_prefix_ram_lw_scratchpad.disable_branch_tail_for_x64 = true;
  native_prefix_ram_lw_scratchpad.enable_native_prefix_for_x64 = true;
  native_prefix_ram_lw_scratchpad.enable_ram_load_fastpath_for_x64 =
      true;
  native_prefix_ram_lw_scratchpad
      .require_native_memory_helper_when_available = true;
  native_prefix_ram_lw_scratchpad
      .require_native_prefix_ram_load_preflight_non_ram_when_available =
      true;
  native_prefix_ram_lw_scratchpad.require_no_native_ram_load_fastpath =
      true;
  cases.push_back(native_prefix_ram_lw_scratchpad);

  CpuCompareCase native_prefix_store_reject{};
  native_prefix_store_reject.name = "native_prefix_store_still_rejected";
  native_prefix_store_reject.initial_gpr[1] = 1u;
  native_prefix_store_reject.initial_gpr[2] = 0xAABBCCDDu;
  native_prefix_store_reject.initial_gpr[8] = 0x80011750u;
  native_prefix_store_reject.memory.push_back({0x00011750u, 0u});
  native_prefix_store_reject.compare_memory_addresses.push_back(0x00011750u);
  native_prefix_store_reject.program = {
      enc_i(0x09, 0, 3, 7),
      enc_i(0x2B, 8, 2, 0),
      enc_i(0x05, 1, 0, 1),
      enc_i(0x09, 0, 4, 0x0011),
      enc_i(0x09, 0, 5, 0x0022),
  };
  native_prefix_store_reject.instructions = 5;
  native_prefix_store_reject.disable_branch_tail_for_x64 = true;
  native_prefix_store_reject.enable_native_prefix_for_x64 = true;
  native_prefix_store_reject.enable_ram_load_fastpath_for_x64 = true;
  cases.push_back(native_prefix_store_reject);

  CpuCompareCase native_prefix_ram_lw_base_written_aggressive{};
  native_prefix_ram_lw_base_written_aggressive.name =
      "native_prefix_ram_load_base_written_aggressive";
  native_prefix_ram_lw_base_written_aggressive.initial_gpr[1] = 1u;
  native_prefix_ram_lw_base_written_aggressive.initial_gpr[8] = 0x80011700u;
  native_prefix_ram_lw_base_written_aggressive.memory.push_back(
      {0x00011710u, 0x13572468u});
  native_prefix_ram_lw_base_written_aggressive.program = {
      enc_i(0x09, 8, 8, 0x0010),
      enc_i(0x23, 8, 2, 0),
      enc_i(0x05, 1, 0, 1),
      enc_i(0x09, 0, 3, 0x0011),
      enc_i(0x09, 0, 4, 0x0022),
      enc_i(0x09, 0, 5, 0x0033),
  };
  native_prefix_ram_lw_base_written_aggressive.instructions = 5;
  native_prefix_ram_lw_base_written_aggressive.disable_branch_tail_for_x64 =
      true;
  native_prefix_ram_lw_base_written_aggressive.enable_native_prefix_for_x64 =
      true;
  native_prefix_ram_lw_base_written_aggressive
      .enable_aggressive_native_prefix_ram_for_x64 = true;
  native_prefix_ram_lw_base_written_aggressive
      .enable_ram_load_fastpath_for_x64 = true;
  native_prefix_ram_lw_base_written_aggressive
      .require_native_prefix_entry_when_available = true;
  native_prefix_ram_lw_base_written_aggressive
      .require_native_prefix_ram_load_entry_when_available = true;
  native_prefix_ram_lw_base_written_aggressive
      .require_native_prefix_ram_load_aggressive_entry_when_available = true;
  native_prefix_ram_lw_base_written_aggressive
      .require_native_prefix_ram_load_full_preflight_when_available = true;
  native_prefix_ram_lw_base_written_aggressive
      .require_native_prefix_bne_blocker_when_available = true;
  native_prefix_ram_lw_base_written_aggressive
      .require_native_ram_load_fastpath_when_available = true;
  cases.push_back(native_prefix_ram_lw_base_written_aggressive);

  CpuCompareCase native_prefix_ram_lw_scratchpad_adaptive{};
  native_prefix_ram_lw_scratchpad_adaptive.name =
      "native_prefix_ram_load_scratchpad_adaptive";
  native_prefix_ram_lw_scratchpad_adaptive.initial_gpr[1] = 1u;
  native_prefix_ram_lw_scratchpad_adaptive.initial_gpr[8] = 0x1F800000u;
  native_prefix_ram_lw_scratchpad_adaptive.memory.push_back(
      {0x1F800000u, 0x55667788u});
  native_prefix_ram_lw_scratchpad_adaptive.program = {
      enc_i(0x09, 0, 3, 7),
      enc_i(0x23, 8, 2, 0),
      enc_i(0x05, 1, 0, 0xFFFD),
      enc_i(0x09, 0, 4, 1),
  };
  native_prefix_ram_lw_scratchpad_adaptive.instructions = 20;
  native_prefix_ram_lw_scratchpad_adaptive.disable_branch_tail_for_x64 = true;
  native_prefix_ram_lw_scratchpad_adaptive.enable_native_prefix_for_x64 = true;
  native_prefix_ram_lw_scratchpad_adaptive
      .enable_aggressive_native_prefix_ram_for_x64 = true;
  native_prefix_ram_lw_scratchpad_adaptive.enable_ram_load_fastpath_for_x64 =
      true;
  native_prefix_ram_lw_scratchpad_adaptive
      .require_native_memory_helper_when_available = true;
  native_prefix_ram_lw_scratchpad_adaptive
      .require_native_prefix_ram_load_preflight_non_ram_when_available =
      true;
  native_prefix_ram_lw_scratchpad_adaptive
      .require_native_prefix_ram_load_adaptive_disable_when_available = true;
  native_prefix_ram_lw_scratchpad_adaptive
      .require_native_prefix_ram_load_adaptive_direct_entry_when_available =
      true;
  native_prefix_ram_lw_scratchpad_adaptive.require_no_native_ram_load_fastpath =
      true;
  cases.push_back(native_prefix_ram_lw_scratchpad_adaptive);

  CpuCompareCase branch_tail_disabled{};
  branch_tail_disabled.name = "native_branch_tail_disabled_gate";
  branch_tail_disabled.initial_gpr[1] = 1u;
  branch_tail_disabled.program = {
      enc_i(0x05, 1, 0, 1),
      enc_i(0x09, 0, 2, 1),
      0,
  };
  branch_tail_disabled.instructions = 2;
  branch_tail_disabled.disable_branch_tail_for_x64 = true;
  cases.push_back(branch_tail_disabled);

  CpuCompareCase branch_tail_blacklisted{};
  branch_tail_blacklisted.name = "native_branch_tail_pc_blacklist";
  branch_tail_blacklisted.initial_gpr[1] = 1u;
  branch_tail_blacklisted.program = branch_tail_disabled.program;
  branch_tail_blacklisted.instructions = 2;
  branch_tail_blacklisted.blacklist_branch_tail_for_x64 = true;
  cases.push_back(branch_tail_blacklisted);

  CpuCompareCase branch_irq_before{};
  branch_irq_before.name = "native_branch_tail_irq_pending_before_branch";
  branch_irq_before.initial_gpr[1] = 1u;
  branch_irq_before.initial_cop0_sr_bits = 1u | (1u << 10);
  branch_irq_before.initial_irq_mask = 1u;
  branch_irq_before.initial_irq_pending = true;
  branch_irq_before.program = {
      enc_i(0x05, 1, 0, 1),
      enc_i(0x09, 0, 2, 1),
      0,
  };
  branch_irq_before.instructions = 1;
  branch_irq_before.require_v4_entry_exception_native_when_available = true;
  cases.push_back(branch_irq_before);

  // An interrupt taken at a GTE command must still run the command: hardware
  // has already issued it, EPC keeps pointing at it, and the handler steps over
  // it. RTPS on zeroed registers divides by zero -> FLAG bit 17.
  CpuCompareCase irq_at_gte_command{};
  irq_at_gte_command.name = "irq_at_gte_command_still_executes";
  irq_at_gte_command.initial_cop0_sr_bits = 1u | (1u << 10) | (1u << 30);
  irq_at_gte_command.initial_irq_mask = 1u;
  irq_at_gte_command.initial_irq_pending = true;
  irq_at_gte_command.program = {
      0x4A180001u, // RTPS
      0,
  };
  irq_at_gte_command.instructions = 1;
  irq_at_gte_command.expect_gte_flags_mask = 0x20000u;
  irq_at_gte_command.expect_gte_flags_value = 0x20000u;
  cases.push_back(irq_at_gte_command);

  CpuCompareCase branch_irq_delay{};
  branch_irq_delay.name = "native_branch_tail_irq_pending_before_delay";
  branch_irq_delay.initial_gpr[1] = 1u;
  branch_irq_delay.initial_cop0_sr_bits = 1u | (1u << 10);
  branch_irq_delay.initial_irq_mask = 1u;
  branch_irq_delay.request_irq_on_branch = true;
  branch_irq_delay.program = {
      enc_i(0x05, 1, 0, 1),
      enc_i(0x09, 0, 2, 1),
      0,
  };
  branch_irq_delay.instructions = 2;
  branch_irq_delay.segment_instructions = {1u, 1u};
  branch_irq_delay.allow_partial_native_branch_tail = true;
  branch_irq_delay.compare_segment_states = true;
  branch_irq_delay.native_branch_should_be_taken = true;
  branch_irq_delay.require_v4_native_entry_when_available = true;
  branch_irq_delay.require_v4_native_branch_entry_when_available = true;
  branch_irq_delay.require_v4_pending_delay_native_when_available = true;
  cases.push_back(branch_irq_delay);

  CpuCompareCase pending_delay_irq{};
  pending_delay_irq.name = "v4_pending_delay_irq_sampling";
  pending_delay_irq.start_pc = 0xA0010004u;
  pending_delay_irq.initial_next_pc = 0xA0010040u;
  pending_delay_irq.initial_pending_delay_slot = true;
  pending_delay_irq.initial_pending_branch_taken = true;
  pending_delay_irq.initial_pending_branch_pc = 0xA0010000u;
  pending_delay_irq.initial_cop0_sr_bits = 1u | (1u << 10);
  pending_delay_irq.initial_irq_mask = 1u;
  pending_delay_irq.initial_irq_pending = true;
  pending_delay_irq.initial_gpr[3] = 0u;
  pending_delay_irq.program = {
      enc_i(0x09, 3, 3, 1),
      0u,
  };
  pending_delay_irq.instructions = 2u;
  // The delay slot and the interrupt sampled after it must both remain in V4.
  pending_delay_irq.require_v4_native_entry_when_available = true;
  pending_delay_irq.require_v4_pending_delay_native_when_available = true;
  cases.push_back(pending_delay_irq);

  CpuCompareCase branch_mmio_body{};
  branch_mmio_body.name = "native_branch_tail_mmio_body_read_write";
  branch_mmio_body.initial_gpr[1] = 0x1F801070u;
  branch_mmio_body.initial_gpr[2] = 1u;
  branch_mmio_body.program = {
      enc_i(0x2B, 1, 2, 4),
      enc_i(0x23, 1, 3, 4),
      0,
      enc_i(0x05, 3, 0, 1),
      enc_i(0x09, 0, 4, 1),
      0,
  };
  branch_mmio_body.instructions = 5;
  branch_mmio_body.require_full_native_when_available = true;
  branch_mmio_body.require_native_memory_helper_when_available = true;
  branch_mmio_body.require_native_mmio_when_available = true;
  branch_mmio_body.require_native_branch_tail_when_available = true;
  branch_mmio_body.native_branch_should_be_taken = true;
  cases.push_back(branch_mmio_body);

  CpuCompareCase branch_mmio_load_delay{};
  branch_mmio_load_delay.name = "native_branch_tail_taken_mmio_load_delay";
  branch_mmio_load_delay.initial_gpr[1] = 1u;
  branch_mmio_load_delay.initial_gpr[6] = 0x1F801070u;
  branch_mmio_load_delay.initial_irq_mask = 1u;
  branch_mmio_load_delay.initial_irq_pending = true;
  branch_mmio_load_delay.program = {
      enc_i(0x05, 1, 0, 1),
      enc_i(0x23, 6, 5, 0),
      0,
  };
  branch_mmio_load_delay.instructions = 2;
  branch_mmio_load_delay.require_full_native_when_available = true;
  branch_mmio_load_delay.require_native_memory_helper_when_available = true;
  branch_mmio_load_delay.require_native_mmio_when_available = true;
  branch_mmio_load_delay.require_native_branch_tail_when_available = true;
  branch_mmio_load_delay.native_branch_should_be_taken = true;
  branch_mmio_load_delay
      .require_native_branch_delay_memory_helper_when_available = true;
  cases.push_back(branch_mmio_load_delay);

  CpuCompareCase branch_mmio_store_delay{};
  branch_mmio_store_delay.name = "native_branch_tail_taken_mmio_store_delay";
  branch_mmio_store_delay.initial_gpr[1] = 1u;
  branch_mmio_store_delay.initial_gpr[2] = 1u;
  branch_mmio_store_delay.initial_gpr[6] = 0x1F801070u;
  branch_mmio_store_delay.program = {
      enc_i(0x05, 1, 0, 1),
      enc_i(0x2B, 6, 2, 4),
      0,
  };
  branch_mmio_store_delay.instructions = 2;
  branch_mmio_store_delay.require_full_native_when_available = true;
  branch_mmio_store_delay.require_native_memory_helper_when_available = true;
  branch_mmio_store_delay.require_native_mmio_when_available = true;
  branch_mmio_store_delay.require_native_branch_tail_when_available = true;
  branch_mmio_store_delay.native_branch_should_be_taken = true;
  branch_mmio_store_delay
      .require_native_branch_delay_memory_helper_when_available = true;
  cases.push_back(branch_mmio_store_delay);

  CpuCompareCase branch_not_taken_lw_delay{};
  branch_not_taken_lw_delay.name =
      "native_branch_tail_not_taken_memory_delay";
  branch_not_taken_lw_delay.initial_gpr[1] = 1u;
  branch_not_taken_lw_delay.initial_gpr[2] = 1u;
  branch_not_taken_lw_delay.initial_gpr[6] = 0x80011160u;
  branch_not_taken_lw_delay.memory.push_back(
      {0x00011160u, 0x11223344u});
  branch_not_taken_lw_delay.program = {
      enc_i(0x05, 1, 2, 1),
      enc_i(0x23, 6, 5, 0),
      0,
  };
  branch_not_taken_lw_delay.instructions = 2;
  branch_not_taken_lw_delay.require_full_native_when_available = true;
  branch_not_taken_lw_delay.require_native_memory_helper_when_available = true;
  branch_not_taken_lw_delay.require_native_branch_tail_when_available = true;
  branch_not_taken_lw_delay
      .require_native_branch_delay_memory_helper_when_available = true;
  cases.push_back(branch_not_taken_lw_delay);

  CpuCompareCase repeated_mmio_branch{};
  repeated_mmio_branch.name = "native_repeated_mmio_status_reads_branch";
  repeated_mmio_branch.initial_gpr[1] = 0x1F801070u;
  repeated_mmio_branch.initial_irq_mask = 1u;
  repeated_mmio_branch.initial_irq_pending = true;
  repeated_mmio_branch.program = {
      enc_i(0x23, 1, 2, 0),
      0,
      enc_i(0x23, 1, 3, 0),
      0,
      enc_i(0x05, 2, 3, 1),
      0,
      0,
  };
  repeated_mmio_branch.instructions = 6;
  repeated_mmio_branch.require_full_native_when_available = true;
  repeated_mmio_branch.require_native_memory_helper_when_available = true;
  repeated_mmio_branch.require_native_mmio_when_available = true;
  repeated_mmio_branch.require_native_branch_tail_when_available = true;
  cases.push_back(repeated_mmio_branch);

  CpuCompareCase mmio_load_delay_branch{};
  mmio_load_delay_branch.name = "native_mmio_load_delay_then_branch";
  mmio_load_delay_branch.initial_gpr[1] = 0x1F801070u;
  mmio_load_delay_branch.initial_gpr[2] = 0u;
  mmio_load_delay_branch.initial_irq_mask = 1u;
  mmio_load_delay_branch.initial_irq_pending = true;
  mmio_load_delay_branch.program = {
      enc_i(0x23, 1, 2, 0),
      enc_i(0x05, 2, 0, 1),
      0,
      0,
  };
  mmio_load_delay_branch.instructions = 3;
  mmio_load_delay_branch.require_full_native_when_available = true;
  mmio_load_delay_branch.require_native_memory_helper_when_available = true;
  mmio_load_delay_branch.require_native_mmio_when_available = true;
  mmio_load_delay_branch.require_native_branch_tail_when_available = true;
  cases.push_back(mmio_load_delay_branch);

  CpuCompareCase mmio_write_branch{};
  mmio_write_branch.name = "native_mmio_write_then_branch";
  mmio_write_branch.initial_gpr[1] = 0x1F801070u;
  mmio_write_branch.initial_gpr[2] = 1u;
  mmio_write_branch.program = {
      enc_i(0x2B, 1, 2, 4),
      enc_i(0x05, 2, 0, 1),
      0,
      0,
  };
  mmio_write_branch.instructions = 3;
  mmio_write_branch.require_full_native_when_available = true;
  mmio_write_branch.require_native_memory_helper_when_available = true;
  mmio_write_branch.require_native_mmio_when_available = true;
  mmio_write_branch.require_native_branch_tail_when_available = true;
  mmio_write_branch.native_branch_should_be_taken = true;
  cases.push_back(mmio_write_branch);

  CpuCompareCase mmio_enable_pending_irq{};
  mmio_enable_pending_irq.name =
      "v4_mmio_imask_enable_resamples_pending_irq";
  mmio_enable_pending_irq.start_pc = 0xA0010000u;
  mmio_enable_pending_irq.initial_gpr[1] = 0x1F801070u;
  mmio_enable_pending_irq.initial_gpr[2] = 1u;
  mmio_enable_pending_irq.initial_gpr[3] = 0x12345678u;
  // VBlank starts pending but masked. The native SW makes it eligible; the
  // following ADDIU must be preempted by interrupt entry.
  mmio_enable_pending_irq.initial_cop0_sr_bits = 0x401u;
  mmio_enable_pending_irq.initial_irq_mask = 0u;
  mmio_enable_pending_irq.initial_irq_pending = true;
  mmio_enable_pending_irq.program = {
      enc_i(0x2B, 1, 2, 4), // SW r2,I_MASK(r1)
      enc_i(0x09, 3, 3, 1), // must not retire before the IRQ
      0u,
  };
  mmio_enable_pending_irq.instructions = 2u;
  mmio_enable_pending_irq.require_v4_native_entry_when_available = true;
  mmio_enable_pending_irq.require_v4_native_store_entry_when_available = true;
  mmio_enable_pending_irq.require_v4_mmio_native_when_available = true;
  mmio_enable_pending_irq
      .require_v4_entry_exception_native_when_available = true;
  cases.push_back(mmio_enable_pending_irq);

  CpuCompareCase mmio_chain_cause_sync{};
  mmio_chain_cause_sync.name =
      "v4_mmio_chain_continues_with_synced_cause_ip2";
  mmio_chain_cause_sync.initial_gpr[1] = 0x1F801070u;
  mmio_chain_cause_sync.initial_gpr[2] = 1u;
  mmio_chain_cause_sync.initial_gpr[6] = 2u;
  // VBlank is pending while SR.IEc=0, so no interrupt is ever taken and the
  // resident chain keeps running across each I_MASK write. Cause.IP2 must
  // still follow the IRQ line at every boundary for the MFC0 reads.
  mmio_chain_cause_sync.initial_cop0_sr_bits = 0x400u;
  mmio_chain_cause_sync.initial_irq_mask = 0u;
  mmio_chain_cause_sync.initial_irq_pending = true;
  mmio_chain_cause_sync.program = {
      enc_i(0x2B, 1, 0, 4),                     // SW r0,I_MASK(r1): line low
      (0x10u << 26) | (4u << 16) | (13u << 11), // MFC0 r4,Cause
      enc_i(0x2B, 1, 2, 4),                     // SW r2,I_MASK(r1): line high
      (0x10u << 26) | (7u << 16) | (13u << 11), // MFC0 r7,Cause
      enc_r(8, 4, 8, 0, 0x21),                  // ADDU r8,r8,r4
      enc_r(9, 7, 9, 0, 0x21),                  // ADDU r9,r9,r7
      enc_i(0x09, 5, 5, 1),                     // ADDIU r5,r5,1
      enc_i(0x05, 5, 6, 0xFFF8u),               // BNE r5,r6,start
      0u,
  };
  pad_cpu_compare_program(mmio_chain_cause_sync, 18u);
  mmio_chain_cause_sync.require_v4_native_entry_when_available = true;
  cases.push_back(mmio_chain_cause_sync);

  // With SR.IsC set, a store hits the I-cache and invalidates the line instead
  // of writing memory (the BIOS FlushCache idiom). Line 1 is warmed, then
  // invalidated by an isolated SW on every pass, so each return to it must
  // refill. Ordinary stores do not touch the I-cache, so native store paths
  // must perform this invalidation on the isolated path themselves.
  CpuCompareCase isolated_store_block{};
  isolated_store_block.name = "v4_isolated_cache_store_invalidates_line";
  isolated_store_block.initial_gpr[1] = kCpuComparePc;
  isolated_store_block.initial_cop0_sr_bits = 1u << 16;
  isolated_store_block.program = {
      enc_j(0x02, kCpuComparePc + 0x10u), // 0x00: J line 1 (warm it)
      0u,                                 // 0x04
      enc_i(0x2B, 1, 0, 0x10),            // 0x08: SW r0,0x10(r1) (isolated)
      0u,                                 // 0x0C
      enc_j(0x02, kCpuComparePc + 0x08u), // 0x10: J 0x08
      0u,                                 // 0x14
  };
  isolated_store_block.instructions = 16u;
  isolated_store_block.require_v4_native_entry_when_available = true;
  cases.push_back(isolated_store_block);

  // Same, with the isolated SW in a branch delay slot (pending-delay store).
  CpuCompareCase isolated_store_delay = isolated_store_block;
  isolated_store_delay.name = "v4_isolated_cache_delay_store_invalidates_line";
  isolated_store_delay.program = {
      enc_j(0x02, kCpuComparePc + 0x10u), // 0x00: J line 1 (warm it)
      0u,                                 // 0x04
      enc_j(0x02, kCpuComparePc + 0x10u), // 0x08: J line 1
      enc_i(0x2B, 1, 0, 0x10),            // 0x0C: SW r0,0x10(r1) (delay)
      enc_j(0x02, kCpuComparePc + 0x08u), // 0x10: J 0x08
      0u,                                 // 0x14
  };
  cases.push_back(isolated_store_delay);

  // A branch head whose delay slot is a load yields before the slot. Once the
  // loop is hot, the precompiled delay fragment runs inside the resident chain.
  CpuCompareCase split_delay_native{};
  split_delay_native.name = "v4_split_branch_load_delay_continues_natively";
  split_delay_native.initial_gpr[1] = 0x80012000u;
  split_delay_native.initial_gpr[6] = 3u;
  split_delay_native.memory = {{0x00012000u, 0x11111111u}};
  split_delay_native.program = {
      enc_i(0x09, 5, 5, 1),        // ADDIU r5,r5,1
      enc_i(0x05, 5, 6, 0xFFFEu),  // BNE r5,r6,start
      enc_i(0x23, 1, 3, 0),        // LW r3,0(r1) (delay slot)
      enc_r(4, 3, 4, 0, 0x21),     // ADDU r4,r4,r3
      0u,
  };
  split_delay_native.instructions = 10u;
  split_delay_native.require_v4_native_entry_when_available = true;
  cases.push_back(split_delay_native);

  // Branch at the last word of an I-cache line with its load delay slot on the
  // next line (line 1). The branch first compiles while line 1 is warm, so its
  // delay cache is filled. Line 1 is then evicted by code 4 KiB away (same
  // index, different tag) whose first word has the SAME bits as the delay
  // slot, so only the tag shows the line is stale. On the second pass the
  // delay fetch must refill before the slot retires. The SIO STAT read that
  // follows as the first instruction on line 1 observes an 8-cycle transfer
  // primed a few instructions earlier; if the refill slipped into that block
  // its MMIO would see a timestamp 4 cycles early and a different STAT.
  const u32 alias_line = kCpuComparePc + 0x1010u;
  CpuCompareCase split_delay_alias{};
  split_delay_alias.name = "v4_split_branch_crossline_delay_alias_refills";
  split_delay_alias.initial_gpr[1] = 0x80012000u;
  split_delay_alias.initial_gpr[5] = 1u;
  split_delay_alias.initial_gpr[10] = 0x1F801000u;
  // Prime the SIO transfer right after the alias LW (instruction 10).
  split_delay_alias.mutations.push_back(
      {10u, 0x80012004u, 0u, false, true});
  split_delay_alias.memory = {
      {0x00012000u, 0x22222222u},
      {alias_line + 0u, enc_i(0x23, 1, 3, 0)},             // same bits as 0x10
      {alias_line + 4u, enc_j(0x02, kCpuComparePc + 0x0Cu)}, // back to branch
      {alias_line + 8u, 0u},
  };
  split_delay_alias.program = {
      enc_j(0x02, kCpuComparePc + 0x1Cu), // 0x00: J 0x1C (warm line 1)
      0u,                                 // 0x04
      0u,                                 // 0x08
      enc_i(0x05, 5, 0, 12),              // 0x0C: BNE r5,r0,+12 -> 0x40
      enc_i(0x23, 1, 3, 0),               // 0x10: LW r3,0(r1) (delay slot)
      enc_i(0x23, 10, 7, 0x44),           // 0x14: LW r7,SIO STAT(r10)
      0u,                                 // 0x18
      enc_j(0x02, kCpuComparePc + 0x0Cu), // 0x1C: J branch
      0u,                                 // 0x20
      0u, 0u, 0u, 0u, 0u, 0u, 0u,         // 0x24..0x3C
      enc_i(0x09, 0, 5, 0),               // 0x40: ADDIU r5,r0,0
      enc_j(0x02, alias_line),            // 0x44: J alias (evicts line 1)
      0u,                                 // 0x48
  };
  split_delay_alias.instructions = 15u;
  split_delay_alias.require_v4_native_entry_when_available = true;
  cases.push_back(split_delay_alias);

  // Same shape, but line 1 is invalidated in place (valid cleared, tag and
  // words unchanged) instead of evicted, so only the valid bit shows the delay
  // fetch must refill. The invalidation targets the alias address: same line
  // index, but a different code page, so the branch's translation survives.
  CpuCompareCase split_delay_invalid = split_delay_alias;
  split_delay_invalid.name =
      "v4_split_branch_crossline_delay_invalidated_refills";
  split_delay_invalid.program[0x44 / 4] =
      enc_j(0x02, kCpuComparePc + 0x0Cu); // 0x44: J branch (no alias trip)
  split_delay_invalid.mutations = {
      {7u, 0x80012004u, 0u, false, true},  // prime SIO after ADDIU @0x40
      {9u, alias_line, 0u, true, false},   // invalidate line 1 via alias
  };
  split_delay_invalid.instructions = 12u;
  cases.push_back(split_delay_invalid);

  // An ordinary SW patches B's RAM word while line B stays cached (stores do
  // not touch the I-cache), so B's next translation is compiled from the old
  // cached bits. An alias then evicts line B; returning to B refills it from
  // RAM and the dispatcher finds the translation stale only after that refill.
  // The refill is B's own fetch, so B must retire in the same slice even
  // though the 3-cycle budget is exhausted, as Cpu::step() does.
  const u32 stale_b = kCpuComparePc + 0x40u;
  CpuCompareCase stale_after_refill{};
  stale_after_refill.name = "v4_stale_after_refill_retires_started_instruction";
  stale_after_refill.initial_gpr[1] = kCpuComparePc;
  stale_after_refill.initial_gpr[2] = enc_i(0x09, 4, 4, 2); // ADDIU r4,r4,2
  stale_after_refill.memory = {
      {(stale_b + 0x1000u) & 0x1FFFFFFFu, enc_j(0x02, stale_b)}, // alias: J B
      {(stale_b + 0x1004u) & 0x1FFFFFFFu, 0u},
  };
  stale_after_refill.program.assign(0x4Cu / 4u, 0u);
  stale_after_refill.program[0x00 / 4] = enc_j(0x03, stale_b);   // JAL B (warm)
  stale_after_refill.program[0x08 / 4] = enc_i(0x2B, 1, 2, 0x40); // SW r2,B
  stale_after_refill.program[0x0C / 4] = enc_j(0x03, stale_b);   // JAL B (old)
  stale_after_refill.program[0x14 / 4] =
      enc_j(0x02, stale_b + 0x1000u);                             // J alias
  stale_after_refill.program[0x40 / 4] = enc_i(0x09, 4, 4, 1);   // B: ADDIU
  stale_after_refill.program[0x44 / 4] = enc_r(31, 0, 0, 0, 0x08); // JR ra
  stale_after_refill.instructions = 18u;
  stale_after_refill.segment_instructions.assign(18u, 1u);
  stale_after_refill.run_slice_cycle_budget = 3u;
  stale_after_refill.compare_segment_states = true;
  stale_after_refill.require_v4_native_entry_when_available = true;
  cases.push_back(stale_after_refill);

  CpuCompareCase native_memory_mid_block_irq{};
  native_memory_mid_block_irq.name = "native_memory_mid_block_irq_state";
  native_memory_mid_block_irq.initial_gpr[1] = 0x1F801070u;
  native_memory_mid_block_irq.initial_gpr[2] = 1u;
  native_memory_mid_block_irq.initial_cop0_sr_bits = 0x401u;
  native_memory_mid_block_irq.initial_irq_pending = true;
  native_memory_mid_block_irq.program = {
      enc_i(0x2B, 1, 2, 4),
      enc_i(0x05, 0, 0, 1),
      0,
  };
  native_memory_mid_block_irq.instructions = 2;
  native_memory_mid_block_irq.allow_partial_native_branch_tail = true;
  native_memory_mid_block_irq.require_native_entry_when_available = true;
  native_memory_mid_block_irq.require_native_memory_helper_when_available =
      true;
  native_memory_mid_block_irq.require_native_mmio_when_available = true;
  cases.push_back(native_memory_mid_block_irq);

  CpuCompareCase native_memory_then_decoded_load{};
  native_memory_then_decoded_load.name =
      "native_memory_then_decoded_consumes_load";
  native_memory_then_decoded_load.initial_gpr[1] = 0x800111A0u;
  native_memory_then_decoded_load.initial_gpr[2] = 0x11111111u;
  native_memory_then_decoded_load.memory.push_back(
      {0x000111A0u, 0x22222222u});
  native_memory_then_decoded_load.program.assign(17u, 0u);
  native_memory_then_decoded_load.program[15] = enc_i(0x23, 1, 2, 0);
  native_memory_then_decoded_load.program[16] =
      enc_r(2, 0, 3, 0, 0x21);
  native_memory_then_decoded_load.instructions = 17;
  native_memory_then_decoded_load.segment_instructions = {16u, 1u};
  native_memory_then_decoded_load.segment_native_tiers = {
      {true, true, true}, {false, true, true}};
  native_memory_then_decoded_load.compare_segment_states = true;
  native_memory_then_decoded_load.enable_ram_load_fastpath_for_x64 = true;
  native_memory_then_decoded_load
      .require_native_ram_load_fastpath_when_available = true;
  native_memory_then_decoded_load
      .require_native_memory_tier_entry_when_available = true;
  cases.push_back(native_memory_then_decoded_load);

  CpuCompareCase decoded_then_native_fast_load{};
  decoded_then_native_fast_load.name =
      "decoded_load_then_coherent_load_rejects_active_delay";
  decoded_then_native_fast_load.initial_gpr[1] = 0x80011420u;
  decoded_then_native_fast_load.initial_gpr[2] = 0x11111111u;
  decoded_then_native_fast_load.initial_gpr[3] = 0x33333333u;
  decoded_then_native_fast_load.memory.push_back(
      {0x00011420u, 0x22222222u});
  decoded_then_native_fast_load.memory.push_back(
      {0x00011424u, 0x44444444u});
  decoded_then_native_fast_load.program.assign(17u, 0u);
  decoded_then_native_fast_load.program[0] = enc_i(0x23, 1, 2, 0);
  decoded_then_native_fast_load.program[1] = enc_i(0x23, 1, 3, 4);
  decoded_then_native_fast_load.program[2] =
      enc_r(2, 0, 4, 0, 0x21);
  decoded_then_native_fast_load.instructions = 17;
  decoded_then_native_fast_load.segment_instructions = {1u, 16u};
  decoded_then_native_fast_load.segment_native_tiers = {
      {false, true, true}, {true, true, true}};
  decoded_then_native_fast_load.compare_segment_states = true;
  decoded_then_native_fast_load.enable_ram_load_fastpath_for_x64 = true;
  // The coherent emitter deliberately rejects a block entered with a live
  // load delay. Spyro reaches this state in normal gameplay, and accepting it
  // before the cross-block hazard is modeled exactly causes timing/state
  // divergence. Keep this as an explicit accuracy gate for the safe fallback.
  decoded_then_native_fast_load.expect_x64_fallback = true;
  cases.push_back(decoded_then_native_fast_load);

  CpuCompareCase decoded_then_native_memory_load{};
  decoded_then_native_memory_load.name =
      "decoded_load_then_native_memory_helper";
  decoded_then_native_memory_load.initial_gpr[1] = 0x800111B0u;
  decoded_then_native_memory_load.initial_gpr[2] = 0x11111111u;
  decoded_then_native_memory_load.memory.push_back(
      {0x000111B0u, 0x22222222u});
  decoded_then_native_memory_load.compare_memory_addresses.push_back(
      0x000111B4u);
  decoded_then_native_memory_load.program.assign(17u, 0u);
  decoded_then_native_memory_load.program[0] = enc_i(0x23, 1, 2, 0);
  decoded_then_native_memory_load.program[1] = enc_i(0x2B, 1, 2, 4);
  decoded_then_native_memory_load.program[3] =
      enc_r(2, 0, 3, 0, 0x21);
  decoded_then_native_memory_load.instructions = 17;
  decoded_then_native_memory_load.segment_instructions = {1u, 16u};
  decoded_then_native_memory_load.segment_native_tiers = {
      {false, true, true}, {true, true, true}};
  decoded_then_native_memory_load.compare_segment_states = true;
  decoded_then_native_memory_load.require_native_memory_helper_when_available =
      true;
  decoded_then_native_memory_load
      .require_native_memory_tier_entry_when_available = true;
  cases.push_back(decoded_then_native_memory_load);

  CpuCompareCase native_alu_then_memory{};
  native_alu_then_memory.name = "native_alu_then_native_memory_block";
  native_alu_then_memory.initial_gpr[1] = 0x800111C0u;
  native_alu_then_memory.program.assign(32u, 0u);
  native_alu_then_memory.program[0] = enc_i(0x09, 0, 2, 5);
  native_alu_then_memory.program[16] = enc_i(0x2B, 1, 2, 0);
  native_alu_then_memory.compare_memory_addresses.push_back(0x000111C0u);
  native_alu_then_memory.instructions = 32;
  native_alu_then_memory.segment_instructions = {16u, 16u};
  native_alu_then_memory.compare_segment_states = true;
  native_alu_then_memory.require_native_memory_helper_when_available = true;
  native_alu_then_memory.require_native_memory_tier_entry_when_available =
      true;
  native_alu_then_memory.require_native_alu_tier_entry_when_available = true;
  cases.push_back(native_alu_then_memory);

  CpuCompareCase native_memory_then_alu{};
  native_memory_then_alu.name = "native_memory_then_native_alu_block";
  native_memory_then_alu.initial_gpr[1] = 0x800111D0u;
  native_memory_then_alu.initial_gpr[2] = 0xA5A5A5A5u;
  native_memory_then_alu.program.assign(32u, 0u);
  native_memory_then_alu.program[0] = enc_i(0x2B, 1, 2, 0);
  native_memory_then_alu.program[16] = enc_i(0x09, 0, 3, 7);
  native_memory_then_alu.compare_memory_addresses.push_back(0x000111D0u);
  native_memory_then_alu.instructions = 32;
  native_memory_then_alu.segment_instructions = {16u, 16u};
  native_memory_then_alu.compare_segment_states = true;
  native_memory_then_alu.require_native_memory_helper_when_available = true;
  native_memory_then_alu.require_native_memory_tier_entry_when_available =
      true;
  native_memory_then_alu.require_native_alu_tier_entry_when_available = true;
  cases.push_back(native_memory_then_alu);

  CpuCompareCase jal{};
  jal.name = "jal_link_delay";
  jal.program = {
      enc_j(0x03, kCpuComparePc + 0x10u),
      enc_i(0x09, 0, 5, 0x0055),
      enc_i(0x09, 0, 6, 0x0066),
      0,
      enc_r(31, 0, 7, 0, 0x21),
      enc_i(0x09, 0, 8, 0x0088),
  };
  jal.instructions = 4;
  cases.push_back(jal);

  CpuCompareCase jalr{};
  jalr.name = "jalr_link_delay";
  jalr.initial_gpr[8] = kCpuComparePc + 0x10u;
  jalr.program = {
      enc_r(8, 0, 9, 0, 0x09),
      enc_i(0x09, 0, 5, 0x0055),
      enc_i(0x09, 0, 6, 0x0066),
      0,
      enc_r(9, 0, 10, 0, 0x21),
      enc_i(0x09, 0, 11, 0x0077),
  };
  jalr.instructions = 4;
  cases.push_back(jalr);

  CpuCompareCase load_delay{};
  load_delay.name = "load_delay_lw";
  load_delay.initial_gpr[2] = 0x11111111u;
  load_delay.program = {
      enc_i(0x0F, 0, 1, 0x8001),
      enc_i(0x23, 1, 2, 0x1000),
      enc_r(2, 0, 3, 0, 0x21),
      enc_r(2, 0, 4, 0, 0x21),
  };
  load_delay.memory.push_back({0x00011000u, 0x12345678u});
  load_delay.require_full_native_when_available = true;
  load_delay.require_native_memory_helper_when_available = true;
  load_delay.require_v4_native_entry_when_available = true;
  load_delay.require_v4_native_load_entry_when_available = true;
  pad_cpu_compare_program(load_delay);
  cases.push_back(load_delay);

  CpuCompareCase ram_fast_load_delay = load_delay;
  ram_fast_load_delay.name = "native_ram_fast_load_delay";
  ram_fast_load_delay.enable_ram_load_fastpath_for_x64 = true;
  ram_fast_load_delay.require_native_memory_helper_when_available = false;
  ram_fast_load_delay.require_native_ram_load_fastpath_when_available = true;
  cases.push_back(ram_fast_load_delay);

  CpuCompareCase load_entry_alu{};
  load_entry_alu.name = "native_load_delay_entry_alu_then_memory";
  load_entry_alu.initial_gpr[1] = 0x80011080u;
  load_entry_alu.initial_gpr[2] = 0x11111111u;
  load_entry_alu.program = {
      enc_i(0x23, 1, 2, 0),
      enc_r(2, 0, 3, 0, 0x21),
      enc_i(0x2B, 1, 3, 4),
      enc_i(0x23, 1, 4, 4),
      0,
  };
  load_entry_alu.memory.push_back({0x00011080u, 0x22222222u});
  load_entry_alu.segment_instructions = {1u, 16u};
  load_entry_alu.require_native_memory_helper_when_available = true;
  load_entry_alu.require_native_helper_load_delay_entry_when_available = true;
  pad_cpu_compare_program(load_entry_alu, 17u);
  cases.push_back(load_entry_alu);

  CpuCompareCase load_entry_memory{};
  load_entry_memory.name = "native_load_delay_entry_memory_then_memory";
  load_entry_memory.initial_gpr[1] = 0x80011090u;
  load_entry_memory.initial_gpr[2] = 0x11111111u;
  load_entry_memory.program = {
      enc_i(0x23, 1, 2, 0),
      enc_i(0x2B, 1, 2, 4),
      enc_i(0x23, 1, 3, 4),
      0,
  };
  load_entry_memory.memory.push_back({0x00011090u, 0x22222222u});
  load_entry_memory.segment_instructions = {1u, 16u};
  load_entry_memory.require_native_memory_helper_when_available = true;
  load_entry_memory.require_native_helper_load_delay_entry_when_available =
      true;
  pad_cpu_compare_program(load_entry_memory, 17u);
  cases.push_back(load_entry_memory);

  CpuCompareCase memory_load_store{};
  memory_load_store.name = "native_memory_load_store";
  memory_load_store.initial_gpr[1] = 0x80011020u;
  memory_load_store.initial_gpr[2] = 0xCAFEBABEu;
  memory_load_store.program = {
      enc_i(0x2B, 1, 2, 0),
      enc_i(0x23, 1, 3, 0),
      0,
  };
  memory_load_store.require_full_native_when_available = true;
  memory_load_store.require_native_memory_helper_when_available = true;
  pad_cpu_compare_program(memory_load_store);
  cases.push_back(memory_load_store);

  CpuCompareCase sign_loads{};
  sign_loads.name = "native_memory_sign_zero_loads";
  sign_loads.initial_gpr[1] = 0x80011030u;
  sign_loads.program = {
      enc_i(0x20, 1, 2, 2),
      enc_i(0x24, 1, 3, 3),
      enc_i(0x21, 1, 4, 2),
      enc_i(0x25, 1, 5, 0),
      0,
  };
  sign_loads.memory.push_back({0x00011030u, 0x80FF7F01u});
  sign_loads.require_full_native_when_available = true;
  sign_loads.require_native_memory_helper_when_available = true;
  pad_cpu_compare_program(sign_loads);
  cases.push_back(sign_loads);

  CpuCompareCase ram_fast_load_widths{};
  ram_fast_load_widths.name = "native_ram_fast_load_widths_sign_zero";
  ram_fast_load_widths.initial_gpr[1] = 0x80011400u;
  ram_fast_load_widths.memory.push_back({0x00011400u, 0x8001FF80u});
  ram_fast_load_widths.program = {
      enc_i(0x20, 1, 2, 0),
      0,
      enc_i(0x24, 1, 3, 1),
      0,
      enc_i(0x21, 1, 4, 0),
      0,
      enc_i(0x25, 1, 5, 2),
      0,
      enc_i(0x23, 1, 6, 0),
      0,
  };
  ram_fast_load_widths.enable_ram_load_fastpath_for_x64 = true;
  ram_fast_load_widths.require_native_ram_load_fastpath_when_available = true;
  ram_fast_load_widths.require_full_native_when_available = true;
  pad_cpu_compare_program(ram_fast_load_widths);
  cases.push_back(ram_fast_load_widths);

  CpuCompareCase reduced_ram_load_widths{};
  reduced_ram_load_widths.name =
      "reduced_helper_ram_load_widths_sign_zero";
  reduced_ram_load_widths.initial_gpr[1] = 0x80011500u;
  reduced_ram_load_widths.memory.push_back({0x00011500u, 0x8001FF80u});
  reduced_ram_load_widths.program = {
      enc_i(0x20, 1, 2, 0), 0,
      enc_i(0x24, 1, 3, 1), 0,
      enc_i(0x21, 1, 4, 0), 0,
      enc_i(0x25, 1, 5, 2), 0,
      enc_i(0x23, 1, 6, 0), 0,
  };
  reduced_ram_load_widths.enable_ram_load_fastpath_for_x64 = true;
  reduced_ram_load_widths.require_full_native_when_available = true;
  pad_cpu_compare_program(reduced_ram_load_widths);
  cases.push_back(reduced_ram_load_widths);

  CpuCompareCase reduced_ram_load_boundary{};
  reduced_ram_load_boundary.name =
      "decoded_load_then_reduced_helper_ram_load";
  reduced_ram_load_boundary.initial_gpr[1] = 0x80011520u;
  reduced_ram_load_boundary.initial_gpr[2] = 0x11111111u;
  reduced_ram_load_boundary.initial_gpr[3] = 0x22222222u;
  reduced_ram_load_boundary.memory.push_back(
      {0x00011520u, 0xA1B2C3D4u});
  reduced_ram_load_boundary.memory.push_back(
      {0x00011524u, 0x55667788u});
  reduced_ram_load_boundary.program = {
      enc_i(0x23, 1, 2, 0),
      enc_i(0x23, 1, 3, 4),
      enc_r(2, 0, 4, 0, 0x21),
      enc_r(3, 0, 5, 0, 0x21),
  };
  pad_cpu_compare_program(reduced_ram_load_boundary, 17u);
  reduced_ram_load_boundary.segment_instructions = {1u, 16u};
  reduced_ram_load_boundary.segment_native_tiers = {
      {false, true, true}, {true, true, true}};
  reduced_ram_load_boundary.compare_segment_states = true;
  reduced_ram_load_boundary.enable_ram_load_fastpath_for_x64 = true;
  cases.push_back(reduced_ram_load_boundary);

  CpuCompareCase reduced_ram_load_cancel{};
  reduced_ram_load_cancel.name =
      "reduced_helper_ram_load_same_register_cancel";
  reduced_ram_load_cancel.initial_gpr[1] = 0x80011540u;
  reduced_ram_load_cancel.initial_gpr[2] = 0x11111111u;
  reduced_ram_load_cancel.memory.push_back(
      {0x00011540u, 0xDEADBEEFu});
  reduced_ram_load_cancel.program = {
      enc_i(0x23, 1, 2, 0),
      enc_i(0x09, 0, 2, 7),
      enc_r(2, 0, 3, 0, 0x21),
  };
  reduced_ram_load_cancel.enable_ram_load_fastpath_for_x64 = true;
  reduced_ram_load_cancel.require_full_native_when_available = true;
  pad_cpu_compare_program(reduced_ram_load_cancel);
  cases.push_back(reduced_ram_load_cancel);

  CpuCompareCase reduced_ram_load_mmio_reject{};
  reduced_ram_load_mmio_reject.name =
      "reduced_helper_ram_load_mmio_rejected";
  reduced_ram_load_mmio_reject.initial_gpr[1] = 0x1F801800u;
  reduced_ram_load_mmio_reject.program = {
      enc_i(0x24, 1, 2, 0), 0,
  };
  reduced_ram_load_mmio_reject.enable_ram_load_fastpath_for_x64 = true;
  reduced_ram_load_mmio_reject.require_native_entry_when_available = true;
  reduced_ram_load_mmio_reject
      .require_native_memory_helper_when_available = true;
  reduced_ram_load_mmio_reject.require_native_mmio_when_available = true;
  reduced_ram_load_mmio_reject
      .require_no_native_reduced_helper_ram_load_entry = true;
  reduced_ram_load_mmio_reject.require_no_native_ram_load_fastpath = true;
  pad_cpu_compare_program(reduced_ram_load_mmio_reject);
  cases.push_back(reduced_ram_load_mmio_reject);

  CpuCompareCase reduced_ram_load_scratchpad_reject{};
  reduced_ram_load_scratchpad_reject.name =
      "reduced_helper_ram_load_scratchpad_rejected";
  reduced_ram_load_scratchpad_reject.initial_gpr[1] = 0x1F800000u;
  reduced_ram_load_scratchpad_reject.memory.push_back(
      {0x1F800000u, 0x55667788u});
  reduced_ram_load_scratchpad_reject.program = {
      enc_i(0x23, 1, 2, 0), 0,
  };
  reduced_ram_load_scratchpad_reject.enable_ram_load_fastpath_for_x64 =
      true;
  reduced_ram_load_scratchpad_reject
      .require_native_entry_when_available = true;
  reduced_ram_load_scratchpad_reject
      .require_native_memory_helper_when_available = true;
  reduced_ram_load_scratchpad_reject
      .require_no_native_reduced_helper_ram_load_entry = true;
  reduced_ram_load_scratchpad_reject.require_no_native_ram_load_fastpath =
      true;
  pad_cpu_compare_program(reduced_ram_load_scratchpad_reject);
  cases.push_back(reduced_ram_load_scratchpad_reject);

  CpuCompareCase reduced_ram_load_unaligned_reject{};
  reduced_ram_load_unaligned_reject.name =
      "reduced_helper_ram_load_unaligned_rejected";
  reduced_ram_load_unaligned_reject.initial_gpr[1] = 0x80011562u;
  reduced_ram_load_unaligned_reject.program = {
      enc_i(0x23, 1, 2, 0),
      enc_i(0x09, 0, 3, 3),
  };
  reduced_ram_load_unaligned_reject.enable_ram_load_fastpath_for_x64 = true;
  reduced_ram_load_unaligned_reject.require_native_entry_when_available =
      true;
  reduced_ram_load_unaligned_reject
      .require_native_memory_exception_when_available = true;
  reduced_ram_load_unaligned_reject
      .require_no_native_reduced_helper_ram_load_entry = true;
  reduced_ram_load_unaligned_reject.require_no_native_ram_load_fastpath =
      true;
  pad_cpu_compare_program(reduced_ram_load_unaligned_reject);
  cases.push_back(reduced_ram_load_unaligned_reject);

  CpuCompareCase stores{};
  stores.name = "native_memory_byte_half_word_stores";
  stores.initial_gpr[1] = 0x80011040u;
  stores.initial_gpr[2] = 0xCAFEBABEu;
  stores.program = {
      enc_i(0x2B, 1, 2, 0),
      enc_i(0x28, 1, 2, 4),
      enc_i(0x29, 1, 2, 6),
      enc_i(0x23, 1, 3, 0),
      enc_i(0x24, 1, 4, 4),
      enc_i(0x25, 1, 5, 6),
      0,
  };
  stores.require_full_native_when_available = true;
  stores.require_native_memory_helper_when_available = true;
  pad_cpu_compare_program(stores);
  cases.push_back(stores);

  CpuCompareCase mixed_memory_alu{};
  mixed_memory_alu.name = "native_memory_mixed_alu_load_delay";
  mixed_memory_alu.initial_gpr[1] = 0x80011060u;
  mixed_memory_alu.program = {
      enc_i(0x09, 0, 2, 1),
      enc_i(0x23, 1, 3, 0),
      enc_r(3, 2, 4, 0, 0x21),
      enc_r(3, 2, 5, 0, 0x21),
      enc_i(0x2B, 1, 5, 4),
      enc_i(0x23, 1, 6, 4),
      0,
      enc_r(6, 2, 7, 0, 0x21),
  };
  mixed_memory_alu.memory.push_back({0x00011060u, 5u});
  mixed_memory_alu.require_full_native_when_available = true;
  mixed_memory_alu.require_native_memory_helper_when_available = true;
  pad_cpu_compare_program(mixed_memory_alu);
  cases.push_back(mixed_memory_alu);

  CpuCompareCase memory_load_alu_same_reg_store{};
  memory_load_alu_same_reg_store.name =
      "native_memory_load_alu_same_reg_cancels_delay";
  memory_load_alu_same_reg_store.initial_gpr[1] = 0x80011220u;
  memory_load_alu_same_reg_store.initial_gpr[2] = 0x11111111u;
  memory_load_alu_same_reg_store.memory.push_back(
      {0x00011220u, 0xA5A5A5A5u});
  memory_load_alu_same_reg_store.compare_memory_addresses.push_back(
      0x00011224u);
  memory_load_alu_same_reg_store.program = {
      enc_i(0x23, 1, 2, 0),
      enc_i(0x09, 0, 2, 7),
      enc_i(0x2B, 1, 2, 4),
  };
  pad_cpu_compare_program(memory_load_alu_same_reg_store);
  memory_load_alu_same_reg_store.instructions = 3u;
  memory_load_alu_same_reg_store.require_native_entry_when_available = true;
  memory_load_alu_same_reg_store
      .require_native_memory_helper_when_available = true;
  memory_load_alu_same_reg_store
      .require_native_memory_tier_entry_when_available = true;
  memory_load_alu_same_reg_store.enable_ram_load_fastpath_for_x64 = true;
  memory_load_alu_same_reg_store
      .require_native_ram_load_fastpath_when_available = true;
  memory_load_alu_same_reg_store.segment_instructions.assign(3u, 1u);
  memory_load_alu_same_reg_store.compare_segment_states = true;
  memory_load_alu_same_reg_store.allow_partial_native_memory_helper = true;
  cases.push_back(memory_load_alu_same_reg_store);

  CpuCompareCase mmio_helper{};
  mmio_helper.name = "native_mmio_safe_helper_store";
  mmio_helper.initial_gpr[1] = 0x1F801080u;
  mmio_helper.initial_gpr[2] = 0u;
  mmio_helper.program = {
      enc_i(0x2B, 1, 2, 0),
      0,
  };
  mmio_helper.require_full_native_when_available = true;
  mmio_helper.require_native_memory_helper_when_available = true;
  mmio_helper.require_v4_native_entry_when_available = true;
  mmio_helper.require_v4_native_store_entry_when_available = true;
  mmio_helper.require_v4_mmio_native_when_available = true;
  pad_cpu_compare_program(mmio_helper);
  cases.push_back(mmio_helper);

  CpuCompareCase cdrom_status_helper{};
  cdrom_status_helper.name = "native_memory_cdrom_status_read";
  cdrom_status_helper.initial_gpr[1] = 0x1F801800u;
  cdrom_status_helper.program = {
      enc_i(0x24, 1, 2, 0),
      0,
      enc_r(2, 0, 3, 0, 0x21),
  };
  cdrom_status_helper.require_full_native_when_available = true;
  cdrom_status_helper.require_native_memory_helper_when_available = true;
  cdrom_status_helper.require_native_mmio_when_available = true;
  cdrom_status_helper.require_no_native_ram_load_fastpath = true;
  cdrom_status_helper.require_v4_native_entry_when_available = true;
  cdrom_status_helper.require_v4_native_load_entry_when_available = true;
  cdrom_status_helper.require_v4_mmio_native_when_available = true;
  pad_cpu_compare_program(cdrom_status_helper);
  cases.push_back(cdrom_status_helper);

  CpuCompareCase expansion_bus_load{};
  expansion_bus_load.name = "v4_expansion_bus_load_native";
  expansion_bus_load.initial_gpr[1] = 0x1F000084u;
  expansion_bus_load.program = {
      enc_i(0x20, 1, 2, 0), // LB from BIOS expansion/device space
      0,                    // retire the load delay
  };
  expansion_bus_load.require_full_native_when_available = true;
  expansion_bus_load.require_native_memory_helper_when_available = true;
  expansion_bus_load.require_native_mmio_when_available = true;
  expansion_bus_load.require_no_native_ram_load_fastpath = true;
  expansion_bus_load.require_v4_native_entry_when_available = true;
  expansion_bus_load.require_v4_native_load_entry_when_available = true;
  expansion_bus_load.require_v4_mmio_native_when_available = true;
  pad_cpu_compare_program(expansion_bus_load);
  cases.push_back(expansion_bus_load);

  CpuCompareCase joy_status_dynamic_lhu{};
  joy_status_dynamic_lhu.name = "v4_joy_status_dynamic_lhu_native";
  joy_status_dynamic_lhu.initial_gpr[17] = 0x1F801040u;
  joy_status_dynamic_lhu.program = {
      enc_i(0x25, 17, 13, 4), // LHU r13, JOY_STAT via dynamic MMIO address
      0,                       // retire the load delay
  };
  joy_status_dynamic_lhu.require_full_native_when_available = true;
  joy_status_dynamic_lhu.require_native_memory_helper_when_available = true;
  joy_status_dynamic_lhu.require_native_mmio_when_available = true;
  joy_status_dynamic_lhu.require_no_native_ram_load_fastpath = true;
  joy_status_dynamic_lhu.require_v4_native_entry_when_available = true;
  joy_status_dynamic_lhu.require_v4_native_load_entry_when_available = true;
  joy_status_dynamic_lhu.require_v4_mmio_native_when_available = true;
  pad_cpu_compare_program(joy_status_dynamic_lhu);
  cases.push_back(joy_status_dynamic_lhu);

  CpuCompareCase scratchpad_slow_load{};
  scratchpad_slow_load.name = "native_scratchpad_load_stays_helper";
  scratchpad_slow_load.initial_gpr[1] = 0x1F800000u;
  scratchpad_slow_load.memory.push_back({0x1F800000u, 0x55667788u});
  scratchpad_slow_load.program = {
      enc_i(0x23, 1, 2, 0),
      0,
  };
  scratchpad_slow_load.require_no_native_ram_load_fastpath = true;
  scratchpad_slow_load.require_native_memory_helper_when_available = true;
  scratchpad_slow_load.require_full_native_when_available = true;
  pad_cpu_compare_program(scratchpad_slow_load);
  cases.push_back(scratchpad_slow_load);

  CpuCompareCase dma_status_helper{};
  dma_status_helper.name = "native_memory_dma_status_read_write";
  dma_status_helper.initial_gpr[1] = 0x1F801080u;
  dma_status_helper.program = {
      enc_i(0x23, 1, 2, 0x70),
      0,
      enc_i(0x2B, 1, 2, 0x70),
  };
  dma_status_helper.require_full_native_when_available = true;
  dma_status_helper.require_native_memory_helper_when_available = true;
  dma_status_helper.require_native_mmio_when_available = true;
  dma_status_helper.require_v4_native_entry_when_available = true;
  dma_status_helper.require_v4_native_load_entry_when_available = true;
  dma_status_helper.require_v4_native_store_entry_when_available = true;
  dma_status_helper.require_v4_mmio_native_when_available = true;
  pad_cpu_compare_program(dma_status_helper);
  cases.push_back(dma_status_helper);

  CpuCompareCase syscall_exception{};
  syscall_exception.name = "exception_syscall";
  syscall_exception.program = {
      enc_i(0x09, 0, 1, 5),
      0x0000000Cu,
      enc_i(0x09, 0, 2, 6),
  };
  syscall_exception.instructions = 2;
  syscall_exception.require_full_native_when_available = true;
  syscall_exception.require_v4_native_entry_when_available = true;
  syscall_exception.require_v4_exception_native_when_available = true;
  cases.push_back(syscall_exception);

  CpuCompareCase break_exception{};
  break_exception.name = "exception_break";
  break_exception.program = {
      enc_i(0x09, 0, 1, 5),
      0x0000000Du,
      enc_i(0x09, 0, 2, 6),
  };
  break_exception.instructions = 2;
  break_exception.require_full_native_when_available = true;
  break_exception.require_v4_native_entry_when_available = true;
  break_exception.require_v4_exception_native_when_available = true;
  cases.push_back(break_exception);

  CpuCompareCase unaligned_pc_native{};
  unaligned_pc_native.name = "v4_unaligned_pc_exception_native";
  unaligned_pc_native.start_pc = 0xA0010001u;
  unaligned_pc_native.instructions = 1u;
  unaligned_pc_native.require_v4_entry_exception_native_when_available = true;
  cases.push_back(unaligned_pc_native);

  CpuCompareCase unaligned_lw{};
  unaligned_lw.name = "exception_unaligned_lw";
  unaligned_lw.initial_gpr[1] = 0x80011002u;
  unaligned_lw.program = {
      enc_i(0x23, 1, 2, 0),
      enc_i(0x09, 0, 3, 3),
  };
  unaligned_lw.require_native_entry_when_available = true;
  unaligned_lw.require_native_memory_helper_when_available = true;
  unaligned_lw.require_native_memory_exception_when_available = true;
  unaligned_lw.require_no_native_ram_load_fastpath = true;
  pad_cpu_compare_program(unaligned_lw);
  cases.push_back(unaligned_lw);

  CpuCompareCase unaligned_lh{};
  unaligned_lh.name = "exception_unaligned_lh";
  unaligned_lh.initial_gpr[1] = 0x80011001u;
  unaligned_lh.program = {
      enc_i(0x21, 1, 2, 0),
      enc_i(0x09, 0, 3, 3),
  };
  unaligned_lh.require_native_entry_when_available = true;
  unaligned_lh.require_native_memory_helper_when_available = true;
  unaligned_lh.require_native_memory_exception_when_available = true;
  unaligned_lh.require_no_native_ram_load_fastpath = true;
  pad_cpu_compare_program(unaligned_lh);
  cases.push_back(unaligned_lh);

  CpuCompareCase unaligned_sw{};
  unaligned_sw.name = "exception_unaligned_sw";
  unaligned_sw.initial_gpr[1] = 0x80011002u;
  unaligned_sw.initial_gpr[2] = 0x12345678u;
  unaligned_sw.program = {
      enc_i(0x2B, 1, 2, 0),
      enc_i(0x09, 0, 3, 3),
  };
  unaligned_sw.require_native_entry_when_available = true;
  unaligned_sw.require_native_memory_helper_when_available = true;
  unaligned_sw.require_native_memory_exception_when_available = true;
  pad_cpu_compare_program(unaligned_sw);
  cases.push_back(unaligned_sw);

  CpuCompareCase unaligned_sh{};
  unaligned_sh.name = "exception_unaligned_sh";
  unaligned_sh.initial_gpr[1] = 0x80011001u;
  unaligned_sh.initial_gpr[2] = 0x12345678u;
  unaligned_sh.program = {
      enc_i(0x29, 1, 2, 0),
      enc_i(0x09, 0, 3, 3),
  };
  unaligned_sh.require_native_entry_when_available = true;
  unaligned_sh.require_native_memory_helper_when_available = true;
  unaligned_sh.require_native_memory_exception_when_available = true;
  pad_cpu_compare_program(unaligned_sh);
  cases.push_back(unaligned_sh);

  CpuCompareCase cop0{};
  cop0.name = "unsafe_cop0_fallback";
  cop0.program = {
      (0x10u << 26) | (0u << 21) | (2u << 16) | (12u << 11),
      0,
  };
  cop0.instructions = 2;
  cop0.require_full_native_when_available = true;
  cases.push_back(cop0);

  CpuCompareCase cop2{};
  cop2.name = "unsafe_cop2_gte_fallback";
  cop2.program = {
      (0x12u << 26) | (0u << 21) | (2u << 16) | (0u << 11),
      0,
  };
  cop2.instructions = 2;
  cop2.require_full_native_when_available = true;
  cop2.require_v2_helper_entry_when_available = true;
  cases.push_back(cop2);

  CpuCompareCase unsupported_strict{};
  unsupported_strict.name = "unsafe_unsupported_opcode_exception";
  unsupported_strict.program = {
      0xFC000000u,
      enc_i(0x09, 0, 2, 2),
  };
  unsupported_strict.instructions = 1;
  unsupported_strict.expect_x64_fallback = true;
  cases.push_back(unsupported_strict);

  CpuCompareCase unknown_primary{};
  unknown_primary.name = "unknown_primary_fallback_nop";
  unknown_primary.program = {
      0xFC000000u,
      enc_i(0x09, 0, 2, 2),
  };
  unknown_primary.instructions = 2;
  unknown_primary.experimental_unknown_fallback = true;
  unknown_primary.expect_x64_fallback = true;
  cases.push_back(unknown_primary);

  CpuCompareCase unknown_special{};
  unknown_special.name = "unknown_special_fallback_rd_zero";
  unknown_special.initial_gpr[5] = 0x12345678u;
  unknown_special.program = {
      enc_r(0, 0, 5, 0, 0x3F),
      enc_i(0x09, 0, 6, 6),
  };
  unknown_special.instructions = 2;
  unknown_special.experimental_unknown_fallback = true;
  unknown_special.expect_x64_fallback = true;
  cases.push_back(unknown_special);

  CpuCompareCase ram_invalidation{};
  ram_invalidation.name = "ram_code_invalidation";
  ram_invalidation.program = {
      enc_i(0x09, 0, 1, 1),
      enc_i(0x09, 0, 2, 2),
      enc_i(0x09, 0, 3, 3),
  };
  ram_invalidation.mutations.push_back(
      {1u, kCpuComparePc + 4u, enc_i(0x09, 0, 2, 0x0022)});
  ram_invalidation.instructions = 3;
  cases.push_back(ram_invalidation);

  CpuCompareCase v4_page_local_invalidation{};
  v4_page_local_invalidation.name = "v4_page_local_code_invalidation";
  v4_page_local_invalidation.start_pc = 0xA0010000u;
  v4_page_local_invalidation.program = {
      enc_i(0x09, 1, 1, 1),
      enc_j(0x02, v4_page_local_invalidation.start_pc),
      0u,
  };
  v4_page_local_invalidation.mutations.push_back(
      {3u, v4_page_local_invalidation.start_pc,
       enc_i(0x09, 1, 1, 2)});
  v4_page_local_invalidation.instructions = 6u;
  v4_page_local_invalidation.require_v4_native_entry_when_available = true;
  v4_page_local_invalidation
      .require_v4_page_local_invalidation_when_available = true;
  cases.push_back(v4_page_local_invalidation);

  CpuCompareCase v4_cached_same_page_retention{};
  v4_cached_same_page_retention.name =
      "v4_cached_same_page_unrelated_write_retains_block";
  v4_cached_same_page_retention.start_pc = 0x80010000u;
  v4_cached_same_page_retention.program = {
      enc_i(0x09, 1, 1, 1),
      enc_j(0x02, v4_cached_same_page_retention.start_pc),
      0u,
  };
  // Touch another 16-byte I-cache line on the same 4 KiB physical page.
  // The cached loop's exact guest I-cache snapshot remains valid.
  v4_cached_same_page_retention.mutations.push_back(
      {3u, v4_cached_same_page_retention.start_pc + 0x40u, 0xDEADBEEFu});
  v4_cached_same_page_retention.instructions = 6u;
  v4_cached_same_page_retention.require_v4_native_entry_when_available = true;
  v4_cached_same_page_retention
      .require_v4_cached_same_page_retention_when_available = true;
  cases.push_back(v4_cached_same_page_retention);

  CpuCompareCase v4_cached_icache_revalidation{};
  v4_cached_icache_revalidation.name =
      "v4_cached_icache_alias_revalidates_without_recompile";
  v4_cached_icache_revalidation.start_pc = 0x80010000u;
  v4_cached_icache_revalidation.program = {
      enc_i(0x09, 1, 1, 1),
      enc_j(0x02, v4_cached_icache_revalidation.start_pc),
      0u,
  };
  // +0x1000 maps to the same direct-mapped I-cache index but is a different
  // physical code page. It invalidates/refills the guest line without changing
  // the loop's instruction words.
  v4_cached_icache_revalidation.mutations.push_back(
      {3u, v4_cached_icache_revalidation.start_pc + 0x1000u, 0xDEADBEEFu});
  v4_cached_icache_revalidation.instructions = 6u;
  v4_cached_icache_revalidation.require_v4_native_entry_when_available = true;
  v4_cached_icache_revalidation.require_v4_icache_revalidation_when_available =
      true;
  cases.push_back(v4_cached_icache_revalidation);

  // Found by the Grim Reaper hardware simulator: corrupted code can contain encodings real
  // programs never use. REGIMM with rt = 0x1F (BGEZ with the unused "likely" and "link" bits
  // set) feeding a SWC0 in its delay slot made the backends differ by one or two cycles per pass.
  {
    CpuCompareCase odd_regimm{};
    odd_regimm.name = "odd_regimm_rt1f_branch_to_self_with_swc0_delay_slot";
    odd_regimm.start_pc = 0x80010000u;
    odd_regimm.program = {0x06FFFFFFu, 0xE3000000u, 0u};
    odd_regimm.instructions = 24u;
    cases.push_back(odd_regimm);

    CpuCompareCase odd_branch_nop{};
    odd_branch_nop.name = "odd_regimm_rt1f_branch_to_self_with_nop_delay_slot";
    odd_branch_nop.start_pc = 0x80010000u;
    odd_branch_nop.program = {0x06FFFFFFu, 0u, 0u};
    odd_branch_nop.instructions = 24u;
    cases.push_back(odd_branch_nop);

    for (u32 variant = 0; variant < 6; ++variant) {
      static const char *const kNames[] = {"swc0_first_cop0_reg12", "sw_first_reg12", "swc0_first_cop0_reg0",
                                           "sw_first_reg0", "swc0_first_via_r24_cop0_reg0",
                                           "sw_first_via_r24_reg0"};
      CpuCompareCase c{};
      c.name = kNames[variant];
      c.start_pc = 0x80010000u;
      static const u32 kStores[] = {0xE00C0000u, 0xAC0C0000u, 0xE0000000u, 0xAC000000u, 0xE3000000u, 0xAF000000u};
      c.program = {kStores[variant], enc_i(0x09, 0, 2, 1), 0u};
      c.instructions = 2u;
      cases.push_back(c);
    }

    // SWC0 of a plain COP0 register: the stored value and the address must both survive the
    // register read (the native register read once clobbered the address in RAX).
    CpuCompareCase swc0_value{};
    swc0_value.name = "swc0_stores_cop0_register_value_to_the_right_address";
    swc0_value.start_pc = 0x80010000u;
    swc0_value.program = {enc_i(0x0F, 0, 1, 0xBEEF), enc_i(0x0D, 1, 1, 0x1234), 0x40811800u /* mtc0 r1,$3 */,
                          0u, 0xE0030100u /* swc0 $3,0x100(r0) */, 0u};
    swc0_value.instructions = 6u;
    swc0_value.compare_memory_addresses = {0x100u, 0x80000100u};
    cases.push_back(swc0_value);

    CpuCompareCase swc0_alone{};
    swc0_alone.name = "swc0_straight_line";
    swc0_alone.start_pc = 0x80010000u;
    swc0_alone.program = {0xE3000000u, enc_i(0x09, 0, 1, 1), 0u};
    swc0_alone.instructions = 2u;
    cases.push_back(swc0_alone);
  }

  // Code changed in RAM behind the CPU (no store, no cache flush): a cached line keeps its
  // stale words until it is refilled, and both backends must keep executing exactly that.
  // Used by the Grim Reaper hardware simulator and by DMA into live code.
  {
    CpuCompareCase stale_cached_loop{};
    stale_cached_loop.name = "ram_code_change_behind_cpu_cached_line_stays_stale";
    stale_cached_loop.start_pc = 0x80010000u;
    stale_cached_loop.program = {
        enc_i(0x09, 1, 1, 1),
        enc_j(0x02, stale_cached_loop.start_pc),
        0u,
    };
    stale_cached_loop.mutations.push_back(
        {3u, stale_cached_loop.start_pc, enc_i(0x09, 1, 1, 5), false});
    stale_cached_loop.instructions = 12u;
    cases.push_back(stale_cached_loop);

    // The two blocks alias in the I-cache, so every pass refills from memory and the
    // changed word takes effect at the next refill.
    CpuCompareCase refill_after_change{};
    refill_after_change.name = "ram_code_change_behind_cpu_visible_after_refill";
    refill_after_change.start_pc = 0x80010000u;
    refill_after_change.program = {
        enc_i(0x09, 1, 1, 1),
        enc_j(0x02, 0x80011000u),
        0u,
    };
    refill_after_change.memory = {
        {0x80011000u, enc_i(0x09, 2, 2, 1)},
        {0x80011004u, enc_j(0x02, refill_after_change.start_pc)},
        {0x80011008u, 0u},
    };
    refill_after_change.mutations.push_back(
        {7u, refill_after_change.start_pc, enc_i(0x09, 1, 1, 9), false});
    refill_after_change.instructions = 40u;
    cases.push_back(refill_after_change);

    // A line that was never fetched: the changed word is simply what memory holds.
    CpuCompareCase uncached_change{};
    uncached_change.name = "ram_code_change_behind_cpu_before_first_fetch";
    uncached_change.start_pc = 0x80010000u;
    uncached_change.program = {
        enc_i(0x09, 1, 1, 1),
        enc_j(0x02, 0x80010100u),
        0u,
    };
    uncached_change.memory = {
        {0x80010100u, enc_i(0x09, 2, 2, 1)},
        {0x80010104u, enc_j(0x02, uncached_change.start_pc)},
        {0x80010108u, 0u},
    };
    uncached_change.mutations.push_back(
        {1u, 0x80010100u, enc_i(0x09, 2, 2, 7), false});
    uncached_change.instructions = 20u;
    cases.push_back(uncached_change);

    // The changed word sits in the middle of a line the loop is running through.
    CpuCompareCase mid_line_change{};
    mid_line_change.name = "ram_code_change_behind_cpu_mid_line_then_new_block";
    mid_line_change.start_pc = 0x80010000u;
    mid_line_change.program = {
        enc_i(0x09, 1, 1, 1),
        enc_i(0x09, 2, 2, 1),
        enc_i(0x09, 3, 3, 1),
        enc_j(0x02, mid_line_change.start_pc),
        0u,
    };
    mid_line_change.mutations.push_back(
        {6u, mid_line_change.start_pc + 8u, enc_i(0x09, 3, 3, 3), false});
    mid_line_change.instructions = 40u;
    cases.push_back(mid_line_change);
  }

  CpuCompareCase v4_native_icache_revalidation{};
  v4_native_icache_revalidation.name =
      "v4_native_icache_alias_revalidates_inside_dispatch";
  v4_native_icache_revalidation.start_pc = 0x80010000u;
  v4_native_icache_revalidation.program = {
      enc_i(0x09, 1, 1, 1),
      enc_j(0x02, 0x80011000u),
      0u,
  };
  v4_native_icache_revalidation.memory = {
      {0x80011000u, enc_i(0x09, 2, 2, 1)},
      {0x80011004u,
       enc_j(0x02, v4_native_icache_revalidation.start_pc)},
      {0x80011008u, 0u},
  };
  v4_native_icache_revalidation.instructions = 18u;
  v4_native_icache_revalidation.require_v4_native_entry_when_available = true;
  v4_native_icache_revalidation
      .require_v4_native_icache_revalidation_when_available = true;
  cases.push_back(v4_native_icache_revalidation);

  CpuCompareCase v4_translation_reset_slot_reuse{};
  v4_translation_reset_slot_reuse.name =
      "v4_translation_reset_does_not_alias_reused_block_slot";
  v4_translation_reset_slot_reuse.start_pc = 0x80010000u;
  // X (0x80010000) compiles into block slot 0. After a translation reset, Y
  // (0x80010040) is compiled first and reuses slot 0 under the new epoch. X's
  // surviving dispatch cell must not run Y's translation when Y jumps back.
  v4_translation_reset_slot_reuse.program = {
      enc_i(0x09, 1, 1, 1),
      enc_j(0x02, 0x80010040u),
      0u,
  };
  v4_translation_reset_slot_reuse.memory = {
      {0x80010040u, enc_i(0x09, 2, 2, 1)},
      {0x80010044u, enc_j(0x02, v4_translation_reset_slot_reuse.start_pc)},
      {0x80010048u, 0u},
  };
  {
    CpuCompareCodeMutation flush{};
    flush.after_instructions = 3u;
    flush.flush_backend = true;
    v4_translation_reset_slot_reuse.mutations.push_back(flush);
  }
  v4_translation_reset_slot_reuse.instructions = 18u;
  v4_translation_reset_slot_reuse.require_v4_native_entry_when_available = true;
  cases.push_back(v4_translation_reset_slot_reuse);

  CpuCompareCase v4_swr_cycle_budget_boundary{};
  v4_swr_cycle_budget_boundary.name =
      "v4_swr_ram_preserves_cycle_budget_boundary";
  v4_swr_cycle_budget_boundary.start_pc = 0xA0010000u;
  v4_swr_cycle_budget_boundary.initial_gpr[1] = 0x00012000u;
  v4_swr_cycle_budget_boundary.initial_gpr[2] = 0xAABBCCDDu;
  v4_swr_cycle_budget_boundary.memory = {
      {0x00012000u, 0x11223344u},
  };
  // The fused block is ADDIU (1) + SWR RAM RMW (7) + ADDIU (1).
  // With a five-cycle slice the interpreter retires ADDIU+SWR and overshoots
  // by the single current instruction. The recompiler must not admit the
  // trailing ADDIU merely because stale store metadata says SWR costs 3.
  v4_swr_cycle_budget_boundary.program = {
      enc_i(0x09, 0, 3, 1),
      enc_i(0x2E, 1, 2, 1),
      enc_i(0x09, 0, 4, 0x44),
  };
  v4_swr_cycle_budget_boundary.instructions = 3u;
  v4_swr_cycle_budget_boundary.run_slice_cycle_budget = 5u;
  v4_swr_cycle_budget_boundary.compare_segment_states = true;
  v4_swr_cycle_budget_boundary.require_v4_native_entry_when_available = true;
  v4_swr_cycle_budget_boundary.require_v4_native_store_entry_when_available = true;
  cases.push_back(v4_swr_cycle_budget_boundary);

  CpuCompareCase v4_cold_icache_first_mmio_timestamp{};
  v4_cold_icache_first_mmio_timestamp.name =
      "v4_cold_icache_first_mmio_preserves_pre_fetch_timestamp";
  v4_cold_icache_first_mmio_timestamp.start_pc = 0x80010000u;
  v4_cold_icache_first_mmio_timestamp.initial_gpr[1] = 0x1F801044u;
  v4_cold_icache_first_mmio_timestamp.program = {
      enc_i(0x23, 1, 2, 0), // LW r2, PAD/SIO STAT as the first cold opcode
  };
  v4_cold_icache_first_mmio_timestamp.instructions = 1u;
  v4_cold_icache_first_mmio_timestamp.prime_sio_before_run = true;
  v4_cold_icache_first_mmio_timestamp.require_v4_native_entry_when_available =
      true;
  v4_cold_icache_first_mmio_timestamp.require_v4_mmio_native_when_available =
      true;
  cases.push_back(v4_cold_icache_first_mmio_timestamp);

  CpuCompareCase v4_icache_first_mmio_timestamp{};
  v4_icache_first_mmio_timestamp.name =
      "v4_icache_refill_first_mmio_preserves_pre_fetch_timestamp";
  v4_icache_first_mmio_timestamp.start_pc = 0x80010000u;
  v4_icache_first_mmio_timestamp.initial_gpr[1] = 0x1F801044u;
  v4_icache_first_mmio_timestamp.program = {
      enc_i(0x23, 1, 2, 0),  // LW r2, PAD/SIO STAT -- first block instruction
      enc_j(0x02, v4_icache_first_mmio_timestamp.start_pc),
      0u,
  };
  // First pass compiles the block and returns to its start. Then evict the
  // direct-mapped I-cache entry through an aliased line without invalidating the
  // compiled block, and start an eight-cycle SIO transfer at that exact CPU
  // boundary. Resident revalidation refills A by four cycles before dispatch.
  // Cpu::step() keeps that refill in cycle_penalty_ while the first MMIO access
  // still observes the pre-fetch CPU timestamp. Native execution must do the
  // same: after the one-instruction second segment SIO must still have all eight
  // cycles remaining, not four.
  v4_icache_first_mmio_timestamp.mutations.push_back(
      {3u, v4_icache_first_mmio_timestamp.start_pc + 0x1000u,
       0xDEADBEEFu, true, true});
  v4_icache_first_mmio_timestamp.instructions = 4u;
  v4_icache_first_mmio_timestamp.compare_segment_states = true;
  v4_icache_first_mmio_timestamp.require_v4_native_entry_when_available = true;
  v4_icache_first_mmio_timestamp.require_v4_mmio_native_when_available = true;
  // The alias/refill construction above supplies the timing condition. Generic
  // revalidation topology is already gated by the dedicated cache tests, so do
  // not require their exact block-count shape here.
  cases.push_back(v4_icache_first_mmio_timestamp);

  CpuCompareCase v4_pending_delay_cold_mmio_timestamp{};
  v4_pending_delay_cold_mmio_timestamp.name =
      "v4_pending_delay_cold_mmio_preserves_pre_fetch_timestamp";
  v4_pending_delay_cold_mmio_timestamp.start_pc = 0x80010010u;
  v4_pending_delay_cold_mmio_timestamp.initial_gpr[1] = 0x1F801044u;
  v4_pending_delay_cold_mmio_timestamp.initial_next_pc = 0x80010040u;
  v4_pending_delay_cold_mmio_timestamp.initial_pending_delay_slot = true;
  v4_pending_delay_cold_mmio_timestamp.initial_pending_branch_taken = true;
  v4_pending_delay_cold_mmio_timestamp.initial_pending_branch_pc = 0x8001000Cu;
  v4_pending_delay_cold_mmio_timestamp.program = {
      enc_i(0x23, 1, 2, 0), // LW r2, PAD/SIO STAT in the pending delay slot
  };
  v4_pending_delay_cold_mmio_timestamp.memory = {
      {0x80010040u, 0u},
  };
  v4_pending_delay_cold_mmio_timestamp.instructions = 1u;
  v4_pending_delay_cold_mmio_timestamp.prime_sio_before_run = true;
  v4_pending_delay_cold_mmio_timestamp.require_v4_native_entry_when_available =
      true;
  v4_pending_delay_cold_mmio_timestamp
      .require_v4_pending_delay_native_when_available = true;
  v4_pending_delay_cold_mmio_timestamp.require_v4_mmio_native_when_available =
      true;
  cases.push_back(v4_pending_delay_cold_mmio_timestamp);

  CpuCompareCase v4_cold_icache_first_cdrom_timestamp{};
  v4_cold_icache_first_cdrom_timestamp.name =
      "v4_cold_icache_first_cdrom_preserves_pre_fetch_timestamp";
  v4_cold_icache_first_cdrom_timestamp.start_pc = 0x80010000u;
  v4_cold_icache_first_cdrom_timestamp.initial_gpr[1] = 0x1F801800u;
  v4_cold_icache_first_cdrom_timestamp.program = {
      enc_i(0x24, 1, 2, 0), // LBU r2, CD-ROM status as the first cold opcode
  };
  v4_cold_icache_first_cdrom_timestamp.instructions = 1u;
  v4_cold_icache_first_cdrom_timestamp.require_v4_native_entry_when_available =
      true;
  v4_cold_icache_first_cdrom_timestamp.require_v4_mmio_native_when_available =
      true;
  cases.push_back(v4_cold_icache_first_cdrom_timestamp);

  CpuCompareCase v4_sio_scheduler_boundary{};
  v4_sio_scheduler_boundary.name =
      "v4_sio_mmio_returns_at_scheduler_boundary";
  v4_sio_scheduler_boundary.start_pc = 0xA0010000u;
  v4_sio_scheduler_boundary.initial_gpr[1] = 0x1F801040u;
  v4_sio_scheduler_boundary.initial_gpr[2] = 1u;
  v4_sio_scheduler_boundary.program = {
      enc_i(0x28, 1, 2, 0),   // SB r2, SIO DATA: requests a scheduler boundary
      enc_i(0x09, 0, 3, 1),   // must remain unretired in this run_slice
      enc_i(0x09, 0, 4, 2),
  };
  v4_sio_scheduler_boundary.instructions = 3u;
  v4_sio_scheduler_boundary.preserve_timing_boundary_short_return = true;
  v4_sio_scheduler_boundary.require_v4_native_entry_when_available = true;
  v4_sio_scheduler_boundary.require_v4_mmio_native_when_available = true;
  cases.push_back(v4_sio_scheduler_boundary);

  CpuCompareCase v4_cdrom_irq_ack_resident_timestamp{};
  v4_cdrom_irq_ack_resident_timestamp.name =
      "v4_cdrom_irq_ack_uses_resident_cycle_timestamp";
  v4_cdrom_irq_ack_resident_timestamp.initial_gpr[1] = 0x1F801800u;
  v4_cdrom_irq_ack_resident_timestamp.initial_gpr[2] = 1u;
  v4_cdrom_irq_ack_resident_timestamp.initial_gpr[3] = 0x1Fu;
  v4_cdrom_irq_ack_resident_timestamp.program = {
      enc_i(0x28, 1, 2, 0), // SB r2, 0(r1): select CD-ROM index 1
      0u,                    // NOP: accrue one resident cycle after boundary
      enc_i(0x28, 1, 3, 3), // SB r3, 3(r1): acknowledge active INT3
  };
  v4_cdrom_irq_ack_resident_timestamp.instructions = 3u;
  v4_cdrom_irq_ack_resident_timestamp.prime_cdrom_irq_before_run = true;
  v4_cdrom_irq_ack_resident_timestamp.require_v4_native_entry_when_available =
      true;
  v4_cdrom_irq_ack_resident_timestamp.require_v4_mmio_native_when_available =
      true;
  cases.push_back(v4_cdrom_irq_ack_resident_timestamp);

  CpuCompareCase v4_pending_delay_refill_budget_boundary{};
  v4_pending_delay_refill_budget_boundary.name =
      "v4_pending_delay_refill_preserves_cycle_budget";
  // Put a taken branch at the final word of an I-cache line. Its first fetch
  // costs 4 cycles and the taken branch costs 2, leaving one cycle in the
  // seven-cycle slice. The delay slot begins the next cold I-cache line: its
  // 4-cycle refill must be allowed to overshoot only together with that one
  // architectural delay-slot instruction. The recompiler must not underflow
  // the remaining cycle budget and continue executing the branch target.
  v4_pending_delay_refill_budget_boundary.start_pc = 0x8001000Cu;
  v4_pending_delay_refill_budget_boundary.initial_gpr[1] = 1u;
  v4_pending_delay_refill_budget_boundary.program = {
      enc_i(0x05, 1, 0, 4),       // BNE -> 0x80010020
      0u,                          // delay slot on next I-cache line
      enc_i(0x09, 0, 7, 0x7777), // untaken-path sentinel
      0u,
      0u,
      enc_i(0x09, 0, 2, 1),      // branch target
      enc_i(0x09, 0, 3, 2),
  };
  v4_pending_delay_refill_budget_boundary.instructions = 4u;
  v4_pending_delay_refill_budget_boundary.run_slice_cycle_budget = 7u;
  v4_pending_delay_refill_budget_boundary.compare_segment_states = true;
  v4_pending_delay_refill_budget_boundary
      .require_v4_native_entry_when_available = true;
  v4_pending_delay_refill_budget_boundary
      .require_v4_pending_delay_native_when_available = true;
  cases.push_back(v4_pending_delay_refill_budget_boundary);

  CpuCompareCase v4_icache_branch_refill_budget_boundary{};
  v4_icache_branch_refill_budget_boundary.name =
      "v4_icache_refill_crossing_budget_retires_started_branch";
  v4_icache_branch_refill_budget_boundary.start_pc = 0x8001000Cu;
  v4_icache_branch_refill_budget_boundary.program = {
      enc_i(0x09, 0, 1, 1),          // cold-line ADDIU: 4 refill + 1
      enc_j(0x03, 0x80010020u),       // next cold line: refill crosses budget
      enc_i(0x09, 0, 2, 0x22),        // delay slot must remain pending
      0u,
      0u,
      enc_i(0x09, 0, 3, 0x33),        // JAL target
  };
  // After the first instruction the slice has consumed 5/8 cycles. Fetching
  // JAL costs four more and therefore crosses the deadline. Cpu::step() has
  // already started JAL, so it must still retire the branch itself (2 cycles)
  // and stop before the delay slot: 11 cycles / 2 instructions total.
  v4_icache_branch_refill_budget_boundary.instructions = 4u;
  v4_icache_branch_refill_budget_boundary.run_slice_cycle_budget = 8u;
  v4_icache_branch_refill_budget_boundary.compare_segment_states = true;
  v4_icache_branch_refill_budget_boundary
      .require_v4_native_entry_when_available = true;
  v4_icache_branch_refill_budget_boundary
      .require_v4_native_branch_entry_when_available = true;
  v4_icache_branch_refill_budget_boundary
      .require_v4_pending_delay_native_when_available = true;
  cases.push_back(v4_icache_branch_refill_budget_boundary);

  CpuCompareCase v4_icache_cycle_budget_boundary{};
  v4_icache_cycle_budget_boundary.name =
      "v4_icache_alias_preserves_cycle_budget_boundary";
  v4_icache_cycle_budget_boundary.start_pc = 0x80010000u;
  v4_icache_cycle_budget_boundary.program = {
      enc_i(0x09, 1, 1, 1),
      enc_j(0x02, 0x80011000u),
      0u,
  };
  v4_icache_cycle_budget_boundary.memory = {
      {0x80011000u, enc_i(0x09, 2, 2, 1)},
      {0x80011004u,
       enc_j(0x02, v4_icache_cycle_budget_boundary.start_pc)},
      {0x80011008u, 0u},
  };
  // A then B consume exactly enough work that the next aliased A-line refill
  // reaches the 20-cycle slice deadline. The interpreter and historical
  // recompiler still execute one architectural instruction after that refill.
  // Compare the first run_slice boundary, not only the final converged state.
  v4_icache_cycle_budget_boundary.instructions = 9u;
  v4_icache_cycle_budget_boundary.run_slice_cycle_budget = 20u;
  v4_icache_cycle_budget_boundary.compare_segment_states = true;
  v4_icache_cycle_budget_boundary.require_v4_native_entry_when_available = true;
  cases.push_back(v4_icache_cycle_budget_boundary);

  // Keep uncached KSEG1 smoke gates as a direct no-I-cache baseline. Cacheable
  // native execution is separately gated by native_control_state_icache_cycles.
  CpuCompareCase v4_uncached_alu{};
  v4_uncached_alu.name = "v4_uncached_native_alu";
  v4_uncached_alu.start_pc = 0xA0010000u;
  v4_uncached_alu.initial_gpr[1] = 7u;
  v4_uncached_alu.program = {
      enc_i(0x09, 1, 2, 5),
      enc_i(0x0D, 2, 3, 0x0030),
      enc_r(0, 3, 4, 1, 0x00),
  };
  pad_cpu_compare_program(v4_uncached_alu, 32u);
  v4_uncached_alu.require_v4_native_entry_when_available = true;
  cases.push_back(v4_uncached_alu);

  CpuCompareCase v4_uncached_variable_shifts{};
  v4_uncached_variable_shifts.name = "v4_uncached_native_variable_shifts";
  v4_uncached_variable_shifts.start_pc = 0xA0010000u;
  v4_uncached_variable_shifts.initial_gpr[1] = 0x81234567u;
  v4_uncached_variable_shifts.initial_gpr[2] = 5u;
  v4_uncached_variable_shifts.program = {
      enc_r(2, 1, 3, 0, 0x04), // SLLV
      enc_r(2, 1, 4, 0, 0x06), // SRLV
      enc_r(2, 1, 5, 0, 0x07), // SRAV
  };
  pad_cpu_compare_program(v4_uncached_variable_shifts, 32u);
  v4_uncached_variable_shifts.require_v4_native_entry_when_available = true;
  cases.push_back(v4_uncached_variable_shifts);

  CpuCompareCase v4_uncached_folded_branch{};
  v4_uncached_folded_branch.name = "v4_uncached_folded_alu_branch_tail";
  v4_uncached_folded_branch.start_pc = 0xA0010000u;
  v4_uncached_folded_branch.initial_gpr[1] = 1u;
  v4_uncached_folded_branch.program = {
      enc_i(0x09, 1, 2, 4),       // ADDIU prefix
      enc_i(0x0D, 2, 3, 0x10),    // ORI prefix
      enc_i(0x05, 3, 0, 1),       // BNE
      enc_i(0x09, 0, 4, 0x44),    // delay slot
      0,
  };
  v4_uncached_folded_branch.instructions = 4u;
  v4_uncached_folded_branch.require_v4_native_entry_when_available = true;
  v4_uncached_folded_branch.require_v4_native_branch_entry_when_available =
      true;
  v4_uncached_folded_branch.require_v4_folded_branch_block_when_available =
      true;
  cases.push_back(v4_uncached_folded_branch);

  CpuCompareCase v4_uncached_beq{};
  v4_uncached_beq.name = "v4_uncached_native_beq_delay";
  v4_uncached_beq.start_pc = 0xA0010000u;
  v4_uncached_beq.initial_gpr[1] = 0x1234u;
  v4_uncached_beq.initial_gpr[2] = 0x1234u;
  v4_uncached_beq.program = {
      enc_i(0x04, 1, 2, 2),
      enc_i(0x09, 0, 5, 0x0055),
      enc_i(0x09, 0, 6, 0x0066),
      enc_i(0x09, 0, 7, 0x0077),
  };
  v4_uncached_beq.instructions = 2u;
  v4_uncached_beq.require_v4_native_entry_when_available = true;
  v4_uncached_beq.require_v4_native_branch_entry_when_available = true;
  cases.push_back(v4_uncached_beq);

  CpuCompareCase v4_crossline_cold_delay{};
  v4_crossline_cold_delay.name =
      "v4_crossline_branch_cold_delay_stays_native";
  v4_crossline_cold_delay.start_pc = 0x8001000Cu;
  v4_crossline_cold_delay.initial_gpr[1] = 1u;
  v4_crossline_cold_delay.program = {
      enc_i(0x04, 1, 1, 4),       // BEQ at end of I-cache line -> 0x80010020
      enc_i(0x09, 0, 5, 0x0055),  // delay slot starts the next cold line
      0u,
      0u,
      0u,
      enc_i(0x09, 0, 6, 0x0066),
  };
  v4_crossline_cold_delay.instructions = 2u;
  v4_crossline_cold_delay.require_v4_native_entry_when_available = true;
  v4_crossline_cold_delay.require_v4_native_branch_entry_when_available = true;
  v4_crossline_cold_delay.require_v4_pending_delay_native_when_available = true;
  cases.push_back(v4_crossline_cold_delay);

  CpuCompareCase v4_uncached_page_boundary_delay{};
  v4_uncached_page_boundary_delay.name =
      "v4_uncached_page_boundary_branch_delay_stays_native";
  v4_uncached_page_boundary_delay.start_pc = 0xA0010FFCu;
  v4_uncached_page_boundary_delay.initial_gpr[1] = 1u;
  v4_uncached_page_boundary_delay.program = {
      enc_i(0x04, 1, 1, 4),
      enc_i(0x09, 0, 5, 0x0055),
      0u,
      0u,
      0u,
      enc_i(0x09, 0, 6, 0x0066),
  };
  v4_uncached_page_boundary_delay.instructions = 2u;
  v4_uncached_page_boundary_delay.require_v4_native_entry_when_available = true;
  v4_uncached_page_boundary_delay
      .require_v4_native_branch_entry_when_available = true;
  v4_uncached_page_boundary_delay
      .require_v4_pending_delay_native_when_available = true;
  cases.push_back(v4_uncached_page_boundary_delay);

  CpuCompareCase v4_split_scheduler_beq{};
  v4_split_scheduler_beq.name = "v4_split_scheduler_beq_delay_state";
  v4_split_scheduler_beq.start_pc = 0xA0010000u;
  v4_split_scheduler_beq.initial_gpr[1] = 0x1234u;
  v4_split_scheduler_beq.initial_gpr[2] = 0x1234u;
  v4_split_scheduler_beq.program = {
      enc_i(0x04, 1, 2, 2),       // BEQ taken; execute alone in segment 0.
      enc_i(0x09, 0, 5, 0x0055),  // Delay slot executes in segment 1.
      enc_i(0x09, 0, 6, 0x0066),
      enc_i(0x09, 0, 7, 0x0077),
  };
  v4_split_scheduler_beq.instructions = 2u;
  v4_split_scheduler_beq.segment_instructions = {1u, 1u};
  v4_split_scheduler_beq.compare_segment_states = true;
  v4_split_scheduler_beq.require_v4_native_entry_when_available = true;
  v4_split_scheduler_beq.require_v4_native_branch_entry_when_available = true;
  v4_split_scheduler_beq.require_v4_pending_delay_native_when_available = true;
  cases.push_back(v4_split_scheduler_beq);

  CpuCompareCase v4_split_scheduler_store{};
  v4_split_scheduler_store.name = "v4_split_scheduler_store_delay_native";
  v4_split_scheduler_store.start_pc = 0xA0010000u;
  v4_split_scheduler_store.initial_gpr[1] = 1u;
  v4_split_scheduler_store.initial_gpr[2] = 0x80012200u;
  v4_split_scheduler_store.initial_gpr[3] = 0x13579BDFu;
  v4_split_scheduler_store.memory.push_back({0x00012200u, 0u});
  v4_split_scheduler_store.compare_memory_addresses.push_back(0x00012200u);
  v4_split_scheduler_store.program = {
      enc_i(0x04, 1, 1, 1),  // BEQ taken; execute alone in segment 0.
      enc_i(0x2B, 2, 3, 0),  // SW executes as the pending delay slot.
      0,
  };
  v4_split_scheduler_store.instructions = 2u;
  v4_split_scheduler_store.segment_instructions = {1u, 1u};
  v4_split_scheduler_store.compare_segment_states = true;
  v4_split_scheduler_store.require_v4_native_entry_when_available = true;
  v4_split_scheduler_store.require_v4_native_branch_entry_when_available = true;
  v4_split_scheduler_store.require_v4_pending_delay_native_when_available = true;
  cases.push_back(v4_split_scheduler_store);

  CpuCompareCase v4_split_scheduler_scratch_store = v4_split_scheduler_store;
  v4_split_scheduler_scratch_store.name =
      "v4_split_scheduler_scratchpad_store_delay_native";
  v4_split_scheduler_scratch_store.initial_gpr[2] = 0x1F800200u;
  v4_split_scheduler_scratch_store.memory = {{0x1F800200u, 0u}};
  v4_split_scheduler_scratch_store.compare_memory_addresses = {0x1F800200u};
  v4_split_scheduler_scratch_store
      .require_v4_native_store_entry_when_available = true;
  cases.push_back(v4_split_scheduler_scratch_store);

  CpuCompareCase v4_split_scheduler_store_fault{};
  v4_split_scheduler_store_fault.name =
      "v4_split_scheduler_store_delay_addr_error_native";
  v4_split_scheduler_store_fault.start_pc = 0xA0010000u;
  v4_split_scheduler_store_fault.initial_gpr[1] = 1u;
  v4_split_scheduler_store_fault.initial_gpr[2] = 0x80012202u;
  v4_split_scheduler_store_fault.initial_gpr[3] = 0x13579BDFu;
  v4_split_scheduler_store_fault.program = {
      enc_i(0x04, 1, 1, 1), // BEQ taken; execute alone in segment 0.
      enc_i(0x2B, 2, 3, 0), // Misaligned SW faults in the delay slot.
      0,
  };
  v4_split_scheduler_store_fault.instructions = 2u;
  v4_split_scheduler_store_fault.segment_instructions = {1u, 1u};
  v4_split_scheduler_store_fault.compare_segment_states = true;
  v4_split_scheduler_store_fault.require_v4_native_entry_when_available = true;
  v4_split_scheduler_store_fault
      .require_v4_pending_delay_native_when_available = true;
  v4_split_scheduler_store_fault
      .require_v4_exception_native_when_available = true;
  cases.push_back(v4_split_scheduler_store_fault);

  CpuCompareCase v4_split_scheduler_store_mmio{};
  v4_split_scheduler_store_mmio.name =
      "v4_split_scheduler_store_delay_mmio_native";
  v4_split_scheduler_store_mmio.start_pc = 0xA0010000u;
  v4_split_scheduler_store_mmio.initial_gpr[1] = 1u;
  v4_split_scheduler_store_mmio.initial_gpr[2] = 0x1F801074u;
  v4_split_scheduler_store_mmio.initial_gpr[3] = 1u;
  v4_split_scheduler_store_mmio.program = {
      enc_i(0x05, 1, 0, 1), // BNE taken; execute alone in segment 0.
      enc_i(0x2B, 2, 3, 0), // SW I_MASK through the native device bridge.
      0,
  };
  v4_split_scheduler_store_mmio.instructions = 2u;
  v4_split_scheduler_store_mmio.segment_instructions = {1u, 1u};
  v4_split_scheduler_store_mmio.compare_segment_states = true;
  v4_split_scheduler_store_mmio.require_v4_native_entry_when_available = true;
  v4_split_scheduler_store_mmio
      .require_v4_pending_delay_native_when_available = true;
  v4_split_scheduler_store_mmio.require_v4_mmio_native_when_available = true;
  cases.push_back(v4_split_scheduler_store_mmio);

  CpuCompareCase v4_split_scheduler_swl{};
  v4_split_scheduler_swl.name = "v4_split_scheduler_swl_delay_native";
  v4_split_scheduler_swl.start_pc = 0xA0010000u;
  v4_split_scheduler_swl.initial_gpr[1] = 1u;
  v4_split_scheduler_swl.initial_gpr[2] = 0x80012222u;
  v4_split_scheduler_swl.initial_gpr[3] = 0x89ABCDEFu;
  v4_split_scheduler_swl.memory.push_back({0x00012220u, 0x11223344u});
  v4_split_scheduler_swl.compare_memory_addresses.push_back(0x00012220u);
  v4_split_scheduler_swl.program = {
      enc_i(0x04, 1, 1, 1), // BEQ taken.
      enc_i(0x2A, 2, 3, 0), // SWL offset 2 in the delay slot.
      0,
  };
  v4_split_scheduler_swl.instructions = 2u;
  v4_split_scheduler_swl.segment_instructions = {1u, 1u};
  v4_split_scheduler_swl.compare_segment_states = true;
  v4_split_scheduler_swl.require_v4_native_entry_when_available = true;
  v4_split_scheduler_swl.require_v4_pending_delay_native_when_available = true;
  cases.push_back(v4_split_scheduler_swl);

  CpuCompareCase v4_split_scheduler_load{};
  v4_split_scheduler_load.name = "v4_split_scheduler_load_delay_native";
  v4_split_scheduler_load.start_pc = 0xA0010000u;
  v4_split_scheduler_load.initial_gpr[1] = 1u;
  v4_split_scheduler_load.initial_gpr[2] = 0x80012210u;
  v4_split_scheduler_load.memory.push_back({0x00012210u, 0x2468ACE0u});
  v4_split_scheduler_load.program = {
      enc_i(0x05, 1, 0, 1),  // BNE taken; execute alone in segment 0.
      enc_i(0x23, 2, 3, 0),  // LW executes as the pending delay slot.
      0,                     // Target retires the load delay.
  };
  v4_split_scheduler_load.instructions = 3u;
  v4_split_scheduler_load.segment_instructions = {1u, 2u};
  v4_split_scheduler_load.compare_segment_states = true;
  v4_split_scheduler_load.require_v4_native_entry_when_available = true;
  v4_split_scheduler_load.require_v4_native_load_entry_when_available = true;
  // Branch execution is independently gated by the split-branch cases above.
  // This case specifically proves that the pending LW itself never uses an
  // opcode helper after the scheduler split.
  v4_split_scheduler_load.require_v4_pending_delay_native_when_available = true;
  cases.push_back(v4_split_scheduler_load);

  CpuCompareCase v4_split_scheduler_hilo{};
  v4_split_scheduler_hilo.name = "v4_split_scheduler_hilo_delay_native";
  v4_split_scheduler_hilo.start_pc = 0xA0010000u;
  v4_split_scheduler_hilo.initial_gpr[1] = 1u;
  v4_split_scheduler_hilo.initial_gpr[3] = 0x1234ABCDu;
  v4_split_scheduler_hilo.program = {
      enc_i(0x05, 1, 0, 1),       // BNE taken.
      enc_r(3, 0, 0, 0, 0x11),    // MTHI in the delay slot.
      enc_r(0, 0, 4, 0, 0x10),    // MFHI at the target.
  };
  v4_split_scheduler_hilo.instructions = 3u;
  v4_split_scheduler_hilo.segment_instructions = {1u, 2u};
  v4_split_scheduler_hilo.compare_segment_states = true;
  v4_split_scheduler_hilo.require_v4_native_entry_when_available = true;
  v4_split_scheduler_hilo.require_v4_pending_delay_native_when_available = true;
  v4_split_scheduler_hilo.require_v4_hilo_native_when_available = true;
  cases.push_back(v4_split_scheduler_hilo);

  CpuCompareCase v4_split_scheduler_muldiv{};
  v4_split_scheduler_muldiv.name = "v4_split_scheduler_muldiv_delay_native";
  v4_split_scheduler_muldiv.start_pc = 0xA0010000u;
  v4_split_scheduler_muldiv.initial_gpr[1] = 1u;
  v4_split_scheduler_muldiv.initial_gpr[2] = 7u;
  v4_split_scheduler_muldiv.initial_gpr[3] = 9u;
  v4_split_scheduler_muldiv.program = {
      enc_i(0x05, 1, 0, 1),       // BNE taken.
      enc_r(2, 3, 0, 0, 0x19),    // MULTU in the delay slot.
      enc_r(0, 0, 4, 0, 0x12),    // MFLO at the target.
  };
  v4_split_scheduler_muldiv.instructions = 3u;
  v4_split_scheduler_muldiv.segment_instructions = {1u, 2u};
  v4_split_scheduler_muldiv.compare_segment_states = true;
  v4_split_scheduler_muldiv.require_v4_native_entry_when_available = true;
  v4_split_scheduler_muldiv.require_v4_pending_delay_native_when_available = true;
  v4_split_scheduler_muldiv.require_v4_muldiv_native_when_available = true;
  cases.push_back(v4_split_scheduler_muldiv);

  CpuCompareCase v4_split_scheduler_cop0{};
  v4_split_scheduler_cop0.name = "v4_split_scheduler_cop0_delay_native";
  v4_split_scheduler_cop0.start_pc = 0xA0010000u;
  v4_split_scheduler_cop0.initial_gpr[1] = 1u;
  v4_split_scheduler_cop0.initial_gpr[2] = 0x00000300u;
  v4_split_scheduler_cop0.program = {
      enc_i(0x05, 1, 0, 1), // BNE taken.
      (0x10u << 26) | (4u << 21) | (2u << 16) | (13u << 11), // MTC0 Cause
      (0x10u << 26) | (0u << 21) | (3u << 16) | (13u << 11), // MFC0 Cause
  };
  v4_split_scheduler_cop0.instructions = 3u;
  v4_split_scheduler_cop0.segment_instructions = {1u, 2u};
  v4_split_scheduler_cop0.compare_segment_states = true;
  v4_split_scheduler_cop0.require_v4_native_entry_when_available = true;
  v4_split_scheduler_cop0.require_v4_pending_delay_native_when_available = true;
  v4_split_scheduler_cop0.require_v4_cop0_native_when_available = true;
  cases.push_back(v4_split_scheduler_cop0);

  CpuCompareCase v4_split_scheduler_cop2{};
  v4_split_scheduler_cop2.name = "v4_split_scheduler_cop2_delay_native";
  v4_split_scheduler_cop2.start_pc = 0xA0010000u;
  v4_split_scheduler_cop2.initial_gpr[1] = 1u;
  v4_split_scheduler_cop2.initial_gpr[2] = 0x44332211u;
  v4_split_scheduler_cop2.program = {
      enc_i(0x05, 1, 0, 1), // BNE taken.
      (0x12u << 26) | (4u << 21) | (2u << 16) | (6u << 11), // MTC2 data 6.
      (0x12u << 26) | (0u << 21) | (3u << 16) | (6u << 11), // MFC2 data 6.
      0u, // Retire the MFC2 load delay.
  };
  v4_split_scheduler_cop2.instructions = 4u;
  v4_split_scheduler_cop2.segment_instructions = {1u, 3u};
  v4_split_scheduler_cop2.compare_segment_states = true;
  v4_split_scheduler_cop2.require_v4_native_entry_when_available = true;
  v4_split_scheduler_cop2.require_v4_pending_delay_native_when_available = true;
  v4_split_scheduler_cop2.require_v4_cop2_native_when_available = true;
  cases.push_back(v4_split_scheduler_cop2);

  CpuCompareCase v4_split_scheduler_lwc2{};
  v4_split_scheduler_lwc2.name = "v4_split_scheduler_lwc2_delay_native";
  v4_split_scheduler_lwc2.start_pc = 0xA0010000u;
  v4_split_scheduler_lwc2.initial_gpr[1] = 1u;
  v4_split_scheduler_lwc2.initial_gpr[2] = 0x80012230u;
  v4_split_scheduler_lwc2.memory.push_back({0x00012230u, 0x2468ACE0u});
  v4_split_scheduler_lwc2.program = {
      enc_i(0x05, 1, 0, 1),      // BNE taken.
      enc_i(0x32, 2, 6, 0),      // LWC2 data 6 in the delay slot.
      (0x12u << 26) | (0u << 21) | (3u << 16) | (6u << 11), // MFC2 data 6.
      0u,                         // Retire the MFC2 CPU load delay.
  };
  v4_split_scheduler_lwc2.instructions = 4u;
  v4_split_scheduler_lwc2.segment_instructions = {1u, 3u};
  v4_split_scheduler_lwc2.compare_segment_states = true;
  v4_split_scheduler_lwc2.require_v4_native_entry_when_available = true;
  v4_split_scheduler_lwc2.require_v4_pending_delay_native_when_available = true;
  v4_split_scheduler_lwc2.require_v4_cop2_native_when_available = true;
  cases.push_back(v4_split_scheduler_lwc2);

  CpuCompareCase v4_split_scheduler_swc2{};
  v4_split_scheduler_swc2.name = "v4_split_scheduler_swc2_delay_native";
  v4_split_scheduler_swc2.start_pc = 0xA0010000u;
  v4_split_scheduler_swc2.initial_gpr[1] = 1u;
  v4_split_scheduler_swc2.initial_gpr[2] = 0x55667788u;
  v4_split_scheduler_swc2.initial_gpr[3] = 0x80012240u;
  v4_split_scheduler_swc2.memory.push_back({0x00012240u, 0u});
  v4_split_scheduler_swc2.compare_memory_addresses.push_back(0x00012240u);
  v4_split_scheduler_swc2.program = {
      (0x12u << 26) | (4u << 21) | (2u << 16) | (6u << 11), // MTC2 data 6.
      enc_i(0x05, 1, 0, 1),      // BNE taken.
      enc_i(0x3A, 3, 6, 0),      // SWC2 data 6 in the delay slot.
      0u,
  };
  v4_split_scheduler_swc2.instructions = 4u;
  v4_split_scheduler_swc2.segment_instructions = {1u, 1u, 2u};
  v4_split_scheduler_swc2.compare_segment_states = true;
  v4_split_scheduler_swc2.require_v4_native_entry_when_available = true;
  v4_split_scheduler_swc2.require_v4_pending_delay_native_when_available = true;
  v4_split_scheduler_swc2.require_v4_cop2_native_when_available = true;
  cases.push_back(v4_split_scheduler_swc2);

  CpuCompareCase v4_nested_branch_delay{};
  v4_nested_branch_delay.name = "v4_nested_branch_in_delay_slot_native";
  v4_nested_branch_delay.start_pc = 0xA0010000u;
  v4_nested_branch_delay.initial_gpr[1] = 1u;
  v4_nested_branch_delay.initial_gpr[2] = 1u;
  v4_nested_branch_delay.program = {
      enc_i(0x05, 1, 0, 1),       // Outer BNE -> 0xA0010008.
      enc_i(0x05, 2, 0, 1),       // Inner BNE occupies outer delay slot.
      enc_i(0x09, 0, 4, 0x0044),  // Inner delay slot at outer target.
      enc_i(0x09, 0, 5, 0x0055),  // Inner target.
  };
  v4_nested_branch_delay.instructions = 4u;
  v4_nested_branch_delay.segment_instructions = {1u, 1u, 2u};
  v4_nested_branch_delay.compare_segment_states = true;
  v4_nested_branch_delay.require_v4_native_entry_when_available = true;
  v4_nested_branch_delay.require_v4_pending_delay_native_when_available = true;
  v4_nested_branch_delay.require_v4_native_branch_entry_when_available = true;
  cases.push_back(v4_nested_branch_delay);

  CpuCompareCase v4_nested_likely_annul{};
  v4_nested_likely_annul.name = "v4_nested_branch_likely_annul_native";
  v4_nested_likely_annul.start_pc = 0xA0010000u;
  v4_nested_likely_annul.initial_gpr[1] = 1u;
  v4_nested_likely_annul.initial_gpr[2] = 1u;
  v4_nested_likely_annul.initial_gpr[3] = 2u;
  v4_nested_likely_annul.program = {
      enc_i(0x05, 1, 0, 1),       // Outer BNE -> 0xA0010008.
      enc_i(0x14, 2, 3, 1),       // BEQL not taken; annul its own delay.
      enc_i(0x09, 0, 4, 0x0044),  // Must be annulled.
      enc_i(0x09, 0, 5, 0x0055),  // Execution resumes here.
      0u,
  };
  v4_nested_likely_annul.instructions = 3u;
  v4_nested_likely_annul.segment_instructions = {1u, 1u, 1u};
  v4_nested_likely_annul.compare_segment_states = true;
  v4_nested_likely_annul.require_v4_native_entry_when_available = true;
  v4_nested_likely_annul.require_v4_pending_delay_native_when_available = true;
  v4_nested_likely_annul.require_v4_native_branch_entry_when_available = true;
  cases.push_back(v4_nested_likely_annul);

  CpuCompareCase v4_isolated_store{};
  v4_isolated_store.name = "v4_cache_isolated_store_native";
  v4_isolated_store.start_pc = 0xA0010000u;
  v4_isolated_store.initial_cop0_sr_bits = 1u << 16;
  v4_isolated_store.initial_gpr[1] = 0x80012250u;
  v4_isolated_store.initial_gpr[2] = 0xDEADBEEFu;
  v4_isolated_store.memory.push_back({0x00012250u, 0x11223344u});
  v4_isolated_store.compare_memory_addresses.push_back(0x00012250u);
  v4_isolated_store.program = {
      enc_i(0x2B, 1, 2, 0), // SW is discarded while IsC is set.
      0u,
  };
  v4_isolated_store.instructions = 1u;
  v4_isolated_store.require_v4_native_entry_when_available = true;
  v4_isolated_store.require_v4_native_store_entry_when_available = true;
  cases.push_back(v4_isolated_store);

  CpuCompareCase v4_isolated_swl{};
  v4_isolated_swl.name = "v4_cache_isolated_swl_native";
  v4_isolated_swl.start_pc = 0xA0010000u;
  v4_isolated_swl.initial_cop0_sr_bits = 1u << 16;
  v4_isolated_swl.initial_gpr[1] = 0x80012261u;
  v4_isolated_swl.initial_gpr[2] = 0xAABBCCDDu;
  v4_isolated_swl.memory.push_back({0x00012260u, 0x55667788u});
  v4_isolated_swl.compare_memory_addresses.push_back(0x00012260u);
  v4_isolated_swl.program = {
      enc_i(0x2A, 1, 2, 0), // SWL still performs its aligned read, no write.
      0u,
  };
  v4_isolated_swl.instructions = 1u;
  v4_isolated_swl.require_v4_native_entry_when_available = true;
  v4_isolated_swl.require_v4_native_store_entry_when_available = true;
  cases.push_back(v4_isolated_swl);

  CpuCompareCase v4_isolated_delay_store{};
  v4_isolated_delay_store.name = "v4_cache_isolated_store_delay_native";
  v4_isolated_delay_store.start_pc = 0xA0010000u;
  v4_isolated_delay_store.initial_cop0_sr_bits = 1u << 16;
  v4_isolated_delay_store.initial_gpr[1] = 1u;
  v4_isolated_delay_store.initial_gpr[2] = 0x80012270u;
  v4_isolated_delay_store.initial_gpr[3] = 0xCAFEBABEu;
  v4_isolated_delay_store.memory.push_back({0x00012270u, 0x01020304u});
  v4_isolated_delay_store.compare_memory_addresses.push_back(0x00012270u);
  v4_isolated_delay_store.program = {
      enc_i(0x05, 1, 0, 1), // BNE taken.
      enc_i(0x2B, 2, 3, 0), // Isolated SW in delay slot.
      0u,
  };
  v4_isolated_delay_store.instructions = 2u;
  v4_isolated_delay_store.segment_instructions = {1u, 1u};
  v4_isolated_delay_store.compare_segment_states = true;
  v4_isolated_delay_store.require_v4_native_entry_when_available = true;
  v4_isolated_delay_store.require_v4_pending_delay_native_when_available = true;
  cases.push_back(v4_isolated_delay_store);

  CpuCompareCase v4_split_scheduler_addi{};
  v4_split_scheduler_addi.name = "v4_split_scheduler_addi_delay_native";
  v4_split_scheduler_addi.start_pc = 0xA0010000u;
  v4_split_scheduler_addi.initial_gpr[1] = 1u;
  v4_split_scheduler_addi.initial_gpr[2] = 40u;
  v4_split_scheduler_addi.program = {
      enc_i(0x04, 1, 1, 1),  // BEQ taken; execute alone in segment 0.
      enc_i(0x08, 2, 2, 2),  // ADDI delay slot, no overflow.
      0,
  };
  v4_split_scheduler_addi.instructions = 2u;
  v4_split_scheduler_addi.segment_instructions = {1u, 1u};
  v4_split_scheduler_addi.compare_segment_states = true;
  v4_split_scheduler_addi.require_v4_native_entry_when_available = true;
  v4_split_scheduler_addi.require_v4_native_branch_entry_when_available = true;
  v4_split_scheduler_addi.require_v4_pending_delay_native_when_available = true;
  cases.push_back(v4_split_scheduler_addi);

  CpuCompareCase v4_split_scheduler_addi_overflow{};
  v4_split_scheduler_addi_overflow.name =
      "v4_split_scheduler_addi_delay_overflow_fallback";
  v4_split_scheduler_addi_overflow.start_pc = 0xA0010000u;
  v4_split_scheduler_addi_overflow.initial_gpr[1] = 1u;
  v4_split_scheduler_addi_overflow.initial_gpr[2] = 0x7FFFFFFFu;
  v4_split_scheduler_addi_overflow.program = {
      enc_i(0x04, 1, 1, 1),  // BEQ taken; execute alone in segment 0.
      enc_i(0x08, 2, 2, 1),  // ADDI overflows in the delay slot.
      0,
  };
  v4_split_scheduler_addi_overflow.instructions = 2u;
  v4_split_scheduler_addi_overflow.segment_instructions = {1u, 1u};
  v4_split_scheduler_addi_overflow.compare_segment_states = true;
  v4_split_scheduler_addi_overflow.require_v4_native_entry_when_available = true;
  v4_split_scheduler_addi_overflow
      .require_v4_native_branch_entry_when_available = true;
  cases.push_back(v4_split_scheduler_addi_overflow);

  CpuCompareCase v4_uncached_bne_not_taken{};
  v4_uncached_bne_not_taken.name = "v4_uncached_native_bne_not_taken_delay";
  v4_uncached_bne_not_taken.start_pc = 0xA0010000u;
  v4_uncached_bne_not_taken.initial_gpr[1] = 0x1234u;
  v4_uncached_bne_not_taken.initial_gpr[2] = 0x1234u;
  v4_uncached_bne_not_taken.program = {
      enc_i(0x05, 1, 2, 2),
      // Mutate an input in the delay slot: BNE must have captured "not taken"
      // before this executes.
      enc_i(0x09, 2, 2, 1),
      enc_i(0x09, 0, 6, 0x0066),
      enc_i(0x09, 0, 7, 0x0077),
  };
  v4_uncached_bne_not_taken.instructions = 2u;
  v4_uncached_bne_not_taken.require_v4_native_entry_when_available = true;
  v4_uncached_bne_not_taken.require_v4_native_branch_entry_when_available =
      true;
  cases.push_back(v4_uncached_bne_not_taken);

  CpuCompareCase v4_uncached_jal{};
  v4_uncached_jal.name = "v4_uncached_native_jal_link_delay";
  v4_uncached_jal.start_pc = 0xA0010000u;
  v4_uncached_jal.program = {
      enc_j(0x03, v4_uncached_jal.start_pc + 0x10u),
      enc_r(31, 0, 5, 0, 0x21),
      0,
      0,
      enc_i(0x09, 0, 6, 0x0066),
  };
  v4_uncached_jal.instructions = 2u;
  v4_uncached_jal.require_v4_native_entry_when_available = true;
  v4_uncached_jal.require_v4_native_branch_entry_when_available = true;
  cases.push_back(v4_uncached_jal);

  CpuCompareCase v4_uncached_jr_load_capture{};
  v4_uncached_jr_load_capture.name =
      "v4_uncached_native_jr_pending_load_capture";
  v4_uncached_jr_load_capture.start_pc = 0xA0010000u;
  // JR must capture the old r8 target before the pending load retires.
  // The delay slot, however, must observe the newly committed r8 value.
  v4_uncached_jr_load_capture.initial_gpr[8] =
      v4_uncached_jr_load_capture.start_pc + 0x10u;
  v4_uncached_jr_load_capture.initial_load_reg = 8u;
  v4_uncached_jr_load_capture.initial_load_value =
      v4_uncached_jr_load_capture.start_pc + 0x20u;
  v4_uncached_jr_load_capture.program = {
      enc_r(8, 0, 0, 0, 0x08),
      enc_r(8, 0, 5, 0, 0x21),
      0,
      0,
      enc_i(0x09, 0, 6, 0x0066),
  };
  v4_uncached_jr_load_capture.instructions = 2u;
  v4_uncached_jr_load_capture.require_v4_native_entry_when_available = true;
  v4_uncached_jr_load_capture.require_v4_native_branch_entry_when_available =
      true;
  cases.push_back(v4_uncached_jr_load_capture);

  CpuCompareCase v4_uncached_jalr{};
  v4_uncached_jalr.name = "v4_uncached_native_jalr_link_delay";
  v4_uncached_jalr.start_pc = 0xA0010000u;
  v4_uncached_jalr.initial_gpr[8] =
      v4_uncached_jalr.start_pc + 0x10u;
  v4_uncached_jalr.program = {
      enc_r(8, 0, 9, 0, 0x09),
      // JALR's link register must already be visible in the delay slot.
      enc_r(9, 0, 5, 0, 0x21),
      0,
      0,
      enc_i(0x09, 0, 6, 0x0066),
  };
  v4_uncached_jalr.instructions = 2u;
  v4_uncached_jalr.require_v4_native_entry_when_available = true;
  v4_uncached_jalr.require_v4_native_branch_entry_when_available = true;
  cases.push_back(v4_uncached_jalr);

  CpuCompareCase v4_uncached_blez{};
  v4_uncached_blez.name = "v4_uncached_native_blez_taken";
  v4_uncached_blez.start_pc = 0xA0010000u;
  v4_uncached_blez.initial_gpr[8] = 0u;
  v4_uncached_blez.program = {
      enc_i(0x06, 8, 0, 2),
      enc_i(0x09, 0, 5, 0x0055),
      enc_i(0x09, 0, 6, 0x0066),
      enc_i(0x09, 0, 7, 0x0077),
  };
  v4_uncached_blez.instructions = 2u;
  v4_uncached_blez.require_v4_native_entry_when_available = true;
  v4_uncached_blez.require_v4_native_branch_entry_when_available = true;
  cases.push_back(v4_uncached_blez);

  CpuCompareCase v4_uncached_bgtz{};
  v4_uncached_bgtz.name = "v4_uncached_native_bgtz_not_taken";
  v4_uncached_bgtz.start_pc = 0xA0010000u;
  v4_uncached_bgtz.initial_gpr[8] = 0u;
  v4_uncached_bgtz.program = {
      enc_i(0x07, 8, 0, 2),
      enc_i(0x09, 0, 5, 0x0055),
      enc_i(0x09, 0, 6, 0x0066),
      enc_i(0x09, 0, 7, 0x0077),
  };
  v4_uncached_bgtz.instructions = 2u;
  v4_uncached_bgtz.require_v4_native_entry_when_available = true;
  v4_uncached_bgtz.require_v4_native_branch_entry_when_available = true;
  cases.push_back(v4_uncached_bgtz);

  CpuCompareCase v4_uncached_bltzal{};
  v4_uncached_bltzal.name = "v4_uncached_native_bltzal_not_taken_link";
  v4_uncached_bltzal.start_pc = 0xA0010000u;
  v4_uncached_bltzal.initial_gpr[8] = 1u;
  v4_uncached_bltzal.program = {
      // REGIMM link variants write RA even when the condition is false.
      enc_i(0x01, 8, 0x10, 2),
      enc_r(31, 0, 5, 0, 0x21),
      enc_i(0x09, 0, 6, 0x0066),
      enc_i(0x09, 0, 7, 0x0077),
  };
  v4_uncached_bltzal.instructions = 2u;
  v4_uncached_bltzal.require_v4_native_entry_when_available = true;
  v4_uncached_bltzal.require_v4_native_branch_entry_when_available = true;
  cases.push_back(v4_uncached_bltzal);

  CpuCompareCase v4_uncached_bgez{};
  v4_uncached_bgez.name = "v4_uncached_native_bgez_taken";
  v4_uncached_bgez.start_pc = 0xA0010000u;
  v4_uncached_bgez.initial_gpr[8] = 0u;
  v4_uncached_bgez.program = {
      enc_i(0x01, 8, 0x01, 2),
      enc_i(0x09, 0, 5, 0x0055),
      enc_i(0x09, 0, 6, 0x0066),
      enc_i(0x09, 0, 7, 0x0077),
  };
  v4_uncached_bgez.instructions = 2u;
  v4_uncached_bgez.require_v4_native_entry_when_available = true;
  v4_uncached_bgez.require_v4_native_branch_entry_when_available = true;
  cases.push_back(v4_uncached_bgez);

  CpuCompareCase v4_uncached_guarded_addi_delay{};
  v4_uncached_guarded_addi_delay.name =
      "v4_uncached_native_branch_guarded_addi_delay";
  v4_uncached_guarded_addi_delay.start_pc = 0xA0010000u;
  v4_uncached_guarded_addi_delay.initial_gpr[1] = 1u;
  v4_uncached_guarded_addi_delay.initial_gpr[2] = 40u;
  v4_uncached_guarded_addi_delay.program = {
      enc_i(0x05, 1, 0, 1), // BNE taken
      enc_i(0x08, 2, 2, 2), // ADDI r2,r2,2 in the delay slot
      0,
  };
  v4_uncached_guarded_addi_delay.instructions = 2u;
  v4_uncached_guarded_addi_delay.require_v4_native_entry_when_available = true;
  v4_uncached_guarded_addi_delay
      .require_v4_native_branch_entry_when_available = true;
  cases.push_back(v4_uncached_guarded_addi_delay);

  CpuCompareCase v4_uncached_branch_store_delay{};
  v4_uncached_branch_store_delay.name =
      "v4_uncached_native_branch_store_delay";
  v4_uncached_branch_store_delay.start_pc = 0xA0010000u;
  v4_uncached_branch_store_delay.initial_gpr[1] = 1u;
  v4_uncached_branch_store_delay.initial_gpr[2] = 0x80012080u;
  v4_uncached_branch_store_delay.initial_gpr[3] = 0x89ABCDEFu;
  v4_uncached_branch_store_delay.memory.push_back({0x00012080u, 0u});
  v4_uncached_branch_store_delay.compare_memory_addresses.push_back(
      0x00012080u);
  v4_uncached_branch_store_delay.program = {
      enc_i(0x04, 1, 1, 1), // BEQ taken
      enc_i(0x2B, 2, 3, 0), // SW in the delay slot
      0,
  };
  v4_uncached_branch_store_delay.instructions = 2u;
  v4_uncached_branch_store_delay.require_v4_native_entry_when_available = true;
  v4_uncached_branch_store_delay
      .require_v4_native_store_entry_when_available = true;
  v4_uncached_branch_store_delay
      .require_v4_native_branch_entry_when_available = true;
  cases.push_back(v4_uncached_branch_store_delay);

  CpuCompareCase v4_uncached_delay_exception_native{};
  v4_uncached_delay_exception_native.name =
      "v4_uncached_branch_delay_overflow_native";
  v4_uncached_delay_exception_native.start_pc = 0xA0010000u;
  v4_uncached_delay_exception_native.initial_gpr[1] = 1u;
  v4_uncached_delay_exception_native.initial_gpr[2] = 0x7FFFFFFFu;
  v4_uncached_delay_exception_native.program = {
      enc_i(0x05, 1, 0, 1), // BNE taken
      enc_i(0x08, 2, 2, 1), // ADDI overflow in the delay slot
      0,
  };
  v4_uncached_delay_exception_native.instructions = 2u;
  v4_uncached_delay_exception_native.require_v4_native_entry_when_available =
      true;
  v4_uncached_delay_exception_native
      .require_v4_native_branch_entry_when_available = true;
  v4_uncached_delay_exception_native
      .require_v4_pending_delay_native_when_available = true;
  v4_uncached_delay_exception_native
      .require_v4_exception_native_when_available = true;
  cases.push_back(v4_uncached_delay_exception_native);

  CpuCompareCase v4_hot_timer_lhu{};
  v4_hot_timer_lhu.name = "v4_hot_timer_lhu_native";
  v4_hot_timer_lhu.start_pc = 0xA0010000u;
  v4_hot_timer_lhu.initial_gpr[1] = 0x1F801120u;
  v4_hot_timer_lhu.program = {
      enc_i(0x25, 1, 2, 0), // LHU timer 1 counter
      0,                    // retire the load delay
  };
  v4_hot_timer_lhu.instructions = 2u;
  v4_hot_timer_lhu.require_v4_native_entry_when_available = true;
  v4_hot_timer_lhu.require_v4_native_load_entry_when_available = true;
  v4_hot_timer_lhu.require_v4_hot_mmio16_native_when_available = true;
  cases.push_back(v4_hot_timer_lhu);

  CpuCompareCase v4_hot_irq_lhu{};
  v4_hot_irq_lhu.name = "v4_hot_irq_lhu_native";
  v4_hot_irq_lhu.start_pc = 0xA0010000u;
  v4_hot_irq_lhu.initial_gpr[1] = 0x1F801070u;
  v4_hot_irq_lhu.initial_irq_mask = 1u;
  v4_hot_irq_lhu.initial_irq_pending = true;
  v4_hot_irq_lhu.program = {
      enc_i(0x25, 1, 2, 0), // LHU I_STAT
      0,                    // retire the load delay
  };
  v4_hot_irq_lhu.instructions = 2u;
  v4_hot_irq_lhu.require_v4_native_entry_when_available = true;
  v4_hot_irq_lhu.require_v4_native_load_entry_when_available = true;
  v4_hot_irq_lhu.require_v4_hot_mmio16_native_when_available = true;
  cases.push_back(v4_hot_irq_lhu);

  CpuCompareCase v4_hilo_transfer{};
  v4_hilo_transfer.name = "v4_hilo_transfer_native";
  v4_hilo_transfer.start_pc = 0xA0010000u;
  v4_hilo_transfer.initial_gpr[1] = 0x12345678u;
  v4_hilo_transfer.initial_gpr[3] = 0x89ABCDEFu;
  v4_hilo_transfer.program = {
      enc_r(1, 0, 0, 0, 0x11), // MTHI r1
      enc_r(0, 0, 2, 0, 0x10), // MFHI r2
      enc_r(3, 0, 0, 0, 0x13), // MTLO r3
      enc_r(0, 0, 4, 0, 0x12), // MFLO r4
  };
  v4_hilo_transfer.instructions = 4u;
  v4_hilo_transfer.require_v4_native_entry_when_available = true;
  v4_hilo_transfer.require_v4_hilo_native_when_available = true;
  cases.push_back(v4_hilo_transfer);

  CpuCompareCase v4_mflo_muldiv_stall{};
  v4_mflo_muldiv_stall.name = "v4_mflo_muldiv_stall";
  v4_mflo_muldiv_stall.start_pc = 0xA0010000u;
  v4_mflo_muldiv_stall.initial_gpr[1] = 0x00100000u;
  v4_mflo_muldiv_stall.initial_gpr[2] = 3u;
  v4_mflo_muldiv_stall.program = {
      enc_r(1, 2, 0, 0, 0x18), // MULT r1,r2 (helper establishes scoreboard)
      enc_r(0, 0, 3, 0, 0x12), // MFLO r3 must honor result-ready stall
  };
  v4_mflo_muldiv_stall.instructions = 2u;
  v4_mflo_muldiv_stall.require_v4_native_entry_when_available = true;
  v4_mflo_muldiv_stall.require_v4_muldiv_native_when_available = true;
  cases.push_back(v4_mflo_muldiv_stall);

  CpuCompareCase v4_mfhi_div_stall{};
  v4_mfhi_div_stall.name = "v4_mfhi_div_stall";
  v4_mfhi_div_stall.start_pc = 0xA0010000u;
  v4_mfhi_div_stall.initial_gpr[1] = 100u;
  v4_mfhi_div_stall.initial_gpr[2] = 7u;
  v4_mfhi_div_stall.program = {
      enc_r(1, 2, 0, 0, 0x1A), // DIV r1,r2
      enc_r(0, 0, 3, 0, 0x10), // MFHI r3 must honor divide latency
  };
  v4_mfhi_div_stall.instructions = 2u;
  v4_mfhi_div_stall.require_v4_native_entry_when_available = true;
  v4_mfhi_div_stall.require_v4_muldiv_native_when_available = true;
  cases.push_back(v4_mfhi_div_stall);

  CpuCompareCase v4_div_edge_cases{};
  v4_div_edge_cases.name = "v4_div_edge_cases_native";
  v4_div_edge_cases.start_pc = 0xA0010000u;
  v4_div_edge_cases.initial_gpr[1] = 123u;
  v4_div_edge_cases.initial_gpr[2] = 0u;
  v4_div_edge_cases.initial_gpr[3] = 0xFFFFFF85u; // -123
  v4_div_edge_cases.initial_gpr[4] = 0x80000000u;
  v4_div_edge_cases.initial_gpr[5] = 0xFFFFFFFFu;
  v4_div_edge_cases.program = {
      enc_r(1, 2, 0, 0, 0x1A), // DIV +123,0
      enc_r(0, 0, 6, 0, 0x12), // MFLO = -1
      enc_r(0, 0, 7, 0, 0x10), // MFHI = +123
      enc_r(3, 2, 0, 0, 0x1A), // DIV -123,0
      enc_r(0, 0, 8, 0, 0x12), // MFLO = +1
      enc_r(0, 0, 9, 0, 0x10), // MFHI = -123
      enc_r(4, 5, 0, 0, 0x1A), // INT_MIN / -1
      enc_r(0, 0, 10, 0, 0x12),
      enc_r(0, 0, 11, 0, 0x10),
      enc_r(1, 2, 0, 0, 0x1B), // DIVU 123,0
      enc_r(0, 0, 12, 0, 0x12),
      enc_r(0, 0, 13, 0, 0x10),
  };
  v4_div_edge_cases.instructions = 12u;
  v4_div_edge_cases.require_v4_native_entry_when_available = true;
  v4_div_edge_cases.require_v4_muldiv_native_when_available = true;
  cases.push_back(v4_div_edge_cases);

  CpuCompareCase v4_cop0_transfers{};
  v4_cop0_transfers.name = "v4_cop0_transfers_native";
  v4_cop0_transfers.start_pc = 0xA0010000u;
  v4_cop0_transfers.initial_gpr[1] = 0x0040003Cu;
  v4_cop0_transfers.initial_gpr[2] = 0x00000000u;
  v4_cop0_transfers.program = {
      (0x10u << 26) | (4u << 21) | (1u << 16) | (12u << 11), // MTC0 r1,SR
      (0x10u << 26) | (0u << 21) | (3u << 16) | (12u << 11), // MFC0 SR,r3
      0,                                                     // retire load
      (0x10u << 26) | (4u << 21) | (2u << 16) | (13u << 11), // MTC0 r2,Cause
      (0x10u << 26) | (0u << 21) | (4u << 16) | (13u << 11), // MFC0 Cause,r4
      0,
      (0x10u << 26) | (0x10u << 21) | 0x10u,                 // RFE
      (0x10u << 26) | (0u << 21) | (5u << 16) | (12u << 11), // MFC0 SR,r5
      0,
      (0x10u << 26) | (0u << 21) | (6u << 16) | (15u << 11), // MFC0 PRId,r6
      0,
  };
  v4_cop0_transfers.instructions = 11u;
  v4_cop0_transfers.require_v4_native_entry_when_available = true;
  v4_cop0_transfers.require_v4_cop0_native_when_available = true;
  cases.push_back(v4_cop0_transfers);

  CpuCompareCase v4_uncached_incoming_load{};
  v4_uncached_incoming_load.name =
      "v4_uncached_native_incoming_load_delay";
  v4_uncached_incoming_load.start_pc = 0xA0010000u;
  v4_uncached_incoming_load.initial_gpr[2] = 0x11111111u;
  v4_uncached_incoming_load.initial_load_reg = 2u;
  v4_uncached_incoming_load.initial_load_value = 0x12345678u;
  v4_uncached_incoming_load.program = {
      // First instruction sees old r2; load retires after operand capture.
      enc_r(2, 0, 3, 0, 0x21),
      // Second instruction sees the committed load.
      enc_r(2, 0, 4, 0, 0x21),
  };
  pad_cpu_compare_program(v4_uncached_incoming_load, 32u);
  v4_uncached_incoming_load.require_v4_native_entry_when_available = true;
  cases.push_back(v4_uncached_incoming_load);

  CpuCompareCase v4_uncached_incoming_load_cancel{};
  v4_uncached_incoming_load_cancel.name =
      "v4_uncached_native_incoming_load_cancel";
  v4_uncached_incoming_load_cancel.start_pc = 0xA0010000u;
  v4_uncached_incoming_load_cancel.initial_gpr[2] = 0x11111111u;
  v4_uncached_incoming_load_cancel.initial_load_reg = 2u;
  v4_uncached_incoming_load_cancel.initial_load_value = 0x12345678u;
  v4_uncached_incoming_load_cancel.program = {
      // A same-register ALU write cancels the pending load.
      enc_i(0x09, 0, 2, 5),
      enc_r(2, 0, 3, 0, 0x21),
  };
  pad_cpu_compare_program(v4_uncached_incoming_load_cancel, 32u);
  v4_uncached_incoming_load_cancel.require_v4_native_entry_when_available =
      true;
  cases.push_back(v4_uncached_incoming_load_cancel);

  CpuCompareCase v4_uncached_prefix_load_tail{};
  v4_uncached_prefix_load_tail.name =
      "v4_uncached_native_alu_prefix_load_delay_tail";
  v4_uncached_prefix_load_tail.start_pc = 0xA0010000u;
  v4_uncached_prefix_load_tail.initial_gpr[1] = 0x80012000u;
  v4_uncached_prefix_load_tail.initial_gpr[2] = 0x00000010u;
  v4_uncached_prefix_load_tail.memory.push_back({0x00012004u, 0x12345678u});
  v4_uncached_prefix_load_tail.program = {
      enc_i(0x09, 1, 1, 4),  // ADDIU prefix: point at the load word
      enc_i(0x23, 1, 2, 0),  // LW r2
      enc_i(0x09, 2, 3, 1),  // load-delay slot must see old r2 => r3=0x11
      0xFFFFFFFFu,
  };
  v4_uncached_prefix_load_tail.instructions = 3u;
  v4_uncached_prefix_load_tail.require_v4_native_entry_when_available = true;
  v4_uncached_prefix_load_tail.require_v4_native_load_entry_when_available =
      true;
  v4_uncached_prefix_load_tail.require_v4_load_tail_block_when_available =
      true;
  cases.push_back(v4_uncached_prefix_load_tail);

  CpuCompareCase v4_uncached_prefix_load_branch{};
  v4_uncached_prefix_load_branch.name =
      "v4_uncached_native_alu_prefix_load_branch_delay";
  v4_uncached_prefix_load_branch.start_pc = 0xA0010000u;
  v4_uncached_prefix_load_branch.initial_gpr[1] = 0x800120A0u;
  v4_uncached_prefix_load_branch.initial_gpr[2] = 0u;
  v4_uncached_prefix_load_branch.memory.push_back({0x000120A4u, 1u});
  v4_uncached_prefix_load_branch.program = {
      enc_i(0x09, 1, 1, 4),       // ADDIU prefix
      enc_i(0x23, 1, 2, 0),       // LW r2 <- 1
      enc_i(0x05, 2, 0, 2),       // BNE must see old r2==0: not taken
      enc_i(0x09, 0, 4, 0x0044),  // delay slot sees committed load
      0xFFFFFFFFu,
      enc_i(0x09, 0, 5, 0x0055),
  };
  v4_uncached_prefix_load_branch.instructions = 4u;
  v4_uncached_prefix_load_branch.require_v4_native_entry_when_available = true;
  v4_uncached_prefix_load_branch.require_v4_native_load_entry_when_available =
      true;
  v4_uncached_prefix_load_branch.require_v4_native_branch_entry_when_available =
      true;
  v4_uncached_prefix_load_branch.require_v4_load_branch_fusion_when_available =
      true;
  cases.push_back(v4_uncached_prefix_load_branch);

  CpuCompareCase v4_uncached_prefix_unaligned_load{};
  v4_uncached_prefix_unaligned_load.name =
      "v4_uncached_native_alu_prefix_unaligned_load_exit";
  v4_uncached_prefix_unaligned_load.start_pc = 0xA0010000u;
  v4_uncached_prefix_unaligned_load.initial_gpr[1] = 0x80012100u;
  v4_uncached_prefix_unaligned_load.program = {
      enc_i(0x09, 1, 1, 1),  // prefix commits r1=...101
      enc_i(0x23, 1, 2, 0),  // unaligned LW -> interpreter exception exit
      0xFFFFFFFFu,
  };
  v4_uncached_prefix_unaligned_load.instructions = 2u;
  v4_uncached_prefix_unaligned_load.require_v4_native_entry_when_available =
      true;
  cases.push_back(v4_uncached_prefix_unaligned_load);

  CpuCompareCase v4_uncached_load_tail{};
  v4_uncached_load_tail.name = "v4_uncached_native_load_alu_tail";
  v4_uncached_load_tail.start_pc = 0xA0010000u;
  v4_uncached_load_tail.initial_gpr[1] = 0x80012000u;
  v4_uncached_load_tail.initial_gpr[2] = 0x11111111u;
  v4_uncached_load_tail.memory.push_back({0x00012000u, 0x12345678u});
  v4_uncached_load_tail.program = {
      enc_i(0x23, 1, 2, 0),       // LW schedules r2
      enc_r(2, 0, 3, 0, 0x21),    // sees old r2
      enc_r(2, 0, 4, 0, 0x21),    // sees loaded r2
      0x0000000Cu,                 // SYSCALL terminates native decode
  };
  v4_uncached_load_tail.instructions = 3u;
  v4_uncached_load_tail.require_v4_native_entry_when_available = true;
  v4_uncached_load_tail.require_v4_native_load_entry_when_available = true;
  v4_uncached_load_tail.require_v4_load_tail_block_when_available = true;
  cases.push_back(v4_uncached_load_tail);

  CpuCompareCase v4_uncached_load_widths{};
  v4_uncached_load_widths.name = "v4_uncached_native_load_widths";
  v4_uncached_load_widths.start_pc = 0xA0010000u;
  v4_uncached_load_widths.initial_gpr[1] = 0x80012000u;
  // Little-endian bytes: 01 7F FF 80.
  v4_uncached_load_widths.memory.push_back({0x00012000u, 0x80FF7F01u});
  v4_uncached_load_widths.program = {
      enc_i(0x20, 1, 2, 2), // LB   -> FFFFFFFF
      enc_i(0x24, 1, 3, 3), // LBU  -> 00000080
      enc_i(0x21, 1, 4, 2), // LH   -> FFFF80FF
      enc_i(0x25, 1, 5, 2), // LHU  -> 000080FF
      0,
  };
  v4_uncached_load_widths.instructions = 5u;
  v4_uncached_load_widths.require_v4_native_entry_when_available = true;
  v4_uncached_load_widths.require_v4_native_load_entry_when_available = true;
  cases.push_back(v4_uncached_load_widths);

  CpuCompareCase v4_cached_load_alu_branch{};
  v4_cached_load_alu_branch.name =
      "v4_cached_fused_load_alu_branch_delay";
  v4_cached_load_alu_branch.start_pc = 0x80010000u;
  v4_cached_load_alu_branch.initial_gpr[1] = 0x80012000u;
  v4_cached_load_alu_branch.initial_gpr[2] = 0x00000010u;
  v4_cached_load_alu_branch.memory.push_back({0x00012000u, 0x00000020u});
  v4_cached_load_alu_branch.program = {
      enc_i(0x23, 1, 2, 0),
      enc_i(0x09, 2, 3, 1),
      enc_i(0x05, 2, 0, 2),
      enc_i(0x09, 0, 5, 0x55),
      0u,
  };
  v4_cached_load_alu_branch.instructions = 4u;
  v4_cached_load_alu_branch.require_v4_native_entry_when_available = true;
  v4_cached_load_alu_branch.require_v4_native_load_entry_when_available = true;
  v4_cached_load_alu_branch.require_v4_native_branch_entry_when_available = true;
  v4_cached_load_alu_branch.require_v4_load_branch_fusion_when_available = true;
  cases.push_back(v4_cached_load_alu_branch);

  CpuCompareCase v4_uncached_load_branch_delay{};
  v4_uncached_load_branch_delay.name =
      "v4_uncached_fused_load_branch_delay_capture";
  v4_uncached_load_branch_delay.start_pc = 0xA0010000u;
  v4_uncached_load_branch_delay.initial_gpr[1] = 0x80012000u;
  v4_uncached_load_branch_delay.initial_gpr[2] = 0u;
  v4_uncached_load_branch_delay.memory.push_back({0x00012000u, 1u});
  v4_uncached_load_branch_delay.program = {
      enc_i(0x23, 1, 2, 0),
      enc_i(0x04, 2, 0, 2),
      enc_r(2, 0, 5, 0, 0x21),
      0u,
  };
  v4_uncached_load_branch_delay.instructions = 3u;
  v4_uncached_load_branch_delay.require_v4_native_entry_when_available = true;
  v4_uncached_load_branch_delay.require_v4_native_load_entry_when_available =
      true;
  v4_uncached_load_branch_delay.require_v4_native_branch_entry_when_available =
      true;
  v4_uncached_load_branch_delay.require_v4_load_branch_fusion_when_available =
      true;
  cases.push_back(v4_uncached_load_branch_delay);

  CpuCompareCase v4_uncached_store_widths{};
  v4_uncached_store_widths.name = "v4_uncached_native_store_widths";
  v4_uncached_store_widths.start_pc = 0xA0010000u;
  v4_uncached_store_widths.initial_gpr[1] = 0x80012000u;
  v4_uncached_store_widths.initial_gpr[2] = 0xA1B2C3D4u;
  v4_uncached_store_widths.memory.push_back({0x00012000u, 0x11223344u});
  v4_uncached_store_widths.compare_memory_addresses.push_back(0x00012000u);
  v4_uncached_store_widths.program = {
      enc_i(0x28, 1, 2, 0), // SB
      enc_i(0x29, 1, 2, 2), // SH
      enc_i(0x2B, 1, 2, 0), // SW
      0u,
  };
  v4_uncached_store_widths.instructions = 4u;
  v4_uncached_store_widths.require_v4_native_entry_when_available = true;
  v4_uncached_store_widths.require_v4_native_store_entry_when_available = true;
  cases.push_back(v4_uncached_store_widths);

  CpuCompareCase v4_uncached_store_pending_load{};
  v4_uncached_store_pending_load.name =
      "v4_uncached_native_store_pending_load_source";
  v4_uncached_store_pending_load.start_pc = 0xA0010000u;
  v4_uncached_store_pending_load.initial_gpr[1] = 0x80012020u;
  v4_uncached_store_pending_load.initial_gpr[2] = 0x11223344u;
  v4_uncached_store_pending_load.initial_load_reg = 2u;
  v4_uncached_store_pending_load.initial_load_value = 0xAABBCCDDu;
  v4_uncached_store_pending_load.memory.push_back({0x00012020u, 0u});
  v4_uncached_store_pending_load.compare_memory_addresses.push_back(
      0x00012020u);
  v4_uncached_store_pending_load.program = {
      enc_i(0x2B, 1, 2, 0), // must store old r2, then delayed load commits
      0xFFFFFFFFu,           // stop V4 tail formation after the tested store
  };
  v4_uncached_store_pending_load.instructions = 1u;
  v4_uncached_store_pending_load.require_v4_native_entry_when_available = true;
  v4_uncached_store_pending_load.require_v4_native_store_entry_when_available =
      true;
  cases.push_back(v4_uncached_store_pending_load);

  CpuCompareCase v4_uncached_store_alu_tail{};
  v4_uncached_store_alu_tail.name = "v4_uncached_native_store_alu_tail";
  v4_uncached_store_alu_tail.start_pc = 0xA0010000u;
  v4_uncached_store_alu_tail.initial_gpr[1] = 0x80012040u;
  v4_uncached_store_alu_tail.initial_gpr[2] = 0x12345678u;
  v4_uncached_store_alu_tail.memory.push_back({0x00012040u, 0u});
  v4_uncached_store_alu_tail.compare_memory_addresses.push_back(0x00012040u);
  v4_uncached_store_alu_tail.program = {
      enc_i(0x2B, 1, 2, 0),       // SW
      enc_i(0x09, 2, 3, 1),       // ADDIU
      enc_i(0x0D, 3, 4, 0x0040),  // ORI
      0xFFFFFFFFu,                 // stop after the intended fused tail
  };
  v4_uncached_store_alu_tail.instructions = 3u;
  v4_uncached_store_alu_tail.require_v4_native_entry_when_available = true;
  v4_uncached_store_alu_tail.require_v4_native_store_entry_when_available = true;
  v4_uncached_store_alu_tail.require_v4_store_tail_block_when_available = true;
  cases.push_back(v4_uncached_store_alu_tail);

  CpuCompareCase v4_uncached_prefix_store_tail{};
  v4_uncached_prefix_store_tail.name =
      "v4_uncached_native_alu_prefix_store_tail";
  v4_uncached_prefix_store_tail.start_pc = 0xA0010000u;
  v4_uncached_prefix_store_tail.initial_gpr[1] = 0x80012080u;
  v4_uncached_prefix_store_tail.initial_gpr[2] = 0xCAFEBABEu;
  v4_uncached_prefix_store_tail.memory.push_back({0x00012084u, 0u});
  v4_uncached_prefix_store_tail.compare_memory_addresses.push_back(0x00012084u);
  v4_uncached_prefix_store_tail.program = {
      enc_i(0x09, 1, 1, 4),       // ADDIU prefix
      enc_i(0x2B, 1, 2, 0),       // SW
      enc_i(0x09, 2, 3, 1),       // ALU tail
      0xFFFFFFFFu,
  };
  v4_uncached_prefix_store_tail.instructions = 3u;
  v4_uncached_prefix_store_tail.require_v4_native_entry_when_available = true;
  v4_uncached_prefix_store_tail.require_v4_native_store_entry_when_available =
      true;
  v4_uncached_prefix_store_tail.require_v4_store_tail_block_when_available =
      true;
  cases.push_back(v4_uncached_prefix_store_tail);

  CpuCompareCase v4_uncached_prefix_store_branch{};
  v4_uncached_prefix_store_branch.name =
      "v4_uncached_native_alu_prefix_store_branch_delay";
  v4_uncached_prefix_store_branch.start_pc = 0xA0010000u;
  v4_uncached_prefix_store_branch.initial_gpr[1] = 0x800120C0u;
  v4_uncached_prefix_store_branch.initial_gpr[2] = 0x0BADF00Du;
  v4_uncached_prefix_store_branch.initial_gpr[3] = 1u;
  v4_uncached_prefix_store_branch.memory.push_back({0x000120C4u, 0u});
  v4_uncached_prefix_store_branch.compare_memory_addresses.push_back(0x000120C4u);
  v4_uncached_prefix_store_branch.program = {
      enc_i(0x09, 1, 1, 4),       // ADDIU prefix
      enc_i(0x2B, 1, 2, 0),       // SW
      enc_i(0x05, 3, 0, 1),       // BNE taken
      enc_i(0x09, 0, 4, 0x0044),  // delay slot
      0xFFFFFFFFu,
  };
  v4_uncached_prefix_store_branch.instructions = 4u;
  v4_uncached_prefix_store_branch.require_v4_native_entry_when_available = true;
  v4_uncached_prefix_store_branch.require_v4_native_store_entry_when_available =
      true;
  v4_uncached_prefix_store_branch.require_v4_native_branch_entry_when_available =
      true;
  v4_uncached_prefix_store_branch.require_v4_store_branch_fusion_when_available =
      true;
  cases.push_back(v4_uncached_prefix_store_branch);

  CpuCompareCase v4_uncached_prefix_unaligned_store{};
  v4_uncached_prefix_unaligned_store.name =
      "v4_uncached_native_alu_prefix_unaligned_store_exit";
  v4_uncached_prefix_unaligned_store.start_pc = 0xA0010000u;
  v4_uncached_prefix_unaligned_store.initial_gpr[1] = 0x80012120u;
  v4_uncached_prefix_unaligned_store.initial_gpr[2] = 0x12345678u;
  v4_uncached_prefix_unaligned_store.program = {
      enc_i(0x09, 1, 1, 1),  // prefix commits r1=...121
      enc_i(0x2B, 1, 2, 0),  // unaligned SW -> interpreter exception exit
      0xFFFFFFFFu,
  };
  v4_uncached_prefix_unaligned_store.instructions = 2u;
  v4_uncached_prefix_unaligned_store.require_v4_native_entry_when_available =
      true;
  cases.push_back(v4_uncached_prefix_unaligned_store);

  CpuCompareCase v4_uncached_store_branch{};
  v4_uncached_store_branch.name = "v4_uncached_native_store_branch_delay";
  v4_uncached_store_branch.start_pc = 0xA0010000u;
  v4_uncached_store_branch.initial_gpr[1] = 0x80012060u;
  v4_uncached_store_branch.initial_gpr[2] = 0x89ABCDEFu;
  v4_uncached_store_branch.initial_gpr[3] = 1u;
  v4_uncached_store_branch.memory.push_back({0x00012060u, 0u});
  v4_uncached_store_branch.compare_memory_addresses.push_back(0x00012060u);
  v4_uncached_store_branch.program = {
      enc_i(0x2B, 1, 2, 0),       // SW
      enc_i(0x09, 3, 3, 1),       // ADDIU tail => r3 = 2
      enc_i(0x05, 3, 0, 1),       // BNE taken
      enc_i(0x09, 0, 4, 0x0044),  // delay slot
      0xFFFFFFFFu,
  };
  v4_uncached_store_branch.instructions = 4u;
  v4_uncached_store_branch.require_v4_native_entry_when_available = true;
  v4_uncached_store_branch.require_v4_native_store_entry_when_available = true;
  v4_uncached_store_branch.require_v4_native_branch_entry_when_available = true;
  v4_uncached_store_branch.require_v4_store_branch_fusion_when_available = true;
  cases.push_back(v4_uncached_store_branch);

  CpuCompareCase v4_uncached_store_smc{};
  v4_uncached_store_smc.name = "v4_uncached_store_code_page_native";
  v4_uncached_store_smc.start_pc = 0xA0010000u;
  v4_uncached_store_smc.initial_gpr[1] =
      v4_uncached_store_smc.start_pc + 0x0Cu;
  v4_uncached_store_smc.initial_gpr[2] = 0xCAFEBABEu;
  v4_uncached_store_smc.memory.push_back({0x0001000Cu, 0u});
  v4_uncached_store_smc.compare_memory_addresses.push_back(0x0001000Cu);
  v4_uncached_store_smc.program = {
      enc_i(0x2B, 1, 2, 0),
  };
  v4_uncached_store_smc.instructions = 1u;
  v4_uncached_store_smc.require_v4_store_smc_native_when_available = true;
  cases.push_back(v4_uncached_store_smc);

  CpuCompareCase v4_uncached_store_same_page_other_line{};
  v4_uncached_store_same_page_other_line.name =
      "v4_uncached_store_same_page_other_line_fast";
  v4_uncached_store_same_page_other_line.start_pc = 0xA0010000u;
  v4_uncached_store_same_page_other_line.initial_gpr[1] =
      v4_uncached_store_same_page_other_line.start_pc + 0x40u;
  v4_uncached_store_same_page_other_line.initial_gpr[2] = 0x13579BDFu;
  v4_uncached_store_same_page_other_line.memory.push_back(
      {0x00010040u, 0u});
  v4_uncached_store_same_page_other_line.compare_memory_addresses.push_back(
      0x00010040u);
  v4_uncached_store_same_page_other_line.program = {
      enc_i(0x2B, 1, 2, 0),
      0xFFFFFFFFu,
  };
  v4_uncached_store_same_page_other_line.instructions = 1u;
  v4_uncached_store_same_page_other_line.require_v4_native_entry_when_available =
      true;
  v4_uncached_store_same_page_other_line
      .require_v4_native_store_entry_when_available = true;
  cases.push_back(v4_uncached_store_same_page_other_line);

  CpuCompareCase v4_uncached_resident_chain{};
  v4_uncached_resident_chain.name = "v4_uncached_resident_chain";
  v4_uncached_resident_chain.start_pc = 0xA0010000u;
  // 32 baseline-native NOPs form block A. The J + delay-slot NOP form block B.
  // On the first trip A and B are compiled separately; once B jumps back to A,
  // the resident x64 path must take a direct known edge without returning to C++.
  v4_uncached_resident_chain.program.assign(32u, 0u);
  v4_uncached_resident_chain.program.push_back(
      enc_j(0x02, v4_uncached_resident_chain.start_pc));
  v4_uncached_resident_chain.program.push_back(0u);
  v4_uncached_resident_chain.instructions = 100u;
  v4_uncached_resident_chain.require_v4_native_entry_when_available = true;
  v4_uncached_resident_chain.require_v4_native_branch_entry_when_available =
      true;
  v4_uncached_resident_chain.require_v4_native_chain_when_available = true;
  cases.push_back(v4_uncached_resident_chain);

  append_deterministic_random_compare_cases(cases);
  // ── Event scheduler: sync-on-access ────────────────────────────────
  // Devices are advanced lazily, so an MMIO read must first bring the device
  // up to the exact start-of-instruction cycle of the access. Timer 2 counts
  // the system clock from the MODE write (which resets it), so a later counter
  // read has to return precisely the cycles elapsed between the two accesses,
  // in both backends and on both the hot 16-bit bridge and the general path.
  struct TimerReadVariant {
    const char *name;
    u32 load_op;
    bool hot16;
  };
  const TimerReadVariant timer_read_variants[] = {
      {"sched_timer_counter_lhu_exact", 0x25, true},
      {"sched_timer_counter_lw_exact", 0x23, false},
  };
  for (const TimerReadVariant &variant : timer_read_variants) {
    CpuCompareCase test{};
    test.name = variant.name;
    test.start_pc = 0xA0010000u;
    test.initial_gpr[1] = 0x1F801120u;
    test.program.push_back(enc_i(0x2B, 1, 0, 4)); // SW r0 -> T2 MODE (resets)
    for (int i = 0; i < 20; ++i) {
      test.program.push_back(0); // NOPs
    }
    test.program.push_back(enc_i(variant.load_op, 1, 2, 0)); // read T2 COUNTER
    test.program.push_back(0);                               // retire load delay
    test.instructions = static_cast<u32>(test.program.size());
    test.segment_instructions = {21u, 1u, 1u};
    test.compare_segment_states = true;
    test.expect_gpr_reg = 2;
    test.expect_gpr_mask = 0xFFFFu;
    test.expect_gpr_is_segment_cycles = true;
    test.expect_gpr_segment = 0;
    test.require_v4_native_entry_when_available = true;
    test.require_v4_native_load_entry_when_available = true;
    test.require_v4_hot_mmio16_native_when_available = variant.hot16;
    cases.push_back(test);
  }

  // A CD-ROM command's INT3 must be visible at the first flag read after its
  // deadline and not before, even though the drive was never ticked between.
  struct CdReadVariant {
    const char *name;
    u32 loop_iterations;
    u32 expected_flag;
  };
  const CdReadVariant cd_read_variants[] = {
      {"sched_cdrom_flag_before_deadline", 100u, 0u},
      {"sched_cdrom_flag_after_deadline", 6000u, 3u},
  };
  for (const CdReadVariant &variant : cd_read_variants) {
    CpuCompareCase test{};
    test.name = variant.name;
    test.start_pc = 0xA0010000u;
    test.initial_gpr[1] = 0x1F801800u;
    test.initial_gpr[3] = variant.loop_iterations;
    test.initial_gpr[4] = 1u;
    test.initial_gpr[5] = 0x1Fu;
    test.program = {
        enc_i(0x28, 1, 4, 0),      // SB index = 1
        enc_i(0x28, 1, 5, 2),      // SB interrupt enable = 0x1F
        enc_i(0x28, 1, 0, 0),      // SB index = 0
        enc_i(0x28, 1, 4, 1),      // SB command = GetStat
        enc_i(0x09, 3, 3, 0xFFFF), // loop: ADDIU r3,r3,-1
        enc_i(0x05, 3, 0, 0xFFFE), // BNE r3,r0,loop
        0,                         //   delay slot
        enc_i(0x28, 1, 4, 0),      // SB index = 1
        enc_i(0x24, 1, 2, 3),      // LBU r2 = interrupt flag register
        0,                         // retire load delay
    };
    test.instructions = 4u + variant.loop_iterations * 3u + 3u;
    test.expect_gpr_reg = 2;
    test.expect_gpr_mask = 0x7u;
    test.expect_gpr_value = variant.expected_flag;
    test.require_v4_native_entry_when_available = true;
    cases.push_back(test);
  }

  // RAM is mirrored through the 8 MB window selected by RAM_SIZE; every mirror
  // is real RAM and carries the main-RAM bus penalty. The interpreter used to
  // charge it only below 2 MB (THPS2 keeps its stack at 0x807FFFF0), which made
  // the two backends disagree on timing.
  {
    CpuCompareCase store_case{};
    store_case.name = "mirrored_ram_store_penalty_8mb_window";
    store_case.start_pc = 0xA0010000u;
    store_case.initial_gpr[1] = 0x807FFFD0u;
    store_case.initial_gpr[3] = 0x12345678u;
    store_case.program = {
        enc_i(0x2B, 1, 3, 0x20), // SW r3, 0x20(r1) -> 0x807FFFF0
        enc_i(0x23, 1, 4, 0x20), // LW r4, 0x20(r1)
        0,                       // retire load delay
    };
    store_case.instructions = 3u;
    store_case.compare_memory_addresses.push_back(0x001FFFF0u);
    store_case.segment_instructions = {1u, 1u, 1u};
    store_case.compare_segment_states = true;
    store_case.expect_gpr_reg = 4;
    store_case.expect_gpr_value = 0x12345678u;
    cases.push_back(store_case);
  }

  return cases;
}

static int run_cpu_backend_compare_test_impl(bool memory_only = false) {
  LOG_INFO("=== CPU Backend Compare Test ===");
  const bool saved_override = g_cpu_execution_mode_cli_override;
  const CpuExecutionMode saved_override_value = g_cpu_execution_mode_cli_value;
  const bool saved_unknown_fallback =
      g_experimental_unhandled_special_returns_zero;
  const bool saved_force_native = g_cpu_x64_jit_force_compile;
  const bool saved_branch_tail_enabled = g_cpu_x64_jit_branch_tail_enabled;
  const bool saved_branch_tail_cli_override =
      g_cpu_x64_jit_branch_tail_cli_override;
  const bool saved_branch_tail_cli_value =
      g_cpu_x64_jit_branch_tail_cli_value;
  const std::vector<u32> saved_branch_tail_blacklist =
      g_cpu_x64_jit_branch_tail_blacklist;
  const bool saved_compare_irq_on_branch =
      g_cpu_backend_compare_irq_on_branch;
  const bool saved_compare_partial_branch_tail =
      g_cpu_backend_compare_allow_partial_branch_tail;
  const bool saved_compare_partial_memory =
      g_cpu_backend_compare_allow_partial_memory_helper;
  const bool saved_compare_test_active = g_cpu_backend_compare_test_active;
  const bool saved_all_native_cli_override =
      g_cpu_x64_jit_all_native_cli_override;
  const bool saved_all_native_cli_value =
      g_cpu_x64_jit_all_native_cli_value;
  const bool saved_memory_native_cli_override =
      g_cpu_x64_jit_native_memory_cli_override;
  const bool saved_memory_native_cli_value =
      g_cpu_x64_jit_native_memory_cli_value;
  const bool saved_alu_native_cli_override =
      g_cpu_x64_jit_native_alu_cli_override;
  const bool saved_alu_native_cli_value =
      g_cpu_x64_jit_native_alu_cli_value;
  const bool saved_ram_load_fastpath =
      g_cpu_x64_jit_ram_load_fastpath_enabled;
  const bool saved_reduced_helper_branch_tail =
      g_cpu_x64_jit_reduced_helper_branch_tail_enabled;
  const bool saved_aggressive_reduced_helper_branch_tail =
      g_cpu_x64_jit_aggressive_reduced_helper_branch_tail_enabled;
  const bool saved_native_prefix = g_cpu_x64_jit_native_prefix_enabled;
  const bool saved_aggressive_native_prefix_ram =
      g_cpu_x64_jit_aggressive_native_prefix_ram_enabled;
  const bool saved_aggressive_native_prefix_ram_cli_override =
      g_cpu_x64_jit_aggressive_native_prefix_ram_cli_override;
  const bool saved_aggressive_native_prefix_ram_cli_value =
      g_cpu_x64_jit_aggressive_native_prefix_ram_cli_value;
  g_cpu_backend_compare_test_active = true;
  g_cpu_x64_jit_force_compile = true;
  g_cpu_x64_jit_aggressive_native_prefix_ram_cli_override = false;

  LOG_INFO(
      "CPU backend compare: reference=Interpreter targets=Recompiler%s",
      memory_only ? " scope=memory-only" : "");
  int failures = 0;
  if (!run_gte_final_accumulator_regression()) {
    ++failures;
  }
  if (!run_gte_register_sign_extension_regression()) {
    ++failures;
  }
  if (!run_gte_writable_mac_regression()) {
    ++failures;
  }
  const std::array<CpuExecutionMode, 2> modes = {
      CpuExecutionMode::Interpreter,
      CpuExecutionMode::Recompiler,
  };

  for (const CpuCompareCase &test_case : make_cpu_compare_cases()) {
    if (memory_only) {
      const std::string_view name(test_case.name);
      const bool memory_case =
          test_case.require_native_memory_helper_when_available ||
          test_case.require_native_memory_exception_when_available ||
          test_case.require_native_memory_tier_entry_when_available ||
          test_case
              .require_native_reduced_helper_ram_load_entry_when_available ||
          test_case
              .require_native_reduced_helper_branch_tail_ram_load_entry_when_available ||
          test_case
              .require_native_aggressive_reduced_helper_branch_tail_entry_when_available ||
          test_case
              .require_native_aggressive_reduced_helper_branch_tail_store_entry_when_available ||
          test_case
              .require_native_aggressive_reduced_helper_branch_tail_mixed_entry_when_available ||
          test_case.require_native_prefix_ram_load_entry_when_available ||
          test_case
              .require_native_prefix_ram_load_preflight_non_ram_when_available ||
          test_case
              .require_native_prefix_ram_load_aggressive_entry_when_available ||
          test_case
              .require_native_prefix_ram_load_full_preflight_when_available ||
          test_case
              .require_native_prefix_ram_load_adaptive_disable_when_available ||
          test_case
              .require_native_prefix_ram_load_adaptive_direct_entry_when_available ||
          test_case.require_native_prefix_reject_store_when_available ||
          test_case.require_reduced_helper_preflight_mmio_when_available ||
          test_case.require_reduced_helper_preflight_unaligned_when_available ||
          test_case.require_reduced_helper_preflight_non_ram_when_available ||
          test_case
              .require_reduced_helper_branch_tail_preflight_mmio_when_available ||
          test_case
              .require_reduced_helper_branch_tail_preflight_unaligned_when_available ||
          test_case
              .require_reduced_helper_branch_tail_preflight_non_ram_when_available ||
          test_case
              .require_reduced_helper_branch_tail_reject_load_base_written_when_available ||
          test_case
              .require_aggressive_reduced_helper_branch_tail_preflight_non_ram_when_available ||
          test_case
              .require_aggressive_reduced_helper_branch_tail_preflight_code_page_when_available ||
          test_case
              .require_aggressive_reduced_helper_branch_tail_direct_preflight_when_available ||
          test_case
              .require_aggressive_reduced_helper_branch_tail_full_preflight_when_available ||
          test_case
              .require_aggressive_reduced_helper_branch_tail_adaptive_disable_when_available ||
          test_case
              .require_aggressive_reduced_helper_branch_tail_adaptive_direct_entry_when_available ||
          test_case.disable_memory_native_for_x64 ||
          name.find("memory") != std::string_view::npos ||
          name.find("mmio") != std::string_view::npos;
      if (!memory_case) {
        continue;
      }
    }
    g_experimental_unhandled_special_returns_zero =
        test_case.experimental_unknown_fallback;

    CpuCompareRunResult reference =
        run_cpu_compare_case_once(test_case, CpuExecutionMode::Interpreter);

    for (CpuExecutionMode mode : modes) {
      CpuCompareRunResult result =
          (mode == CpuExecutionMode::Interpreter)
              ? reference
              : run_cpu_compare_case_once(test_case, mode);

      bool pass = cpu_debug_states_equal(reference.state, result.state);
      const bool state_pass = pass;
      bool segment_state_pass = true;
      bool segment_peripheral_pass = true;
      if (test_case.compare_segment_states) {
        segment_state_pass =
            reference.segment_states.size() == result.segment_states.size();
        const size_t segment_count = std::min(reference.segment_states.size(),
                                              result.segment_states.size());
        for (size_t segment = 0; segment < segment_count; ++segment) {
          if (!cpu_debug_states_equal(reference.segment_states[segment],
                                      result.segment_states[segment])) {
            segment_state_pass = false;
            // Compact mode reports only the final per-case summary.
          }
        }
        segment_peripheral_pass =
            reference.segment_peripherals.size() ==
            result.segment_peripherals.size();
        const size_t peripheral_segment_count =
            std::min(reference.segment_peripherals.size(),
                     result.segment_peripherals.size());
        for (size_t segment = 0; segment < peripheral_segment_count;
             ++segment) {
          if (!cpu_compare_peripherals_equal(
                  reference.segment_peripherals[segment],
                  result.segment_peripherals[segment])) {
            segment_peripheral_pass = false;
          }
        }
      }
      const bool irq_state_pass =
          reference.irq_stat == result.irq_stat &&
          reference.irq_mask == result.irq_mask;
      const bool memory_state_pass =
          reference.memory_values == result.memory_values;
      const bool peripheral_state_pass = cpu_compare_peripherals_equal(
          reference.peripherals, result.peripherals);
      const bool expected_state_pass =
          cpu_compare_expected_state_pass(test_case, mode, result);
      bool native_check_pass = true;
      const char *native_check = "not_required";

      if (mode == CpuExecutionMode::Recompiler &&
                  test_case.require_v4_store_smc_native_when_available) {
        if (!result.stats.native_available) {
          native_check = "skip_v4_native_unavailable";
        } else {
          const bool native_smc =
              result.stats.native_memory_blocks_compiled != 0u &&
              result.stats.native_memory_fastpath_stores != 0u &&
              result.stats.jit_v4_helper_instructions == 0u &&
              result.stats.fallback_instructions == 0u &&
              result.stats.interpreter_fallback_steps == 0u;
          native_check = native_smc ? "v4_store_smc_native"
                                    : "v4_store_smc_helper";
          native_check_pass = native_smc;
        }
      } else if (mode == CpuExecutionMode::Recompiler &&
                 test_case.require_v4_entry_exception_native_when_available) {
        if (!result.stats.native_available) {
          native_check = "skip_v4_native_unavailable";
        } else {
          const bool native_entry_exception =
              result.stats.native_instructions >= test_case.instructions &&
              result.stats.jit_v4_helper_instructions == 0u &&
              result.stats.fallback_instructions == 0u &&
              result.stats.interpreter_fallback_steps == 0u;
          native_check = native_entry_exception
                             ? "v4_entry_exception_native"
                             : "v4_entry_exception_helper";
          native_check_pass = native_entry_exception;
        }
      } else if (mode == CpuExecutionMode::Recompiler &&
          (test_case.require_v4_native_entry_when_available ||
           test_case.require_v4_native_load_entry_when_available ||
           test_case.require_v4_native_store_entry_when_available ||
           test_case.require_v4_store_tail_block_when_available ||
           test_case.require_v4_store_branch_fusion_when_available ||
           test_case.require_v4_load_tail_block_when_available ||
           test_case.require_v4_load_branch_fusion_when_available ||
           test_case.require_v4_native_branch_entry_when_available ||
           test_case.require_v4_pending_delay_native_when_available ||
           test_case.require_v4_hot_mmio16_native_when_available ||
            test_case.require_v4_mmio_native_when_available ||
           test_case.require_v4_hilo_native_when_available ||
            test_case.require_v4_muldiv_native_when_available ||
            test_case.require_v4_cop0_native_when_available ||
             test_case.require_v4_cop2_native_when_available ||
            test_case.require_v4_unaligned_native_when_available ||
            test_case.require_v4_exception_native_when_available ||
           test_case.require_v4_folded_branch_block_when_available ||
           test_case.require_v4_page_local_invalidation_when_available ||
           test_case.require_v4_cached_same_page_retention_when_available ||
           test_case.require_v4_icache_revalidation_when_available ||
           test_case.require_v4_native_icache_revalidation_when_available ||
           test_case.require_v4_native_chain_when_available ||
           test_case.require_v4_crossline_block_when_available ||
           test_case.require_v4_constant_address_memory_when_available)) {
        if (!result.stats.native_available) {
          native_check = "skip_v4_native_unavailable";
        } else {
          const bool native_entered =
              result.stats.native_blocks_compiled != 0 &&
              result.stats.native_block_entries != 0 &&
              result.stats.native_instructions != 0 &&
              result.stats.native_code_bytes != 0;
          const bool load_entered =
              !test_case.require_v4_native_load_entry_when_available ||
              result.stats.native_memory_fastpath_loads != 0;
          const bool store_entered =
              !test_case.require_v4_native_store_entry_when_available ||
              result.stats.native_memory_fastpath_stores != 0;
          const bool store_tail_folded =
              !test_case.require_v4_store_tail_block_when_available ||
              (result.stats.native_memory_blocks_compiled == 1u &&
               result.stats.native_alu_blocks_compiled == 0u &&
               result.stats.native_block_entries == 1u);
          const bool store_branch_fused =
              !test_case.require_v4_store_branch_fusion_when_available ||
              (result.stats.native_memory_blocks_compiled == 1u &&
               result.stats.native_branch_tail_blocks_compiled == 1u &&
               result.stats.native_alu_blocks_compiled == 0u &&
               result.stats.native_block_entries == 1u);
          const bool load_tail_folded =
              !test_case.require_v4_load_tail_block_when_available ||
              (result.stats.native_memory_blocks_compiled == 1u &&
               result.stats.native_alu_blocks_compiled == 0u &&
               result.stats.native_block_entries == 1u);
          const bool load_branch_fused =
              !test_case.require_v4_load_branch_fusion_when_available ||
              (result.stats.native_memory_blocks_compiled == 1u &&
               result.stats.native_branch_tail_blocks_compiled == 1u &&
               result.stats.native_alu_blocks_compiled == 0u &&
               result.stats.native_block_entries == 1u);
          const bool branch_entered =
              !test_case.require_v4_native_branch_entry_when_available ||
              result.stats.native_branch_tail_entries != 0;
          const bool pending_delay_native =
              !test_case.require_v4_pending_delay_native_when_available ||
              result.stats.jit_v4_helper_instructions == 0u;
          const bool hot_mmio16_native =
              !test_case.require_v4_hot_mmio16_native_when_available ||
              result.stats.jit_v4_helper_instructions == 0u;
          const bool mmio_native =
              !test_case.require_v4_mmio_native_when_available ||
              result.stats.jit_v4_helper_instructions == 0u;
          const bool hilo_native =
              !test_case.require_v4_hilo_native_when_available ||
              result.stats.jit_v4_helper_instructions == 0u;
          const bool muldiv_native =
              !test_case.require_v4_muldiv_native_when_available ||
              result.stats.jit_v4_helper_instructions == 0u;
          const bool cop0_native =
              !test_case.require_v4_cop0_native_when_available ||
              result.stats.jit_v4_helper_instructions == 0u;
          const bool cop2_native =
              !test_case.require_v4_cop2_native_when_available ||
              result.stats.jit_v4_helper_instructions == 0u;
          const bool unaligned_native =
              !test_case.require_v4_unaligned_native_when_available ||
              result.stats.jit_v4_helper_instructions == 0u;
          const bool exception_native =
              !test_case.require_v4_exception_native_when_available ||
              result.stats.jit_v4_helper_instructions == 0u;
          const bool folded_branch =
              !test_case.require_v4_folded_branch_block_when_available ||
              (result.stats.native_branch_tail_blocks_compiled != 0 &&
               result.stats.native_alu_blocks_compiled == 0);
          const bool page_local_invalidation =
              !test_case.require_v4_page_local_invalidation_when_available ||
              (result.stats.invalidations != 0u &&
               result.stats.native_blocks_compiled >= 2u &&
               result.stats.block_count == 1u);
          const bool cached_same_page_retained =
              !test_case.require_v4_cached_same_page_retention_when_available ||
              (result.stats.invalidations != 0u &&
               result.stats.native_blocks_compiled == 1u &&
               result.stats.block_count == 1u);
          const bool icache_revalidated =
              !test_case.require_v4_icache_revalidation_when_available ||
              (result.stats.native_blocks_compiled == 1u &&
               result.stats.block_count == 1u &&
               result.stats.cache_misses != 0u &&
               result.stats.cache_hits != 0u);
          const bool native_icache_revalidated =
              !test_case.require_v4_native_icache_revalidation_when_available ||
              (result.stats.native_blocks_compiled == 2u &&
               result.stats.native_dispatch_generation_exits == 0u &&
               result.stats.recompiler_frame_revalidate_successes >= 3u &&
               result.stats.recompiler_frame_icache_refills >= 3u);
          const bool chain_entered =
              !test_case.require_v4_native_chain_when_available ||
              (result.stats.native_chain_entries != 0 &&
               result.stats.native_linked_transitions != 0 &&
               result.stats.native_direct_link_transitions != 0 &&
               result.stats.native_chain_max_blocks > 1u);
          bool crossline_block =
              !test_case.require_v4_crossline_block_when_available;
          for (size_t size = 13u;
               !crossline_block &&
               size < result.stats.native_compiled_block_size_histogram.size();
               ++size) {
            crossline_block =
                result.stats.native_compiled_block_size_histogram[size] != 0u;
          }
          const bool constant_address_memory =
              !test_case.require_v4_constant_address_memory_when_available ||
              (result.stats.native_constant_address_load_blocks_compiled != 0u &&
               result.stats.native_constant_address_store_blocks_compiled != 0u);
          if (!native_entered) {
            native_check = "v4_native_missing";
          } else if (!load_entered) {
            native_check = "v4_load_missing";
          } else if (!store_entered) {
            native_check = "v4_store_missing";
          } else if (!store_tail_folded) {
            native_check = "v4_store_tail_not_folded";
          } else if (!store_branch_fused) {
            native_check = "v4_store_branch_not_fused";
          } else if (!load_tail_folded) {
            native_check = "v4_load_tail_not_folded";
          } else if (!load_branch_fused) {
            native_check = "v4_load_branch_not_fused";
          } else if (!branch_entered) {
            native_check = "v4_branch_missing";
          } else if (!pending_delay_native) {
            native_check = "v4_pending_delay_helper";
          } else if (!hot_mmio16_native) {
            native_check = "v4_hot_mmio16_helper";
          } else if (!mmio_native) {
            native_check = "v4_mmio_helper";
          } else if (!hilo_native) {
            native_check = "v4_hilo_helper";
          } else if (!muldiv_native) {
            native_check = "v4_muldiv_helper";
          } else if (!cop0_native) {
            native_check = "v4_cop0_helper";
          } else if (!cop2_native) {
            native_check = "v4_cop2_helper";
          } else if (!unaligned_native) {
            native_check = "v4_unaligned_helper";
          } else if (!exception_native) {
            native_check = "v4_exception_helper";
          } else if (!folded_branch) {
            native_check = "v4_branch_not_folded";
          } else if (!page_local_invalidation) {
            native_check = "v4_global_invalidation";
          } else if (!cached_same_page_retained) {
            native_check = "v4_cached_same_page_recompiled";
          } else if (!icache_revalidated) {
            native_check = "v4_icache_revalidation_missing";
          } else if (!native_icache_revalidated) {
            native_check = "v4_native_icache_revalidation_missing";
          } else if (!chain_entered) {
            native_check = "v4_chain_missing";
          } else if (!crossline_block) {
            native_check = "v4_crossline_block_missing";
          } else if (!constant_address_memory) {
            native_check = "v4_constant_address_memory_missing";
          } else {
            native_check = "v4_native_entered";
          }
          native_check_pass =
              native_entered && load_entered && store_entered &&
              store_tail_folded && store_branch_fused &&
              load_tail_folded && load_branch_fused &&
              branch_entered && pending_delay_native && hot_mmio16_native &&
              mmio_native && hilo_native && muldiv_native && cop0_native && cop2_native &&
              unaligned_native && exception_native && folded_branch &&
              page_local_invalidation &&
              cached_same_page_retained && icache_revalidated &&
              native_icache_revalidated && chain_entered && crossline_block &&
              constant_address_memory;
        }
      }

      if (mode == CpuExecutionMode::Recompiler &&
          result.stats.native_available &&
          (result.stats.jit_v4_helper_instructions != 0u ||
           result.stats.fallback_instructions != 0u ||
           result.stats.interpreter_fallback_steps != 0u)) {
        native_check = result.stats.jit_v4_helper_instructions != 0u
                           ? "v4_semantic_helper"
                           : "v4_interpreter_fallback";
        native_check_pass = false;
      }

      if (mode == CpuExecutionMode::X64JitV2 &&
          test_case.require_v2_native_entry_when_available) {
        if (!result.stats.native_available) {
          native_check = "skip_v2_native_unavailable";
        } else {
          const bool native_entered =
              result.stats.native_blocks_compiled != 0 &&
              result.stats.native_block_entries != 0 &&
              result.stats.native_instructions != 0 &&
              result.stats.native_code_bytes != 0;
          native_check = native_entered ? "v2_native_entered"
                                        : "v2_native_missing";
          native_check_pass = native_entered;
        }
      }

      if (mode == CpuExecutionMode::X64JitV2 &&
          test_case.require_v2_store_branch_entry_when_available) {
        if (!result.stats.native_available) {
          native_check = "skip_v2_store_branch_unavailable";
        } else {
          const bool store_branch_entered =
              result.stats.native_branch_tail_entries != 0 &&
              result.stats.native_memory_fastpath_stores >= 2 &&
              result.stats.native_branch_taken != 0 &&
              result.stats.native_instructions != 0;
          native_check = store_branch_entered ? "v2_store_branch_entered"
                                              : "v2_store_branch_missing";
          native_check_pass = store_branch_entered;
        }
      }

      if (mode == CpuExecutionMode::X64JitV2 &&
          test_case.require_v2_branch_not_taken_entry_when_available) {
        if (!result.stats.native_available) {
          native_check = "skip_v2_branch_unavailable";
        } else {
          const bool branch_entered =
              result.stats.native_branch_tail_entries != 0 &&
              result.stats.native_branch_not_taken != 0 &&
              result.stats.native_instructions >= 2;
          native_check = branch_entered ? "v2_branch_not_taken_entered"
                                        : "v2_branch_not_taken_missing";
          native_check_pass = branch_entered;
        }
      }

      if (mode == CpuExecutionMode::X64JitV2 &&
          test_case.require_v2_helper_entry_when_available) {
        if (!result.stats.native_available) {
          native_check = "skip_v2_helper_unavailable";
        } else {
          const bool helper_entered =
              result.stats.jit_v2_helper_entries != 0 &&
              result.stats.jit_v2_helper_instructions != 0 &&
              result.stats.decoded_instructions == 0 &&
              result.stats.fallback_instructions == 0 &&
              result.stats.interpreter_fallback_steps == 0;
          native_check = helper_entered ? "v2_helper_entered"
                                        : "v2_helper_missing";
          native_check_pass = helper_entered;
        }
      }

      if (mode == CpuExecutionMode::X64Jit) {
        if (test_case.require_full_native_when_available) {
          if (!result.stats.native_available) {
            native_check = "skip_native_unavailable";
          } else {
            const bool fully_native =
                result.stats.native_blocks_compiled != 0 &&
                result.stats.native_block_entries != 0 &&
                result.stats.native_instructions >= test_case.instructions &&
                result.stats.native_code_bytes != 0 &&
                result.stats.decoded_instructions == 0 &&
                result.stats.fallback_instructions == 0 &&
                result.stats.interpreter_fallback_steps == 0;
            native_check = fully_native ? "native_full" : "native_missing";
            native_check_pass = fully_native;
          }
        } else if (test_case.require_native_entry_when_available) {
          if (!result.stats.native_available) {
            native_check = "skip_native_unavailable";
          } else {
            const bool allow_post_exception_interpreter =
                test_case.require_native_memory_exception_when_available;
            const bool native_entered =
                result.stats.native_blocks_compiled != 0 &&
                result.stats.native_block_entries != 0 &&
                result.stats.native_instructions != 0 &&
                result.stats.native_code_bytes != 0 &&
                result.stats.decoded_instructions == 0;
            const bool fallback_ok =
                allow_post_exception_interpreter ||
                (result.stats.fallback_instructions == 0 &&
                 result.stats.interpreter_fallback_steps == 0);
            native_check = native_entered ? "native_entered"
                                          : "native_missing";
            native_check_pass = native_entered && fallback_ok;
            if (native_entered && !fallback_ok) {
              native_check = "unexpected_fallback";
            }
          }
        } else if (test_case.expect_x64_fallback) {
          if (!result.stats.native_available) {
            native_check = "skip_native_unavailable";
          } else {
            const bool clean_fallback =
                result.stats.native_block_entries == 0 &&
                result.stats.native_instructions == 0;
            native_check = clean_fallback ? "clean_fallback"
                                          : "unexpected_native";
            native_check_pass = clean_fallback;
          }
        }

        if (native_check_pass && result.stats.native_available &&
            test_case.require_native_memory_helper_when_available &&
            result.stats.native_memory_helper_calls == 0 &&
            result.stats.native_memory_fastpath_loads == 0) {
          native_check = "native_memory_helper_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_native_memory_exception_when_available &&
            result.stats.native_memory_exception_exits == 0) {
          native_check = "native_memory_exception_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_native_helper_load_delay_entry_when_available) {
          const bool load_delay_native =
              result.stats.native_helper_load_delay_entries != 0 &&
              result.stats.native_helper_load_delay_passes != 0 &&
              result.stats.native_block_entries != 0 &&
              result.stats.native_instructions != 0;
          native_check = load_delay_native ? "native_load_delay_entry"
                                           : "native_load_delay_entry_missing";
          native_check_pass = load_delay_native;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_native_branch_tail_when_available) {
          // The coherent emitter no longer exposes the experimental branch-tail
          // tiers. Require the block to enter native code and account for guest
          // instructions; architectural branch state is compared below.
          const bool branch_tail_native =
              result.stats.native_block_entries != 0 &&
              result.stats.native_instructions != 0;
          native_check = branch_tail_native ? "native_branch"
                                             : "native_branch_missing";
          native_check_pass = branch_tail_native;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_native_branch_delay_memory_helper_when_available &&
            result.stats.native_branch_delay_slot_memory_helpers == 0) {
          native_check = "native_branch_delay_memory_helper_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_native_mmio_when_available &&
            result.stats.mmio_accesses == 0) {
          native_check = "native_mmio_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_branch_tail_disabled_fallback_when_available &&
            result.stats.native_branch_tail_disabled_fallbacks == 0) {
          native_check = "branch_tail_disabled_fallback_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_branch_tail_blacklisted_fallback_when_available &&
            result.stats.native_branch_tail_blacklisted_fallbacks == 0) {
          native_check = "branch_tail_blacklist_fallback_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_all_native_disabled_fallback_when_available &&
            result.stats.native_all_disabled_fallbacks == 0) {
          native_check = "all_native_disabled_fallback_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_memory_native_disabled_fallback_when_available &&
            result.stats.native_memory_disabled_fallbacks == 0) {
          native_check = "memory_native_disabled_fallback_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_alu_native_disabled_fallback_when_available &&
            result.stats.native_alu_disabled_fallbacks == 0) {
          native_check = "alu_native_disabled_fallback_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_native_memory_tier_entry_when_available &&
            result.stats.native_memory_block_entries == 0) {
          native_check = "native_memory_tier_entry_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_native_alu_tier_entry_when_available &&
            result.stats.native_alu_block_entries == 0) {
          native_check = "native_alu_tier_entry_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_native_reduced_helper_ram_load_entry_when_available &&
            result.stats.native_reduced_helper_ram_load_entries == 0) {
          native_check = "native_reduced_helper_ram_load_entry_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_native_reduced_helper_branch_tail_entry_when_available &&
            result.stats.native_branch_tail_reduced_helper_entries == 0) {
          native_check = "native_reduced_helper_branch_tail_entry_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_native_reduced_helper_branch_tail_ram_load_entry_when_available &&
            result.stats.native_branch_tail_reduced_helper_ram_load_entries ==
                0) {
          native_check =
              "native_reduced_helper_branch_tail_ram_load_entry_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_native_aggressive_reduced_helper_branch_tail_entry_when_available &&
            result.stats
                    .native_branch_tail_aggressive_reduced_helper_entries ==
                0) {
          native_check =
              "native_aggressive_reduced_helper_branch_tail_entry_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_native_aggressive_reduced_helper_branch_tail_store_entry_when_available &&
            result.stats
                    .native_branch_tail_aggressive_reduced_helper_store_entries ==
                0) {
          native_check =
              "native_aggressive_reduced_helper_branch_tail_store_entry_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_native_aggressive_reduced_helper_branch_tail_mixed_entry_when_available &&
            result.stats
                    .native_branch_tail_aggressive_reduced_helper_mixed_memory_entries ==
                0) {
          native_check =
              "native_aggressive_reduced_helper_branch_tail_mixed_entry_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_native_prefix_entry_when_available &&
            result.stats.native_prefix_entries == 0) {
          native_check = "native_prefix_entry_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_native_prefix_bne_blocker_when_available &&
            result.stats.native_prefix_blocker_bne == 0) {
          native_check = "native_prefix_bne_blocker_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_native_prefix_beq_blocker_when_available &&
            result.stats.native_prefix_blocker_beq == 0) {
          native_check = "native_prefix_beq_blocker_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_native_prefix_jr_blocker_when_available &&
            result.stats.native_prefix_blocker_jr == 0) {
          native_check = "native_prefix_jr_blocker_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_native_prefix_cop2_blocker_when_available &&
            result.stats.native_prefix_blocker_cop2 == 0) {
          native_check = "native_prefix_cop2_blocker_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_native_prefix_other_blocker_when_available &&
            result.stats.native_prefix_blocker_other == 0) {
          native_check = "native_prefix_other_blocker_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_native_prefix_ram_load_entry_when_available &&
            result.stats.native_prefix_ram_load_entries == 0) {
          native_check = "native_prefix_ram_load_entry_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_native_prefix_ram_load_aggressive_entry_when_available &&
            result.stats.native_prefix_ram_load_aggressive_entries == 0) {
          native_check = "native_prefix_ram_load_aggressive_entry_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_native_prefix_ram_load_full_preflight_when_available &&
            result.stats.native_prefix_ram_load_preflight_full_attempts == 0) {
          native_check = "native_prefix_ram_load_full_preflight_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_native_prefix_ram_load_preflight_non_ram_when_available &&
            result.stats.native_prefix_ram_load_preflight_non_ram == 0) {
          native_check = "native_prefix_ram_load_non_ram_preflight_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_native_prefix_ram_load_adaptive_disable_when_available &&
            result.stats.native_prefix_ram_load_adaptive_disabled_blocks ==
                0) {
          native_check = "native_prefix_ram_load_adaptive_disable_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_native_prefix_ram_load_adaptive_direct_entry_when_available &&
            result.stats.native_prefix_ram_load_adaptive_direct_entries ==
                0) {
          native_check = "native_prefix_ram_load_adaptive_direct_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_native_prefix_reject_store_when_available &&
            result.stats.native_prefix_reject_store == 0) {
          native_check = "native_prefix_store_reject_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_no_native_reduced_helper_ram_load_entry &&
            result.stats.native_reduced_helper_ram_load_entries != 0) {
          native_check = "unexpected_native_reduced_helper_ram_load_entry";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_no_native_reduced_helper_branch_tail_ram_load_entry &&
            result.stats.native_branch_tail_reduced_helper_ram_load_entries !=
                0) {
          native_check =
              "unexpected_native_reduced_helper_branch_tail_ram_load_entry";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_no_native_instruction_helpers_when_available &&
            (result.stats.native_prepare_helper_calls != 0 ||
             result.stats.native_finish_helper_calls != 0 ||
             result.stats.native_memory_helper_calls != 0 ||
             result.stats.native_branch_helper_calls != 0)) {
          native_check = "native_instruction_helpers_forbidden";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_no_native_branch_tail_helpers_when_available &&
            (result.stats.native_branch_tail_prepare_helper_calls != 0 ||
             result.stats.native_branch_tail_finish_helper_calls != 0 ||
             result.stats.native_branch_tail_memory_helper_calls != 0 ||
             result.stats.native_branch_tail_branch_helper_calls != 0)) {
          native_check = "native_branch_tail_helpers_forbidden";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_reduced_helper_preflight_mmio_when_available &&
            result.stats.native_reduced_helper_ram_load_preflight_mmio == 0) {
          native_check = "native_reduced_helper_mmio_preflight_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_reduced_helper_preflight_unaligned_when_available &&
            result.stats.native_reduced_helper_ram_load_preflight_unaligned ==
                0) {
          native_check = "native_reduced_helper_unaligned_preflight_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_reduced_helper_preflight_non_ram_when_available &&
            result.stats.native_reduced_helper_ram_load_preflight_non_ram ==
                0) {
          native_check = "native_reduced_helper_non_ram_preflight_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_reduced_helper_branch_tail_preflight_mmio_when_available &&
            result.stats
                    .native_branch_tail_reduced_helper_ram_load_preflight_mmio ==
                0) {
          native_check =
              "native_reduced_helper_branch_tail_mmio_preflight_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_reduced_helper_branch_tail_preflight_unaligned_when_available &&
            result.stats
                    .native_branch_tail_reduced_helper_ram_load_preflight_unaligned ==
                0) {
          native_check =
              "native_reduced_helper_branch_tail_unaligned_preflight_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_reduced_helper_branch_tail_preflight_non_ram_when_available &&
            result.stats
                    .native_branch_tail_reduced_helper_ram_load_preflight_non_ram ==
                0) {
          native_check =
              "native_reduced_helper_branch_tail_non_ram_preflight_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_reduced_helper_branch_tail_reject_load_base_written_when_available &&
            result.stats
                    .native_branch_tail_reduced_helper_reject_load_base_written ==
                0) {
          native_check =
              "native_reduced_helper_branch_tail_load_base_write_reject_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_aggressive_reduced_helper_branch_tail_preflight_non_ram_when_available &&
            result.stats
                    .native_branch_tail_aggressive_reduced_helper_preflight_non_ram ==
                0) {
          native_check =
              "native_aggressive_reduced_helper_branch_tail_non_ram_preflight_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_aggressive_reduced_helper_branch_tail_preflight_code_page_when_available &&
            result.stats
                    .native_branch_tail_aggressive_reduced_helper_preflight_code_page ==
                0) {
          native_check =
              "native_aggressive_reduced_helper_branch_tail_code_page_preflight_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_aggressive_reduced_helper_branch_tail_direct_preflight_when_available &&
            result.stats
                    .native_branch_tail_aggressive_reduced_helper_preflight_direct_attempts ==
                0) {
          native_check =
              "native_aggressive_reduced_helper_branch_tail_direct_preflight_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_aggressive_reduced_helper_branch_tail_full_preflight_when_available &&
            result.stats
                    .native_branch_tail_aggressive_reduced_helper_preflight_full_attempts ==
                0) {
          native_check =
              "native_aggressive_reduced_helper_branch_tail_full_preflight_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_aggressive_reduced_helper_branch_tail_adaptive_disable_when_available &&
            result.stats
                    .native_branch_tail_aggressive_reduced_helper_adaptive_disabled_blocks ==
                0) {
          native_check =
              "native_aggressive_reduced_helper_branch_tail_adaptive_disable_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_aggressive_reduced_helper_branch_tail_adaptive_direct_entry_when_available &&
            result.stats
                    .native_branch_tail_aggressive_reduced_helper_adaptive_direct_entries ==
                0) {
          native_check =
              "native_aggressive_reduced_helper_branch_tail_adaptive_direct_entry_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_native_ram_load_fastpath_when_available &&
            result.stats.native_memory_fastpath_loads == 0 &&
            result.stats.native_memory_helper_calls == 0) {
          // Direct RAM access and precise memory helpers are both valid native
          // block paths.  The removed experimental tier used to require its
          // own fast-path counter even when the coherent block ran natively.
          native_check = "native_memory_path_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_no_native_ram_load_fastpath &&
            result.stats.native_memory_fastpath_loads != 0) {
          native_check = "unexpected_native_ram_load_fastpath";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            result.stats.native_memory_fastpath_mmio_loads != 0) {
          native_check = "native_mmio_fastpath_forbidden";
          native_check_pass = false;
        }
      }

      pass = pass && segment_state_pass && segment_peripheral_pass &&
             irq_state_pass && peripheral_state_pass && memory_state_pass &&
             expected_state_pass && native_check_pass;
      const char *outcome = cpu_compare_outcome(mode, result.stats);
      (void)outcome;

      if (!pass) {
        ++failures;
        log_cpu_compare_failure_summary(
            test_case, mode, reference, result, state_pass,
            segment_state_pass, irq_state_pass, memory_state_pass,
            peripheral_state_pass, segment_peripheral_pass,
            expected_state_pass, native_check_pass, native_check);
      }
    }
  }

  g_experimental_unhandled_special_returns_zero = saved_unknown_fallback;
  g_cpu_x64_jit_force_compile = saved_force_native;
  g_cpu_x64_jit_branch_tail_enabled = saved_branch_tail_enabled;
  g_cpu_x64_jit_branch_tail_cli_override =
      saved_branch_tail_cli_override;
  g_cpu_x64_jit_branch_tail_cli_value = saved_branch_tail_cli_value;
  g_cpu_x64_jit_branch_tail_blacklist = saved_branch_tail_blacklist;
  g_cpu_backend_compare_irq_on_branch = saved_compare_irq_on_branch;
  g_cpu_backend_compare_allow_partial_branch_tail =
      saved_compare_partial_branch_tail;
  g_cpu_backend_compare_allow_partial_memory_helper =
      saved_compare_partial_memory;
  g_cpu_x64_jit_all_native_cli_override = saved_all_native_cli_override;
  g_cpu_x64_jit_all_native_cli_value = saved_all_native_cli_value;
  g_cpu_x64_jit_native_memory_cli_override =
      saved_memory_native_cli_override;
  g_cpu_x64_jit_native_memory_cli_value = saved_memory_native_cli_value;
  g_cpu_x64_jit_native_alu_cli_override = saved_alu_native_cli_override;
  g_cpu_x64_jit_native_alu_cli_value = saved_alu_native_cli_value;
  g_cpu_x64_jit_ram_load_fastpath_enabled = saved_ram_load_fastpath;
  g_cpu_x64_jit_reduced_helper_branch_tail_enabled =
      saved_reduced_helper_branch_tail;
  g_cpu_x64_jit_aggressive_reduced_helper_branch_tail_enabled =
      saved_aggressive_reduced_helper_branch_tail;
  g_cpu_x64_jit_native_prefix_enabled = saved_native_prefix;
  g_cpu_x64_jit_aggressive_native_prefix_ram_enabled =
      saved_aggressive_native_prefix_ram;
  g_cpu_x64_jit_aggressive_native_prefix_ram_cli_override =
      saved_aggressive_native_prefix_ram_cli_override;
  g_cpu_x64_jit_aggressive_native_prefix_ram_cli_value =
      saved_aggressive_native_prefix_ram_cli_value;
  g_cpu_backend_compare_test_active = saved_compare_test_active;
  g_cpu_execution_mode_cli_override = saved_override;
  g_cpu_execution_mode_cli_value = saved_override_value;

  if (failures != 0) {
    LOG_ERROR(
        "CPU backend compare test failed: %d case(s) reference=Interpreter",
        failures);
    return 1;
  }
  LOG_INFO(
      "CPU backend compare test passed: reference=Interpreter targets=Recompiler");
  return 0;
}
} // namespace

int run_cpu_backend_compare_test(bool memory_only) {
  return run_cpu_backend_compare_test_impl(memory_only);
}

