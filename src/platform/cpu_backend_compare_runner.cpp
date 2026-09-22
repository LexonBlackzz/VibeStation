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
  bool request_irq_on_branch = false;
  u32 instructions = 0;
  bool expect_final_control_state = false;
  u32 expected_pc = 0;
  u32 expected_next_pc = 0;
  u32 expected_current_pc = 0;
  u64 expected_cycles = 0;
  bool experimental_unknown_fallback = false;
  bool require_full_native_when_available = false;
  bool require_native_entry_when_available = false;
  bool require_v2_native_entry_when_available = false;
  bool require_v4_native_entry_when_available = false;
  bool require_v4_native_load_entry_when_available = false;
  bool require_v4_native_store_entry_when_available = false;
  bool require_v4_store_smc_fallback_when_available = false;
  bool require_v4_load_tail_block_when_available = false;
  bool require_v4_load_branch_fusion_when_available = false;
  bool require_v4_native_branch_entry_when_available = false;
  bool require_v4_folded_branch_block_when_available = false;
  bool require_v4_page_local_invalidation_when_available = false;
  bool require_v4_cached_same_page_retention_when_available = false;
  bool require_v4_native_chain_when_available = false;
  bool require_v4_clean_fallback_when_available = false;
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
};

static bool cpu_compare_peripherals_equal(
    const CpuComparePeripheralState &a,
    const CpuComparePeripheralState &b) {
  return a.irq_stat == b.irq_stat && a.irq_mask == b.irq_mask &&
         a.dma_dpcr == b.dma_dpcr && a.dma_dicr == b.dma_dicr &&
         a.cd_sector_count == b.cd_sector_count &&
         a.cd_read_lba == b.cd_read_lba &&
         a.cd_active_lba == b.cd_active_lba &&
         a.cd_data_index == b.cd_data_index &&
         a.cd_busy_cycles == b.cd_busy_cycles &&
         a.cd_pending_irqs == b.cd_pending_irqs &&
         a.cd_response_size == b.cd_response_size &&
         a.cd_last_irq == b.cd_last_irq &&
         a.cd_data_ready == b.cd_data_ready &&
         a.cd_data_request == b.cd_data_request;
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
  case CpuExecutionMode::X64JitV4:
    return "X64JitV4";
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
      "first_reg=%d:%08X/%08X native_instr=%llu decoded_instr=%llu "
      "fallback_instr=%llu",
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
      first_reg, first_reg_ref, first_reg_actual,
      static_cast<unsigned long long>(actual.stats.native_instructions),
      static_cast<unsigned long long>(actual.stats.decoded_instructions),
      static_cast<unsigned long long>(actual.stats.fallback_instructions));
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
                                            const CpuDebugState &state) {
  if (!test_case.expect_final_control_state) {
    return true;
  }

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

  field("pc", test_case.expected_pc, state.pc);
  field("next_pc", test_case.expected_next_pc, state.next_pc);
  field("current_pc", test_case.expected_current_pc, state.current_pc);
  field("cycles", test_case.expected_cycles, state.cycles);
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
  initial.next_pc = test_case.start_pc + 4u;
  initial.current_pc = 0;
  initial.cycles = 0;
  initial.load_reg = test_case.initial_load_reg;
  initial.load_value = test_case.initial_load_value;
  initial.next_load_reg = 0;
  initial.next_load_value = 0;
  initial.in_delay_slot = false;
  initial.pending_delay_slot = false;
  initial.pending_branch_taken = false;
  initial.pending_branch_pc = 0;
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
    CpuRunSliceResult segment = sys->cpu().run_slice(100000u, instruction_count);
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
      sys->write32(mutation.addr, mutation.value);
      if (mutation.invalidate_icache_line) {
        sys->cpu().debug_invalidate_icache_line(mutation.addr);
      }
    }
  }
  if (executed < test_case.instructions) {
    run_segment(test_case.instructions - executed);
  }
  out.state = sys->cpu().debug_state();
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
      "jit_v2_scratchpad_sw_sw_bne_delay_loop";
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
  cases.push_back(overflow_add);

  CpuCompareCase overflow_sub{};
  overflow_sub.name = "decoded_sub_overflow";
  overflow_sub.initial_gpr[1] = 0x80000000u;
  overflow_sub.initial_gpr[2] = 1u;
  overflow_sub.program = {enc_r(1, 2, 3, 0, 0x22)};
  overflow_sub.instructions = 1;
  cases.push_back(overflow_sub);

  CpuCompareCase overflow_addi{};
  overflow_addi.name = "decoded_addi_overflow";
  overflow_addi.initial_gpr[1] = 0x7FFFFFFFu;
  overflow_addi.program = {enc_i(0x08, 1, 2, 1)};
  overflow_addi.instructions = 1;
  cases.push_back(overflow_addi);

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
  cases.push_back(unaligned_merge);

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
  cases.push_back(cop_transfers);

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
      "native_aggressive_reduced_helper_branch_tail_sw_scratchpad_rejected";
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
  aggressive_sw_scratchpad_reject.expect_x64_fallback = true;
  aggressive_sw_scratchpad_reject
      .require_aggressive_reduced_helper_branch_tail_preflight_non_ram_when_available =
      true;
  cases.push_back(aggressive_sw_scratchpad_reject);

  CpuCompareCase aggressive_sw_scratchpad_adaptive{};
  aggressive_sw_scratchpad_adaptive.name =
      "native_aggressive_reduced_helper_branch_tail_sw_scratchpad_adaptive";
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
  aggressive_sw_scratchpad_adaptive.expect_x64_fallback = true;
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
      "native_aggressive_reduced_helper_branch_tail_sb_scratchpad_rejected";
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
  aggressive_sb_scratchpad_reject.expect_x64_fallback = true;
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
  cases.push_back(branch_irq_before);

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
  cases.push_back(branch_irq_delay);

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
  pad_cpu_compare_program(cdrom_status_helper);
  cases.push_back(cdrom_status_helper);

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
  cases.push_back(break_exception);

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

  CpuCompareCase v4_uncached_delay_exception_fallback{};
  v4_uncached_delay_exception_fallback.name =
      "v4_uncached_branch_delay_overflow_fallback";
  v4_uncached_delay_exception_fallback.start_pc = 0xA0010000u;
  v4_uncached_delay_exception_fallback.initial_gpr[1] = 1u;
  v4_uncached_delay_exception_fallback.initial_gpr[2] = 0x7FFFFFFFu;
  v4_uncached_delay_exception_fallback.program = {
      enc_i(0x05, 1, 0, 1), // BNE taken
      enc_i(0x08, 2, 2, 1), // ADDI overflow in the delay slot
      0,
  };
  v4_uncached_delay_exception_fallback.instructions = 2u;
  v4_uncached_delay_exception_fallback.require_v4_clean_fallback_when_available =
      true;
  cases.push_back(v4_uncached_delay_exception_fallback);

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
      enc_i(0x05, 2, 0, 1),
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
      enc_i(0x04, 2, 0, 1),
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
  };
  v4_uncached_store_pending_load.instructions = 1u;
  v4_uncached_store_pending_load.require_v4_native_entry_when_available = true;
  v4_uncached_store_pending_load.require_v4_native_store_entry_when_available =
      true;
  cases.push_back(v4_uncached_store_pending_load);

  CpuCompareCase v4_uncached_store_smc{};
  v4_uncached_store_smc.name = "v4_uncached_store_code_page_fallback";
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
  v4_uncached_store_smc.require_v4_store_smc_fallback_when_available = true;
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
  // the resident x64 dispatcher must execute A->B->A without returning to C++.
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
      "CPU backend compare: reference=Interpreter targets=DecodedBlockInterpreter,X64Jit,X64JitV2,X64JitV3,X64JitV4%s",
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
  const std::array<CpuExecutionMode, 6> modes = {
      CpuExecutionMode::Interpreter,
      CpuExecutionMode::DecodedBlockInterpreter,
      CpuExecutionMode::X64Jit,
      CpuExecutionMode::X64JitV2,
      CpuExecutionMode::X64JitV3,
      CpuExecutionMode::X64JitV4,
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
          cpu_compare_expected_state_pass(test_case, mode, result.state);
      bool native_check_pass = true;
      const char *native_check = "not_required";

      if (mode == CpuExecutionMode::X64JitV4 &&
          test_case.require_v4_clean_fallback_when_available) {
        if (!result.stats.native_available) {
          native_check = "skip_v4_native_unavailable";
        } else {
          const bool clean_fallback =
              result.stats.native_block_entries == 0 &&
              result.stats.native_instructions == 0;
          native_check = clean_fallback ? "v4_clean_fallback"
                                        : "v4_unexpected_native";
          native_check_pass = clean_fallback;
        }
      } else if (mode == CpuExecutionMode::X64JitV4 &&
                 test_case.require_v4_store_smc_fallback_when_available) {
        if (!result.stats.native_available) {
          native_check = "skip_v4_native_unavailable";
        } else {
          const bool guarded =
              result.stats.native_memory_blocks_compiled != 0u &&
              result.stats.native_memory_fastpath_stores == 0u &&
              result.stats.fallback_instructions != 0u;
          native_check = guarded ? "v4_store_smc_guarded"
                                 : "v4_store_smc_not_guarded";
          native_check_pass = guarded;
        }
      } else if (mode == CpuExecutionMode::X64JitV4 &&
          (test_case.require_v4_native_entry_when_available ||
           test_case.require_v4_native_load_entry_when_available ||
           test_case.require_v4_native_store_entry_when_available ||
           test_case.require_v4_load_tail_block_when_available ||
           test_case.require_v4_load_branch_fusion_when_available ||
           test_case.require_v4_native_branch_entry_when_available ||
           test_case.require_v4_folded_branch_block_when_available ||
           test_case.require_v4_page_local_invalidation_when_available ||
           test_case.require_v4_cached_same_page_retention_when_available ||
           test_case.require_v4_native_chain_when_available)) {
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
          const bool folded_branch =
              !test_case.require_v4_folded_branch_block_when_available ||
              (result.stats.native_branch_tail_blocks_compiled != 0 &&
               result.stats.native_alu_blocks_compiled == 0);
          const bool page_local_invalidation =
              !test_case.require_v4_page_local_invalidation_when_available ||
              (result.stats.invalidations != 0u &&
               result.stats.block_count >= 2u &&
               result.stats.native_blocks_compiled >= 2u);
          const bool cached_same_page_retained =
              !test_case.require_v4_cached_same_page_retention_when_available ||
              (result.stats.invalidations != 0u &&
               result.stats.native_blocks_compiled == 1u &&
               result.stats.block_count == 1u);
          const bool chain_entered =
              !test_case.require_v4_native_chain_when_available ||
              (result.stats.native_chain_entries != 0 &&
               result.stats.native_linked_transitions != 0 &&
               result.stats.native_chain_max_blocks > 1u);
          if (!native_entered) {
            native_check = "v4_native_missing";
          } else if (!load_entered) {
            native_check = "v4_load_missing";
          } else if (!store_entered) {
            native_check = "v4_store_missing";
          } else if (!load_tail_folded) {
            native_check = "v4_load_tail_not_folded";
          } else if (!load_branch_fused) {
            native_check = "v4_load_branch_not_fused";
          } else if (!branch_entered) {
            native_check = "v4_branch_missing";
          } else if (!folded_branch) {
            native_check = "v4_branch_not_folded";
          } else if (!page_local_invalidation) {
            native_check = "v4_global_invalidation";
          } else if (!cached_same_page_retained) {
            native_check = "v4_cached_same_page_recompiled";
          } else if (!chain_entered) {
            native_check = "v4_chain_missing";
          } else {
            native_check = "v4_native_entered";
          }
          native_check_pass =
              native_entered && load_entered && store_entered &&
              load_tail_folded && load_branch_fused &&
              branch_entered && folded_branch && page_local_invalidation &&
              cached_same_page_retained && chain_entered;
        }
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
      "CPU backend compare test passed: reference=Interpreter targets=DecodedBlockInterpreter,X64Jit,X64JitV2,X64JitV3,X64JitV4");
  return 0;
}
} // namespace

int run_cpu_backend_compare_test(bool memory_only) {
  return run_cpu_backend_compare_test_impl(memory_only);
}

