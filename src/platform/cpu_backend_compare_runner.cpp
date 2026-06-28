#include "platform/cpu_backend_compare_runner.h"
#include "core/system.h"
#include <array>
#include <memory>
#include <string_view>
#include <vector>

namespace {
constexpr u32 kCpuComparePc = 0x80010000u;

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
  std::vector<u32> program;
  std::vector<CpuCompareMemoryWord> memory;
  std::vector<u32> compare_memory_addresses;
  std::vector<CpuCompareCodeMutation> mutations;
  std::vector<u32> segment_instructions;
  std::vector<CpuCompareNativeTierMode> segment_native_tiers;
  std::array<u32, 32> initial_gpr{};
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
  bool allow_partial_native_memory_helper = false;
  bool compare_segment_states = false;
  bool disable_all_native_for_x64 = false;
  bool disable_memory_native_for_x64 = false;
  bool disable_alu_native_for_x64 = false;
  bool require_all_native_disabled_fallback_when_available = false;
  bool require_memory_native_disabled_fallback_when_available = false;
  bool require_alu_native_disabled_fallback_when_available = false;
  bool require_native_memory_tier_entry_when_available = false;
  bool require_native_alu_tier_entry_when_available = false;
  bool require_native_reduced_helper_ram_load_entry_when_available = false;
  bool require_no_native_reduced_helper_ram_load_entry = false;
  bool require_no_native_instruction_helpers_when_available = false;
  bool require_reduced_helper_preflight_mmio_when_available = false;
  bool require_reduced_helper_preflight_unaligned_when_available = false;
  bool require_reduced_helper_preflight_non_ram_when_available = false;
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
              kCpuComparePc + static_cast<u32>(i * 4u),
              test_case.program[i]);
  }
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
    LOG_ERROR("CPU_COMPARE_EXPECTED_DIFF name=%s mode=%s field=%s expected=0x%llX actual=0x%llX",
              test_case.name, cpu_compare_mode_name(mode), field_name,
              static_cast<unsigned long long>(expected),
              static_cast<unsigned long long>(actual));
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
    sys->write32((kCpuComparePc & 0x1FFFFFFFu) + static_cast<u32>(i * 4u),
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
  initial.pc = kCpuComparePc;
  initial.next_pc = kCpuComparePc + 4u;
  initial.current_pc = 0;
  initial.cycles = 0;
  initial.load_reg = 0;
  initial.load_value = 0;
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
    g_cpu_x64_jit_branch_tail_blacklist.push_back(kCpuComparePc);
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
  native_memory_disabled.expect_x64_fallback = true;
  native_memory_disabled
      .require_memory_native_disabled_fallback_when_available = true;
  cases.push_back(native_memory_disabled);

  CpuCompareCase native_alu_disabled{};
  native_alu_disabled.name = "x64_native_alu_disabled_gate";
  native_alu_disabled.program.assign(16u, 0u);
  native_alu_disabled.instructions = 16;
  native_alu_disabled.disable_alu_native_for_x64 = true;
  native_alu_disabled.expect_x64_fallback = true;
  native_alu_disabled.require_alu_native_disabled_fallback_when_available =
      true;
  cases.push_back(native_alu_disabled);

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
      .require_native_helper_load_delay_entry_when_available = true;
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
  branch_tail_disabled.expect_x64_fallback = true;
  branch_tail_disabled.require_branch_tail_disabled_fallback_when_available =
      true;
  cases.push_back(branch_tail_disabled);

  CpuCompareCase branch_tail_blacklisted{};
  branch_tail_blacklisted.name = "native_branch_tail_pc_blacklist";
  branch_tail_blacklisted.initial_gpr[1] = 1u;
  branch_tail_blacklisted.program = branch_tail_disabled.program;
  branch_tail_blacklisted.instructions = 2;
  branch_tail_blacklisted.blacklist_branch_tail_for_x64 = true;
  branch_tail_blacklisted.expect_x64_fallback = true;
  branch_tail_blacklisted
      .require_branch_tail_blacklisted_fallback_when_available = true;
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
  branch_irq_before.expect_x64_fallback = true;
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
  branch_irq_delay.require_native_branch_tail_when_available = true;
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
  native_memory_then_decoded_load
      .require_native_reduced_helper_ram_load_entry_when_available = true;
  native_memory_then_decoded_load
      .require_no_native_instruction_helpers_when_available = true;
  cases.push_back(native_memory_then_decoded_load);

  CpuCompareCase decoded_then_native_fast_load{};
  decoded_then_native_fast_load.name =
      "decoded_load_then_native_ram_fast_load";
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
  decoded_then_native_fast_load
      .require_native_ram_load_fastpath_when_available = true;
  decoded_then_native_fast_load
      .require_native_memory_tier_entry_when_available = true;
  decoded_then_native_fast_load
      .require_native_reduced_helper_ram_load_entry_when_available = true;
  decoded_then_native_fast_load
      .require_no_native_instruction_helpers_when_available = true;
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
  jal.expect_x64_fallback = true;
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
  jalr.expect_x64_fallback = true;
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
  reduced_ram_load_widths
      .require_native_reduced_helper_ram_load_entry_when_available = true;
  reduced_ram_load_widths
      .require_no_native_instruction_helpers_when_available = true;
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
  reduced_ram_load_boundary
      .require_native_reduced_helper_ram_load_entry_when_available = true;
  reduced_ram_load_boundary
      .require_no_native_instruction_helpers_when_available = true;
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
  reduced_ram_load_cancel
      .require_native_reduced_helper_ram_load_entry_when_available = true;
  reduced_ram_load_cancel
      .require_no_native_instruction_helpers_when_available = true;
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
  reduced_ram_load_mmio_reject.expect_x64_fallback = true;
  reduced_ram_load_mmio_reject.require_native_mmio_when_available = true;
  reduced_ram_load_mmio_reject
      .require_no_native_reduced_helper_ram_load_entry = true;
  reduced_ram_load_mmio_reject
      .require_reduced_helper_preflight_mmio_when_available = true;
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
  reduced_ram_load_scratchpad_reject.expect_x64_fallback = true;
  reduced_ram_load_scratchpad_reject
      .require_no_native_reduced_helper_ram_load_entry = true;
  reduced_ram_load_scratchpad_reject
      .require_reduced_helper_preflight_non_ram_when_available = true;
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
  reduced_ram_load_unaligned_reject.expect_x64_fallback = true;
  reduced_ram_load_unaligned_reject
      .require_no_native_reduced_helper_ram_load_entry = true;
  reduced_ram_load_unaligned_reject
      .require_reduced_helper_preflight_unaligned_when_available = true;
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
  syscall_exception.expect_x64_fallback = true;
  cases.push_back(syscall_exception);

  CpuCompareCase break_exception{};
  break_exception.name = "exception_break";
  break_exception.program = {
      enc_i(0x09, 0, 1, 5),
      0x0000000Du,
      enc_i(0x09, 0, 2, 6),
  };
  break_exception.instructions = 2;
  break_exception.expect_x64_fallback = true;
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
  cop0.expect_x64_fallback = true;
  cases.push_back(cop0);

  CpuCompareCase cop2{};
  cop2.name = "unsafe_cop2_gte_fallback";
  cop2.program = {
      (0x12u << 26) | (0u << 21) | (2u << 16) | (0u << 11),
      0,
  };
  cop2.instructions = 2;
  cop2.expect_x64_fallback = true;
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
  g_cpu_backend_compare_test_active = true;
  g_cpu_x64_jit_force_compile = true;

  int failures = 0;
  const std::array<CpuExecutionMode, 3> modes = {
      CpuExecutionMode::Interpreter,
      CpuExecutionMode::DecodedBlockInterpreter,
      CpuExecutionMode::X64Jit,
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
          test_case.require_reduced_helper_preflight_mmio_when_available ||
          test_case.require_reduced_helper_preflight_unaligned_when_available ||
          test_case.require_reduced_helper_preflight_non_ram_when_available ||
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
            log_cpu_debug_state_diff(
                test_case.name, cpu_compare_mode_name(mode),
                reference.segment_states[segment],
                result.segment_states[segment]);
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
            result.stats.native_memory_helper_calls == 0) {
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
          const u64 expected_branch_count =
              test_case.native_branch_should_be_taken
                  ? result.stats.native_branch_taken
                  : result.stats.native_branch_not_taken;
          u64 expected_opcode_entries = 1u;
          u64 expected_opcode_outcomes = 1u;
          if (test_case.native_branch_primary_op == 0x07u) {
            expected_opcode_entries =
                result.stats.native_branch_tail_bgtz_entries;
            expected_opcode_outcomes =
                test_case.native_branch_should_be_taken
                    ? result.stats.native_branch_tail_bgtz_taken
                    : result.stats.native_branch_tail_bgtz_not_taken;
          } else if (test_case.native_branch_primary_op == 0x06u) {
            expected_opcode_entries =
                result.stats.native_branch_tail_blez_entries;
            expected_opcode_outcomes =
                test_case.native_branch_should_be_taken
                    ? result.stats.native_branch_tail_blez_taken
                    : result.stats.native_branch_tail_blez_not_taken;
          }
          const bool branch_tail_native =
              result.stats.native_branch_tail_blocks_compiled != 0 &&
              result.stats.native_branch_tail_entries != 0 &&
              expected_branch_count != 0 && expected_opcode_entries != 0 &&
              expected_opcode_outcomes != 0;
          native_check = branch_tail_native ? "native_branch_tail"
                                             : "native_branch_tail_missing";
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
            test_case.require_no_native_reduced_helper_ram_load_entry &&
            result.stats.native_reduced_helper_ram_load_entries != 0) {
          native_check = "unexpected_native_reduced_helper_ram_load_entry";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_no_native_instruction_helpers_when_available &&
            (result.stats.native_prepare_helper_calls != 0 ||
             result.stats.native_finish_helper_calls != 0 ||
             result.stats.native_memory_helper_calls != 0)) {
          native_check = "native_instruction_helpers_forbidden";
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
            test_case.require_native_ram_load_fastpath_when_available &&
            result.stats.native_memory_fastpath_loads == 0) {
          native_check = "native_ram_load_fastpath_missing";
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
      LOG_INFO(
          "CPU_COMPARE name=%s mode=%s result=%s outcome=%s native_check=%s pc=0x%08X next_pc=0x%08X current_pc=0x%08X instr=%u cycles=%llu decoded_instr=%llu native_instr=%llu fallback_instr=%llu native_mem_helpers=%llu native_mem_exits=%llu helper_ld_entries=%llu helper_ld_passes=%llu helper_ld_fallbacks=%llu forced_reason=%s forced_slices=%llu forced_instr=%llu native_blocks=%llu native_attempts=%llu native_successes=%llu native_compiled=%llu native_entries=%llu native_code_bytes=%llu native_available=%u",
          test_case.name, cpu_compare_mode_name(mode),
          pass ? "PASS" : "FAIL", outcome, native_check, result.state.pc,
          result.state.next_pc, result.state.current_pc, result.run.instructions,
          static_cast<unsigned long long>(result.state.cycles),
          static_cast<unsigned long long>(result.stats.decoded_instructions),
          static_cast<unsigned long long>(result.stats.native_instructions),
          static_cast<unsigned long long>(result.stats.fallback_instructions),
          static_cast<unsigned long long>(
              result.stats.native_memory_helper_calls),
          static_cast<unsigned long long>(
              result.stats.native_memory_exception_exits),
          static_cast<unsigned long long>(
              result.stats.native_helper_load_delay_entries),
          static_cast<unsigned long long>(
              result.stats.native_helper_load_delay_passes),
          static_cast<unsigned long long>(
              result.stats.native_helper_load_delay_fallbacks),
          cpu_forced_interpreter_reason_name(
              result.stats.forced_interpreter_last_reason),
          static_cast<unsigned long long>(
              result.stats.forced_interpreter_slices),
          static_cast<unsigned long long>(
              result.stats.forced_interpreter_instructions),
          static_cast<unsigned long long>(result.stats.native_blocks),
          static_cast<unsigned long long>(result.stats.native_compile_attempts),
          static_cast<unsigned long long>(result.stats.native_compile_successes),
          static_cast<unsigned long long>(result.stats.native_blocks_compiled),
          static_cast<unsigned long long>(result.stats.native_block_entries),
          static_cast<unsigned long long>(result.stats.native_code_bytes),
          result.stats.native_available ? 1u : 0u);

      if (mode == CpuExecutionMode::X64Jit &&
          (test_case.require_full_native_when_available ||
           test_case.require_native_entry_when_available ||
           test_case.require_native_memory_helper_when_available ||
           test_case.require_native_memory_exception_when_available ||
           test_case.require_native_helper_load_delay_entry_when_available ||
           test_case.require_native_branch_tail_when_available ||
           test_case
               .require_native_reduced_helper_ram_load_entry_when_available ||
           test_case
               .require_native_branch_delay_memory_helper_when_available)) {
        LOG_INFO(
            "CPU_COMPARE_NATIVE name=%s required=1 available=%u compiled=%u entered=%u native_instr=%llu decoded_instr=%llu fallback_instr=%llu native_mem_helpers=%llu native_mem_exits=%llu helper_ld_entries=%llu helper_ld_passes=%llu helper_ld_fallbacks=%llu code_bytes=%llu attempts=%llu successes=%llu force=%u hot_threshold=%u min_block=%u",
            test_case.name, result.stats.native_available ? 1u : 0u,
            result.stats.native_blocks_compiled != 0 ? 1u : 0u,
            result.stats.native_block_entries != 0 ? 1u : 0u,
            static_cast<unsigned long long>(result.stats.native_instructions),
            static_cast<unsigned long long>(result.stats.decoded_instructions),
            static_cast<unsigned long long>(result.stats.fallback_instructions),
            static_cast<unsigned long long>(
                result.stats.native_memory_helper_calls),
            static_cast<unsigned long long>(
                result.stats.native_memory_exception_exits),
            static_cast<unsigned long long>(
                result.stats.native_helper_load_delay_entries),
            static_cast<unsigned long long>(
                result.stats.native_helper_load_delay_passes),
            static_cast<unsigned long long>(
                result.stats.native_helper_load_delay_fallbacks),
            static_cast<unsigned long long>(result.stats.native_code_bytes),
            static_cast<unsigned long long>(result.stats.native_compile_attempts),
            static_cast<unsigned long long>(
                result.stats.native_compile_successes),
            g_cpu_x64_jit_force_compile ? 1u : 0u,
            g_cpu_x64_jit_hot_block_threshold,
            g_cpu_x64_jit_min_block_instructions);
      }

      if (mode == CpuExecutionMode::X64Jit &&
          test_case.require_native_branch_tail_when_available) {
        LOG_INFO(
            "CPU_COMPARE_BRANCH_TAIL name=%s blocks_compiled=%llu entries=%llu taken=%llu not_taken=%llu rejects=%llu delay_memory_helpers=%llu",
            test_case.name,
            static_cast<unsigned long long>(
                result.stats.native_branch_tail_blocks_compiled),
            static_cast<unsigned long long>(
                result.stats.native_branch_tail_entries),
            static_cast<unsigned long long>(result.stats.native_branch_taken),
            static_cast<unsigned long long>(
                result.stats.native_branch_not_taken),
            static_cast<unsigned long long>(
                result.stats.native_branch_tail_rejects),
            static_cast<unsigned long long>(
                result.stats.native_branch_delay_slot_memory_helpers));
      }

      if (mode == CpuExecutionMode::X64Jit &&
          (test_case.enable_ram_load_fastpath_for_x64 ||
           test_case.require_native_ram_load_fastpath_when_available ||
           test_case.require_no_native_ram_load_fastpath)) {
        LOG_INFO("CPU_COMPARE_RAM_FASTPATH name=%s enabled=%u fast_loads=%llu memory_helpers=%llu ram_helpers=%llu scratch_helpers=%llu bios_helpers=%llu mmio_helpers=%llu unknown_helpers=%llu unaligned_helpers=%llu mmio_fast_loads=%llu",
                 test_case.name,
                 test_case.enable_ram_load_fastpath_for_x64 ? 1u : 0u,
                 static_cast<unsigned long long>(
                     result.stats.native_memory_fastpath_loads),
                 static_cast<unsigned long long>(
                     result.stats.native_memory_helper_calls),
                 static_cast<unsigned long long>(
                     result.stats.native_memory_helper_ram_calls),
                 static_cast<unsigned long long>(
                     result.stats.native_memory_helper_scratchpad_calls),
                 static_cast<unsigned long long>(
                     result.stats.native_memory_helper_bios_calls),
                 static_cast<unsigned long long>(
                     result.stats.native_memory_helper_mmio_calls),
                 static_cast<unsigned long long>(
                     result.stats.native_memory_helper_unknown_calls),
                 static_cast<unsigned long long>(
                     result.stats.native_memory_helper_unaligned_calls),
                 static_cast<unsigned long long>(
                     result.stats.native_memory_fastpath_mmio_loads));
      }

      if (mode == CpuExecutionMode::X64Jit &&
          (test_case
               .require_native_reduced_helper_ram_load_entry_when_available ||
           test_case.require_no_native_reduced_helper_ram_load_entry ||
           test_case.require_reduced_helper_preflight_mmio_when_available ||
           test_case
               .require_reduced_helper_preflight_unaligned_when_available ||
           test_case
               .require_reduced_helper_preflight_non_ram_when_available)) {
        LOG_INFO("CPU_COMPARE_REDUCED_HELPER_RAM name=%s entries=%llu ram_load_entries=%llu fast_loads=%llu preflight_fallbacks=%llu preflight_mmio=%llu preflight_unaligned=%llu preflight_non_ram=%llu preflight_disabled=%llu native_helpers=%llu",
                 test_case.name,
                 static_cast<unsigned long long>(
                     result.stats.native_reduced_helper_entries),
                 static_cast<unsigned long long>(
                     result.stats.native_reduced_helper_ram_load_entries),
                 static_cast<unsigned long long>(
                     result.stats.native_memory_fastpath_loads),
                 static_cast<unsigned long long>(
                     result.stats
                         .native_reduced_helper_ram_load_preflight_fallbacks),
                 static_cast<unsigned long long>(
                     result.stats
                         .native_reduced_helper_ram_load_preflight_mmio),
                 static_cast<unsigned long long>(
                     result.stats
                         .native_reduced_helper_ram_load_preflight_unaligned),
                 static_cast<unsigned long long>(
                     result.stats
                         .native_reduced_helper_ram_load_preflight_non_ram),
                 static_cast<unsigned long long>(
                     result.stats
                         .native_reduced_helper_ram_load_preflight_disabled),
                 static_cast<unsigned long long>(
                     result.stats.native_memory_helper_calls));
      }

      if (!pass) {
        ++failures;
        if (!state_pass) {
          log_cpu_debug_state_diff(test_case.name, cpu_compare_mode_name(mode),
                                   reference.state, result.state);
        }
        if (!irq_state_pass) {
          LOG_ERROR("CPU_COMPARE_IRQ_DIFF name=%s mode=%s reference_stat=0x%08X actual_stat=0x%08X reference_mask=0x%08X actual_mask=0x%08X",
                    test_case.name, cpu_compare_mode_name(mode),
                    reference.irq_stat, result.irq_stat,
                    reference.irq_mask, result.irq_mask);
        }
        if (!memory_state_pass) {
          LOG_ERROR("CPU_COMPARE_MEMORY_DIFF name=%s mode=%s reference_count=%zu actual_count=%zu",
                    test_case.name, cpu_compare_mode_name(mode),
                    reference.memory_values.size(),
                    result.memory_values.size());
        }
        if (!peripheral_state_pass || !segment_peripheral_pass) {
          LOG_ERROR("CPU_COMPARE_PERIPHERAL_DIFF name=%s mode=%s final=%u segment=%u reference_dma=0x%08X/0x%08X actual_dma=0x%08X/0x%08X reference_cd=%llu/%d/%d actual_cd=%llu/%d/%d",
                    test_case.name, cpu_compare_mode_name(mode),
                    peripheral_state_pass ? 1u : 0u,
                    segment_peripheral_pass ? 1u : 0u,
                    reference.peripherals.dma_dpcr,
                    reference.peripherals.dma_dicr,
                    result.peripherals.dma_dpcr,
                    result.peripherals.dma_dicr,
                    static_cast<unsigned long long>(
                        reference.peripherals.cd_sector_count),
                    reference.peripherals.cd_read_lba,
                    reference.peripherals.cd_active_lba,
                    static_cast<unsigned long long>(
                        result.peripherals.cd_sector_count),
                    result.peripherals.cd_read_lba,
                    result.peripherals.cd_active_lba);
        }
        if (!native_check_pass) {
          LOG_ERROR(
              "CPU_COMPARE_BACKEND_DIFF name=%s mode=%s expected=%s outcome=%s native_available=%u native_blocks_compiled=%llu native_entries=%llu native_instr=%llu native_mem_helpers=%llu native_mem_exits=%llu helper_ld_entries=%llu helper_ld_passes=%llu helper_ld_fallbacks=%llu native_code_bytes=%llu decoded_instr=%llu fallback_instr=%llu interpreter_fallback_steps=%llu",
              test_case.name, cpu_compare_mode_name(mode), native_check,
              outcome, result.stats.native_available ? 1u : 0u,
              static_cast<unsigned long long>(
                  result.stats.native_blocks_compiled),
              static_cast<unsigned long long>(
                  result.stats.native_block_entries),
              static_cast<unsigned long long>(result.stats.native_instructions),
              static_cast<unsigned long long>(
                  result.stats.native_memory_helper_calls),
              static_cast<unsigned long long>(
                  result.stats.native_memory_exception_exits),
              static_cast<unsigned long long>(
                  result.stats.native_helper_load_delay_entries),
              static_cast<unsigned long long>(
                  result.stats.native_helper_load_delay_passes),
              static_cast<unsigned long long>(
                  result.stats.native_helper_load_delay_fallbacks),
              static_cast<unsigned long long>(result.stats.native_code_bytes),
              static_cast<unsigned long long>(result.stats.decoded_instructions),
              static_cast<unsigned long long>(result.stats.fallback_instructions),
              static_cast<unsigned long long>(
                  result.stats.interpreter_fallback_steps));
        }
        log_cpu_compare_program(test_case);
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
  g_cpu_backend_compare_test_active = saved_compare_test_active;
  g_cpu_execution_mode_cli_override = saved_override;
  g_cpu_execution_mode_cli_value = saved_override_value;

  if (failures != 0) {
    LOG_ERROR("CPU backend compare test failed: %d case(s)", failures);
    return 1;
  }
  LOG_INFO("CPU backend compare test passed");
  return 0;
}
} // namespace

int run_cpu_backend_compare_test(bool memory_only) {
  return run_cpu_backend_compare_test_impl(memory_only);
}

