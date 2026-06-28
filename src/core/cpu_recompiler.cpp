#include "cpu_recompiler.h"
#include "system.h"
#include <algorithm>
#include <array>
#include <string>

namespace {
u64 delta_u64(u64 current, u64 previous) {
  return current >= previous ? current - previous : 0;
}

size_t delta_size(size_t current, size_t previous) {
  return current >= previous ? current - previous : 0;
}

const char *decoded_op_name(DecodedOp op) {
  switch (op) {
  case DecodedOp::Unsupported: return "Unsupported";
  case DecodedOp::Nop: return "Nop";
  case DecodedOp::Sll: return "Sll";
  case DecodedOp::Srl: return "Srl";
  case DecodedOp::Sra: return "Sra";
  case DecodedOp::Sllv: return "Sllv";
  case DecodedOp::Srlv: return "Srlv";
  case DecodedOp::Srav: return "Srav";
  case DecodedOp::Jr: return "Jr";
  case DecodedOp::Jalr: return "Jalr";
  case DecodedOp::Movz: return "Movz";
  case DecodedOp::Movn: return "Movn";
  case DecodedOp::Sync: return "Sync";
  case DecodedOp::Clear: return "Clear";
  case DecodedOp::Syscall: return "Syscall";
  case DecodedOp::Break: return "Break";
  case DecodedOp::Addu: return "Addu";
  case DecodedOp::Subu: return "Subu";
  case DecodedOp::And: return "And";
  case DecodedOp::Or: return "Or";
  case DecodedOp::Xor: return "Xor";
  case DecodedOp::Nor: return "Nor";
  case DecodedOp::Slt: return "Slt";
  case DecodedOp::Sltu: return "Sltu";
  case DecodedOp::Bltz: return "Bltz";
  case DecodedOp::Bgez: return "Bgez";
  case DecodedOp::Bltzal: return "Bltzal";
  case DecodedOp::Bgezal: return "Bgezal";
  case DecodedOp::J: return "J";
  case DecodedOp::Jal: return "Jal";
  case DecodedOp::Beq: return "Beq";
  case DecodedOp::Bne: return "Bne";
  case DecodedOp::Blez: return "Blez";
  case DecodedOp::Bgtz: return "Bgtz";
  case DecodedOp::Addiu: return "Addiu";
  case DecodedOp::Slti: return "Slti";
  case DecodedOp::Sltiu: return "Sltiu";
  case DecodedOp::Andi: return "Andi";
  case DecodedOp::Ori: return "Ori";
  case DecodedOp::Xori: return "Xori";
  case DecodedOp::Lui: return "Lui";
  case DecodedOp::Lb: return "Lb";
  case DecodedOp::Lh: return "Lh";
  case DecodedOp::Lw: return "Lw";
  case DecodedOp::Lbu: return "Lbu";
  case DecodedOp::Lhu: return "Lhu";
  case DecodedOp::Sb: return "Sb";
  case DecodedOp::Sh: return "Sh";
  case DecodedOp::Sw: return "Sw";
  case DecodedOp::Cop0: return "Cop0";
  case DecodedOp::Cop2: return "Cop2";
  default: return "Unknown";
  }
}

const char *native_reject_detail_name(NativeBlockRejectDetail detail) {
  switch (detail) {
  case NativeBlockRejectDetail::Branch: return "branch";
  case NativeBlockRejectDetail::Memory: return "memory";
  case NativeBlockRejectDetail::Cop0: return "cop0";
  case NativeBlockRejectDetail::Cop2: return "cop2_gte";
  case NativeBlockRejectDetail::ExceptionOrUnknown:
    return "exception_unknown";
  case NativeBlockRejectDetail::PcMismatch: return "pc_mismatch";
  case NativeBlockRejectDetail::NextPcMismatch: return "next_pc_mismatch";
  case NativeBlockRejectDetail::BlockStartAfterBranchDelay:
    return "block_start_after_branch_delay";
  case NativeBlockRejectDetail::StaleInvalidBlockState:
    return "stale_invalid_block_state";
  case NativeBlockRejectDetail::InDelaySlot: return "in_delay_slot";
  case NativeBlockRejectDetail::PendingDelaySlot:
    return "pending_delay_slot";
  case NativeBlockRejectDetail::PendingBranchTaken:
    return "pending_branch_taken";
  case NativeBlockRejectDetail::PendingBranchPc:
    return "pending_branch_pc";
  case NativeBlockRejectDetail::LoadDelayState: return "load_delay_state";
  case NativeBlockRejectDetail::IrqState: return "irq_state";
  case NativeBlockRejectDetail::InvalidatedState: return "invalidated_state";
  case NativeBlockRejectDetail::OtherState: return "other_state";
  case NativeBlockRejectDetail::Budget: return "budget";
  case NativeBlockRejectDetail::ICache: return "icache";
  case NativeBlockRejectDetail::Mmio: return "mmio";
  case NativeBlockRejectDetail::Unaligned: return "unaligned";
  case NativeBlockRejectDetail::ExceptionRisk: return "exception_risk";
  case NativeBlockRejectDetail::FallbackInstruction:
    return "fallback_instruction";
  case NativeBlockRejectDetail::UnsupportedInstruction:
    return "unsupported_instruction";
  case NativeBlockRejectDetail::Cold: return "cold";
  case NativeBlockRejectDetail::TooShort: return "too_short";
  case NativeBlockRejectDetail::CompileFailure: return "compile_failure";
  case NativeBlockRejectDetail::BranchTailMissingBranch:
    return "branch_tail_missing_branch";
  case NativeBlockRejectDetail::BranchTailUnsupportedBranch:
    return "branch_tail_unsupported_branch";
  case NativeBlockRejectDetail::BranchTailMissingDelaySlot:
    return "branch_tail_missing_delay_slot";
  case NativeBlockRejectDetail::BranchTailUnsupportedBody:
    return "branch_tail_unsupported_body";
  case NativeBlockRejectDetail::BranchTailUnsupportedDelaySlot:
    return "branch_tail_unsupported_delay_slot";
  case NativeBlockRejectDetail::BranchTailNestedDelayBranch:
    return "branch_tail_nested_delay_branch";
  case NativeBlockRejectDetail::BranchTailDisabled:
    return "branch_tail_disabled";
  case NativeBlockRejectDetail::BranchTailBlacklisted:
    return "branch_tail_blacklisted";
  case NativeBlockRejectDetail::ReducedHelperStore:
    return "reduced_helper_store";
  case NativeBlockRejectDetail::ReducedHelperLoadBaseWritten:
    return "reduced_helper_load_base_written";
  case NativeBlockRejectDetail::ReducedHelperUnsupportedMemory:
    return "reduced_helper_unsupported_memory";
  case NativeBlockRejectDetail::ReducedHelperPreflightDisabled:
    return "reduced_helper_preflight_disabled";
  case NativeBlockRejectDetail::None:
  default:
    return "none";
  }
}

const char *native_block_shape_name(NativeBlockShape shape) {
  switch (shape) {
  case NativeBlockShape::StraightAlu: return "straight_alu";
  case NativeBlockShape::StraightAluRamLoadCandidate:
    return "straight_alu_ram_load";
  case NativeBlockShape::StraightAluRamStoreCandidate:
    return "straight_alu_ram_store";
  case NativeBlockShape::HelperSafeMemory: return "helper_safe_memory";
  case NativeBlockShape::BranchTail: return "branch_tail";
  case NativeBlockShape::FallbackControl: return "fallback_control";
  case NativeBlockShape::Cop0: return "cop0";
  case NativeBlockShape::Cop2: return "cop2_gte";
  case NativeBlockShape::Mmio: return "mmio";
  case NativeBlockShape::ExceptionUnsafe: return "exception_unsafe";
  default: return "unknown";
  }
}

std::string decoded_ops_string(
    const std::array<DecodedOp, DecodedBlock::kMaxInstructions> &ops,
    u32 count) {
  std::string out;
  for (u32 i = 0; i < count && i < ops.size(); ++i) {
    if (!out.empty()) {
      out += ",";
    }
    out += decoded_op_name(ops[i]);
  }
  return out;
}

CpuBackendStats delta_stats(const CpuBackendStats &current,
                            const CpuBackendStats &previous) {
  CpuBackendStats out{};
  out.decoded_blocks =
      delta_u64(current.decoded_blocks, previous.decoded_blocks);
  out.native_blocks = current.native_blocks;
  out.native_compile_attempts =
      delta_u64(current.native_compile_attempts,
                previous.native_compile_attempts);
  out.native_compile_successes =
      delta_u64(current.native_compile_successes,
                previous.native_compile_successes);
  out.native_blocks_compiled =
      delta_u64(current.native_blocks_compiled,
                previous.native_blocks_compiled);
  out.native_branch_tail_blocks_compiled =
      delta_u64(current.native_branch_tail_blocks_compiled,
                previous.native_branch_tail_blocks_compiled);
  out.native_memory_blocks_compiled =
      delta_u64(current.native_memory_blocks_compiled,
                previous.native_memory_blocks_compiled);
  out.native_alu_blocks_compiled =
      delta_u64(current.native_alu_blocks_compiled,
                previous.native_alu_blocks_compiled);
  out.native_reduced_helper_compile_attempts = delta_u64(
      current.native_reduced_helper_compile_attempts,
      previous.native_reduced_helper_compile_attempts);
  out.native_reduced_helper_compile_successes = delta_u64(
      current.native_reduced_helper_compile_successes,
      previous.native_reduced_helper_compile_successes);
  out.native_reduced_helper_blocks_compiled = delta_u64(
      current.native_reduced_helper_blocks_compiled,
      previous.native_reduced_helper_blocks_compiled);
  out.native_reduced_helper_ram_load_blocks_compiled = delta_u64(
      current.native_reduced_helper_ram_load_blocks_compiled,
      previous.native_reduced_helper_ram_load_blocks_compiled);
  out.native_reduced_helper_rejected_blocks = delta_u64(
      current.native_reduced_helper_rejected_blocks,
      previous.native_reduced_helper_rejected_blocks);
  out.native_reduced_helper_rejected_instructions = delta_u64(
      current.native_reduced_helper_rejected_instructions,
      previous.native_reduced_helper_rejected_instructions);
  out.native_reduced_helper_reject_stores = delta_u64(
      current.native_reduced_helper_reject_stores,
      previous.native_reduced_helper_reject_stores);
  out.native_reduced_helper_reject_load_base_written = delta_u64(
      current.native_reduced_helper_reject_load_base_written,
      previous.native_reduced_helper_reject_load_base_written);
  out.native_reduced_helper_reject_unsupported_memory = delta_u64(
      current.native_reduced_helper_reject_unsupported_memory,
      previous.native_reduced_helper_reject_unsupported_memory);
  out.native_compile_failures =
      delta_u64(current.native_compile_failures,
                previous.native_compile_failures);
  out.native_rejected_unsafe_blocks =
      delta_u64(current.native_rejected_unsafe_blocks,
                previous.native_rejected_unsafe_blocks);
  out.native_to_decoded_fallbacks =
      delta_u64(current.native_to_decoded_fallbacks,
                previous.native_to_decoded_fallbacks);
  out.native_hot_threshold_skips =
      delta_u64(current.native_hot_threshold_skips,
                previous.native_hot_threshold_skips);
  out.native_short_block_skips =
      delta_u64(current.native_short_block_skips,
                previous.native_short_block_skips);
  out.native_reject_branch =
      delta_u64(current.native_reject_branch, previous.native_reject_branch);
  out.native_reject_memory =
      delta_u64(current.native_reject_memory, previous.native_reject_memory);
  out.native_reject_cop0 =
      delta_u64(current.native_reject_cop0, previous.native_reject_cop0);
  out.native_reject_cop2 =
      delta_u64(current.native_reject_cop2, previous.native_reject_cop2);
  out.native_reject_exception_unknown =
      delta_u64(current.native_reject_exception_unknown,
                previous.native_reject_exception_unknown);
  out.native_reject_exception_risk =
      delta_u64(current.native_reject_exception_risk,
                previous.native_reject_exception_risk);
  out.native_reject_fallback_instruction =
      delta_u64(current.native_reject_fallback_instruction,
                previous.native_reject_fallback_instruction);
  out.native_reject_unsupported_instruction =
      delta_u64(current.native_reject_unsupported_instruction,
                previous.native_reject_unsupported_instruction);
  out.native_reject_unsafe_state =
      delta_u64(current.native_reject_unsafe_state,
                previous.native_reject_unsafe_state);
  out.native_reject_pc_state =
      delta_u64(current.native_reject_pc_state,
                previous.native_reject_pc_state);
  out.native_reject_branch_delay_state =
      delta_u64(current.native_reject_branch_delay_state,
                previous.native_reject_branch_delay_state);
  out.native_reject_load_delay_state =
      delta_u64(current.native_reject_load_delay_state,
                previous.native_reject_load_delay_state);
  out.native_reject_irq_state =
      delta_u64(current.native_reject_irq_state,
                previous.native_reject_irq_state);
  out.native_reject_invalidated_state =
      delta_u64(current.native_reject_invalidated_state,
                previous.native_reject_invalidated_state);
  out.native_reject_other_state =
      delta_u64(current.native_reject_other_state,
                previous.native_reject_other_state);
  out.native_reject_pc_mismatch =
      delta_u64(current.native_reject_pc_mismatch,
                previous.native_reject_pc_mismatch);
  out.native_reject_next_pc_mismatch =
      delta_u64(current.native_reject_next_pc_mismatch,
                previous.native_reject_next_pc_mismatch);
  out.native_reject_block_start_after_branch_delay =
      delta_u64(current.native_reject_block_start_after_branch_delay,
                previous.native_reject_block_start_after_branch_delay);
  out.native_reject_stale_invalid_block_state =
      delta_u64(current.native_reject_stale_invalid_block_state,
                previous.native_reject_stale_invalid_block_state);
  out.native_reject_branch_delay_in_delay_slot =
      delta_u64(current.native_reject_branch_delay_in_delay_slot,
                previous.native_reject_branch_delay_in_delay_slot);
  out.native_reject_branch_delay_pending_delay_slot =
      delta_u64(current.native_reject_branch_delay_pending_delay_slot,
                previous.native_reject_branch_delay_pending_delay_slot);
  out.native_reject_branch_delay_pending_branch_taken =
      delta_u64(current.native_reject_branch_delay_pending_branch_taken,
                previous.native_reject_branch_delay_pending_branch_taken);
  out.native_reject_branch_delay_pending_branch_pc =
      delta_u64(current.native_reject_branch_delay_pending_branch_pc,
                previous.native_reject_branch_delay_pending_branch_pc);
  out.native_reject_budget =
      delta_u64(current.native_reject_budget, previous.native_reject_budget);
  out.native_reject_icache =
      delta_u64(current.native_reject_icache, previous.native_reject_icache);
  out.native_reject_mmio =
      delta_u64(current.native_reject_mmio, previous.native_reject_mmio);
  out.native_reject_unaligned =
      delta_u64(current.native_reject_unaligned,
                previous.native_reject_unaligned);
  out.native_branch_tail_rejects =
      delta_u64(current.native_branch_tail_rejects,
                previous.native_branch_tail_rejects);
  out.native_branch_tail_disabled_fallbacks =
      delta_u64(current.native_branch_tail_disabled_fallbacks,
                previous.native_branch_tail_disabled_fallbacks);
  out.native_branch_tail_blacklisted_fallbacks =
      delta_u64(current.native_branch_tail_blacklisted_fallbacks,
                previous.native_branch_tail_blacklisted_fallbacks);
  out.native_all_disabled_fallbacks =
      delta_u64(current.native_all_disabled_fallbacks,
                previous.native_all_disabled_fallbacks);
  out.native_memory_disabled_fallbacks =
      delta_u64(current.native_memory_disabled_fallbacks,
                previous.native_memory_disabled_fallbacks);
  out.native_load_disabled_fallbacks =
      delta_u64(current.native_load_disabled_fallbacks,
                previous.native_load_disabled_fallbacks);
  out.native_store_disabled_fallbacks =
      delta_u64(current.native_store_disabled_fallbacks,
                previous.native_store_disabled_fallbacks);
  out.native_mmio_disabled_fallbacks =
      delta_u64(current.native_mmio_disabled_fallbacks,
                previous.native_mmio_disabled_fallbacks);
  out.native_ram_disabled_fallbacks =
      delta_u64(current.native_ram_disabled_fallbacks,
                previous.native_ram_disabled_fallbacks);
  out.native_load_delay_disabled_fallbacks =
      delta_u64(current.native_load_delay_disabled_fallbacks,
                previous.native_load_delay_disabled_fallbacks);
  out.native_mixed_memory_disabled_fallbacks =
      delta_u64(current.native_mixed_memory_disabled_fallbacks,
                previous.native_mixed_memory_disabled_fallbacks);
  out.native_alu_disabled_fallbacks =
      delta_u64(current.native_alu_disabled_fallbacks,
                previous.native_alu_disabled_fallbacks);
  out.native_rejected_block_count =
      delta_u64(current.native_rejected_block_count,
                previous.native_rejected_block_count);
  out.native_rejected_block_instructions =
      delta_u64(current.native_rejected_block_instructions,
                previous.native_rejected_block_instructions);
  out.cache_hits = delta_u64(current.cache_hits, previous.cache_hits);
  out.cache_misses = delta_u64(current.cache_misses, previous.cache_misses);
  out.invalidations =
      delta_u64(current.invalidations, previous.invalidations);
  out.invalidation_queries =
      delta_u64(current.invalidation_queries, previous.invalidation_queries);
  out.invalidation_fast_no_code_page_exits =
      delta_u64(current.invalidation_fast_no_code_page_exits,
                previous.invalidation_fast_no_code_page_exits);
  out.invalidation_blocks_examined =
      delta_u64(current.invalidation_blocks_examined,
                previous.invalidation_blocks_examined);
  out.invalidation_blocks_invalidated =
      delta_u64(current.invalidation_blocks_invalidated,
                previous.invalidation_blocks_invalidated);
  out.flushes = delta_u64(current.flushes, previous.flushes);
  out.decoded_block_entries =
      delta_u64(current.decoded_block_entries,
                previous.decoded_block_entries);
  out.native_block_entries =
      delta_u64(current.native_block_entries, previous.native_block_entries);
  out.native_branch_tail_entries =
      delta_u64(current.native_branch_tail_entries,
                previous.native_branch_tail_entries);
  out.native_branch_taken =
      delta_u64(current.native_branch_taken, previous.native_branch_taken);
  out.native_branch_not_taken =
      delta_u64(current.native_branch_not_taken,
                previous.native_branch_not_taken);
  out.native_branch_tail_bgtz_entries =
      delta_u64(current.native_branch_tail_bgtz_entries,
                previous.native_branch_tail_bgtz_entries);
  out.native_branch_tail_blez_entries =
      delta_u64(current.native_branch_tail_blez_entries,
                previous.native_branch_tail_blez_entries);
  out.native_branch_tail_bgtz_taken =
      delta_u64(current.native_branch_tail_bgtz_taken,
                previous.native_branch_tail_bgtz_taken);
  out.native_branch_tail_bgtz_not_taken =
      delta_u64(current.native_branch_tail_bgtz_not_taken,
                previous.native_branch_tail_bgtz_not_taken);
  out.native_branch_tail_blez_taken =
      delta_u64(current.native_branch_tail_blez_taken,
                previous.native_branch_tail_blez_taken);
  out.native_branch_tail_blez_not_taken =
      delta_u64(current.native_branch_tail_blez_not_taken,
                previous.native_branch_tail_blez_not_taken);
  out.native_branch_tail_reject_bne =
      delta_u64(current.native_branch_tail_reject_bne,
                previous.native_branch_tail_reject_bne);
  out.native_branch_tail_reject_beq =
      delta_u64(current.native_branch_tail_reject_beq,
                previous.native_branch_tail_reject_beq);
  out.native_branch_tail_reject_bgtz =
      delta_u64(current.native_branch_tail_reject_bgtz,
                previous.native_branch_tail_reject_bgtz);
  out.native_branch_tail_reject_blez =
      delta_u64(current.native_branch_tail_reject_blez,
                previous.native_branch_tail_reject_blez);
  out.native_branch_tail_reject_other_opcode =
      delta_u64(current.native_branch_tail_reject_other_opcode,
                previous.native_branch_tail_reject_other_opcode);
  out.native_memory_block_entries =
      delta_u64(current.native_memory_block_entries,
                previous.native_memory_block_entries);
  out.native_alu_block_entries =
      delta_u64(current.native_alu_block_entries,
                previous.native_alu_block_entries);
  out.native_reduced_helper_entries = delta_u64(
      current.native_reduced_helper_entries,
      previous.native_reduced_helper_entries);
  out.native_reduced_helper_ram_load_entries = delta_u64(
      current.native_reduced_helper_ram_load_entries,
      previous.native_reduced_helper_ram_load_entries);
  out.native_reduced_helper_instructions = delta_u64(
      current.native_reduced_helper_instructions,
      previous.native_reduced_helper_instructions);
  out.native_reduced_helper_fallbacks = delta_u64(
      current.native_reduced_helper_fallbacks,
      previous.native_reduced_helper_fallbacks);
  out.native_reduced_helper_load_delay_entries = delta_u64(
      current.native_reduced_helper_load_delay_entries,
      previous.native_reduced_helper_load_delay_entries);
  out.native_reduced_helper_ram_load_preflight_fallbacks = delta_u64(
      current.native_reduced_helper_ram_load_preflight_fallbacks,
      previous.native_reduced_helper_ram_load_preflight_fallbacks);
  out.native_reduced_helper_ram_load_preflight_mmio = delta_u64(
      current.native_reduced_helper_ram_load_preflight_mmio,
      previous.native_reduced_helper_ram_load_preflight_mmio);
  out.native_reduced_helper_ram_load_preflight_unaligned = delta_u64(
      current.native_reduced_helper_ram_load_preflight_unaligned,
      previous.native_reduced_helper_ram_load_preflight_unaligned);
  out.native_reduced_helper_ram_load_preflight_non_ram = delta_u64(
      current.native_reduced_helper_ram_load_preflight_non_ram,
      previous.native_reduced_helper_ram_load_preflight_non_ram);
  out.native_reduced_helper_ram_load_preflight_disabled = delta_u64(
      current.native_reduced_helper_ram_load_preflight_disabled,
      previous.native_reduced_helper_ram_load_preflight_disabled);
  out.native_direct_helper_candidate_blocks = delta_u64(
      current.native_direct_helper_candidate_blocks,
      previous.native_direct_helper_candidate_blocks);
  out.native_reduced_helper_candidate_blocks = delta_u64(
      current.native_reduced_helper_candidate_blocks,
      previous.native_reduced_helper_candidate_blocks);
  out.native_shape_straight_alu = delta_u64(
      current.native_shape_straight_alu,
      previous.native_shape_straight_alu);
  out.native_shape_straight_alu_ram_load = delta_u64(
      current.native_shape_straight_alu_ram_load,
      previous.native_shape_straight_alu_ram_load);
  out.native_shape_straight_alu_ram_store = delta_u64(
      current.native_shape_straight_alu_ram_store,
      previous.native_shape_straight_alu_ram_store);
  out.native_shape_helper_safe_memory = delta_u64(
      current.native_shape_helper_safe_memory,
      previous.native_shape_helper_safe_memory);
  out.native_shape_branch_tail = delta_u64(
      current.native_shape_branch_tail,
      previous.native_shape_branch_tail);
  out.native_shape_fallback_control = delta_u64(
      current.native_shape_fallback_control,
      previous.native_shape_fallback_control);
  out.native_shape_cop0 =
      delta_u64(current.native_shape_cop0, previous.native_shape_cop0);
  out.native_shape_cop2 =
      delta_u64(current.native_shape_cop2, previous.native_shape_cop2);
  out.native_shape_exception_unsafe = delta_u64(
      current.native_shape_exception_unsafe,
      previous.native_shape_exception_unsafe);
  out.compare_flag_leak_warnings =
      delta_u64(current.compare_flag_leak_warnings,
                previous.compare_flag_leak_warnings);
  out.optimized_instructions =
      delta_u64(current.optimized_instructions,
                previous.optimized_instructions);
  out.decoded_instructions =
      delta_u64(current.decoded_instructions, previous.decoded_instructions);
  out.native_instructions =
      delta_u64(current.native_instructions, previous.native_instructions);
  out.native_cycles =
      delta_u64(current.native_cycles, previous.native_cycles);
  out.fallback_instructions =
      delta_u64(current.fallback_instructions,
                previous.fallback_instructions);
  out.interpreter_fallback_steps =
      delta_u64(current.interpreter_fallback_steps,
                previous.interpreter_fallback_steps);
  out.forced_interpreter_slices =
      delta_u64(current.forced_interpreter_slices,
                previous.forced_interpreter_slices);
  out.forced_interpreter_instructions =
      delta_u64(current.forced_interpreter_instructions,
                previous.forced_interpreter_instructions);
  out.forced_interpreter_trace =
      delta_u64(current.forced_interpreter_trace,
                previous.forced_interpreter_trace);
  out.forced_interpreter_deep_diagnostics =
      delta_u64(current.forced_interpreter_deep_diagnostics,
                previous.forced_interpreter_deep_diagnostics);
  out.forced_interpreter_fmv =
      delta_u64(current.forced_interpreter_fmv,
                previous.forced_interpreter_fmv);
  out.forced_interpreter_backend_compare =
      delta_u64(current.forced_interpreter_backend_compare,
                previous.forced_interpreter_backend_compare);
  out.forced_interpreter_other =
      delta_u64(current.forced_interpreter_other,
                previous.forced_interpreter_other);
  out.forced_interpreter_last_reason =
      current.forced_interpreter_last_reason;
  out.memory_helper_calls =
      delta_u64(current.memory_helper_calls, previous.memory_helper_calls);
  out.native_prepare_helper_calls =
      delta_u64(current.native_prepare_helper_calls,
                previous.native_prepare_helper_calls);
  out.native_finish_helper_calls =
      delta_u64(current.native_finish_helper_calls,
                previous.native_finish_helper_calls);
  out.native_branch_helper_calls =
      delta_u64(current.native_branch_helper_calls,
                previous.native_branch_helper_calls);
  out.native_memory_helper_calls =
      delta_u64(current.native_memory_helper_calls,
                previous.native_memory_helper_calls);
  out.native_memory_helper_load_calls =
      delta_u64(current.native_memory_helper_load_calls,
                previous.native_memory_helper_load_calls);
  out.native_memory_helper_store_calls =
      delta_u64(current.native_memory_helper_store_calls,
                previous.native_memory_helper_store_calls);
  out.native_memory_helper_ram_calls =
      delta_u64(current.native_memory_helper_ram_calls,
                previous.native_memory_helper_ram_calls);
  out.native_memory_helper_scratchpad_calls =
      delta_u64(current.native_memory_helper_scratchpad_calls,
                previous.native_memory_helper_scratchpad_calls);
  out.native_memory_helper_bios_calls =
      delta_u64(current.native_memory_helper_bios_calls,
                previous.native_memory_helper_bios_calls);
  out.native_memory_helper_mmio_calls =
      delta_u64(current.native_memory_helper_mmio_calls,
                previous.native_memory_helper_mmio_calls);
  out.native_memory_helper_unknown_calls =
      delta_u64(current.native_memory_helper_unknown_calls,
                previous.native_memory_helper_unknown_calls);
  out.native_memory_helper_unaligned_calls =
      delta_u64(current.native_memory_helper_unaligned_calls,
                previous.native_memory_helper_unaligned_calls);
  out.native_memory_fastpath_loads =
      delta_u64(current.native_memory_fastpath_loads,
                previous.native_memory_fastpath_loads);
  out.native_memory_fastpath_stores =
      delta_u64(current.native_memory_fastpath_stores,
                previous.native_memory_fastpath_stores);
  out.native_memory_fastpath_mmio_loads =
      delta_u64(current.native_memory_fastpath_mmio_loads,
                previous.native_memory_fastpath_mmio_loads);
  out.native_memory_fastpath_load_misses =
      delta_u64(current.native_memory_fastpath_load_misses,
                previous.native_memory_fastpath_load_misses);
  out.native_memory_fastpath_load_miss_disabled =
      delta_u64(current.native_memory_fastpath_load_miss_disabled,
                previous.native_memory_fastpath_load_miss_disabled);
  out.native_memory_fastpath_load_miss_trace =
      delta_u64(current.native_memory_fastpath_load_miss_trace,
                previous.native_memory_fastpath_load_miss_trace);
  out.native_memory_fastpath_load_miss_unaligned =
      delta_u64(current.native_memory_fastpath_load_miss_unaligned,
                previous.native_memory_fastpath_load_miss_unaligned);
  out.native_memory_fastpath_load_miss_non_ram =
      delta_u64(current.native_memory_fastpath_load_miss_non_ram,
                previous.native_memory_fastpath_load_miss_non_ram);
  out.native_memory_exception_exits =
      delta_u64(current.native_memory_exception_exits,
                previous.native_memory_exception_exits);
  out.native_memory_operand_mismatches =
      delta_u64(current.native_memory_operand_mismatches,
                previous.native_memory_operand_mismatches);
  out.native_memory_shared_decoded_calls =
      delta_u64(current.native_memory_shared_decoded_calls,
                previous.native_memory_shared_decoded_calls);
  out.native_branch_delay_slot_memory_helpers =
      delta_u64(current.native_branch_delay_slot_memory_helpers,
                previous.native_branch_delay_slot_memory_helpers);
  out.native_helper_load_delay_entries =
      delta_u64(current.native_helper_load_delay_entries,
                previous.native_helper_load_delay_entries);
  out.native_helper_load_delay_passes =
      delta_u64(current.native_helper_load_delay_passes,
                previous.native_helper_load_delay_passes);
  out.native_helper_load_delay_fallbacks =
      delta_u64(current.native_helper_load_delay_fallbacks,
                previous.native_helper_load_delay_fallbacks);
  out.mmio_accesses =
      delta_u64(current.mmio_accesses, previous.mmio_accesses);
  out.exceptions = delta_u64(current.exceptions, previous.exceptions);
  out.interrupt_exits =
      delta_u64(current.interrupt_exits, previous.interrupt_exits);
  out.budget_exits =
      delta_u64(current.budget_exits, previous.budget_exits);
  out.fallback_exits =
      delta_u64(current.fallback_exits, previous.fallback_exits);
  out.pc_mismatch_exits =
      delta_u64(current.pc_mismatch_exits, previous.pc_mismatch_exits);
  out.invalidated_exits =
      delta_u64(current.invalidated_exits, previous.invalidated_exits);
  out.executed_cycles =
      delta_u64(current.executed_cycles, previous.executed_cycles);
  out.code_bytes = current.code_bytes;
  out.native_code_bytes =
      delta_size(current.native_code_bytes, previous.native_code_bytes);
  return out;
}
} // namespace

DecodedBlock::~DecodedBlock() {
  if (destroy_native_context != nullptr && native_context != nullptr) {
    destroy_native_context(native_context);
  }
  native_context = nullptr;
  native_fn = nullptr;
  destroy_native_context = nullptr;
  native_code_bytes = 0;
}

CpuOptimizedBackend::CpuOptimizedBackend(Cpu &cpu) : cpu_(cpu) {
  stats_.available = true;
}

CpuOptimizedBackend::~CpuOptimizedBackend() = default;

CpuRunSliceResult CpuOptimizedBackend::run_slice(
    u32 max_cycles, u32 max_instructions, CpuExecutionMode requested_mode) {
  CpuRunSliceResult total{};
  if (max_cycles == 0 || max_instructions == 0) {
    return total;
  }

  stats_.active = true;
  stats_.available = true;
  if (!g_cpu_backend_compare_test_active &&
      (g_cpu_backend_compare_irq_on_branch ||
       g_cpu_backend_compare_allow_partial_branch_tail ||
       g_cpu_backend_compare_allow_partial_memory_helper ||
       g_cpu_backend_compare_test_force_interpreter)) {
    ++stats_.compare_flag_leak_warnings;
    LOG_WARN("CPU compare-only flags leaked outside compare test; clearing irq_on_branch=%u partial_branch_tail=%u partial_memory=%u force_interpreter=%u",
             g_cpu_backend_compare_irq_on_branch ? 1u : 0u,
             g_cpu_backend_compare_allow_partial_branch_tail ? 1u : 0u,
             g_cpu_backend_compare_allow_partial_memory_helper ? 1u : 0u,
             g_cpu_backend_compare_test_force_interpreter ? 1u : 0u);
    g_cpu_backend_compare_irq_on_branch = false;
    g_cpu_backend_compare_allow_partial_branch_tail = false;
    g_cpu_backend_compare_allow_partial_memory_helper = false;
    g_cpu_backend_compare_test_force_interpreter = false;
    assert(!g_cpu_backend_compare_irq_on_branch &&
           !g_cpu_backend_compare_allow_partial_branch_tail &&
           !g_cpu_backend_compare_allow_partial_memory_helper &&
           !g_cpu_backend_compare_test_force_interpreter);
  }
  const CpuForcedInterpreterReason forced_reason =
      diagnostics_force_interpreter_reason();
  if (forced_reason != CpuForcedInterpreterReason::None) {
    stats_.forced_interpreter_last_reason = forced_reason;
    ++stats_.forced_interpreter_slices;
    warn_forced_interpreter_once(forced_reason);
    u32 forced_instructions = 0;
    while (total.cycles < max_cycles && total.instructions < max_instructions) {
      const u32 consumed = cpu_.step();
      total.cycles += consumed;
      ++total.instructions;
      ++forced_instructions;
      ++stats_.interpreter_fallback_steps;
      ++stats_.fallback_instructions;
    }
    record_forced_interpreter(forced_reason, forced_instructions);
    return total;
  }
  stats_.forced_interpreter_last_reason = CpuForcedInterpreterReason::None;

  if (requested_mode == CpuExecutionMode::X64Jit && !x64_jit_available()) {
    warn_x64_unavailable_once();
  }

  while (total.cycles < max_cycles && total.instructions < max_instructions) {
    DecodedBlock *block = lookup_or_decode(cpu_.pc_);
    if (block == nullptr ||
        block->interpreter_only_until_frame > current_frame_) {
      const u32 consumed = cpu_.step();
      total.cycles += consumed;
      ++total.instructions;
      ++stats_.interpreter_fallback_steps;
      ++stats_.fallback_instructions;
      continue;
    }

    ++block->entry_count;
    const u32 cycle_budget = max_cycles - total.cycles;
    const u32 instruction_budget = max_instructions - total.instructions;
    CpuBlockRunResult result{};
    const bool requested_x64_native =
        requested_mode == CpuExecutionMode::X64Jit && x64_jit_available();
    const bool try_native =
        requested_x64_native && cpu_x64_jit_all_native_enabled();
    if (try_native) {
      if (!ensure_x64_safety_checked(*block)) {
        ++stats_.native_to_decoded_fallbacks;
        record_native_block_rejection(
            *block, block->native_reject_detail);
        result = execute_block(*block, cycle_budget, instruction_budget);
      } else if (block->native_branch_tail &&
                 !cpu_x64_jit_branch_tail_enabled()) {
        ++stats_.native_to_decoded_fallbacks;
        ++stats_.native_branch_tail_disabled_fallbacks;
        ++stats_.native_rejected_block_count;
        stats_.native_rejected_block_instructions += block->instruction_count;
        record_native_block_rejection(*block,
                                      NativeBlockRejectDetail::BranchTailDisabled);
        result = execute_block(*block, cycle_budget, instruction_budget);
      } else if (block->native_branch_tail &&
                 native_branch_tail_pc_blacklisted(*block)) {
        ++stats_.native_to_decoded_fallbacks;
        ++stats_.native_branch_tail_blacklisted_fallbacks;
        ++stats_.native_rejected_block_count;
        stats_.native_rejected_block_instructions += block->instruction_count;
        record_native_block_rejection(
            *block, NativeBlockRejectDetail::BranchTailBlacklisted);
        result = execute_block(*block, cycle_budget, instruction_budget);
      } else if (block->has_memory &&
                 !cpu_x64_jit_native_memory_enabled()) {
        ++stats_.native_to_decoded_fallbacks;
        ++stats_.native_memory_disabled_fallbacks;
        result = execute_block(*block, cycle_budget, instruction_budget);
      } else if (block->has_load &&
                 g_cpu_x64_jit_disable_native_loads) {
        ++stats_.native_to_decoded_fallbacks;
        ++stats_.native_load_disabled_fallbacks;
        result = execute_block(*block, cycle_budget, instruction_budget);
      } else if (block->has_store &&
                 g_cpu_x64_jit_disable_native_stores) {
        ++stats_.native_to_decoded_fallbacks;
        ++stats_.native_store_disabled_fallbacks;
        result = execute_block(*block, cycle_budget, instruction_budget);
      } else if (block->has_load && block->has_store &&
                 g_cpu_x64_jit_disable_native_mixed_load_store) {
        ++stats_.native_to_decoded_fallbacks;
        ++stats_.native_mixed_memory_disabled_fallbacks;
        result = execute_block(*block, cycle_budget, instruction_budget);
      } else if (block->has_memory &&
                 g_cpu_x64_jit_disable_native_load_delay &&
                 (cpu_.load_.reg != 0u || cpu_.next_load_.reg != 0u)) {
        ++stats_.native_to_decoded_fallbacks;
        ++stats_.native_load_delay_disabled_fallbacks;
        result = execute_block(*block, cycle_budget, instruction_budget);
      } else if (block->native_memory_runtime_filter_reason != 0u) {
        ++stats_.native_to_decoded_fallbacks;
        if (block->native_memory_runtime_filter_reason == 1u) {
          ++stats_.native_mmio_disabled_fallbacks;
        } else {
          ++stats_.native_ram_disabled_fallbacks;
        }
        result = execute_block(*block, cycle_budget, instruction_budget);
      } else if (!block->native_branch_tail && !block->has_memory &&
                 !cpu_x64_jit_native_alu_enabled()) {
        ++stats_.native_to_decoded_fallbacks;
        ++stats_.native_alu_disabled_fallbacks;
        result = execute_block(*block, cycle_budget, instruction_budget);
      } else if (block->instruction_count > instruction_budget &&
                 !(block->native_branch_tail &&
                   g_cpu_backend_compare_allow_partial_branch_tail) &&
                 !(block->has_memory &&
                   g_cpu_backend_compare_allow_partial_memory_helper)) {
        ++stats_.native_to_decoded_fallbacks;
        ++stats_.native_reject_budget;
        ++stats_.native_rejected_block_count;
        stats_.native_rejected_block_instructions += block->instruction_count;
        record_native_block_rejection(*block, NativeBlockRejectDetail::Budget);
        result = execute_block(*block, cycle_budget, instruction_budget);
      } else if (!should_attempt_x64_compile(*block)) {
        ++stats_.native_to_decoded_fallbacks;
        result = execute_block(*block, cycle_budget, instruction_budget);
      } else {
        if (!block->native_compile_attempted) {
          (void)compile_x64_block(*block);
        }
        if (block->native_fn != nullptr) {
          result =
              execute_native_block(*block, cycle_budget, instruction_budget);
        } else {
          ++stats_.native_to_decoded_fallbacks;
          record_native_block_rejection(
              *block, NativeBlockRejectDetail::CompileFailure);
          result = execute_block(*block, cycle_budget, instruction_budget);
        }
      }
    } else {
      if (requested_x64_native && !cpu_x64_jit_all_native_enabled()) {
        ++stats_.native_all_disabled_fallbacks;
      }
      result = execute_block(*block, cycle_budget, instruction_budget);
    }
    record_exit(result.exit_reason);

    if (result.instructions == 0) {
      const u32 consumed = cpu_.step();
      total.cycles += consumed;
      ++total.instructions;
      ++stats_.interpreter_fallback_steps;
      ++stats_.fallback_instructions;
      continue;
    }

    total.cycles += result.cycles;
    total.instructions += result.instructions;
    stats_.executed_cycles += result.cycles;
  }

  return total;
}

void CpuOptimizedBackend::invalidate_range(u32 phys_or_normalized_addr,
                                           u32 size_bytes) {
  ++stats_.invalidation_queries;
  if (size_bytes == 0 || blocks_.empty() || blocks_by_page_.empty()) {
    ++stats_.invalidation_fast_no_code_page_exits;
    return;
  }

  const u32 start = normalized_code_addr(phys_or_normalized_addr);
  const u64 end64 =
      std::min<u64>(static_cast<u64>(start) + static_cast<u64>(size_bytes),
                    0x100000000ull);
  if (end64 <= start) {
    ++stats_.invalidation_fast_no_code_page_exits;
    return;
  }

  const u32 start_page = code_page_for_normalized_addr(start);
  const u32 end_page =
      code_page_for_normalized_addr(static_cast<u32>(end64 - 1u));
  bool any_code_page = false;
  for (u32 page = start_page; page <= end_page; ++page) {
    if (code_page_maybe_compiled(page)) {
      any_code_page = true;
      break;
    }
  }
  if (!any_code_page) {
    ++stats_.invalidation_fast_no_code_page_exits;
    return;
  }

  u32 query_stamp = ++invalidation_query_stamp_;
  if (query_stamp == 0) {
    for (auto &entry : blocks_) {
      entry.second->last_invalidation_query = 0;
    }
    query_stamp = ++invalidation_query_stamp_;
  }

  const u32 end = static_cast<u32>(end64);
  for (u32 page = start_page; page <= end_page; ++page) {
    if (!code_page_maybe_compiled(page)) {
      continue;
    }
    auto page_blocks = blocks_by_page_.find(page);
    if (page_blocks == blocks_by_page_.end()) {
      clear_code_page_compiled(page);
      continue;
    }

    for (DecodedBlock *block : page_blocks->second) {
      if (block == nullptr || block->last_invalidation_query == query_stamp) {
        continue;
      }
      block->last_invalidation_query = query_stamp;
      if (block->invalidated) {
        continue;
      }

      ++stats_.invalidation_blocks_examined;
      for (u32 i = 0; i < block->tracked_range_count; ++i) {
        const auto &range = block->tracked_ranges[i];
        if (ranges_overlap(start, end, range.first, range.second)) {
          mark_block_invalidated(*block);
          ++stats_.invalidation_blocks_invalidated;
          break;
        }
      }
    }
  }
}

void CpuOptimizedBackend::begin_frame(u32 frame_index) {
  current_frame_ = frame_index;
  stats_.active = effective_cpu_execution_mode() != CpuExecutionMode::Interpreter;
  stats_.available = true;
  log_periodic_stats();
}

void CpuOptimizedBackend::flush() {
  blocks_.clear();
  blocks_by_page_.clear();
  rejected_block_profiles_.clear();
  recent_native_branch_tails_.clear();
  native_branch_tail_trace_sequence_ = 0;
  compiled_code_page_bitmap_.fill(0);
  stats_.block_count = 0;
  stats_.interpreter_only_blocks = 0;
  stats_.code_bytes = 0;
  have_stats_log_snapshot_ = false;
  last_stats_log_ = CpuBackendStats{};
  last_stats_log_frame_ = current_frame_;
  ++stats_.flushes;
}

CpuBackendStats CpuOptimizedBackend::stats() const {
  CpuBackendStats out = stats_;
  out.available = true;
  out.native_available = x64_jit_available();
  out.active = effective_cpu_execution_mode() != CpuExecutionMode::Interpreter;
  out.block_count = static_cast<u32>(blocks_.size());
  out.interpreter_only_blocks = 0;
  out.code_bytes = 0;
  out.native_blocks = 0;
  out.native_code_bytes = 0;
  for (const auto &entry : blocks_) {
    const DecodedBlock &block = *entry.second;
    if (block.interpreter_only_until_frame > current_frame_) {
      ++out.interpreter_only_blocks;
    }
    out.code_bytes += sizeof(DecodedBlock);
    if (block.native_fn != nullptr) {
      ++out.native_blocks;
      out.native_code_bytes += block.native_code_bytes;
    }
  }
  return out;
}

bool CpuOptimizedBackend::should_attempt_x64_compile(
    const DecodedBlock &block) {
  if (g_cpu_x64_jit_force_compile) {
    return true;
  }
  if (block.instruction_count < g_cpu_x64_jit_min_block_instructions) {
    ++stats_.native_short_block_skips;
    ++stats_.native_rejected_block_count;
    stats_.native_rejected_block_instructions += block.instruction_count;
    record_native_block_rejection(block, NativeBlockRejectDetail::TooShort);
    return false;
  }
  if (block.entry_count < g_cpu_x64_jit_hot_block_threshold) {
    ++stats_.native_hot_threshold_skips;
    ++stats_.native_rejected_block_count;
    stats_.native_rejected_block_instructions += block.instruction_count;
    record_native_block_rejection(block, NativeBlockRejectDetail::Cold);
    return false;
  }
  return true;
}

DecodedBlock *CpuOptimizedBackend::lookup_or_decode(u32 pc) {
  const u32 key = normalized_code_addr(pc);
  auto history = block_history_.find(key);
  if (history != block_history_.end() &&
      history->second.interpreter_only_until_frame > current_frame_) {
    return nullptr;
  }

  auto existing = blocks_.find(key);
  if (existing != blocks_.end()) {
    DecodedBlock *block = existing->second.get();
    if (!block->invalidated) {
      ++stats_.cache_hits;
      return block;
    }
    unregister_block_pages(*block);
    blocks_.erase(existing);
  }

  ++stats_.cache_misses;
  return decode_block(pc);
}

DecodedBlock *CpuOptimizedBackend::decode_block(u32 pc) {
  if ((pc & 3u) != 0) {
    return nullptr;
  }
  if (blocks_.size() >= kMaxDecodedBlocks) {
    flush();
  }

  auto block = std::make_unique<DecodedBlock>();
  block->start_pc = pc;
  block->start_key = normalized_code_addr(pc);
  block->compile_frame = current_frame_;

  BlockHistory &history = block_history_[block->start_key];
  if (current_frame_ - history.window_start_frame >
      kInterpreterFallbackFrames) {
    history.window_start_frame = current_frame_;
    history.compile_count_in_window = 0;
    history.invalidation_count_in_window = 0;
  }
  ++history.compile_count_in_window;
  if (history.compile_count_in_window > kInterpreterFallbackRecompiles) {
    history.interpreter_only_until_frame =
        current_frame_ + kInterpreterOnlyFrames;
    return nullptr;
  }
  block->interpreter_only_until_frame = history.interpreter_only_until_frame;

  u32 next_pc = pc;
  for (u32 index = 0; index < kMaxBlockInstructions; ++index) {
    const u32 bits = cpu_.read_instruction_for_backend(next_pc);
    const DecodedInstruction inst = decode_instruction(next_pc, bits);
    if (!append_decoded_instruction(*block, inst)) {
      return nullptr;
    }

    if (inst.must_fallback) {
      break;
    }

    if (is_block_terminator(inst)) {
      block->has_control_flow = true;
      const u32 delay_pc = next_pc + 4u;
      const u32 delay_bits = cpu_.read_instruction_for_backend(delay_pc);
      const DecodedInstruction delay_inst =
          decode_instruction(delay_pc, delay_bits);
      if (!append_decoded_instruction(*block, delay_inst)) {
        return nullptr;
      }
      break;
    }

    next_pc += 4u;
  }

  DecodedBlock *raw = block.get();
  ++stats_.decoded_blocks;
  blocks_[raw->start_key] = std::move(block);
  register_block_pages(*raw);
  stats_.block_count = static_cast<u32>(blocks_.size());
  return raw;
}

bool CpuOptimizedBackend::append_decoded_instruction(
    DecodedBlock &block, const DecodedInstruction &inst) {
  if (block.instruction_count >= block.instructions.size() ||
      block.tracked_range_count >= block.tracked_ranges.size()) {
    return false;
  }

  block.instructions[block.instruction_count++] = inst;
  block.tracked_ranges[block.tracked_range_count++] =
      std::make_pair(inst.normalized_addr, inst.normalized_addr + 4u);
  block.has_fallback = block.has_fallback || inst.must_fallback;
  block.has_memory = block.has_memory || inst.may_access_memory;
  switch (inst.op) {
  case DecodedOp::Lb:
  case DecodedOp::Lh:
  case DecodedOp::Lw:
  case DecodedOp::Lbu:
  case DecodedOp::Lhu:
    block.has_load = true;
    break;
  case DecodedOp::Sb:
  case DecodedOp::Sh:
  case DecodedOp::Sw:
    block.has_store = true;
    break;
  default:
    break;
  }
  return true;
}

DecodedInstruction CpuOptimizedBackend::decode_instruction(u32 pc,
                                                           u32 bits) const {
  DecodedInstruction out{};
  out.pc = pc;
  out.bits = bits;
  out.normalized_addr = normalized_code_addr(pc);
  out.rs = static_cast<u8>((bits >> 21) & 0x1Fu);
  out.rt = static_cast<u8>((bits >> 16) & 0x1Fu);
  out.rd = static_cast<u8>((bits >> 11) & 0x1Fu);
  out.shamt = static_cast<u8>((bits >> 6) & 0x1Fu);
  out.imm = static_cast<u16>(bits & 0xFFFFu);
  out.simm = sign_extend_16(out.imm);
  out.cycles = static_cast<u8>(cpu_.instruction_cycles(bits));

  const u32 primary = (bits >> 26) & 0x3Fu;
  switch (primary) {
  case 0x00: {
    const u32 funct = bits & 0x3Fu;
    switch (funct) {
    case 0x00:
      out.op = (bits == 0u) ? DecodedOp::Nop : DecodedOp::Sll;
      break;
    case 0x02: out.op = DecodedOp::Srl; break;
    case 0x03: out.op = DecodedOp::Sra; break;
    case 0x04: out.op = DecodedOp::Sllv; break;
    case 0x06: out.op = DecodedOp::Srlv; break;
    case 0x07: out.op = DecodedOp::Srav; break;
    case 0x08:
      out.op = DecodedOp::Jr;
      out.is_branch = true;
      out.is_unconditional_branch = true;
      break;
    case 0x09:
      out.op = DecodedOp::Jalr;
      out.is_branch = true;
      out.is_unconditional_branch = true;
      break;
    case 0x0A: out.op = DecodedOp::Movz; break;
    case 0x0B: out.op = DecodedOp::Movn; break;
    case 0x0C:
      out.op = DecodedOp::Syscall;
      out.may_raise_exception = true;
      break;
    case 0x0D:
      out.op = DecodedOp::Break;
      out.may_raise_exception = true;
      break;
    case 0x0F: out.op = DecodedOp::Sync; break;
    case 0x14:
    case 0x1C:
    case 0x28:
    case 0x29:
      out.op = DecodedOp::Nop;
      break;
    case 0x21: out.op = DecodedOp::Addu; break;
    case 0x2D:
      out.op = DecodedOp::Addu;
      break;
    case 0x23: out.op = DecodedOp::Subu; break;
    case 0x2F:
      out.op = DecodedOp::Subu;
      break;
    case 0x24: out.op = DecodedOp::And; break;
    case 0x25: out.op = DecodedOp::Or; break;
    case 0x26: out.op = DecodedOp::Xor; break;
    case 0x27: out.op = DecodedOp::Nor; break;
    case 0x2A: out.op = DecodedOp::Slt; break;
    case 0x2B: out.op = DecodedOp::Sltu; break;
    case 0x38: out.op = DecodedOp::Clear; break;
    default:
      out.must_fallback = true;
      break;
    }
    break;
  }
  case 0x01: {
    switch (out.rt) {
    case 0x00: out.op = DecodedOp::Bltz; break;
    case 0x01: out.op = DecodedOp::Bgez; break;
    case 0x10: out.op = DecodedOp::Bltzal; break;
    case 0x11: out.op = DecodedOp::Bgezal; break;
    default:
      out.must_fallback = true;
      break;
    }
    if (!out.must_fallback) {
      out.is_branch = true;
      out.target =
          static_cast<u32>(static_cast<s32>(pc + 4u) + (out.simm << 2));
    }
    break;
  }
  case 0x02:
    out.op = DecodedOp::J;
    out.is_branch = true;
    out.is_unconditional_branch = true;
    out.target = ((pc + 4u) & 0xF0000000u) | ((bits & 0x03FFFFFFu) << 2);
    break;
  case 0x03:
    out.op = DecodedOp::Jal;
    out.is_branch = true;
    out.is_unconditional_branch = true;
    out.target = ((pc + 4u) & 0xF0000000u) | ((bits & 0x03FFFFFFu) << 2);
    break;
  case 0x04: out.op = DecodedOp::Beq; break;
  case 0x05: out.op = DecodedOp::Bne; break;
  case 0x06: out.op = DecodedOp::Blez; break;
  case 0x07: out.op = DecodedOp::Bgtz; break;
  case 0x09: out.op = DecodedOp::Addiu; break;
  case 0x0A: out.op = DecodedOp::Slti; break;
  case 0x0B: out.op = DecodedOp::Sltiu; break;
  case 0x0C: out.op = DecodedOp::Andi; break;
  case 0x0D: out.op = DecodedOp::Ori; break;
  case 0x0E: out.op = DecodedOp::Xori; break;
  case 0x0F: out.op = DecodedOp::Lui; break;
  case 0x10:
    out.op = DecodedOp::Cop0;
    out.must_fallback = true;
    break;
  case 0x12:
    out.op = DecodedOp::Cop2;
    out.must_fallback = true;
    break;
  case 0x20: out.op = DecodedOp::Lb; break;
  case 0x21: out.op = DecodedOp::Lh; break;
  case 0x23: out.op = DecodedOp::Lw; break;
  case 0x24: out.op = DecodedOp::Lbu; break;
  case 0x25: out.op = DecodedOp::Lhu; break;
  case 0x28: out.op = DecodedOp::Sb; break;
  case 0x29: out.op = DecodedOp::Sh; break;
  case 0x2B: out.op = DecodedOp::Sw; break;
  default:
    out.must_fallback = true;
    break;
  }

  switch (out.op) {
  case DecodedOp::Beq:
  case DecodedOp::Bne:
  case DecodedOp::Blez:
  case DecodedOp::Bgtz:
    out.is_branch = true;
    out.target =
        static_cast<u32>(static_cast<s32>(pc + 4u) + (out.simm << 2));
    break;
  case DecodedOp::Lb:
  case DecodedOp::Lh:
  case DecodedOp::Lw:
  case DecodedOp::Lbu:
  case DecodedOp::Lhu:
  case DecodedOp::Sb:
  case DecodedOp::Sh:
  case DecodedOp::Sw:
    out.may_access_memory = true;
    out.may_raise_exception = true;
    break;
  default:
    break;
  }

  return out;
}

CpuBlockRunResult CpuOptimizedBackend::execute_block(
    DecodedBlock &block, u32 max_cycles, u32 max_instructions) {
  CpuBlockRunResult result{};
  if (block.invalidated) {
    result.exit_reason = CpuBlockExitReason::Invalidated;
    return result;
  }

  ++stats_.decoded_block_entries;
  for (u32 i = 0; i < block.instruction_count; ++i) {
    if (result.cycles >= max_cycles || result.instructions >= max_instructions) {
      result.exit_reason = CpuBlockExitReason::Budget;
      break;
    }
    if (block.invalidated) {
      result.exit_reason = CpuBlockExitReason::Invalidated;
      break;
    }

    const DecodedInstruction &inst = block.instructions[i];
    if (cpu_.pc_ != inst.pc) {
      result.exit_reason = CpuBlockExitReason::PcMismatch;
      break;
    }
    if (inst.must_fallback) {
      const u32 consumed = cpu_.step();
      result.cycles += consumed;
      ++result.instructions;
      ++stats_.interpreter_fallback_steps;
      ++stats_.fallback_instructions;
      result.exit_reason = CpuBlockExitReason::Fallback;
      break;
    }

    if (!prepare_instruction(inst, result)) {
      break;
    }
    const bool decoded_executed = execute_decoded_instruction(inst);
    if (!decoded_executed) {
      cpu_.execute(inst.bits);
      ++stats_.fallback_instructions;
    }
    finish_instruction(inst, result, decoded_executed);

    if (cpu_.exception_raised_) {
      result.exit_reason = CpuBlockExitReason::Exception;
      break;
    }
    if (!decoded_executed) {
      result.exit_reason = CpuBlockExitReason::Fallback;
      break;
    }
    if (inst.is_branch) {
      result.exit_reason = CpuBlockExitReason::Branch;
    }
  }

  if (result.exit_reason == CpuBlockExitReason::None) {
    result.exit_reason = CpuBlockExitReason::Budget;
  }
  return result;
}

bool CpuOptimizedBackend::prepare_instruction(const DecodedInstruction &inst,
                                              CpuBlockRunResult &result) {
  cpu_.executing_step_ = true;
  cpu_.current_pc_ = cpu_.pc_;
  g_diag_current_pc = cpu_.current_pc_;
  cpu_.exception_raised_ = false;
  cpu_.cycle_penalty_ = 0;
  cpu_.in_delay_slot_ = cpu_.pending_delay_slot_;
  cpu_.active_branch_pc_ = cpu_.pending_branch_pc_;
  cpu_.pending_delay_slot_ = false;
  cpu_.pending_branch_taken_ = false;
  cpu_.pending_branch_pc_ = 0;

  if (cpu_.sys_->irq_pending()) {
    cpu_.cop0_cause_ |= (1u << 10);
  } else {
    cpu_.cop0_cause_ &= ~(1u << 10);
  }

  if (!cpu_.in_delay_slot_ && cpu_.check_irq()) {
    cpu_.exception(Exception::Interrupt);
    constexpr u32 irq_cycles = 2;
    cpu_.cycles_ += irq_cycles;
    cpu_.executing_step_ = false;
    result.cycles += irq_cycles;
    ++result.instructions;
    result.exit_reason = CpuBlockExitReason::Interrupt;
    ++stats_.interrupt_exits;
    (void)inst;
    return false;
  }

  (void)cpu_.fetch32(cpu_.pc_);
  if (cpu_.exception_raised_) {
    constexpr u32 fault_cycles = 2;
    cpu_.cycles_ += fault_cycles;
    cpu_.executing_step_ = false;
    result.cycles += fault_cycles;
    ++result.instructions;
    result.exit_reason = CpuBlockExitReason::Exception;
    ++stats_.exceptions;
    return false;
  }

  cpu_.pc_ = cpu_.next_pc_;
  cpu_.next_pc_ += 4;
  return true;
}

void CpuOptimizedBackend::finish_instruction(const DecodedInstruction &inst,
                                             CpuBlockRunResult &result,
                                             bool count_decoded) {
  if (!cpu_.exception_raised_) {
    cpu_.advance_load_delay();
  } else {
    ++stats_.exceptions;
  }

  const u32 consumed_cycles =
      cpu_.instruction_cycles(inst.bits) + cpu_.cycle_penalty_;
  cpu_.cycles_ += consumed_cycles;
  cpu_.executing_step_ = false;

  result.cycles += consumed_cycles;
  ++result.instructions;
  if (count_decoded) {
    ++stats_.optimized_instructions;
    ++stats_.decoded_instructions;
  }
}

bool CpuOptimizedBackend::execute_decoded_instruction(
    const DecodedInstruction &inst) {
  auto reg = [&](u8 index) -> u32 { return cpu_.gpr_[index]; };
  auto set_reg = [&](u8 index, u32 value) { cpu_.set_reg(index, value); };
  auto mem_addr = [&]() -> u32 {
    return reg(inst.rs) + static_cast<u32>(inst.simm);
  };
  auto note_memory = [&](u32 addr) {
    ++stats_.memory_helper_calls;
    if (is_mmio_address(addr)) {
      ++stats_.mmio_accesses;
    }
  };

  switch (inst.op) {
  case DecodedOp::Nop:
    break;
  case DecodedOp::Sll:
    set_reg(inst.rd, reg(inst.rt) << inst.shamt);
    break;
  case DecodedOp::Srl:
    set_reg(inst.rd, reg(inst.rt) >> inst.shamt);
    break;
  case DecodedOp::Sra:
    set_reg(inst.rd,
            static_cast<u32>(static_cast<s32>(reg(inst.rt)) >> inst.shamt));
    break;
  case DecodedOp::Sllv:
    set_reg(inst.rd, reg(inst.rt) << (reg(inst.rs) & 0x1Fu));
    break;
  case DecodedOp::Srlv:
    set_reg(inst.rd, reg(inst.rt) >> (reg(inst.rs) & 0x1Fu));
    break;
  case DecodedOp::Srav:
    set_reg(inst.rd, static_cast<u32>(static_cast<s32>(reg(inst.rt)) >>
                                      (reg(inst.rs) & 0x1Fu)));
    break;
  case DecodedOp::Movz:
    if (reg(inst.rt) == 0u) {
      set_reg(inst.rd, reg(inst.rs));
    }
    break;
  case DecodedOp::Movn:
    if (reg(inst.rt) != 0u) {
      set_reg(inst.rd, reg(inst.rs));
    }
    break;
  case DecodedOp::Sync:
    break;
  case DecodedOp::Clear:
    set_reg(inst.rd, 0);
    break;
  case DecodedOp::Jr:
    cpu_.begin_branch(true, reg(inst.rs));
    break;
  case DecodedOp::Jalr:
    set_reg(inst.rd, cpu_.next_pc_);
    cpu_.begin_branch(true, reg(inst.rs));
    break;
  case DecodedOp::Syscall:
    cpu_.exception(Exception::Syscall);
    break;
  case DecodedOp::Break:
    cpu_.exception(Exception::Break);
    break;
  case DecodedOp::Addu:
    set_reg(inst.rd, reg(inst.rs) + reg(inst.rt));
    break;
  case DecodedOp::Subu:
    set_reg(inst.rd, reg(inst.rs) - reg(inst.rt));
    break;
  case DecodedOp::And:
    set_reg(inst.rd, reg(inst.rs) & reg(inst.rt));
    break;
  case DecodedOp::Or:
    set_reg(inst.rd, reg(inst.rs) | reg(inst.rt));
    break;
  case DecodedOp::Xor:
    set_reg(inst.rd, reg(inst.rs) ^ reg(inst.rt));
    break;
  case DecodedOp::Nor:
    set_reg(inst.rd, ~(reg(inst.rs) | reg(inst.rt)));
    break;
  case DecodedOp::Slt:
    set_reg(inst.rd, static_cast<s32>(reg(inst.rs)) <
                         static_cast<s32>(reg(inst.rt))
                         ? 1u
                         : 0u);
    break;
  case DecodedOp::Sltu:
    set_reg(inst.rd, reg(inst.rs) < reg(inst.rt) ? 1u : 0u);
    break;
  case DecodedOp::Bltz:
    cpu_.begin_branch(static_cast<s32>(reg(inst.rs)) < 0, inst.target);
    break;
  case DecodedOp::Bgez:
    cpu_.begin_branch(static_cast<s32>(reg(inst.rs)) >= 0, inst.target);
    break;
  case DecodedOp::Bltzal:
    set_reg(31, cpu_.next_pc_);
    cpu_.begin_branch(static_cast<s32>(reg(inst.rs)) < 0, inst.target);
    break;
  case DecodedOp::Bgezal:
    set_reg(31, cpu_.next_pc_);
    cpu_.begin_branch(static_cast<s32>(reg(inst.rs)) >= 0, inst.target);
    break;
  case DecodedOp::J:
    cpu_.begin_branch(true, inst.target);
    break;
  case DecodedOp::Jal:
    set_reg(31, cpu_.next_pc_);
    cpu_.begin_branch(true, inst.target);
    break;
  case DecodedOp::Beq:
    cpu_.begin_branch(reg(inst.rs) == reg(inst.rt), inst.target);
    break;
  case DecodedOp::Bne:
    cpu_.begin_branch(reg(inst.rs) != reg(inst.rt), inst.target);
    break;
  case DecodedOp::Blez:
    cpu_.begin_branch(static_cast<s32>(reg(inst.rs)) <= 0, inst.target);
    break;
  case DecodedOp::Bgtz:
    cpu_.begin_branch(static_cast<s32>(reg(inst.rs)) > 0, inst.target);
    break;
  case DecodedOp::Addiu:
    set_reg(inst.rt, reg(inst.rs) + static_cast<u32>(inst.simm));
    break;
  case DecodedOp::Slti:
    set_reg(inst.rt, static_cast<s32>(reg(inst.rs)) < inst.simm ? 1u : 0u);
    break;
  case DecodedOp::Sltiu:
    set_reg(inst.rt, reg(inst.rs) < static_cast<u32>(inst.simm) ? 1u : 0u);
    break;
  case DecodedOp::Andi:
    set_reg(inst.rt, reg(inst.rs) & inst.imm);
    break;
  case DecodedOp::Ori:
    set_reg(inst.rt, reg(inst.rs) | inst.imm);
    break;
  case DecodedOp::Xori:
    set_reg(inst.rt, reg(inst.rs) ^ inst.imm);
    break;
  case DecodedOp::Lui:
    set_reg(inst.rt, static_cast<u32>(inst.imm) << 16);
    break;
  case DecodedOp::Lb: {
    const u32 addr = mem_addr();
    note_memory(addr);
    const u8 value = cpu_.load8(addr);
    cpu_.schedule_load(inst.rt, static_cast<u32>(sign_extend_8(value)));
    break;
  }
  case DecodedOp::Lh: {
    const u32 addr = mem_addr();
    note_memory(addr);
    const u16 value = cpu_.load16(addr);
    if (!cpu_.exception_raised_) {
      cpu_.schedule_load(inst.rt, static_cast<u32>(sign_extend_16(value)));
    }
    break;
  }
  case DecodedOp::Lw: {
    const u32 addr = mem_addr();
    note_memory(addr);
    const u32 value = cpu_.load32(addr);
    if (!cpu_.exception_raised_) {
      cpu_.schedule_load(inst.rt, value);
    }
    break;
  }
  case DecodedOp::Lbu: {
    const u32 addr = mem_addr();
    note_memory(addr);
    cpu_.schedule_load(inst.rt, cpu_.load8(addr));
    break;
  }
  case DecodedOp::Lhu: {
    const u32 addr = mem_addr();
    note_memory(addr);
    const u16 value = cpu_.load16(addr);
    if (!cpu_.exception_raised_) {
      cpu_.schedule_load(inst.rt, value);
    }
    break;
  }
  case DecodedOp::Sb: {
    const u32 addr = mem_addr();
    note_memory(addr);
    cpu_.store8(addr, static_cast<u8>(reg(inst.rt)));
    break;
  }
  case DecodedOp::Sh: {
    const u32 addr = mem_addr();
    note_memory(addr);
    cpu_.store16(addr, static_cast<u16>(reg(inst.rt)));
    break;
  }
  case DecodedOp::Sw: {
    const u32 addr = mem_addr();
    note_memory(addr);
    cpu_.store32(addr, reg(inst.rt));
    break;
  }
  default:
    return false;
  }

  cpu_.gpr_[0] = 0;
  return true;
}

CpuForcedInterpreterReason
CpuOptimizedBackend::diagnostics_force_interpreter_reason() const {
  if (g_trace_cpu) {
    return CpuForcedInterpreterReason::TraceCpu;
  }
  if (g_cpu_deep_diagnostics) {
    return CpuForcedInterpreterReason::DeepDiagnostics;
  }
  if (g_log_fmv_diagnostics) {
    return CpuForcedInterpreterReason::FmvDiagnostics;
  }
  if (g_cpu_backend_compare_test_force_interpreter) {
    return CpuForcedInterpreterReason::BackendCompareTest;
  }
  return CpuForcedInterpreterReason::None;
}

bool CpuOptimizedBackend::is_block_terminator(
    const DecodedInstruction &inst) const {
  return inst.is_branch || inst.op == DecodedOp::Syscall ||
         inst.op == DecodedOp::Break;
}

bool CpuOptimizedBackend::is_mmio_address(u32 addr) const {
  const u32 phys = psx::mask_address(addr);
  return phys >= 0x1F801000u && phys < 0x1F803000u;
}

NativeMemoryRegion classify_native_memory_region(u32 addr) {
  const u32 phys = psx::mask_address(addr);
  if (phys < psx::RAM_SIZE) {
    return NativeMemoryRegion::Ram;
  }
  if (phys >= psx::SCRATCHPAD_BASE && phys < psx::IO_BASE) {
    return NativeMemoryRegion::Scratchpad;
  }
  if (phys >= psx::BIOS_BASE &&
      phys < psx::BIOS_BASE + psx::BIOS_SIZE) {
    return NativeMemoryRegion::BiosReadOnly;
  }
  if (phys >= psx::IO_BASE && phys < 0x1F803000u) {
    return NativeMemoryRegion::Mmio;
  }
  return NativeMemoryRegion::UnknownSlow;
}

const char *native_memory_region_name(NativeMemoryRegion region) {
  switch (region) {
  case NativeMemoryRegion::Ram: return "RAM";
  case NativeMemoryRegion::Scratchpad: return "SCRATCHPAD";
  case NativeMemoryRegion::BiosReadOnly: return "BIOS_READ_ONLY";
  case NativeMemoryRegion::Mmio: return "MMIO";
  case NativeMemoryRegion::UnknownSlow:
  default: return "UNKNOWN_SLOW";
  }
}

u32 CpuOptimizedBackend::normalized_code_addr(u32 addr) const {
  const u32 phys = psx::mask_address(addr);
  if (phys < 0x00800000u) {
    return phys & (psx::RAM_SIZE - 1u);
  }
  if (phys >= 0x1F800000u && phys < 0x1F801000u) {
    return phys;
  }
  if (phys >= psx::BIOS_BASE) {
    return phys;
  }
  return phys;
}

u32 CpuOptimizedBackend::code_page_for_normalized_addr(u32 addr) const {
  return addr >> kCodePageShift;
}

bool CpuOptimizedBackend::code_page_maybe_compiled(u32 page) const {
  const u32 word = page >> 6;
  const u32 bit = page & 63u;
  return (compiled_code_page_bitmap_[word] & (1ull << bit)) != 0;
}

void CpuOptimizedBackend::set_code_page_compiled(u32 page) {
  const u32 word = page >> 6;
  const u32 bit = page & 63u;
  compiled_code_page_bitmap_[word] |= (1ull << bit);
}

void CpuOptimizedBackend::clear_code_page_compiled(u32 page) {
  const u32 word = page >> 6;
  const u32 bit = page & 63u;
  compiled_code_page_bitmap_[word] &= ~(1ull << bit);
}

bool CpuOptimizedBackend::register_block_page(DecodedBlock &block, u32 page) {
  for (u32 i = 0; i < block.registered_page_count; ++i) {
    if (block.registered_pages[i] == page) {
      return true;
    }
  }
  if (block.registered_page_count >= block.registered_pages.size()) {
    return false;
  }

  block.registered_pages[block.registered_page_count++] = page;
  blocks_by_page_[page].push_back(&block);
  set_code_page_compiled(page);
  return true;
}

void CpuOptimizedBackend::register_block_pages(DecodedBlock &block) {
  for (u32 i = 0; i < block.tracked_range_count; ++i) {
    const auto &range = block.tracked_ranges[i];
    if (range.second <= range.first) {
      continue;
    }

    const u32 first_page = code_page_for_normalized_addr(range.first);
    const u32 last_page = code_page_for_normalized_addr(range.second - 1u);
    for (u32 page = first_page; page <= last_page; ++page) {
      if (!register_block_page(block, page)) {
        block.invalidated = true;
        return;
      }
    }
  }
}

void CpuOptimizedBackend::unregister_block_pages(DecodedBlock &block) {
  for (u32 i = 0; i < block.registered_page_count; ++i) {
    const u32 page = block.registered_pages[i];
    auto found = blocks_by_page_.find(page);
    if (found == blocks_by_page_.end()) {
      clear_code_page_compiled(page);
      continue;
    }

    auto &page_blocks = found->second;
    page_blocks.erase(std::remove(page_blocks.begin(), page_blocks.end(),
                                  &block),
                      page_blocks.end());
    if (page_blocks.empty()) {
      blocks_by_page_.erase(found);
      clear_code_page_compiled(page);
    }
  }
  block.registered_page_count = 0;
}

bool CpuOptimizedBackend::ranges_overlap(u32 a0, u32 a1, u32 b0,
                                         u32 b1) const {
  return a0 < b1 && b0 < a1;
}

void CpuOptimizedBackend::mark_block_invalidated(DecodedBlock &block) {
  block.invalidated = true;
  ++stats_.invalidations;

  BlockHistory &history = block_history_[block.start_key];
  if (current_frame_ - history.window_start_frame >
      kInterpreterFallbackFrames) {
    history.window_start_frame = current_frame_;
    history.compile_count_in_window = 0;
    history.invalidation_count_in_window = 0;
  }
  ++history.invalidation_count_in_window;

  if (current_frame_ - block.last_invalidate_frame <=
      kInterpreterFallbackFrames) {
    ++block.invalidations_in_window;
  } else {
    block.invalidations_in_window = 1;
  }
  block.last_invalidate_frame = current_frame_;
  ++block.recompiles_in_window;

  if (block.recompiles_in_window >= kInterpreterFallbackRecompiles ||
      block.invalidations_in_window >= kInterpreterFallbackRecompiles ||
      history.compile_count_in_window > kInterpreterFallbackRecompiles ||
      history.invalidation_count_in_window >= kInterpreterFallbackRecompiles) {
    history.interpreter_only_until_frame =
        current_frame_ + kInterpreterOnlyFrames;
    block.interpreter_only_until_frame = history.interpreter_only_until_frame;
  }
}

void CpuOptimizedBackend::record_forced_interpreter(
    CpuForcedInterpreterReason reason, u32 instructions) {
  stats_.forced_interpreter_instructions += instructions;
  switch (reason) {
  case CpuForcedInterpreterReason::TraceCpu:
    stats_.forced_interpreter_trace += instructions;
    break;
  case CpuForcedInterpreterReason::DeepDiagnostics:
    stats_.forced_interpreter_deep_diagnostics += instructions;
    break;
  case CpuForcedInterpreterReason::FmvDiagnostics:
    stats_.forced_interpreter_fmv += instructions;
    break;
  case CpuForcedInterpreterReason::BackendCompareTest:
    stats_.forced_interpreter_backend_compare += instructions;
    break;
  case CpuForcedInterpreterReason::Other:
    stats_.forced_interpreter_other += instructions;
    break;
  case CpuForcedInterpreterReason::None:
  default:
    break;
  }
}

void CpuOptimizedBackend::record_exit(CpuBlockExitReason reason) {
  switch (reason) {
  case CpuBlockExitReason::Budget:
    ++stats_.budget_exits;
    break;
  case CpuBlockExitReason::Fallback:
    ++stats_.fallback_exits;
    break;
  case CpuBlockExitReason::Invalidated:
    ++stats_.invalidated_exits;
    break;
  case CpuBlockExitReason::PcMismatch:
    ++stats_.pc_mismatch_exits;
    break;
  case CpuBlockExitReason::Exception:
  case CpuBlockExitReason::Interrupt:
  case CpuBlockExitReason::Branch:
  case CpuBlockExitReason::None:
  default:
    break;
  }
}

void CpuOptimizedBackend::record_native_reject(
    NativeBlockRejectReason reason) {
  switch (reason) {
  case NativeBlockRejectReason::Branch:
    ++stats_.native_reject_branch;
    break;
  case NativeBlockRejectReason::Memory:
    ++stats_.native_reject_memory;
    break;
  case NativeBlockRejectReason::Cop0:
    ++stats_.native_reject_cop0;
    break;
  case NativeBlockRejectReason::Cop2:
    ++stats_.native_reject_cop2;
    break;
  case NativeBlockRejectReason::ExceptionOrUnknown:
    ++stats_.native_reject_exception_unknown;
    break;
  case NativeBlockRejectReason::Mmio:
    ++stats_.native_reject_mmio;
    break;
  case NativeBlockRejectReason::Unaligned:
    ++stats_.native_reject_unaligned;
    break;
  case NativeBlockRejectReason::None:
  default:
    break;
  }
}

NativeBlockRejectDetail CpuOptimizedBackend::native_reject_detail_for_reason(
    NativeBlockRejectReason reason) const {
  switch (reason) {
  case NativeBlockRejectReason::Branch:
    return NativeBlockRejectDetail::Branch;
  case NativeBlockRejectReason::Memory:
    return NativeBlockRejectDetail::Memory;
  case NativeBlockRejectReason::Cop0:
    return NativeBlockRejectDetail::Cop0;
  case NativeBlockRejectReason::Cop2:
    return NativeBlockRejectDetail::Cop2;
  case NativeBlockRejectReason::ExceptionOrUnknown:
    return NativeBlockRejectDetail::ExceptionOrUnknown;
  case NativeBlockRejectReason::Mmio:
    return NativeBlockRejectDetail::Mmio;
  case NativeBlockRejectReason::Unaligned:
    return NativeBlockRejectDetail::Unaligned;
  case NativeBlockRejectReason::None:
  default:
    return NativeBlockRejectDetail::None;
  }
}

NativeBlockRejectDetail CpuOptimizedBackend::classify_native_pc_state_reject(
    const DecodedBlock &block) const {
  if (block.invalidated || block.native_fn == nullptr ||
      block.native_context == nullptr) {
    return NativeBlockRejectDetail::StaleInvalidBlockState;
  }

  const bool branch_delay_shape =
      cpu_.in_delay_slot_ || cpu_.pending_delay_slot_ ||
      cpu_.pending_branch_taken_ || cpu_.pending_branch_pc_ != 0u;
  if (branch_delay_shape) {
    return NativeBlockRejectDetail::BlockStartAfterBranchDelay;
  }

  if (cpu_.pc_ != block.start_pc) {
    return NativeBlockRejectDetail::PcMismatch;
  }
  if (cpu_.next_pc_ != block.start_pc + 4u) {
    return NativeBlockRejectDetail::NextPcMismatch;
  }
  return NativeBlockRejectDetail::PcMismatch;
}

NativeBlockRejectDetail
CpuOptimizedBackend::record_native_branch_delay_subreasons() {
  NativeBlockRejectDetail first = NativeBlockRejectDetail::None;
  auto note = [&](bool active, u64 &counter,
                  NativeBlockRejectDetail detail) {
    if (!active) {
      return;
    }
    ++counter;
    if (first == NativeBlockRejectDetail::None) {
      first = detail;
    }
  };

  note(cpu_.in_delay_slot_, stats_.native_reject_branch_delay_in_delay_slot,
       NativeBlockRejectDetail::InDelaySlot);
  note(cpu_.pending_delay_slot_,
       stats_.native_reject_branch_delay_pending_delay_slot,
       NativeBlockRejectDetail::PendingDelaySlot);
  note(cpu_.pending_branch_taken_,
       stats_.native_reject_branch_delay_pending_branch_taken,
       NativeBlockRejectDetail::PendingBranchTaken);
  note(cpu_.pending_branch_pc_ != 0u,
       stats_.native_reject_branch_delay_pending_branch_pc,
       NativeBlockRejectDetail::PendingBranchPc);

  return first == NativeBlockRejectDetail::None
             ? NativeBlockRejectDetail::PendingDelaySlot
             : first;
}

void CpuOptimizedBackend::record_native_pc_state_subreason(
    NativeBlockRejectDetail detail) {
  switch (detail) {
  case NativeBlockRejectDetail::PcMismatch:
    ++stats_.native_reject_pc_mismatch;
    break;
  case NativeBlockRejectDetail::NextPcMismatch:
    ++stats_.native_reject_next_pc_mismatch;
    break;
  case NativeBlockRejectDetail::BlockStartAfterBranchDelay:
    ++stats_.native_reject_block_start_after_branch_delay;
    break;
  case NativeBlockRejectDetail::StaleInvalidBlockState:
    ++stats_.native_reject_stale_invalid_block_state;
    break;
  default:
    ++stats_.native_reject_pc_mismatch;
    break;
  }
}

void CpuOptimizedBackend::record_native_block_rejection(
    const DecodedBlock &block, NativeBlockRejectDetail detail) {
  if (!g_cpu_backend_rejected_block_logging) {
    return;
  }

  NativeRejectedBlockProfile &profile =
      rejected_block_profiles_[block.start_key];
  if (profile.rejections == 0) {
    profile.start_pc = block.start_pc;
    profile.instruction_count = block.instruction_count;
    profile.contains_fallback = block.has_fallback;
    for (u32 i = 0; i < block.instruction_count && i < profile.ops.size();
         ++i) {
      const DecodedInstruction &inst = block.instructions[i];
      profile.ops[i] = inst.op;
      profile.contains_branch = profile.contains_branch || inst.is_branch;
      profile.contains_memory =
          profile.contains_memory || inst.may_access_memory;
      profile.contains_cop = profile.contains_cop ||
                             inst.op == DecodedOp::Cop0 ||
                             inst.op == DecodedOp::Cop2;
      profile.contains_syscall_break =
          profile.contains_syscall_break || inst.op == DecodedOp::Syscall ||
          inst.op == DecodedOp::Break;
      profile.contains_fallback =
          profile.contains_fallback || inst.must_fallback;
    }
  }

  ++profile.rejections;
  profile.rejected_instructions += block.instruction_count;
  profile.last_reason = detail;
}

void CpuOptimizedBackend::log_rejected_block_profiles() const {
  if (!g_cpu_backend_rejected_block_logging ||
      rejected_block_profiles_.empty()) {
    return;
  }

  std::vector<const NativeRejectedBlockProfile *> profiles;
  profiles.reserve(rejected_block_profiles_.size());
  for (const auto &entry : rejected_block_profiles_) {
    profiles.push_back(&entry.second);
  }
  std::sort(profiles.begin(), profiles.end(),
            [](const NativeRejectedBlockProfile *a,
               const NativeRejectedBlockProfile *b) {
              if (a->rejections != b->rejections) {
                return a->rejections > b->rejections;
              }
              return a->rejected_instructions > b->rejected_instructions;
            });

  const u32 limit = std::min<u32>(
      std::max<u32>(1u, g_cpu_backend_rejected_block_log_count),
      static_cast<u32>(profiles.size()));
  LOG_INFO("CPU_BACKEND_REJECTED_BLOCKS top=%u tracked=%zu", limit,
           profiles.size());
  for (u32 i = 0; i < limit; ++i) {
    const NativeRejectedBlockProfile &profile = *profiles[i];
    const std::string ops =
        decoded_ops_string(profile.ops, profile.instruction_count);
    LOG_INFO("CPU_BACKEND_REJECTED_BLOCK rank=%u pc=0x%08X count=%llu instructions=%u rejected_instr=%llu reason=%s flags=branch:%u memory:%u cop:%u syscall_break:%u fallback:%u ops=%s",
             i + 1u, profile.start_pc,
             static_cast<unsigned long long>(profile.rejections),
             profile.instruction_count,
             static_cast<unsigned long long>(profile.rejected_instructions),
             native_reject_detail_name(profile.last_reason),
             profile.contains_branch ? 1u : 0u,
             profile.contains_memory ? 1u : 0u,
             profile.contains_cop ? 1u : 0u,
             profile.contains_syscall_break ? 1u : 0u,
             profile.contains_fallback ? 1u : 0u, ops.c_str());
  }
}

bool CpuOptimizedBackend::native_branch_tail_pc_blacklisted(
    const DecodedBlock &block) const {
  const auto contains = [](u32 pc) {
    return std::find(g_cpu_x64_jit_branch_tail_blacklist.begin(),
                     g_cpu_x64_jit_branch_tail_blacklist.end(), pc) !=
           g_cpu_x64_jit_branch_tail_blacklist.end();
  };
  if (contains(block.start_pc)) {
    return true;
  }
  // A single branch instruction can be reached through several decoded
  // suffix blocks when a slice ends part-way through the surrounding loop.
  // Matching the branch PC makes one blacklist entry suppress every such
  // native shape while preserving the historical start-PC behavior.
  return block.native_branch_tail && block.instruction_count >= 2u &&
         contains(block.instructions[block.instruction_count - 2u].pc);
}

void CpuOptimizedBackend::record_native_branch_tail_trace(
    const DecodedBlock &block, bool taken, bool delay_memory, bool delay_mmio,
    bool exception) {
  if (!g_cpu_x64_jit_branch_tail_logging || block.instruction_count < 2u) {
    return;
  }

  const DecodedInstruction &branch =
      block.instructions[block.instruction_count - 2u];
  const DecodedInstruction &delay =
      block.instructions[block.instruction_count - 1u];
  NativeBranchTailTrace trace{};
  trace.sequence = ++native_branch_tail_trace_sequence_;
  trace.start_pc = block.start_pc;
  trace.branch_pc = branch.pc;
  trace.target = branch.target;
  trace.final_pc = cpu_.pc_;
  trace.final_next_pc = cpu_.next_pc_;
  trace.delay_op = delay.op;
  trace.taken = taken;
  trace.delay_memory = delay_memory;
  trace.delay_mmio = delay_mmio;
  trace.exception = exception;
  recent_native_branch_tails_.push_back(trace);

  const size_t limit =
      std::max<size_t>(1u, g_cpu_x64_jit_branch_tail_log_count);
  while (recent_native_branch_tails_.size() > limit) {
    recent_native_branch_tails_.pop_front();
  }
}

void CpuOptimizedBackend::log_native_branch_tail_diagnostics() const {
  if (!g_cpu_x64_jit_branch_tail_logging) {
    return;
  }

  std::vector<const DecodedBlock *> entries;
  for (const auto &entry : blocks_) {
    const DecodedBlock &block = *entry.second;
    if (block.native_branch_tail_entry_count != 0) {
      entries.push_back(&block);
    }
  }
  std::sort(entries.begin(), entries.end(),
            [](const DecodedBlock *a, const DecodedBlock *b) {
              return a->native_branch_tail_entry_count >
                     b->native_branch_tail_entry_count;
            });
  const size_t pc_limit = std::min<size_t>(entries.size(), 16u);
  LOG_INFO("CPU_BRANCH_TAIL_PC_COUNTS tracked=%zu top=%zu", entries.size(),
           pc_limit);
  for (size_t i = 0; i < pc_limit; ++i) {
    const DecodedBlock &block = *entries[i];
    const DecodedInstruction &branch =
        block.instructions[block.instruction_count - 2u];
    std::array<DecodedOp, DecodedBlock::kMaxInstructions> ops{};
    for (u32 op_index = 0; op_index < block.instruction_count; ++op_index) {
      ops[op_index] = block.instructions[op_index].op;
    }
    const std::string op_names =
        decoded_ops_string(ops, block.instruction_count);
    const DecodedInstruction &delay =
        block.instructions[block.instruction_count - 1u];
    LOG_INFO("CPU_BRANCH_TAIL_PC rank=%zu start_pc=0x%08X branch_pc=0x%08X target=0x%08X entries=%llu instructions=%u branch_op=%s delay_op=%s ops=%s",
             i + 1u, block.start_pc, branch.pc, branch.target,
             static_cast<unsigned long long>(
                 block.native_branch_tail_entry_count),
             block.instruction_count, decoded_op_name(branch.op),
             decoded_op_name(delay.op), op_names.c_str());
  }

  struct BranchPcTotal {
    u32 pc = 0;
    u64 entries = 0;
    const DecodedBlock *representative = nullptr;
  };
  std::vector<BranchPcTotal> branch_totals;
  for (const DecodedBlock *block : entries) {
    const u32 branch_pc =
        block->instructions[block->instruction_count - 2u].pc;
    auto total = std::find_if(
        branch_totals.begin(), branch_totals.end(),
        [branch_pc](const BranchPcTotal &value) { return value.pc == branch_pc; });
    if (total == branch_totals.end()) {
      branch_totals.push_back(
          {branch_pc, block->native_branch_tail_entry_count, block});
    } else {
      total->entries += block->native_branch_tail_entry_count;
      if (block->instruction_count > total->representative->instruction_count) {
        total->representative = block;
      }
    }
  }
  std::sort(branch_totals.begin(), branch_totals.end(),
            [](const BranchPcTotal &a, const BranchPcTotal &b) {
              return a.entries > b.entries;
            });
  const size_t branch_pc_limit =
      std::min<size_t>(branch_totals.size(), 16u);
  LOG_INFO("CPU_BRANCH_TAIL_BRANCH_PC_COUNTS tracked=%zu top=%zu",
           branch_totals.size(), branch_pc_limit);
  for (size_t i = 0; i < branch_pc_limit; ++i) {
    const BranchPcTotal &total = branch_totals[i];
    const DecodedBlock &block = *total.representative;
    const DecodedInstruction &branch =
        block.instructions[block.instruction_count - 2u];
    const DecodedInstruction &delay =
        block.instructions[block.instruction_count - 1u];
    std::array<DecodedOp, DecodedBlock::kMaxInstructions> ops{};
    for (u32 op_index = 0; op_index < block.instruction_count; ++op_index) {
      ops[op_index] = block.instructions[op_index].op;
    }
    const std::string op_names =
        decoded_ops_string(ops, block.instruction_count);
    LOG_INFO("CPU_BRANCH_TAIL_BRANCH_PC rank=%zu branch_pc=0x%08X target=0x%08X entries=%llu representative_start=0x%08X instructions=%u branch_op=%s delay_op=%s ops=%s",
             i + 1u, branch.pc, branch.target,
             static_cast<unsigned long long>(total.entries), block.start_pc,
             block.instruction_count, decoded_op_name(branch.op),
             decoded_op_name(delay.op), op_names.c_str());
  }

  LOG_INFO("CPU_BRANCH_TAIL_RECENT count=%zu limit=%u",
           recent_native_branch_tails_.size(),
           g_cpu_x64_jit_branch_tail_log_count);
  for (const NativeBranchTailTrace &trace : recent_native_branch_tails_) {
    LOG_INFO("CPU_BRANCH_TAIL_TRACE seq=%llu start_pc=0x%08X branch_pc=0x%08X target=0x%08X taken=%u final_pc=0x%08X final_next_pc=0x%08X delay_op=%s delay_memory=%u delay_mmio=%u exception=%u",
             static_cast<unsigned long long>(trace.sequence), trace.start_pc,
             trace.branch_pc, trace.target, trace.taken ? 1u : 0u,
             trace.final_pc, trace.final_next_pc,
             decoded_op_name(trace.delay_op), trace.delay_memory ? 1u : 0u,
             trace.delay_mmio ? 1u : 0u, trace.exception ? 1u : 0u);
  }
}

void CpuOptimizedBackend::log_native_helper_pc_diagnostics() const {
  struct HelperPcEntry {
    const DecodedBlock *block = nullptr;
    u64 window_calls = 0;
    u64 total_calls = 0;
  };
  std::vector<HelperPcEntry> entries;
  for (const auto &entry : blocks_) {
    DecodedBlock &block = *entry.second;
    const u64 total = block.native_prepare_helper_call_count +
                      block.native_finish_helper_call_count +
                      block.native_memory_helper_call_count +
                      block.native_branch_helper_call_count;
    const u64 window =
        total >= block.native_helper_calls_logged
            ? total - block.native_helper_calls_logged
            : total;
    block.native_helper_calls_logged = total;
    if (window != 0u) {
      entries.push_back({&block, window, total});
    }
  }
  std::sort(entries.begin(), entries.end(),
            [](const HelperPcEntry &a, const HelperPcEntry &b) {
              return a.window_calls > b.window_calls;
            });
  const size_t limit = std::min<size_t>(entries.size(), 16u);
  LOG_INFO("CPU_NATIVE_HELPER_PC_COUNTS tracked=%zu top=%zu", entries.size(),
           limit);
  for (size_t i = 0; i < limit; ++i) {
    const DecodedBlock &block = *entries[i].block;
    std::array<DecodedOp, DecodedBlock::kMaxInstructions> ops{};
    for (u32 op_index = 0; op_index < block.instruction_count; ++op_index) {
      ops[op_index] = block.instructions[op_index].op;
    }
    const std::string op_names =
        decoded_ops_string(ops, block.instruction_count);
    LOG_INFO("CPU_NATIVE_HELPER_PC rank=%zu pc=0x%08X window_calls=%llu total_calls=%llu prepare=%llu finish=%llu memory=%llu branch=%llu instructions=%u ops=%s",
             i + 1u, block.start_pc,
             static_cast<unsigned long long>(entries[i].window_calls),
             static_cast<unsigned long long>(entries[i].total_calls),
             static_cast<unsigned long long>(
                 block.native_prepare_helper_call_count),
             static_cast<unsigned long long>(
                 block.native_finish_helper_call_count),
             static_cast<unsigned long long>(
                 block.native_memory_helper_call_count),
             static_cast<unsigned long long>(
                 block.native_branch_helper_call_count),
             block.instruction_count, op_names.c_str());
  }
}

void CpuOptimizedBackend::log_native_entry_pc_diagnostics() const {
  std::vector<const DecodedBlock *> entries;
  std::vector<const DecodedBlock *> reduced_candidates;
  std::vector<const DecodedBlock *> reduced_rejections;
  for (const auto &entry : blocks_) {
    const DecodedBlock &block = *entry.second;
    if (block.native_entry_count != 0u) {
      entries.push_back(&block);
    }
    if (block.native_reduced_helper) {
      reduced_candidates.push_back(&block);
    } else if (block.native_safety_checked) {
      reduced_rejections.push_back(&block);
    }
  }
  const auto hottest_first = [](const DecodedBlock *a,
                                const DecodedBlock *b) {
    return a->native_entry_count > b->native_entry_count;
  };
  std::sort(entries.begin(), entries.end(), hottest_first);
  std::sort(reduced_candidates.begin(), reduced_candidates.end(),
            hottest_first);
  std::sort(reduced_rejections.begin(), reduced_rejections.end(),
            hottest_first);

  auto log_entries = [](const char *summary, const char *item,
                        const std::vector<const DecodedBlock *> &blocks) {
    const size_t limit = std::min<size_t>(blocks.size(), 16u);
    LOG_INFO("%s tracked=%zu top=%zu", summary, blocks.size(), limit);
    for (size_t i = 0; i < limit; ++i) {
      const DecodedBlock &block = *blocks[i];
      std::array<DecodedOp, DecodedBlock::kMaxInstructions> ops{};
      for (u32 op_index = 0; op_index < block.instruction_count; ++op_index) {
        ops[op_index] = block.instructions[op_index].op;
      }
      const std::string op_names =
          decoded_ops_string(ops, block.instruction_count);
      LOG_INFO("%s rank=%zu pc=0x%08X entries=%llu instructions=%u shape=%s reduced_helper=%u ram_load=%u reject=%s reduced_reject=%s ops=%s",
               item, i + 1u, block.start_pc,
               static_cast<unsigned long long>(block.native_entry_count),
               block.instruction_count,
               native_block_shape_name(block.native_shape),
               block.native_reduced_helper ? 1u : 0u,
               block.native_reduced_helper_ram_load ? 1u : 0u,
               native_reject_detail_name(block.native_reject_detail),
               native_reject_detail_name(
                   block.native_reduced_helper_reject_detail),
               op_names.c_str());
    }
  };

  log_entries("CPU_NATIVE_ENTRY_PC_COUNTS", "CPU_NATIVE_ENTRY_PC", entries);
  log_entries("CPU_REDUCED_HELPER_ELIGIBLE_PC_COUNTS",
              "CPU_REDUCED_HELPER_ELIGIBLE_PC", reduced_candidates);

  const size_t reject_limit =
      std::min<size_t>(reduced_rejections.size(), 16u);
  LOG_INFO("CPU_REDUCED_HELPER_REJECT_PC_COUNTS tracked=%zu top=%zu",
           reduced_rejections.size(), reject_limit);
  for (size_t i = 0; i < reject_limit; ++i) {
    const DecodedBlock &block = *reduced_rejections[i];
    std::array<DecodedOp, DecodedBlock::kMaxInstructions> ops{};
    for (u32 op_index = 0; op_index < block.instruction_count; ++op_index) {
      ops[op_index] = block.instructions[op_index].op;
    }
    const std::string op_names =
        decoded_ops_string(ops, block.instruction_count);
    LOG_INFO("CPU_REDUCED_HELPER_REJECT_PC rank=%zu pc=0x%08X native_entries=%llu instructions=%u reason=%s reduced_reason=%s shape=%s ops=%s",
             i + 1u, block.start_pc,
             static_cast<unsigned long long>(block.native_entry_count),
             block.instruction_count,
             native_reject_detail_name(block.native_reject_detail),
             native_reject_detail_name(
                 block.native_reduced_helper_reject_detail),
             native_block_shape_name(block.native_shape), op_names.c_str());
  }
}

void CpuOptimizedBackend::warn_forced_interpreter_once(
    CpuForcedInterpreterReason reason) {
  if (reason == CpuForcedInterpreterReason::None ||
      last_forced_interpreter_warning_ == reason) {
    return;
  }
  last_forced_interpreter_warning_ = reason;
  LOG_WARN("CPU backend forced to interpreter: %s",
           cpu_forced_interpreter_reason_name(reason));
}

void CpuOptimizedBackend::log_periodic_stats() {
  if (!g_cpu_backend_stats_logging ||
      effective_cpu_execution_mode() == CpuExecutionMode::Interpreter) {
    have_stats_log_snapshot_ = false;
    last_stats_log_frame_ = current_frame_;
    return;
  }

  const CpuBackendStats current = stats();
  if (!have_stats_log_snapshot_) {
    last_stats_log_ = current;
    last_stats_log_frame_ = current_frame_;
    have_stats_log_snapshot_ = true;
    return;
  }

  const u32 cadence = std::max<u32>(1u, g_cpu_backend_stats_log_frames);
  const u32 frame_delta = current_frame_ - last_stats_log_frame_;
  if (frame_delta < cadence) {
    return;
  }

  const CpuBackendStats delta = delta_stats(current, last_stats_log_);
  const u64 instruction_delta = delta.decoded_instructions +
                                delta.native_instructions +
                                delta.fallback_instructions;
  if (instruction_delta != 0 || delta.native_compile_attempts != 0 ||
      delta.native_to_decoded_fallbacks != 0 ||
      delta.forced_interpreter_slices != 0) {
    log_stats_section(current, delta, frame_delta);
  }

  last_stats_log_ = current;
  last_stats_log_frame_ = current_frame_;
}

void CpuOptimizedBackend::log_stats_section(
    const CpuBackendStats &current, const CpuBackendStats &delta,
    u32 frame_delta) const {
  const double decoded_avg =
      delta.decoded_block_entries == 0
          ? 0.0
          : static_cast<double>(delta.decoded_instructions) /
                static_cast<double>(delta.decoded_block_entries);
  const double native_avg =
      delta.native_block_entries == 0
          ? 0.0
          : static_cast<double>(delta.native_instructions) /
                static_cast<double>(delta.native_block_entries);
  const double rejected_avg =
      delta.native_rejected_block_count == 0
          ? 0.0
          : static_cast<double>(delta.native_rejected_block_instructions) /
                static_cast<double>(delta.native_rejected_block_count);
  const u64 instr_total = delta.decoded_instructions +
                          delta.native_instructions +
                          delta.fallback_instructions;
  const double native_pct =
      instr_total == 0
          ? 0.0
          : (100.0 * static_cast<double>(delta.native_instructions)) /
                static_cast<double>(instr_total);
  const double decoded_pct =
      instr_total == 0
          ? 0.0
          : (100.0 * static_cast<double>(delta.decoded_instructions)) /
                static_cast<double>(instr_total);
  const double fallback_pct =
      instr_total == 0
          ? 0.0
          : (100.0 * static_cast<double>(delta.fallback_instructions)) /
                static_cast<double>(instr_total);
  const u64 entry_total =
      delta.decoded_block_entries + delta.native_block_entries;
  const double native_entry_pct =
      entry_total == 0
          ? 0.0
          : (100.0 * static_cast<double>(delta.native_block_entries)) /
                static_cast<double>(entry_total);
  const double decoded_entry_pct =
      entry_total == 0
          ? 0.0
          : (100.0 * static_cast<double>(delta.decoded_block_entries)) /
                static_cast<double>(entry_total);
  const double native_decode_fallbacks_per_entry =
      delta.native_block_entries == 0
          ? 0.0
          : static_cast<double>(delta.native_to_decoded_fallbacks) /
                static_cast<double>(delta.native_block_entries);

  struct Reason {
    const char *name = "";
    u64 count = 0;
  };
  std::array<Reason, 17> reasons = {{
      {"branch", delta.native_reject_branch},
      {"memory", delta.native_reject_memory},
      {"cop0", delta.native_reject_cop0},
      {"cop2_gte", delta.native_reject_cop2},
      {"exception_unknown", delta.native_reject_exception_unknown},
      {"pc_state", delta.native_reject_pc_state},
      {"branch_delay_state", delta.native_reject_branch_delay_state},
      {"load_delay_state", delta.native_reject_load_delay_state},
      {"irq_state", delta.native_reject_irq_state},
      {"invalidated_state", delta.native_reject_invalidated_state},
      {"other_state", delta.native_reject_other_state},
      {"budget", delta.native_reject_budget},
      {"icache", delta.native_reject_icache},
      {"mmio", delta.native_reject_mmio},
      {"unaligned", delta.native_reject_unaligned},
      {"cold", delta.native_hot_threshold_skips},
      {"too_short", delta.native_short_block_skips},
  }};
  std::sort(reasons.begin(), reasons.end(),
            [](const Reason &a, const Reason &b) {
              return a.count > b.count;
            });

  std::array<Reason, 11> exact_reasons = {{
      {"pc_mismatch", delta.native_reject_pc_mismatch},
      {"next_pc_mismatch", delta.native_reject_next_pc_mismatch},
      {"block_start_after_branch_delay",
       delta.native_reject_block_start_after_branch_delay},
      {"stale_invalid_block_state",
       delta.native_reject_stale_invalid_block_state},
      {"in_delay_slot", delta.native_reject_branch_delay_in_delay_slot},
      {"pending_delay_slot",
       delta.native_reject_branch_delay_pending_delay_slot},
      {"pending_branch_taken",
       delta.native_reject_branch_delay_pending_branch_taken},
      {"pending_branch_pc",
       delta.native_reject_branch_delay_pending_branch_pc},
      {"exception_risk", delta.native_reject_exception_risk},
      {"fallback_instruction", delta.native_reject_fallback_instruction},
      {"unsupported_instruction",
       delta.native_reject_unsupported_instruction},
  }};
  std::sort(exact_reasons.begin(), exact_reasons.end(),
            [](const Reason &a, const Reason &b) {
              return a.count > b.count;
            });

  LOG_INFO("=== CPU Backend Stats ===");
  LOG_INFO("CPU_BACKEND_STATS frame=%u frames=%u mode=%s blocks=%u native_blocks=%llu code_bytes=%zu native_code_bytes=%zu",
           current_frame_, frame_delta,
           cpu_execution_mode_name(effective_cpu_execution_mode()),
           current.block_count,
           static_cast<unsigned long long>(current.native_blocks),
           current.code_bytes, current.native_code_bytes);
  LOG_INFO("CPU_BACKEND_STATS entries decoded=%llu native=%llu avg_decoded=%.2f avg_native=%.2f",
           static_cast<unsigned long long>(delta.decoded_block_entries),
           static_cast<unsigned long long>(delta.native_block_entries),
           decoded_avg, native_avg);
  LOG_INFO("CPU_BACKEND_STATS entry_mix native_pct=%.2f decoded_pct=%.2f native_to_decoded_fallbacks_per_native_entry=%.3f",
           native_entry_pct, decoded_entry_pct,
           native_decode_fallbacks_per_entry);
  LOG_INFO("CPU_BACKEND_STATS rejected_blocks count=%llu instructions=%llu avg_rejected=%.2f",
           static_cast<unsigned long long>(delta.native_rejected_block_count),
           static_cast<unsigned long long>(
               delta.native_rejected_block_instructions),
           rejected_avg);
  LOG_INFO("CPU_BACKEND_STATS instructions decoded=%llu native=%llu fallback=%llu native_pct=%.2f decoded_pct=%.2f fallback_pct=%.2f",
           static_cast<unsigned long long>(delta.decoded_instructions),
           static_cast<unsigned long long>(delta.native_instructions),
           static_cast<unsigned long long>(delta.fallback_instructions),
           native_pct, decoded_pct, fallback_pct);
  LOG_INFO("CPU_BACKEND_STATS forced_interpreter reason=%s slices=%llu instructions=%llu trace=%llu deep_diagnostics=%llu fmv=%llu backend_compare=%llu other=%llu",
           cpu_forced_interpreter_reason_name(
               current.forced_interpreter_last_reason),
           static_cast<unsigned long long>(delta.forced_interpreter_slices),
           static_cast<unsigned long long>(
               delta.forced_interpreter_instructions),
           static_cast<unsigned long long>(delta.forced_interpreter_trace),
           static_cast<unsigned long long>(
               delta.forced_interpreter_deep_diagnostics),
           static_cast<unsigned long long>(delta.forced_interpreter_fmv),
           static_cast<unsigned long long>(
               delta.forced_interpreter_backend_compare),
           static_cast<unsigned long long>(delta.forced_interpreter_other));
  LOG_INFO("CPU_BACKEND_STATS native attempts=%llu successes=%llu compiled=%llu failures=%llu unsafe=%llu decoded_fallbacks=%llu",
           static_cast<unsigned long long>(delta.native_compile_attempts),
           static_cast<unsigned long long>(delta.native_compile_successes),
           static_cast<unsigned long long>(delta.native_blocks_compiled),
           static_cast<unsigned long long>(delta.native_compile_failures),
           static_cast<unsigned long long>(delta.native_rejected_unsafe_blocks),
           static_cast<unsigned long long>(delta.native_to_decoded_fallbacks));
  const double reduced_helper_avg =
      delta.native_reduced_helper_entries == 0
          ? 0.0
          : static_cast<double>(delta.native_reduced_helper_instructions) /
                static_cast<double>(delta.native_reduced_helper_entries);
  LOG_INFO("CPU_BACKEND_STATS reduced_helper attempts=%llu successes=%llu compiled=%llu ram_load_compiled=%llu entries=%llu ram_load_entries=%llu instructions=%llu avg_block=%.2f load_delay_entries=%llu rejected_blocks=%llu rejected_instructions=%llu fallbacks=%llu",
           static_cast<unsigned long long>(
               delta.native_reduced_helper_compile_attempts),
           static_cast<unsigned long long>(
               delta.native_reduced_helper_compile_successes),
           static_cast<unsigned long long>(
               delta.native_reduced_helper_blocks_compiled),
           static_cast<unsigned long long>(
               delta.native_reduced_helper_ram_load_blocks_compiled),
           static_cast<unsigned long long>(delta.native_reduced_helper_entries),
           static_cast<unsigned long long>(
               delta.native_reduced_helper_ram_load_entries),
           static_cast<unsigned long long>(
               delta.native_reduced_helper_instructions),
           reduced_helper_avg,
           static_cast<unsigned long long>(
               delta.native_reduced_helper_load_delay_entries),
           static_cast<unsigned long long>(
               delta.native_reduced_helper_rejected_blocks),
           static_cast<unsigned long long>(
               delta.native_reduced_helper_rejected_instructions),
           static_cast<unsigned long long>(
               delta.native_reduced_helper_fallbacks));
  LOG_INFO("CPU_BACKEND_STATS reduced_helper_ram_load rejects_store=%llu rejects_base_written=%llu rejects_unsupported=%llu preflight_fallbacks=%llu preflight_mmio=%llu preflight_unaligned=%llu preflight_non_ram=%llu preflight_disabled=%llu",
           static_cast<unsigned long long>(
               delta.native_reduced_helper_reject_stores),
           static_cast<unsigned long long>(
               delta.native_reduced_helper_reject_load_base_written),
           static_cast<unsigned long long>(
               delta.native_reduced_helper_reject_unsupported_memory),
           static_cast<unsigned long long>(
               delta.native_reduced_helper_ram_load_preflight_fallbacks),
           static_cast<unsigned long long>(
               delta.native_reduced_helper_ram_load_preflight_mmio),
           static_cast<unsigned long long>(
               delta.native_reduced_helper_ram_load_preflight_unaligned),
           static_cast<unsigned long long>(
               delta.native_reduced_helper_ram_load_preflight_non_ram),
           static_cast<unsigned long long>(
               delta.native_reduced_helper_ram_load_preflight_disabled));
  LOG_INFO("CPU_BACKEND_STATS native_candidates reduced_helper=%llu direct_helper=%llu",
           static_cast<unsigned long long>(
               delta.native_reduced_helper_candidate_blocks),
           static_cast<unsigned long long>(
               delta.native_direct_helper_candidate_blocks));
  LOG_INFO("CPU_BACKEND_STATS native_shapes straight_alu=%llu straight_alu_ram_load=%llu straight_alu_ram_store=%llu helper_safe_memory=%llu branch_tail=%llu fallback_control=%llu cop0=%llu cop2_gte=%llu mmio_calls=%llu exception_unsafe=%llu",
           static_cast<unsigned long long>(delta.native_shape_straight_alu),
           static_cast<unsigned long long>(
               delta.native_shape_straight_alu_ram_load),
           static_cast<unsigned long long>(
               delta.native_shape_straight_alu_ram_store),
           static_cast<unsigned long long>(
               delta.native_shape_helper_safe_memory),
           static_cast<unsigned long long>(delta.native_shape_branch_tail),
           static_cast<unsigned long long>(
               delta.native_shape_fallback_control),
           static_cast<unsigned long long>(delta.native_shape_cop0),
           static_cast<unsigned long long>(delta.native_shape_cop2),
           static_cast<unsigned long long>(
               delta.native_memory_helper_mmio_calls),
           static_cast<unsigned long long>(
               delta.native_shape_exception_unsafe));
  LOG_INFO("CPU_BACKEND_STATS branch_tail enabled=%u native_branch_tail_blocks_compiled=%llu native_branch_tail_entries=%llu native_branch_taken=%llu native_branch_not_taken=%llu native_branch_tail_rejects=%llu disabled_fallbacks=%llu blacklisted_fallbacks=%llu native_branch_delay_slot_memory_helpers=%llu",
           cpu_x64_jit_branch_tail_enabled() ? 1u : 0u,
           static_cast<unsigned long long>(
               delta.native_branch_tail_blocks_compiled),
           static_cast<unsigned long long>(delta.native_branch_tail_entries),
           static_cast<unsigned long long>(delta.native_branch_taken),
           static_cast<unsigned long long>(delta.native_branch_not_taken),
           static_cast<unsigned long long>(delta.native_branch_tail_rejects),
           static_cast<unsigned long long>(
               delta.native_branch_tail_disabled_fallbacks),
           static_cast<unsigned long long>(
               delta.native_branch_tail_blacklisted_fallbacks),
           static_cast<unsigned long long>(
               delta.native_branch_delay_slot_memory_helpers));
  LOG_INFO("CPU_BACKEND_STATS branch_tail_signed bgtz_entries=%llu bgtz_taken=%llu bgtz_not_taken=%llu blez_entries=%llu blez_taken=%llu blez_not_taken=%llu",
           static_cast<unsigned long long>(
               delta.native_branch_tail_bgtz_entries),
           static_cast<unsigned long long>(
               delta.native_branch_tail_bgtz_taken),
           static_cast<unsigned long long>(
               delta.native_branch_tail_bgtz_not_taken),
           static_cast<unsigned long long>(
               delta.native_branch_tail_blez_entries),
           static_cast<unsigned long long>(
               delta.native_branch_tail_blez_taken),
           static_cast<unsigned long long>(
               delta.native_branch_tail_blez_not_taken));
  LOG_INFO("CPU_BACKEND_STATS branch_tail_reject_opcodes bne=%llu beq=%llu bgtz=%llu blez=%llu other=%llu",
           static_cast<unsigned long long>(
               delta.native_branch_tail_reject_bne),
           static_cast<unsigned long long>(
               delta.native_branch_tail_reject_beq),
           static_cast<unsigned long long>(
               delta.native_branch_tail_reject_bgtz),
           static_cast<unsigned long long>(
               delta.native_branch_tail_reject_blez),
           static_cast<unsigned long long>(
               delta.native_branch_tail_reject_other_opcode));
  LOG_INFO("CPU_BACKEND_STATS native_tiers all_enabled=%u memory_enabled=%u alu_enabled=%u branch_tail_enabled=%u memory_compiled=%llu memory_entries=%llu alu_compiled=%llu alu_entries=%llu all_disabled_fallbacks=%llu memory_disabled_fallbacks=%llu alu_disabled_fallbacks=%llu compare_flag_leak_warnings=%llu",
           cpu_x64_jit_all_native_enabled() ? 1u : 0u,
           cpu_x64_jit_native_memory_enabled() ? 1u : 0u,
           cpu_x64_jit_native_alu_enabled() ? 1u : 0u,
           cpu_x64_jit_branch_tail_enabled() ? 1u : 0u,
           static_cast<unsigned long long>(
               delta.native_memory_blocks_compiled),
           static_cast<unsigned long long>(
               delta.native_memory_block_entries),
           static_cast<unsigned long long>(
               delta.native_alu_blocks_compiled),
           static_cast<unsigned long long>(delta.native_alu_block_entries),
           static_cast<unsigned long long>(
               delta.native_all_disabled_fallbacks),
           static_cast<unsigned long long>(
               delta.native_memory_disabled_fallbacks),
           static_cast<unsigned long long>(
               delta.native_alu_disabled_fallbacks),
           static_cast<unsigned long long>(delta.compare_flag_leak_warnings));
  LOG_INFO("CPU_BACKEND_STATS native_memory_filters loads_disabled=%u stores_disabled=%u mmio_disabled=%u ram_disabled=%u load_delay_disabled=%u mixed_disabled=%u load_fallbacks=%llu store_fallbacks=%llu mmio_fallbacks=%llu ram_fallbacks=%llu load_delay_fallbacks=%llu mixed_fallbacks=%llu",
           g_cpu_x64_jit_disable_native_loads ? 1u : 0u,
           g_cpu_x64_jit_disable_native_stores ? 1u : 0u,
           g_cpu_x64_jit_disable_native_mmio ? 1u : 0u,
           g_cpu_x64_jit_disable_native_ram ? 1u : 0u,
           g_cpu_x64_jit_disable_native_load_delay ? 1u : 0u,
           g_cpu_x64_jit_disable_native_mixed_load_store ? 1u : 0u,
           static_cast<unsigned long long>(delta.native_load_disabled_fallbacks),
           static_cast<unsigned long long>(delta.native_store_disabled_fallbacks),
           static_cast<unsigned long long>(delta.native_mmio_disabled_fallbacks),
           static_cast<unsigned long long>(delta.native_ram_disabled_fallbacks),
           static_cast<unsigned long long>(
               delta.native_load_delay_disabled_fallbacks),
           static_cast<unsigned long long>(
               delta.native_mixed_memory_disabled_fallbacks));
  LOG_INFO("CPU_BACKEND_STATS native_gating hot_threshold=%u min_block=%u force=%u cold_skips=%llu short_skips=%llu",
           g_cpu_x64_jit_hot_block_threshold,
           g_cpu_x64_jit_min_block_instructions,
           g_cpu_x64_jit_force_compile ? 1u : 0u,
           static_cast<unsigned long long>(delta.native_hot_threshold_skips),
           static_cast<unsigned long long>(delta.native_short_block_skips));
  LOG_INFO("CPU_BACKEND_STATS diagnostic_forces trace=%u deep=%u fmv=%u forced_active=%u",
           g_trace_cpu ? 1u : 0u, g_cpu_deep_diagnostics ? 1u : 0u,
           g_log_fmv_diagnostics ? 1u : 0u,
           (delta.forced_interpreter_slices != 0 ||
            current.forced_interpreter_last_reason !=
                CpuForcedInterpreterReason::None)
               ? 1u
               : 0u);
  LOG_INFO("CPU_BACKEND_STATS top_rejections %s=%llu %s=%llu %s=%llu",
           reasons[0].name, static_cast<unsigned long long>(reasons[0].count),
           reasons[1].name, static_cast<unsigned long long>(reasons[1].count),
           reasons[2].name, static_cast<unsigned long long>(reasons[2].count));
  LOG_INFO("CPU_BACKEND_STATS top_exact_rejections %s=%llu %s=%llu %s=%llu",
           exact_reasons[0].name,
           static_cast<unsigned long long>(exact_reasons[0].count),
           exact_reasons[1].name,
           static_cast<unsigned long long>(exact_reasons[1].count),
           exact_reasons[2].name,
           static_cast<unsigned long long>(exact_reasons[2].count));
  LOG_INFO("CPU_BACKEND_STATS rejection_counts branch=%llu memory=%llu cop0=%llu cop2_gte=%llu exception_unknown=%llu exception_risk=%llu fallback_instruction=%llu unsupported_instruction=%llu unsafe_state=%llu pc_state=%llu branch_delay_state=%llu load_delay_state=%llu irq_state=%llu invalidated_state=%llu other_state=%llu budget=%llu icache=%llu mmio=%llu unaligned=%llu",
           static_cast<unsigned long long>(delta.native_reject_branch),
           static_cast<unsigned long long>(delta.native_reject_memory),
           static_cast<unsigned long long>(delta.native_reject_cop0),
           static_cast<unsigned long long>(delta.native_reject_cop2),
           static_cast<unsigned long long>(
               delta.native_reject_exception_unknown),
           static_cast<unsigned long long>(
               delta.native_reject_exception_risk),
           static_cast<unsigned long long>(
               delta.native_reject_fallback_instruction),
           static_cast<unsigned long long>(
               delta.native_reject_unsupported_instruction),
           static_cast<unsigned long long>(delta.native_reject_unsafe_state),
           static_cast<unsigned long long>(delta.native_reject_pc_state),
           static_cast<unsigned long long>(
               delta.native_reject_branch_delay_state),
           static_cast<unsigned long long>(
               delta.native_reject_load_delay_state),
           static_cast<unsigned long long>(delta.native_reject_irq_state),
           static_cast<unsigned long long>(
               delta.native_reject_invalidated_state),
           static_cast<unsigned long long>(delta.native_reject_other_state),
           static_cast<unsigned long long>(delta.native_reject_budget),
           static_cast<unsigned long long>(delta.native_reject_icache),
           static_cast<unsigned long long>(delta.native_reject_mmio),
           static_cast<unsigned long long>(delta.native_reject_unaligned));
  LOG_INFO("CPU_BACKEND_STATS rejection_exact pc_mismatch=%llu next_pc_mismatch=%llu block_start_after_branch_delay=%llu stale_invalid_block_state=%llu branch_delay_in_delay_slot=%llu branch_delay_pending_delay_slot=%llu branch_delay_pending_branch_taken=%llu branch_delay_pending_branch_pc=%llu",
           static_cast<unsigned long long>(delta.native_reject_pc_mismatch),
           static_cast<unsigned long long>(
               delta.native_reject_next_pc_mismatch),
           static_cast<unsigned long long>(
               delta.native_reject_block_start_after_branch_delay),
           static_cast<unsigned long long>(
               delta.native_reject_stale_invalid_block_state),
           static_cast<unsigned long long>(
               delta.native_reject_branch_delay_in_delay_slot),
           static_cast<unsigned long long>(
               delta.native_reject_branch_delay_pending_delay_slot),
           static_cast<unsigned long long>(
               delta.native_reject_branch_delay_pending_branch_taken),
           static_cast<unsigned long long>(
               delta.native_reject_branch_delay_pending_branch_pc));
  const u64 native_helper_calls = delta.native_prepare_helper_calls +
                                  delta.native_finish_helper_calls +
                                  delta.native_memory_helper_calls +
                                  delta.native_branch_helper_calls;
  const double helper_calls_per_block =
      delta.native_block_entries != 0
          ? static_cast<double>(native_helper_calls) /
                static_cast<double>(delta.native_block_entries)
          : 0.0;
  const double helper_calls_per_instruction =
      delta.native_instructions != 0
          ? static_cast<double>(native_helper_calls) /
                static_cast<double>(delta.native_instructions)
          : 0.0;
  const u64 direct_helper_entries =
      delta.native_block_entries >= delta.native_reduced_helper_entries
          ? delta.native_block_entries - delta.native_reduced_helper_entries
          : 0u;
  const u64 direct_helper_instructions =
      delta.native_instructions >= delta.native_reduced_helper_instructions
          ? delta.native_instructions -
                delta.native_reduced_helper_instructions
          : 0u;
  const double direct_helpers_per_block =
      direct_helper_entries == 0
          ? 0.0
          : static_cast<double>(native_helper_calls) /
                static_cast<double>(direct_helper_entries);
  const double direct_helpers_per_instruction =
      direct_helper_instructions == 0
          ? 0.0
          : static_cast<double>(native_helper_calls) /
                static_cast<double>(direct_helper_instructions);
  const u64 ram_fastpath_load_attempts =
      delta.native_memory_fastpath_loads +
      delta.native_memory_fastpath_load_misses;
  const double ram_fastpath_hit_pct =
      ram_fastpath_load_attempts == 0
          ? 0.0
          : (100.0 *
             static_cast<double>(delta.native_memory_fastpath_loads)) /
                static_cast<double>(ram_fastpath_load_attempts);
  LOG_INFO("CPU_BACKEND_STATS native_helpers total=%llu prepare=%llu finish=%llu memory=%llu branch=%llu per_native_block=%.3f per_native_instruction=%.3f",
           static_cast<unsigned long long>(native_helper_calls),
           static_cast<unsigned long long>(delta.native_prepare_helper_calls),
           static_cast<unsigned long long>(delta.native_finish_helper_calls),
           static_cast<unsigned long long>(delta.native_memory_helper_calls),
           static_cast<unsigned long long>(delta.native_branch_helper_calls),
           helper_calls_per_block, helper_calls_per_instruction);
  LOG_INFO("CPU_BACKEND_STATS direct_helpers entries=%llu instructions=%llu per_block=%.3f per_instruction=%.3f reduced_helper_calls=0",
           static_cast<unsigned long long>(direct_helper_entries),
           static_cast<unsigned long long>(direct_helper_instructions),
           direct_helpers_per_block, direct_helpers_per_instruction);
  LOG_INFO("CPU_BACKEND_STATS memory_helper_regions ram=%llu scratchpad=%llu bios_read_only=%llu mmio=%llu unknown_slow=%llu unaligned=%llu",
           static_cast<unsigned long long>(
               delta.native_memory_helper_ram_calls),
           static_cast<unsigned long long>(
               delta.native_memory_helper_scratchpad_calls),
           static_cast<unsigned long long>(
               delta.native_memory_helper_bios_calls),
           static_cast<unsigned long long>(
               delta.native_memory_helper_mmio_calls),
           static_cast<unsigned long long>(
               delta.native_memory_helper_unknown_calls),
           static_cast<unsigned long long>(
               delta.native_memory_helper_unaligned_calls));
  LOG_INFO("CPU_BACKEND_STATS memory_helpers total=%llu native=%llu native_loads=%llu native_stores=%llu shared_decoded=%llu operand_mismatches=%llu ram_load_fastpath_enabled=%u native_fast_loads=%llu native_fast_stores=%llu mmio_fast_loads=%llu native_exception_exits=%llu mmio=%llu exceptions=%llu",
           static_cast<unsigned long long>(delta.memory_helper_calls),
           static_cast<unsigned long long>(delta.native_memory_helper_calls),
           static_cast<unsigned long long>(
               delta.native_memory_helper_load_calls),
           static_cast<unsigned long long>(
               delta.native_memory_helper_store_calls),
           static_cast<unsigned long long>(
               delta.native_memory_shared_decoded_calls),
           static_cast<unsigned long long>(
               delta.native_memory_operand_mismatches),
           g_cpu_x64_jit_ram_load_fastpath_enabled ? 1u : 0u,
           static_cast<unsigned long long>(delta.native_memory_fastpath_loads),
           static_cast<unsigned long long>(delta.native_memory_fastpath_stores),
           static_cast<unsigned long long>(
               delta.native_memory_fastpath_mmio_loads),
           static_cast<unsigned long long>(
               delta.native_memory_exception_exits),
           static_cast<unsigned long long>(delta.mmio_accesses),
           static_cast<unsigned long long>(delta.exceptions));
  LOG_INFO("CPU_BACKEND_STATS ram_load_fastpath requested=%u active_without_trace=%u memory_trace=%u trace_ram=%u trace_bus=%u native_ram_disabled=%u hits=%llu misses=%llu miss_disabled=%llu miss_trace=%llu miss_unaligned=%llu miss_non_ram=%llu hit_pct=%.2f",
           g_cpu_x64_jit_ram_load_fastpath_enabled ? 1u : 0u,
           (g_cpu_x64_jit_ram_load_fastpath_enabled &&
            !g_cpu_x64_jit_memory_trace && !g_trace_ram && !g_trace_bus &&
            !g_cpu_x64_jit_disable_native_ram)
               ? 1u
               : 0u,
           g_cpu_x64_jit_memory_trace ? 1u : 0u,
           g_trace_ram ? 1u : 0u,
           g_trace_bus ? 1u : 0u,
           g_cpu_x64_jit_disable_native_ram ? 1u : 0u,
           static_cast<unsigned long long>(
               delta.native_memory_fastpath_loads),
           static_cast<unsigned long long>(
               delta.native_memory_fastpath_load_misses),
           static_cast<unsigned long long>(
               delta.native_memory_fastpath_load_miss_disabled),
           static_cast<unsigned long long>(
               delta.native_memory_fastpath_load_miss_trace),
           static_cast<unsigned long long>(
               delta.native_memory_fastpath_load_miss_unaligned),
           static_cast<unsigned long long>(
               delta.native_memory_fastpath_load_miss_non_ram),
           ram_fastpath_hit_pct);
  LOG_INFO("CPU_BACKEND_STATS helper_load_delay entries=%llu passes=%llu fallbacks=%llu",
           static_cast<unsigned long long>(
               delta.native_helper_load_delay_entries),
           static_cast<unsigned long long>(
               delta.native_helper_load_delay_passes),
           static_cast<unsigned long long>(
               delta.native_helper_load_delay_fallbacks));
  LOG_INFO("CPU_BACKEND_STATS cache hits=%llu misses=%llu invalidations=%llu invalidation_queries=%llu no_code_page=%llu examined=%llu invalidated=%llu flushes=%llu",
           static_cast<unsigned long long>(delta.cache_hits),
           static_cast<unsigned long long>(delta.cache_misses),
           static_cast<unsigned long long>(delta.invalidations),
           static_cast<unsigned long long>(delta.invalidation_queries),
           static_cast<unsigned long long>(
               delta.invalidation_fast_no_code_page_exits),
           static_cast<unsigned long long>(delta.invalidation_blocks_examined),
           static_cast<unsigned long long>(
               delta.invalidation_blocks_invalidated),
           static_cast<unsigned long long>(delta.flushes));
  log_rejected_block_profiles();
  log_native_branch_tail_diagnostics();
  log_native_entry_pc_diagnostics();
  log_native_helper_pc_diagnostics();
  LOG_INFO("=== End CPU Backend Stats ===");
}

void CpuOptimizedBackend::warn_x64_unavailable_once() {
  static bool process_warned = false;
  if (x64_unavailable_warned_ || process_warned) {
    return;
  }
  x64_unavailable_warned_ = true;
  process_warned = true;
  LOG_WARN(
      "CPU: x64 JIT mode requested, but no real native emitter is enabled yet; using decoded block interpreter.");
}
