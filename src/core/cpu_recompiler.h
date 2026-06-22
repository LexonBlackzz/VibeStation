#pragma once

#include "cpu.h"
#include <array>
#include <cstddef>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

enum class DecodedOp : u16 {
  Unsupported,
  Nop,

  Sll,
  Srl,
  Sra,
  Sllv,
  Srlv,
  Srav,
  Jr,
  Jalr,
  Syscall,
  Break,
  Addu,
  Subu,
  And,
  Or,
  Xor,
  Nor,
  Slt,
  Sltu,

  Bltz,
  Bgez,
  Bltzal,
  Bgezal,
  J,
  Jal,
  Beq,
  Bne,
  Blez,
  Bgtz,
  Addiu,
  Slti,
  Sltiu,
  Andi,
  Ori,
  Xori,
  Lui,

  Lb,
  Lh,
  Lw,
  Lbu,
  Lhu,
  Sb,
  Sh,
  Sw,

  Cop0,
  Cop2,
};

enum class CpuBlockExitReason : u8 {
  None,
  Branch,
  Exception,
  Interrupt,
  Budget,
  Fallback,
  Invalidated,
  PcMismatch,
};

enum class NativeBlockRejectReason : u8 {
  None,
  Branch,
  Memory,
  Cop0,
  Cop2,
  ExceptionOrUnknown,
  Mmio,
  Unaligned,
};

enum class NativeBlockRejectDetail : u8 {
  None,
  Branch,
  Memory,
  Cop0,
  Cop2,
  ExceptionOrUnknown,
  PcMismatch,
  NextPcMismatch,
  BlockStartAfterBranchDelay,
  StaleInvalidBlockState,
  InDelaySlot,
  PendingDelaySlot,
  PendingBranchTaken,
  PendingBranchPc,
  LoadDelayState,
  IrqState,
  InvalidatedState,
  OtherState,
  Budget,
  ICache,
  Mmio,
  Unaligned,
  Cold,
  TooShort,
  CompileFailure,
  BranchTailMissingBranch,
  BranchTailUnsupportedBranch,
  BranchTailMissingDelaySlot,
  BranchTailUnsupportedBody,
  BranchTailUnsupportedDelaySlot,
  BranchTailNestedDelayBranch,
  BranchTailDisabled,
  BranchTailBlacklisted,
};

enum class NativeMemoryRegion : u8 {
  Ram,
  Scratchpad,
  BiosReadOnly,
  Mmio,
  UnknownSlow,
};

NativeMemoryRegion classify_native_memory_region(u32 addr);
const char *native_memory_region_name(NativeMemoryRegion region);

struct CpuBlockRunResult {
  u32 cycles = 0;
  u32 instructions = 0;
  CpuBlockExitReason exit_reason = CpuBlockExitReason::None;
};

struct DecodedInstruction {
  DecodedOp op = DecodedOp::Unsupported;
  u32 pc = 0;
  u32 bits = 0;
  u32 normalized_addr = 0;
  u32 target = 0;
  s32 simm = 0;
  u16 imm = 0;
  u8 rs = 0;
  u8 rt = 0;
  u8 rd = 0;
  u8 shamt = 0;
  u8 cycles = 1;
  bool is_branch = false;
  bool is_unconditional_branch = false;
  bool may_raise_exception = false;
  bool may_access_memory = false;
  bool must_fallback = false;
};

struct DecodedBlock {
  static constexpr u32 kMaxInstructions = 17;
  using NativeFn = void (*)(void *, CpuBlockRunResult *);

  DecodedBlock() = default;
  ~DecodedBlock();

  DecodedBlock(const DecodedBlock &) = delete;
  DecodedBlock &operator=(const DecodedBlock &) = delete;

  u32 start_pc = 0;
  u32 start_key = 0;
  std::array<DecodedInstruction, kMaxInstructions> instructions{};
  u32 instruction_count = 0;
  std::array<std::pair<u32, u32>, kMaxInstructions> tracked_ranges{};
  u32 tracked_range_count = 0;
  std::array<u32, kMaxInstructions> registered_pages{};
  u32 registered_page_count = 0;
  u32 last_invalidation_query = 0;
  u64 entry_count = 0;
  u64 native_branch_tail_entry_count = 0;
  u64 native_prepare_helper_call_count = 0;
  u64 native_finish_helper_call_count = 0;
  u64 native_memory_helper_call_count = 0;
  u64 native_branch_helper_call_count = 0;
  mutable u64 native_helper_calls_logged = 0;
  NativeFn native_fn = nullptr;
  void *native_context = nullptr;
  void (*destroy_native_context)(void *) = nullptr;
  size_t native_code_bytes = 0;
  u32 compile_frame = 0;
  u32 last_invalidate_frame = 0;
  u32 invalidations_in_window = 0;
  u32 recompiles_in_window = 0;
  u32 interpreter_only_until_frame = 0;
  bool invalidated = false;
  bool has_control_flow = false;
  bool has_fallback = false;
  bool has_memory = false;
  bool has_load = false;
  bool has_store = false;
  u8 native_memory_runtime_filter_reason = 0;
  bool native_compile_attempted = false;
  bool native_safety_checked = false;
  bool native_stage1_safe = false;
  bool native_branch_tail = false;
  bool native_rejected_unsafe = false;
  NativeBlockRejectReason native_reject_reason =
      NativeBlockRejectReason::None;
  NativeBlockRejectDetail native_reject_detail =
      NativeBlockRejectDetail::None;
};

class CpuOptimizedBackend {
public:
  explicit CpuOptimizedBackend(Cpu &cpu);
  ~CpuOptimizedBackend();

  bool decoded_available() const { return true; }
  bool x64_jit_available() const;
  CpuRunSliceResult run_slice(u32 max_cycles, u32 max_instructions,
                              CpuExecutionMode requested_mode);
  void invalidate_range(u32 phys_or_normalized_addr, u32 size_bytes);
  void begin_frame(u32 frame_index);
  void flush();
  CpuBackendStats stats() const;

private:
  static constexpr u32 kMaxBlockInstructions = 16;
  static constexpr u32 kInterpreterFallbackRecompiles = 3;
  static constexpr u32 kInterpreterFallbackFrames = 15;
  static constexpr u32 kInterpreterOnlyFrames = 60;
  static constexpr size_t kMaxDecodedBlocks = 16384u;
  static constexpr u32 kCodePageShift = 12;
  static constexpr u32 kCodePageSize = 1u << kCodePageShift;
  static constexpr u32 kCodePageCount = 1u << (32u - kCodePageShift);
  static constexpr u32 kCodePageBitmapWordCount = kCodePageCount / 64u;

  struct BlockHistory {
    u32 window_start_frame = 0;
    u32 compile_count_in_window = 0;
    u32 invalidation_count_in_window = 0;
    u32 interpreter_only_until_frame = 0;
  };

  struct NativeRejectedBlockProfile {
    u32 start_pc = 0;
    u32 instruction_count = 0;
    u64 rejections = 0;
    u64 rejected_instructions = 0;
    NativeBlockRejectDetail last_reason = NativeBlockRejectDetail::None;
    bool contains_branch = false;
    bool contains_memory = false;
    bool contains_cop = false;
    bool contains_syscall_break = false;
    bool contains_fallback = false;
    std::array<DecodedOp, DecodedBlock::kMaxInstructions> ops{};
  };

  struct NativeBranchTailTrace {
    u64 sequence = 0;
    u32 start_pc = 0;
    u32 branch_pc = 0;
    u32 target = 0;
    u32 final_pc = 0;
    u32 final_next_pc = 0;
    DecodedOp delay_op = DecodedOp::Unsupported;
    bool taken = false;
    bool delay_memory = false;
    bool delay_mmio = false;
    bool exception = false;
  };

  DecodedBlock *lookup_or_decode(u32 pc);
  DecodedBlock *decode_block(u32 pc);
  DecodedInstruction decode_instruction(u32 pc, u32 bits) const;
  bool append_decoded_instruction(DecodedBlock &block,
                                  const DecodedInstruction &inst);
  CpuBlockRunResult execute_block(DecodedBlock &block, u32 max_cycles,
                                  u32 max_instructions);
  CpuBlockRunResult execute_native_block(DecodedBlock &block, u32 max_cycles,
                                         u32 max_instructions);
  bool execute_decoded_instruction(const DecodedInstruction &inst);
  bool prepare_instruction(const DecodedInstruction &inst,
                           CpuBlockRunResult &result);
  void finish_instruction(const DecodedInstruction &inst,
                          CpuBlockRunResult &result, bool count_decoded);
  CpuForcedInterpreterReason diagnostics_force_interpreter_reason() const;
  bool is_block_terminator(const DecodedInstruction &inst) const;
  NativeBlockRejectReason classify_x64_reject_reason(
      const DecodedBlock &block) const;
  bool ensure_x64_safety_checked(DecodedBlock &block);
  bool should_attempt_x64_compile(const DecodedBlock &block);
  bool compile_x64_block(DecodedBlock &block);
  static bool x64_native_prepare_instruction(void *context, u32 index,
                                             CpuBlockRunResult *result);
  static bool x64_native_memory_instruction(void *context, u32 op,
                                            u32 rt_or_value, u32 addr);
  static void x64_native_branch_instruction(void *context, u32 taken);
  static bool x64_native_finish_instruction(void *context, u32 index,
                                            CpuBlockRunResult *result,
                                            u32 memory_instruction);
  bool is_mmio_address(u32 addr) const;
  u32 normalized_code_addr(u32 addr) const;
  u32 code_page_for_normalized_addr(u32 addr) const;
  bool code_page_maybe_compiled(u32 page) const;
  void set_code_page_compiled(u32 page);
  void clear_code_page_compiled(u32 page);
  bool register_block_page(DecodedBlock &block, u32 page);
  void register_block_pages(DecodedBlock &block);
  void unregister_block_pages(DecodedBlock &block);
  bool ranges_overlap(u32 a0, u32 a1, u32 b0, u32 b1) const;
  void mark_block_invalidated(DecodedBlock &block);
  void record_forced_interpreter(CpuForcedInterpreterReason reason,
                                 u32 instructions);
  void record_exit(CpuBlockExitReason reason);
  void record_native_reject(NativeBlockRejectReason reason);
  NativeBlockRejectDetail native_reject_detail_for_reason(
      NativeBlockRejectReason reason) const;
  NativeBlockRejectDetail classify_native_pc_state_reject(
      const DecodedBlock &block) const;
  NativeBlockRejectDetail record_native_branch_delay_subreasons();
  void record_native_pc_state_subreason(NativeBlockRejectDetail detail);
  void record_native_block_rejection(
      const DecodedBlock &block, NativeBlockRejectDetail detail);
  void log_rejected_block_profiles() const;
  bool native_branch_tail_pc_blacklisted(const DecodedBlock &block) const;
  void record_native_branch_tail_trace(const DecodedBlock &block, bool taken,
                                       bool delay_memory, bool delay_mmio,
                                       bool exception);
  void log_native_branch_tail_diagnostics() const;
  void log_native_helper_pc_diagnostics() const;
  void warn_forced_interpreter_once(CpuForcedInterpreterReason reason);
  void log_periodic_stats();
  void log_stats_section(const CpuBackendStats &current,
                         const CpuBackendStats &delta,
                         u32 frame_delta) const;
  void warn_x64_unavailable_once();

  Cpu &cpu_;
  std::unordered_map<u32, std::unique_ptr<DecodedBlock>> blocks_;
  std::unordered_map<u32, BlockHistory> block_history_;
  std::unordered_map<u32, NativeRejectedBlockProfile> rejected_block_profiles_;
  std::deque<NativeBranchTailTrace> recent_native_branch_tails_;
  std::unordered_map<u32, std::vector<DecodedBlock *>> blocks_by_page_;
  std::array<u64, kCodePageBitmapWordCount> compiled_code_page_bitmap_{};
  u32 invalidation_query_stamp_ = 0;
  u64 native_branch_tail_trace_sequence_ = 0;
  u64 native_memory_trace_emitted_ = 0;
  u32 current_frame_ = 0;
  u32 last_stats_log_frame_ = 0;
  bool x64_unavailable_warned_ = false;
  bool have_stats_log_snapshot_ = false;
  CpuBackendStats last_stats_log_{};
  CpuBackendStats stats_{};
  CpuForcedInterpreterReason last_forced_interpreter_warning_ =
      CpuForcedInterpreterReason::None;
};
