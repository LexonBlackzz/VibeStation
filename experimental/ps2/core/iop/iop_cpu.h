#pragma once

#include "common/types.h"

#include <array>
#include <memory>
#include <string>

namespace ps2 {

class IopBus;
class IopJit;

struct IopSyscallRecord {
    u64 instruction = 0;
    u32 pc = 0;
    u32 encoded = 0;
    u32 v0 = 0;
    std::array<u32, 4> args{};
};

struct IopCpuState {
    std::array<u32, 32> gpr{};
    u32 hi = 0;
    u32 lo = 0;
    u32 pc = 0;
    u32 next_pc = 4;
    std::array<u32, 32> cop0{};
    std::array<u32, 32> gte_data{};
    std::array<u32, 32> gte_ctrl{};
    u64 instructions_executed = 0;
    u32 last_pc = 0;
    u32 last_instruction = 0;
    std::array<u64, 32> exception_counts{};
    std::array<IopSyscallRecord, 64> recent_syscalls{};
    u32 recent_syscall_count = 0;
    u32 recent_syscall_next = 0;
};

class IopCpu {
public:
    explicit IopCpu(IopBus& bus);
    ~IopCpu();

    void reset(u32 entry_point = 0xBFC00000u);
    bool step(std::string& error);
    // Hot system-run entry: the caller owns a reusable empty error buffer.
    // Successful instructions never modify it, avoiding millions of
    // redundant std::string::clear() calls during BIOS execution.
    bool step_hot(std::string& error);
    [[nodiscard]] bool in_osdsys_idle_loop() const;
    bool skip_osdsys_idle_pair();
    u64 skip_osdsys_idle_pairs(u64 max_pairs);
    u64 run(u64 instruction_budget, std::string& error);
    // Execute only side-effect-free/native-safe R3000A blocks. This path
    // deliberately does not advance IOP hardware time; Ps2System batches the
    // exact retired cycle count at a verified event-free boundary.
    u32 run_native_quiet(u32 maximum_instructions);
    [[nodiscard]] u64 jit_native_instructions() const;
    [[nodiscard]] u64 jit_native_blocks() const;
    [[nodiscard]] u64 jit_native_chains() const;
    [[nodiscard]] u64 jit_guard_exits() const;
    [[nodiscard]] u64 jit_code_store_exits() const;
    [[nodiscard]] u64 jit_run_calls() const;
    [[nodiscard]] u64 jit_entry_attempts() const;
    [[nodiscard]] u64 jit_entry_successes() const;
    [[nodiscard]] u64 jit_compile_failures() const;
    [[nodiscard]] u64 jit_load_delay_entry_retires() const;
    [[nodiscard]] u64 jit_entry_reject(u32 reason) const;
    [[nodiscard]] u64 jit_residency_instructions() const;
    [[nodiscard]] u64 jit_residency_max() const;
    [[nodiscard]] std::array<u64, 16> jit_residency_histogram() const;

    [[nodiscard]] const IopCpuState& state() const { return state_; }
    [[nodiscard]] IopCpuState& state() { return state_; }

    [[nodiscard]] bool halted() const { return halted_; }
    [[nodiscard]] const std::string& halt_reason() const { return halt_reason_; }
    void clear_halt();

private:
    friend class IopJit;
    bool step_internal(std::string& error, bool clear_error);

    struct PendingLoad {
        bool valid = false;
        u32 reg = 0;
        u32 value = 0;
    };

    [[nodiscard]] static s16 immediate(u32 instruction);
    [[nodiscard]] static u32 branch_target(u32 pc, s16 imm);

    void write_gpr(u32 index, u32 value);
    void schedule_load(u32 index, u32 value);
    [[nodiscard]] u32 load_merge_base(u32 index) const;

    bool fail(
        u32 pc,
        u32 instruction,
        const std::string& reason,
        std::string& error);
    void raise_exception(u32 code, u32 pc, bool in_delay_slot);

    bool execute_special(
        u32 pc,
        u32 instruction,
        bool in_delay_slot,
        std::string& error);
    bool execute_regimm(
        u32 pc,
        u32 instruction,
        std::string& error);
    bool execute_cop0(
        u32 pc,
        u32 instruction,
        std::string& error);
    bool execute_cop2(
        u32 pc,
        u32 instruction,
        std::string& error);

    IopBus& bus_;
    IopCpuState state_{};
    std::unique_ptr<IopJit> jit_;

    PendingLoad pending_load_{};
    PendingLoad next_load_{};
    u32 direct_write_mask_ = 0;
    bool next_is_delay_slot_ = false;

    bool halted_ = false;
    std::string halt_reason_;
};

} // namespace ps2
