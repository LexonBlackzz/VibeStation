#pragma once

#include "common/types.h"

#include <array>
#include <string>

namespace ps2 {

class EeBus;

struct EeGpr {
    u64 lo = 0;
    u64 hi = 0;
};

struct EeTlbEntry {
    u32 page_mask = 0;
    u32 entry_hi = 0;
    u32 entry_lo0 = 0;
    u32 entry_lo1 = 0;
};

struct EeCpuState {
    std::array<EeGpr, 32> gpr{};
    u64 hi = 0;
    u64 lo = 0;
    u64 hi1 = 0;
    u64 lo1 = 0;
    u32 pc = 0;
    u32 next_pc = 4;
    std::array<u32, 32> cop0{};
    std::array<EeTlbEntry, 48> tlb{};
    std::array<u32, 32> fpr{};
    std::array<u32, 32> fcr{};
    u32 fpu_acc = 0;
    std::array<EeGpr, 32> vu_vf{};
    std::array<u32, 32> vu_vi{};
    u64 instructions_executed = 0;
    u32 last_pc = 0;
    u32 last_instruction = 0;
};

class EeCpu {
public:
    explicit EeCpu(EeBus& bus) : bus_(bus) {}

    void reset(u32 entry_point = 0);
    bool step(std::string& error);
    u64 run(u64 instruction_budget, std::string& error);

    [[nodiscard]] const EeCpuState& state() const { return state_; }
    [[nodiscard]] EeCpuState& state() { return state_; }
    [[nodiscard]] bool halted() const { return halted_; }
    [[nodiscard]] const std::string& halt_reason() const { return halt_reason_; }
    void clear_halt();

private:
    [[nodiscard]] static s16 immediate(u32 instruction);
    [[nodiscard]] static u32 branch_target(u32 pc, s16 imm);
    [[nodiscard]] static u64 sign_extend_word(u32 value);
    [[nodiscard]] u64 gpr_u64(u32 index) const;
    [[nodiscard]] s64 gpr_s64(u32 index) const;

    void write_gpr64(u32 index, u64 value);
    void write_gpr_word(u32 index, u32 value);
    void branch_likely_not_taken(u32 pc);

    bool fail(
        u32 pc,
        u32 instruction,
        const std::string& reason,
        std::string& error);
    bool execute_special(u32 pc, u32 instruction, std::string& error);
    bool execute_regimm(u32 pc, u32 instruction, std::string& error);
    bool execute_cop0(u32 pc, u32 instruction, std::string& error);
    bool execute_cop1(u32 pc, u32 instruction, std::string& error);
    bool execute_cop2(u32 pc, u32 instruction, std::string& error);
    bool execute_mmi(u32 pc, u32 instruction, std::string& error);

    EeBus& bus_;
    EeCpuState state_{};
    bool halted_ = false;
    std::string halt_reason_;
};

} // namespace ps2
