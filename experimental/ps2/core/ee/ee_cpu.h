#pragma once

#include "common/types.h"
#include "core/ee/ee_jit.h"

#include <array>
#include <memory>
#include <string>

namespace ps2 {

class EeBus;
class Vu1;

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

struct EeSyscallRecord {
    u64 instruction = 0;
    u32 pc = 0;
    u32 number = 0;
    std::array<u64, 4> args{};
};

struct EeCpuState {
    std::array<EeGpr, 32> gpr{};
    u64 hi = 0;
    u64 lo = 0;
    u64 hi1 = 0;
    u64 lo1 = 0;
    u32 sa = 0;
    u32 pc = 0;
    u32 next_pc = 4;
    std::array<u32, 32> cop0{};
    std::array<EeTlbEntry, 48> tlb{};
    std::array<u32, 32> fpr{};
    std::array<u32, 32> fcr{};
    u32 fpu_acc = 0;
    std::array<EeGpr, 32> vu_vf{};
    EeGpr vu_acc{};
    std::array<u32, 32> vu_vi{};
    u64 instructions_executed = 0;
    u32 last_pc = 0;
    u32 last_instruction = 0;
    std::array<u64, 32> exception_counts{};
    std::array<EeSyscallRecord, 64> recent_syscalls{};
    u32 recent_syscall_count = 0;
    u32 recent_syscall_next = 0;
};

class EeCpu {
public:
    explicit EeCpu(EeBus& bus, Vu1* vu0_micro = nullptr)
        : bus_(bus), vu0_micro_(vu0_micro) {}

    void reset(u32 entry_point = 0);
    bool step(std::string& error);
    bool step_predecoded(u32 instruction, std::string& error);
    // Execute one instruction while the system layer guarantees that no
    // external EE interrupt or hardware-timer event can occur. The caller
    // must apply one deferred EeBus::tick cycle after successful retirement.
    bool step_quiet(std::string& error);
    // Same quiet path when the caller already fetched and classified the
    // current instruction. Valid only for direct-mapped instruction fetches.
    bool step_quiet_predecoded(u32 instruction, std::string& error);
    bool step_quiet_unchecked_predecoded(
        u32 instruction, std::string& error);
    // Execute the hot quiet-interpreter subset without re-entering the full
    // per-instruction decoder. With instruction_ram set and instructions null,
    // fetch directly from EE RAM and follow supported control flow across
    // basic-block boundaries. Fresh fetches preserve self-modifying-code
    // behavior without cached-block generation checks.
    u32 run_quiet_fast_prefix(
        u32 block_pc,
        const u32* instructions,
        u32 instruction_count,
        u32 maximum_instructions,
        bool* store_executed = nullptr,
        const u8* instruction_ram = nullptr,
        u8* direct_ram = nullptr,
        u32* page_generations = nullptr);
    u32 run_native_block(
        u32 pc,
        u32 page_generation,
        const u32* instructions,
        u32 instruction_count,
        u32 maximum_instructions,
        const u8* ram_data = nullptr,
        u32* page_generations = nullptr);
    u64 run(u64 instruction_budget, std::string& error);
    // Retire verified instructions from the OSDSYS eight-instruction idle
    // loop. The current PC may be at any phase of the loop. The caller is
    // responsible for advancing the other devices by the same cycle count
    // and for guarding against interrupts within that interval.
    bool skip_bios_idle_iteration();
    bool skip_bios_idle_iterations(u32 iterations);
    bool skip_bios_idle_instructions(u32 instructions);
    // Retire verified iterations of the BIOS's 16-byte RAM clear loop.
    bool skip_bios_zero_loop(u32 iterations);
    bool skip_bios_nibble_loop(u32 iterations);
    u32 skip_bios_count_wait(u32 max_iterations);
    u32 skip_bios_countdown_wait(u32 max_iterations);
    bool skip_bios_copy_iteration();
    bool skip_bios_copy_iterations(u32 iterations);
    bool skip_bios_mmio_poll_iteration();
    u32 skip_bios_mmio_poll_iterations(u32 max_iterations);
    bool skip_bios_literal_iteration();
    u32 skip_bios_literal_iterations(u32 max_iterations);
    // Validate/collapse the retail 0x7A SifGetReg(4) syscall wrapper.
    // The system layer is responsible for sampling SMFLAG at the exact
    // instruction offset when active IOP execution can change it.
    [[nodiscard]] bool can_skip_hot_sif_getreg() const;
    bool skip_hot_sif_getreg(u32 value);

    [[nodiscard]] const EeCpuState& state() const { return state_; }
    [[nodiscard]] EeCpuState& state() { return state_; }
    [[nodiscard]] bool halted() const { return halted_; }
    [[nodiscard]] const std::string& halt_reason() const { return halt_reason_; }
    void clear_halt();
    void set_jit_enabled(bool enabled) { jit_enabled_ = enabled; }
    void clear_jit_cache() { jit_.clear(); }
    [[nodiscard]] bool jit_enabled() const { return jit_enabled_; }
    [[nodiscard]] const EeJit& jit() const { return jit_; }
    [[nodiscard]] const std::array<u64, 64>&
    fast_prefix_fallback_opcodes() const {
        return *fast_prefix_fallback_opcodes_;
    }
    [[nodiscard]] const std::array<u32, 64>&
    fast_prefix_cop2_fallback_words() const {
        return *fast_prefix_cop2_fallback_words_;
    }
    [[nodiscard]] const std::array<u64, 64>&
    fast_prefix_cop2_fallback_counts() const {
        return *fast_prefix_cop2_fallback_counts_;
    }
    [[nodiscard]] u32 fast_prefix_cop2_fallback_count() const {
        return fast_prefix_cop2_fallback_count_;
    }
    [[nodiscard]] const std::array<u64, 64>&
    fast_prefix_control_fallback_keys() const {
        return *fast_prefix_control_fallback_keys_;
    }
    [[nodiscard]] const std::array<u64, 64>&
    fast_prefix_control_fallback_counts() const {
        return *fast_prefix_control_fallback_counts_;
    }
    [[nodiscard]] u32 fast_prefix_control_fallback_count() const {
        return fast_prefix_control_fallback_count_;
    }
    [[nodiscard]] const std::array<u64, 128>&
    fast_prefix_memory_fallback_keys() const {
        return *fast_prefix_memory_fallback_keys_;
    }
    [[nodiscard]] const std::array<u64, 128>&
    fast_prefix_memory_fallback_counts() const {
        return *fast_prefix_memory_fallback_counts_;
    }
    [[nodiscard]] u32 fast_prefix_memory_fallback_count() const {
        return fast_prefix_memory_fallback_count_;
    }
    [[nodiscard]] u32 hot_sif_getreg_read_offset() const {
        return hot_sif_getreg_read_offset_;
    }

    // VU0 macro mode (EE COP2) and VIF0 micro mode share one architectural
    // register file. These helpers bridge the bootstrap interpreter state.
    void sync_vu0_to_micro();
    void sync_vu0_from_micro();
    void set_vu0_micro_running(bool running);

private:
    bool step_internal(
        std::string& error,
        bool quiet,
        const u32* prefetched_instruction = nullptr,
        bool skip_interrupt_check = false);
    bool skip_bios_literal_iteration_impl(bool verify_code);
    [[nodiscard]] static s16 immediate(u32 instruction);
    [[nodiscard]] static u32 branch_target(u32 pc, s16 imm);
    [[nodiscard]] static u64 sign_extend_word(u32 value);
    [[nodiscard]] u64 gpr_u64(u32 index) const;
    [[nodiscard]] s64 gpr_s64(u32 index) const;

    void write_gpr64(u32 index, u64 value);
    void write_gpr_word(u32 index, u32 value);
    void branch_likely_not_taken(u32 pc);
    void raise_exception(
        u32 code,
        u32 pc,
        bool in_delay_slot,
        bool tlb_refill = false);
    [[nodiscard]] bool translate_address(
        u32 virtual_address,
        bool store,
        u32 fault_pc,
        bool in_delay_slot,
        u32& translated);

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
    bool run_vu0_micro(u32 start_address, std::string& error);
    bool execute_mmi(u32 pc, u32 instruction, std::string& error);

    EeBus& bus_;
    Vu1* vu0_micro_ = nullptr;
    EeCpuState state_{};
    std::unique_ptr<std::array<u64, 64>>
        fast_prefix_fallback_opcodes_ =
            std::make_unique<std::array<u64, 64>>();
    std::unique_ptr<std::array<u32, 64>>
        fast_prefix_cop2_fallback_words_ =
            std::make_unique<std::array<u32, 64>>();
    std::unique_ptr<std::array<u64, 64>>
        fast_prefix_cop2_fallback_counts_ =
            std::make_unique<std::array<u64, 64>>();
    u32 fast_prefix_cop2_fallback_count_ = 0u;
    std::unique_ptr<std::array<u64, 64>>
        fast_prefix_control_fallback_keys_ =
            std::make_unique<std::array<u64, 64>>();
    std::unique_ptr<std::array<u64, 64>>
        fast_prefix_control_fallback_counts_ =
            std::make_unique<std::array<u64, 64>>();
    u32 fast_prefix_control_fallback_count_ = 0u;
    std::unique_ptr<std::array<u64, 128>>
        fast_prefix_memory_fallback_keys_ =
            std::make_unique<std::array<u64, 128>>();
    std::unique_ptr<std::array<u64, 128>>
        fast_prefix_memory_fallback_counts_ =
            std::make_unique<std::array<u64, 128>>();
    u32 fast_prefix_memory_fallback_count_ = 0u;

    bool halted_ = false;
    bool next_is_delay_slot_ = false;
    bool current_is_delay_slot_ = false;
    bool memory_exception_pending_ = false;
    bool hot_sif_getreg_diag_inflight_ = false;
    u64 hot_sif_getreg_diag_start_ = 0u;
    u32 hot_sif_getreg_read_offset_ = 0u;
    std::string halt_reason_;
    EeJit jit_{};
    bool jit_enabled_ = false;
};

} // namespace ps2
