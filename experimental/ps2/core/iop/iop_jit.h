#pragma once

#include "common/types.h"

#include <array>
#include <cstddef>
#include <vector>

namespace ps2 {

class IopCpu;
struct IopCpuState;

// Native x64 execution tier for the PS2 IOP's R3000A.
//
// This is intentionally shaped after VibeStation's mature PS1 V4 backend:
// cache translated blocks, keep execution resident across basic blocks, use
// direct RAM stores when they are provably safe, and return to the existing
// interpreter for architectural corners which are not native yet.
class IopJit {
public:
    IopJit() = default;
    ~IopJit();
    IopJit(const IopJit&) = delete;
    IopJit& operator=(const IopJit&) = delete;

    u32 run(IopCpu& cpu, u32 maximum_instructions);
    void clear();

    [[nodiscard]] u64 block_compiled_count() const {
        return block_compiled_count_;
    }
    [[nodiscard]] u64 block_executed_count() const {
        return block_executed_count_;
    }
    [[nodiscard]] u64 instruction_count() const {
        return instruction_count_;
    }
    [[nodiscard]] u64 chain_count() const { return chain_count_; }
    [[nodiscard]] u64 guard_exit_count() const {
        return guard_exit_count_;
    }
    [[nodiscard]] u64 code_store_exit_count() const {
        return code_store_exit_count_;
    }
    [[nodiscard]] u64 cache_flush_count() const {
        return cache_flush_count_;
    }
    [[nodiscard]] u64 run_call_count() const { return run_call_count_; }
    [[nodiscard]] u64 native_entry_attempt_count() const {
        return native_entry_attempt_count_;
    }
    [[nodiscard]] u64 native_entry_success_count() const {
        return native_entry_success_count_;
    }
    [[nodiscard]] u64 compile_failure_count() const {
        return compile_failure_count_;
    }
    [[nodiscard]] u64 load_delay_entry_retire_count() const {
        return load_delay_entry_retire_count_;
    }
    [[nodiscard]] u64 native_delay_slot_count() const {
        return native_delay_slot_count_;
    }
    [[nodiscard]] const std::array<u64, 64>& compile_stop_opcodes() const {
        return compile_stop_opcodes_;
    }
    [[nodiscard]] const std::array<u64, 64>& delay_slot_stop_opcodes() const {
        return delay_slot_stop_opcodes_;
    }
    [[nodiscard]] const std::array<u64, 5>& entry_rejects() const {
        return entry_rejects_;
    }
    [[nodiscard]] u64 native_residency_instruction_count() const {
        return native_residency_instruction_count_;
    }
    [[nodiscard]] u64 native_residency_max() const {
        return native_residency_max_;
    }
    [[nodiscard]] const std::array<u64, 16>&
    native_residency_histogram() const {
        return native_residency_histogram_;
    }

private:
    using BlockFunction = u32 (*)(IopCpuState*, u8*, u32*);

    struct BlockEntry {
        u32 pc = 0;
        u32 generation = 0;
        u8 instruction_count = 0;
        std::array<u32, 32> words{};
        BlockFunction function = nullptr;
        bool control_flow = false;
        bool uses_ram = false;
        u32 ram_store_mask = 0;
        bool known = false;
    };

    struct DelayEntry {
        u32 instruction = 0;
        BlockFunction function = nullptr;
        bool uses_ram = false;
        bool known = false;
    };

    struct Page {
        void* address = nullptr;
        std::size_t used = 0;
    };

    BlockFunction compile_block(
        u32 pc,
        u32 code_page,
        const u32* words,
        u32 word_count,
        u32& compiled_instructions,
        bool& control_flow,
        bool& uses_ram,
        u32& ram_store_mask,
        u32& blocker_opcode,
        bool& blocker_is_delay_slot);
    void release_code_cache();

    std::vector<BlockEntry> entries_{32768};
    std::array<DelayEntry, 4096> delay_entries_{};
    std::vector<Page> pages_{};

    u64 block_compiled_count_ = 0;
    u64 block_executed_count_ = 0;
    u64 instruction_count_ = 0;
    u64 chain_count_ = 0;
    u64 guard_exit_count_ = 0;
    u64 code_store_exit_count_ = 0;
    u64 cache_flush_count_ = 0;
    u64 run_call_count_ = 0;
    u64 native_entry_attempt_count_ = 0;
    u64 native_entry_success_count_ = 0;
    u64 compile_failure_count_ = 0;
    u64 load_delay_entry_retire_count_ = 0;
    u64 native_delay_slot_count_ = 0;
    std::array<u64, 64> compile_stop_opcodes_{};
    std::array<u64, 64> delay_slot_stop_opcodes_{};
    std::array<u64, 5> entry_rejects_{};
    u64 native_residency_instruction_count_ = 0;
    u64 native_residency_max_ = 0;
    std::array<u64, 16> native_residency_histogram_{};
};

} // namespace ps2
