#pragma once

#include "common/types.h"

#include <array>
#include <cstddef>
#include <vector>

namespace ps2 {

struct EeCpuState;

// First-stage EE recompiler. Compiled functions implement one pure register
// instruction; EeCpu still owns fetch, exceptions, interrupts, PC and timing.
// This intentionally does not compile memory operations or control flow yet.
class EeJit {
public:
    EeJit() = default;
    ~EeJit();
    EeJit(const EeJit&) = delete;
    EeJit& operator=(const EeJit&) = delete;

    bool execute(EeCpuState& state, u32 instruction);
    u32 execute_block(
        EeCpuState& state,
        u32 pc,
        u32 page_generation,
        const u32* instructions,
        u32 instruction_count,
        u32 maximum_instructions,
        const u8* ram_data,
        u32* page_generations,
        bool& control_flow,
        u32 yield_pc = 0u,
        u8* scratchpad_data = nullptr);
    void clear();

    [[nodiscard]] u64 compiled_count() const { return compiled_count_; }
    [[nodiscard]] u64 executed_count() const { return executed_count_; }
    [[nodiscard]] u64 block_compiled_count() const {
        return block_compiled_count_;
    }
    [[nodiscard]] u64 block_executed_count() const {
        return block_executed_count_;
    }
    [[nodiscard]] u64 block_instruction_count() const {
        return block_instruction_count_;
    }
    [[nodiscard]] u64 block_fastmem_load_count() const {
        return block_fastmem_load_count_;
    }
    [[nodiscard]] u64 block_guard_bailout_count() const {
        return block_guard_bailout_count_;
    }
    [[nodiscard]] u64 block_fastmem_store_count() const {
        return block_fastmem_store_count_;
    }
    [[nodiscard]] u64 block_code_store_exit_count() const {
        return block_code_store_exit_count_;
    }
    [[nodiscard]] u64 successor_link_attempt_count() const {
        return successor_link_attempt_count_;
    }
    [[nodiscard]] u64 successor_link_hit_count() const {
        return successor_link_hit_count_;
    }
    [[nodiscard]] u64 cache_flush_count() const {
        return cache_flush_count_;
    }
    [[nodiscard]] u64 native_entry_attempt_count() const {
        return native_entry_attempt_count_;
    }
    [[nodiscard]] u64 native_entry_success_count() const {
        return native_entry_success_count_;
    }
    [[nodiscard]] u64 block_compile_failure_count() const {
        return block_compile_failure_count_;
    }
    [[nodiscard]] const std::array<u64, 64>& compile_stop_opcodes() const {
        return compile_stop_opcodes_;
    }
    [[nodiscard]] const std::array<u64, 64>& delay_slot_stop_opcodes() const {
        return delay_slot_stop_opcodes_;
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
    using Function = void (*)(EeCpuState*);
    using BlockFunction =
        u32 (*)(EeCpuState*, const u8*, u32*, u8*);
    struct Entry {
        u32 instruction = 0;
        Function function = nullptr;
        bool known = false;
    };
    struct BlockEntry {
        // Keep the fields touched at every native block boundary compact.
        // The 128-byte guest snapshot is cold on linked hot-path dispatch and
        // deliberately lives after the execution/link metadata.
        u32 pc = 0;
        u32 page_generation = 0;
        BlockFunction function = nullptr;
        BlockEntry* linked_successor = nullptr;
        u32 linked_successor_pc = 0;
        u32 linked_successor_generation = 0;
        u32 ram_load_mask = 0;
        u32 ram_store_mask = 0;
        u8 instruction_count = 0;
        bool control_flow = false;
        bool annul_capable = false;
        bool uses_ram = false;
        u8 guard_bail_streak = 0;
        u8 guard_skip_remaining = 0;
        bool known = false;
        std::array<u32, 32> words{};
    };
    struct Page {
        void* address = nullptr;
        std::size_t used = 0;
    };

    void release_code_cache();
    Function compile(u32 instruction);
    BlockFunction compile_block(
        u32 pc,
        const u32* instructions,
        u32 instruction_count,
        u32& compiled_instructions,
        bool& control_flow,
        bool& uses_ram,
        u32& blocker_opcode,
        bool& blocker_is_delay_slot);
    std::array<Entry, 4096> entries_{};
    std::vector<BlockEntry> block_entries_{32768};
    std::vector<Page> pages_{};
    u64 compiled_count_ = 0;
    u64 executed_count_ = 0;
    u64 block_compiled_count_ = 0;
    u64 block_executed_count_ = 0;
    u64 block_instruction_count_ = 0;
    u64 block_fastmem_load_count_ = 0;
    u64 block_guard_bailout_count_ = 0;
    u64 block_fastmem_store_count_ = 0;
    u64 block_code_store_exit_count_ = 0;
    u64 successor_link_attempt_count_ = 0;
    u64 successor_link_hit_count_ = 0;
    u64 cache_flush_count_ = 0;
    u64 native_entry_attempt_count_ = 0;
    u64 native_entry_success_count_ = 0;
    u64 block_compile_failure_count_ = 0;
    std::array<u64, 64> compile_stop_opcodes_{};
    std::array<u64, 64> delay_slot_stop_opcodes_{};
    u64 native_residency_instruction_count_ = 0;
    u64 native_residency_max_ = 0;
    std::array<u64, 16> native_residency_histogram_{};
};

} // namespace ps2
