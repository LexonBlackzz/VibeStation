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
        u32 yield_pc = 0u);
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
    [[nodiscard]] u64 cache_flush_count() const {
        return cache_flush_count_;
    }

private:
    using Function = void (*)(EeCpuState*);
    using BlockFunction = u32 (*)(EeCpuState*, const u8*, u32*);
    struct Entry {
        u32 instruction = 0;
        Function function = nullptr;
        bool known = false;
    };
    struct BlockEntry {
        u32 pc = 0;
        u32 page_generation = 0;
        u8 instruction_count = 0;
        BlockFunction function = nullptr;
        bool control_flow = false;
        bool uses_ram = false;
        u32 ram_load_mask = 0;
        u32 ram_store_mask = 0;
        bool known = false;
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
        bool& uses_ram);
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
    u64 cache_flush_count_ = 0;
};

} // namespace ps2
