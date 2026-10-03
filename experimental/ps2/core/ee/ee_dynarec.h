#pragma once

#include "common/types.h"

#include <array>
#include <cstddef>
#include <vector>

namespace ps2 {

struct EeCpuState;

// Second-generation EE block recompiler.
//
// Unlike EeJit, this backend owns multi-instruction RAM-resident basic blocks,
// keeps a small hot GPR set in host registers for the lifetime of a block,
// performs guarded main-RAM fastmem, and links compiled successor blocks inside
// one dynarec dispatch. Device/event timing remains owned by Ps2System: execute()
// receives an exact retirement deadline and never runs past it.
class EeDynarec {
public:
    enum class ExitReason : u32 {
        None = 0,
        Deadline,
        Unsupported,
        Guard,
        CodeInvalidated,
        Cop0Write,
    };

    struct RunResult {
        u32 retired = 0;
        ExitReason reason = ExitReason::Unsupported;
    };

    EeDynarec();
    ~EeDynarec();
    EeDynarec(const EeDynarec&) = delete;
    EeDynarec& operator=(const EeDynarec&) = delete;

    void clear();

    RunResult execute(
        EeCpuState& state,
        u32 maximum_instructions,
        u8* ram_data,
        u32* page_generations,
        u8* code_page_tracked);

    [[nodiscard]] u64 compiled_blocks() const { return compiled_blocks_; }
    [[nodiscard]] u64 executed_blocks() const { return executed_blocks_; }
    [[nodiscard]] u64 executed_instructions() const {
        return executed_instructions_;
    }
    [[nodiscard]] u64 link_hits() const { return link_hits_; }
    [[nodiscard]] u64 link_misses() const { return link_misses_; }
    [[nodiscard]] u64 guard_exits() const { return guard_exits_; }
    [[nodiscard]] u64 code_invalidation_exits() const {
        return code_invalidation_exits_;
    }
    [[nodiscard]] u64 cop0_write_exits() const {
        return cop0_write_exits_;
    }
    [[nodiscard]] u64 fastmem_loads() const { return fastmem_loads_; }
    [[nodiscard]] u64 fastmem_stores() const { return fastmem_stores_; }
    [[nodiscard]] u64 register_cache_hits() const {
        return register_cache_hits_;
    }
    [[nodiscard]] u64 register_cache_flushes() const {
        return register_cache_flushes_;
    }
    [[nodiscard]] u64 cache_flushes() const { return cache_flushes_; }
    [[nodiscard]] u64 fused_static_jumps() const {
        return fused_static_jumps_;
    }
    [[nodiscard]] u64 fused_conditional_branches() const {
        return fused_conditional_branches_;
    }
    [[nodiscard]] u64 conditional_side_exits() const {
        return conditional_side_exits_;
    }
    [[nodiscard]] u64 dispatch_calls() const { return dispatch_calls_; }
    [[nodiscard]] u64 deadline_exits() const { return deadline_exits_; }
    [[nodiscard]] u64 unsupported_exits() const { return unsupported_exits_; }
    [[nodiscard]] const std::array<u64, 64>& unsupported_opcodes() const {
        return unsupported_opcodes_;
    }

private:
    using NativeFunction =
        u32 (*)(EeCpuState*, u8*, u32*, u8*);

    struct Block;

    struct SideExit {
        u32 retired = 0;
        u32 target_pc = 0;
        Block* link = nullptr;
        u32 link_generation = 0;
    };

    struct Block {
        u32 pc = 0;
        u32 page_generation = 0;
        u32 instruction_count = 0;
        u32 sequential_pc = 0;
        u32 taken_pc = 0;
        u32 fallthrough_pc = 0;
        u32 code_page = 0;
        u32 compile_budget = 0;
        std::array<u32, 4> source_pages{};
        std::array<u32, 4> source_generations{};
        u8 source_page_count = 0;
        std::array<SideExit, 8> side_exits{};
        u8 side_exit_count = 0;
        u32 fastmem_loads = 0;
        u32 fastmem_stores = 0;
        u32 cached_register_uses = 0;
        bool conditional_branch = false;
        bool branch_likely = false;
        bool control_flow = false;
        bool ends_with_cop0_write = false;
        NativeFunction function = nullptr;
        void* code_page_owner = nullptr;
        Block* sequential_link = nullptr;
        Block* taken_link = nullptr;
        Block* fallthrough_link = nullptr;
        u32 sequential_link_generation = 0;
        u32 taken_link_generation = 0;
        u32 fallthrough_link_generation = 0;
    };

    struct CodePage {
        void* address = nullptr;
        std::size_t used = 0;
    };

    Block* lookup_or_compile(
        u32 pc,
        u32 compile_limit,
        u8* ram_data,
        u32* page_generations,
        u8* code_page_tracked);
    Block* resolve_link(
        Block& source,
        u32 target_pc,
        u32 compile_limit,
        Block*& slot,
        u32& generation_slot,
        u8* ram_data,
        u32* page_generations,
        u8* code_page_tracked);
    void release_code_cache();

    std::vector<Block> blocks_;
    std::vector<CodePage> code_pages_;

    u64 compiled_blocks_ = 0;
    u64 executed_blocks_ = 0;
    u64 executed_instructions_ = 0;
    u64 link_hits_ = 0;
    u64 link_misses_ = 0;
    u64 guard_exits_ = 0;
    u64 code_invalidation_exits_ = 0;
    u64 cop0_write_exits_ = 0;
    u64 fastmem_loads_ = 0;
    u64 fastmem_stores_ = 0;
    u64 register_cache_hits_ = 0;
    u64 register_cache_flushes_ = 0;
    u64 cache_flushes_ = 0;
    u64 fused_static_jumps_ = 0;
    u64 fused_conditional_branches_ = 0;
    u64 conditional_side_exits_ = 0;
    u64 dispatch_calls_ = 0;
    u64 deadline_exits_ = 0;
    u64 unsupported_exits_ = 0;
    std::array<u64, 64> unsupported_opcodes_{};
};

} // namespace ps2
