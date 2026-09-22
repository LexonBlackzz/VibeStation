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
    void clear();

    [[nodiscard]] u64 compiled_count() const { return compiled_count_; }
    [[nodiscard]] u64 executed_count() const { return executed_count_; }

private:
    using Function = void (*)(EeCpuState*);
    struct Entry {
        u32 instruction = 0;
        Function function = nullptr;
        bool known = false;
    };
    struct Page {
        void* address = nullptr;
        std::size_t used = 0;
    };

    Function compile(u32 instruction);
    std::array<Entry, 4096> entries_{};
    std::vector<Page> pages_{};
    u64 compiled_count_ = 0;
    u64 executed_count_ = 0;
};

} // namespace ps2
