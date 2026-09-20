#include "core/ps2_system.h"

#include <charconv>
#include <cstdint>
#include <iostream>
#include <string>
#include <string_view>

namespace {

ps2::u64 parse_budget(const char* text, ps2::u64 fallback) {
    if (text == nullptr) {
        return fallback;
    }

    const std::string_view input(text);
    ps2::u64 value = 0;
    const auto result =
        std::from_chars(
            input.data(),
            input.data() + input.size(),
            value);
    if (result.ec != std::errc{} ||
        result.ptr != input.data() + input.size() ||
        value == 0) {
        return fallback;
    }
    return value;
}

void print_state(const ps2::Ps2System& system) {
    const auto& ee = system.ee().state();
    const auto& iop = system.iop().state();

    std::cout
        << "EE_PC=0x" << std::hex << std::uppercase << ee.pc
        << " EE_LAST_PC=0x" << ee.last_pc
        << " EE_LAST_OP=0x" << ee.last_instruction
        << std::dec
        << " EE_INSTRUCTIONS=" << ee.instructions_executed
        << '\n';

    std::cout
        << "IOP_PC=0x" << std::hex << std::uppercase << iop.pc
        << " IOP_LAST_PC=0x" << iop.last_pc
        << " IOP_LAST_OP=0x" << iop.last_instruction
        << " IOP_STATUS=0x" << iop.cop0[12]
        << " IOP_CAUSE=0x" << iop.cop0[13]
        << " IOP_EPC=0x" << iop.cop0[14]
        << std::dec
        << " IOP_INSTRUCTIONS=" << iop.instructions_executed
        << '\n';

    std::cout
        << "IOP_ISTAT=0x" << std::hex << std::uppercase
        << system.iop_intc().status()
        << " IOP_IMASK=0x" << system.iop_intc().mask()
        << " IOP_ICTRL=0x" << system.iop_intc().control()
        << std::dec
        << " SCHEDULER_TICK=" << system.scheduler().now()
        << '\n';
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr
            << "usage: vibestation_ps2_bios_trace <bios.bin> "
               "[ee-instruction-budget]\n";
        return 64;
    }

    constexpr ps2::u64 kDefaultBudget = 20'000'000;
    constexpr ps2::u64 kChunk = 100'000;

    const ps2::u64 budget =
        parse_budget(argc >= 3 ? argv[2] : nullptr, kDefaultBudget);

    ps2::Ps2System system;
    std::string error;

    if (!system.load_bios(argv[1], error)) {
        std::cerr << "BIOS_LOAD_ERROR=" << error << '\n';
        return 2;
    }
    if (!system.boot_bios(error)) {
        std::cerr << "BIOS_BOOT_ERROR=" << error << '\n';
        return 3;
    }

    ps2::u64 remaining = budget;
    while (remaining > 0 && !system.halted()) {
        const ps2::u64 request =
            remaining < kChunk ? remaining : kChunk;
        const ps2::u64 ran = system.run_ee(request, error);
        if (ran > remaining) {
            break;
        }
        remaining -= ran;

        if (ran == 0 && !system.halted()) {
            std::cerr << "TRACE_STALLED_WITHOUT_HALT\n";
            print_state(system);
            return 4;
        }
    }

    print_state(system);

    if (system.halted()) {
        std::cout << "HALT_REASON=" << system.halt_reason() << '\n';
        return 10;
    }

    std::cout
        << "TRACE_BUDGET_EXHAUSTED=" << budget << '\n';
    return 0;
}
