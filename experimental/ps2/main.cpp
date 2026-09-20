#include "core/ps2_system.h"

#include <iomanip>
#include <iostream>

int main() {
    ps2::Ps2System system;

    constexpr ps2::u32 kProbeAddress = 0x00100000u;
    constexpr ps2::u32 kProbeValue = 0x12345678u;

    if (!system.bus().write32(kProbeAddress, kProbeValue)) {
        std::cerr << "PS2 smoke test failed: EE RAM write failed.\n";
        return 1;
    }

    ps2::u32 value = 0;
    if (!system.bus().read32(kProbeAddress, value) || value != kProbeValue) {
        std::cerr << "PS2 smoke test failed: EE RAM readback mismatch.\n";
        return 1;
    }

    std::cout << "VibeStation PS2 Experimental Core\n\n";
    std::cout << "EE RAM:       " << (ps2::EeRam::kSize / (1024u * 1024u)) << " MiB\n";
    std::cout << "EE CPU:       initialized\n";
    std::cout << "Scheduler:    initialized\n";
    std::cout << "RAM probe:    0x" << std::hex << std::uppercase
              << std::setw(8) << std::setfill('0') << value << std::dec << "\n\n";
    std::cout << "PS2 core smoke test passed.\n";
    return 0;
}
