#include "core/ps2_system.h"

#include <cstdlib>
#include <iostream>
#include <vector>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool test_ram_little_endian() {
    ps2::Ps2System system;

    constexpr ps2::u32 address = 0x00001000u;
    if (!expect(system.bus().write32(address, 0x44332211u), "write32 failed")) {
        return false;
    }

    ps2::u8 byte = 0;
    return expect(system.bus().read8(address + 0, byte) && byte == 0x11u,
                  "byte 0 mismatch") &&
           expect(system.bus().read8(address + 1, byte) && byte == 0x22u,
                  "byte 1 mismatch") &&
           expect(system.bus().read8(address + 2, byte) && byte == 0x33u,
                  "byte 2 mismatch") &&
           expect(system.bus().read8(address + 3, byte) && byte == 0x44u,
                  "byte 3 mismatch");
}

bool test_ram_aliases() {
    ps2::Ps2System system;

    constexpr ps2::u32 physical = 0x00123450u;
    constexpr ps2::u32 kseg0 = 0x80123450u;
    constexpr ps2::u32 kseg1 = 0xA0123450u;
    constexpr ps2::u32 value = 0xCAFEBABEu;

    if (!expect(system.bus().write32(kseg0, value), "KSEG0 write failed")) {
        return false;
    }

    ps2::u32 readback = 0;
    return expect(system.bus().read32(physical, readback) && readback == value,
                  "physical alias mismatch") &&
           expect(system.bus().read32(kseg1, readback) && readback == value,
                  "KSEG1 alias mismatch");
}

bool test_ram_bounds() {
    ps2::Ps2System system;

    ps2::u32 value = 0;
    return expect(!system.bus().read32(static_cast<ps2::u32>(ps2::EeRam::kSize), value),
                  "out-of-range read unexpectedly succeeded") &&
           expect(!system.bus().write32(
                      static_cast<ps2::u32>(ps2::EeRam::kSize - 2), 0xFFFFFFFFu),
                  "cross-boundary write unexpectedly succeeded");
}

bool test_scheduler_ordering() {
    ps2::Scheduler scheduler;
    std::vector<ps2::EventType> fired;

    scheduler.schedule(ps2::EventType::Gs, 20);
    scheduler.schedule(ps2::EventType::Vif0, 10);
    scheduler.schedule(ps2::EventType::Gif, 10);

    scheduler.run_until(20, [&](const ps2::Scheduler::Event& event) {
        fired.push_back(event.type);
    });

    return expect(fired.size() == 3, "wrong scheduler event count") &&
           expect(fired[0] == ps2::EventType::Vif0,
                  "first same-timestamp event lost insertion order") &&
           expect(fired[1] == ps2::EventType::Gif,
                  "second same-timestamp event lost insertion order") &&
           expect(fired[2] == ps2::EventType::Gs,
                  "later event fired out of order") &&
           expect(scheduler.now() == 20, "scheduler time mismatch");
}

bool test_scheduler_cancel() {
    ps2::Scheduler scheduler;
    int fired = 0;

    const auto cancelled = scheduler.schedule(ps2::EventType::EeDmac, 4);
    scheduler.schedule(ps2::EventType::Gs, 8);
    scheduler.cancel(cancelled);

    scheduler.run_until(8, [&](const ps2::Scheduler::Event&) {
        ++fired;
    });

    return expect(fired == 1, "cancelled scheduler event fired");
}

bool test_ee_reset_state() {
    ps2::Ps2System system;
    system.reset(0x00100000u);

    const auto& state = system.ee().state();
    return expect(state.pc == 0x00100000u, "EE reset PC mismatch") &&
           expect(state.next_pc == 0x00100004u, "EE reset next PC mismatch") &&
           expect(state.gpr[0].lo == 0 && state.gpr[0].hi == 0,
                  "EE r0 reset state mismatch");
}

} // namespace

int main() {
    bool ok = true;
    ok = test_ram_little_endian() && ok;
    ok = test_ram_aliases() && ok;
    ok = test_ram_bounds() && ok;
    ok = test_scheduler_ordering() && ok;
    ok = test_scheduler_cancel() && ok;
    ok = test_ee_reset_state() && ok;

    if (!ok) {
        return EXIT_FAILURE;
    }

    std::cout << "VibeStation PS2 scaffold tests passed.\n";
    return EXIT_SUCCESS;
}
