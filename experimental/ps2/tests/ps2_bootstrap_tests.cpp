#include "core/ps2_system.h"

#include <bit>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) std::cerr << "FAIL: " << message << '\n';
    return condition;
}

bool step_at(ps2::Ps2System& system, ps2::u32 pc, ps2::u32 instruction) {
    if (!system.bus().write32(pc, instruction)) return false;
    system.ee().reset(pc);
    std::string error;
    return system.ee().step(error);
}

bool test_mmi_por_128() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x2000;
    const ps2::u32 op = (0x1Cu << 26) | (1u << 21) | (2u << 16) |
                        (3u << 11) | (0x12u << 6) | 0x29u;
    system.ee().reset(pc);
    system.ee().state().gpr[1] = {0x00FF00FF00FF00FFull, 0xAAAAAAAA55555555ull};
    system.ee().state().gpr[2] = {0xFF00FF00FF00FF00ull, 0x55555555AAAAAAAAull};
    bool ok = system.bus().write32(pc, op);
    std::string error;
    ok = expect(ok && system.ee().step(error), "POR execution failed") && ok;
    ok = expect(system.ee().state().gpr[3].lo == 0xFFFFFFFFFFFFFFFFull,
                "POR low half mismatch") && ok;
    ok = expect(system.ee().state().gpr[3].hi == 0xFFFFFFFFFFFFFFFFull,
                "POR high half mismatch") && ok;
    return ok;
}

bool test_unaligned_doubleword_merges() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x2000;
    constexpr ps2::u32 base = 0x1000;
    constexpr ps2::u64 mem = 0x8877665544332211ull;
    bool ok = system.bus().write64(base, mem);
    std::string error;

    ps2::u32 ldl = (0x1Au << 26) | (2u << 21) | (3u << 16);
    system.bus().write32(pc, ldl);
    system.ee().reset(pc);
    system.ee().state().gpr[2].lo = base;
    system.ee().state().gpr[3].lo = 0x0123456789ABCDEFull;
    ok = expect(system.ee().step(error), "LDL failed") && ok;
    ok = expect(system.ee().state().gpr[3].lo == 0x1123456789ABCDEFull,
                "LDL merge mismatch") && ok;

    ps2::u32 ldr = (0x1Bu << 26) | (2u << 21) | (3u << 16) | 7u;
    system.bus().write32(pc, ldr);
    system.ee().reset(pc);
    system.ee().state().gpr[2].lo = base;
    system.ee().state().gpr[3].lo = 0x0123456789ABCDEFull;
    ok = expect(system.ee().step(error), "LDR failed") && ok;
    ok = expect(system.ee().state().gpr[3].lo == 0x0123456789ABCD88ull,
                "LDR merge mismatch") && ok;
    return ok;
}

bool test_bootstrap_mmio() {
    ps2::Ps2System system;
    bool ok = true;

    ok = expect(system.bus().write16(0xBA000008u, 0x1234u), "DVE write failed") && ok;
    ps2::u16 h = 0;
    ok = expect(system.bus().read16(0xBA000008u, h) && h == 0x1234u,
                "DVE readback mismatch") && ok;

    ps2::u32 value = 0;
    ok = expect(system.bus().write32(0x10003000u, 9u), "GIF CTRL write failed") && ok;
    ok = expect(system.bus().read32(0x10003000u, value) && value == 9u,
                "GIF CTRL readback mismatch") && ok;
    ok = expect(system.bus().write64(0x10006000u, 0x1122334455667788ull) &&
                system.bus().write64(0x10006008u, 0x99AABBCCDDEEFF00ull),
                "GIF FIFO write failed") && ok;

    for (ps2::u32 i = 0; i < 4; ++i) {
        const ps2::u32 base = 0x10000000u + i * 0x800u;
        ok = expect(system.bus().write32(base + 0x20u, 0xBEEFu), "timer COMP write failed") && ok;
        ok = expect(system.bus().read32(base + 0x20u, value) && value == 0xBEEFu,
                    "timer COMP read mismatch") && ok;
        ok = expect(system.bus().write32(base + 0x30u, 0x1234u), "timer HOLD write failed") && ok;
    }

    ok = expect(system.bus().write32(0x10008080u, 0xFFFFFFFFu), "DMAC SADR write failed") && ok;
    ok = expect(system.bus().read32(0x10008080u, value) && value == 0x3FF0u,
                "DMAC SADR mask mismatch") && ok;
    ok = expect(system.bus().write32(0x1000E010u, 0x00010000u), "DMAC STAT mask toggle failed") && ok;
    ok = expect(system.bus().read32(0x1000E010u, value) && value == 0x00010000u,
                "DMAC STAT mask did not toggle on") && ok;
    ok = expect(system.bus().write32(0x1000E010u, 0x00010000u) &&
                system.bus().read32(0x1000E010u, value) && value == 0,
                "DMAC STAT mask did not toggle off") && ok;

    ok = expect(system.bus().write32(0x10003C10u, 1u), "VIF1 reset failed") && ok;
    ok = expect(system.bus().write32(0x10003C20u, 2u), "VIF1 ERR write failed") && ok;
    ok = expect(system.bus().read32(0x10003C20u, value) && value == 2u,
                "VIF1 ERR readback mismatch") && ok;
    ok = expect(system.bus().write64(0x10005000u, 0x0123456789ABCDEFull),
                "VIF1 FIFO write failed") && ok;

    ok = expect(system.bus().write32(0x10002010u, 0x40000000u), "IPU reset write failed") && ok;
    ok = expect(system.bus().read32(0x10002010u, value) && value == 0,
                "IPU reset did not clear CTRL") && ok;
    ok = expect(system.bus().write64(0x10007010u, 0xCAFEBABEDEADBEEFull),
                "IPU input FIFO write failed") && ok;

    return ok;
}

bool test_vu_mapping_and_cop2() {
    ps2::Ps2System system;
    bool ok = system.bus().write32(0x11004000u, 0xAABBCCDDu);
    ps2::u32 value = 0;
    ok = expect(ok && system.bus().read32(0x11005000u, value) && value == 0xAABBCCDDu,
                "VU0 data mirror mismatch") && ok;

    constexpr ps2::u32 pc = 0x2000;
    // CTC2 r2, FBRST then CFC2 r3, FBRST.
    const ps2::u32 ctc2 = (0x12u << 26) | (0x06u << 21) | (2u << 16) | (28u << 11);
    system.bus().write32(pc, ctc2);
    system.ee().reset(pc);
    system.ee().state().gpr[2].lo = 0x202u;
    std::string error;
    ok = expect(system.ee().step(error), "CTC2 FBRST failed") && ok;
    ok = expect(system.ee().state().vu_vi[28] == 0, "FBRST writable mask mismatch") && ok;

    const ps2::u32 cfc2 = (0x12u << 26) | (0x02u << 21) | (3u << 16) | (28u << 11);
    system.bus().write32(pc, cfc2);
    system.ee().reset(pc);
    ok = expect(system.ee().step(error), "CFC2 FBRST failed") && ok;
    ok = expect(system.ee().state().gpr[3].lo == 0, "CFC2 FBRST result mismatch") && ok;
    return ok;
}

bool test_fpu_accumulator() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x2000;
    const ps2::u32 adda = (0x11u << 26) | (0x10u << 21) | (1u << 16) | 0x18u;
    system.bus().write32(pc, adda);
    system.ee().reset(pc);
    system.ee().state().fpr[0] = std::bit_cast<ps2::u32>(1.5f);
    system.ee().state().fpr[1] = std::bit_cast<ps2::u32>(2.25f);
    std::string error;
    bool ok = expect(system.ee().step(error), "ADDA.S failed");
    ok = expect(std::bit_cast<float>(system.ee().state().fpu_acc) == 3.75f,
                "ADDA.S accumulator mismatch") && ok;
    return ok;
}

} // namespace

int main() {
    bool ok = true;
    ok = test_mmi_por_128() && ok;
    ok = test_unaligned_doubleword_merges() && ok;
    ok = test_bootstrap_mmio() && ok;
    ok = test_vu_mapping_and_cop2() && ok;
    ok = test_fpu_accumulator() && ok;
    if (!ok) return EXIT_FAILURE;
    std::cout << "VibeStation PS2 bootstrap tests passed.\n";
    return EXIT_SUCCESS;
}
