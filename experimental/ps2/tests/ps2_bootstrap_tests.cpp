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

bool test_mmi_padduw() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x2000;
    const ps2::u32 op = (0x1Cu << 26) | (1u << 21) | (2u << 16) |
                        (3u << 11) | (0x10u << 6) | 0x28u;
    system.ee().reset(pc);
    system.ee().state().gpr[1] = {0xFFFFFFFF00000001ull, 0x800000007FFFFFFFull};
    system.ee().state().gpr[2] = {0x0000000200000002ull, 0x8000000000000001ull};
    bool ok = system.bus().write32(pc, op);
    std::string error;
    ok = expect(ok && system.ee().step(error), "PADDUW execution failed") && ok;
    ok = expect(system.ee().state().gpr[3].lo == 0xFFFFFFFF00000003ull,
                "PADDUW low lanes mismatch") && ok;
    ok = expect(system.ee().state().gpr[3].hi == 0xFFFFFFFF80000000ull,
                "PADDUW high lanes mismatch") && ok;
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

bool test_ee_intc_register_semantics() {
    ps2::Ps2System system;
    ps2::u32 value = 0;
    bool ok = expect(system.bus().write32(0x1000F010u, 0x0003u), "INTC_MASK initial toggle failed");
    ok = expect(system.bus().read32(0x1000F010u, value) && value == 0x0003u,
                "INTC_MASK initial value mismatch") && ok;
    ok = expect(system.bus().write32(0x1000F010u, 0x0001u) &&
                system.bus().read32(0x1000F010u, value) && value == 0x0002u,
                "INTC_MASK XOR semantics mismatch") && ok;

    ok = expect(system.iop_bus().write32(0x1F801450u, 0x2u), "IOP SBUS INTC raise failed") && ok;
    ok = expect(system.bus().read32(0x1000F000u, value) && value == 0x0002u,
                "INTC_STAT raise mismatch") && ok;
    ok = expect(system.bus().write32(0x1000F000u, 0x0002u) &&
                system.bus().read32(0x1000F000u, value) && value == 0,
                "INTC_STAT write-one-to-clear mismatch") && ok;
    ok = expect(system.iop_bus().write32(0x1F801450u, 0x2u), "IOP SBUS IRQ write failed") && ok;
    ok = expect(system.bus().read32(0x1000F000u, value) && (value & 0x2u) != 0,
                "IOP SBUS write did not raise EE INTC bit 1") && ok;
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

bool test_ee_intc_cpu_exception() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x2800;
    bool ok = expect(system.bus().write32(pc, 0), "INTC test NOP write failed");
    ok = expect(system.bus().write32(0x1000F010u, 1u << 1), "INTC test mask enable failed") && ok;
    ok = expect(system.iop_bus().write32(0x1F801450u, 0x2u), "INTC CPU source raise failed") && ok;
    system.ee().reset(pc);
    system.ee().state().cop0[12] = 0x00010401u; // EIE | IP2 mask | IE
    std::string error;
    ok = expect(system.ee().step(error), "INTC CPU exception failed") && ok;
    ok = expect(system.ee().state().pc == 0x80000200u, "INTC vector mismatch") && ok;
    ok = expect(system.ee().state().cop0[14] == pc, "INTC EPC mismatch") && ok;
    ok = expect((system.ee().state().cop0[13] & 0x0000047Cu) == 0x00000400u,
                "INTC Cause mismatch") && ok;
    ok = expect((system.ee().state().cop0[12] & 0x2u) != 0, "INTC did not set EXL") && ok;
    return ok;
}

bool test_ee_di_ei_privilege_gate() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x2C00;
    // EI / DI COP0 functions.
    const ps2::u32 ei = 0x42000039u;
    const ps2::u32 di = 0x42000038u;
    std::string error;
    bool ok = expect(system.bus().write32(pc, ei) && system.bus().write32(pc + 4u, di),
                     "EI/DI test code write failed");

    system.ee().reset(pc);
    system.ee().state().cop0[12] = 0x10u; // user KSU, _EDI=EXL=ERL=0
    ok = expect(system.ee().step(error), "gated EI execution failed") && ok;
    ok = expect((system.ee().state().cop0[12] & 0x10000u) == 0,
                "EI ignored privilege gate") && ok;

    system.ee().reset(pc + 4u);
    system.ee().state().cop0[12] = 0x10010u; // user KSU, EIE=1, _EDI=0
    ok = expect(system.ee().step(error), "gated DI execution failed") && ok;
    ok = expect((system.ee().state().cop0[12] & 0x10000u) != 0,
                "DI ignored privilege gate") && ok;

    system.ee().reset(pc);
    system.ee().state().cop0[12] = 0; // kernel KSU
    ok = expect(system.ee().step(error), "kernel EI execution failed") && ok;
    ok = expect((system.ee().state().cop0[12] & 0x10000u) != 0,
                "kernel EI did not set EIE") && ok;
    return ok;
}

bool test_syscall_exception() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x2000;
    bool ok = expect(system.bus().write32(pc, 0x0000000Cu), "SYSCALL write failed");
    system.ee().reset(pc);
    system.ee().state().cop0[12] = 0; // BEV=ERL=EXL=0
    std::string error;
    ok = expect(system.ee().step(error), "SYSCALL execution failed") && ok;
    ok = expect(system.ee().state().pc == 0x80000180u, "SYSCALL vector mismatch") && ok;
    ok = expect(system.ee().state().cop0[14] == pc, "SYSCALL EPC mismatch") && ok;
    ok = expect((system.ee().state().cop0[13] & 0x7Cu) == 0x20u, "SYSCALL cause mismatch") && ok;
    ok = expect((system.ee().state().cop0[13] & 0x80000000u) == 0, "SYSCALL BD set unexpectedly") && ok;
    ok = expect((system.ee().state().cop0[12] & 0x2u) != 0, "SYSCALL did not set EXL") && ok;

    ok = expect(system.bus().write32(0x80000180u, 0x42000018u), "ERET write failed") && ok;
    ok = expect(system.ee().step(error), "ERET execution failed") && ok;
    ok = expect(system.ee().state().pc == pc, "ERET return PC mismatch") && ok;
    ok = expect((system.ee().state().cop0[12] & 0x2u) == 0, "ERET did not clear EXL") && ok;
    return ok;
}

bool test_syscall_delay_slot_exception() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x2400;
    // BEQ r0,r0,+1 followed by SYSCALL in the mandatory delay slot.
    bool ok = expect(system.bus().write32(pc, 0x10000001u), "delay branch write failed");
    ok = expect(system.bus().write32(pc + 4u, 0x0000000Cu), "delay SYSCALL write failed") && ok;
    system.ee().reset(pc);
    system.ee().state().cop0[12] = 0;
    std::string error;
    ok = expect(system.ee().step(error), "delay branch execution failed") && ok;
    ok = expect(system.ee().step(error), "delay-slot SYSCALL failed") && ok;
    ok = expect(system.ee().state().pc == 0x80000180u, "delay SYSCALL vector mismatch") && ok;
    ok = expect(system.ee().state().cop0[14] == pc, "delay SYSCALL EPC mismatch") && ok;
    ok = expect((system.ee().state().cop0[13] & 0x80000000u) != 0, "delay SYSCALL BD missing") && ok;
    return ok;
}


bool test_video_timing_vblank_irqs() {
    ps2::VideoTiming timing;
    ps2::EeHw ee_hw;
    ps2::IopIntc iop_intc;
    timing.reset();
    ee_hw.reset();
    iop_intc.reset();

    bool ok = true;
    ps2::u32 value = 0;
    timing.tick(ps2::VideoTiming::kNtscRenderCycles - 1u, ee_hw, iop_intc);
    ok = expect(ee_hw.read32(0x1000F000u, value) && value == 0,
                "VBlank start fired early") && ok;
    ok = expect(iop_intc.status() == 0, "IOP VBlank start fired early") && ok;

    timing.tick(1, ee_hw, iop_intc);
    ok = expect(ee_hw.read32(0x1000F000u, value) && (value & (1u << 2)) != 0,
                "EE VBlank-start INTC source missing") && ok;
    ok = expect((iop_intc.status() & (1u << 0)) != 0,
                "IOP VBlank-start interrupt missing") && ok;
    ok = expect(timing.phase() == ps2::VideoTiming::Phase::VBlank,
                "video timing did not enter VBlank") && ok;

    timing.tick(ps2::VideoTiming::kNtscVBlankCycles, ee_hw, iop_intc);
    ok = expect(ee_hw.read32(0x1000F000u, value) && (value & (1u << 3)) != 0,
                "EE VBlank-end INTC source missing") && ok;
    ok = expect((iop_intc.status() & (1u << 11)) != 0,
                "IOP VBlank-end interrupt missing") && ok;
    ok = expect(timing.phase() == ps2::VideoTiming::Phase::Render,
                "video timing did not return to render") && ok;
    ok = expect(timing.fields_started() == 1,
                "video timing field counter mismatch") && ok;
    return ok;
}


bool test_gif_packet_decode() {
    ps2::Ps2System system;
    bool ok = true;

    constexpr ps2::u32 fifo = ps2::GsCore::kGifFifoBase;

    // PACKED A+D packet: PRIM then FRAME_1.
    const ps2::u64 packed_tag_lo =
        2ull | (1ull << 15) | (1ull << 60);
    ok = expect(system.bus().write64(fifo, packed_tag_lo) &&
                system.bus().write64(fifo + 8u, 0xEull),
                "GIF packed tag write failed") && ok;
    ok = expect(system.bus().write64(fifo, 6ull) &&
                system.bus().write64(fifo + 8u, 0x00ull),
                "GIF PRIM A+D write failed") && ok;
    ok = expect(system.bus().write64(fifo, 0x1122334455667788ull) &&
                system.bus().write64(fifo + 8u, 0x4Cull),
                "GIF FRAME_1 A+D write failed") && ok;
    ok = expect(system.gs_core().register_value(0x00) == 6ull,
                "GIF PRIM register mismatch") && ok;
    ok = expect(system.gs_core().register_value(0x4C) == 0x1122334455667788ull,
                "GIF FRAME_1 register mismatch") && ok;

    // REGLIST packet with PRIM + XYZ2 in a single qword.
    const ps2::u64 reglist_tag_lo =
        1ull | (1ull << 15) | (1ull << 58) | (2ull << 60);
    ok = expect(system.bus().write64(fifo, reglist_tag_lo) &&
                system.bus().write64(fifo + 8u, 0x50ull),
                "GIF reglist tag write failed") && ok;
    const ps2::u64 xyz = 0x001234560078009Aull;
    ok = expect(system.bus().write64(fifo, 3ull) &&
                system.bus().write64(fifo + 8u, xyz),
                "GIF reglist payload write failed") && ok;
    ok = expect(system.gs_core().register_value(0x00) == 3ull,
                "GIF reglist PRIM mismatch") && ok;
    ok = expect(system.gs_core().register_value(0x05) == xyz,
                "GIF reglist XYZ2 mismatch") && ok;

    // IMAGE packet accounting.
    const ps2::u64 image_tag_lo =
        1ull | (1ull << 15) | (2ull << 58);
    ok = expect(system.bus().write64(fifo, image_tag_lo) &&
                system.bus().write64(fifo + 8u, 0),
                "GIF image tag write failed") && ok;
    ok = expect(system.bus().write64(fifo, 0x0123456789ABCDEFull) &&
                system.bus().write64(fifo + 8u, 0xFEDCBA9876543210ull),
                "GIF image payload write failed") && ok;

    const auto& stats = system.gs_core().stats();
    ok = expect(stats.gif_tags == 3, "GIF tag count mismatch") && ok;
    ok = expect(stats.packed_writes == 2, "GIF packed write count mismatch") && ok;
    ok = expect(stats.reglist_writes == 2, "GIF reglist write count mismatch") && ok;
    ok = expect(stats.image_qwords == 1, "GIF image qword count mismatch") && ok;
    ok = expect(stats.vertices == 1, "GIF vertex kick count mismatch") && ok;
    ok = expect(stats.eop_packets == 3, "GIF EOP count mismatch") && ok;
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
    ok = test_mmi_padduw() && ok;
    ok = test_unaligned_doubleword_merges() && ok;
    ok = test_bootstrap_mmio() && ok;
    ok = test_ee_intc_register_semantics() && ok;
    ok = test_vu_mapping_and_cop2() && ok;
    ok = test_ee_intc_cpu_exception() && ok;
    ok = test_ee_di_ei_privilege_gate() && ok;
    ok = test_syscall_exception() && ok;
    ok = test_syscall_delay_slot_exception() && ok;
    ok = test_video_timing_vblank_irqs() && ok;
    ok = test_gif_packet_decode() && ok;
    ok = test_fpu_accumulator() && ok;
    if (!ok) return EXIT_FAILURE;
    std::cout << "VibeStation PS2 bootstrap tests passed.\n";
    return EXIT_SUCCESS;
}
