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


bool test_gif_dma_engine() {
    ps2::Ps2System system;
    ps2::GifDma dma;
    dma.reset();
    std::string error;
    bool ok = true;

    constexpr ps2::u32 dmac_ctrl = 0x1000E000u;
    constexpr ps2::u32 dmac_stat = 0x1000E010u;
    constexpr ps2::u32 gif_chcr = 0x1000A000u;
    constexpr ps2::u32 gif_madr = 0x1000A010u;
    constexpr ps2::u32 gif_qwc = 0x1000A020u;
    constexpr ps2::u32 gif_tadr = 0x1000A030u;

    const ps2::u64 gif_tag =
        1ull | (1ull << 15) | (1ull << 60);
    constexpr ps2::u64 ad_descriptor = 0xEull;

    // Normal mode: two qwords from RAM become one GIF tag and one A+D write.
    ok = expect(system.bus().write64(0x4000u, gif_tag) &&
                system.bus().write64(0x4008u, ad_descriptor) &&
                system.bus().write64(0x4010u, 6ull) &&
                system.bus().write64(0x4018u, 0x00ull),
                "GIF DMA normal payload setup failed") && ok;
    ok = expect(system.bus().write32(dmac_ctrl, 1u) &&
                system.bus().write32(dmac_stat, 1u << 18) &&
                system.bus().write32(gif_madr, 0x4000u) &&
                system.bus().write32(gif_qwc, 2u) &&
                system.bus().write32(gif_chcr, 0x101u),
                "GIF DMA normal register setup failed") && ok;
    ok = expect(dma.service(system.bus(), system.gs_core(), error),
                "GIF DMA normal first service failed") && ok;
    ok = expect(dma.service(system.bus(), system.gs_core(), error),
                "GIF DMA normal second service failed") && ok;

    ps2::u32 value = 0;
    ok = expect(system.gs_core().register_value(0x00) == 6ull,
                "GIF DMA normal did not reach GS") && ok;
    ok = expect(system.bus().read32(gif_qwc, value) && value == 0,
                "GIF DMA normal QWC did not reach zero") && ok;
    ok = expect(system.bus().read32(gif_chcr, value) && (value & 0x100u) == 0,
                "GIF DMA normal STR did not clear") && ok;
    ok = expect(system.bus().read32(dmac_stat, value) && (value & (1u << 2)) != 0,
                "GIF DMA normal completion cause missing") && ok;
    ok = expect(system.bus().dmac_pending(),
                "GIF DMA normal completion did not assert DMAC pending") && ok;

    // Clear the completion cause, then run an END source-chain tag.
    ok = expect(system.bus().write32(dmac_stat, 1u << 2),
                "GIF DMA status acknowledge failed") && ok;
    const ps2::u32 dma_tag0 = 2u | (7u << 28);
    const ps2::u64 dma_tag_lo = static_cast<ps2::u64>(dma_tag0);
    ok = expect(system.bus().write64(0x5000u, dma_tag_lo) &&
                system.bus().write64(0x5008u, 0) &&
                system.bus().write64(0x5010u, gif_tag) &&
                system.bus().write64(0x5018u, ad_descriptor) &&
                system.bus().write64(0x5020u, 0xA5A5ull) &&
                system.bus().write64(0x5028u, 0x4Cull),
                "GIF DMA chain payload setup failed") && ok;
    ok = expect(system.bus().write32(gif_tadr, 0x5000u) &&
                system.bus().write32(gif_qwc, 0u) &&
                system.bus().write32(gif_chcr, 0x105u),
                "GIF DMA chain register setup failed") && ok;
    ok = expect(dma.service(system.bus(), system.gs_core(), error),
                "GIF DMA chain first service failed") && ok;
    ok = expect(dma.service(system.bus(), system.gs_core(), error),
                "GIF DMA chain second service failed") && ok;
    ok = expect(system.gs_core().register_value(0x4C) == 0xA5A5ull,
                "GIF DMA END chain did not reach GS") && ok;
    ok = expect(system.bus().read32(gif_chcr, value) && (value & 0x100u) == 0,
                "GIF DMA END chain STR did not clear") && ok;
    ok = expect(system.bus().dmac_pending(),
                "GIF DMA END chain did not assert DMAC pending") && ok;
    return ok;
}


bool test_gs_vram_swizzle_addresses() {
    ps2::GsVram vram;
    bool ok = true;

    ok = expect(vram.write_pixel(0, 8, 0, 0, 1, 0x44332211u),
                "PSMCT32 swizzle write failed") && ok;
    ok = expect(vram.byte_at(256) == 0x11u &&
                vram.byte_at(257) == 0x22u &&
                vram.byte_at(258) == 0x33u &&
                vram.byte_at(259) == 0x44u,
                "PSMCT32 x=8 block address mismatch") && ok;

    vram.reset();
    ok = expect(vram.write_pixel(2, 16, 0, 0, 1, 0xBEEFu),
                "PSMCT16 swizzle write failed") && ok;
    ok = expect(vram.byte_at(512) == 0xEFu &&
                vram.byte_at(513) == 0xBEu,
                "PSMCT16 x=16 block address mismatch") && ok;

    vram.reset();
    ok = expect(vram.write_pixel(10, 32, 0, 0, 1, 0x1234u),
                "PSMCT16S swizzle write failed") && ok;
    ok = expect(vram.byte_at(4096) == 0x34u &&
                vram.byte_at(4097) == 0x12u,
                "PSMCT16S x=32 block address mismatch") && ok;

    return ok;
}

bool test_gs_host_to_local_image_transfer() {
    ps2::GsCore gs;
    gs.reset();

    auto ad_packet = [&](ps2::u32 address, ps2::u64 value) {
        const ps2::u64 tag =
            1ull | (1ull << 15) | (1ull << 60);
        gs.write_gif_qword(tag, 0xEull);
        gs.write_gif_qword(value, address);
    };

    bool ok = true;

    // 32-bit upload: 4 pixels, one IMAGE qword.
    const ps2::u64 blit32 =
        (static_cast<ps2::u64>(1u) << 48); // DBP=0, DBW=1, DPSM=PSMCT32
    const ps2::u64 pos32 =
        (static_cast<ps2::u64>(8u) << 32); // DSAX=8, DSAY=0
    const ps2::u64 reg32 =
        4ull | (1ull << 32); // 4x1
    ad_packet(0x50, blit32);
    ad_packet(0x51, pos32);
    ad_packet(0x52, reg32);
    ad_packet(0x53, 0);

    const ps2::u64 image_tag32 =
        1ull | (1ull << 15) | (2ull << 58);
    gs.write_gif_qword(image_tag32, 0);
    gs.write_gif_qword(
        0x2222222211111111ull,
        0x4444444433333333ull);

    ok = expect(!gs.transfer_active(),
                "PSMCT32 transfer did not complete") && ok;
    ok = expect(gs.vram().read_pixel(0, 8, 0, 0, 1) == 0x11111111u &&
                gs.vram().read_pixel(0, 9, 0, 0, 1) == 0x22222222u &&
                gs.vram().read_pixel(0, 10, 0, 0, 1) == 0x33333333u &&
                gs.vram().read_pixel(0, 11, 0, 0, 1) == 0x44444444u,
                "PSMCT32 IMAGE upload pixel mismatch") && ok;

    // 24-bit upload: six tightly-packed pixels span two GIF qwords.
    gs.reset();
    const ps2::u64 blit24 =
        (static_cast<ps2::u64>(1u) << 48) |
        (static_cast<ps2::u64>(1u) << 56);
    const ps2::u64 reg24 = 6ull | (1ull << 32);
    ad_packet(0x50, blit24);
    ad_packet(0x51, 0);
    ad_packet(0x52, reg24);
    ad_packet(0x53, 0);

    const ps2::u64 image_tag24 =
        2ull | (1ull << 15) | (2ull << 58);
    gs.write_gif_qword(image_tag24, 0);

    // Pixel byte stream:
    // 030201 060504 090807 0C0B0A 0F0E0D 121110, then qword padding.
    gs.write_gif_qword(
        0x0807060504030201ull,
        0x100F0E0D0C0B0A09ull);
    gs.write_gif_qword(
        0x0000000000001211ull,
        0);

    ok = expect(!gs.transfer_active(),
                "PSMCT24 transfer did not complete") && ok;
    ok = expect(gs.vram().read_pixel(1, 0, 0, 0, 1) == 0x030201u &&
                gs.vram().read_pixel(1, 1, 0, 0, 1) == 0x060504u &&
                gs.vram().read_pixel(1, 2, 0, 0, 1) == 0x090807u &&
                gs.vram().read_pixel(1, 3, 0, 0, 1) == 0x0C0B0Au &&
                gs.vram().read_pixel(1, 4, 0, 0, 1) == 0x0F0E0Du &&
                gs.vram().read_pixel(1, 5, 0, 0, 1) == 0x121110u,
                "PSMCT24 qword-carry upload mismatch") && ok;

    const auto& stats = gs.stats();
    ok = expect(stats.host_to_local_transfers == 1 &&
                stats.host_to_local_pixels == 6 &&
                stats.image_qwords == 2,
                "host-to-local transfer statistics mismatch") && ok;
    ok = expect((gs.register_value(0x53) & 0x3u) == 3u,
                "completed transfer did not deactivate TRXDIR") && ok;

    return ok;
}


bool test_gs_untextured_rasterization() {
    auto ad = [](ps2::GsCore& gs, ps2::u32 address, ps2::u64 value) {
        const ps2::u64 tag =
            1ull | (1ull << 15) | (1ull << 60);
        gs.write_gif_qword(tag, 0xEull);
        gs.write_gif_qword(value, address);
    };

    auto xyz = [](ps2::u32 x_fp, ps2::u32 y_fp, ps2::u32 z = 0) {
        return static_cast<ps2::u64>(x_fp & 0xFFFFu) |
               (static_cast<ps2::u64>(y_fp & 0xFFFFu) << 16) |
               (static_cast<ps2::u64>(z) << 32);
    };

    const ps2::u64 frame =
        static_cast<ps2::u64>(1u) << 16; // FBP=0, FBW=1, PSMCT32, FBMSK=0.
    const ps2::u64 scissor =
        (static_cast<ps2::u64>(31u) << 16) |
        (static_cast<ps2::u64>(31u) << 48);

    bool ok = true;

    // Flat untextured 2x2 sprite.
    {
        ps2::GsCore gs;
        gs.reset();
        ad(gs, 0x1A, 1u); // PRMODECONT.AC = use PRIM attributes.
        ad(gs, 0x18, 0u); // XYOFFSET_1
        ad(gs, 0x40, scissor);
        ad(gs, 0x47, 0u); // TEST_1 disabled
        ad(gs, 0x4C, frame);
        ad(gs, 0x00, 6u); // sprite
        ad(gs, 0x01, 0x44332211u);
        ad(gs, 0x05, xyz(0, 0));
        ad(gs, 0x05, xyz(32, 32));

        ok = expect(gs.vram().read_pixel(0, 0, 0, 0, 1) == 0x44332211u &&
                    gs.vram().read_pixel(0, 1, 0, 0, 1) == 0x44332211u &&
                    gs.vram().read_pixel(0, 0, 1, 0, 1) == 0x44332211u &&
                    gs.vram().read_pixel(0, 1, 1, 0, 1) == 0x44332211u,
                    "GS sprite raster pixels mismatch") && ok;
        ok = expect(gs.vram().read_pixel(0, 2, 2, 0, 1) == 0,
                    "GS sprite raster overran rectangle") && ok;
        ok = expect(gs.stats().raster_draws == 1 &&
                    gs.stats().raster_pixels == 4 &&
                    gs.stats().skipped_raster_draws == 0,
                    "GS sprite raster statistics mismatch") && ok;
    }

    // Flat untextured triangle list.
    {
        ps2::GsCore gs;
        gs.reset();
        ad(gs, 0x1A, 1u);
        ad(gs, 0x18, 0u);
        ad(gs, 0x40, scissor);
        ad(gs, 0x47, 0u);
        ad(gs, 0x4C, frame);
        ad(gs, 0x00, 3u); // triangle list
        ad(gs, 0x01, 0x88776655u);
        ad(gs, 0x05, xyz(0, 0));
        ad(gs, 0x05, xyz(48, 0));
        ad(gs, 0x05, xyz(0, 48));

        ok = expect(gs.vram().read_pixel(0, 0, 0, 0, 1) == 0x88776655u,
                    "GS triangle failed to cover interior pixel") && ok;
        ok = expect(gs.vram().read_pixel(0, 2, 2, 0, 1) == 0,
                    "GS triangle covered exterior pixel") && ok;
        ok = expect(gs.stats().raster_draws == 1 &&
                    gs.stats().raster_pixels != 0,
                    "GS triangle raster statistics mismatch") && ok;
    }

    // Textured draw must be observable as skipped, not silently approximated.
    {
        ps2::GsCore gs;
        gs.reset();
        ad(gs, 0x1A, 1u);
        ad(gs, 0x18, 0u);
        ad(gs, 0x40, scissor);
        ad(gs, 0x47, 0u);
        ad(gs, 0x4C, frame);
        ad(gs, 0x00, 6u | (1u << 4)); // sprite + TME
        ad(gs, 0x01, 0xFFFFFFFFu);
        ad(gs, 0x05, xyz(0, 0));
        ad(gs, 0x05, xyz(32, 32));

        ok = expect(gs.stats().raster_draws == 0 &&
                    gs.stats().skipped_raster_draws == 1,
                    "unsupported textured draw was not skipped") && ok;
        ok = expect(gs.vram().read_pixel(0, 0, 0, 0, 1) == 0,
                    "unsupported textured draw modified VRAM") && ok;
    }

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
    ok = test_gif_dma_engine() && ok;
    ok = test_gs_vram_swizzle_addresses() && ok;
    ok = test_gs_host_to_local_image_transfer() && ok;
    ok = test_gs_untextured_rasterization() && ok;
    ok = test_fpu_accumulator() && ok;
    if (!ok) return EXIT_FAILURE;
    std::cout << "VibeStation PS2 bootstrap tests passed.\n";
    return EXIT_SUCCESS;
}
