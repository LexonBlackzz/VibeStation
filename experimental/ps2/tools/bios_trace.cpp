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

    std::cout << "EE_GPR";
    for (ps2::u32 index = 0; index < ee.gpr.size(); ++index) {
        std::cout
            << " R" << std::dec << index << "=0x"
            << std::hex << std::uppercase << ee.gpr[index].lo;
    }
    std::cout << std::dec << '\n';

    for (ps2::u32 channel = 9u; channel <= 10u; ++channel) {
        const ps2::u32 base = 0x1F801490u + channel * 0x10u;
        ps2::u32 madr = 0;
        ps2::u32 bcr = 0;
        ps2::u32 chcr = 0;
        ps2::u32 tadr = 0;
        (void)system.iop_bus().read32(base, madr);
        (void)system.iop_bus().read32(base + 4u, bcr);
        (void)system.iop_bus().read32(base + 8u, chcr);
        (void)system.iop_bus().read32(base + 12u, tadr);
        std::cout
            << "IOP_DMAC" << channel
            << "_MADR=0x" << std::hex << std::uppercase << madr
            << " BCR=0x" << bcr
            << " CHCR=0x" << chcr
            << " TADR=0x" << tadr
            << std::dec << '\n';

        if (channel == 9u && tadr != 0u) {
            ps2::u32 tag[4]{};
            for (ps2::u32 index = 0; index < 4u; ++index) {
                (void)system.iop_bus().read32(
                    tadr + index * 4u,
                    tag[index]);
            }
            std::cout
                << "IOP_SIF0_TAG=0x" << std::hex << std::uppercase
                << tag[0] << ",0x" << tag[1]
                << ",0x" << tag[2] << ",0x" << tag[3]
                << " SOURCE=0x" << (tag[0] & 0x00FFFFFFu)
                << std::dec << '\n';
        }
    }

    auto print_code = [&](const char* label, ps2::u32 center) {
        std::cout << label;
        const ps2::u32 code_base = (center - 32u) & ~3u;
        for (ps2::u32 offset = 0; offset < 68u; offset += 4u) {
            ps2::u32 instruction = 0;
            const ps2::u32 address = code_base + offset;
            if (system.bus().read32(address, instruction)) {
                std::cout
                    << " [0x" << std::hex << std::uppercase << address
                    << "]=0x" << instruction;
            }
        }
        std::cout << std::dec << '\n';
    };
    print_code("EE_CODE", ee.pc);
    print_code("EE_RA_CODE", static_cast<ps2::u32>(ee.gpr[31].lo));
    print_code("EE_EPC_CODE", ee.cop0[14]);

    std::cout
        << "EE_STATUS=0x" << std::hex << std::uppercase << ee.cop0[12]
        << " EE_CAUSE=0x" << ee.cop0[13]
        << " EE_EPC=0x" << ee.cop0[14]
        << " EE_BADVADDR=0x" << ee.cop0[8]
        << " EE_COUNT=0x" << ee.cop0[9]
        << " EE_COMPARE=0x" << ee.cop0[11]
        << std::dec << '\n';

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

    std::cout << "IOP_GPR";
    for (ps2::u32 index = 0; index < iop.gpr.size(); ++index) {
        std::cout
            << " R" << std::dec << index << "=0x"
            << std::hex << std::uppercase << iop.gpr[index];
    }
    std::cout << std::dec << '\n';

    auto print_iop_code = [&](const char* label, ps2::u32 center) {
        std::cout << label;
        const ps2::u32 code_base = (center - 32u) & ~3u;
        for (ps2::u32 offset = 0; offset < 68u; offset += 4u) {
            ps2::u32 instruction = 0;
            const ps2::u32 address = code_base + offset;
            if (system.iop_bus().read32(address, instruction)) {
                std::cout
                    << " [0x" << std::hex << std::uppercase << address
                    << "]=0x" << instruction;
            }
        }
        std::cout << std::dec << '\n';
    };
    print_iop_code("IOP_CODE", iop.pc);
    print_iop_code("IOP_RA_CODE", iop.gpr[31]);

    std::cout
        << "IOP_ISTAT=0x" << std::hex << std::uppercase
        << system.iop_intc().status()
        << " IOP_IMASK=0x" << system.iop_intc().mask()
        << " IOP_ICTRL=0x" << system.iop_intc().control()
        << std::dec
        << " SCHEDULER_TICK=" << system.scheduler().now()
        << '\n';

    if (system.iop_halted()) {
        std::cout
            << "IOP_HALTED=1 IOP_HALT_REASON="
            << system.iop().halt_reason()
            << '\n';
    } else {
        std::cout << "IOP_HALTED=0\n";
    }

    for (ps2::u32 channel = 0; channel < 2u; ++channel) {
        ps2::u32 vif_stat = 0;
        ps2::u32 vif_chcr = 0;
        ps2::u32 vif_qwc = 0;
        const ps2::u32 vif_base = 0x10003800u + channel * 0x400u;
        const ps2::u32 dma_base = 0x10008000u + channel * 0x1000u;
        (void)system.bus().read32(vif_base, vif_stat);
        (void)system.bus().read32(dma_base, vif_chcr);
        (void)system.bus().read32(dma_base + 0x20u, vif_qwc);
        std::cout
            << "VIF" << channel << "_STAT=0x"
            << std::hex << std::uppercase << vif_stat
            << " VIF" << channel << "_CHCR=0x" << vif_chcr
            << " VIF" << channel << "_QWC=0x" << vif_qwc
            << std::dec << '\n';
    }

    constexpr ps2::u32 kDmacChannels[] = {
        0x10008000u, 0x10009000u, 0x1000A000u, 0x1000B000u,
        0x1000B400u, 0x1000C000u, 0x1000C400u, 0x1000C800u,
        0x1000D000u, 0x1000D400u,
    };
    for (ps2::u32 channel = 0; channel < 10u; ++channel) {
        ps2::u32 chcr = 0;
        ps2::u32 madr = 0;
        ps2::u32 qwc = 0;
        ps2::u32 tadr = 0;
        const ps2::u32 base = kDmacChannels[channel];
        (void)system.bus().read32(base, chcr);
        (void)system.bus().read32(base + 0x10u, madr);
        (void)system.bus().read32(base + 0x20u, qwc);
        (void)system.bus().read32(base + 0x30u, tadr);
        std::cout
            << "DMAC" << channel
            << "_CHCR=0x" << std::hex << std::uppercase << chcr
            << " MADR=0x" << madr
            << " QWC=0x" << qwc
            << " TADR=0x" << tadr
            << std::dec << '\n';
    }

    ps2::u32 dmac_ctrl = 0;
    ps2::u32 dmac_stat = 0;
    ps2::u32 dmac_pcr = 0;
    ps2::u32 ee_intc_stat = 0;
    ps2::u32 ee_intc_mask = 0;
    (void)system.bus().read32(0x1000E000u, dmac_ctrl);
    (void)system.bus().read32(0x1000E010u, dmac_stat);
    (void)system.bus().read32(0x1000E020u, dmac_pcr);
    (void)system.bus().read32(0x1000F000u, ee_intc_stat);
    (void)system.bus().read32(0x1000F010u, ee_intc_mask);
    std::cout
        << "DMAC_CTRL=0x" << std::hex << std::uppercase << dmac_ctrl
        << " DMAC_STAT=0x" << dmac_stat
        << " DMAC_PCR=0x" << dmac_pcr
        << " EE_INTC_STAT=0x" << ee_intc_stat
        << " EE_INTC_MASK=0x" << ee_intc_mask
        << std::dec << '\n';

    ps2::u32 sif_regs[4]{};
    for (ps2::u32 index = 0; index < 4u; ++index) {
        (void)system.bus().read32(
            0x1000F200u + index * 0x10u,
            sif_regs[index]);
    }
    ps2::u32 iop_sbus = 0;
    (void)system.iop_bus().read32(0x1F801450u, iop_sbus);
    std::cout
        << "SIF_MSCOM=0x" << std::hex << std::uppercase << sif_regs[0]
        << " SIF_SMCOM=0x" << sif_regs[1]
        << " SIF_MSFLAG=0x" << sif_regs[2]
        << " SIF_SMFLAG=0x" << sif_regs[3]
        << " IOP_SBUS_1450=0x" << iop_sbus
        << std::dec << '\n';

    const auto& vu = system.vu1();
    const auto& vu_stats = vu.stats();
    std::cout
        << "VU1_RUNNING=" << (vu.running() ? 1 : 0)
        << " VU1_PC=0x" << std::hex << std::uppercase << vu.pc()
        << std::dec
        << " VU1_INSTRUCTIONS=" << vu_stats.instructions
        << " VU1_UNSUPPORTED_UPPER=" << vu_stats.unsupported_upper
        << " VU1_UNSUPPORTED_LOWER=" << vu_stats.unsupported_lower
        << " VU1_XGKICKS=" << vu_stats.xgkicks
        << " VU1_XGKICK_QWORDS=" << vu_stats.xgkick_qwords
        << '\n';

    const auto& gs = system.gs_core();
    const auto& gs_stats = gs.stats();
    std::cout
        << "GS_GIF_TAGS=" << gs_stats.gif_tags
        << " GS_GIF_QWORDS=" << gs_stats.gif_qwords
        << " GS_REGISTER_WRITES=" << gs_stats.register_writes
        << " GS_PRIMITIVES=" << gs_stats.primitives
        << " GS_RASTER_DRAWS=" << gs_stats.raster_draws
        << " GS_RASTER_PIXELS=" << gs_stats.raster_pixels
        << " GS_UNSUPPORTED_TRANSFERS=" << gs_stats.unsupported_transfers
        << " GS_UNSUPPORTED_PACKED=" << gs_stats.unsupported_packed
        << '\n';

    std::cout
        << "GS_BITBLTBUF=0x" << std::hex << std::uppercase
        << gs.register_value(0x50u)
        << " GS_TRXPOS=0x" << gs.register_value(0x51u)
        << " GS_TRXREG=0x" << gs.register_value(0x52u)
        << " GS_TRXDIR=0x" << gs.register_value(0x53u)
        << std::dec
        << " GS_TRANSFER_ACTIVE=" << (gs.transfer_active() ? 1 : 0)
        << " GS_TRANSFER_REMAINING=" << gs.transfer_pixels_remaining()
        << " GS_TRANSFER_PSM=0x" << std::hex << std::uppercase
        << gs.transfer_psm() << std::dec
        << '\n';

    const auto& display = system.gs_display();
    ps2::u64 framebuffer_hash = 1469598103934665603ull;
    for (const ps2::u32 pixel : display.rgba8()) {
        framebuffer_hash ^= pixel;
        framebuffer_hash *= 1099511628211ull;
    }

    std::cout
        << "DISPLAY_VALID=" << (display.valid() ? 1 : 0)
        << " DISPLAY_WIDTH=" << display.width()
        << " DISPLAY_HEIGHT=" << display.height()
        << " DISPLAY_CIRCUIT=" << display.circuit()
        << " DISPLAY_PSM=0x" << std::hex << std::uppercase << display.psm()
        << std::dec
        << " DISPLAY_GENERATION=" << display.generation()
        << " DISPLAY_NONZERO_PIXELS=" << display.nonzero_pixel_count()
        << " DISPLAY_HASH=0x" << std::hex << std::uppercase
        << framebuffer_hash << std::dec
        << '\n';

    ps2::u64 pmode = 0;
    ps2::u64 smode2 = 0;
    ps2::u64 dispfb1 = 0;
    ps2::u64 display1 = 0;
    ps2::u64 dispfb2 = 0;
    ps2::u64 display2 = 0;
    ps2::u64 bgcolor = 0;
    (void)system.gs_privileged().read64(0x12000000u, pmode);
    (void)system.gs_privileged().read64(0x12000020u, smode2);
    (void)system.gs_privileged().read64(0x12000070u, dispfb1);
    (void)system.gs_privileged().read64(0x12000080u, display1);
    (void)system.gs_privileged().read64(0x12000090u, dispfb2);
    (void)system.gs_privileged().read64(0x120000A0u, display2);
    (void)system.gs_privileged().read64(0x120000E0u, bgcolor);
    std::cout
        << "PCRTC_PMODE=0x" << std::hex << std::uppercase << pmode
        << " PCRTC_SMODE2=0x" << smode2
        << " PCRTC_DISPFB1=0x" << dispfb1
        << " PCRTC_DISPLAY1=0x" << display1
        << " PCRTC_DISPFB2=0x" << dispfb2
        << " PCRTC_DISPLAY2=0x" << display2
        << " PCRTC_BGCOLOR=0x" << bgcolor
        << std::dec << '\n';
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
        system.refresh_display();

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
