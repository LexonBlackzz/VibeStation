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

    std::cout << "EE_EXCEPTIONS";
    for (ps2::u32 code = 0; code < ee.exception_counts.size(); ++code) {
        if (ee.exception_counts[code] != 0) {
            std::cout
                << " [" << code << "]="
                << ee.exception_counts[code];
        }
    }
    std::cout << '\n';

    for (ps2::u32 offset = 0;
         offset < ee.recent_syscall_count;
         ++offset) {
        const ps2::u32 index =
            (ee.recent_syscall_next +
             static_cast<ps2::u32>(ee.recent_syscalls.size()) -
             ee.recent_syscall_count + offset) %
            static_cast<ps2::u32>(ee.recent_syscalls.size());
        const auto& call = ee.recent_syscalls[index];
        std::cout
            << "EE_SYSCALL[" << offset << "]"
            << " INS=" << call.instruction
            << " PC=0x" << std::hex << std::uppercase << call.pc
            << " NUM=0x" << call.number
            << " ARGS=0x" << call.args[0]
            << ",0x" << call.args[1]
            << ",0x" << call.args[2]
            << ",0x" << call.args[3]
            << std::dec << '\n';
    }

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

            const ps2::u32 source = tag[0] & 0x00FFFFFFu;
            const ps2::u32 payload_words =
                (tag[1] & 0x000FFFFFu) < 32u
                    ? (tag[1] & 0x000FFFFFu)
                    : 32u;
            std::cout << "IOP_SIF0_PAYLOAD";
            for (ps2::u32 index = 0; index < payload_words; ++index) {
                ps2::u32 word = 0;
                if (system.iop_bus().read32(source + index * 4u, word)) {
                    std::cout
                        << " [" << std::dec << index << "]=0x"
                        << std::hex << std::uppercase << word;
                }
            }
            std::cout << std::dec << '\n';
        }
    }

    ps2::u32 dma4_madr = 0;
    ps2::u32 dma4_bcr = 0;
    ps2::u32 dma4_chcr = 0;
    ps2::u32 dma_icr = 0;
    ps2::u32 dma_icr2 = 0;
    (void)system.iop_bus().read32(0x1F8010C0u, dma4_madr);
    (void)system.iop_bus().read32(0x1F8010C4u, dma4_bcr);
    (void)system.iop_bus().read32(0x1F8010C8u, dma4_chcr);
    (void)system.iop_bus().read32(0x1F8010F4u, dma_icr);
    (void)system.iop_bus().read32(0x1F801574u, dma_icr2);
    std::cout
        << "IOP_DMAC4_MADR=0x" << std::hex << std::uppercase << dma4_madr
        << " BCR=0x" << dma4_bcr
        << " CHCR=0x" << dma4_chcr
        << " DICR=0x" << dma_icr
        << " DICR2=0x" << dma_icr2
        << std::dec << '\n';

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

    auto print_iop_words = [&](const char* label, ps2::u32 address) {
        std::cout << label;
        for (ps2::u32 index = 0; index < 16u; ++index) {
            ps2::u32 word = 0;
            if (system.iop_bus().read32(address + index * 4u, word)) {
                std::cout
                    << " [" << std::dec << index << "]=0x"
                    << std::hex << std::uppercase << word;
            }
        }
        std::cout << std::dec << '\n';
    };
    print_iop_words("IOP_SIF_COMMAND", 0x00019800u);
    print_iop_words("IOP_RPC_BUFFER", 0x000467B8u);
    print_iop_words("IOP_RPC_SERVER", 0x00046770u);

    std::cout << "SPU2_REGS";
    for (const ps2::u32 address : {
             0x1F90019Au, 0x1F90019Cu, 0x1F90019Eu,
             0x1F9001A8u, 0x1F9001AAu, 0x1F9001B0u,
             0x1F900344u,
             0x1F90059Au, 0x1F90059Cu, 0x1F90059Eu,
             0x1F9005A8u, 0x1F9005AAu, 0x1F9005B0u,
             0x1F900744u}) {
        ps2::u16 value = 0;
        if (system.iop_bus().read16(address, value)) {
            std::cout
                << " [0x" << std::hex << std::uppercase << address
                << "]=0x" << value;
        }
    }
    std::cout << std::dec << '\n';

    std::cout << "IOP_EXCEPTIONS";
    for (ps2::u32 code = 0; code < iop.exception_counts.size(); ++code) {
        if (iop.exception_counts[code] != 0) {
            std::cout
                << " [" << code << "]="
                << iop.exception_counts[code];
        }
    }
    std::cout << '\n';

    for (ps2::u32 offset = 0;
         offset < iop.recent_syscall_count;
         ++offset) {
        const ps2::u32 index =
            (iop.recent_syscall_next +
             static_cast<ps2::u32>(iop.recent_syscalls.size()) -
             iop.recent_syscall_count + offset) %
            static_cast<ps2::u32>(iop.recent_syscalls.size());
        const auto& call = iop.recent_syscalls[index];
        std::cout
            << "IOP_SYSCALL[" << offset << "]"
            << " INS=" << call.instruction
            << " PC=0x" << std::hex << std::uppercase << call.pc
            << " CODE=0x" << call.encoded
            << " V0=0x" << call.v0
            << " ARGS=0x" << call.args[0]
            << ",0x" << call.args[1]
            << ",0x" << call.args[2]
            << ",0x" << call.args[3]
            << std::dec << '\n';
    }

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

    std::cout << "IOP_THREAD_CANDIDATES" << '\n';
    for (ps2::u32 address = 0;
         address + 0x90u < 0x00200000u;
         address += 4u) {
        ps2::u16 tag = 0;
        ps2::u16 tid = 0;
        ps2::u8 status = 0;
        ps2::u16 priority = 0;
        ps2::u32 reg_context = 0;
        ps2::u32 entry = 0;
        ps2::u16 wait_state = 0;
        ps2::u32 wait_id = 0;
        ps2::u32 next = 0;
        if (!system.iop_bus().read16(address + 0x08u, tag) ||
            tag != 0x7F01u ||
            !system.iop_bus().read16(address + 0x0Au, tid) ||
            !system.iop_bus().read8(address + 0x0Cu, status) ||
            !system.iop_bus().read16(address + 0x0Eu, priority) ||
            !system.iop_bus().read32(address + 0x10u, reg_context) ||
            !system.iop_bus().read16(address + 0x1Cu, wait_state) ||
            !system.iop_bus().read32(address + 0x20u, wait_id) ||
            !system.iop_bus().read32(address + 0x24u, next) ||
            !system.iop_bus().read32(address + 0x38u, entry)) {
            continue;
        }
        if (tid >= 256u ||
            (status != 1u && status != 2u && status != 4u &&
             status != 8u && status != 12u && status != 16u)) {
            continue;
        }

        ps2::u32 saved_pc = 0;
        if (reg_context < 0x00200000u) {
            (void)system.iop_bus().read32(
                reg_context + 0x8Cu,
                saved_pc);
        }
        std::cout
            << "IOP_THREAD TCB=0x" << std::hex << std::uppercase
            << address
            << " TID=0x" << tid
            << " STATUS=0x" << static_cast<unsigned>(status)
            << " PRI=0x" << priority
            << " PC=0x" << saved_pc
            << " ENTRY=0x" << entry
            << " WAIT=0x" << wait_state
            << " WAIT_ID=0x" << wait_id
            << " NEXT=0x" << next
            << std::dec << '\n';
    }

    std::cout
        << "IOP_ISTAT=0x" << std::hex << std::uppercase
        << system.iop_intc().status()
        << " IOP_IMASK=0x" << system.iop_intc().mask()
        << " IOP_ICTRL=0x" << system.iop_intc().control()
        << std::dec
        << " SCHEDULER_TICK=" << system.scheduler().now()
        << '\n';

    constexpr ps2::u32 kIopTimerBases[] = {
        0x1F801100u, 0x1F801110u, 0x1F801120u,
        0x1F801480u, 0x1F801490u, 0x1F8014A0u,
    };
    for (ps2::u32 index = 0; index < 6u; ++index) {
        ps2::u32 count = 0;
        ps2::u32 mode = 0;
        ps2::u32 target = 0;
        (void)system.iop_bus().read32(kIopTimerBases[index], count);
        (void)system.iop_bus().read32(kIopTimerBases[index] + 4u, mode);
        (void)system.iop_bus().read32(kIopTimerBases[index] + 8u, target);
        std::cout
            << "IOP_TIMER" << std::dec << index
            << " COUNT=0x" << std::hex << std::uppercase << count
            << " MODE=0x" << mode
            << " TARGET=0x" << target
            << std::dec << '\n';
    }

    ps2::u8 cdvd_scommand = 0;
    ps2::u8 cdvd_sready = 0;
    ps2::u8 cdvd_intr_stat = 0;
    (void)system.iop_bus().read8(0x1F402016u, cdvd_scommand);
    (void)system.iop_bus().read8(0x1F402017u, cdvd_sready);
    (void)system.iop_bus().read8(0x1F402008u, cdvd_intr_stat);
    std::cout
        << "CDVD_SCOMMAND=0x" << std::hex << std::uppercase
        << static_cast<unsigned>(cdvd_scommand)
        << " CDVD_SREADY=0x" << static_cast<unsigned>(cdvd_sready)
        << " CDVD_INTR_STAT=0x"
        << static_cast<unsigned>(cdvd_intr_stat)
        << std::dec << '\n';

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
        ps2::u32 vif_code = 0;
        ps2::u32 vif_base_reg = 0;
        ps2::u32 vif_ofst = 0;
        ps2::u32 vif_tops = 0;
        ps2::u32 vif_itop = 0;
        ps2::u32 vif_top = 0;
        ps2::u32 vif_chcr = 0;
        ps2::u32 vif_qwc = 0;
        const ps2::u32 vif_base = 0x10003800u + channel * 0x400u;
        const ps2::u32 dma_base = 0x10008000u + channel * 0x1000u;
        (void)system.bus().read32(vif_base, vif_stat);
        (void)system.bus().read32(vif_base + 0x80u, vif_code);
        (void)system.bus().read32(vif_base + 0xA0u, vif_base_reg);
        (void)system.bus().read32(vif_base + 0xB0u, vif_ofst);
        (void)system.bus().read32(vif_base + 0xC0u, vif_tops);
        (void)system.bus().read32(vif_base + 0xD0u, vif_itop);
        (void)system.bus().read32(vif_base + 0xE0u, vif_top);
        (void)system.bus().read32(dma_base, vif_chcr);
        (void)system.bus().read32(dma_base + 0x20u, vif_qwc);
        std::cout
            << "VIF" << channel << "_STAT=0x"
            << std::hex << std::uppercase << vif_stat
            << " VIF" << channel << "_CODE=0x" << vif_code
            << " BASE=0x" << vif_base_reg
            << " OFST=0x" << vif_ofst
            << " TOPS=0x" << vif_tops
            << " ITOP=0x" << vif_itop
            << " TOP=0x" << vif_top
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

    ps2::u32 vif1_madr = 0;
    (void)system.bus().read32(0x10009010u, vif1_madr);
    const ps2::u32 vif1_dump_start =
        vif1_madr >= 0x100u ? (vif1_madr - 0x100u) & ~0xFu : 0u;
    std::cout << "VIF1_DMA_MEMORY";
    for (ps2::u32 offset = 0; offset < 0x120u; offset += 4u) {
        ps2::u32 word = 0;
        if (system.bus().read32(vif1_dump_start + offset, word)) {
            std::cout
                << " [0x" << std::hex << std::uppercase
                << (vif1_dump_start + offset) << "]=0x" << word;
        }
    }
    std::cout << std::dec << '\n';

    std::cout << "VIF1_SPR_TAG_MEMORY";
    for (ps2::u32 address = 0x70002280u;
         address < 0x70002320u;
         address += 4u) {
        ps2::u32 word = 0;
        if (system.bus().read32(address, word)) {
            std::cout
                << " [0x" << std::hex << std::uppercase
                << address << "]=0x" << word;
        }
    }
    std::cout << std::dec << '\n';

    const auto& vif1 = system.vif1_dma();
    for (ps2::u32 offset = 0; offset < vif1.recent_tag_count(); ++offset) {
        const ps2::u32 index =
            (vif1.recent_tag_next() +
             static_cast<ps2::u32>(vif1.recent_tags().size()) -
             vif1.recent_tag_count() + offset) %
            static_cast<ps2::u32>(vif1.recent_tags().size());
        const auto& tag = vif1.recent_tags()[index];
        std::cout
            << "VIF1_TAG[" << offset << "] ADDR=0x"
            << std::hex << std::uppercase << tag.address
            << " TAG0=0x" << tag.tag0
            << " TAG1=0x" << tag.tag1
            << " NEXT=0x" << tag.next_tadr
            << " MADR=0x" << tag.madr
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

    const auto& sif_stats = system.sif_dma().stats();
    std::cout
        << "SIF0_PACKETS=" << sif_stats.sif0_packets
        << " SIF0_PADDED_PACKETS="
        << sif_stats.sif0_padded_packets
        << " SIF0_PADDING_WORDS="
        << sif_stats.sif0_padding_words
        << " SIF0_STREAM_MISMATCHES="
        << sif_stats.sif0_stream_mismatches
        << '\n';
    for (ps2::u32 offset = 0;
         offset < sif_stats.recent_sif0_count;
         ++offset) {
        const ps2::u32 index =
            (sif_stats.recent_sif0_next +
             static_cast<ps2::u32>(
                 sif_stats.recent_sif0_packets.size()) -
             sif_stats.recent_sif0_count + offset) %
            static_cast<ps2::u32>(
                sif_stats.recent_sif0_packets.size());
        const auto& packet = sif_stats.recent_sif0_packets[index];
        std::cout
            << "SIF0_RECENT[" << offset << "]"
            << " SRC_TAG=0x" << std::hex << std::uppercase
            << packet.source_tag
            << " WORDS=0x" << packet.source_words
            << " EE_TAG=0x" << packet.destination_tag
            << " DEST=0x" << packet.destination
            << " HEAD=0x" << packet.payload[0]
            << ",0x" << packet.payload[1]
            << ",0x" << packet.payload[2]
            << ",0x" << packet.payload[3]
            << std::dec << '\n';
    }
    std::cout << "SIF1_PACKETS=" << sif_stats.sif1_packets << '\n';
    for (ps2::u32 offset = 0;
         offset < sif_stats.recent_sif1_count;
         ++offset) {
        const ps2::u32 index =
            (sif_stats.recent_sif1_next +
             static_cast<ps2::u32>(
                 sif_stats.recent_sif1_packets.size()) -
             sif_stats.recent_sif1_count + offset) %
            static_cast<ps2::u32>(
                sif_stats.recent_sif1_packets.size());
        const auto& packet = sif_stats.recent_sif1_packets[index];
        std::cout
            << "SIF1_RECENT[" << offset << "]"
            << " TAG=0x" << std::hex << std::uppercase
            << packet.destination_tag
            << " WORDS=0x" << packet.words
            << " DEST=0x" << packet.destination
            << " HEAD=0x" << packet.payload[0]
            << ",0x" << packet.payload[1]
            << ",0x" << packet.payload[2]
            << ",0x" << packet.payload[3]
            << std::dec << '\n';
    }

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

    std::cout
        << "GS_DRAW_REGS"
        << " PRIM=0x" << std::hex << std::uppercase
        << gs.register_value(0x00u)
        << " RGBAQ=0x" << gs.register_value(0x01u)
        << " ST=0x" << gs.register_value(0x02u)
        << " UV=0x" << gs.register_value(0x03u)
        << " XYZF2=0x" << gs.register_value(0x04u)
        << " XYZ2=0x" << gs.register_value(0x05u)
        << " TEX0_1=0x" << gs.register_value(0x06u)
        << " TEX0_2=0x" << gs.register_value(0x07u)
        << " XYOFFSET_1=0x" << gs.register_value(0x18u)
        << " XYOFFSET_2=0x" << gs.register_value(0x19u)
        << " SCISSOR_1=0x" << gs.register_value(0x40u)
        << " SCISSOR_2=0x" << gs.register_value(0x41u)
        << " FRAME_1=0x" << gs.register_value(0x4Cu)
        << " FRAME_2=0x" << gs.register_value(0x4Du)
        << std::dec << '\n';

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

        if (!error.empty()) {
            std::cerr << "TRACE_ERROR=" << error << '\n';
            print_state(system);
            return 5;
        }

        const ps2::u64 executed = budget - remaining;
        if (executed >= 200'000'000u &&
            (executed % 100'000u) == 0u) {
            const auto& stats = system.gs_core().stats();
            std::cerr
                << "TRACE_PROGRESS EE=" << executed
                << " PC=0x" << std::hex << std::uppercase
                << system.ee().state().pc
                << std::dec
                << " GIF_QWORDS=" << stats.gif_qwords
                << " IMAGE_QWORDS=" << stats.image_qwords
                << " PRIMITIVES=" << stats.primitives
                << " DRAWS=" << stats.raster_draws
                << " PIXELS=" << stats.raster_pixels
                << " TRANSFER_REMAINING="
                << system.gs_core().transfer_pixels_remaining()
                << '\n';
        }

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
