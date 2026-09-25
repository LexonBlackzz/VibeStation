#include "core/ps2_system.h"

#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

namespace {

void write_le16(std::ofstream& out, std::uint16_t value) {
    const char bytes[2] = {
        static_cast<char>(value),
        static_cast<char>(value >> 8),
    };
    out.write(bytes, sizeof(bytes));
}

void write_le32(std::ofstream& out, std::uint32_t value) {
    const char bytes[4] = {
        static_cast<char>(value),
        static_cast<char>(value >> 8),
        static_cast<char>(value >> 16),
        static_cast<char>(value >> 24),
    };
    out.write(bytes, sizeof(bytes));
}

bool write_pcm16_wav(
    const char* path,
    const std::vector<ps2::s16>& interleaved_stereo) {
    if (path == nullptr) return false;

    std::ofstream out(path, std::ios::binary);
    if (!out) return false;

    constexpr std::uint16_t channels = 2;
    constexpr std::uint16_t bits_per_sample = 16;
    constexpr std::uint32_t sample_rate = ps2::Spu2::kSampleRate;
    constexpr std::uint16_t block_align =
        channels * (bits_per_sample / 8u);
    constexpr std::uint32_t byte_rate =
        sample_rate * block_align;

    const std::uint32_t data_bytes =
        static_cast<std::uint32_t>(
            interleaved_stereo.size() * sizeof(ps2::s16));

    out.write("RIFF", 4);
    write_le32(out, 36u + data_bytes);
    out.write("WAVE", 4);
    out.write("fmt ", 4);
    write_le32(out, 16u);
    write_le16(out, 1u);
    write_le16(out, channels);
    write_le32(out, sample_rate);
    write_le32(out, byte_rate);
    write_le16(out, block_align);
    write_le16(out, bits_per_sample);
    out.write("data", 4);
    write_le32(out, data_bytes);

    for (const ps2::s16 sample : interleaved_stereo)
        write_le16(out, static_cast<std::uint16_t>(sample));

    return static_cast<bool>(out);
}

void print_audio_stats(
    const std::vector<ps2::s16>& pcm) {
    std::uint64_t nonzero = 0;
    std::uint32_t peak = 0;
    long double square_sum = 0.0L;

    for (const ps2::s16 sample : pcm) {
        const std::int32_t signed_sample = sample;
        const std::uint32_t magnitude =
            static_cast<std::uint32_t>(
                signed_sample < 0 ? -signed_sample : signed_sample);
        if (magnitude != 0u) ++nonzero;
        peak = std::max(peak, magnitude);
        const long double normalized =
            static_cast<long double>(signed_sample) / 32768.0L;
        square_sum += normalized * normalized;
    }

    const long double rms =
        pcm.empty()
            ? 0.0L
            : std::sqrt(
                square_sum /
                static_cast<long double>(pcm.size()));
    const long double rms_dbfs =
        rms > 0.0L
            ? 20.0L * std::log10(rms)
            : -INFINITY;
    const long double peak_dbfs =
        peak != 0u
            ? 20.0L * std::log10(
                static_cast<long double>(peak) / 32768.0L)
            : -INFINITY;

    std::cout
        << "SPU2_CAPTURE_FRAMES=" << (pcm.size() / 2u)
        << " SPU2_CAPTURE_NONZERO_SAMPLES=" << nonzero
        << " SPU2_CAPTURE_PEAK=" << peak
        << " SPU2_CAPTURE_PEAK_DBFS="
        << static_cast<double>(peak_dbfs)
        << " SPU2_CAPTURE_RMS_DBFS="
        << static_cast<double>(rms_dbfs)
        << '\n';
}

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

bool simple_register_instruction(ps2::u32 instruction) {
    if (instruction == 0u) return true;
    const ps2::u32 opcode = instruction >> 26;
    if (opcode == 0x09u || opcode == 0x0Cu || opcode == 0x0Du ||
        opcode == 0x0Eu || opcode == 0x0Fu || opcode == 0x19u) return true;
    if (opcode != 0u) return false;
    const ps2::u32 funct = instruction & 63u;
    const ps2::u32 rs = (instruction >> 21) & 31u;
    const ps2::u32 sa = (instruction >> 6) & 31u;
    if (funct == 0u || funct == 2u || funct == 3u) return rs == 0u;
    return sa == 0u && (funct == 0x21u || funct == 0x23u ||
                        funct == 0x24u || funct == 0x25u ||
                        funct == 0x26u || funct == 0x2Du);
}

ps2::u32 simple_register_run(const ps2::Ps2System& system) {
    const ps2::u32 pc = system.ee().state().pc;
    if (pc >= ps2::EeRam::kSize) return 0;
    ps2::u32 count = 0;
    for (; count < 32u && pc + count * 4u < ps2::EeRam::kSize; ++count) {
        ps2::u32 instruction = 0;
        if (!system.bus().fetch32(pc + count * 4u, instruction) ||
            !simple_register_instruction(instruction)) break;
    }
    return count;
}

bool visible_frame_ready(const ps2::Ps2System& system) {
    const auto& stats = system.gs_core().stats();
    const bool gs_wrote_pixels =
        stats.host_to_local_pixels != 0 ||
        stats.local_to_local_pixels != 0 ||
        stats.raster_pixels != 0;
    if (!gs_wrote_pixels || !system.gs_display().valid()) {
        return false;
    }

    for (const ps2::u32 pixel : system.gs_display().rgba8()) {
        if ((pixel & 0x00FFFFFFu) != 0) {
            return true;
        }
    }
    return false;
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

    std::cout
        << "EE_HOT_SIF_GETREG_READ_OFFSET="
        << system.ee().hot_sif_getreg_read_offset()
        << '\n';

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

    const auto& spu2_stats = system.spu2().debug_stats();
    std::cout
        << "SPU2_DEBUG"
        << " REG_WRITES=" << spu2_stats.register_writes
        << " DMA_WRITE_HALFWORDS=" << spu2_stats.dma_write_halfwords
        << " DMA_READ_HALFWORDS=" << spu2_stats.dma_read_halfwords
        << " KON_WRITES=" << spu2_stats.key_on_writes
        << " KOFF_WRITES=" << spu2_stats.key_off_writes
        << " PARTIAL_KON=" << spu2_stats.partial_key_on_writes
        << " VOICE_PARAM_WRITES=" << spu2_stats.voice_param_writes
        << " NONZERO_VOL_WRITES=" << spu2_stats.nonzero_volume_writes
        << " NONZERO_ADSR_WRITES=" << spu2_stats.nonzero_adsr_writes
        << " NONDEFAULT_PITCH_WRITES="
        << spu2_stats.nondefault_pitch_writes
        << " MAX_WRITTEN_VOL=" << spu2_stats.max_written_volume
        << " FIRST_AUDIBLE_PARAM_FRAME="
        << spu2_stats.first_nonzero_voice_param_frame
        << " FIRST_PARTIAL_KON_FRAME="
        << spu2_stats.first_partial_key_on_frame
        << " VOICES_KON=" << spu2_stats.keyed_on_voices
        << " VOICES_KOFF=" << spu2_stats.keyed_off_voices
        << " DECODED_BLOCKS=" << spu2_stats.decoded_blocks
        << " DECODED_NONZERO=" << spu2_stats.decoded_nonzero_samples
        << " DECODED_PEAK=" << spu2_stats.decoded_peak
        << " MIX_FRAMES=" << spu2_stats.mixed_frames
        << " MIX_ACTIVE_FRAMES="
        << spu2_stats.mixer_frames_with_active_voice
        << " MAX_ACTIVE=" << spu2_stats.max_active_voices
        << " PRE_MASTER_PEAK=" << spu2_stats.pre_master_peak
        << " OUTPUT_PEAK=" << spu2_stats.output_peak
        << '\n';

    for (ps2::u32 core = 0; core < 2u; ++core) {
        const ps2::u32 base = core * 0x400u;
        ps2::u16 kon_lo = 0, kon_hi = 0;
        ps2::u16 koff_lo = 0, koff_hi = 0;
        ps2::u16 vmixl_lo = 0, vmixl_hi = 0;
        ps2::u16 vmixr_lo = 0, vmixr_hi = 0;
        ps2::u16 mmix = 0;
        (void)system.spu2().read16(base + 0x1A0u, kon_lo);
        (void)system.spu2().read16(base + 0x1A2u, kon_hi);
        (void)system.spu2().read16(base + 0x1A4u, koff_lo);
        (void)system.spu2().read16(base + 0x1A6u, koff_hi);
        (void)system.spu2().read16(base + 0x188u, vmixl_lo);
        (void)system.spu2().read16(base + 0x18Au, vmixl_hi);
        (void)system.spu2().read16(base + 0x190u, vmixr_lo);
        (void)system.spu2().read16(base + 0x192u, vmixr_hi);
        (void)system.spu2().read16(base + 0x198u, mmix);
        std::cout
            << "SPU2_CORE" << core
            << " KON=0x" << std::hex << std::uppercase
            << kon_lo << ":" << kon_hi
            << " KOFF=0x" << koff_lo << ":" << koff_hi
            << " VMIXL=0x" << vmixl_lo << ":" << vmixl_hi
            << " VMIXR=0x" << vmixr_lo << ":" << vmixr_hi
            << " MMIX=0x" << mmix
            << std::dec << '\n';
    }

    for (ps2::u32 core = 0; core < 2u; ++core) {
        const ps2::u32 base = core * 0x400u;
        for (ps2::u32 voice = 0; voice < 24u; ++voice) {
            const ps2::u32 vbase = base + voice * 0x10u;
            ps2::u16 voll = 0, volr = 0, pitch = 0;
            ps2::u16 adsr1 = 0, adsr2 = 0, envx = 0;
            (void)system.spu2().read16(vbase + 0x0u, voll);
            (void)system.spu2().read16(vbase + 0x2u, volr);
            (void)system.spu2().read16(vbase + 0x4u, pitch);
            (void)system.spu2().read16(vbase + 0x6u, adsr1);
            (void)system.spu2().read16(vbase + 0x8u, adsr2);
            (void)system.spu2().read16(vbase + 0xAu, envx);
            if ((voll | volr | pitch | adsr1 | adsr2 | envx) == 0u)
                continue;
            std::cout
                << "SPU2_VOICE C=" << core
                << " V=" << voice
                << " VOLL=0x" << std::hex << std::uppercase << voll
                << " VOLR=0x" << volr
                << " PITCH=0x" << pitch
                << " ADSR1=0x" << adsr1
                << " ADSR2=0x" << adsr2
                << " ENVX=0x" << envx
                << std::dec << '\n';
        }
    }

    // Retail SCPH-39001 OSDSND (rspu2_driver) SIF RPC server. Dump
    // the live server/queue objects so we can distinguish "packet arrived"
    // from "RPC thread actually consumed it".
    constexpr ps2::u32 kOsdSndServer = 0x001EC190u;
    std::array<ps2::u32, 17> osdsnd_server{};
    bool osdsnd_server_ok = true;
    for (ps2::u32 word = 0; word < osdsnd_server.size(); ++word) {
        osdsnd_server_ok =
            system.iop_bus().read32(
                kOsdSndServer + word * 4u,
                osdsnd_server[word]) &&
            osdsnd_server_ok;
    }
    if (osdsnd_server_ok) {
        std::cout << "OSDSND_RPC_SERVER";
        for (ps2::u32 word = 0; word < osdsnd_server.size(); ++word) {
            std::cout
                << " [" << word << "]=0x"
                << std::hex << std::uppercase
                << osdsnd_server[word] << std::dec;
        }
        std::cout << '\n';

        const ps2::u32 queue =
            osdsnd_server[16] & 0x001FFFFFu;
        if (queue != 0u) {
            std::array<ps2::u32, 6> q{};
            bool queue_ok = true;
            for (ps2::u32 word = 0; word < q.size(); ++word) {
                queue_ok =
                    system.iop_bus().read32(
                        queue + word * 4u, q[word]) &&
                    queue_ok;
            }
            if (queue_ok) {
                std::cout
                    << "OSDSND_RPC_QUEUE ADDR=0x"
                    << std::hex << std::uppercase << queue
                    << " THREAD=0x" << q[0]
                    << " ACTIVE=0x" << q[1]
                    << " LINK=0x" << q[2]
                    << " START=0x" << q[3]
                    << " END=0x" << q[4]
                    << " NEXT=0x" << q[5]
                    << std::dec << '\n';
            }
        }
    }

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

    std::cout << "SPU2_MIX_REGS";
    for (const ps2::u32 address : {
             0x1F900188u, 0x1F90018Au,
             0x1F90018Cu, 0x1F90018Eu,
             0x1F900190u, 0x1F900192u,
             0x1F900194u, 0x1F900196u,
             0x1F900198u, 0x1F90019Au,
             0x1F9001A0u, 0x1F9001A2u,
             0x1F9001A4u, 0x1F9001A6u,
             0x1F900340u, 0x1F900342u,
             0x1F900588u, 0x1F90058Au,
             0x1F90058Cu, 0x1F90058Eu,
             0x1F900590u, 0x1F900592u,
             0x1F900594u, 0x1F900596u,
             0x1F900598u, 0x1F90059Au,
             0x1F9005A0u, 0x1F9005A2u,
             0x1F9005A4u, 0x1F9005A6u,
             0x1F900740u, 0x1F900742u,
             0x1F900760u, 0x1F900762u,
             0x1F900788u, 0x1F90078Au,
             0x1F900790u, 0x1F900792u}) {
        ps2::u16 value = 0;
        if (system.iop_bus().read16(address, value)) {
            std::cout
                << " [0x" << std::hex << std::uppercase
                << address << "]=0x" << value;
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

    const auto& rc_debug =
        system.iop_bus().root_counter_debug();
    for (ps2::u32 index = 0; index < 6u; ++index) {
        std::cout
            << "IOP_TIMER_DEBUG" << index
            << " COUNT_WRITES=" << rc_debug.count_writes[index]
            << " MODE_WRITES=" << rc_debug.mode_writes[index]
            << " TARGET_WRITES=" << rc_debug.target_writes[index]
            << " TARGET_EVENTS=" << rc_debug.target_events[index]
            << " OVERFLOW_EVENTS=" << rc_debug.overflow_events[index]
            << " IRQ_EVENTS=" << rc_debug.irq_events[index]
            << " LAST_MODE_WRITE=0x"
            << std::hex << std::uppercase
            << rc_debug.last_mode_write[index]
            << " LAST_TARGET_WRITE=0x"
            << rc_debug.last_target_write[index]
            << " FIRST_NONZERO_TARGET=0x"
            << rc_debug.first_nonzero_target[index]
            << std::dec << '\n';
    }

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
    std::cout
        << "SIF1_PACKETS=" << sif_stats.sif1_packets
        << " RPC_CALLS=" << sif_stats.rpc_calls
        << " SOUND_RPC_CALLS=" << sif_stats.sound_rpc_calls
        << " SOUND_ST_INIT=" << sif_stats.sound_st_init_calls
        << " SOUND_BGM_OPEN=" << sif_stats.sound_bgm_open_calls
        << " SOUND_TICK_MODE=" << sif_stats.sound_tick_mode_calls
        << " SOUND_MASTER_VOL=" << sif_stats.sound_master_volume_calls
        << " SOUND_BGM_PLAY=" << sif_stats.sound_bgm_play_calls
        << " SOUND_BGM_STOP=" << sif_stats.sound_bgm_stop_calls
        << " SOUND_TIMER_START=" << sif_stats.sound_timer_start_calls
        << " SOUND_SE_PLAY=" << sif_stats.sound_se_play_calls
        << " SOUND_SETPARAM=" << sif_stats.sound_set_param_calls
        << " SOUND_SETSWITCH=" << sif_stats.sound_set_switch_calls
        << " SOUND_SETADDR=" << sif_stats.sound_set_addr_calls
        << '\n';
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

    for (ps2::u32 offset = 0;
         offset < sif_stats.recent_rpc_count;
         ++offset) {
        const ps2::u32 index =
            (sif_stats.recent_rpc_next +
             static_cast<ps2::u32>(
                 sif_stats.recent_rpc_calls.size()) -
             sif_stats.recent_rpc_count + offset) %
            static_cast<ps2::u32>(
                sif_stats.recent_rpc_calls.size());
        const auto& rpc = sif_stats.recent_rpc_calls[index];
        std::cout
            << "RPC_RECENT[" << offset << "]"
            << " SID=0x" << std::hex << std::uppercase
            << rpc.sid
            << " FNO=0x" << rpc.rpc_number
            << " SIZE=0x" << rpc.send_size
            << " SERVER=0x" << rpc.server
            << " BUF=0x" << rpc.server_buffer
            << std::dec;
        if (rpc.payload_words != 0u) {
            std::cout << " ARGS";
            for (ps2::u32 word = 0;
                 word < rpc.payload_words;
                 ++word) {
                std::cout
                    << " [" << word << "]=0x"
                    << std::hex << std::uppercase
                    << rpc.payload[word]
                    << std::dec;
            }
        }
        if (rpc.sid == 0x80000601u &&
            rpc.rpc_number == 0x5009u &&
            rpc.payload_words >= 3u) {
            const ps2::u32 sequence =
                rpc.payload[2] & 0x001FFFFFu;
            std::cout
                << " BGM_SLOT="
                << (rpc.payload[1] & 0xFFFFu)
                << " BGM_PTR=0x"
                << std::hex << std::uppercase
                << sequence << std::dec;
            if (sequence != 0u) {
                std::cout << " BGM_HEAD";
                for (ps2::u32 word = 0; word < 8u; ++word) {
                    ps2::u32 value = 0;
                    if (system.iop_bus().read32(
                            sequence + word * 4u,
                            value)) {
                        std::cout
                            << " [" << word << "]=0x"
                            << std::hex << std::uppercase
                            << value << std::dec;
                    }
                }
            }
        }
        std::cout << '\n';
    }

    const auto& vu0 = system.vu0();
    const auto& vu0_stats = vu0.stats();
    std::cout
        << "VU0_RUNNING=" << (vu0.running() ? 1 : 0)
        << " VU0_PC=0x" << std::hex << std::uppercase << vu0.pc()
        << std::dec
        << " VU0_INSTRUCTIONS=" << vu0_stats.instructions
        << " VU0_UNSUPPORTED_UPPER=" << vu0_stats.unsupported_upper
        << " VU0_UNSUPPORTED_LOWER=" << vu0_stats.unsupported_lower
        << '\n';

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
        << " GS_TEXTURED_DRAWS=" << gs_stats.textured_raster_draws
        << " GS_TEXTURE_SAMPLES=" << gs_stats.texture_samples
        << " GS_NONZERO_TEXTURE_SAMPLES="
        << gs_stats.nonzero_texture_samples
        << " GS_TEXTURE_ALPHA_SAMPLES="
        << gs_stats.texture_alpha_samples
        << " GS_NONZERO_SHADED_SAMPLES="
        << gs_stats.nonzero_shaded_samples
        << " GS_NONZERO_RASTER_INPUTS="
        << gs_stats.nonzero_raster_inputs
        << " GS_NONZERO_INPUTS_WITH_ALPHA="
        << gs_stats.nonzero_inputs_with_alpha
        << " GS_NONZERO_RASTER_COLORS="
        << gs_stats.nonzero_raster_colors
        << " GS_NONZERO_INPUTS_BLEND="
        << gs_stats.nonzero_inputs_with_blend
        << " GS_NONZERO_INPUTS_NO_BLEND="
        << gs_stats.nonzero_inputs_without_blend
        << " GS_HOST_TO_LOCAL_PIXELS=" << gs_stats.host_to_local_pixels
        << " GS_LOCAL_TO_LOCAL_PIXELS=" << gs_stats.local_to_local_pixels
        << " GS_UNSUPPORTED_TRANSFERS=" << gs_stats.unsupported_transfers
        << " GS_UNSUPPORTED_PACKED=" << gs_stats.unsupported_packed
        << " GS_SKIPPED_RASTER_DRAWS=" << gs_stats.skipped_raster_draws
        << " GS_PARALLEL_SPRITE_DRAWS=" << gs_stats.parallel_sprite_draws
        << " GS_PARALLEL_SPRITE_PIXELS=" << gs_stats.parallel_sprite_pixels
        << " GS_PARALLEL_SPRITE_HELPER_JOBS="
        << gs_stats.parallel_sprite_helper_jobs
        << " GS_UNSUPPORTED_TARGET_DRAWS=" << gs_stats.unsupported_target_draws
        << " GS_UNSUPPORTED_TEXTURE_DRAWS=" << gs_stats.unsupported_texture_draws
        << '\n';
    std::cout << "GS_RASTER_PRIMITIVES";
    for (ps2::u32 i = 0; i < gs_stats.raster_draws_by_primitive.size(); ++i) {
        if (gs_stats.raster_draws_by_primitive[i] == 0u) continue;
        std::cout
            << " P" << i << "_DRAWS="
            << gs_stats.raster_draws_by_primitive[i]
            << " P" << i << "_PIXELS="
            << gs_stats.raster_pixels_by_primitive[i];
    }
    std::cout << '\n';

    std::cout << "GS_RASTER_TIME_NS";
    for (ps2::u32 i = 0; i < gs_stats.raster_ns_by_primitive.size(); ++i) {
        if (gs_stats.raster_ns_by_primitive[i] == 0u) continue;
        std::cout
            << " P" << i << '='
            << gs_stats.raster_ns_by_primitive[i];
    }
    std::cout << '\n';

    std::cout << "GS_RASTER_SIZE_TIME";
    constexpr const char* kRasterSizeLabels[] = {
        "LT64", "LT256", "LT1K", "LT4K", "LT16K", "LT64K", "GE64K"};
    for (std::size_t i = 0; i < gs_stats.raster_ns_by_size.size(); ++i) {
        if (gs_stats.raster_draws_by_size[i] == 0u) continue;
        std::cout
            << ' ' << kRasterSizeLabels[i]
            << "_D=" << gs_stats.raster_draws_by_size[i]
            << "_P=" << gs_stats.raster_pixels_by_size[i]
            << "_NS=" << gs_stats.raster_ns_by_size[i];
    }
    std::cout << '\n';

    std::cout
        << "GS_TEXTURE_COORD_PIXELS"
        << " SPRITE=" << gs_stats.textured_sprite_pixels
        << " SPRITE_FST=" << gs_stats.textured_sprite_fst_pixels
        << " SPRITE_CONST_Q=" << gs_stats.textured_sprite_constant_q_pixels
        << " SPRITE_VAR_Q=" << gs_stats.textured_sprite_variable_q_pixels
        << " TRI=" << gs_stats.textured_triangle_pixels
        << " TRI_FST=" << gs_stats.textured_triangle_fst_pixels
        << " TRI_CONST_Q=" << gs_stats.textured_triangle_constant_q_pixels
        << " TRI_VAR_Q=" << gs_stats.textured_triangle_variable_q_pixels
        << '\n';

    std::cout
        << "GS_GPU_CANDIDATES DRAWS="
        << gs_stats.gpu_candidate_sprite_draws
        << " PIXELS="
        << gs_stats.gpu_candidate_sprite_pixels
        << '\n';

    std::cout << "GS_GPU_CANDIDATE_SIGNATURES";
    for (ps2::u32 i = 0;
         i < gs_stats.gpu_candidate_signature_count;
         ++i) {
        std::cout
            << " S0x" << std::hex << std::uppercase
            << gs_stats.gpu_candidate_signatures[i]
            << std::dec
            << "_D=" << gs_stats.gpu_candidate_signature_draws[i]
            << "_P=" << gs_stats.gpu_candidate_signature_pixels[i];
    }
    std::cout << '\n';

    std::cout << "GS_TEXTURE_PSM_DRAWS";
    for (ps2::u32 i = 0; i < gs_stats.texture_draws_by_psm.size(); ++i) {
        if (gs_stats.texture_draws_by_psm[i] == 0u) continue;
        std::cout << " PSM" << i << '=' << gs_stats.texture_draws_by_psm[i];
    }
    std::cout << '\n';

    std::cout << "GS_TEXTURE_PSM_TIME_NS";
    for (ps2::u32 i = 0; i < gs_stats.texture_ns_by_psm.size(); ++i) {
        if (gs_stats.texture_ns_by_psm[i] == 0u) continue;
        std::cout
            << " PSM" << i << '='
            << gs_stats.texture_ns_by_psm[i];
    }
    std::cout << '\n';

    std::cout << "GS_PSM16_STATE_PROFILE";
    for (ps2::u32 i = 0; i < gs_stats.psm16_state_ns.size(); ++i) {
        if (gs_stats.psm16_state_ns[i] == 0u) continue;
        std::cout
            << " M" << i
            << "_D=" << gs_stats.psm16_state_draws[i]
            << "_P=" << gs_stats.psm16_state_pixels[i]
            << "_NS=" << gs_stats.psm16_state_ns[i];
    }
    std::cout << '\n';

    std::cout << "GS_PSM16_ALPHA_STATE_PROFILE";
    for (ps2::u32 i = 0; i < gs_stats.psm16_alpha_state_ns.size(); ++i) {
        if (gs_stats.psm16_alpha_state_ns[i] == 0u) continue;
        std::cout
            << " K" << i
            << "_D=" << gs_stats.psm16_alpha_state_draws[i]
            << "_P=" << gs_stats.psm16_alpha_state_pixels[i]
            << "_NS=" << gs_stats.psm16_alpha_state_ns[i];
    }
    std::cout << '\n';

    std::cout << "GS_PSM16_FIX_PROFILE";
    for (ps2::u32 i = 0; i < gs_stats.psm16_fix_ns.size(); ++i) {
        if (gs_stats.psm16_fix_ns[i] == 0u) continue;
        std::cout
            << " F" << i
            << "_D=" << gs_stats.psm16_fix_draws[i]
            << "_NS=" << gs_stats.psm16_fix_ns[i];
    }
    std::cout << '\n';

    std::cout
        << "GS_LAST_UNSUPPORTED PRIM=0x" << std::hex << std::uppercase
        << gs_stats.last_unsupported_prim
        << " FRAME=0x" << gs_stats.last_unsupported_frame
        << " ZBUF=0x" << gs_stats.last_unsupported_zbuf
        << " TEST=0x" << gs_stats.last_unsupported_test
        << " TEX0=0x" << gs_stats.last_unsupported_tex0
        << std::dec << '\n';

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

    for (ps2::u32 i = 0;
         i < gs_stats.first_unsupported_transfer_count;
         ++i) {
        const auto& failed = gs_stats.first_unsupported_transfers[i];
        std::cout
            << "GS_UNSUPPORTED_TRANSFER[" << i << "] REASON="
            << failed.reason
            << " BITBLTBUF=0x" << std::hex << std::uppercase
            << failed.bitbltbuf
            << " TRXPOS=0x" << failed.trxpos
            << " TRXREG=0x" << failed.trxreg
            << " TRXDIR=0x" << failed.trxdir
            << std::dec << '\n';
    }

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
        << " ALPHA_1=0x" << gs.register_value(0x42u)
        << " TEST_1=0x" << gs.register_value(0x47u)
        << " PRMODECONT=0x" << gs.register_value(0x1Au)
        << " PRMODE=0x" << gs.register_value(0x1Bu)
        << " COLCLAMP=0x" << gs.register_value(0x46u)
        << std::dec << '\n';

    if (gs_stats.first_nonzero_input_valid) {
        std::cout
            << "GS_FIRST_COLOR_INPUT ALPHA=0x" << std::hex
            << std::uppercase << gs_stats.first_nonzero_input_alpha
            << " TEST=0x" << gs_stats.first_nonzero_input_test
            << " FRAME=0x" << gs_stats.first_nonzero_input_frame
            << " PRIM=0x" << gs_stats.first_nonzero_input_prim
            << " RGBAQ=0x" << gs_stats.first_nonzero_input_rgbaq
            << " INPUT_RGBA=0x" << gs_stats.first_nonzero_input_rgba
            << " TEX0=0x" << gs_stats.first_nonzero_input_tex0
            << " TEXA=0x" << gs_stats.first_nonzero_input_texa
            << " ST=0x" << gs_stats.first_nonzero_input_st
            << " UV=0x" << gs_stats.first_nonzero_input_uv
            << std::dec << '\n';

        std::cout
            << "GS_FIRST_TEXTURE_SAMPLE X="
            << gs_stats.first_texture_sample_x
            << " Y=" << gs_stats.first_texture_sample_y
            << " RGBA=0x" << std::hex << std::uppercase
            << gs_stats.first_texture_sample_rgba
            << std::dec << '\n';

        const ps2::u64 tex0 = gs_stats.first_nonzero_input_tex0;
        const ps2::u32 bp = static_cast<ps2::u32>(tex0 & 0x3FFFu);
        const ps2::u32 bw = static_cast<ps2::u32>((tex0 >> 14) & 0x3Fu);
        const ps2::u32 psm = static_cast<ps2::u32>((tex0 >> 20) & 0x3Fu);
        const ps2::u32 width = 1u << ((tex0 >> 26) & 0xFu);
        const ps2::u32 height = 1u << ((tex0 >> 30) & 0xFu);
        if (bw != 0u && width <= 1024u && height <= 1024u &&
            ps2::GsVram::supported_color_psm(psm)) {
            ps2::u64 color_pixels = 0;
            ps2::u64 alpha_pixels = 0;
            for (ps2::u32 y = 0; y < height; ++y) {
                for (ps2::u32 x = 0; x < width; ++x) {
                    const ps2::u32 raw = gs.vram().read_pixel(
                        psm, x, y, bp, bw);
                    if ((raw & 0x00FFFFFFu) != 0u) ++color_pixels;
                    if ((raw & 0xFF000000u) != 0u) ++alpha_pixels;
                }
            }
            std::cout
                << "GS_FIRST_TEXTURE COLOR_PIXELS=" << color_pixels
                << " ALPHA_PIXELS=" << alpha_pixels
                << " WIDTH=" << width
                << " HEIGHT=" << height
                << '\n';
        }
    }
    if (gs_stats.first_alpha_input_valid) {
        std::cout
            << "GS_FIRST_ALPHA_INPUT ALPHA=0x" << std::hex
            << std::uppercase << gs_stats.first_alpha_input_alpha
            << " PRIM=0x" << gs_stats.first_alpha_input_prim
            << " TEX0=0x" << gs_stats.first_alpha_input_tex0
            << " RGBAQ=0x" << gs_stats.first_alpha_input_rgbaq
            << " INPUT_RGBA=0x" << gs_stats.first_alpha_input_rgba
            << std::dec << '\n';
    }

    const auto& display = system.gs_display();
    ps2::u64 framebuffer_hash = 1469598103934665603ull;
    for (const ps2::u32 pixel : display.rgba8()) {
        framebuffer_hash ^= pixel;
        framebuffer_hash *= 1099511628211ull;
    }

    std::cout
        << "VISIBLE_FRAME_READY=" << (visible_frame_ready(system) ? 1 : 0)
        << " DISPLAY_VALID=" << (display.valid() ? 1 : 0)
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

    if (display.nonzero_pixel_count() != 0) {
        for (std::size_t i = 0; i < display.rgba8().size(); ++i) {
            const ps2::u32 pixel = display.rgba8()[i];
            if ((pixel & 0x00FFFFFFu) == 0) continue;
            std::cout
                << "DISPLAY_FIRST_NONZERO_INDEX=" << i
                << " DISPLAY_FIRST_NONZERO_RGBA=0x"
                << std::hex << std::uppercase << pixel
                << std::dec << '\n';
            break;
        }
    }

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

    if (display.valid()) {
        ps2::u64 nonzero_vram_bytes = 0;
        for (const ps2::u8 byte : gs.vram().data()) {
            if (byte != 0u) ++nonzero_vram_bytes;
        }
        auto count_nonzero = [&](ps2::u32 bp, ps2::u32 bw, ps2::u32 psm) {
            ps2::u64 count = 0;
            for (ps2::u32 y = 0; y < display.height(); ++y) {
                for (ps2::u32 x = 0; x < display.width(); ++x) {
                    if ((gs.vram().read_pixel(psm, x, y, bp, bw) &
                         0x00FFFFFFu) != 0u) {
                        ++count;
                    }
                }
            }
            return count;
        };
        const ps2::u64 frame1 = gs.register_value(0x4Cu);
        const ps2::u32 draw_bp = static_cast<ps2::u32>(frame1 & 0x1FFu) << 5;
        const ps2::u32 draw_bw = static_cast<ps2::u32>((frame1 >> 16) & 0x3Fu);
        const ps2::u32 draw_psm = static_cast<ps2::u32>((frame1 >> 24) & 0x3Fu);
        const ps2::u32 display_bp = static_cast<ps2::u32>(dispfb2 & 0x1FFu) << 5;
        const ps2::u32 display_bw = static_cast<ps2::u32>((dispfb2 >> 9) & 0x3Fu);
        const ps2::u32 display_psm = static_cast<ps2::u32>((dispfb2 >> 15) & 0x1Fu);
        std::cout
            << "GS_DRAW_FRAME_NONZERO="
            << count_nonzero(draw_bp, draw_bw, draw_psm)
            << " GS_DISPLAY_FRAME_NONZERO="
            << count_nonzero(display_bp, display_bw, display_psm)
            << " GS_VRAM_NONZERO_BYTES=" << nonzero_vram_bytes
            << '\n';
    }
}

bool write_display_ppm(
    const char* path,
    const ps2::GsDisplay& display) {
    if (path == nullptr || !display.valid()) return false;
    std::ofstream out(path, std::ios::binary);
    if (!out) return false;
    out << "P6\n" << display.width() << ' ' << display.height()
        << "\n255\n";
    for (const ps2::u32 pixel : display.rgba8()) {
        const char rgb[3] = {
            static_cast<char>(pixel),
            static_cast<char>(pixel >> 8),
            static_cast<char>(pixel >> 16),
        };
        out.write(rgb, sizeof(rgb));
    }
    return static_cast<bool>(out);
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr
            << "usage: vibestation_ps2_bios_trace <bios.bin> "
               "[ee-instruction-budget] [display.ppm] "
               "[--ee-jit|--profile|--gs-thread|--detailed-gs-stats|--audio-only] "
               "[--wav=audio.wav]\n";
        return 64;
    }

    constexpr ps2::u64 kDefaultBudget = 20'000'000;
    // A million EE instructions is well under one emulated video frame, and
    // avoids repeatedly decoding a full 640x448 display for tiny chunks.
    constexpr ps2::u64 kChunk = 1'000'000;

    const ps2::u64 budget =
        parse_budget(argc >= 3 ? argv[2] : nullptr, kDefaultBudget);
    bool ee_jit = false;
    bool pc_samples = false;
    bool profile = false;
    bool gs_thread = false;
    bool detailed_gs_stats = false;
    bool audio_only = false;
    const char* display_path = nullptr;
    std::string wav_path;
    for (int index = 3; index < argc; ++index) {
        const std::string_view option(argv[index]);
        if (option == "--ee-jit") ee_jit = true;
        else if (option == "--pc-samples") pc_samples = true;
        else if (option == "--profile") profile = true;
        else if (option == "--gs-thread") gs_thread = true;
        else if (option == "--detailed-gs-stats") detailed_gs_stats = true;
        else if (option == "--audio-only") audio_only = true;
        else if (option.starts_with("--wav=") && option.size() > 6u) {
            wav_path = std::string(option.substr(6));
        }
        else if (display_path == nullptr && !option.starts_with("--")) {
            display_path = argv[index];
        } else {
            std::cerr << "Unknown or duplicate trace option: " << option << '\n';
            return 64;
        }
    }

    ps2::Ps2System system;
    system.ee().set_jit_enabled(ee_jit);
    std::cout << "EE_BACKEND="
              << (ee_jit ? "experimental-x64-jit" : "cached-interpreter")
              << '\n';
    system.gs_core().set_async_rasterization(gs_thread);
    system.gs_core().set_detailed_raster_stats(detailed_gs_stats);
    system.gs_core().set_raster_timing_enabled(profile);
    system.gs_core().set_rasterization_enabled(!audio_only);
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
    bool first_visible_reported = false;
    std::chrono::steady_clock::time_point first_visible_time{};
    std::clock_t first_visible_cpu = 0;
    ps2::u64 first_visible_field = 0;
    std::chrono::nanoseconds run_time{};
    std::chrono::nanoseconds display_time{};
    std::vector<ps2::s16> captured_pcm;
    const auto wall_start = std::chrono::steady_clock::now();
    while (remaining > 0 && !system.halted()) {
        const ps2::u64 request =
            remaining < kChunk ? remaining : kChunk;
        const auto run_begin = std::chrono::steady_clock::now();
        const ps2::u64 ran = system.run_ee(request, error);
        if (profile) run_time += std::chrono::steady_clock::now() - run_begin;
        if (ran > remaining) {
            break;
        }
        remaining -= ran;
        // VBlank already refreshes the composed display at the correct
        // emulated boundary. In threaded profile runs, forcing another full
        // scanout every 1M EE instructions drains the async raster worker and
        // measures synchronization overhead instead of emulation throughput.
        if (!audio_only && !(profile && gs_thread)) {
            const auto display_begin = std::chrono::steady_clock::now();
            system.refresh_display();
            if (profile) {
                display_time +=
                    std::chrono::steady_clock::now() - display_begin;
            }
        }

        if (!wav_path.empty()) {
            const std::size_t queued =
                system.spu2().queued_frames();
            if (queued != 0u) {
                auto pcm =
                    system.spu2().take_samples(queued);
                captured_pcm.insert(
                    captured_pcm.end(),
                    pcm.begin(),
                    pcm.end());
            }
        }

        if (!audio_only &&
            !first_visible_reported &&
            system.gs_display().nonzero_pixel_count() != 0u) {
            first_visible_reported = true;
            first_visible_time = std::chrono::steady_clock::now();
            first_visible_cpu = std::clock();
            first_visible_field = system.video_fields_started();
            std::cerr
                << "TRACE_FIRST_VISIBLE EE=" << (budget - remaining)
                << " NONZERO="
                << system.gs_display().nonzero_pixel_count()
                << " WALL_MS=" << std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - wall_start).count()
                << '\n';
        }

        if (!error.empty()) {
            std::cerr << "TRACE_ERROR=" << error << '\n';
            print_state(system);
            return 5;
        }

        const ps2::u64 executed = budget - remaining;
        if (pc_samples) {
            std::cerr << "TRACE_PC EE=" << executed
                << " PC=0x" << std::hex << std::uppercase
                << system.ee().state().pc
                << " IOP_PC=0x" << system.iop().state().pc
                << std::dec << " IOP_HALTED=" << system.iop_halted()
                << " PURE_RUN=" << simple_register_run(system)
                << '\n';
        }
        if (executed >= 200'000'000u &&
            (executed % 10'000'000u) == 0u) {
            std::cerr
                << "TRACE_PROGRESS EE=" << executed
                << " PC=0x" << std::hex << std::uppercase
                << system.ee().state().pc
                << std::dec
                << " GIF_QWORDS="
                << system.gs_core().submitted_gif_qwords()
                << " PRIMITIVES="
                << system.gs_core().submitted_primitives();
            if (gs_thread) {
                // Worker-owned raster counters are intentionally deferred
                // until final print_state(), which performs one synchronization.
                std::cerr << " RASTER_STATS=deferred";
            } else {
                const auto& stats = system.gs_core().stats();
                std::cerr
                    << " IMAGE_QWORDS=" << stats.image_qwords
                    << " DRAWS=" << stats.raster_draws
                    << " PIXELS=" << stats.raster_pixels;
            }
            std::cerr
                << " WALL_MS=" << std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - wall_start).count()
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

    const auto wall_end = std::chrono::steady_clock::now();
    const auto cpu_end = std::clock();
    print_state(system);
    if (profile) {
        if (first_visible_reported) {
            const double visible_seconds =
                std::chrono::duration<double>(
                    wall_end - first_visible_time).count();
            const ps2::u64 visible_fields =
                system.video_fields_started() - first_visible_field;
            const double visible_cpu_seconds =
                static_cast<double>(cpu_end - first_visible_cpu) /
                CLOCKS_PER_SEC;
            std::cout << "PROFILE_VISIBLE_FIELDS=" << visible_fields
                << " PROFILE_VISIBLE_SECONDS=" << visible_seconds
                << " PROFILE_FIELD_RATE="
                << (visible_seconds > 0 ? visible_fields / visible_seconds : 0.0)
                << " PROFILE_FIELD_CPU_RATE="
                << (visible_cpu_seconds > 0 ? visible_fields / visible_cpu_seconds : 0.0)
                << '\n';
        }
        std::cout << "PROFILE_RUN_MS="
            << std::chrono::duration_cast<std::chrono::milliseconds>(run_time).count()
            << " PROFILE_DISPLAY_MS="
            << std::chrono::duration_cast<std::chrono::milliseconds>(display_time).count()
            << '\n';
    }
    std::cout << "EE_SKIPPED_BIOS_IDLE_ITERATIONS="
              << system.skipped_bios_idle_iterations()
              << " EE_SKIPPED_BIOS_IDLE_OFFPHASE_BATCHES="
              << system.skipped_bios_idle_offphase_batches() << '\n';
    std::cout << "EE_SKIPPED_BIOS_ZERO_ITERATIONS="
              << system.skipped_bios_zero_iterations() << '\n';
    std::cout << "EE_SKIPPED_BIOS_NIBBLE_ITERATIONS="
              << system.skipped_bios_nibble_iterations() << '\n';
    std::cout << "EE_SKIPPED_BIOS_COUNT_WAIT_ITERATIONS="
              << system.skipped_bios_count_wait_iterations() << '\n';
    std::cout << "EE_SKIPPED_BIOS_COUNTDOWN_ITERATIONS="
              << system.skipped_bios_countdown_iterations() << '\n';
    std::cout << "EE_SKIPPED_BIOS_COPY_ITERATIONS="
              << system.skipped_bios_copy_iterations() << '\n';
    std::cout << "EE_SKIPPED_BIOS_MMIO_POLL_ITERATIONS="
              << system.skipped_bios_mmio_poll_iterations() << '\n';
    std::cout << "EE_SIF_POLL_FAST_SAMPLES="
              << system.sif_poll_fast_samples()
              << " EE_SIF_POLL_STABLE_RETURNS="
              << system.sif_poll_stable_returns()
              << " EE_FAST_SIF_GETREG_CALLS="
              << system.fast_sif_getreg_calls() << '\n';
    std::cout << "EE_FAST_SIF_GETREG_REJECTS";
    const auto& sif_rejects =
        system.fast_sif_getreg_rejects();
    for (ps2::u32 i = 0; i < sif_rejects.size(); ++i) {
        if (sif_rejects[i] != 0u) {
            std::cout << " [" << i << "]=" << sif_rejects[i];
        }
    }
    std::cout << '\n';
    std::cout << "IOP_SKIPPED_IDLE_PAIRS="
              << system.skipped_iop_idle_pairs() << '\n';
    std::cout << "EE_SKIPPED_BIOS_LITERAL_ITERATIONS="
              << system.skipped_bios_literal_iterations() << '\n';
    std::cout << "EE_QUIET_BATCH_INSTRUCTIONS="
              << system.quiet_ee_batch_instructions()
              << " EE_QUIET_ACTIVE_IOP_INSTRUCTIONS="
              << system.quiet_ee_active_iop_instructions()
              << " EE_QUIET_BATCHES="
              << system.quiet_ee_batches()
              << " EE_BLOCK_INSTRUCTIONS="
              << system.quiet_block_instructions()
              << " EE_BLOCK_HITS="
              << system.quiet_block_hits()
              << " EE_BLOCK_COMPILES="
              << system.quiet_block_compiles()
              << " EE_FAST_INTERPRETER_INSTRUCTIONS="
              << system.fast_interpreter_instructions()
              << " EE_FAST_INTERPRETER_CALLS="
              << system.fast_interpreter_calls()
              << " EE_QUIET_SUPERBATCH_CALLS="
              << system.quiet_superbatch_calls()
              << " EE_QUIET_SUPERBATCH_INSTRUCTIONS="
              << system.quiet_superbatch_instructions() << '\n';
    std::cout << "EE_QUIET_HOT_BLOCKS";
    for (const auto& [pc, hits] : system.quiet_block_hotspots(32u)) {
        std::cout
            << " PC_0x" << std::hex << std::uppercase << pc
            << std::dec << "=" << hits;
    }
    std::cout << '\n';

    for (const auto& [pc, hits] : system.quiet_block_hotspots(20u)) {
        std::cout
            << "EE_HOT_CODE PC=0x" << std::hex << std::uppercase << pc
            << std::dec << " HITS=" << hits;
        for (int word = -4; word <= 12; ++word) {
            const ps2::u32 address =
                pc + static_cast<ps2::u32>(word * 4);
            ps2::u32 opcode = 0;
            if (system.bus().read32(address, opcode)) {
                std::cout
                    << " A" << (word >= 0 ? "+" : "") << word * 4
                    << "=0x" << std::hex << std::uppercase << opcode
                    << std::dec;
            }
        }
        std::cout << '\n';
    }

    std::cout << "EE_JIT_BLOCK_INSTRUCTIONS="
              << system.ee().jit().block_instruction_count()
              << " EE_JIT_BLOCK_EXECUTIONS="
              << system.ee().jit().block_executed_count()
              << " EE_JIT_BLOCK_COMPILES="
              << system.ee().jit().block_compiled_count()
              << " EE_JIT_FASTMEM_LOADS="
              << system.ee().jit().block_fastmem_load_count()
              << " EE_JIT_FASTMEM_STORES="
              << system.ee().jit().block_fastmem_store_count()
              << " EE_JIT_CODE_STORE_EXITS="
              << system.ee().jit().block_code_store_exit_count()
              << " EE_JIT_GUARD_BAILOUTS="
              << system.ee().jit().block_guard_bailout_count()
              << " EE_JIT_CACHE_FLUSHES="
              << system.ee().jit().cache_flush_count() << '\n';
    auto fallback_opcodes = system.native_fallback_opcodes();
    std::cout << "EE_NATIVE_FALLBACK_TOP";
    for (ps2::u32 rank = 0u; rank < 8u; ++rank) {
        ps2::u32 best_opcode = 0u;
        ps2::u64 best_count = 0u;
        for (ps2::u32 opcode = 0u; opcode < fallback_opcodes.size(); ++opcode) {
            if (fallback_opcodes[opcode] > best_count) {
                best_opcode = opcode;
                best_count = fallback_opcodes[opcode];
            }
        }
        if (best_count == 0u) break;
        std::cout << " 0x" << std::hex << best_opcode
                  << std::dec << ':' << best_count;
        fallback_opcodes[best_opcode] = 0u;
    }
    std::cout << '\n';

    const auto& idle_reasons = system.idle_skip_reasons();
    std::cout << "EE_IDLE_SKIP_REASONS";
    for (auto count : idle_reasons) std::cout << ' ' << count;
    std::cout << '\n';

    if (ee_jit) {
        std::cout << "EE_JIT_COMPILED=" << system.ee().jit().compiled_count()
                  << " EE_JIT_EXECUTED="
                  << system.ee().jit().executed_count() << '\n';
    }

    if (display_path != nullptr) {
        if (!write_display_ppm(display_path, system.gs_display())) {
            std::cerr << "DISPLAY_DUMP_ERROR=" << display_path << '\n';
        } else {
            std::cout << "DISPLAY_DUMP=" << display_path << '\n';
        }
    }

    if (!wav_path.empty()) {
        const std::size_t queued =
            system.spu2().queued_frames();
        if (queued != 0u) {
            auto pcm = system.spu2().take_samples(queued);
            captured_pcm.insert(
                captured_pcm.end(),
                pcm.begin(),
                pcm.end());
        }

        print_audio_stats(captured_pcm);
        if (!write_pcm16_wav(
                wav_path.c_str(),
                captured_pcm)) {
            std::cerr
                << "SPU2_WAV_DUMP_ERROR="
                << wav_path << '\n';
        } else {
            std::cout
                << "SPU2_WAV_DUMP="
                << wav_path << '\n';
        }
    }

    if (system.halted()) {
        std::cout << "HALT_REASON=" << system.halt_reason() << '\n';
        return 10;
    }

    std::cout
        << "TRACE_BUDGET_EXHAUSTED=" << budget << '\n';
    return 0;
}
