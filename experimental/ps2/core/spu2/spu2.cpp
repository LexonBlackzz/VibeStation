#include "core/spu2/spu2.h"

#include "core/iop/iop_ram.h"

#include <algorithm>
#include <cstdint>

namespace ps2 {
namespace {

constexpr u32 kCoreStride = 0x400u;
constexpr u32 kVoiceParamStride = 0x10u;
constexpr u32 kVoiceAddrStride = 0x0Cu;

constexpr u32 kVoiceVolL = 0x000u;
constexpr u32 kVoiceVolR = 0x002u;
constexpr u32 kVoicePitch = 0x004u;

constexpr u32 kKeyOn = 0x1A0u;
constexpr u32 kKeyOff = 0x1A4u;
constexpr u32 kTransferAddr = 0x1A8u;
constexpr u32 kTransferData = 0x1ACu;
constexpr u32 kTransferData2 = 0x1AEu;
constexpr u32 kVoiceStartAddr = 0x1C0u;
constexpr u32 kVoiceLoopAddr = 0x1C4u;
constexpr u32 kVoiceNextAddr = 0x1C8u;
constexpr u32 kStatx = 0x344u;

constexpr u8 kLoopEnd = 1u << 0;
constexpr u8 kLoopRepeat = 1u << 1;
constexpr u8 kLoopStart = 1u << 2;

constexpr s32 kPredictor[5][2] = {
    {0, 0},
    {60, 0},
    {115, -52},
    {98, -55},
    {122, -60},
};

s16 clamp16(s32 value) {
    return static_cast<s16>(
        std::clamp<s32>(value, -32768, 32767));
}

s32 apply_volume(s16 sample, u16 reg) {
    // Static SPU2 volume is normally expressed around 0x3fff = unity.
    // Volume sweeps are deliberately approximated by their magnitude in this
    // first dry mixer rather than attempting timing-sensitive slide behavior.
    s32 volume = static_cast<s16>(reg);
    if ((reg & 0x8000u) != 0u)
        volume = static_cast<s32>(reg & 0x7FFFu);
    return (static_cast<s32>(sample) * volume) >> 14;
}

} // namespace

Spu2::Spu2()
    : ram_(kRamHalfwords, 0) {
    reset();
}

void Spu2::reset() {
    regs_.fill(0);
    std::fill(ram_.begin(), ram_.end(), 0);
    cores_ = {};
    cycle_phase_ = 0;
    pcm_queue_.clear();

    // Keep the bootstrap-visible reset value at zero. DMA completion will
    // raise the transfer-ready bit just like the earlier compatibility path.
    set_raw16(kStatx, 0u);
    set_raw16(kCoreStride + kStatx, 0u);
}

u16 Spu2::raw16(u32 offset) const {
    if (offset + 1u >= regs_.size()) return 0;
    return static_cast<u16>(regs_[offset]) |
           (static_cast<u16>(regs_[offset + 1u]) << 8);
}

void Spu2::set_raw16(u32 offset, u16 value) {
    if (offset + 1u >= regs_.size()) return;
    regs_[offset] = static_cast<u8>(value);
    regs_[offset + 1u] = static_cast<u8>(value >> 8);
}

bool Spu2::read8(u32 offset, u8& value) const {
    if (offset >= regs_.size()) return false;
    value = regs_[offset];
    return true;
}

bool Spu2::read16(u32 offset, u16& value) const {
    if (offset + 2u > regs_.size()) return false;
    value = raw16(offset);
    return true;
}

bool Spu2::read32(u32 offset, u32& value) const {
    if (offset + 4u > regs_.size()) return false;
    value = static_cast<u32>(raw16(offset)) |
            (static_cast<u32>(raw16(offset + 2u)) << 16);
    return true;
}

bool Spu2::write8(u32 offset, u8 value) {
    if (offset >= regs_.size()) return false;
    const u32 aligned = offset & ~1u;
    u16 current = raw16(aligned);
    if ((offset & 1u) != 0u)
        current = static_cast<u16>(
            (current & 0x00FFu) |
            (static_cast<u16>(value) << 8));
    else
        current = static_cast<u16>(
            (current & 0xFF00u) | value);
    return write16(aligned, current);
}

bool Spu2::write16(u32 offset, u16 value) {
    if (offset + 2u > regs_.size()) return false;
    set_raw16(offset, value);
    handle_register_write(offset, value);
    return true;
}

bool Spu2::write32(u32 offset, u32 value) {
    if (offset + 4u > regs_.size()) return false;
    return write16(offset, static_cast<u16>(value)) &&
           write16(offset + 2u, static_cast<u16>(value >> 16));
}

u32 Spu2::address_register(
    u32 core,
    u32 base,
    u32 voice) const {
    const u32 offset =
        core * kCoreStride + base + voice * kVoiceAddrStride;
    // SPU2 exposes the high halfword first, low halfword second.
    return ((static_cast<u32>(raw16(offset)) & 0xFu) << 16) |
           raw16(offset + 2u);
}

void Spu2::refresh_transfer_address(u32 core) {
    if (core >= cores_.size()) return;
    cores_[core].transfer_addr =
        address_register(core, kTransferAddr) & 0xFFFFFu;
}

void Spu2::handle_register_write(u32 offset, u16 value) {
    const u32 core = (offset >= kCoreStride) ? 1u : 0u;
    const u32 local = offset - core * kCoreStride;

    if (local == kTransferAddr ||
        local == kTransferAddr + 2u) {
        refresh_transfer_address(core);
        return;
    }

    if (local == kTransferData ||
        local == kTransferData2) {
        Core& c = cores_[core];
        ram_[c.transfer_addr & 0xFFFFFu] = value;
        c.transfer_addr = (c.transfer_addr + 1u) & 0xFFFFFu;
        set_raw16(
            core * kCoreStride + kTransferAddr,
            static_cast<u16>((c.transfer_addr >> 16) & 0xFu));
        set_raw16(
            core * kCoreStride + kTransferAddr + 2u,
            static_cast<u16>(c.transfer_addr));
        return;
    }

    if (local == kKeyOn) {
        key_on(core, value, 0u);
    } else if (local == kKeyOn + 2u) {
        key_on(core, value & 0xFFu, 16u);
    } else if (local == kKeyOff) {
        key_off(core, value, 0u);
    } else if (local == kKeyOff + 2u) {
        key_off(core, value & 0xFFu, 16u);
    }
}

void Spu2::key_on(u32 core, u32 mask, u32 first_voice) {
    if (core >= cores_.size()) return;
    for (u32 bit = 0; bit < 16u && first_voice + bit < 24u; ++bit) {
        if ((mask & (1u << bit)) == 0u) continue;
        Voice& v = cores_[core].voices[first_voice + bit];
        v = {};
        v.active = true;
        v.current_addr =
            address_register(core, kVoiceStartAddr, first_voice + bit) &
            0xFFFF8u;
        v.loop_addr =
            address_register(core, kVoiceLoopAddr, first_voice + bit) &
            0xFFFF8u;
        if (v.loop_addr == 0u) v.loop_addr = v.current_addr;
        v.decoded_pos = 28u;
    }
}

void Spu2::key_off(u32 core, u32 mask, u32 first_voice) {
    if (core >= cores_.size()) return;
    for (u32 bit = 0; bit < 16u && first_voice + bit < 24u; ++bit) {
        if ((mask & (1u << bit)) != 0u)
            cores_[core].voices[first_voice + bit].active = false;
    }
}

bool Spu2::decode_block(u32 core, u32 voice_index) {
    if (core >= cores_.size() || voice_index >= 24u) return false;
    Voice& voice = cores_[core].voices[voice_index];
    if (!voice.active) return false;

    const u32 address = voice.current_addr & 0xFFFF8u;
    const u16 header = ram_[address & 0xFFFFFu];
    const u32 shift = header & 0xFu;
    const u32 predictor_id = (header >> 4) & 0xFu;
    const u32 predictor = predictor_id <= 4u ? predictor_id : 0u;
    voice.block_flags = static_cast<u8>(header >> 8);

    if ((voice.block_flags & kLoopStart) != 0u)
        voice.loop_addr = address;

    u32 out = 0;
    for (u32 byte_index = 0; byte_index < 14u; ++byte_index) {
        const u16 packed =
            ram_[(address + 1u + byte_index / 2u) & 0xFFFFFu];
        const u8 byte = static_cast<u8>(
            packed >> ((byte_index & 1u) * 8u));

        for (u32 nibble_index = 0; nibble_index < 2u; ++nibble_index) {
            s32 nibble =
                (byte >> (nibble_index * 4u)) & 0xFu;
            if ((nibble & 8) != 0) nibble -= 16;

            s32 sample = (nibble << 12) >> shift;
            sample += (
                kPredictor[predictor][0] * voice.prev1 +
                kPredictor[predictor][1] * voice.prev2 +
                32) >> 6;
            sample = std::clamp<s32>(sample, -32768, 32767);

            voice.decoded[out++] = static_cast<s16>(sample);
            voice.prev2 = voice.prev1;
            voice.prev1 = sample;
        }
    }

    voice.decoded_pos = 0;
    const u32 addr_reg =
        core * kCoreStride +
        kVoiceNextAddr +
        voice_index * kVoiceAddrStride;
    set_raw16(
        addr_reg,
        static_cast<u16>((voice.current_addr >> 16) & 0xFu));
    set_raw16(addr_reg + 2u, static_cast<u16>(voice.current_addr));
    return true;
}

s16 Spu2::voice_sample(u32 core, u32 voice_index) {
    Voice& voice = cores_[core].voices[voice_index];
    if (!voice.active) return 0;
    if (voice.decoded_pos >= voice.decoded.size() &&
        !decode_block(core, voice_index)) {
        return 0;
    }
    return voice.decoded[voice.decoded_pos];
}

void Spu2::advance_voice(u32 core, u32 voice_index) {
    Voice& voice = cores_[core].voices[voice_index];
    if (!voice.active) return;

    ++voice.decoded_pos;
    if (voice.decoded_pos < voice.decoded.size()) return;

    if ((voice.block_flags & kLoopEnd) != 0u) {
        if ((voice.block_flags & kLoopRepeat) != 0u) {
            voice.current_addr = voice.loop_addr & 0xFFFF8u;
        } else {
            voice.active = false;
            return;
        }
    } else {
        voice.current_addr =
            (voice.current_addr + 8u) & 0xFFFFFu;
    }

    voice.decoded_pos = 28u;
    (void)decode_block(core, voice_index);
}

void Spu2::mix_one_sample() {
    s32 left = 0;
    s32 right = 0;

    for (u32 core = 0; core < 2u; ++core) {
        for (u32 voice_index = 0; voice_index < 24u; ++voice_index) {
            Voice& voice = cores_[core].voices[voice_index];
            if (!voice.active) continue;

            const u32 base =
                core * kCoreStride +
                voice_index * kVoiceParamStride;
            const u16 pitch = raw16(base + kVoicePitch) & 0x3FFFu;
            const s16 sample = voice_sample(core, voice_index);

            left += apply_volume(
                sample,
                raw16(base + kVoiceVolL));
            right += apply_volume(
                sample,
                raw16(base + kVoiceVolR));

            voice.phase += pitch;
            while (voice.active && voice.phase >= 0x1000u) {
                voice.phase -= 0x1000u;
                advance_voice(core, voice_index);
            }
        }
    }

    push_sample(clamp16(left), clamp16(right));
}

void Spu2::push_sample(s16 left, s16 right) {
    constexpr std::size_t kMaxQueuedFrames = kSampleRate;
    while (pcm_queue_.size() / 2u >= kMaxQueuedFrames) {
        pcm_queue_.pop_front();
        pcm_queue_.pop_front();
    }
    pcm_queue_.push_back(left);
    pcm_queue_.push_back(right);
}

void Spu2::tick(u64 iop_cycles) {
    cycle_phase_ += iop_cycles;
    while (cycle_phase_ >= kIopCyclesPerSample) {
        cycle_phase_ -= kIopCyclesPerSample;
        mix_one_sample();
    }
}

bool Spu2::dma_write(
    u32 core,
    const IopRam& iop_ram,
    u32 madr,
    u32 halfwords) {
    if (core >= cores_.size()) return false;
    Core& c = cores_[core];

    const u32 ram_mask =
        static_cast<u32>(IopRam::kSize - 1u);
    for (u32 i = 0; i < halfwords; ++i) {
        u16 value = 0;
        const u32 source =
            (madr + i * 2u) & ram_mask;
        if (!iop_ram.read16(source, value)) return false;
        ram_[c.transfer_addr & 0xFFFFFu] = value;
        c.transfer_addr =
            (c.transfer_addr + 1u) & 0xFFFFFu;
    }

    set_raw16(
        core * kCoreStride + kTransferAddr,
        static_cast<u16>((c.transfer_addr >> 16) & 0xFu));
    set_raw16(
        core * kCoreStride + kTransferAddr + 2u,
        static_cast<u16>(c.transfer_addr));
    return true;
}

bool Spu2::dma_read(
    u32 core,
    IopRam& iop_ram,
    u32 madr,
    u32 halfwords) {
    if (core >= cores_.size()) return false;
    Core& c = cores_[core];

    const u32 ram_mask =
        static_cast<u32>(IopRam::kSize - 1u);
    for (u32 i = 0; i < halfwords; ++i) {
        const u32 destination =
            (madr + i * 2u) & ram_mask;
        if (!iop_ram.write16(
                destination,
                ram_[c.transfer_addr & 0xFFFFFu])) {
            return false;
        }
        c.transfer_addr =
            (c.transfer_addr + 1u) & 0xFFFFFu;
    }

    set_raw16(
        core * kCoreStride + kTransferAddr,
        static_cast<u16>((c.transfer_addr >> 16) & 0xFu));
    set_raw16(
        core * kCoreStride + kTransferAddr + 2u,
        static_cast<u16>(c.transfer_addr));
    return true;
}

std::vector<s16> Spu2::take_samples(std::size_t max_frames) {
    const std::size_t samples =
        std::min<std::size_t>(
            pcm_queue_.size(),
            max_frames * 2u);
    std::vector<s16> out;
    out.reserve(samples);
    for (std::size_t i = 0; i < samples; ++i) {
        out.push_back(pcm_queue_.front());
        pcm_queue_.pop_front();
    }
    return out;
}

} // namespace ps2
