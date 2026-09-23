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
constexpr u32 kVoiceAdsr1 = 0x006u;
constexpr u32 kVoiceAdsr2 = 0x008u;
constexpr u32 kVoiceEnvx = 0x00Au;
constexpr u32 kVoiceVolxL = 0x00Cu;
constexpr u32 kVoiceVolxR = 0x00Eu;

constexpr u32 kVmixL = 0x188u;
constexpr u32 kVmixEl = 0x18Cu;
constexpr u32 kVmixR = 0x190u;
constexpr u32 kVmixEr = 0x194u;
constexpr u32 kKeyOn = 0x1A0u;
constexpr u32 kKeyOff = 0x1A4u;
constexpr u32 kTransferAddr = 0x1A8u;
constexpr u32 kTransferData = 0x1ACu;
constexpr u32 kTransferData2 = 0x1AEu;
constexpr u32 kVoiceStartAddr = 0x1C0u;
constexpr u32 kVoiceLoopAddr = 0x1C4u;
constexpr u32 kVoiceNextAddr = 0x1C8u;
constexpr u32 kEndx = 0x340u;
constexpr u32 kStatx = 0x344u;

constexpr u32 kMasterVolL0 = 0x760u;
constexpr u32 kMasterVolR0 = 0x762u;
constexpr u32 kMasterVolL1 = 0x788u;
constexpr u32 kMasterVolR1 = 0x78Au;

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

s32 volume_register_value(u16 reg) {
    if ((reg & 0x8000u) == 0u) {
        // Static SPU2 volume is a signed 15-bit value. Hardware expands it
        // to the signed 16-bit mixer domain by shifting once.
        return static_cast<s16>(
            static_cast<u16>(reg << 1u));
    }

    // Volume slides are uncommon during BIOS startup. Until the slide engine
    // is implemented, preserve the encoded magnitude instead of muting it.
    return static_cast<s32>(reg & 0x7FFFu);
}

s32 apply_volume(s32 sample, u16 reg) {
    return (sample * volume_register_value(reg)) >> 15;
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

    // Keep the bootstrap-visible STATX reset value at zero. DMA completion
    // raises the transfer-ready bit just like the earlier compatibility path.
    set_raw16(kStatx, 0u);
    set_raw16(kCoreStride + kStatx, 0u);

    for (u32 core = 0; core < 2u; ++core) {
        const u32 base = core * kCoreStride;
        for (u32 gate : {kVmixL, kVmixEl, kVmixR, kVmixEr}) {
            set_raw16(base + gate, 0xFFFFu);
            set_raw16(base + gate + 2u, 0x00FFu);
        }
        cores_[core].endx = 0x00FFFFFFu;
        write_endx(core);
    }
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
        v.envelope_phase = EnvelopePhase::Attack;
        v.envelope_counter = 0;
        v.envelope_value = 0;
        v.current_addr =
            address_register(core, kVoiceStartAddr, first_voice + bit) &
            0xFFFF8u;
        v.loop_addr =
            address_register(core, kVoiceLoopAddr, first_voice + bit) &
            0xFFFF8u;
        if (v.loop_addr == 0u) v.loop_addr = v.current_addr;
        v.decoded_pos = 28u;
        cores_[core].endx &= ~(1u << (first_voice + bit));
        write_endx(core);
    }
}

void Spu2::key_off(u32 core, u32 mask, u32 first_voice) {
    if (core >= cores_.size()) return;
    for (u32 bit = 0; bit < 16u && first_voice + bit < 24u; ++bit) {
        if ((mask & (1u << bit)) != 0u) {
            Voice& v = cores_[core].voices[first_voice + bit];
            if (v.active &&
                v.envelope_phase != EnvelopePhase::Stopped) {
                v.envelope_phase = EnvelopePhase::Release;
                v.envelope_counter = 0;
            }
        }
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

s16 Spu2::interpolated_voice_sample(
    u32 core,
    u32 voice_index) {
    Voice& voice = cores_[core].voices[voice_index];
    const s32 a = voice_sample(core, voice_index);
    if (!voice.active) return 0;

    // A small linear interpolator is enough to remove the zero-order-hold
    // stepping from pitched BIOS samples. A future full SPU2 pass can replace
    // this with the hardware's Gaussian four-tap interpolation.
    s32 b = a;
    if (voice.decoded_pos + 1u < voice.decoded.size())
        b = voice.decoded[voice.decoded_pos + 1u];

    const u32 frac = voice.phase & 0xFFFu;
    return clamp16(
        (a * static_cast<s32>(0x1000u - frac) +
         b * static_cast<s32>(frac)) >> 12);
}

void Spu2::advance_voice(u32 core, u32 voice_index) {
    Voice& voice = cores_[core].voices[voice_index];
    if (!voice.active) return;

    ++voice.decoded_pos;
    if (voice.decoded_pos < voice.decoded.size()) return;

    if ((voice.block_flags & kLoopEnd) != 0u) {
        cores_[core].endx |= 1u << voice_index;
        write_endx(core);

        if ((voice.block_flags & kLoopRepeat) != 0u) {
            voice.current_addr = voice.loop_addr & 0xFFFF8u;
        } else {
            voice.active = false;
            voice.envelope_phase = EnvelopePhase::Stopped;
            voice.envelope_value = 0;
            const u32 base =
                core * kCoreStride +
                voice_index * kVoiceParamStride;
            set_raw16(base + kVoiceEnvx, 0u);
            return;
        }
    } else {
        voice.current_addr =
            (voice.current_addr + 8u) & 0xFFFFFu;
    }

    voice.decoded_pos = 28u;
    (void)decode_block(core, voice_index);
}

void Spu2::update_envelope(
    u32 core,
    u32 voice_index) {
    Voice& voice = cores_[core].voices[voice_index];
    if (!voice.active ||
        voice.envelope_phase == EnvelopePhase::Stopped) {
        voice.envelope_value = 0;
        return;
    }

    const u32 base =
        core * kCoreStride +
        voice_index * kVoiceParamStride;
    const u16 adsr1 = raw16(base + kVoiceAdsr1);
    const u16 adsr2 = raw16(base + kVoiceAdsr2);

    bool decrease = false;
    bool exponential = false;
    u32 shift = 0;
    s32 step = 0;
    s32 target = 0;

    switch (voice.envelope_phase) {
    case EnvelopePhase::Attack:
        exponential = (adsr1 & 0x8000u) != 0u;
        shift = (adsr1 >> 10) & 0x1Fu;
        step = 7 - static_cast<s32>((adsr1 >> 8) & 0x3u);
        target = 0x7FFF;
        break;

    case EnvelopePhase::Decay:
        decrease = true;
        exponential = true;
        shift = (adsr1 >> 4) & 0xFu;
        step = -8;
        target = static_cast<s32>((adsr1 & 0xFu) + 1u) << 11;
        break;

    case EnvelopePhase::Sustain:
        decrease = (adsr2 & 0x4000u) != 0u;
        exponential = (adsr2 & 0x8000u) != 0u;
        shift = (adsr2 >> 8) & 0x1Fu;
        step = 7 - static_cast<s32>((adsr2 >> 6) & 0x3u);
        if (decrease) step = ~step;
        target = 0;
        break;

    case EnvelopePhase::Release:
        decrease = true;
        exponential = (adsr2 & 0x20u) != 0u;
        shift = adsr2 & 0x1Fu;
        step = -8;
        target = 0;
        break;

    case EnvelopePhase::Stopped:
        return;
    }

    const u32 shift_down = shift > 11u ? shift - 11u : 0u;
    u32 counter_inc =
        shift_down >= 16u ? 0u : (0x8000u >> shift_down);

    const u32 shift_up = shift < 11u ? 11u - shift : 0u;
    s32 level_inc =
        step * static_cast<s32>(1u << std::min<u32>(shift_up, 15u));

    if (exponential) {
        if (!decrease &&
            voice.envelope_value > 0x6000) {
            counter_inc >>= 2u;
        }

        if (decrease) {
            level_inc = static_cast<s16>(
                (level_inc * voice.envelope_value) >> 15);
        }
    }

    counter_inc = std::max<u32>(1u, counter_inc);
    voice.envelope_counter += counter_inc;

    if (voice.envelope_counter >= 0x8000u) {
        voice.envelope_counter = 0;
        voice.envelope_value = std::clamp<s32>(
            voice.envelope_value + level_inc,
            0,
            0x7FFF);
    }

    if (voice.envelope_phase == EnvelopePhase::Sustain) {
        if (voice.envelope_value == 0) {
            voice.envelope_phase = EnvelopePhase::Stopped;
            voice.active = false;
        }
    } else {
        const bool reached =
            (!decrease && voice.envelope_value >= target) ||
            (decrease && voice.envelope_value <= target);
        if (reached) {
            const u8 next =
                static_cast<u8>(voice.envelope_phase) + 1u;
            if (next >
                static_cast<u8>(EnvelopePhase::Release)) {
                voice.envelope_phase = EnvelopePhase::Stopped;
                voice.active = false;
                voice.envelope_value = 0;
            } else {
                voice.envelope_phase =
                    static_cast<EnvelopePhase>(next);
            }
        }
    }

    set_raw16(
        base + kVoiceEnvx,
        static_cast<u16>(voice.envelope_value));
}

bool Spu2::voice_gate_enabled(
    u32 core,
    u32 voice,
    bool right) const {
    const u32 base = core * kCoreStride;
    const u32 dry_reg = right ? kVmixR : kVmixL;
    const u32 wet_reg = right ? kVmixEr : kVmixEl;

    const auto mask24 = [&](u32 reg) {
        return static_cast<u32>(raw16(base + reg)) |
               ((static_cast<u32>(
                    raw16(base + reg + 2u)) & 0xFFu) << 16);
    };

    const u32 bit = 1u << voice;
    const bool dry = (mask24(dry_reg) & bit) != 0u;
    const bool wet = (mask24(wet_reg) & bit) != 0u;

    // Until the reverb engine lands, route wet-only voices through the dry
    // output so BIOS sounds remain audible rather than disappearing entirely.
    return dry || wet;
}

s32 Spu2::apply_master_volume(
    u32 core,
    s32 sample,
    bool right) const {
    const u32 reg =
        core == 0u
            ? (right ? kMasterVolR0 : kMasterVolL0)
            : (right ? kMasterVolR1 : kMasterVolL1);
    return apply_volume(sample, raw16(reg));
}

void Spu2::write_endx(u32 core) {
    if (core >= cores_.size()) return;
    const u32 base = core * kCoreStride + kEndx;
    set_raw16(
        base,
        static_cast<u16>(cores_[core].endx));
    set_raw16(
        base + 2u,
        static_cast<u16>((cores_[core].endx >> 16) & 0xFFu));
}

void Spu2::mix_one_sample() {
    s32 output_left = 0;
    s32 output_right = 0;

    for (u32 core = 0; core < 2u; ++core) {
        s32 core_left = 0;
        s32 core_right = 0;

        for (u32 voice_index = 0; voice_index < 24u; ++voice_index) {
            Voice& voice = cores_[core].voices[voice_index];
            if (!voice.active) continue;

            const u32 base =
                core * kCoreStride +
                voice_index * kVoiceParamStride;
            const u16 pitch =
                raw16(base + kVoicePitch) & 0x3FFFu;
            const s16 sample =
                interpolated_voice_sample(core, voice_index);

            update_envelope(core, voice_index);
            if (!voice.active &&
                voice.envelope_value == 0) {
                continue;
            }

            const s32 enveloped =
                (static_cast<s32>(sample) *
                 voice.envelope_value) >> 15;

            const s32 current_left =
                volume_register_value(
                    raw16(base + kVoiceVolL));
            const s32 current_right =
                volume_register_value(
                    raw16(base + kVoiceVolR));
            set_raw16(
                base + kVoiceVolxL,
                static_cast<u16>(current_left));
            set_raw16(
                base + kVoiceVolxR,
                static_cast<u16>(current_right));

            if (voice_gate_enabled(
                    core, voice_index, false)) {
                core_left +=
                    (enveloped * current_left) >> 15;
            }
            if (voice_gate_enabled(
                    core, voice_index, true)) {
                core_right +=
                    (enveloped * current_right) >> 15;
            }

            voice.phase += pitch;
            while (voice.active &&
                   voice.phase >= 0x1000u) {
                voice.phase -= 0x1000u;
                advance_voice(core, voice_index);
            }
        }

        output_left +=
            apply_master_volume(core, core_left, false);
        output_right +=
            apply_master_volume(core, core_right, true);
    }

    push_sample(
        clamp16(output_left),
        clamp16(output_right));
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
