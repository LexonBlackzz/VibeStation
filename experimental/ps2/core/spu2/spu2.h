#pragma once

#include "common/types.h"

#include <array>
#include <cstddef>
#include <deque>
#include <vector>

namespace ps2 {

class IopRam;

class Spu2 {
public:
    struct DebugStats {
        u64 mixed_frames = 0;
        u64 nonzero_output_frames = 0;
        u64 keyed_on_voices = 0;
        u64 keyed_off_voices = 0;
        u64 decoded_blocks = 0;
        u64 decoded_nonzero_samples = 0;
        u64 dma_write_halfwords = 0;
        u64 dma_read_halfwords = 0;
        u32 max_active_voices = 0;
    };

    static constexpr u32 kSampleRate = 48000u;
    static constexpr u32 kIopCyclesPerSample = 768u;
    static constexpr u32 kRamHalfwords = 0x100000u;

    Spu2();

    void reset();
    void tick(u64 iop_cycles);

    [[nodiscard]] bool read8(u32 offset, u8& value) const;
    [[nodiscard]] bool read16(u32 offset, u16& value) const;
    [[nodiscard]] bool read32(u32 offset, u32& value) const;
    [[nodiscard]] bool write8(u32 offset, u8 value);
    [[nodiscard]] bool write16(u32 offset, u16 value);
    [[nodiscard]] bool write32(u32 offset, u32 value);

    bool dma_write(
        u32 core,
        const IopRam& ram,
        u32 madr,
        u32 halfwords);
    bool dma_read(
        u32 core,
        IopRam& ram,
        u32 madr,
        u32 halfwords);

    [[nodiscard]] std::vector<s16> take_samples(
        std::size_t max_frames = 4096u);
    [[nodiscard]] std::size_t queued_frames() const {
        return pcm_queue_.size() / 2u;
    }
    [[nodiscard]] const DebugStats& debug_stats() const {
        return debug_stats_;
    }
    [[nodiscard]] u32 active_voice_count() const;

private:
    enum class EnvelopePhase : u8 {
        Stopped = 0,
        Attack = 1,
        Decay = 2,
        Sustain = 3,
        Release = 4,
    };

    struct Voice {
        bool active = false;
        s32 prev1 = 0;
        s32 prev2 = 0;
        u32 current_addr = 0;
        u32 loop_addr = 0;
        u32 phase = 0;
        u32 decoded_pos = 28;
        u8 block_flags = 0;

        EnvelopePhase envelope_phase = EnvelopePhase::Stopped;
        u32 envelope_counter = 0;
        s32 envelope_value = 0;

        std::array<s16, 28> decoded{};
    };

    struct Core {
        std::array<Voice, 24> voices{};
        u32 transfer_addr = 0;
        u32 endx = 0x00FFFFFFu;
    };

    [[nodiscard]] u16 raw16(u32 offset) const;
    void set_raw16(u32 offset, u16 value);
    [[nodiscard]] u32 address_register(
        u32 core,
        u32 base,
        u32 voice = 0) const;
    void refresh_transfer_address(u32 core);
    void handle_register_write(u32 offset, u16 value);
    void key_on(u32 core, u32 mask, u32 first_voice);
    void key_off(u32 core, u32 mask, u32 first_voice);

    [[nodiscard]] bool decode_block(u32 core, u32 voice);
    [[nodiscard]] s16 voice_sample(u32 core, u32 voice);
    [[nodiscard]] s16 interpolated_voice_sample(u32 core, u32 voice);
    void advance_voice(u32 core, u32 voice);
    void update_envelope(u32 core, u32 voice);
    [[nodiscard]] bool voice_gate_enabled(
        u32 core,
        u32 voice,
        bool right) const;
    [[nodiscard]] s32 apply_master_volume(
        u32 core,
        s32 sample,
        bool right) const;
    void write_endx(u32 core);
    void mix_one_sample();
    void push_sample(s16 left, s16 right);

    std::array<u8, 0x10000> regs_{};
    std::vector<u16> ram_;
    std::array<Core, 2> cores_{};
    u64 cycle_phase_ = 0;
    std::deque<s16> pcm_queue_;
    DebugStats debug_stats_{};
};

} // namespace ps2
