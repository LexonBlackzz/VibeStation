#pragma once

#include "common/types.h"

#include <array>

namespace ps2 {

struct GsStats {
    u64 gif_tags = 0;
    u64 gif_qwords = 0;
    u64 eop_packets = 0;
    u64 register_writes = 0;
    u64 packed_writes = 0;
    u64 reglist_writes = 0;
    u64 image_qwords = 0;
    u64 unsupported_packed = 0;
    u64 vertices = 0;
    u64 primitives = 0;
};

class GsCore {
public:
    static constexpr u32 kGifFifoBase = 0x10006000u;

    void reset();

    [[nodiscard]] bool write_gif_fifo32(u32 physical, u32 value);
    [[nodiscard]] bool write_gif_fifo64(u32 physical, u64 value);
    void write_gif_qword(u64 lo, u64 hi);

    [[nodiscard]] u64 register_value(u32 address) const {
        return registers_[address & 0x7Fu];
    }
    [[nodiscard]] const GsStats& stats() const { return stats_; }
    [[nodiscard]] bool packet_active() const { return gif_.active; }
    [[nodiscard]] u32 current_prim() const {
        return static_cast<u32>(registers_[0] & 0x7u);
    }

private:
    struct GifState {
        bool active = false;
        bool eop = false;
        u32 mode = 0;
        u32 nreg = 0;
        u32 reg_cursor = 0;
        u32 values_remaining = 0;
        u64 regs = 0;
    };

    void begin_tag(u64 lo, u64 hi);
    void finish_packet();
    void process_packed(u32 descriptor, u64 lo, u64 hi);
    void process_reglist_value(u32 descriptor, u64 value);
    void write_register(u32 address, u64 value);
    void note_vertex_kick();

    std::array<u64, 0x80> registers_{};
    std::array<u32, 4> fifo_words_{};
    u8 fifo_word_mask_ = 0;
    GifState gif_{};
    GsStats stats_{};
    u32 primitive_vertex_count_ = 0;
};

} // namespace ps2
