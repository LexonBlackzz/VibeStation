#pragma once

#include "common/types.h"
#include "core/gs/gs_vram.h"
#include "core/gs/gs_rasterizer.h"

#include <array>

namespace ps2 {

class GsPrivileged;

struct GsStats {
    u64 gif_tags = 0;
    u64 gif_qwords = 0;
    u64 eop_packets = 0;
    u64 register_writes = 0;
    u64 packed_writes = 0;
    u64 reglist_writes = 0;
    u64 image_qwords = 0;
    u64 image_bytes = 0;
    u64 host_to_local_transfers = 0;
    u64 host_to_local_pixels = 0;
    u64 local_to_host_transfers = 0;
    u64 local_to_host_pixels = 0;
    u64 local_to_host_qwords = 0;
    u64 local_to_host_bytes = 0;
    u64 local_to_local_transfers = 0;
    u64 local_to_local_pixels = 0;
    u64 unsupported_transfers = 0;
    u64 unsupported_packed = 0;
    u64 vertices = 0;
    u64 primitives = 0;
    u64 raster_draws = 0;
    u64 raster_pixels = 0;
    u64 textured_raster_draws = 0;
    u64 texture_samples = 0;
    u64 skipped_raster_draws = 0;
    u64 signal_events = 0;
    u64 finish_events = 0;
    u64 label_events = 0;
};

class GsCore {
public:
    static constexpr u32 kGifFifoBase = 0x10006000u;

    void reset();
    void attach_privileged(GsPrivileged& privileged) { privileged_ = &privileged; }

    [[nodiscard]] bool write_gif_fifo32(u32 physical, u32 value);
    [[nodiscard]] bool write_gif_fifo64(u32 physical, u64 value);
    void write_gif_qword(u64 lo, u64 hi);
    [[nodiscard]] bool read_local_to_host_qword(u64& lo, u64& hi);

    [[nodiscard]] u64 register_value(u32 address) const {
        return registers_[address & 0x7Fu];
    }
    [[nodiscard]] const GsStats& stats() const { return stats_; }
    [[nodiscard]] const GsVram& vram() const { return vram_; }
    [[nodiscard]] GsVram& vram() { return vram_; }
    [[nodiscard]] bool packet_active() const { return gif_.active; }
    [[nodiscard]] bool transfer_active() const { return transfer_.active; }
    [[nodiscard]] u32 transfer_pixels_remaining() const {
        return transfer_.total_pixels > transfer_.pixel_index
            ? transfer_.total_pixels - transfer_.pixel_index
            : 0;
    }
    [[nodiscard]] u32 transfer_psm() const { return transfer_.psm; }
    [[nodiscard]] u32 current_prim() const {
        return static_cast<u32>(registers_[0] & 0x7u);
    }

private:
    struct TransferState {
        bool active = false;
        bool local_to_host = false;
        u32 bp = 0;
        u32 bw = 0;
        u32 psm = 0;
        u32 dsax = 0;
        u32 dsay = 0;
        u32 width = 0;
        u32 height = 0;
        bool dirx = false;
        bool diry = false;
        u32 pixel_index = 0;
        u32 total_pixels = 0;
        std::array<u8, 32> pending{};
        u32 pending_size = 0;
    };

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
    void begin_host_to_local();
    void begin_local_to_host();
    void execute_local_to_local();
    void consume_image_qword(u64 lo, u64 hi);
    void consume_pending_pixels();
    void submit_vertex(u64 xyz, bool xyzf);
    void emit_primitive(
        const GsRasterVertex& a,
        const GsRasterVertex& b,
        const GsRasterVertex& c,
        u32 vertex_count);
    [[nodiscard]] u64 effective_prim() const;
    [[nodiscard]] GsRasterContext raster_context() const;
    [[nodiscard]] bool raster_state_supported() const;

    std::array<u64, 0x80> registers_{};
    std::array<u32, 4> fifo_words_{};
    u8 fifo_word_mask_ = 0;
    GifState gif_{};
    TransferState transfer_{};
    GsStats stats_{};
    GsVram vram_{};
    std::array<GsRasterVertex, 3> draw_vertices_{};
    u32 draw_vertex_count_ = 0;
    GsPrivileged* privileged_ = nullptr;
};

} // namespace ps2
