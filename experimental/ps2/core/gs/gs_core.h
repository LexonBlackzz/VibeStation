#pragma once

#include "common/types.h"
#include "core/gs/gs_vram.h"
#include "core/gs/gs_rasterizer.h"
#include "core/gs/gs_gpu_backend.h"

#include <array>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

namespace ps2 {

class GsPrivileged;

struct GsUnsupportedTransfer {
    u32 reason = 0;
    u64 bitbltbuf = 0;
    u64 trxpos = 0;
    u64 trxreg = 0;
    u64 trxdir = 0;
};

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
    std::array<GsUnsupportedTransfer, 8> first_unsupported_transfers{};
    u32 first_unsupported_transfer_count = 0;
    u64 unsupported_packed = 0;
    u64 vertices = 0;
    u64 primitives = 0;
    u64 raster_draws = 0;
    u64 raster_pixels = 0;
    u64 parallel_sprite_draws = 0;
    u64 parallel_sprite_pixels = 0;
    u64 parallel_sprite_helper_jobs = 0;
    u64 gpu_sprite_draws = 0;
    u64 gpu_sprite_pixels = 0;
    u64 gpu_syncs_to_cpu = 0;
    // Large dependency-safe sprite profile used to grow the GPU fast path.
    u64 gpu_candidate_sprite_draws = 0;
    u64 gpu_candidate_sprite_pixels = 0;
    std::array<u64, 64> gpu_candidate_texture_psm_draws{};
    std::array<u64, 64> gpu_candidate_texture_psm_pixels{};
    std::array<u64, 64> gpu_candidate_frame_psm_draws{};
    // Key: A/B/C/D [7:0], ZTST [9:8], COLCLAMP bit10, PABE bit11,
    // ABE bit12, ZTE bit13, ZMASK bit14, ATE bit15, DATE bit16.
    std::array<u64, 131072> gpu_candidate_state_draws{};
    std::array<u64, 4> gpu_candidate_tfx_draws{};
    std::array<u64, 2> gpu_candidate_tcc_draws{};
    std::array<u64, 256> gpu_candidate_fix_draws{};
    std::array<u64, 8> raster_draws_by_primitive{};
    std::array<u64, 8> raster_pixels_by_primitive{};
    std::array<u64, 8> raster_ns_by_primitive{};
    // Draw-size histogram bins: <64, <256, <1K, <4K, <16K, <64K, >=64K pixels.
    std::array<u64, 7> raster_draws_by_size{};
    std::array<u64, 7> raster_pixels_by_size{};
    std::array<u64, 7> raster_ns_by_size{};
    std::array<u64, 64> texture_draws_by_psm{};
    std::array<u64, 64> texture_ns_by_psm{};
    // PSMCT16-only draw-state profile. Bits:
    // 0 ABE, 1 ATE, 2 DATE, 3 ZTE, 4 depth write,
    // 5 FBMSK, 6 dither, 7 active scan mask.
    std::array<u64, 256> psm16_state_draws{};
    std::array<u64, 256> psm16_state_pixels{};
    std::array<u64, 256> psm16_state_ns{};
    // Exact hot PSMCT16 blend/depth state. Key:
    // [7:0] ALPHA A/B/C/D selectors, [9:8] ZTST,
    // bit10 COLCLAMP, bit11 PABE.
    std::array<u64, 4096> psm16_alpha_state_draws{};
    std::array<u64, 4096> psm16_alpha_state_pixels{};
    std::array<u64, 4096> psm16_alpha_state_ns{};
    std::array<u64, 256> psm16_fix_draws{};
    std::array<u64, 256> psm16_fix_ns{};
    u64 textured_sprite_pixels = 0;
    u64 textured_sprite_fst_pixels = 0;
    u64 textured_sprite_constant_q_pixels = 0;
    u64 textured_sprite_variable_q_pixels = 0;
    u64 textured_triangle_pixels = 0;
    u64 textured_triangle_fst_pixels = 0;
    u64 textured_triangle_constant_q_pixels = 0;
    u64 textured_triangle_variable_q_pixels = 0;
    u64 textured_raster_draws = 0;
    u64 texture_samples = 0;
    u64 nonzero_texture_samples = 0;
    u64 texture_alpha_samples = 0;
    u32 first_texture_sample_x = 0xFFFFFFFFu;
    u32 first_texture_sample_y = 0;
    u32 first_texture_sample_rgba = 0;
    u64 nonzero_shaded_samples = 0;
    u64 nonzero_raster_inputs = 0;
    u64 nonzero_inputs_with_alpha = 0;
    u64 nonzero_raster_colors = 0;
    u64 nonzero_inputs_with_blend = 0;
    u64 nonzero_inputs_without_blend = 0;
    bool first_nonzero_input_valid = false;
    u64 first_nonzero_input_alpha = 0;
    u64 first_nonzero_input_test = 0;
    u64 first_nonzero_input_frame = 0;
    u64 first_nonzero_input_prim = 0;
    u64 first_nonzero_input_rgbaq = 0;
    u32 first_nonzero_input_rgba = 0;
    u64 first_nonzero_input_tex0 = 0;
    u64 first_nonzero_input_texa = 0;
    u64 first_nonzero_input_st = 0;
    u64 first_nonzero_input_uv = 0;
    bool first_alpha_input_valid = false;
    u32 first_alpha_input_rgba = 0;
    u64 first_alpha_input_alpha = 0;
    u64 first_alpha_input_prim = 0;
    u64 first_alpha_input_tex0 = 0;
    u64 first_alpha_input_rgbaq = 0;
    u64 skipped_raster_draws = 0;
    u64 unsupported_target_draws = 0;
    u64 unsupported_texture_draws = 0;
    u64 last_unsupported_prim = 0;
    u64 last_unsupported_frame = 0;
    u64 last_unsupported_zbuf = 0;
    u64 last_unsupported_test = 0;
    u64 last_unsupported_tex0 = 0;
    u64 signal_events = 0;
    u64 finish_events = 0;
    u64 label_events = 0;
};

class GsCore {
public:
    static constexpr u32 kGifFifoBase = 0x10006000u;

    ~GsCore();

    void reset();
    void set_async_rasterization(bool enabled);
    void set_gpu_backend(GsGpuBackend* backend);
    [[nodiscard]] bool gpu_backend_active() const {
        return gpu_backend_ != nullptr && gpu_backend_->available();
    }
    [[nodiscard]] const char* gpu_backend_name() const {
        return gpu_backend_active() ? gpu_backend_->name() : "software";
    }
    void set_rasterization_enabled(bool enabled) {
        rasterization_enabled_ = enabled;
    }
    void set_detailed_raster_stats(bool enabled) {
        detailed_raster_stats_ = enabled;
    }
    void set_raster_timing_enabled(bool enabled) {
        raster_timing_enabled_ = enabled;
    }
    void flush_pending_draws() const;
    void attach_privileged(GsPrivileged& privileged) { privileged_ = &privileged; }

    [[nodiscard]] bool write_gif_fifo32(u32 physical, u32 value);
    [[nodiscard]] bool write_gif_fifo64(u32 physical, u64 value);
    void write_gif_qword(u64 lo, u64 hi);
    [[nodiscard]] bool read_local_to_host_qword(u64& lo, u64& hi);

    [[nodiscard]] u64 register_value(u32 address) const {
        return registers_[address & 0x7Fu];
    }
    [[nodiscard]] const GsStats& stats() const {
        flush_pending_draws();
        return stats_;
    }
    // These counters are updated by GIF submission on the emulation thread,
    // not by the async raster worker. UI telemetry can read them without
    // draining queued raster work.
    [[nodiscard]] u64 submitted_gif_qwords() const {
        return stats_.gif_qwords;
    }
    [[nodiscard]] u64 submitted_primitives() const {
        return stats_.primitives;
    }
    [[nodiscard]] const GsVram& vram() const {
        flush_pending_draws();
        return vram_;
    }
    [[nodiscard]] GsVram& vram() {
        flush_pending_draws();
        return vram_;
    }
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
    struct RasterCommand {
        GsRasterContext context{};
        GsRasterVertex a{};
        GsRasterVertex b{};
        GsRasterVertex c{};
        u32 primitive = 0;
        u32 vertex_count = 0;
        u64 effective_primitive = 0;
        u64 alpha = 0;
        u64 test = 0;
        u64 frame = 0;
        u64 rgbaq = 0;
        u64 tex0 = 0;
        u64 texa = 0;
        u64 st = 0;
        u64 uv = 0;
    };

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
    void record_unsupported_transfer(u32 reason);
    void submit_vertex(u64 xyz, bool xyzf);
    void emit_primitive(
        const GsRasterVertex& a,
        const GsRasterVertex& b,
        const GsRasterVertex& c,
        u32 vertex_count);
    void execute_raster_command(const RasterCommand& command);
    void raster_worker_main();
    void start_raster_helpers();
    void stop_raster_helpers();
    void raster_helper_main(u32 helper_index);
    bool try_execute_parallel_sprite(
        const RasterCommand& command,
        u64& pixels);
    bool try_execute_gpu_sprite(
        const RasterCommand& command,
        u64& pixels);
    void synchronize_gpu_to_cpu() const;
    [[nodiscard]] u64 effective_prim() const;
    [[nodiscard]] GsRasterContext raster_context() const;

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
    bool async_rasterization_ = false;
    bool rasterization_enabled_ = true;
    bool detailed_raster_stats_ = false;
    bool raster_timing_enabled_ = false;
    mutable GsGpuBackend* gpu_backend_ = nullptr;
    mutable std::mutex raster_mutex_{};
    mutable std::condition_variable raster_condition_{};
    mutable std::condition_variable raster_completed_condition_{};
    std::deque<RasterCommand> raster_queue_{};
    std::thread raster_worker_{};
    bool raster_worker_stop_ = false;
    u64 raster_enqueued_ = 0;
    u64 raster_completed_ = 0;

    // Keep one emulation thread and spare host capacity outside the raster
    // pool. Small CI hosts still select only hardware_concurrency()-2 helpers,
    // while desktop CPUs can use up to six raster lanes total.
    static constexpr u32 kMaxRasterHelpers = 5u;
    std::array<std::thread, kMaxRasterHelpers> raster_helpers_{};
    u32 raster_helper_count_ = 0u;
    std::mutex raster_parallel_mutex_{};
    std::condition_variable raster_parallel_condition_{};
    std::condition_variable raster_parallel_done_condition_{};
    bool raster_parallel_stop_ = false;
    u64 raster_parallel_generation_ = 0u;
    u32 raster_parallel_pending_ = 0u;
    const GsRasterContext* raster_parallel_context_ = nullptr;
    const GsRasterVertex* raster_parallel_a_ = nullptr;
    const GsRasterVertex* raster_parallel_b_ = nullptr;
    std::array<s32, kMaxRasterHelpers + 2u>
        raster_parallel_boundaries_{};
    std::array<u64, kMaxRasterHelpers + 1u>
        raster_parallel_counts_{};
};

} // namespace ps2
