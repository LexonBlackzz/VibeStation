#pragma once
// VibeStation - Centralized persisted configuration

#include "types.h"
#include <array>
#include <string>

struct Config {
    // Paths
    std::string bios_path;
    std::string rom_directory;
    std::string log_file_path = "vibestation_runtime.log";

    // Display
    bool vsync = true;
    DeinterlaceMode deinterlace_mode = DeinterlaceMode::Weave;
    OutputResolutionMode output_resolution_mode = OutputResolutionMode::R320x240;
    bool bilinear_filtering = false;
    bool gpu_hardware_rasterizer = true;
    bool gpu_fast_mode = false;
    bool gpu_extreme_fast_mode = false;
    bool low_spec_mode = false;

    // CPU
    CpuExecutionMode cpu_execution_mode = CpuExecutionMode::Interpreter;

    // SPU
    struct SpuConfig {
        u32 target_latency_ms = 80;
        u32 soft_latency_ms = 100;
        u32 max_latency_ms = 300;
        float output_buffer_seconds = 0.06f;
        float xa_buffer_seconds = 0.12f;
        bool enable_audio_queue = true;
        bool enable_smooth_trim = true;
        bool enable_lag_stutter = true;
        bool enable_slowdown_stutter = false;
        bool show_audio_stats = true;
        bool audio_stats_log = false;
        bool advanced_sound_status = false;
    } spu;

    // Memory cards
    static constexpr int kMemoryCardSlotCount = 2;
    std::array<int, kMemoryCardSlotCount> memory_card_slot_mode = {0, 0};

    // Rewind
    bool rewind_enabled = false;
    int rewind_buffer_seconds = 5;

    // Performance
    int turbo_speed_percent = 200;
    int slowdown_speed_percent = 50;
    bool direct_disc_boot = false;
    bool spu_diagnostic_mode = false;
    bool discord_rich_presence = false;

    // Logging
    LogLevel log_level = LogLevel::Info;
    bool log_timestamps = true;
    bool log_collapse_repeats = true;
    bool log_fmv_diagnostics = false;
    u32 log_repeat_flush = 1000;
    u32 log_category_mask = 0xFFFFFFFFu;

    // Tracing
    struct TraceConfig {
        bool dma = false;
        bool cdrom = false;
        bool cpu = false;
        bool bus = false;
        bool ram = false;
        bool gpu = false;
        bool spu = false;
        bool irq = false;
        bool timer = false;
        bool sio = false;
        u32 burst_cpu = 128;
        u32 stride_cpu = 32768;
        u32 burst_bus = 256;
        u32 stride_bus = 16384;
        u32 burst_ram = 32;
        u32 stride_ram = 131072;
        u32 burst_dma = 64;
        u32 stride_dma = 2048;
        u32 burst_cdrom = 128;
        u32 stride_cdrom = 256;
        u32 burst_gpu = 512;
        u32 stride_gpu = 2048;
        u32 burst_spu = 128;
        u32 stride_spu = 4096;
        u32 burst_irq = 128;
        u32 stride_irq = 2048;
        u32 burst_timer = 64;
        u32 stride_timer = 2048;
        u32 burst_sio = 64;
        u32 stride_sio = 2048;
    } trace;

    // Diagnostics
    bool cpu_deep_diagnostics = false;
    bool detailed_profiling = false;

    // Experimental
    bool experimental_bios_size_mode = false;
    bool unsafe_ps2_bios_mode = false;
    bool experimental_unhandled_special_returns_zero = false;
    bool experimental_dma_command_sanitizer = false;

    // MDEC debug
    struct MdecDebugConfig {
        bool disable_dma1_reorder = false;
        bool disable_chroma = false;
        bool disable_luma = false;
        bool force_solid_output = false;
        bool swap_input_halfwords = false;
        bool compare_macroblocks = false;
        bool upload_probe = false;
        u8 color_block_mask = 0x0F;
    } mdec_debug;

    static Config load(const std::string& path);
    void save(const std::string& path) const;
    void apply_to_globals() const;
};
