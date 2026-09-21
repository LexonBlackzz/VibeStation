#include "config.h"

#include <algorithm>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace {

int log_level_to_config_value(LogLevel level) {
    switch (level) {
    case LogLevel::Debug: return 0;
    case LogLevel::Info:  return 1;
    case LogLevel::Warn:  return 2;
    case LogLevel::Error: return 3;
    }
    return 1;
}

LogLevel parse_log_level_config(const std::string& value, LogLevel fallback) {
    if (value == "0" || value == "debug" || value == "DEBUG") {
        return LogLevel::Debug;
    }
    if (value == "1" || value == "info" || value == "INFO") {
        return LogLevel::Info;
    }
    if (value == "2" || value == "warn" || value == "warning" ||
        value == "WARN" || value == "WARNING") {
        return LogLevel::Warn;
    }
    if (value == "3" || value == "error" || value == "ERROR") {
        return LogLevel::Error;
    }
    return fallback;
}

int normalize_turbo_speed_percent(int v) {
    if (v <= 0) return 0;
    return (v >= 400) ? 400 : std::max(101, v);
}

int normalize_slowdown_speed_percent(int v) {
    return std::max(1, std::min(99, v));
}

CpuExecutionMode parse_cpu_mode(const std::string& v, CpuExecutionMode fallback) {
    std::string lower = v;
    std::transform(lower.begin(), lower.end(), lower.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    lower.erase(std::remove(lower.begin(), lower.end(), '-'), lower.end());
    lower.erase(std::remove(lower.begin(), lower.end(), '_'), lower.end());
    if (lower == "1" || lower == "decoded" || lower == "decodedblock" ||
        lower == "blockinterpreter" || lower == "blockinterp" || lower == "block") {
        return CpuExecutionMode::DecodedBlockInterpreter;
    }
    if (lower == "2" || lower == "x64jit" || lower == "recompiler" ||
        lower == "jit" || lower == "dynarec") {
        return CpuExecutionMode::X64Jit;
    }
    if (lower == "3" || lower == "x64jitv2" || lower == "jitv2" ||
        lower == "dynarecv2" || lower == "recompilerv2") {
        return CpuExecutionMode::X64JitV2;
    }
    if (lower == "0" || lower == "interpreter" || lower == "interp") {
        return CpuExecutionMode::Interpreter;
    }
    return fallback;
}

int cpu_mode_to_int(CpuExecutionMode mode) {
    switch (mode) {
    case CpuExecutionMode::DecodedBlockInterpreter: return 1;
    case CpuExecutionMode::X64Jit: return 2;
    case CpuExecutionMode::X64JitV2: return 3;
    case CpuExecutionMode::Interpreter:
    default: return 0;
    }
}

} // anonymous namespace

Config Config::load(const std::string& path) {
    Config cfg;

    std::ifstream in(path);
    if (!in.is_open()) {
        return cfg;
    }

    json j;
    try {
        in >> j;
    } catch (const json::parse_error&) {
        return cfg;
    }

    auto get_str = [&](const char* key, std::string& dest) {
        if (j.contains(key) && j[key].is_string()) {
            dest = j[key].get<std::string>();
        }
    };
    auto get_bool = [&](const char* key, bool& dest) {
        if (j.contains(key) && j[key].is_boolean()) {
            dest = j[key].get<bool>();
        }
    };
    auto get_int = [&](const char* key, int& dest) {
        if (j.contains(key) && j[key].is_number_integer()) {
            dest = j[key].get<int>();
        }
    };
    auto get_uint = [&](const char* key, u32& dest) {
        if (j.contains(key) && j[key].is_number_unsigned()) {
            dest = j[key].get<u32>();
        }
    };
    auto get_float = [&](const char* key, float& dest) {
        if (j.contains(key) && j[key].is_number()) {
            dest = j[key].get<float>();
        }
    };

    // Paths
    get_str("bios_path", cfg.bios_path);
    get_str("rom_directory", cfg.rom_directory);
    get_str("log_file_path", cfg.log_file_path);

    // Display
    get_bool("vsync", cfg.vsync);
    get_bool("bilinear_filtering", cfg.bilinear_filtering);
    get_bool("gpu_fast_mode", cfg.gpu_fast_mode);
    get_bool("gpu_extreme_fast_mode", cfg.gpu_extreme_fast_mode);
    get_bool("low_spec_mode", cfg.low_spec_mode);
    if (j.contains("deinterlace_mode") && j["deinterlace_mode"].is_number_unsigned()) {
        u32 v = j["deinterlace_mode"].get<u32>();
        cfg.deinterlace_mode = static_cast<DeinterlaceMode>(std::min(2u, v));
    }
    if (j.contains("output_resolution_mode") && j["output_resolution_mode"].is_number_unsigned()) {
        u32 v = j["output_resolution_mode"].get<u32>();
        cfg.output_resolution_mode = static_cast<OutputResolutionMode>(std::min(2u, v));
    }

    // CPU
    if (j.contains("cpu_execution_mode")) {
        if (j["cpu_execution_mode"].is_number_integer()) {
            int v = j["cpu_execution_mode"].get<int>();
            cfg.cpu_execution_mode = cpu_execution_mode_from_config_value(v);
        } else if (j["cpu_execution_mode"].is_string()) {
            cfg.cpu_execution_mode = parse_cpu_mode(j["cpu_execution_mode"].get<std::string>(),
                cfg.cpu_execution_mode);
        }
    }
    // cpu_x64_jit and all of its experimental tuning keys are obsolete. Keep
    // accepting older JSON files, but compilation policy is automatic and the
    // next save intentionally omits the object.

    // Memory cards
    if (j.contains("memory_card_slot1_mode") && j["memory_card_slot1_mode"].is_number_integer()) {
        cfg.memory_card_slot_mode[0] = std::clamp(j["memory_card_slot1_mode"].get<int>(), 0, 2);
    }
    if (j.contains("memory_card_slot2_mode") && j["memory_card_slot2_mode"].is_number_integer()) {
        cfg.memory_card_slot_mode[1] = std::clamp(j["memory_card_slot2_mode"].get<int>(), 0, 2);
    }

    // Performance
    if (j.contains("turbo_speed_percent") && j["turbo_speed_percent"].is_number_integer()) {
        cfg.turbo_speed_percent = normalize_turbo_speed_percent(j["turbo_speed_percent"].get<int>());
    }
    if (j.contains("slowdown_speed_percent") && j["slowdown_speed_percent"].is_number_integer()) {
        cfg.slowdown_speed_percent = normalize_slowdown_speed_percent(j["slowdown_speed_percent"].get<int>());
    }
    // Rewind
    get_bool("rewind_enabled", cfg.rewind_enabled);
    if (j.contains("rewind_buffer_seconds") && j["rewind_buffer_seconds"].is_number_integer()) {
        cfg.rewind_buffer_seconds = std::clamp(j["rewind_buffer_seconds"].get<int>(), 1, 10);
    }

    get_bool("direct_disc_boot", cfg.direct_disc_boot);
    get_bool("spu_diagnostic_mode", cfg.spu_diagnostic_mode);
    get_bool("discord_rich_presence", cfg.discord_rich_presence);

    // SPU
    auto spu = j.value("spu", json::object());
    auto spu_uint = [&](const char* key, u32& dest) {
        if (spu.contains(key) && spu[key].is_number_unsigned()) {
            dest = spu[key].get<u32>();
        }
    };
    auto spu_bool = [&](const char* key, bool& dest) {
        if (spu.contains(key) && spu[key].is_boolean()) {
            dest = spu[key].get<bool>();
        }
    };
    auto spu_float = [&](const char* key, float& dest) {
        if (spu.contains(key) && spu[key].is_number()) {
            dest = spu[key].get<float>();
        }
    };
    spu_uint("target_latency_ms", cfg.spu.target_latency_ms);
    spu_uint("soft_latency_ms", cfg.spu.soft_latency_ms);
    spu_uint("max_latency_ms", cfg.spu.max_latency_ms);
    spu_float("output_buffer_seconds", cfg.spu.output_buffer_seconds);
    spu_float("xa_buffer_seconds", cfg.spu.xa_buffer_seconds);
    spu_bool("enable_audio_queue", cfg.spu.enable_audio_queue);
    spu_bool("enable_smooth_trim", cfg.spu.enable_smooth_trim);
    spu_bool("enable_lag_stutter", cfg.spu.enable_lag_stutter);
    spu_bool("enable_slowdown_stutter", cfg.spu.enable_slowdown_stutter);
    spu_bool("show_audio_stats", cfg.spu.show_audio_stats);
    spu_bool("audio_stats_log", cfg.spu.audio_stats_log);
    spu_bool("advanced_sound_status", cfg.spu.advanced_sound_status);

    // Logging
    if (j.contains("log_level")) {
        if (j["log_level"].is_string()) {
            cfg.log_level = parse_log_level_config(j["log_level"].get<std::string>(), cfg.log_level);
        } else if (j["log_level"].is_number_integer()) {
            int v = j["log_level"].get<int>();
            cfg.log_level = static_cast<LogLevel>(std::clamp(v, 0, 3));
        }
    }
    get_bool("log_timestamps", cfg.log_timestamps);
    get_bool("log_collapse_repeats", cfg.log_collapse_repeats);
    get_bool("log_fmv_diagnostics", cfg.log_fmv_diagnostics);
    get_uint("log_repeat_flush", cfg.log_repeat_flush);
    get_uint("log_category_mask", cfg.log_category_mask);

    // Tracing
    auto tr = j.value("trace", json::object());
    auto tr_bool = [&](const char* key, bool& dest) {
        if (tr.contains(key) && tr[key].is_boolean()) {
            dest = tr[key].get<bool>();
        }
    };
    auto tr_uint = [&](const char* key, u32& dest) {
        if (tr.contains(key) && tr[key].is_number_unsigned()) {
            dest = tr[key].get<u32>();
        }
    };
    tr_bool("dma", cfg.trace.dma);
    tr_bool("cdrom", cfg.trace.cdrom);
    tr_bool("cpu", cfg.trace.cpu);
    tr_bool("bus", cfg.trace.bus);
    tr_bool("ram", cfg.trace.ram);
    tr_bool("gpu", cfg.trace.gpu);
    tr_bool("spu", cfg.trace.spu);
    tr_bool("irq", cfg.trace.irq);
    tr_bool("timer", cfg.trace.timer);
    tr_bool("sio", cfg.trace.sio);
    tr_uint("burst_cpu", cfg.trace.burst_cpu);
    tr_uint("stride_cpu", cfg.trace.stride_cpu);
    tr_uint("burst_bus", cfg.trace.burst_bus);
    tr_uint("stride_bus", cfg.trace.stride_bus);
    tr_uint("burst_ram", cfg.trace.burst_ram);
    tr_uint("stride_ram", cfg.trace.stride_ram);
    tr_uint("burst_dma", cfg.trace.burst_dma);
    tr_uint("stride_dma", cfg.trace.stride_dma);
    tr_uint("burst_cdrom", cfg.trace.burst_cdrom);
    tr_uint("stride_cdrom", cfg.trace.stride_cdrom);
    tr_uint("burst_gpu", cfg.trace.burst_gpu);
    tr_uint("stride_gpu", cfg.trace.stride_gpu);
    tr_uint("burst_spu", cfg.trace.burst_spu);
    tr_uint("stride_spu", cfg.trace.stride_spu);
    tr_uint("burst_irq", cfg.trace.burst_irq);
    tr_uint("stride_irq", cfg.trace.stride_irq);
    tr_uint("burst_timer", cfg.trace.burst_timer);
    tr_uint("stride_timer", cfg.trace.stride_timer);
    tr_uint("burst_sio", cfg.trace.burst_sio);
    tr_uint("stride_sio", cfg.trace.stride_sio);

    // Diagnostics
    get_bool("cpu_deep_diagnostics", cfg.cpu_deep_diagnostics);
    get_bool("detailed_profiling", cfg.detailed_profiling);

    // Experimental
    get_bool("experimental_bios_size_mode", cfg.experimental_bios_size_mode);
    get_bool("unsafe_ps2_bios_mode", cfg.unsafe_ps2_bios_mode);
    get_bool("experimental_unhandled_special_returns_zero",
        cfg.experimental_unhandled_special_returns_zero);
    get_bool("experimental_dma_command_sanitizer", cfg.experimental_dma_command_sanitizer);

    // MDEC debug
    auto mdec = j.value("mdec_debug", json::object());
    auto mdec_bool = [&](const char* key, bool& dest) {
        if (mdec.contains(key) && mdec[key].is_boolean()) {
            dest = mdec[key].get<bool>();
        }
    };
    mdec_bool("disable_dma1_reorder", cfg.mdec_debug.disable_dma1_reorder);
    mdec_bool("disable_chroma", cfg.mdec_debug.disable_chroma);
    mdec_bool("disable_luma", cfg.mdec_debug.disable_luma);
    mdec_bool("force_solid_output", cfg.mdec_debug.force_solid_output);
    mdec_bool("swap_input_halfwords", cfg.mdec_debug.swap_input_halfwords);
    mdec_bool("compare_macroblocks", cfg.mdec_debug.compare_macroblocks);
    mdec_bool("upload_probe", cfg.mdec_debug.upload_probe);
    if (mdec.contains("color_block_mask") && mdec["color_block_mask"].is_number_unsigned()) {
        cfg.mdec_debug.color_block_mask = static_cast<u8>(mdec["color_block_mask"].get<u32>() & 0x0Fu);
    }

    // Backward compatibility: migrate old flat keys
    if (j.contains("spu_output_latency_ms") && j["spu_output_latency_ms"].is_number_unsigned()) {
        cfg.spu.target_latency_ms = std::clamp(j["spu_output_latency_ms"].get<u32>(), 10u, 500u);
    }
    if (j.contains("spu_xa_latency_ms") && j["spu_xa_latency_ms"].is_number_unsigned()) {
        u32 ms = j["spu_xa_latency_ms"].get<u32>();
        cfg.spu.xa_buffer_seconds = static_cast<float>(std::min(2000u, ms)) / 1000.0f;
    }

    // Clamp SPU values
    cfg.spu.output_buffer_seconds = std::max(0.05f, std::min(8.0f, cfg.spu.output_buffer_seconds));
    cfg.spu.target_latency_ms = std::clamp(cfg.spu.target_latency_ms, 10u, 500u);
    cfg.spu.soft_latency_ms = std::clamp(cfg.spu.soft_latency_ms,
        cfg.spu.target_latency_ms, 750u);
    cfg.spu.max_latency_ms = std::clamp(cfg.spu.max_latency_ms,
        cfg.spu.soft_latency_ms, 1000u);
    cfg.spu.xa_buffer_seconds = std::max(0.0f, std::min(5.0f, cfg.spu.xa_buffer_seconds));

    if (!cfg.gpu_fast_mode) {
        cfg.gpu_extreme_fast_mode = false;
    }
    if (cfg.unsafe_ps2_bios_mode) {
        cfg.experimental_bios_size_mode = true;
    }

    return cfg;
}

void Config::save(const std::string& path) const {
    json j;

    // Paths
    j["bios_path"] = bios_path;
    j["rom_directory"] = rom_directory;
    j["log_file_path"] = log_file_path;

    // Display
    j["vsync"] = vsync;
    j["bilinear_filtering"] = bilinear_filtering;
    j["gpu_fast_mode"] = gpu_fast_mode;
    j["gpu_extreme_fast_mode"] = gpu_fast_mode && gpu_extreme_fast_mode;
    j["low_spec_mode"] = low_spec_mode;
    j["deinterlace_mode"] = static_cast<int>(deinterlace_mode);
    j["output_resolution_mode"] = static_cast<int>(output_resolution_mode);

    // CPU
    j["cpu_execution_mode"] = cpu_mode_to_int(cpu_execution_mode);

    // Memory cards
    j["memory_card_slot1_mode"] = std::clamp(memory_card_slot_mode[0], 0, 2);
    j["memory_card_slot2_mode"] = std::clamp(memory_card_slot_mode[1], 0, 2);

    // Rewind
    j["rewind_enabled"] = rewind_enabled;
    j["rewind_buffer_seconds"] = std::clamp(rewind_buffer_seconds, 1, 10);

    // Performance
    j["turbo_speed_percent"] = turbo_speed_percent;
    j["slowdown_speed_percent"] = slowdown_speed_percent;
    j["direct_disc_boot"] = direct_disc_boot;
    j["spu_diagnostic_mode"] = spu_diagnostic_mode;
    j["discord_rich_presence"] = discord_rich_presence;

    // SPU
    j["spu"] = {
        {"target_latency_ms", spu.target_latency_ms},
        {"soft_latency_ms", spu.soft_latency_ms},
        {"max_latency_ms", spu.max_latency_ms},
        {"output_buffer_seconds", spu.output_buffer_seconds},
        {"xa_buffer_seconds", spu.xa_buffer_seconds},
        {"enable_audio_queue", spu.enable_audio_queue},
        {"enable_smooth_trim", spu.enable_smooth_trim},
        {"enable_lag_stutter", spu.enable_lag_stutter},
        {"enable_slowdown_stutter", spu.enable_slowdown_stutter},
        {"show_audio_stats", spu.show_audio_stats},
        {"audio_stats_log", spu.audio_stats_log},
        {"advanced_sound_status", spu.advanced_sound_status},
    };

    // Logging
    j["log_level"] = log_level_to_config_value(log_level);
    j["log_timestamps"] = log_timestamps;
    j["log_collapse_repeats"] = log_collapse_repeats;
    j["log_fmv_diagnostics"] = log_fmv_diagnostics;
    j["log_repeat_flush"] = log_repeat_flush;
    j["log_category_mask"] = log_category_mask;

    // Tracing
    j["trace"] = {
        {"dma", trace.dma},
        {"cdrom", trace.cdrom},
        {"cpu", trace.cpu},
        {"bus", trace.bus},
        {"ram", trace.ram},
        {"gpu", trace.gpu},
        {"spu", trace.spu},
        {"irq", trace.irq},
        {"timer", trace.timer},
        {"sio", trace.sio},
        {"burst_cpu", trace.burst_cpu},
        {"stride_cpu", trace.stride_cpu},
        {"burst_bus", trace.burst_bus},
        {"stride_bus", trace.stride_bus},
        {"burst_ram", trace.burst_ram},
        {"stride_ram", trace.stride_ram},
        {"burst_dma", trace.burst_dma},
        {"stride_dma", trace.stride_dma},
        {"burst_cdrom", trace.burst_cdrom},
        {"stride_cdrom", trace.stride_cdrom},
        {"burst_gpu", trace.burst_gpu},
        {"stride_gpu", trace.stride_gpu},
        {"burst_spu", trace.burst_spu},
        {"stride_spu", trace.stride_spu},
        {"burst_irq", trace.burst_irq},
        {"stride_irq", trace.stride_irq},
        {"burst_timer", trace.burst_timer},
        {"stride_timer", trace.stride_timer},
        {"burst_sio", trace.burst_sio},
        {"stride_sio", trace.stride_sio},
    };

    // Diagnostics
    j["cpu_deep_diagnostics"] = cpu_deep_diagnostics;
    j["detailed_profiling"] = detailed_profiling;

    // Experimental
    j["experimental_bios_size_mode"] = experimental_bios_size_mode;
    j["unsafe_ps2_bios_mode"] = unsafe_ps2_bios_mode;
    j["experimental_unhandled_special_returns_zero"] = experimental_unhandled_special_returns_zero;
    j["experimental_dma_command_sanitizer"] = experimental_dma_command_sanitizer;

    // MDEC debug
    j["mdec_debug"] = {
        {"disable_dma1_reorder", mdec_debug.disable_dma1_reorder},
        {"disable_chroma", mdec_debug.disable_chroma},
        {"disable_luma", mdec_debug.disable_luma},
        {"force_solid_output", mdec_debug.force_solid_output},
        {"swap_input_halfwords", mdec_debug.swap_input_halfwords},
        {"compare_macroblocks", mdec_debug.compare_macroblocks},
        {"upload_probe", mdec_debug.upload_probe},
        {"color_block_mask", static_cast<unsigned>(mdec_debug.color_block_mask & 0x0Fu)},
    };

    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (out.is_open()) {
        out << j.dump(2) << "\n";
    }
}

void Config::apply_to_globals() const {
    // Paths are App members, not globals - caller handles bios_path_ and rom_directory_

    // Display
    g_low_spec_mode = low_spec_mode;
    g_gpu_fast_mode = gpu_fast_mode;
    g_gpu_extreme_fast_mode = gpu_extreme_fast_mode;
    g_bilinear_filtering = bilinear_filtering;
    g_deinterlace_mode = deinterlace_mode;
    g_output_resolution_mode = output_resolution_mode;

    // CPU
    g_cpu_execution_mode = cpu_execution_mode;

    // SPU
    g_spu_audio_target_latency_ms = spu.target_latency_ms;
    g_spu_audio_soft_latency_ms = spu.soft_latency_ms;
    g_spu_audio_max_latency_ms = spu.max_latency_ms;
    g_spu_output_buffer_seconds = spu.output_buffer_seconds;
    g_spu_xa_buffer_seconds = spu.xa_buffer_seconds;
    g_spu_enable_audio_queue = spu.enable_audio_queue;
    g_spu_enable_smooth_trim = spu.enable_smooth_trim;
    g_spu_enable_lag_stutter = spu.enable_lag_stutter;
    g_spu_enable_slowdown_stutter = spu.enable_slowdown_stutter;
    g_spu_show_audio_stats = spu.show_audio_stats;
    g_spu_audio_stats_log = spu.audio_stats_log;
    g_spu_advanced_sound_status = spu.advanced_sound_status;

    // Logging
    g_log_level = log_level;
    g_log_timestamp = log_timestamps;
    g_log_dedupe = log_collapse_repeats;
    g_log_fmv_diagnostics = log_fmv_diagnostics;
    g_log_dedupe_flush = log_repeat_flush;
    g_log_category_mask = log_category_mask;

    // Tracing
    g_trace_dma = trace.dma;
    g_trace_cdrom = trace.cdrom;
    g_trace_cpu = trace.cpu;
    g_trace_bus = trace.bus;
    g_trace_ram = trace.ram;
    g_trace_gpu = trace.gpu;
    g_trace_spu = trace.spu;
    g_trace_irq = trace.irq;
    g_trace_timer = trace.timer;
    g_trace_sio = trace.sio;
    g_trace_burst_cpu = trace.burst_cpu;
    g_trace_stride_cpu = trace.stride_cpu;
    g_trace_burst_bus = trace.burst_bus;
    g_trace_stride_bus = trace.stride_bus;
    g_trace_burst_ram = trace.burst_ram;
    g_trace_stride_ram = trace.stride_ram;
    g_trace_burst_dma = trace.burst_dma;
    g_trace_stride_dma = trace.stride_dma;
    g_trace_burst_cdrom = trace.burst_cdrom;
    g_trace_stride_cdrom = trace.stride_cdrom;
    g_trace_burst_gpu = trace.burst_gpu;
    g_trace_stride_gpu = trace.stride_gpu;
    g_trace_burst_spu = trace.burst_spu;
    g_trace_stride_spu = trace.stride_spu;
    g_trace_burst_irq = trace.burst_irq;
    g_trace_stride_irq = trace.stride_irq;
    g_trace_burst_timer = trace.burst_timer;
    g_trace_stride_timer = trace.stride_timer;
    g_trace_burst_sio = trace.burst_sio;
    g_trace_stride_sio = trace.stride_sio;

    // Diagnostics
    g_cpu_deep_diagnostics = cpu_deep_diagnostics;
    g_profile_detailed_timing = detailed_profiling;

    // Experimental
    g_experimental_bios_size_mode = experimental_bios_size_mode;
    g_unsafe_ps2_bios_mode = unsafe_ps2_bios_mode;
    g_experimental_unhandled_special_returns_zero = experimental_unhandled_special_returns_zero;
    g_experimental_dma_command_sanitizer = experimental_dma_command_sanitizer;

    // MDEC debug
    g_mdec_debug_disable_dma1_reorder = mdec_debug.disable_dma1_reorder;
    g_mdec_debug_disable_chroma = mdec_debug.disable_chroma;
    g_mdec_debug_disable_luma = mdec_debug.disable_luma;
    g_mdec_debug_force_solid_output = mdec_debug.force_solid_output;
    g_mdec_debug_swap_input_halfwords = mdec_debug.swap_input_halfwords;
    g_mdec_debug_compare_macroblocks = mdec_debug.compare_macroblocks;
    g_mdec_debug_upload_probe = mdec_debug.upload_probe;
    g_mdec_debug_color_block_mask = mdec_debug.color_block_mask;
}
