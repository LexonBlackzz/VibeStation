#pragma once
// VibeStation - Common types and utilities

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// Fixed-width integer aliases
using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;
using s8 = int8_t;
using s16 = int16_t;
using s32 = int32_t;
using s64 = int64_t;

// Bit manipulation helpers
constexpr u32 bit(u32 value, int n) { return (value >> n) & 1; }
constexpr u32 bits(u32 value, int hi, int lo) {
  return (value >> lo) & ((1u << (hi - lo + 1)) - 1);
}
constexpr s32 sign_extend_16(u16 v) { return static_cast<s16>(v); }
constexpr s32 sign_extend_8(u8 v) { return static_cast<s8>(v); }

// Logging
enum class LogLevel { Debug, Info, Warn, Error };
enum class LogCategory : u32 {
  General = 1u << 0,
  App = 1u << 1,
  Cpu = 1u << 2,
  Bus = 1u << 3,
  Ram = 1u << 4,
  Dma = 1u << 5,
  Cdrom = 1u << 6,
  Gpu = 1u << 7,
  Spu = 1u << 8,
  Sio = 1u << 9,
  Timer = 1u << 10,
  Irq = 1u << 11,
  Input = 1u << 12,
  Bios = 1u << 13,
};

inline LogLevel g_log_level = LogLevel::Info;
inline FILE *g_log_file = nullptr;
inline u32 g_log_category_mask = 0xFFFFFFFFu;
inline bool g_log_timestamp = true;
inline bool g_log_dedupe = true;
inline u32 g_log_dedupe_flush = 1000;
inline std::string g_log_last_line;
inline u64 g_log_repeat_count = 0;
struct LogUiEntry {
  u64 seq = 0;
  LogLevel level = LogLevel::Info;
  LogCategory category = LogCategory::General;
  std::string timestamp;
  std::string prefix;
  std::string message;
};
inline constexpr size_t kLogUiEntryLimit = 2000u;
inline std::deque<LogUiEntry> g_log_ui_entries;
inline std::mutex g_log_ui_mutex;
inline u64 g_log_ui_next_seq = 1u;

inline bool g_trace_dma = false;
inline bool g_trace_cdrom = false;
inline bool g_trace_cpu = false;
inline bool g_trace_bus = false;
inline bool g_trace_ram = false;
inline bool g_trace_gpu = false;
inline bool g_trace_spu = false;
inline bool g_trace_irq = false;
inline bool g_trace_timer = false;
inline bool g_trace_sio = false;
// Expensive CPU sanity diagnostics (helper-range probes, suspicious RA/SP checks).
// Keep disabled for normal/turbo gameplay performance.
inline bool g_cpu_deep_diagnostics = true;
// Expensive RAM watchpoint diagnostics around BIOS boot streams.
inline bool g_ram_watch_diagnostics = false;
inline u32 g_trace_burst_cpu = 128;
inline u32 g_trace_stride_cpu = 32768;
inline u32 g_trace_burst_bus = 256;
inline u32 g_trace_stride_bus = 16384;
inline u32 g_trace_burst_ram = 32;
inline u32 g_trace_stride_ram = 131072;
inline u32 g_trace_burst_dma = 64;
inline u32 g_trace_stride_dma = 2048;
inline u32 g_trace_burst_cdrom = 128;
inline u32 g_trace_stride_cdrom = 256;
inline u32 g_trace_burst_gpu = 512;
inline u32 g_trace_stride_gpu = 2048;
inline u32 g_trace_burst_spu = 128;
inline u32 g_trace_stride_spu = 4096;
inline u32 g_trace_burst_irq = 128;
inline u32 g_trace_stride_irq = 2048;
inline u32 g_trace_burst_timer = 64;
inline u32 g_trace_stride_timer = 2048;
inline u32 g_trace_burst_sio = 64;
inline u32 g_trace_stride_sio = 2048;
inline bool g_experimental_bios_size_mode = false;
inline bool g_unsafe_ps2_bios_mode = false;
inline bool g_experimental_unhandled_special_returns_zero = false;
inline bool g_low_spec_mode = false;
inline bool g_gpu_fast_mode = false;
inline bool g_spu_advanced_sound_status = false;
inline u32 g_spu_desired_samples = 64u;
inline bool g_spu_enable_audio_queue = true;
inline bool g_spu_force_audio_queue = false;
inline float g_spu_output_buffer_seconds = 5.0f;
inline float g_spu_xa_buffer_seconds = 0.12f;
// Runtime MDEC isolation toggles for FMV debugging.
inline bool g_mdec_debug_disable_dma1_reorder = false;
inline bool g_mdec_debug_disable_chroma = false;
inline bool g_mdec_debug_disable_luma = false;
inline bool g_mdec_debug_force_solid_output = false;
inline bool g_mdec_debug_swap_input_halfwords = false;
// Extra MDEC validation re-decodes and repacks every colored macroblock.
// Keep it opt-in because it is expensive enough to impact FMV playback.
inline bool g_mdec_debug_compare_macroblocks = false;
// Upload probe hooks DMA, RAM, and GPU image-load paths. Keep it opt-in.
inline bool g_mdec_debug_upload_probe = false;
// Bitmask for colored 8x8 luma blocks inside a 16x16 macroblock:
// bit0=Y1 (top-left), bit1=Y2 (top-right), bit2=Y3 (bottom-left), bit3=Y4 (bottom-right).
inline u8 g_mdec_debug_color_block_mask = 0x0Fu;
// Keep detailed timing opt-in; enabling it in debug builds can dominate frame
// time due to high-frequency clock sampling in hot loops.
inline bool g_profile_detailed_timing = false;

enum class DeinterlaceMode : u8 {
  Weave = 0,
  Bob = 1,
  Blend = 2,
};

enum class OutputResolutionMode : u8 {
  R320x240 = 0,
  R640x480 = 1,
  R1024x768 = 2,
};

inline DeinterlaceMode g_deinterlace_mode = DeinterlaceMode::Weave;
inline OutputResolutionMode g_output_resolution_mode =
    OutputResolutionMode::R320x240;

inline constexpr u32 log_category_bit(LogCategory cat) {
  return static_cast<u32>(cat);
}

inline bool log_category_enabled(LogCategory cat) {
  return (g_log_category_mask & log_category_bit(cat)) != 0;
}

inline bool trace_should_log(u64 &counter, u32 burst, u32 stride) {
  ++counter;
  if (counter <= static_cast<u64>(burst)) {
    return true;
  }
  if (stride == 0) {
    return false;
  }
  return (counter % static_cast<u64>(stride)) == 0;
}

inline const char *log_category_name(LogCategory cat) {
  switch (cat) {
  case LogCategory::General:
    return "General";
  case LogCategory::App:
    return "App";
  case LogCategory::Cpu:
    return "CPU";
  case LogCategory::Bus:
    return "BUS";
  case LogCategory::Ram:
    return "RAM";
  case LogCategory::Dma:
    return "DMA";
  case LogCategory::Cdrom:
    return "CDROM";
  case LogCategory::Gpu:
    return "GPU";
  case LogCategory::Spu:
    return "SPU";
  case LogCategory::Sio:
    return "SIO";
  case LogCategory::Timer:
    return "Timer";
  case LogCategory::Irq:
    return "IRQ";
  case LogCategory::Input:
    return "Input";
  case LogCategory::Bios:
    return "BIOS";
  default:
    return "General";
  }
}

inline LogCategory log_detect_category(const char *msg) {
  if (!msg || !msg[0])
    return LogCategory::General;
  if (std::strncmp(msg, "CPU:", 4) == 0)
    return LogCategory::Cpu;
  if (std::strncmp(msg, "BUS:", 4) == 0)
    return LogCategory::Bus;
  if (std::strncmp(msg, "RAM:", 4) == 0)
    return LogCategory::Ram;
  if (std::strncmp(msg, "DMA:", 4) == 0)
    return LogCategory::Dma;
  if (std::strncmp(msg, "CDROM:", 6) == 0)
    return LogCategory::Cdrom;
  if (std::strncmp(msg, "GPU:", 4) == 0)
    return LogCategory::Gpu;
  if (std::strncmp(msg, "SPU:", 4) == 0)
    return LogCategory::Spu;
  if (std::strncmp(msg, "SIO:", 4) == 0)
    return LogCategory::Sio;
  if (std::strncmp(msg, "Timers:", 7) == 0)
    return LogCategory::Timer;
  if (std::strncmp(msg, "IRQ:", 4) == 0)
    return LogCategory::Irq;
  if (std::strncmp(msg, "Input:", 6) == 0)
    return LogCategory::Input;
  if (std::strncmp(msg, "BIOS:", 5) == 0)
    return LogCategory::Bios;
  if (std::strncmp(msg, "Renderer:", 9) == 0)
    return LogCategory::App;
  return LogCategory::General;
}

inline void log_push_ui_entry(LogLevel level, LogCategory category,
                              const char *prefix, const char *buffer,
                              const char *timestamp) {
  std::lock_guard<std::mutex> lock(g_log_ui_mutex);
  if (g_log_ui_entries.size() >= kLogUiEntryLimit) {
    g_log_ui_entries.pop_front();
  }
  LogUiEntry entry;
  entry.seq = g_log_ui_next_seq++;
  entry.level = level;
  entry.category = category;
  entry.timestamp = timestamp ? std::string(timestamp) : std::string();
  entry.prefix = prefix ? std::string(prefix) : std::string();
  entry.message = buffer ? std::string(buffer) : std::string();
  g_log_ui_entries.push_back(std::move(entry));
}

inline void log_clear_ui_entries() {
  std::lock_guard<std::mutex> lock(g_log_ui_mutex);
  g_log_ui_entries.clear();
}

inline void log_emit_line(LogLevel level, LogCategory category,
                          const char *prefix, const char *buffer) {
  char ts[32] = {};
  if (g_log_timestamp) {
    const auto now = std::chrono::system_clock::now();
    const std::time_t tt = std::chrono::system_clock::to_time_t(now);
    std::tm tmv{};
#if defined(_WIN32)
    localtime_s(&tmv, &tt);
#else
    localtime_r(&tt, &tmv);
#endif
    std::strftime(ts, sizeof(ts), "%H:%M:%S", &tmv);
  }

  if (g_log_timestamp) {
    std::printf("[%s] %s%s\n", ts, prefix, buffer);
  } else {
    std::printf("%s%s\n", prefix, buffer);
  }
  std::fflush(stdout);
  log_push_ui_entry(level, category, prefix, buffer, g_log_timestamp ? ts : "");

  if (g_log_file) {
    if (g_log_timestamp) {
      std::fprintf(g_log_file, "[%s] %s%s\n", ts, prefix, buffer);
    } else {
      std::fprintf(g_log_file, "%s%s\n", prefix, buffer);
    }
    std::fflush(g_log_file);
  }
}

inline void log_flush_repeats() {
  if (g_log_repeat_count == 0 || g_log_last_line.empty()) {
    g_log_repeat_count = 0;
    return;
  }
  char msg[128];
  std::snprintf(msg, sizeof(msg), "Previous line repeated %llu times",
                static_cast<unsigned long long>(g_log_repeat_count));
  log_emit_line(LogLevel::Info, LogCategory::General, "[INF] ", msg);
  g_log_repeat_count = 0;
}

inline void log_write_categorized(LogCategory cat, const char *prefix, const char *fmt, ...) {
  if (!log_category_enabled(cat)) {
    return;
  }

  char buffer[2048];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  std::string composed(prefix);
  composed += buffer;

  if (g_log_dedupe) {
    if (!g_log_last_line.empty() && composed == g_log_last_line) {
      ++g_log_repeat_count;
      if (g_log_dedupe_flush > 0 &&
          (g_log_repeat_count % static_cast<u64>(g_log_dedupe_flush)) == 0) {
        log_flush_repeats();
      }
      return;
    }
    log_flush_repeats();
    g_log_last_line = composed;
  }

  LogLevel level = LogLevel::Info;
  if (std::strcmp(prefix, "[DBG] ") == 0) {
    level = LogLevel::Debug;
  } else if (std::strcmp(prefix, "[WRN] ") == 0) {
    level = LogLevel::Warn;
  } else if (std::strcmp(prefix, "[ERR] ") == 0) {
    level = LogLevel::Error;
  }

  log_emit_line(level, cat, prefix, buffer);
}

#define LOG_DEBUG(fmt, ...)                                                     \
  do {                                                                          \
    if (g_log_level <= LogLevel::Debug) {                                       \
      static LogCategory _cat = log_detect_category(fmt);                       \
      log_write_categorized(_cat, "[DBG] ", fmt, ##__VA_ARGS__);                 \
    }                                                                           \
  } while (0)
#define LOG_INFO(fmt, ...)                                                      \
  do {                                                                          \
    if (g_log_level <= LogLevel::Info) {                                        \
      static LogCategory _cat = log_detect_category(fmt);                       \
      log_write_categorized(_cat, "[INF] ", fmt, ##__VA_ARGS__);                 \
    }                                                                           \
  } while (0)
#define LOG_WARN(fmt, ...)                                                      \
  do {                                                                          \
    if (g_log_level <= LogLevel::Warn) {                                        \
      static LogCategory _cat = log_detect_category(fmt);                       \
      log_write_categorized(_cat, "[WRN] ", fmt, ##__VA_ARGS__);                 \
    }                                                                           \
  } while (0)
#define LOG_ERROR(fmt, ...)                                                     \
  do {                                                                          \
    if (g_log_level <= LogLevel::Error) {                                       \
      static LogCategory _cat = log_detect_category(fmt);                       \
      log_write_categorized(_cat, "[ERR] ", fmt, ##__VA_ARGS__);                 \
    }                                                                           \
  } while (0)

#define LOG_CAT_DEBUG(cat, fmt, ...)                                            \
  do {                                                                          \
    if (g_log_level <= LogLevel::Debug)                                         \
      log_write_categorized(cat, "[DBG] ", fmt, ##__VA_ARGS__);                \
  } while (0)
#define LOG_CAT_INFO(cat, fmt, ...)                                             \
  do {                                                                          \
    if (g_log_level <= LogLevel::Info)                                          \
      log_write_categorized(cat, "[INF] ", fmt, ##__VA_ARGS__);                \
  } while (0)
#define LOG_CAT_WARN(cat, fmt, ...)                                             \
  do {                                                                          \
    if (g_log_level <= LogLevel::Warn)                                          \
      log_write_categorized(cat, "[WRN] ", fmt, ##__VA_ARGS__);                \
  } while (0)
#define LOG_CAT_ERROR(cat, fmt, ...)                                            \
  do {                                                                          \
    if (g_log_level <= LogLevel::Error)                                         \
      log_write_categorized(cat, "[ERR] ", fmt, ##__VA_ARGS__);                \
  } while (0)

// PS1 constants
namespace psx {
constexpr u32 CPU_CLOCK_HZ = 33'868'800;     // 33.8688 MHz
constexpr u32 CYCLES_PER_FRAME = CPU_CLOCK_HZ / 60;
constexpr u32 RAM_SIZE = 2 * 1024 * 1024;    // 2 MB
constexpr u32 SCRATCHPAD_SIZE = 1024;        // 1 KB
constexpr u32 BIOS_SIZE = 512 * 1024;        // 512 KB
constexpr u32 VRAM_WIDTH = 1024;
constexpr u32 VRAM_HEIGHT = 512;

// Memory map base addresses
constexpr u32 RAM_BASE = 0x00000000;
constexpr u32 EXPANSION1_BASE = 0x1F000000;
constexpr u32 SCRATCHPAD_BASE = 0x1F800000;
constexpr u32 IO_BASE = 0x1F801000;
constexpr u32 EXPANSION2_BASE = 0x1F802000;
constexpr u32 BIOS_BASE = 0x1FC00000;

// KSEG address regions
constexpr u32 KUSEG_BASE = 0x00000000;
constexpr u32 KSEG0_BASE = 0x80000000;
constexpr u32 KSEG1_BASE = 0xA0000000;
constexpr u32 KSEG2_BASE = 0xC0000000;

// Mask to convert virtual to physical address
inline u32 mask_address(u32 addr) {
  static constexpr u32 REGION_MASK[8] = {
      0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
      0x7FFFFFFF,
      0x1FFFFFFF,
      0xFFFFFFFF, 0xFFFFFFFF};
  return addr & REGION_MASK[addr >> 29];
}
} // namespace psx

