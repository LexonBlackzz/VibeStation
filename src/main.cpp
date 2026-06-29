// ── VibeStation — PS1 Emulator ──────────────────────────────────────
// Entry point

#define SDL_MAIN_HANDLED // Prevent SDL from redefining main()
#include "core/system.h"
#include "core/input_recorder.h"
#include "input/controller.h"
#include "platform/cpu_backend_compare_runner.h"
#include "ui/app.h"
#include "vibestation_version.h"
#include <SDL.h>
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <array>
#include <limits>
#include <memory>
#include <sstream>
#include <string_view>
#include <vector>

struct AutoInputConfig {
  u16 mask = 0;
  int start_frame = 1;
  int end_frame = 0;
  int period_frames = 1;
  int hold_frames = 1;

  bool enabled() const { return mask != 0; }
};

static AutoInputConfig g_auto_input;

// Input recording/playback
static InputRecorder::Config g_input_recorder_config;

static bool parse_psx_button_name(const std::string &name, PsxButton &button) {
  std::string v = name;
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  v.erase(std::remove(v.begin(), v.end(), '-'), v.end());
  v.erase(std::remove(v.begin(), v.end(), '_'), v.end());

  if (v == "select") button = PsxButton::Select;
  else if (v == "start") button = PsxButton::Start;
  else if (v == "up") button = PsxButton::Up;
  else if (v == "right") button = PsxButton::Right;
  else if (v == "down") button = PsxButton::Down;
  else if (v == "left") button = PsxButton::Left;
  else if (v == "l1") button = PsxButton::L1;
  else if (v == "r1") button = PsxButton::R1;
  else if (v == "l2") button = PsxButton::L2;
  else if (v == "r2") button = PsxButton::R2;
  else if (v == "triangle") button = PsxButton::Triangle;
  else if (v == "circle") button = PsxButton::Circle;
  else if (v == "cross" || v == "x") button = PsxButton::Cross;
  else if (v == "square") button = PsxButton::Square;
  else return false;
  return true;
}

static bool add_auto_input_buttons(const std::string &spec) {
  std::istringstream input(spec);
  std::string token;
  bool parsed_any = false;
  while (std::getline(input, token, ',')) {
    const size_t begin = token.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
      token.clear();
    } else {
      const size_t end = token.find_last_not_of(" \t\r\n");
      token = token.substr(begin, end - begin + 1u);
    }
    if (token.empty()) {
      continue;
    }
    PsxButton button{};
    if (!parse_psx_button_name(token, button)) {
      return false;
    }
    g_auto_input.mask |= static_cast<u16>(button);
    parsed_any = true;
  }
  return parsed_any;
}

static u16 auto_input_buttons_for_frame(int frame_index) {
  if (!g_auto_input.enabled() || frame_index < g_auto_input.start_frame) {
    return 0xFFFFu;
  }
  if (g_auto_input.end_frame > 0 && frame_index > g_auto_input.end_frame) {
    return 0xFFFFu;
  }

  const int period = std::max(1, g_auto_input.period_frames);
  const int hold = std::max(1, std::min(g_auto_input.hold_frames, period));
  const int phase = (frame_index - g_auto_input.start_frame) % period;
  if (phase >= hold) {
    return 0xFFFFu;
  }
  return static_cast<u16>(0xFFFFu & ~g_auto_input.mask);
}

#ifdef _WIN32
extern "C" {
__declspec(dllexport) unsigned long NvOptimusEnablement = 0x00000001;
__declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}
#endif

static u32 fnv1a_hash_file(const std::string &path, bool &ok) {
  ok = false;
  std::ifstream file(path, std::ios::binary);
  if (!file.is_open()) {
    return 0;
  }

  u32 hash = 2166136261u;
  char chunk[4096];
  while (file.good()) {
    file.read(chunk, sizeof(chunk));
    std::streamsize got = file.gcount();
    for (std::streamsize i = 0; i < got; ++i) {
      hash ^= static_cast<u8>(chunk[i]);
      hash *= 16777619u;
    }
  }
  ok = true;
  return hash;
}

static u32 fnv1a_hash_samples(const std::vector<s16> &samples) {
  u32 hash = 2166136261u;
  for (const s16 sample : samples) {
    const u16 v = static_cast<u16>(sample);
    hash ^= static_cast<u8>(v & 0xFF);
    hash *= 16777619u;
    hash ^= static_cast<u8>(v >> 8);
    hash *= 16777619u;
  }
  return hash;
}

static std::string cue_trim_copy(std::string text) {
  const size_t begin = text.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return {};
  }
  const size_t end = text.find_last_not_of(" \t\r\n");
  return text.substr(begin, end - begin + 1u);
}

static std::string parse_cue_file_target(const std::string &line) {
  const size_t q0 = line.find('"');
  if (q0 != std::string::npos) {
    const size_t q1 = line.find('"', q0 + 1u);
    if (q1 != std::string::npos && q1 > q0 + 1u) {
      return line.substr(q0 + 1u, q1 - q0 - 1u);
    }
  }

  std::istringstream iss(line);
  std::string token;
  std::string file_name;
  iss >> token >> file_name;
  return file_name;
}

static std::string resolve_first_bin_from_cue_cli(
    const std::filesystem::path &cue_path) {
  std::ifstream cue(cue_path);
  if (!cue.is_open()) {
    return {};
  }

  std::string line;
  while (std::getline(cue, line)) {
    line = cue_trim_copy(line);
    if (line.empty() || line[0] == ';' || line.size() < 4u) {
      continue;
    }

    std::string head = line.substr(0u, 4u);
    std::transform(head.begin(), head.end(), head.begin(),
                   [](unsigned char c) {
                     return static_cast<char>(std::toupper(c));
                   });
    if (head != "FILE") {
      continue;
    }

    const std::string file_name = parse_cue_file_target(line);
    if (file_name.empty()) {
      continue;
    }

    std::filesystem::path resolved(file_name);
    if (!resolved.is_absolute()) {
      resolved = cue_path.parent_path() / resolved;
    }
    return std::filesystem::exists(resolved) ? resolved.string() : std::string();
  }
  return {};
}

static void log_mdec_summary(const char *prefix, const System &sys) {
  const auto &stats = sys.mdec_debug_stats();
  const auto &probe = sys.mdec_upload_probe();
  const auto &compare = sys.mdec_debug_compare();
  const u64 qscale_avg =
      (stats.blocks_decoded != 0u) ? (stats.qscale_sum / stats.blocks_decoded) : 0u;

  LOG_INFO(
      "%s_MDEC decode=%llu set_quant=%llu set_scale=%llu blocks=%llu "
      "dc_only=%llu qscale_zero=%llu qscale_avg=%llu qscale_max=%u "
      "nonzero_coeff=%llu eob=%llu overflow=%llu dma_in_req=%d dma_out_req=%d "
      "out_words=%u out_depth=%u out_block=%u out_mb_seq=%u dma0_seen=%d dma1_seen=%d "
      "gpu_upload_seen=%d gpu_copy_count=%u",
      prefix, static_cast<unsigned long long>(stats.decode_commands),
      static_cast<unsigned long long>(stats.set_quant_commands),
      static_cast<unsigned long long>(stats.set_scale_commands),
      static_cast<unsigned long long>(stats.blocks_decoded),
      static_cast<unsigned long long>(stats.dc_only_blocks),
      static_cast<unsigned long long>(stats.qscale_zero_blocks),
      static_cast<unsigned long long>(qscale_avg), stats.qscale_max,
      static_cast<unsigned long long>(stats.nonzero_coeff_count),
      static_cast<unsigned long long>(stats.eob_markers),
      static_cast<unsigned long long>(stats.overflow_breaks),
      sys.mdec_dma_in_request() ? 1 : 0, sys.mdec_dma_out_request() ? 1 : 0,
      sys.mdec_dma_out_words_available(),
      static_cast<unsigned>(sys.mdec_dma_out_depth()),
      static_cast<unsigned>(sys.mdec_dma_out_block()),
      sys.mdec_dma_out_macroblock_seq(), probe.dma0_seen ? 1 : 0,
      probe.dma1_seen ? 1 : 0,
      probe.gpu_upload_seen ? 1 : 0, probe.gpu_copy_count);

  if (stats.command_history_count != 0u) {
    const u32 latest_index = (stats.command_history_count - 1u) %
                             static_cast<u32>(Mdec::DebugStats::kCommandHistory);
    LOG_INFO("%s_MDEC_LASTCMD id=%u word=0x%08X writes=%u controls=%u", prefix,
             static_cast<unsigned>(stats.command_history_ids[latest_index]),
             stats.command_history_words[latest_index], stats.write_history_count,
             stats.control_history_count);
  }

  if (probe.dma1_seen) {
    LOG_INFO(
        "%s_MDEC_DMA1 base=0x%06X words=%u depth=%u first_block=%u "
        "sample_count=%u a0=0x%06X w0=0x%08X a1=0x%06X w1=0x%08X "
        "mb_hist=%u mb0=%u@0x%06X mbw0=0x%08X",
        prefix, probe.dma1_base_addr, probe.dma1_words,
        static_cast<unsigned>(probe.dma1_depth),
        static_cast<unsigned>(probe.dma1_first_block),
        probe.dma1_sample_count, probe.dma1_addrs[0],
        probe.dma1_words_sample[0], probe.dma1_addrs[1],
        probe.dma1_words_sample[1], probe.dma1_mb_hist_count,
        probe.dma1_mb_hist_seq[0], probe.dma1_mb_hist_addrs[0],
        probe.dma1_mb_hist_words_sample[0]);
  }

  if (probe.gpu_upload_seen) {
    LOG_INFO(
        "%s_GPU_UPLOAD count=%u xy=%u,%u wh=%u,%u words=%u/%u "
        "src_base=0x%06X range=%u sample_count=%u "
        "g0=0x%08X from_dma=%u ga0=0x%06X g1=0x%08X from_dma1=%u ga1=0x%06X "
        "src_writes=%u sw0=0x%08X@0x%06X origin0=0x%02X",
        prefix, probe.gpu_upload_count, static_cast<unsigned>(probe.gpu_x),
        static_cast<unsigned>(probe.gpu_y), static_cast<unsigned>(probe.gpu_w),
        static_cast<unsigned>(probe.gpu_h), probe.gpu_words_seen,
        probe.gpu_total_words, probe.gpu_dma_src_base,
        probe.gpu_dma_src_range_bytes, probe.gpu_sample_count,
        probe.gpu_words_sample[0], static_cast<unsigned>(probe.gpu_word_from_dma[0]),
        probe.gpu_dma_src_addrs[0], probe.gpu_words_sample[1],
        static_cast<unsigned>(probe.gpu_word_from_dma[1]),
        probe.gpu_dma_src_addrs[1], probe.gpu_src_write_sample_count,
        probe.gpu_src_write_values[0], probe.gpu_src_write_addrs[0],
        static_cast<unsigned>(probe.gpu_src_write_origin[0]));
  }

  if (probe.gpu_copy_sample_count != 0u) {
    LOG_INFO(
        "%s_GPU_COPY count=%u sample_count=%u "
        "copy0=%u,%u->%u,%u %ux%u copy1=%u,%u->%u,%u %ux%u",
        prefix, probe.gpu_copy_count, probe.gpu_copy_sample_count,
        static_cast<unsigned>(probe.gpu_copy_src_x[0]),
        static_cast<unsigned>(probe.gpu_copy_src_y[0]),
        static_cast<unsigned>(probe.gpu_copy_dst_x[0]),
        static_cast<unsigned>(probe.gpu_copy_dst_y[0]),
        static_cast<unsigned>(probe.gpu_copy_w[0]),
        static_cast<unsigned>(probe.gpu_copy_h[0]),
        static_cast<unsigned>(probe.gpu_copy_src_x[1]),
        static_cast<unsigned>(probe.gpu_copy_src_y[1]),
        static_cast<unsigned>(probe.gpu_copy_dst_x[1]),
        static_cast<unsigned>(probe.gpu_copy_dst_y[1]),
        static_cast<unsigned>(probe.gpu_copy_w[1]),
        static_cast<unsigned>(probe.gpu_copy_h[1]));
  }

  if (compare.captured || compare.macroblocks_compared != 0u) {
    const char *stage = "none";
    switch (compare.stage) {
    case Mdec::DebugCompare::Stage::Coeff:
      stage = "coeff";
      break;
    case Mdec::DebugCompare::Stage::Idct:
      stage = "idct";
      break;
    case Mdec::DebugCompare::Stage::Rgb:
      stage = "rgb";
      break;
    case Mdec::DebugCompare::Stage::Match:
      stage = "match";
      break;
    case Mdec::DebugCompare::Stage::None:
    default:
      break;
    }
    LOG_INFO(
        "%s_MDEC_COMPARE captured=%d color=%d compared=%llu stage=%s "
        "block=%u index=%u current=0x%08X reference=0x%08X input_halfwords=%u "
        "qscale=%u,%u,%u,%u,%u,%u",
        prefix, compare.captured ? 1 : 0, compare.color_macroblock ? 1 : 0,
        static_cast<unsigned long long>(compare.macroblocks_compared), stage,
        static_cast<unsigned>(compare.mismatch_block),
        static_cast<unsigned>(compare.mismatch_index),
        static_cast<u32>(compare.current_value),
        static_cast<u32>(compare.reference_value), compare.input_halfword_count,
        static_cast<unsigned>(compare.qscales[0]),
        static_cast<unsigned>(compare.qscales[1]),
        static_cast<unsigned>(compare.qscales[2]),
        static_cast<unsigned>(compare.qscales[3]),
        static_cast<unsigned>(compare.qscales[4]),
        static_cast<unsigned>(compare.qscales[5]));
  }
}

static void write_u16_le(std::ofstream &out, u16 v) {
  out.put(static_cast<char>(v & 0xFF));
  out.put(static_cast<char>((v >> 8) & 0xFF));
}

static void write_u32_le(std::ofstream &out, u32 v) {
  out.put(static_cast<char>(v & 0xFF));
  out.put(static_cast<char>((v >> 8) & 0xFF));
  out.put(static_cast<char>((v >> 16) & 0xFF));
  out.put(static_cast<char>((v >> 24) & 0xFF));
}

static bool write_wav_s16_stereo(const std::string &path,
                                 const std::vector<s16> &samples,
                                 u32 sample_rate) {
  std::ofstream out(path, std::ios::binary);
  if (!out.is_open()) {
    return false;
  }
  const u32 data_size = static_cast<u32>(samples.size() * sizeof(s16));
  const u32 riff_size = 36u + data_size;
  const u16 channels = 2;
  const u16 bits_per_sample = 16;
  const u16 block_align = static_cast<u16>(channels * (bits_per_sample / 8u));
  const u32 byte_rate = sample_rate * static_cast<u32>(block_align);

  out.write("RIFF", 4);
  write_u32_le(out, riff_size);
  out.write("WAVE", 4);
  out.write("fmt ", 4);
  write_u32_le(out, 16);
  write_u16_le(out, 1); // PCM
  write_u16_le(out, channels);
  write_u32_le(out, sample_rate);
  write_u32_le(out, byte_rate);
  write_u16_le(out, block_align);
  write_u16_le(out, bits_per_sample);
  out.write("data", 4);
  write_u32_le(out, data_size);
  out.write(reinterpret_cast<const char *>(samples.data()),
            static_cast<std::streamsize>(data_size));
  return out.good();
}

struct GpuDebugDumpRegion {
  u16 x = 0;
  u16 y = 0;
  u16 w = 0;
  u16 h = 0;
  bool valid = false;
};

static GpuDebugDumpRegion compute_gpu_poly_source_region(
    const GpuCommandDebugInfo &gcmd, u32 poly_index) {
  GpuDebugDumpRegion region{};
  const u8 vertex_count = gcmd.poly_vertex_count[poly_index];
  if (!gcmd.poly_textured[poly_index] || vertex_count < 3) {
    return region;
  }

  u8 min_u = 255;
  u8 min_v = 255;
  u8 max_u = 0;
  u8 max_v = 0;
  for (u8 i = 0; i < vertex_count && i < 4; ++i) {
    min_u = std::min(min_u, gcmd.poly_u[poly_index][i]);
    min_v = std::min(min_v, gcmd.poly_v[poly_index][i]);
    max_u = std::max(max_u, gcmd.poly_u[poly_index][i]);
    max_v = std::max(max_v, gcmd.poly_v[poly_index][i]);
  }

  const u16 texpage = gcmd.poly_texpage[poly_index];
  const u8 depth = gcmd.poly_depth[poly_index];
  const u16 tex_base_x = static_cast<u16>((texpage & 0xFu) * 64u);
  const u16 tex_base_y = static_cast<u16>(((texpage >> 4) & 0x1u) * 256u);
  u16 src_x = tex_base_x;
  u16 src_w = 0;
  switch (depth & 0x3u) {
  case 0: {
    src_x = static_cast<u16>(tex_base_x + (min_u >> 2));
    src_w = static_cast<u16>((max_u >> 2) - (min_u >> 2) + 1u);
    break;
  }
  case 1: {
    src_x = static_cast<u16>(tex_base_x + (min_u >> 1));
    src_w = static_cast<u16>((max_u >> 1) - (min_u >> 1) + 1u);
    break;
  }
  case 2:
    src_x = static_cast<u16>(tex_base_x + min_u);
    src_w = static_cast<u16>(max_u - min_u + 1u);
    break;
  default:
    return region;
  }

  region.x = static_cast<u16>(src_x & (psx::VRAM_WIDTH - 1u));
  region.y = static_cast<u16>((tex_base_y + min_v) & (psx::VRAM_HEIGHT - 1u));
  region.w = src_w;
  region.h = static_cast<u16>(max_v - min_v + 1u);
  region.valid = true;
  return region;
}

static u32 compute_gpu_poly_bbox_area(const GpuCommandDebugInfo &gcmd,
                                      u32 poly_index) {
  const u8 vertex_count = gcmd.poly_vertex_count[poly_index];
  if (vertex_count < 3) {
    return 0;
  }
  s16 min_x = gcmd.poly_x[poly_index][0];
  s16 max_x = gcmd.poly_x[poly_index][0];
  s16 min_y = gcmd.poly_y[poly_index][0];
  s16 max_y = gcmd.poly_y[poly_index][0];
  for (u8 i = 1; i < vertex_count && i < 4; ++i) {
    min_x = std::min(min_x, gcmd.poly_x[poly_index][i]);
    max_x = std::max(max_x, gcmd.poly_x[poly_index][i]);
    min_y = std::min(min_y, gcmd.poly_y[poly_index][i]);
    max_y = std::max(max_y, gcmd.poly_y[poly_index][i]);
  }
  const int width = std::max(0, static_cast<int>(max_x) - static_cast<int>(min_x));
  const int height = std::max(0, static_cast<int>(max_y) - static_cast<int>(min_y));
  return static_cast<u32>(width * height);
}

static u32 compute_gpu_poly_visible_area(const GpuCommandDebugInfo &gcmd,
                                         u32 poly_index,
                                         const DisplayDebugInfo &gdisp) {
  const u8 vertex_count = gcmd.poly_vertex_count[poly_index];
  if (vertex_count < 3 || gdisp.display_vram_width <= 0 ||
      gdisp.display_vram_height <= 0) {
    return 0;
  }
  s16 min_x = gcmd.poly_x[poly_index][0];
  s16 max_x = gcmd.poly_x[poly_index][0];
  s16 min_y = gcmd.poly_y[poly_index][0];
  s16 max_y = gcmd.poly_y[poly_index][0];
  for (u8 i = 1; i < vertex_count && i < 4; ++i) {
    min_x = std::min(min_x, gcmd.poly_x[poly_index][i]);
    max_x = std::max(max_x, gcmd.poly_x[poly_index][i]);
    min_y = std::min(min_y, gcmd.poly_y[poly_index][i]);
    max_y = std::max(max_y, gcmd.poly_y[poly_index][i]);
  }
  const int clipped_min_x =
      std::max(static_cast<int>(min_x), gdisp.display_vram_left);
  const int clipped_max_x = std::min(static_cast<int>(max_x),
                                     gdisp.display_vram_left +
                                         gdisp.display_vram_width);
  const int clipped_min_y =
      std::max(static_cast<int>(min_y), gdisp.display_vram_top);
  const int clipped_max_y = std::min(static_cast<int>(max_y),
                                     gdisp.display_vram_top +
                                         gdisp.display_vram_height);
  const int width = std::max(0, clipped_max_x - clipped_min_x);
  const int height = std::max(0, clipped_max_y - clipped_min_y);
  return static_cast<u32>(width * height);
}

static void dump_gpu_debug_frame(std::ofstream &out, int frame_index,
                                 const System &sys) {
  const auto gdisp = sys.gpu_display_debug_info();
  const auto gcmd = sys.gpu_command_debug_info();
  const auto &probe = sys.mdec_upload_probe();

  out << "FRAME " << frame_index << "\n";
  out << "DISPLAY mode=" << gdisp.mode_width << "x" << gdisp.mode_height
      << " width=" << gdisp.width << " height=" << gdisp.height
      << " src=" << gdisp.src_width << "x" << gdisp.src_height
      << " vram=(" << gdisp.display_vram_left << "," << gdisp.display_vram_top
      << ") wh=(" << gdisp.display_vram_width << "," << gdisp.display_vram_height
      << ") skip=(" << gdisp.display_skip_x << ",0)"
      << " div=" << gdisp.divisor << " 24bit=" << (gdisp.is_24bit ? 1 : 0)
      << " interlace=" << (gdisp.interlaced ? 1 : 0)
      << " texwin_mask=(" << gdisp.tex_window_mask_x << ","
      << gdisp.tex_window_mask_y << ") texwin_off=("
      << gdisp.tex_window_off_x << "," << gdisp.tex_window_off_y << ")\n";
  out << "GP1 area=(" << gdisp.x_start << "," << gdisp.y_start << ")"
      << " hrange=(" << gdisp.x1 << "," << gdisp.x2 << ")"
      << " vrange=(" << gdisp.y1 << "," << gdisp.y2 << ")"
      << " raw_area=0x" << std::hex << gcmd.gp1_display_area_raw
      << " raw_h=0x" << gcmd.gp1_horizontal_range_raw
      << " raw_v=0x" << gcmd.gp1_vertical_range_raw
      << " raw_mode=0x" << gcmd.gp1_display_mode_raw << std::dec
      << " counts=(" << gcmd.gp1_display_area_count << ","
      << gcmd.gp1_horizontal_range_count << ","
      << gcmd.gp1_vertical_range_count << ","
      << gcmd.gp1_display_mode_count << ")\n";
  out << "COUNTS tri=" << gcmd.gp0_textured_tri_count
      << " quad=" << gcmd.gp0_textured_quad_count
      << " rect=" << gcmd.gp0_textured_rect_count
      << " poly=" << gcmd.gp0_poly_count << "\n";

  const u32 upload_hist_count =
      std::min<u32>(probe.gpu_hist_count,
                    static_cast<u32>(System::MdecUploadProbe::kUploadHistory));
  for (u32 i = 0; i < upload_hist_count; ++i) {
    const u32 hist_index =
        (probe.gpu_hist_count - 1u - i) %
        static_cast<u32>(System::MdecUploadProbe::kUploadHistory);
    out << "UP[" << i << "] xy=(" << probe.gpu_hist_x[hist_index] << ","
        << probe.gpu_hist_y[hist_index] << ") wh=("
        << probe.gpu_hist_w[hist_index] << "," << probe.gpu_hist_h[hist_index]
        << ") src=" << (probe.gpu_hist_from_dma[hist_index] ? "DMA2" : "CPU")
        << "\n";
  }
  if (probe.gpu_dma_src_range_bytes != 0) {
    out << "GPU_SRC_COVERAGE base=0x" << std::hex << probe.gpu_dma_src_base
        << std::dec << " bytes=" << probe.gpu_dma_src_range_bytes
        << " expected_words=" << probe.gpu_src_words_expected
        << " seen=" << probe.gpu_src_words_seen
        << " dma1=" << probe.gpu_src_words_dma1
        << " dma_other=" << probe.gpu_src_words_dma_other
        << " cpu=" << probe.gpu_src_words_cpu
        << " missing=" << probe.gpu_src_words_missing
        << " entries=" << probe.gpu_src_write_entries << "\n";
  }
  if (probe.gpu_upload_seen) {
    out << "GPU_UPLOAD_CUR count=" << probe.gpu_upload_count << " xy=("
        << probe.gpu_x << "," << probe.gpu_y << ") wh=(" << probe.gpu_w
        << "," << probe.gpu_h << ") words=" << probe.gpu_words_seen << "/"
        << probe.gpu_total_words << " sample=" << probe.gpu_sample_count;
    for (u32 i = 0; i < probe.gpu_sample_count; ++i) {
      out << " w" << i << "=0x" << std::hex << probe.gpu_words_sample[i]
          << "@0x" << probe.gpu_dma_src_addrs[i] << std::dec
          << (probe.gpu_word_from_dma[i] ? ":dma" : ":cpu");
    }
    out << "\n";
  }

  const u32 poly_count =
      std::min<u32>(gcmd.gp0_poly_count,
                    static_cast<u32>(GpuCommandDebugInfo::kRecentPolys));
  std::array<u32, GpuCommandDebugInfo::kRecentPolys> indices{};
  for (u32 i = 0; i < poly_count; ++i) {
    indices[static_cast<size_t>(i)] =
        (gcmd.gp0_poly_count - 1u - i) %
        static_cast<u32>(GpuCommandDebugInfo::kRecentPolys);
  }

  out << "RECENT\n";
  const u32 rect_count =
      std::min<u32>(gcmd.gp0_textured_rect_count,
                    static_cast<u32>(GpuCommandDebugInfo::kRecentRects));
  for (u32 i = 0; i < std::min<u32>(rect_count, 40u); ++i) {
    const u32 rect_index =
        (gcmd.gp0_textured_rect_count - 1u - i) %
        static_cast<u32>(GpuCommandDebugInfo::kRecentRects);
    out << "  RECT[" << i << "] op=0x" << std::hex
        << static_cast<unsigned>(gcmd.rect_opcode[rect_index]) << std::dec
        << " xy=(" << gcmd.rect_x[rect_index] << ","
        << gcmd.rect_y[rect_index] << ") wh=("
        << gcmd.rect_w[rect_index] << "," << gcmd.rect_h[rect_index]
        << ") uv=(" << static_cast<unsigned>(gcmd.rect_u[rect_index]) << ","
        << static_cast<unsigned>(gcmd.rect_v[rect_index]) << ")"
        << " page=0x" << std::hex
        << static_cast<unsigned>(gcmd.rect_texpage[rect_index])
        << " clut=0x" << static_cast<unsigned>(gcmd.rect_clut[rect_index])
        << std::dec << " depth="
        << static_cast<unsigned>(gcmd.rect_depth[rect_index])
        << " raw=" << static_cast<unsigned>(gcmd.rect_raw[rect_index])
        << " rgb=(" << static_cast<unsigned>(gcmd.rect_r[rect_index]) << ","
        << static_cast<unsigned>(gcmd.rect_g[rect_index]) << ","
        << static_cast<unsigned>(gcmd.rect_b[rect_index]) << ")\n";
  }
  for (u32 i = 0; i < std::min<u32>(poly_count, 64u); ++i) {
    const u32 poly_index = indices[static_cast<size_t>(i)];
    const GpuDebugDumpRegion src =
        compute_gpu_poly_source_region(gcmd, poly_index);
    out << "  POLY[" << i << "] op=0x" << std::hex
        << static_cast<unsigned>(gcmd.poly_opcode[poly_index]) << std::dec
        << " tex=" << static_cast<unsigned>(gcmd.poly_textured[poly_index])
        << " vc=" << static_cast<unsigned>(gcmd.poly_vertex_count[poly_index])
        << " sh=" << static_cast<unsigned>(gcmd.poly_shaded[poly_index])
        << " semi=" << static_cast<unsigned>(gcmd.poly_semi[poly_index])
        << " blend=" << static_cast<unsigned>(gcmd.poly_blend[poly_index])
        << " page=0x" << std::hex << static_cast<unsigned>(gcmd.poly_texpage[poly_index])
        << " clut=0x" << static_cast<unsigned>(gcmd.poly_clut[poly_index]) << std::dec
        << " src=(" << src.x << "," << src.y << ")+(" << src.w << "," << src.h << ")"
        << " xy0=(" << gcmd.poly_x[poly_index][0] << "," << gcmd.poly_y[poly_index][0] << ")"
        << " xy1=(" << gcmd.poly_x[poly_index][1] << "," << gcmd.poly_y[poly_index][1] << ")"
        << " xy2=(" << gcmd.poly_x[poly_index][2] << "," << gcmd.poly_y[poly_index][2] << ")"
        << " xy3=(" << gcmd.poly_x[poly_index][3] << "," << gcmd.poly_y[poly_index][3] << ")"
        << " rgb0=(" << static_cast<unsigned>(gcmd.poly_r[poly_index][0]) << ","
        << static_cast<unsigned>(gcmd.poly_g[poly_index][0]) << ","
        << static_cast<unsigned>(gcmd.poly_b[poly_index][0]) << ")\n";
  }

  std::sort(indices.begin(), indices.begin() + static_cast<std::ptrdiff_t>(poly_count),
            [&](u32 a, u32 b) {
              return compute_gpu_poly_visible_area(gcmd, a, gdisp) >
                     compute_gpu_poly_visible_area(gcmd, b, gdisp);
            });
  out << "VISIBLE\n";
  for (u32 i = 0, shown = 0; i < poly_count && shown < 8; ++i) {
    const u32 poly_index = indices[static_cast<size_t>(i)];
    const u32 vis = compute_gpu_poly_visible_area(gcmd, poly_index, gdisp);
    if (vis == 0) {
      continue;
    }
    out << "  VIS[" << shown << "] area=" << vis << " op=0x" << std::hex
        << static_cast<unsigned>(gcmd.poly_opcode[poly_index]) << std::dec
        << " tex=" << static_cast<unsigned>(gcmd.poly_textured[poly_index])
        << " sh=" << static_cast<unsigned>(gcmd.poly_shaded[poly_index])
        << " xy0=(" << gcmd.poly_x[poly_index][0] << "," << gcmd.poly_y[poly_index][0] << ")"
        << " xy1=(" << gcmd.poly_x[poly_index][1] << "," << gcmd.poly_y[poly_index][1] << ")"
        << " xy2=(" << gcmd.poly_x[poly_index][2] << "," << gcmd.poly_y[poly_index][2] << ")"
        << " xy3=(" << gcmd.poly_x[poly_index][3] << "," << gcmd.poly_y[poly_index][3] << ")"
        << " rgb0=(" << static_cast<unsigned>(gcmd.poly_r[poly_index][0]) << ","
        << static_cast<unsigned>(gcmd.poly_g[poly_index][0]) << ","
        << static_cast<unsigned>(gcmd.poly_b[poly_index][0]) << ")\n";
    ++shown;
  }

  std::sort(indices.begin(), indices.begin() + static_cast<std::ptrdiff_t>(poly_count),
            [&](u32 a, u32 b) {
              return compute_gpu_poly_bbox_area(gcmd, a) >
                     compute_gpu_poly_bbox_area(gcmd, b);
            });
  out << "BIG\n";
  for (u32 i = 0; i < std::min<u32>(poly_count, 8u); ++i) {
    const u32 poly_index = indices[static_cast<size_t>(i)];
    out << "  BIG[" << i << "] area=" << compute_gpu_poly_bbox_area(gcmd, poly_index)
        << " op=0x" << std::hex << static_cast<unsigned>(gcmd.poly_opcode[poly_index])
        << std::dec << " tex=" << static_cast<unsigned>(gcmd.poly_textured[poly_index])
        << " sh=" << static_cast<unsigned>(gcmd.poly_shaded[poly_index])
        << " xy0=(" << gcmd.poly_x[poly_index][0] << "," << gcmd.poly_y[poly_index][0] << ")"
        << " xy1=(" << gcmd.poly_x[poly_index][1] << "," << gcmd.poly_y[poly_index][1] << ")"
        << " xy2=(" << gcmd.poly_x[poly_index][2] << "," << gcmd.poly_y[poly_index][2] << ")"
        << " xy3=(" << gcmd.poly_x[poly_index][3] << "," << gcmd.poly_y[poly_index][3] << ")"
        << " rgb0=(" << static_cast<unsigned>(gcmd.poly_r[poly_index][0]) << ","
        << static_cast<unsigned>(gcmd.poly_g[poly_index][0]) << ","
        << static_cast<unsigned>(gcmd.poly_b[poly_index][0]) << ")\n";
  }
  out << "\n";
}

static bool parse_log_level(const std::string &s, LogLevel &out) {
  std::string v = s;
  std::transform(v.begin(), v.end(), v.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (v == "debug") {
    out = LogLevel::Debug;
    return true;
  }
  if (v == "info") {
    out = LogLevel::Info;
    return true;
  }
  if (v == "warn" || v == "warning") {
    out = LogLevel::Warn;
    return true;
  }
  if (v == "error") {
    out = LogLevel::Error;
    return true;
  }
  return false;
}

static bool parse_cpu_execution_mode(const std::string &s,
                                     CpuExecutionMode &out) {
  std::string v = s;
  std::transform(v.begin(), v.end(), v.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  v.erase(std::remove(v.begin(), v.end(), '-'), v.end());
  v.erase(std::remove(v.begin(), v.end(), '_'), v.end());
  if (v == "interpreter" || v == "interp") {
    out = CpuExecutionMode::Interpreter;
    return true;
  }
  if (v == "decoded" || v == "decodedblock" || v == "blockinterpreter" ||
      v == "blockinterp" || v == "block") {
    out = CpuExecutionMode::DecodedBlockInterpreter;
    return true;
  }
  if (v == "x64jit" || v == "jit" || v == "dynarec" || v == "recompiler") {
    out = CpuExecutionMode::X64Jit;
    return true;
  }
  return false;
}

static u32 category_from_name(const std::string &name) {
  std::string v = name;
  std::transform(v.begin(), v.end(), v.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (v == "general")
    return log_category_bit(LogCategory::General);
  if (v == "app")
    return log_category_bit(LogCategory::App);
  if (v == "cpu")
    return log_category_bit(LogCategory::Cpu);
  if (v == "bus")
    return log_category_bit(LogCategory::Bus);
  if (v == "ram")
    return log_category_bit(LogCategory::Ram);
  if (v == "dma")
    return log_category_bit(LogCategory::Dma);
  if (v == "cdrom")
    return log_category_bit(LogCategory::Cdrom);
  if (v == "gpu")
    return log_category_bit(LogCategory::Gpu);
  if (v == "spu")
    return log_category_bit(LogCategory::Spu);
  if (v == "sio")
    return log_category_bit(LogCategory::Sio);
  if (v == "timer")
    return log_category_bit(LogCategory::Timer);
  if (v == "irq")
    return log_category_bit(LogCategory::Irq);
  if (v == "input")
    return log_category_bit(LogCategory::Input);
  if (v == "bios")
    return log_category_bit(LogCategory::Bios);
  return 0;
}

static void apply_trace_list(const std::string &value) {
  std::string v = value;
  std::transform(v.begin(), v.end(), v.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (v == "all") {
    g_trace_dma = true;
    g_trace_cdrom = true;
    g_trace_cpu = true;
    g_trace_bus = true;
    g_trace_ram = true;
    g_trace_gpu = true;
    g_trace_spu = true;
    g_trace_irq = true;
    g_trace_timer = true;
    g_trace_sio = true;
    return;
  }
  if (v == "none") {
    g_trace_dma = false;
    g_trace_cdrom = false;
    g_trace_cpu = false;
    g_trace_bus = false;
    g_trace_ram = false;
    g_trace_gpu = false;
    g_trace_spu = false;
    g_trace_irq = false;
    g_trace_timer = false;
    g_trace_sio = false;
    return;
  }

  std::istringstream iss(v);
  std::string tok;
  while (std::getline(iss, tok, ',')) {
    if (tok == "dma")
      g_trace_dma = true;
    else if (tok == "cdrom")
      g_trace_cdrom = true;
    else if (tok == "cpu")
      g_trace_cpu = true;
    else if (tok == "bus")
      g_trace_bus = true;
    else if (tok == "ram")
      g_trace_ram = true;
    else if (tok == "gpu")
      g_trace_gpu = true;
    else if (tok == "spu")
      g_trace_spu = true;
    else if (tok == "irq")
      g_trace_irq = true;
    else if (tok == "timer")
      g_trace_timer = true;
    else if (tok == "sio")
      g_trace_sio = true;
  }
}

static bool parse_u32_value(const std::string &text, u32 &out) {
  if (text.empty()) {
    return false;
  }
  char *end = nullptr;
  unsigned long parsed = std::strtoul(text.c_str(), &end, 10);
  if (end == text.c_str() || *end != '\0') {
    return false;
  }
  out = static_cast<u32>(std::max<unsigned long>(1ul, parsed));
  return true;
}

static void apply_trace_tuning_list(const std::string &value, bool is_burst) {
  std::string v = value;
  std::transform(v.begin(), v.end(), v.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  std::istringstream iss(v);
  std::string tok;
  while (std::getline(iss, tok, ',')) {
    const size_t eq = tok.find('=');
    if (eq == std::string::npos) {
      continue;
    }
    const std::string key = tok.substr(0, eq);
    const std::string val = tok.substr(eq + 1);
    u32 n = 0;
    if (!parse_u32_value(val, n)) {
      continue;
    }

    u32 *target = nullptr;
    if (is_burst) {
      if (key == "cpu")
        target = &g_trace_burst_cpu;
      else if (key == "bus")
        target = &g_trace_burst_bus;
      else if (key == "ram")
        target = &g_trace_burst_ram;
      else if (key == "dma")
        target = &g_trace_burst_dma;
      else if (key == "cdrom")
        target = &g_trace_burst_cdrom;
      else if (key == "gpu")
        target = &g_trace_burst_gpu;
      else if (key == "spu")
        target = &g_trace_burst_spu;
      else if (key == "irq")
        target = &g_trace_burst_irq;
      else if (key == "timer")
        target = &g_trace_burst_timer;
      else if (key == "sio")
        target = &g_trace_burst_sio;
    } else {
      if (key == "cpu")
        target = &g_trace_stride_cpu;
      else if (key == "bus")
        target = &g_trace_stride_bus;
      else if (key == "ram")
        target = &g_trace_stride_ram;
      else if (key == "dma")
        target = &g_trace_stride_dma;
      else if (key == "cdrom")
        target = &g_trace_stride_cdrom;
      else if (key == "gpu")
        target = &g_trace_stride_gpu;
      else if (key == "spu")
        target = &g_trace_stride_spu;
      else if (key == "irq")
        target = &g_trace_stride_irq;
      else if (key == "timer")
        target = &g_trace_stride_timer;
      else if (key == "sio")
        target = &g_trace_stride_sio;
    }

    if (target != nullptr) {
      *target = n;
    }
  }
}

static void apply_category_list(const std::string &value) {
  std::string v = value;
  std::transform(v.begin(), v.end(), v.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (v == "all") {
    g_log_category_mask = 0xFFFFFFFFu;
    return;
  }
  if (v == "none") {
    g_log_category_mask = 0u;
    return;
  }

  g_log_category_mask = 0u;
  std::istringstream iss(v);
  std::string tok;
  while (std::getline(iss, tok, ',')) {
    g_log_category_mask |= category_from_name(tok);
  }
}

enum class PcBand {
  BiosRom,
  BiosRam,
  BiosMenuWait,
  NonBios,
};

static PcBand classify_pc_band(u32 pc) {
  if (pc >= 0xBFC00000 && pc < 0xBFC80000) {
    return PcBand::BiosRom;
  }
  if ((pc >= 0x80059D00 && pc <= 0x80059DFF) ||
      (pc >= 0xA0059D00 && pc <= 0xA0059DFF)) {
    return PcBand::BiosMenuWait;
  }
  if ((pc >= 0x80000000 && pc < 0x80080000) ||
      (pc >= 0xA0000000 && pc < 0xA0080000)) {
    return PcBand::BiosRam;
  }
  return PcBand::NonBios;
}

static const char *pc_band_name(PcBand band) {
  switch (band) {
  case PcBand::BiosRom:
    return "bios_rom";
  case PcBand::BiosRam:
    return "bios_ram";
  case PcBand::BiosMenuWait:
    return "bios_menu_wait";
  case PcBand::NonBios:
  default:
    return "non_bios";
  }
}


static int run_bios_test(const std::string &bios_path, int steps) {
  const bool owns_log = (g_log_file == nullptr);
  if (owns_log) {
    g_log_file = std::fopen("bios_test.log", "w");
  }
  LOG_INFO("=== VibeStation BIOS Test ===");
  LOG_INFO("BIOS path: %s", bios_path.c_str());

  std::error_code ec;
  const bool exists = std::filesystem::exists(bios_path, ec);
  if (!exists || ec) {
    LOG_ERROR("BIOS file not found");
    if (owns_log && g_log_file) {
      log_flush_repeats();
      std::fclose(g_log_file);
      g_log_file = nullptr;
    }
    return 1;
  }

  const auto size = std::filesystem::file_size(bios_path, ec);
  if (ec) {
    LOG_ERROR("Failed to stat BIOS file");
    if (owns_log && g_log_file) {
      log_flush_repeats();
      std::fclose(g_log_file);
      g_log_file = nullptr;
    }
    return 1;
  }
  LOG_INFO("BIOS size: %llu bytes", static_cast<unsigned long long>(size));

  bool hash_ok = false;
  const u32 hash = fnv1a_hash_file(bios_path, hash_ok);
  if (hash_ok) {
    LOG_INFO("BIOS FNV-1a: 0x%08X", hash);
  }

  auto sys = std::make_unique<System>();
  if (!sys->load_bios(bios_path)) {
    LOG_ERROR("System failed to load BIOS");
    if (owns_log && g_log_file) {
      log_flush_repeats();
      std::fclose(g_log_file);
      g_log_file = nullptr;
    }
    return 1;
  }

  LOG_INFO("Detected BIOS: %s", sys->bios().get_info().c_str());
  sys->reset();

  const u32 first_instr = sys->read32(sys->cpu().pc());
  LOG_INFO("CPU reset PC: 0x%08X", sys->cpu().pc());
  LOG_INFO("First BIOS instruction: 0x%08X", first_instr);
  LOG_INFO("Stepping %d CPU instructions...", steps);

  for (int i = 0; i < steps; ++i) {
    sys->step();
    if (i < 32 || ((i + 1) % 5000 == 0)) {
      LOG_INFO("step=%d pc=0x%08X cycles=%llu", i + 1, sys->cpu().pc(),
               static_cast<unsigned long long>(sys->cpu().cycle_count()));
    }
  }

  LOG_INFO("BIOS test complete");
  if (owns_log && g_log_file) {
    log_flush_repeats();
    std::fclose(g_log_file);
    g_log_file = nullptr;
  }
  return 0;
}

static int run_frame_test(const std::string &bios_path, int frames,
                          const std::string &bin_path = "",
                          const std::string &cue_path = "",
                          const std::string &gpu_debug_path = "") {
  auto dump_vram_ppm = [](System &sys, int frame_index) {
    char path[128];
    std::snprintf(path, sizeof(path), "frame_%04d_vram.ppm", frame_index);
    std::ofstream out(path, std::ios::binary);
    if (!out.is_open()) {
      return;
    }
    out << "P6\n" << psx::VRAM_WIDTH << " " << psx::VRAM_HEIGHT << "\n255\n";
    const u16 *vram = sys.gpu().vram();
    for (int i = 0; i < static_cast<int>(psx::VRAM_WIDTH * psx::VRAM_HEIGHT);
         ++i) {
      const u16 p = vram[i];
      const u8 r = static_cast<u8>((p & 0x1F) << 3);
      const u8 g = static_cast<u8>(((p >> 5) & 0x1F) << 3);
      const u8 b = static_cast<u8>(((p >> 10) & 0x1F) << 3);
      out.put(static_cast<char>(r));
      out.put(static_cast<char>(g));
      out.put(static_cast<char>(b));
    }
    LOG_INFO("Frame test: wrote %s", path);
  };
  auto dump_vram_raw = [](System &sys, int frame_index) {
    char path[128];
    std::snprintf(path, sizeof(path), "frame_%04d_vram.bin", frame_index);
    std::ofstream out(path, std::ios::binary);
    if (!out.is_open()) {
      return;
    }
    const u16 *vram = sys.gpu().vram();
    for (int i = 0; i < static_cast<int>(psx::VRAM_WIDTH * psx::VRAM_HEIGHT);
         ++i) {
      const u16 p = vram[i];
      out.put(static_cast<char>(p & 0xFFu));
      out.put(static_cast<char>((p >> 8) & 0xFFu));
    }
    LOG_INFO("Frame test: wrote %s", path);
  };
  auto dump_display_ppm = [](System &sys, int frame_index) {
    std::vector<u32> rgba;
    const DisplaySampleInfo sample =
        sys.gpu().build_presented_display_rgba(rgba, false);
    if (sample.width <= 0 || sample.height <= 0 || rgba.empty()) {
      return;
    }

    char path[128];
    std::snprintf(path, sizeof(path), "frame_%04d_display.ppm", frame_index);
    std::ofstream out(path, std::ios::binary);
    if (!out.is_open()) {
      return;
    }
    out << "P6\n" << sample.width << " " << sample.height << "\n255\n";
    for (u32 p : rgba) {
      out.put(static_cast<char>(p & 0xFFu));
      out.put(static_cast<char>((p >> 8) & 0xFFu));
      out.put(static_cast<char>((p >> 16) & 0xFFu));
    }
    LOG_INFO("Frame test: wrote %s", path);
  };
  auto should_dump_vram = [frames](int frame_index) {
    if (frame_index == frames) {
      return true;
    }
    if (frame_index == 60) {
      return true;
    }
    return frame_index >= 300 && (frame_index % 300) == 0;
  };
  const bool owns_log = (g_log_file == nullptr);
  if (owns_log) {
    g_log_file = std::fopen("bios_test.log", "w");
  }
  LOG_INFO("=== VibeStation Frame Test ===");
  LOG_INFO("BIOS path: %s", bios_path.c_str());
  LOG_INFO("Frames: %d", frames);
  if (g_auto_input.enabled()) {
    LOG_INFO("Auto input: mask=0x%04X start=%d end=%d period=%d hold=%d",
             static_cast<unsigned>(g_auto_input.mask), g_auto_input.start_frame,
             g_auto_input.end_frame, g_auto_input.period_frames,
             g_auto_input.hold_frames);
  }
  if (!gpu_debug_path.empty()) {
    LOG_INFO("GPU debug file: %s", gpu_debug_path.c_str());
  }

  std::ofstream gpu_debug_out;
  if (!gpu_debug_path.empty()) {
    gpu_debug_out.open(gpu_debug_path, std::ios::trunc);
    if (!gpu_debug_out.is_open()) {
      LOG_ERROR("Failed to open GPU debug file");
      if (owns_log && g_log_file) {
        log_flush_repeats();
        std::fclose(g_log_file);
        g_log_file = nullptr;
      }
      return 1;
    }
  }

  auto sys = std::make_unique<System>();
  if (!sys->load_bios(bios_path)) {
    LOG_ERROR("System failed to load BIOS");
    if (owns_log && g_log_file) {
      log_flush_repeats();
      std::fclose(g_log_file);
      g_log_file = nullptr;
    }
    return 1;
  }
  if (!cue_path.empty()) {
    LOG_INFO("Disc cue: %s", cue_path.c_str());
    LOG_INFO("Disc bin: %s", bin_path.c_str());
    if (!sys->load_game(bin_path, cue_path)) {
      LOG_ERROR("Failed to load disc image");
      if (owns_log && g_log_file) {
        log_flush_repeats();
        std::fclose(g_log_file);
        g_log_file = nullptr;
      }
      return 1;
    }
  }

  if (!cue_path.empty()) {
    if (!sys->boot_disc()) {
      LOG_ERROR("Frame test failed to boot disc");
      if (owns_log && g_log_file) {
        log_flush_repeats();
        std::fclose(g_log_file);
        g_log_file = nullptr;
      }
      return 1;
    }
  } else {
    sys->reset();
  }
  bool saw_cd_getid = false;
  bool saw_cd_setloc = false;
  bool saw_cd_seekl = false;
  bool saw_cd_readn_or_reads = false;
  bool saw_cd_read_cmd = false;
  bool saw_cd_sector_visible = false;
  bool saw_tx_cmd42 = false;
  bool saw_full_pad_poll = false;
  bool saw_logo_present = false;
  bool logo_visible_persisted = false;
  bool fell_back_to_bios_after_non_bios = false;
  bool logo_candidate = false;
  bool logo_condition_a = false;
  bool logo_condition_b = false;
  bool logo_condition_c = false;
  int first_cd_getid_frame = -1;
  int first_cd_setloc_frame = -1;
  int first_cd_seekl_frame = -1;
  int first_cd_readn_or_reads_frame = -1;
  int first_cd_read_cmd_frame = -1;
  int first_cd_sector_visible_frame = -1;
  int first_tx_cmd42_frame = -1;
  int first_full_pad_poll_frame = -1;
  int first_logo_present_frame = -1;
  int first_logo_persisted_frame = -1;
  int first_black_after_logo_frame = -1;
  int first_fell_back_frame = -1;
  int first_logo_candidate_frame = -1;
  double frame_test_cpu_ms_total = 0.0;
  double frame_test_core_ms_total = 0.0;
  double frame_test_cpu_ms_min = std::numeric_limits<double>::max();
  double frame_test_cpu_ms_max = 0.0;
  double frame_test_core_ms_min = std::numeric_limits<double>::max();
  double frame_test_core_ms_max = 0.0;
  sys->cpu().notify_cpu_backend_frame(0u);

  for (int i = 0; i < frames; ++i) {
    sys->sio().set_button_state(auto_input_buttons_for_frame(i + 1));
    sys->run_frame();
    sys->cpu().notify_cpu_backend_frame(static_cast<u32>(i + 1));
    const double frame_cpu_ms = sys->profiling_stats().cpu_ms;
    const double frame_core_ms = sys->profiling_stats().total_ms;
    frame_test_cpu_ms_total += frame_cpu_ms;
    frame_test_core_ms_total += frame_core_ms;
    frame_test_cpu_ms_min = std::min(frame_test_cpu_ms_min, frame_cpu_ms);
    frame_test_cpu_ms_max = std::max(frame_test_cpu_ms_max, frame_cpu_ms);
    frame_test_core_ms_min = std::min(frame_test_core_ms_min, frame_core_ms);
    frame_test_core_ms_max = std::max(frame_test_core_ms_max, frame_core_ms);
    const System::BootDiagnostics &diag = sys->boot_diag();
    const u32 pc = sys->cpu().pc();

    if (!saw_cd_read_cmd && diag.saw_cd_read_cmd) {
      saw_cd_read_cmd = true;
      first_cd_read_cmd_frame = i + 1;
      LOG_INFO("FRAME_MARKER first_cd_read_cmd frame=%d count=%llu",
               first_cd_read_cmd_frame,
               static_cast<unsigned long long>(diag.cd_read_command_count));
    }
    if (!saw_cd_sector_visible && diag.saw_cd_sector_visible) {
      saw_cd_sector_visible = true;
      first_cd_sector_visible_frame = i + 1;
      LOG_INFO("FRAME_MARKER first_cd_sector_visible frame=%d",
               first_cd_sector_visible_frame);
    }
    if (!saw_cd_getid && diag.saw_cd_getid) {
      saw_cd_getid = true;
      first_cd_getid_frame = i + 1;
      LOG_INFO("FRAME_MARKER first_cd_getid frame=%d", first_cd_getid_frame);
    }
    if (!saw_cd_setloc && diag.saw_cd_setloc) {
      saw_cd_setloc = true;
      first_cd_setloc_frame = i + 1;
      LOG_INFO("FRAME_MARKER first_cd_setloc frame=%d", first_cd_setloc_frame);
    }
    if (!saw_cd_seekl && diag.saw_cd_seekl) {
      saw_cd_seekl = true;
      first_cd_seekl_frame = i + 1;
      LOG_INFO("FRAME_MARKER first_cd_seekl frame=%d", first_cd_seekl_frame);
    }
    if (!saw_cd_readn_or_reads && diag.saw_cd_readn_or_reads) {
      saw_cd_readn_or_reads = true;
      first_cd_readn_or_reads_frame = i + 1;
      LOG_INFO("FRAME_MARKER first_cd_readn_or_reads frame=%d",
               first_cd_readn_or_reads_frame);
    }
    if (!saw_tx_cmd42 && diag.saw_tx_cmd42) {
      saw_tx_cmd42 = true;
      first_tx_cmd42_frame = i + 1;
      LOG_INFO("FRAME_MARKER first_tx_cmd42 frame=%d count=%llu",
               first_tx_cmd42_frame,
               static_cast<unsigned long long>(diag.pad_cmd42_count));
    }
    if (!saw_full_pad_poll && diag.saw_full_pad_poll && diag.saw_tx_cmd42) {
      saw_full_pad_poll = true;
      first_full_pad_poll_frame = i + 1;
      LOG_INFO("FRAME_MARKER first_full_pad_poll frame=%d packets=%llu "
               "buttons=0x%04X",
               first_full_pad_poll_frame,
               static_cast<unsigned long long>(diag.pad_packet_count),
               static_cast<unsigned>(diag.last_pad_buttons));
    }
    if (!saw_logo_present && diag.saw_logo_present) {
      saw_logo_present = true;
      first_logo_present_frame = i + 1;
      LOG_INFO("FRAME_MARKER first_logo_present frame=%d hash=0x%08X non_black=%llu",
               first_logo_present_frame, diag.display_hash,
               static_cast<unsigned long long>(diag.display_non_black_pixels));
    }
    if (!logo_visible_persisted && diag.logo_visible_persisted) {
      logo_visible_persisted = true;
      first_logo_persisted_frame = i + 1;
      LOG_INFO("FRAME_MARKER logo_visible_persisted frame=%d run=%u",
               first_logo_persisted_frame,
               static_cast<unsigned>(diag.logo_visible_run_frames));
    }
    if (first_black_after_logo_frame < 0 &&
        diag.first_black_after_logo_frame >= 0) {
      first_black_after_logo_frame = diag.first_black_after_logo_frame;
      LOG_INFO("FRAME_MARKER first_black_after_logo frame=%d",
               first_black_after_logo_frame);
    }
    if (!fell_back_to_bios_after_non_bios && diag.fell_back_to_bios_after_non_bios) {
      fell_back_to_bios_after_non_bios = true;
      first_fell_back_frame = i + 1;
      LOG_INFO("FRAME_MARKER fell_back_to_bios frame=%d pc=0x%08X",
               first_fell_back_frame, pc);
    }

    logo_condition_a = diag.saw_cd_read_cmd && diag.saw_cd_sector_visible;
    logo_condition_b = !diag.fell_back_to_bios_after_non_bios;
    logo_condition_c = diag.logo_visible_persisted;
    const bool logo_now = logo_condition_a && logo_condition_b && logo_condition_c;
    if (!logo_candidate && logo_now) {
      logo_candidate = true;
      first_logo_candidate_frame = i + 1;
      LOG_INFO("FRAME_MARKER logo_candidate frame=%d", first_logo_candidate_frame);
    }

    if (should_dump_vram(i + 1)) {
      dump_vram_ppm(*sys, i + 1);
      dump_vram_raw(*sys, i + 1);
      dump_display_ppm(*sys, i + 1);
    }
    if (gpu_debug_out.is_open()) {
      dump_gpu_debug_frame(gpu_debug_out, i + 1, *sys);
    }
    if ((i + 1) <= 10 || ((i + 1) % 30 == 0)) {
      const u32 instr = sys->read32(pc);
      LOG_INFO(
          "frame=%d pc=0x%08X instr=0x%08X cycles=%llu i_stat=0x%08X i_mask=0x%08X",
          i + 1, pc, instr,
          static_cast<unsigned long long>(sys->cpu().cycle_count()),
          sys->irq().stat(), sys->irq().mask());
    }
    if (g_frame_state_log_frames != 0u &&
        (static_cast<u32>(i + 1) % g_frame_state_log_frames) == 0u) {
      sys->debug_log_frame_state();
    }
  }
  const System::BootDiagnostics &diag = sys->boot_diag();
  const double cpu_core_ms_avg =
      frame_test_cpu_ms_total / static_cast<double>(frames);
  const double core_ms_avg =
      frame_test_core_ms_total / static_cast<double>(frames);
  const double target_fps = sys->target_fps();
  const double frame_budget_ms = target_fps > 0.0 ? 1000.0 / target_fps : 0.0;
  const double slowdown_percent =
      frame_budget_ms > 0.0 && core_ms_avg > frame_budget_ms
          ? std::clamp((1.0 - frame_budget_ms / core_ms_avg) * 100.0,
                       0.0, 100.0)
          : 0.0;
  LOG_INFO("FRAME_PERF cpu_core_ms=%.3f core_ms=%.3f cpu_core_min_ms=%.3f cpu_core_max_ms=%.3f core_min_ms=%.3f core_max_ms=%.3f slowdown_percent=%.2f target_fps=%.3f detailed=%u",
           cpu_core_ms_avg, core_ms_avg, frame_test_cpu_ms_min,
           frame_test_cpu_ms_max, frame_test_core_ms_min,
           frame_test_core_ms_max, slowdown_percent, target_fps,
           g_profile_detailed_timing ? 1u : 0u);
  LOG_INFO(
      "FRAME_SUMMARY saw_cd_read_cmd=%d saw_cd_sector_visible=%d "
      "saw_tx_cmd42=%d saw_full_pad_poll=%d saw_cd_getid=%d "
      "saw_cd_setloc=%d saw_cd_seekl=%d saw_cd_readn_or_reads=%d "
      "saw_logo_present=%d logo_visible_persisted=%d fell_back_to_bios=%d "
      "logo_candidate=%d logo_a=%d logo_b=%d logo_c=%d "
      "first_cd_read_cmd_frame=%d first_cd_sector_visible_frame=%d "
      "first_tx_cmd42_frame=%d first_full_pad_poll_frame=%d first_cd_getid_frame=%d "
      "first_cd_setloc_frame=%d first_cd_seekl_frame=%d "
      "first_cd_readn_or_reads_frame=%d first_logo_present_frame=%d first_logo_persisted_frame=%d "
      "first_black_after_logo_frame=%d "
      "first_fell_back_frame=%d first_logo_candidate_frame=%d "
      "pad_packets=%llu padpoll_ch0=%llu padpoll_ch1=%llu sio_invalid_seq=%llu "
      "last_pad_buttons=0x%04X last_sio_tx=0x%02X last_sio_rx=0x%02X "
      "joy_stat=0x%04X joy_ctrl=0x%04X "
      "sio_irq_assert=%llu sio_irq_ack=%llu "
      "cdread_count=%llu cd_sector_count=%llu cd_lba=%d cd_irq1=%llu cd_irq3=%llu "
      "cd_resp_promotions=%llu cd_read_stalls=%llu cd_status_e0=%llu cd_status_e0_streak=%llu "
      "display_hash=0x%08X display_non_black=%llu display_wh=%ux%u "
      "display_start=%u,%u display_enabled=%u display_24=%u final_pc=0x%08X",
      saw_cd_read_cmd ? 1 : 0, saw_cd_sector_visible ? 1 : 0,
      saw_tx_cmd42 ? 1 : 0, saw_full_pad_poll ? 1 : 0, saw_cd_getid ? 1 : 0,
      saw_cd_setloc ? 1 : 0, saw_cd_seekl ? 1 : 0,
      saw_cd_readn_or_reads ? 1 : 0, saw_logo_present ? 1 : 0,
      logo_visible_persisted ? 1 : 0, fell_back_to_bios_after_non_bios ? 1 : 0,
      logo_candidate ? 1 : 0,
      logo_condition_a ? 1 : 0, logo_condition_b ? 1 : 0,
      logo_condition_c ? 1 : 0, first_cd_read_cmd_frame,
      first_cd_sector_visible_frame, first_tx_cmd42_frame,
      first_full_pad_poll_frame,
      first_cd_getid_frame, first_cd_setloc_frame, first_cd_seekl_frame,
      first_cd_readn_or_reads_frame, first_logo_present_frame,
      first_logo_persisted_frame, first_black_after_logo_frame,
      first_fell_back_frame, first_logo_candidate_frame,
      static_cast<unsigned long long>(diag.pad_packet_count),
      static_cast<unsigned long long>(diag.ch0_poll_count),
      static_cast<unsigned long long>(diag.ch1_poll_count),
      static_cast<unsigned long long>(diag.sio_invalid_seq_count),
      static_cast<unsigned>(diag.last_pad_buttons),
      static_cast<unsigned>(diag.last_sio_tx),
      static_cast<unsigned>(diag.last_sio_rx),
      static_cast<unsigned>(diag.last_joy_stat),
      static_cast<unsigned>(diag.last_joy_ctrl),
      static_cast<unsigned long long>(diag.sio_irq_assert_count),
      static_cast<unsigned long long>(diag.sio_irq_ack_count),
      static_cast<unsigned long long>(diag.cd_read_command_count),
      static_cast<unsigned long long>(sys->cdrom().sector_count()),
      sys->cdrom().current_read_lba(),
      static_cast<unsigned long long>(diag.cd_irq_int1_count),
      static_cast<unsigned long long>(diag.cd_irq_int3_count),
      static_cast<unsigned long long>(sys->cdrom().response_promotion_count()),
      static_cast<unsigned long long>(sys->cdrom().read_buffer_stall_count()),
      static_cast<unsigned long long>(sys->cdrom().status_e0_poll_count()),
      static_cast<unsigned long long>(sys->cdrom().status_e0_streak_max()),
      diag.display_hash,
      static_cast<unsigned long long>(diag.display_non_black_pixels),
      static_cast<unsigned>(diag.display_width),
      static_cast<unsigned>(diag.display_height),
      static_cast<unsigned>(diag.display_x_start),
      static_cast<unsigned>(diag.display_y_start),
      static_cast<unsigned>(diag.display_enabled),
      static_cast<unsigned>(diag.display_is_24bit), sys->cpu().pc());
  log_mdec_summary("FRAME", *sys);
  LOG_INFO("Frame test complete");
  if (gpu_debug_out.is_open()) {
    gpu_debug_out.flush();
  }
  if (owns_log && g_log_file) {
    log_flush_repeats();
    std::fclose(g_log_file);
    g_log_file = nullptr;
  }
  return 0;
}

static int run_spu_audio_test(const std::string &bios_path, int frames,
                              const std::string &bin_path,
                              const std::string &cue_path,
                              const std::string &wav_path) {
  const bool owns_log = (g_log_file == nullptr);
  if (owns_log) {
    g_log_file = std::fopen("spu_audio_test.log", "w");
  }

  LOG_INFO("=== VibeStation SPU Audio Test ===");
  LOG_INFO("BIOS path: %s", bios_path.c_str());
  LOG_INFO("Frames: %d", frames);
  if (!cue_path.empty()) {
    LOG_INFO("Disc BIN: %s", bin_path.c_str());
    LOG_INFO("Disc CUE: %s", cue_path.c_str());
  }
  if (!wav_path.empty()) {
    LOG_INFO("WAV out: %s", wav_path.c_str());
  }

  auto sys = std::make_unique<System>();
  if (!sys->load_bios(bios_path)) {
    LOG_ERROR("SPU_TEST_FAIL reason=bios_load");
    if (owns_log && g_log_file) {
      log_flush_repeats();
      std::fclose(g_log_file);
      g_log_file = nullptr;
    }
    return 1;
  }

  if (!cue_path.empty()) {
    if (!sys->load_game(bin_path, cue_path)) {
      LOG_ERROR("SPU_TEST_FAIL reason=disc_load");
      if (owns_log && g_log_file) {
        log_flush_repeats();
        std::fclose(g_log_file);
        g_log_file = nullptr;
      }
      return 1;
    }
    if (!sys->boot_disc()) {
      LOG_ERROR("SPU_TEST_FAIL reason=boot_disc");
      if (owns_log && g_log_file) {
        log_flush_repeats();
        std::fclose(g_log_file);
        g_log_file = nullptr;
      }
      return 1;
    }
  } else {
    sys->reset();
  }

  sys->clear_spu_audio_capture();
  sys->set_spu_audio_capture(true);
  for (int i = 0; i < frames; ++i) {
    sys->run_frame();
  }
  sys->set_spu_audio_capture(false);

  const auto &samples = sys->spu_audio_capture_samples();
  const auto &diag = sys->spu_audio_diag();
  const u32 hash = fnv1a_hash_samples(samples);
  bool wrote_wav = false;
  if (!wav_path.empty()) {
    wrote_wav = write_wav_s16_stereo(wav_path, samples, 44100);
    if (!wrote_wav) {
      LOG_ERROR("SPU_TEST_FAIL reason=wav_write");
    }
  }

  LOG_INFO(
      "SPU_SUMMARY gaussian_active=%d reverb_enabled=%d "
      "reverb_config_seen=%d generated_frames=%llu queued_frames=%llu "
      "dropped_frames=%llu overrun_events=%llu capture_frames=%llu "
      "reverb_mix_frames=%llu reverb_ram_writes=%llu key_on=%llu key_off=%llu "
      "end_flag=%llu loop_end=%llu nonloop_end=%llu "
      "off_end_flag=%llu off_release_env0=%llu release_to_off=%llu "
      "kon_retrigger=%llu koff_high_env=%llu "
      "kon_to_retrigger_events=%llu kon_to_retrigger_min=%u kon_to_retrigger_max=%u kon_to_retrigger_avg=%llu "
      "kon_to_koff_events=%llu kon_to_koff_min=%u kon_to_koff_max=%u kon_to_koff_avg=%llu "
      "kon_to_koff_min_voice=%u kon_to_koff_min_addr=0x%05X kon_to_koff_min_pitch=0x%04X kon_to_koff_min_adsr2=0x%04X "
      "koff_short_window=%llu "
      "key_write_unsynced=%llu key_write_unsynced_max_lag=%llu key_write_spu_off=%llu "
      "keyon_ignored_off=%llu keyoff_ignored_off=%llu "
      "spucnt_en_set=%llu spucnt_en_clear=%llu "
      "spu_dis_forced_off=%llu spu_dis_span_events=%llu spu_dis_span_min=%u spu_dis_span_max=%u spu_dis_span_avg=%llu "
      "v16_kon_w=%llu v16_koff_w=%llu "
      "v16_kw2kw_events=%llu v16_kw2kw_min_cycles=%llu v16_kw2kw_max_cycles=%llu v16_kw2kw_avg_cycles=%llu "
      "v16_kw2kw_min_samples=%u v16_kw2kw_max_samples=%u v16_kw2kw_avg_samples=%llu "
      "v16_kon_a=%llu v16_koff_a=%llu "
      "v16_ka2ko_events=%llu v16_ka2ko_min=%u v16_ka2ko_max=%u v16_ka2ko_avg=%llu "
      "v16_kon_reapply=%llu v16_koff_wo_kon=%llu "
      "v16_strobe_kon=%llu v16_strobe_koff=%llu v16_strobe_both=%llu "
      "active_voice_peak=%u active_voice_avg_x100=%llu "
      "voice_cap_frames=%llu no_voice_frames=%llu muted_output_frames=%llu cd_frames_mixed=%llu "
      "spucnt_writes=%llu spucnt_mute_toggles=%llu spucnt_mute_set=%llu spucnt_mute_clear=%llu "
      "spucnt_last=0x%04X queue_peak_bytes=%u "
      "release_total=%llu release_min=%u release_max=%u release_fast=%llu "
      "queue_last_bytes=%u clip_dry=%llu clip_wet=%llu clip_out=%llu "
      "reverb_guard=%llu peak_dry_l=%.4f peak_dry_r=%.4f "
      "peak_wet_l=%.4f peak_wet_r=%.4f peak_mix_l=%.4f peak_mix_r=%.4f "
      "wav_written=%d wav_hash=0x%08X wav_samples=%zu",
      diag.gaussian_active ? 1 : 0, diag.reverb_enabled ? 1 : 0,
      diag.saw_reverb_config_write ? 1 : 0,
      static_cast<unsigned long long>(diag.generated_frames),
      static_cast<unsigned long long>(diag.queued_frames),
      static_cast<unsigned long long>(diag.dropped_frames),
      static_cast<unsigned long long>(diag.overrun_events),
      static_cast<unsigned long long>(diag.capture_frames),
      static_cast<unsigned long long>(diag.reverb_mix_frames),
      static_cast<unsigned long long>(diag.reverb_ram_writes),
      static_cast<unsigned long long>(diag.key_on_events),
      static_cast<unsigned long long>(diag.key_off_events),
      static_cast<unsigned long long>(diag.end_flag_events),
      static_cast<unsigned long long>(diag.loop_end_events),
      static_cast<unsigned long long>(diag.nonloop_end_events),
      static_cast<unsigned long long>(diag.off_due_to_end_flag),
      static_cast<unsigned long long>(diag.off_due_to_release_env0),
      static_cast<unsigned long long>(diag.release_to_off_events),
      static_cast<unsigned long long>(diag.kon_retrigger_events),
      static_cast<unsigned long long>(diag.koff_high_env_events),
      static_cast<unsigned long long>(diag.kon_to_retrigger_events),
      diag.kon_to_retrigger_min_samples, diag.kon_to_retrigger_max_samples,
      static_cast<unsigned long long>(
          (diag.kon_to_retrigger_events != 0)
              ? (diag.kon_to_retrigger_total_samples /
                 diag.kon_to_retrigger_events)
              : 0ull),
      static_cast<unsigned long long>(diag.kon_to_koff_events),
      diag.kon_to_koff_min_samples, diag.kon_to_koff_max_samples,
      static_cast<unsigned long long>(
          (diag.kon_to_koff_events != 0)
              ? (diag.kon_to_koff_total_samples / diag.kon_to_koff_events)
              : 0ull),
      static_cast<unsigned>(diag.kon_to_koff_min_voice),
      static_cast<unsigned>(diag.kon_to_koff_min_addr),
      static_cast<unsigned>(diag.kon_to_koff_min_pitch),
      static_cast<unsigned>(diag.kon_to_koff_min_adsr2),
      static_cast<unsigned long long>(diag.koff_short_window_events),
      static_cast<unsigned long long>(diag.key_write_unsynced_events),
      static_cast<unsigned long long>(diag.key_write_unsynced_max_cpu_lag),
      static_cast<unsigned long long>(diag.key_write_while_spu_disabled),
      static_cast<unsigned long long>(diag.keyon_ignored_while_disabled),
      static_cast<unsigned long long>(diag.keyoff_ignored_while_disabled),
      static_cast<unsigned long long>(diag.spucnt_enable_set_events),
      static_cast<unsigned long long>(diag.spucnt_enable_clear_events),
      static_cast<unsigned long long>(diag.spu_disable_forced_off_voices),
      static_cast<unsigned long long>(diag.spu_disable_span_events),
      diag.spu_disable_span_min_samples,
      diag.spu_disable_span_max_samples,
      static_cast<unsigned long long>(
          (diag.spu_disable_span_events != 0)
              ? (diag.spu_disable_span_total_samples /
                 diag.spu_disable_span_events)
              : 0ull),
      static_cast<unsigned long long>(diag.v16_kon_write_events),
      static_cast<unsigned long long>(diag.v16_koff_write_events),
      static_cast<unsigned long long>(diag.v16_kon_write_to_koff_events),
      static_cast<unsigned long long>(diag.v16_kon_write_to_koff_min_cpu_cycles),
      static_cast<unsigned long long>(diag.v16_kon_write_to_koff_max_cpu_cycles),
      static_cast<unsigned long long>(
          (diag.v16_kon_write_to_koff_events != 0)
              ? (diag.v16_kon_write_to_koff_total_cpu_cycles /
                 diag.v16_kon_write_to_koff_events)
              : 0ull),
      diag.v16_kon_write_to_koff_min_samples,
      diag.v16_kon_write_to_koff_max_samples,
      static_cast<unsigned long long>(
          (diag.v16_kon_write_to_koff_events != 0)
              ? (diag.v16_kon_write_to_koff_total_samples /
                 diag.v16_kon_write_to_koff_events)
              : 0ull),
      static_cast<unsigned long long>(diag.v16_kon_apply_events),
      static_cast<unsigned long long>(diag.v16_koff_apply_events),
      static_cast<unsigned long long>(diag.v16_kon_to_koff_apply_events),
      diag.v16_kon_to_koff_apply_min_samples,
      diag.v16_kon_to_koff_apply_max_samples,
      static_cast<unsigned long long>(
          (diag.v16_kon_to_koff_apply_events != 0)
              ? (diag.v16_kon_to_koff_apply_total_samples /
                 diag.v16_kon_to_koff_apply_events)
              : 0ull),
      static_cast<unsigned long long>(diag.v16_kon_reapply_without_koff),
      static_cast<unsigned long long>(diag.v16_koff_without_kon),
      static_cast<unsigned long long>(diag.v16_strobe_samples_with_kon),
      static_cast<unsigned long long>(diag.v16_strobe_samples_with_koff),
      static_cast<unsigned long long>(diag.v16_strobe_samples_with_both),
      static_cast<unsigned>(diag.active_voice_peak),
      static_cast<unsigned long long>(
          (diag.active_voice_samples != 0)
              ? ((diag.active_voice_accum * 100ull) / diag.active_voice_samples)
              : 0ull),
      static_cast<unsigned long long>(diag.voice_cap_frames),
      static_cast<unsigned long long>(diag.no_voice_frames),
      static_cast<unsigned long long>(diag.muted_output_frames),
      static_cast<unsigned long long>(diag.cd_frames_mixed),
      static_cast<unsigned long long>(diag.spucnt_writes),
      static_cast<unsigned long long>(diag.spucnt_mute_toggle_events),
      static_cast<unsigned long long>(diag.spucnt_mute_set_events),
      static_cast<unsigned long long>(diag.spucnt_mute_clear_events),
      static_cast<unsigned>(diag.spucnt_last),
      diag.queue_peak_bytes,
      static_cast<unsigned long long>(diag.release_samples_total),
      diag.release_samples_min, diag.release_samples_max,
      static_cast<unsigned long long>(diag.release_fast_events),
      diag.queue_last_bytes,
      static_cast<unsigned long long>(diag.clip_events_dry),
      static_cast<unsigned long long>(diag.clip_events_wet),
      static_cast<unsigned long long>(diag.clip_events_out),
      static_cast<unsigned long long>(diag.reverb_guard_events),
      static_cast<double>(diag.peak_dry_l), static_cast<double>(diag.peak_dry_r),
      static_cast<double>(diag.peak_wet_l), static_cast<double>(diag.peak_wet_r),
      static_cast<double>(diag.peak_mix_l), static_cast<double>(diag.peak_mix_r),
      wrote_wav ? 1 : 0, hash, samples.size());

  const bool pass = diag.gaussian_active && diag.saw_reverb_config_write &&
                    !samples.empty() &&
                    (diag.generated_frames >= static_cast<u64>(frames * 700));
  if (!pass) {
    LOG_ERROR("SPU_TEST_FAIL reason=summary_gate");
  } else {
    LOG_INFO("SPU_TEST_PASS");
  }

  if (owns_log && g_log_file) {
    log_flush_repeats();
    std::fclose(g_log_file);
    g_log_file = nullptr;
  }
  return pass ? 0 : 2;
}

static int run_boot_disc_test(const std::string &bios_path, int frames,
                              const std::string &bin_path,
                              const std::string &cue_path) {
  const bool owns_log = (g_log_file == nullptr);
  if (owns_log) {
    g_log_file = std::fopen("boot_disc_test.log", "w");
  }

  LOG_INFO("=== VibeStation Boot Disc Test ===");
  LOG_INFO("BIOS path: %s", bios_path.c_str());
  LOG_INFO("Frames: %d", frames);
  LOG_INFO("Disc BIN: %s", bin_path.c_str());
  LOG_INFO("Disc CUE: %s", cue_path.c_str());
  if (g_auto_input.enabled()) {
    LOG_INFO("Auto input: mask=0x%04X start=%d end=%d period=%d hold=%d",
             static_cast<unsigned>(g_auto_input.mask), g_auto_input.start_frame,
             g_auto_input.end_frame, g_auto_input.period_frames,
             g_auto_input.hold_frames);
  }

  auto sys = std::make_unique<System>();
  if (!sys->load_bios(bios_path)) {
    LOG_ERROR("BOOT_TEST_FAIL reason=bios_load");
    if (owns_log && g_log_file) {
      log_flush_repeats();
      std::fclose(g_log_file);
      g_log_file = nullptr;
    }
    return 1;
  }
  if (!sys->load_game(bin_path, cue_path)) {
    LOG_ERROR("BOOT_TEST_FAIL reason=disc_load");
    if (owns_log && g_log_file) {
      log_flush_repeats();
      std::fclose(g_log_file);
      g_log_file = nullptr;
    }
    return 1;
  }
  if (!sys->boot_disc()) {
    LOG_ERROR("BOOT_TEST_FAIL reason=boot_disc");
    if (owns_log && g_log_file) {
      log_flush_repeats();
      std::fclose(g_log_file);
      g_log_file = nullptr;
    }
    return 1;
  }

  bool saw_cd_command = false;
  bool saw_cd_sector = false;
  bool saw_non_bios_pc = false;
  bool saw_cd_io = false;
  bool saw_sio_io = false;
  bool saw_pad_cmd42 = false;
  bool saw_tx_cmd42 = false;
  bool saw_pad_id = false;
  bool saw_pad_button = false;
  bool saw_full_pad_poll = false;
  bool saw_cd_read_cmd = false;
  bool saw_cd_sector_visible = false;
  bool saw_cd_getid = false;
  bool saw_cd_setloc = false;
  bool saw_cd_seekl = false;
  bool saw_cd_readn_or_reads = false;
  bool saw_logo_present = false;
  bool logo_visible_persisted = false;
  bool fell_back_to_bios_after_non_bios = false;
  bool logo_candidate = false;
  bool logo_condition_a = false;
  bool logo_condition_b = false;
  bool logo_condition_c = false;
  int first_cmd_frame = -1;
  int first_sector_frame = -1;
  int first_non_bios_frame = -1;
  int first_cd_io_frame = -1;
  int first_sio_io_frame = -1;
  int first_pad_cmd42_frame = -1;
  int first_tx_cmd42_frame = -1;
  int first_pad_id_frame = -1;
  int first_pad_button_frame = -1;
  int first_full_pad_poll_frame = -1;
  int first_cd_read_cmd_frame = -1;
  int first_cd_sector_visible_frame = -1;
  int first_cd_getid_frame = -1;
  int first_cd_setloc_frame = -1;
  int first_cd_seekl_frame = -1;
  int first_cd_readn_or_reads_frame = -1;
  int first_logo_present_frame = -1;
  int first_logo_persisted_frame = -1;
  int first_black_after_logo_frame = -1;
  int first_fell_back_frame = -1;
  int first_logo_candidate_frame = -1;
  u32 first_non_bios_pc = 0;
  u32 first_cd_io_addr = 0;
  u32 first_sio_io_addr = 0;
  u64 first_cd_io_cycle = 0;
  u64 first_sio_io_cycle = 0;
  PcBand last_band = classify_pc_band(sys->cpu().pc());
  LOG_INFO("BOOT_STAGE frame=0 pc=0x%08X band=%s", sys->cpu().pc(),
           pc_band_name(last_band));

  for (int i = 0; i < frames; ++i) {
    sys->sio().set_button_state(auto_input_buttons_for_frame(i + 1));
    sys->run_frame();

    const u32 pc = sys->cpu().pc();
    const PcBand band = classify_pc_band(pc);
    const u64 cmd_count = sys->cdrom().command_count();
    const u64 sector_count = sys->cdrom().sector_count();
    const System::BootDiagnostics &diag = sys->boot_diag();

    if (band != last_band) {
      LOG_INFO("BOOT_STAGE frame=%d pc=0x%08X band=%s", i + 1, pc,
               pc_band_name(band));
      last_band = band;
    }

    if (!saw_cd_command && cmd_count > 0) {
      saw_cd_command = true;
      first_cmd_frame = i + 1;
      LOG_INFO("BOOT_MARKER first_cd_command frame=%d total=%llu", first_cmd_frame,
               static_cast<unsigned long long>(cmd_count));
    }

    if (!saw_cd_sector && sector_count > 0) {
      saw_cd_sector = true;
      first_sector_frame = i + 1;
      LOG_INFO("BOOT_MARKER first_cd_sector frame=%d total=%llu", first_sector_frame,
               static_cast<unsigned long long>(sector_count));
    }

    if (!saw_cd_io && diag.saw_cd_io) {
      saw_cd_io = true;
      first_cd_io_frame = i + 1;
      first_cd_io_addr = diag.first_cd_io_addr;
      first_cd_io_cycle = diag.first_cd_io_cycle;
      LOG_INFO("BOOT_MARKER first_cd_io frame=%d cycle=%llu addr=0x%08X",
               first_cd_io_frame,
               static_cast<unsigned long long>(first_cd_io_cycle),
               first_cd_io_addr);
    }

    if (!saw_sio_io && diag.saw_sio_io) {
      saw_sio_io = true;
      first_sio_io_frame = i + 1;
      first_sio_io_addr = diag.first_sio_io_addr;
      first_sio_io_cycle = diag.first_sio_io_cycle;
      LOG_INFO("BOOT_MARKER first_sio_io frame=%d cycle=%llu addr=0x%08X",
               first_sio_io_frame,
               static_cast<unsigned long long>(first_sio_io_cycle),
               first_sio_io_addr);
    }

    if (!saw_pad_cmd42 && diag.saw_pad_cmd42) {
      saw_pad_cmd42 = true;
      first_pad_cmd42_frame = i + 1;
      LOG_INFO("BOOT_MARKER first_pad_cmd42 frame=%d count=%llu",
               first_pad_cmd42_frame,
               static_cast<unsigned long long>(diag.pad_cmd42_count));
    }
    if (!saw_tx_cmd42 && diag.saw_tx_cmd42) {
      saw_tx_cmd42 = true;
      first_tx_cmd42_frame = i + 1;
      LOG_INFO("BOOT_MARKER first_tx_cmd42 frame=%d count=%llu",
               first_tx_cmd42_frame,
               static_cast<unsigned long long>(diag.pad_cmd42_count));
    }

    if (!saw_pad_id && diag.saw_pad_id) {
      saw_pad_id = true;
      first_pad_id_frame = i + 1;
      LOG_INFO("BOOT_MARKER first_pad_id frame=%d", first_pad_id_frame);
    }

    if (!saw_pad_button && diag.saw_pad_button) {
      saw_pad_button = true;
      first_pad_button_frame = i + 1;
      LOG_INFO("BOOT_MARKER first_pad_button frame=%d", first_pad_button_frame);
    }
    if (!saw_full_pad_poll && diag.saw_full_pad_poll && diag.saw_tx_cmd42) {
      saw_full_pad_poll = true;
      first_full_pad_poll_frame = i + 1;
      LOG_INFO("BOOT_MARKER first_full_pad_poll frame=%d packets=%llu "
               "buttons=0x%04X",
               first_full_pad_poll_frame,
               static_cast<unsigned long long>(diag.pad_packet_count),
               static_cast<unsigned>(diag.last_pad_buttons));
    }

    if (!saw_cd_read_cmd && diag.saw_cd_read_cmd) {
      saw_cd_read_cmd = true;
      first_cd_read_cmd_frame = i + 1;
      LOG_INFO("BOOT_MARKER first_cd_read_cmd frame=%d count=%llu",
               first_cd_read_cmd_frame,
               static_cast<unsigned long long>(diag.cd_read_command_count));
    }

    if (!saw_cd_sector_visible && diag.saw_cd_sector_visible) {
      saw_cd_sector_visible = true;
      first_cd_sector_visible_frame = i + 1;
      LOG_INFO("BOOT_MARKER first_cd_sector_visible frame=%d",
               first_cd_sector_visible_frame);
    }

    if (!saw_cd_getid && diag.saw_cd_getid) {
      saw_cd_getid = true;
      first_cd_getid_frame = i + 1;
      LOG_INFO("BOOT_MARKER first_cd_getid frame=%d", first_cd_getid_frame);
    }

    if (!saw_cd_setloc && diag.saw_cd_setloc) {
      saw_cd_setloc = true;
      first_cd_setloc_frame = i + 1;
      LOG_INFO("BOOT_MARKER first_cd_setloc frame=%d", first_cd_setloc_frame);
    }

    if (!saw_cd_seekl && diag.saw_cd_seekl) {
      saw_cd_seekl = true;
      first_cd_seekl_frame = i + 1;
      LOG_INFO("BOOT_MARKER first_cd_seekl frame=%d", first_cd_seekl_frame);
    }

    if (!saw_cd_readn_or_reads && diag.saw_cd_readn_or_reads) {
      saw_cd_readn_or_reads = true;
      first_cd_readn_or_reads_frame = i + 1;
      LOG_INFO("BOOT_MARKER first_cd_readn_or_reads frame=%d",
               first_cd_readn_or_reads_frame);
    }
    if (!saw_logo_present && diag.saw_logo_present) {
      saw_logo_present = true;
      first_logo_present_frame = i + 1;
      LOG_INFO("BOOT_MARKER first_logo_present frame=%d hash=0x%08X non_black=%llu",
               first_logo_present_frame, diag.display_hash,
               static_cast<unsigned long long>(diag.display_non_black_pixels));
    }
    if (!logo_visible_persisted && diag.logo_visible_persisted) {
      logo_visible_persisted = true;
      first_logo_persisted_frame = i + 1;
      LOG_INFO("BOOT_MARKER logo_visible_persisted frame=%d run=%u",
               first_logo_persisted_frame,
               static_cast<unsigned>(diag.logo_visible_run_frames));
    }
    if (first_black_after_logo_frame < 0 &&
        diag.first_black_after_logo_frame >= 0) {
      first_black_after_logo_frame = diag.first_black_after_logo_frame;
      LOG_INFO("BOOT_MARKER first_black_after_logo frame=%d",
               first_black_after_logo_frame);
    }
    if (!fell_back_to_bios_after_non_bios && diag.fell_back_to_bios_after_non_bios) {
      fell_back_to_bios_after_non_bios = true;
      first_fell_back_frame = i + 1;
      LOG_INFO("BOOT_MARKER fell_back_to_bios frame=%d pc=0x%08X",
               first_fell_back_frame, pc);
    }

    if (!saw_non_bios_pc && band == PcBand::NonBios) {
      saw_non_bios_pc = true;
      first_non_bios_frame = i + 1;
      first_non_bios_pc = pc;
      LOG_INFO("BOOT_MARKER first_non_bios_pc frame=%d pc=0x%08X",
               first_non_bios_frame, first_non_bios_pc);
    }

    logo_condition_a = diag.saw_cd_read_cmd && diag.saw_cd_sector_visible;
    logo_condition_b = !diag.fell_back_to_bios_after_non_bios;
    logo_condition_c = diag.logo_visible_persisted;
    const bool logo_now = logo_condition_a && logo_condition_b && logo_condition_c;
    if (!logo_candidate && logo_now) {
      logo_candidate = true;
      first_logo_candidate_frame = i + 1;
      LOG_INFO("BOOT_MARKER logo_candidate frame=%d", first_logo_candidate_frame);
    }

    if ((i + 1) <= 10 || ((i + 1) % 60 == 0)) {
      LOG_INFO("BOOT_FRAME frame=%d pc=0x%08X cmd=%llu sec=%llu cdio=%llu "
               "sioio=%llu pad42=%d tx42=%d padid=%d fullpad=%d padbtn=0x%04X "
               "joy_stat=0x%04X joy_ctrl=0x%04X cdrd=%d cdvis=%d getid=%d "
               "setloc=%d seekl=%d readcmd=%d logo=%d logo_present=%d "
               "fell_back=%d disp_hash=0x%08X disp_non_black=%llu "
               "disp_wh=%ux%u disp_xy=%u,%u disp_en=%u disp_24=%u "
               "i_stat=0x%08X i_mask=0x%08X",
               i + 1, pc, static_cast<unsigned long long>(cmd_count),
               static_cast<unsigned long long>(sector_count),
               static_cast<unsigned long long>(diag.cd_io_count),
               static_cast<unsigned long long>(diag.sio_io_count),
               diag.saw_pad_cmd42 ? 1 : 0, diag.saw_tx_cmd42 ? 1 : 0,
               diag.saw_pad_id ? 1 : 0,
               diag.saw_full_pad_poll ? 1 : 0,
               static_cast<unsigned>(diag.last_pad_buttons),
               static_cast<unsigned>(diag.last_joy_stat),
               static_cast<unsigned>(diag.last_joy_ctrl),
               diag.saw_cd_read_cmd ? 1 : 0,
               diag.saw_cd_sector_visible ? 1 : 0,
               diag.saw_cd_getid ? 1 : 0, diag.saw_cd_setloc ? 1 : 0,
               diag.saw_cd_seekl ? 1 : 0, diag.saw_cd_readn_or_reads ? 1 : 0,
               logo_candidate ? 1 : 0, diag.saw_logo_present ? 1 : 0,
               diag.fell_back_to_bios_after_non_bios ? 1 : 0, diag.display_hash,
               static_cast<unsigned long long>(diag.display_non_black_pixels),
               static_cast<unsigned>(diag.display_width),
               static_cast<unsigned>(diag.display_height),
               static_cast<unsigned>(diag.display_x_start),
               static_cast<unsigned>(diag.display_y_start),
               static_cast<unsigned>(diag.display_enabled),
               static_cast<unsigned>(diag.display_is_24bit),
               sys->irq().stat(), sys->irq().mask());
    }
  }

  const u64 irq_vblank = sys->irq_request_count(Interrupt::VBlank);
  const u64 irq_cdrom = sys->irq_request_count(Interrupt::CDROM);
  const u64 irq_dma = sys->irq_request_count(Interrupt::DMA);
  const u64 irq_timer0 = sys->irq_request_count(Interrupt::Timer0);
  const u64 irq_timer1 = sys->irq_request_count(Interrupt::Timer1);
  const u64 irq_timer2 = sys->irq_request_count(Interrupt::Timer2);
  const System::BootDiagnostics &diag = sys->boot_diag();

  LOG_INFO("BOOT_SUMMARY saw_cmd=%d saw_sector=%d saw_non_bios=%d saw_cd_io=%d "
           "saw_sio_io=%d saw_pad_cmd42=%d saw_tx_cmd42=%d saw_pad_id=%d saw_pad_button=%d "
           "saw_full_pad_poll=%d "
           "saw_cd_read_cmd=%d saw_cd_sector_visible=%d saw_cd_getid=%d "
           "saw_cd_setloc=%d saw_cd_seekl=%d saw_cd_readn_or_reads=%d "
           "saw_logo_present=%d logo_visible_persisted=%d fell_back_to_bios=%d "
           "logo_candidate=%d logo_a=%d logo_b=%d logo_c=%d "
           "first_cmd_frame=%d first_sector_frame=%d "
           "first_non_bios_frame=%d first_non_bios_pc=0x%08X first_cd_io_frame=%d "
           "first_cd_io_cycle=%llu first_cd_io_addr=0x%08X first_sio_io_frame=%d "
           "first_sio_io_cycle=%llu first_sio_io_addr=0x%08X "
           "first_pad_cmd42_frame=%d first_tx_cmd42_frame=%d first_pad_id_frame=%d first_pad_button_frame=%d "
           "first_full_pad_poll_frame=%d "
           "first_cd_read_cmd_frame=%d first_cd_sector_visible_frame=%d "
           "first_cd_getid_frame=%d first_cd_setloc_frame=%d first_cd_seekl_frame=%d "
           "first_cd_readn_or_reads_frame=%d first_logo_present_frame=%d "
           "first_logo_persisted_frame=%d first_black_after_logo_frame=%d "
           "first_fell_back_frame=%d first_logo_candidate_frame=%d "
           "cdio=%llu sioio=%llu pad42_count=%llu padpoll_count=%llu "
           "padpoll_ch0=%llu padpoll_ch1=%llu sio_invalid_seq=%llu "
           "pad_packets=%llu last_pad_buttons=0x%04X last_sio_tx=0x%02X last_sio_rx=0x%02X "
           "joy_stat=0x%04X joy_ctrl=0x%04X "
           "sio_irq_assert=%llu sio_irq_ack=%llu cdread_count=%llu "
           "cd_cmd01=%llu cd_cmd06=%llu cd_cmd08=%llu cd_cmd09=%llu "
           "cd_cmd0A=%llu cd_cmd15=%llu cd_cmd1A=%llu "
           "cd_irq1=%llu cd_irq2=%llu cd_irq3=%llu cd_irq4=%llu cd_irq5=%llu "
           "pending_irqs=%zu cd_busy=%d last_cd_irq=%u "
           "cd_resp_fifo=%zu cd_param_fifo=%zu cd_resp_promotions=%llu cd_read_stalls=%llu "
           "cd_status_e0=%llu cd_status_e0_streak=%llu "
           "display_hash=0x%08X display_non_black=%llu display_wh=%ux%u "
           "display_start=%u,%u display_enabled=%u display_24=%u "
           "irq_vblank=%llu irq_cdrom=%llu irq_dma=%llu irq_t0=%llu irq_t1=%llu "
           "irq_t2=%llu final_pc=0x%08X",
           saw_cd_command ? 1 : 0, saw_cd_sector ? 1 : 0,
           saw_non_bios_pc ? 1 : 0, saw_cd_io ? 1 : 0, saw_sio_io ? 1 : 0,
           saw_pad_cmd42 ? 1 : 0, saw_tx_cmd42 ? 1 : 0,
           saw_pad_id ? 1 : 0, saw_pad_button ? 1 : 0,
           saw_full_pad_poll ? 1 : 0,
           saw_cd_read_cmd ? 1 : 0, saw_cd_sector_visible ? 1 : 0,
           saw_cd_getid ? 1 : 0, saw_cd_setloc ? 1 : 0, saw_cd_seekl ? 1 : 0,
           saw_cd_readn_or_reads ? 1 : 0, saw_logo_present ? 1 : 0,
           logo_visible_persisted ? 1 : 0,
           fell_back_to_bios_after_non_bios ? 1 : 0, logo_candidate ? 1 : 0,
           logo_condition_a ? 1 : 0, logo_condition_b ? 1 : 0,
           logo_condition_c ? 1 : 0,
           first_cmd_frame, first_sector_frame, first_non_bios_frame,
           first_non_bios_pc, first_cd_io_frame,
           static_cast<unsigned long long>(first_cd_io_cycle), first_cd_io_addr,
           first_sio_io_frame, static_cast<unsigned long long>(first_sio_io_cycle),
           first_sio_io_addr, first_pad_cmd42_frame, first_tx_cmd42_frame,
           first_pad_id_frame,
           first_pad_button_frame, first_full_pad_poll_frame,
           first_cd_read_cmd_frame, first_cd_sector_visible_frame,
           first_cd_getid_frame, first_cd_setloc_frame, first_cd_seekl_frame,
           first_cd_readn_or_reads_frame, first_logo_present_frame,
           first_logo_persisted_frame, first_black_after_logo_frame,
           first_fell_back_frame, first_logo_candidate_frame,
           static_cast<unsigned long long>(diag.cd_io_count),
           static_cast<unsigned long long>(diag.sio_io_count),
           static_cast<unsigned long long>(diag.pad_cmd42_count),
           static_cast<unsigned long long>(diag.pad_poll_count),
           static_cast<unsigned long long>(diag.ch0_poll_count),
           static_cast<unsigned long long>(diag.ch1_poll_count),
           static_cast<unsigned long long>(diag.sio_invalid_seq_count),
           static_cast<unsigned long long>(diag.pad_packet_count),
           static_cast<unsigned>(diag.last_pad_buttons),
           static_cast<unsigned>(diag.last_sio_tx),
           static_cast<unsigned>(diag.last_sio_rx),
           static_cast<unsigned>(diag.last_joy_stat),
           static_cast<unsigned>(diag.last_joy_ctrl),
           static_cast<unsigned long long>(diag.sio_irq_assert_count),
           static_cast<unsigned long long>(diag.sio_irq_ack_count),
           static_cast<unsigned long long>(diag.cd_read_command_count),
           static_cast<unsigned long long>(sys->cdrom().command_count_for(0x01u)),
           static_cast<unsigned long long>(sys->cdrom().command_count_for(0x06u)),
           static_cast<unsigned long long>(sys->cdrom().command_count_for(0x08u)),
           static_cast<unsigned long long>(sys->cdrom().command_count_for(0x09u)),
           static_cast<unsigned long long>(sys->cdrom().command_count_for(0x0Au)),
           static_cast<unsigned long long>(sys->cdrom().command_count_for(0x15u)),
           static_cast<unsigned long long>(sys->cdrom().command_count_for(0x1Au)),
           static_cast<unsigned long long>(diag.cd_irq_int1_count),
           static_cast<unsigned long long>(diag.cd_irq_int2_count),
           static_cast<unsigned long long>(diag.cd_irq_int3_count),
           static_cast<unsigned long long>(diag.cd_irq_int4_count),
           static_cast<unsigned long long>(diag.cd_irq_int5_count),
           sys->cdrom().pending_irq_count(), sys->cdrom().busy_cycles_remaining(),
           static_cast<unsigned>(sys->cdrom().last_irq_code()),
           sys->cdrom().response_fifo_size(), sys->cdrom().param_fifo_size(),
           static_cast<unsigned long long>(sys->cdrom().response_promotion_count()),
           static_cast<unsigned long long>(sys->cdrom().read_buffer_stall_count()),
           static_cast<unsigned long long>(sys->cdrom().status_e0_poll_count()),
           static_cast<unsigned long long>(sys->cdrom().status_e0_streak_max()),
           diag.display_hash,
           static_cast<unsigned long long>(diag.display_non_black_pixels),
           static_cast<unsigned>(diag.display_width),
           static_cast<unsigned>(diag.display_height),
           static_cast<unsigned>(diag.display_x_start),
           static_cast<unsigned>(diag.display_y_start),
           static_cast<unsigned>(diag.display_enabled),
           static_cast<unsigned>(diag.display_is_24bit),
           static_cast<unsigned long long>(irq_vblank),
           static_cast<unsigned long long>(irq_cdrom),
           static_cast<unsigned long long>(irq_dma),
           static_cast<unsigned long long>(irq_timer0),
           static_cast<unsigned long long>(irq_timer1),
           static_cast<unsigned long long>(irq_timer2), sys->cpu().pc());
  log_mdec_summary("BOOT", *sys);

  const bool pass = saw_cd_getid && saw_cd_setloc && saw_cd_seekl &&
                    saw_cd_readn_or_reads && saw_cd_sector_visible;
  if (!pass) {
    LOG_ERROR("BOOT_TEST_FAIL reason=progress_gate");
  } else {
    LOG_INFO("BOOT_TEST_PASS");
  }

  if (owns_log && g_log_file) {
    log_flush_repeats();
    std::fclose(g_log_file);
    g_log_file = nullptr;
  }
  return pass ? 0 : 2;
}

int main(int argc, char *argv[]) {
  std::vector<std::string> args;
  args.reserve(static_cast<size_t>(std::max(argc - 1, 0)));
  for (int i = 1; i < argc; ++i) {
    args.emplace_back(argv[i]);
  }

  auto trim_cli_arg = [](const std::string &input) {
    const size_t begin = input.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
      return std::string();
    }
    const size_t end = input.find_last_not_of(" \t\r\n");
    std::string out = input.substr(begin, end - begin + 1u);
    if (out.size() >= 2u && out.front() == '"' && out.back() == '"') {
      out = out.substr(1u, out.size() - 2u);
    }
    return out;
  };
  auto looks_like_disc_path = [&](const std::string &value) {
    std::string lowered = trim_cli_arg(value);
    std::transform(lowered.begin(), lowered.end(), lowered.begin(),
                   [](unsigned char c) {
                     return static_cast<char>(std::tolower(c));
                   });
    return lowered.size() >= 4u &&
           ((lowered.size() >= 4u &&
             lowered.compare(lowered.size() - 4u, 4u, ".cue") == 0) ||
            (lowered.size() >= 4u &&
             lowered.compare(lowered.size() - 4u, 4u, ".bin") == 0));
  };

  std::string wav_out_path;
  std::string gpu_debug_out_path;
  std::string windowed_bios_path;
  std::string windowed_disc_path;
  std::string windowed_bios_only_path;
  bool windowed_direct_boot = false;
  int fmv_diagnostics_override = -1;
  int audio_target_latency_override_ms = -1;
  int audio_soft_latency_override_ms = -1;
  int audio_max_latency_override_ms = -1;
  int audio_queue_override = -1;
  int audio_smooth_trim_override = -1;
  int audio_raw_drift_override = -1;
  int show_audio_stats_override = -1;
  int audio_stats_log_override = -1;
  std::vector<std::string> passthrough;
  passthrough.reserve(args.size());
  for (size_t i = 0; i < args.size(); ++i) {
    const std::string &a = args[i];
    if (a == "--log-level" && (i + 1) < args.size()) {
      LogLevel parsed = LogLevel::Info;
      if (parse_log_level(args[i + 1], parsed)) {
        g_log_level = parsed;
      }
      ++i;
      continue;
    }
    if (a == "--cpu") {
      if ((i + 1) >= args.size()) {
        fprintf(stderr,
                "WARN: --cpu requires interpreter, decoded, or x64jit\n");
        continue;
      }
      CpuExecutionMode parsed = CpuExecutionMode::Interpreter;
      if (parse_cpu_execution_mode(args[i + 1], parsed)) {
        g_cpu_execution_mode_cli_override = true;
        g_cpu_execution_mode_cli_value = parsed;
      } else {
        fprintf(stderr, "WARN: Ignoring invalid CPU backend: %s\n",
                args[i + 1].c_str());
      }
      ++i;
      continue;
    }
    if (a == "--interpreter") {
      g_cpu_execution_mode_cli_override = true;
      g_cpu_execution_mode_cli_value = CpuExecutionMode::Interpreter;
      continue;
    }
    if (a == "--jit" || a == "--recompiler") {
      g_cpu_execution_mode_cli_override = true;
      g_cpu_execution_mode_cli_value = CpuExecutionMode::X64Jit;
      continue;
    }
    if (a == "--decoded" || a == "--block-interpreter") {
      g_cpu_execution_mode_cli_override = true;
      g_cpu_execution_mode_cli_value = CpuExecutionMode::DecodedBlockInterpreter;
      continue;
    }
    if (a == "--jit-hot-threshold" && (i + 1) < args.size()) {
      g_cpu_x64_jit_hot_block_threshold =
          static_cast<u32>(std::max(0, std::atoi(args[i + 1].c_str())));
      ++i;
      continue;
    }
    if (a == "--jit-min-block-instructions" && (i + 1) < args.size()) {
      g_cpu_x64_jit_min_block_instructions =
          static_cast<u32>(std::max(1, std::atoi(args[i + 1].c_str())));
      ++i;
      continue;
    }
    if (a == "--jit-force-compile") {
      g_cpu_x64_jit_force_compile = true;
      continue;
    }
    if (a == "--jit-disable-all-native") {
      g_cpu_x64_jit_all_native_cli_override = true;
      g_cpu_x64_jit_all_native_cli_value = false;
      continue;
    }
    if (a == "--jit-enable-all-native") {
      g_cpu_x64_jit_all_native_cli_override = true;
      g_cpu_x64_jit_all_native_cli_value = true;
      continue;
    }
    if (a == "--jit-disable-native-memory") {
      g_cpu_x64_jit_native_memory_cli_override = true;
      g_cpu_x64_jit_native_memory_cli_value = false;
      continue;
    }
    if (a == "--jit-enable-native-memory") {
      g_cpu_x64_jit_native_memory_cli_override = true;
      g_cpu_x64_jit_native_memory_cli_value = true;
      continue;
    }
    if (a == "--jit-disable-native-loads") {
      g_cpu_x64_jit_disable_native_loads = true;
      continue;
    }
    if (a == "--jit-disable-native-stores") {
      g_cpu_x64_jit_disable_native_stores = true;
      continue;
    }
    if (a == "--jit-disable-native-mmio") {
      g_cpu_x64_jit_disable_native_mmio = true;
      continue;
    }
    if (a == "--jit-disable-native-ram") {
      g_cpu_x64_jit_disable_native_ram = true;
      continue;
    }
    if (a == "--jit-disable-native-load-delay") {
      g_cpu_x64_jit_disable_native_load_delay = true;
      continue;
    }
    if (a == "--jit-disable-native-mixed-load-store") {
      g_cpu_x64_jit_disable_native_mixed_load_store = true;
      continue;
    }
    if (a == "--jit-memory-trace") {
      g_cpu_x64_jit_memory_trace = true;
      continue;
    }
    if (a == "--jit-memory-trace-pc" && (i + 1) < args.size()) {
      char *end = nullptr;
      const unsigned long parsed = std::strtoul(args[i + 1].c_str(), &end, 0);
      if (end != args[i + 1].c_str() && end != nullptr && *end == '\0') {
        g_cpu_x64_jit_memory_trace_pc_set = true;
        g_cpu_x64_jit_memory_trace_pc = static_cast<u32>(parsed);
        g_cpu_x64_jit_memory_trace = true;
      } else {
        fprintf(stderr, "WARN: Ignoring invalid native-memory trace PC: %s\n",
                args[i + 1].c_str());
      }
      ++i;
      continue;
    }
    if (a == "--jit-memory-trace-count" && (i + 1) < args.size()) {
      g_cpu_x64_jit_memory_trace_count =
          static_cast<u32>(std::max(1, std::atoi(args[i + 1].c_str())));
      ++i;
      continue;
    }
    if (a == "--jit-enable-ram-load-fastpath") {
      g_cpu_x64_jit_ram_load_fastpath_cli_override = true;
      g_cpu_x64_jit_ram_load_fastpath_enabled = true;
      continue;
    }
    if (a == "--jit-disable-ram-load-fastpath") {
      g_cpu_x64_jit_ram_load_fastpath_cli_override = true;
      g_cpu_x64_jit_ram_load_fastpath_enabled = false;
      continue;
    }
    if (a == "--jit-disable-native-alu") {
      g_cpu_x64_jit_native_alu_cli_override = true;
      g_cpu_x64_jit_native_alu_cli_value = false;
      continue;
    }
    if (a == "--jit-enable-native-alu") {
      g_cpu_x64_jit_native_alu_cli_override = true;
      g_cpu_x64_jit_native_alu_cli_value = true;
      continue;
    }
    if (a == "--jit-disable-branch-tail") {
      g_cpu_x64_jit_branch_tail_cli_override = true;
      g_cpu_x64_jit_branch_tail_cli_value = false;
      continue;
    }
    if (a == "--jit-enable-branch-tail") {
      g_cpu_x64_jit_branch_tail_cli_override = true;
      g_cpu_x64_jit_branch_tail_cli_value = true;
      continue;
    }
    if (a == "--jit-enable-reduced-helper-branch-tail") {
      g_cpu_x64_jit_reduced_helper_branch_tail_enabled = true;
      continue;
    }
    if (a == "--jit-disable-reduced-helper-branch-tail") {
      g_cpu_x64_jit_reduced_helper_branch_tail_enabled = false;
      continue;
    }
    if (a == "--jit-enable-aggressive-reduced-helper-branch-tail") {
      g_cpu_x64_jit_aggressive_reduced_helper_branch_tail_cli_override = true;
      g_cpu_x64_jit_aggressive_reduced_helper_branch_tail_cli_value = true;
      g_cpu_x64_jit_aggressive_reduced_helper_branch_tail_enabled = true;
      continue;
    }
    if (a == "--jit-disable-aggressive-reduced-helper-branch-tail") {
      g_cpu_x64_jit_aggressive_reduced_helper_branch_tail_cli_override = true;
      g_cpu_x64_jit_aggressive_reduced_helper_branch_tail_cli_value = false;
      g_cpu_x64_jit_aggressive_reduced_helper_branch_tail_enabled = false;
      continue;
    }
    if (a == "--jit-enable-native-prefix") {
      g_cpu_x64_jit_native_prefix_enabled = true;
      continue;
    }
    if (a == "--jit-disable-native-prefix") {
      g_cpu_x64_jit_native_prefix_enabled = false;
      continue;
    }
    if (a == "--jit-enable-aggressive-native-prefix-ram") {
      g_cpu_x64_jit_aggressive_native_prefix_ram_cli_override = true;
      g_cpu_x64_jit_aggressive_native_prefix_ram_cli_value = true;
      g_cpu_x64_jit_aggressive_native_prefix_ram_enabled = true;
      continue;
    }
    if (a == "--jit-disable-aggressive-native-prefix-ram") {
      g_cpu_x64_jit_aggressive_native_prefix_ram_cli_override = true;
      g_cpu_x64_jit_aggressive_native_prefix_ram_cli_value = false;
      g_cpu_x64_jit_aggressive_native_prefix_ram_enabled = false;
      continue;
    }
    if (a == "--jit-branch-tail-log") {
      g_cpu_x64_jit_branch_tail_logging = true;
      g_cpu_backend_stats_logging = true;
      continue;
    }
    if (a == "--jit-branch-tail-log-count" && (i + 1) < args.size()) {
      g_cpu_x64_jit_branch_tail_log_count =
          static_cast<u32>(std::max(1, std::atoi(args[i + 1].c_str())));
      ++i;
      continue;
    }
    if (a == "--jit-branch-tail-blacklist-pc" && (i + 1) < args.size()) {
      char *end = nullptr;
      const unsigned long parsed =
          std::strtoul(args[i + 1].c_str(), &end, 0);
      if (end != args[i + 1].c_str() && end != nullptr && *end == '\0') {
        const u32 pc = static_cast<u32>(parsed);
        if (std::find(g_cpu_x64_jit_branch_tail_blacklist.begin(),
                      g_cpu_x64_jit_branch_tail_blacklist.end(),
                      pc) == g_cpu_x64_jit_branch_tail_blacklist.end()) {
          g_cpu_x64_jit_branch_tail_blacklist.push_back(pc);
        }
      } else {
        fprintf(stderr, "WARN: Ignoring invalid branch-tail PC: %s\n",
                args[i + 1].c_str());
      }
      ++i;
      continue;
    }
    if (a == "--jit-clear-branch-tail-blacklist") {
      g_cpu_x64_jit_branch_tail_blacklist.clear();
      continue;
    }
    if (a == "--cpu-backend-stats-log-frames" && (i + 1) < args.size()) {
      g_cpu_backend_stats_log_frames =
          static_cast<u32>(std::max(1, std::atoi(args[i + 1].c_str())));
      ++i;
      continue;
    }
    if (a == "--frame-state-log-frames" && (i + 1) < args.size()) {
      g_frame_state_log_frames =
          static_cast<u32>(std::max(1, std::atoi(args[i + 1].c_str())));
      ++i;
      continue;
    }
    if (a == "--cpu-backend-stats-log") {
      g_cpu_backend_stats_logging = true;
      continue;
    }
    if (a == "--no-cpu-backend-stats-log") {
      g_cpu_backend_stats_logging = false;
      continue;
    }
    if (a == "--cpu-backend-rejected-block-log" ||
        a == "--cpu-backend-hot-reject-log") {
      g_cpu_backend_rejected_block_logging = true;
      continue;
    }
    if (a == "--no-cpu-backend-rejected-block-log" ||
        a == "--no-cpu-backend-hot-reject-log") {
      g_cpu_backend_rejected_block_logging = false;
      continue;
    }
    if ((a == "--cpu-backend-rejected-block-log-count" ||
         a == "--cpu-backend-hot-reject-log-count") &&
        (i + 1) < args.size()) {
      g_cpu_backend_rejected_block_log_count =
          static_cast<u32>(std::max(1, std::atoi(args[i + 1].c_str())));
      ++i;
      continue;
    }
    if (a == "--trace" && (i + 1) < args.size()) {
      apply_trace_list(args[i + 1]);
      ++i;
      continue;
    }
    if (a == "--trace-rate" && (i + 1) < args.size()) {
      apply_trace_tuning_list(args[i + 1], false);
      ++i;
      continue;
    }
    if (a == "--trace-burst" && (i + 1) < args.size()) {
      apply_trace_tuning_list(args[i + 1], true);
      ++i;
      continue;
    }
    if (a == "--categories" && (i + 1) < args.size()) {
      apply_category_list(args[i + 1]);
      ++i;
      continue;
    }
    if (a == "--timestamp" && (i + 1) < args.size()) {
      std::string v = args[i + 1];
      std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
      g_log_timestamp = !(v == "0" || v == "off" || v == "false");
      ++i;
      continue;
    }
    if (a == "--dedupe" && (i + 1) < args.size()) {
      std::string v = args[i + 1];
      std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
      g_log_dedupe = !(v == "0" || v == "off" || v == "false");
      ++i;
      continue;
    }
    if (a == "--dedupe-flush" && (i + 1) < args.size()) {
      g_log_dedupe_flush = std::max(1, std::atoi(args[i + 1].c_str()));
      ++i;
      continue;
    }
    if (a == "--log-file" && (i + 1) < args.size()) {
      g_log_file = std::fopen(args[i + 1].c_str(), "w");
      if (g_log_file) {
        std::setvbuf(g_log_file, nullptr, _IONBF, 0);
      }
      ++i;
      continue;
    }
    if (a == "--wav-out" && (i + 1) < args.size()) {
      wav_out_path = args[i + 1];
      ++i;
      continue;
    }
    if (a == "--host-wav-out" && (i + 1) < args.size()) {
      g_spu_host_wav_out_path = args[i + 1];
      ++i;
      continue;
    }
    if (a == "--audio-target-latency-ms" && (i + 1) < args.size()) {
      audio_target_latency_override_ms =
          std::clamp(std::atoi(args[i + 1].c_str()), 10, 500);
      ++i;
      continue;
    }
    if (a == "--audio-max-latency-ms" && (i + 1) < args.size()) {
      audio_max_latency_override_ms =
          std::clamp(std::atoi(args[i + 1].c_str()), 10, 1000);
      ++i;
      continue;
    }
    if (a == "--audio-soft-latency-ms" && (i + 1) < args.size()) {
      audio_soft_latency_override_ms =
          std::clamp(std::atoi(args[i + 1].c_str()), 10, 750);
      ++i;
      continue;
    }
    if (a == "--audio-queue") {
      audio_queue_override = 1;
      continue;
    }
    if (a == "--no-audio-queue") {
      audio_queue_override = 0;
      continue;
    }
    if (a == "--audio-smooth-trim") {
      audio_smooth_trim_override = 1;
      continue;
    }
    if (a == "--no-audio-smooth-trim") {
      audio_smooth_trim_override = 0;
      continue;
    }
    if (a == "--audio-raw-drift") {
      audio_raw_drift_override = 1;
      audio_stats_log_override = 1;
      continue;
    }
    if (a == "--no-audio-raw-drift") {
      audio_raw_drift_override = 0;
      continue;
    }
    if (a == "--show-audio-stats") {
      show_audio_stats_override = 1;
      continue;
    }
    if (a == "--hide-audio-stats") {
      show_audio_stats_override = 0;
      continue;
    }
    if (a == "--audio-stats-log") {
      audio_stats_log_override = 1;
      continue;
    }
    if (a == "--no-audio-stats-log") {
      audio_stats_log_override = 0;
      continue;
    }
    if (a == "--lag-stutter") {
      g_spu_enable_lag_stutter = true;
      continue;
    }
    if (a == "--no-lag-stutter") {
      g_spu_enable_lag_stutter = false;
      g_spu_enable_slowdown_stutter = false;
      continue;
    }
    if (a == "--slowdown-stutter") {
      g_spu_enable_lag_stutter = true;
      g_spu_enable_slowdown_stutter = true;
      continue;
    }
    if (a == "--no-slowdown-stutter") {
      g_spu_enable_slowdown_stutter = false;
      continue;
    }
    if (a == "--gpu-debug-file" && (i + 1) < args.size()) {
      gpu_debug_out_path = args[i + 1];
      g_mdec_debug_upload_probe = true;
      ++i;
      continue;
    }
    if ((a == "--auto-input" || a == "--auto-key") && (i + 1) < args.size()) {
      if (!add_auto_input_buttons(args[i + 1])) {
        fprintf(stderr, "WARN: Ignoring invalid auto input spec: %s\n",
                args[i + 1].c_str());
      }
      ++i;
      continue;
    }
    if ((a == "--auto-mash" || a == "--auto-mash-key") &&
        (i + 1) < args.size()) {
      if (!add_auto_input_buttons(args[i + 1])) {
        fprintf(stderr, "WARN: Ignoring invalid auto mash spec: %s\n",
                args[i + 1].c_str());
      } else {
        g_auto_input.period_frames = 6;
        g_auto_input.hold_frames = 2;
      }
      ++i;
      continue;
    }
    if (a == "--auto-input-start" && (i + 1) < args.size()) {
      g_auto_input.start_frame = std::max(1, std::atoi(args[i + 1].c_str()));
      ++i;
      continue;
    }
    if (a == "--auto-input-end" && (i + 1) < args.size()) {
      g_auto_input.end_frame = std::max(0, std::atoi(args[i + 1].c_str()));
      ++i;
      continue;
    }
    if (a == "--auto-input-period" && (i + 1) < args.size()) {
      g_auto_input.period_frames = std::max(1, std::atoi(args[i + 1].c_str()));
      ++i;
      continue;
    }
    if (a == "--auto-input-hold" && (i + 1) < args.size()) {
      g_auto_input.hold_frames = std::max(1, std::atoi(args[i + 1].c_str()));
      ++i;
      continue;
    }
    if (a == "--auto-input-hold-key") {
      g_auto_input.period_frames = 1;
      g_auto_input.hold_frames = 1;
      continue;
    }
    // Input recorder/playback flags
    if (a == "--record-input" && (i + 1) < args.size()) {
      g_input_recorder_config.record_path = trim_cli_arg(args[i + 1]);
      ++i;
      continue;
    }
    if (a == "--play-input" && (i + 1) < args.size()) {
      g_input_recorder_config.playback_path = trim_cli_arg(args[i + 1]);
      ++i;
      continue;
    }
    if (a == "--input-playback-stop-at-end") {
      g_input_recorder_config.end_behavior = InputRecorder::PlaybackEndBehavior::Stop;
      continue;
    }
    if (a == "--input-playback-loop") {
      g_input_recorder_config.end_behavior = InputRecorder::PlaybackEndBehavior::Loop;
      continue;
    }
    if (a == "--windowed-disc" && (i + 2) < args.size()) {
      windowed_bios_path = trim_cli_arg(args[i + 1]);
      size_t consumed = i + 2;
      windowed_disc_path = trim_cli_arg(args[consumed]);
      while (!looks_like_disc_path(windowed_disc_path) &&
             (consumed + 1) < args.size() &&
             (args[consumed + 1].rfind("--", 0) != 0)) {
        ++consumed;
        if (!windowed_disc_path.empty()) {
          windowed_disc_path.push_back(' ');
        }
        windowed_disc_path += trim_cli_arg(args[consumed]);
      }
      i = consumed;
      continue;
    }
    if (a == "--windowed-direct-boot") {
      windowed_direct_boot = true;
      continue;
    }
    if (a == "--windowed-bios" && (i + 1) < args.size()) {
      windowed_bios_only_path = trim_cli_arg(args[i + 1]);
      ++i;
      continue;
    }
    if (a == "--fmv-diagnostics") {
      fmv_diagnostics_override = 1;
      continue;
    }
    if (a == "--mdec-compare") {
      g_mdec_debug_compare_macroblocks = true;
      continue;
    }
    if (a == "--mdec-upload-probe") {
      g_mdec_debug_upload_probe = true;
      continue;
    }
    if (a == "--no-fmv-diagnostics") {
      fmv_diagnostics_override = 0;
      continue;
    }
    if (a == "--experimental-bios-size") {
      if ((i + 1) < args.size()) {
        std::string v = args[i + 1];
        std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) {
          return static_cast<char>(std::tolower(c));
        });
        const bool looks_bool =
            (v == "1" || v == "0" || v == "on" || v == "off" ||
             v == "true" || v == "false");
        if (looks_bool) {
          g_experimental_bios_size_mode =
              !(v == "0" || v == "off" || v == "false");
          ++i;
          continue;
        }
      }
      g_experimental_bios_size_mode = true;
      continue;
    }
    if (a == "--unsafe-ps2-bios-mode") {
      if ((i + 1) < args.size()) {
        std::string v = args[i + 1];
        std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) {
          return static_cast<char>(std::tolower(c));
        });
        const bool looks_bool =
            (v == "1" || v == "0" || v == "on" || v == "off" ||
             v == "true" || v == "false");
        if (looks_bool) {
          g_unsafe_ps2_bios_mode =
              !(v == "0" || v == "off" || v == "false");
          if (g_unsafe_ps2_bios_mode) {
            g_experimental_bios_size_mode = true;
          }
          ++i;
          continue;
        }
      }
      g_unsafe_ps2_bios_mode = true;
      g_experimental_bios_size_mode = true;
      continue;
    }
    if (a == "--detailed-profiling") {
      g_profile_detailed_timing = true;
      continue;
    }
    if (a == "--no-detailed-profiling") {
      g_profile_detailed_timing = false;
      continue;
    }
    if (a == "--experimental-dma-command-sanitizer") {
      if ((i + 1) < args.size()) {
        std::string v = args[i + 1];
        std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) {
          return static_cast<char>(std::tolower(c));
        });
        const bool looks_bool =
            (v == "1" || v == "0" || v == "on" || v == "off" ||
             v == "true" || v == "false");
        if (looks_bool) {
          g_experimental_dma_command_sanitizer =
              !(v == "0" || v == "off" || v == "false");
          ++i;
          continue;
        }
      }
      g_experimental_dma_command_sanitizer = true;
      continue;
    }
    passthrough.push_back(a);
  }

  if (fmv_diagnostics_override >= 0) {
    g_log_fmv_diagnostics = (fmv_diagnostics_override != 0);
  }

  if (!passthrough.empty() &&
      (passthrough[0] == "--cpu-backend-compare-test" ||
       passthrough[0] == "--cpu-compare-test" ||
       passthrough[0] == "--jit-memory-compare-test")) {
    const bool memory_only =
        passthrough[0] == "--jit-memory-compare-test";
    const int rc = run_cpu_backend_compare_test(memory_only);
    if (g_log_file) {
      log_flush_repeats();
      std::fclose(g_log_file);
      g_log_file = nullptr;
    }
    return rc;
  }

  if (passthrough.size() >= 2 && passthrough[0] == "--bios-test") {
    int steps = 20000;
    if (passthrough.size() >= 3) {
      steps = std::max(1, std::atoi(passthrough[2].c_str()));
    }
    const int rc = run_bios_test(passthrough[1], steps);
    if (g_log_file) {
      log_flush_repeats();
      std::fclose(g_log_file);
      g_log_file = nullptr;
    }
    return rc;
  }
  if (passthrough.size() >= 4 && passthrough[0] == "--boot-disc-test") {
    const int frames = std::max(1, std::atoi(passthrough[2].c_str()));
    std::string bin_path;
    std::string cue_path;
    if (passthrough.size() >= 5) {
      bin_path = passthrough[3];
      cue_path = passthrough[4];
    } else {
      cue_path = passthrough[3];
      bin_path = resolve_first_bin_from_cue_cli(cue_path);
      if (bin_path.empty()) {
        LOG_ERROR("BOOT_TEST_FAIL reason=cue_bin_resolve cue=%s", cue_path.c_str());
        if (g_log_file) {
          log_flush_repeats();
          std::fclose(g_log_file);
          g_log_file = nullptr;
        }
        return 1;
      }
    }
    const int rc = run_boot_disc_test(passthrough[1], frames, bin_path, cue_path);
    if (g_log_file) {
      log_flush_repeats();
      std::fclose(g_log_file);
      g_log_file = nullptr;
    }
    return rc;
  }
  if (passthrough.size() >= 3 && passthrough[0] == "--frame-test") {
    int frames = std::max(1, std::atoi(passthrough[2].c_str()));
    int rc = 0;
    if (passthrough.size() >= 5) {
      rc = run_frame_test(passthrough[1], frames, passthrough[3],
                          passthrough[4], gpu_debug_out_path);
    } else if (passthrough.size() >= 4) {
      const std::string cue_path = passthrough[3];
      const std::string bin_path = resolve_first_bin_from_cue_cli(cue_path);
      if (bin_path.empty()) {
        LOG_ERROR("FRAME_TEST_FAIL reason=cue_bin_resolve cue=%s",
                  cue_path.c_str());
        if (g_log_file) {
          log_flush_repeats();
          std::fclose(g_log_file);
          g_log_file = nullptr;
        }
        return 1;
      }
      rc = run_frame_test(passthrough[1], frames, bin_path, cue_path,
                          gpu_debug_out_path);
    } else {
      rc = run_frame_test(passthrough[1], frames, "", "", gpu_debug_out_path);
    }
    if (g_log_file) {
      log_flush_repeats();
      std::fclose(g_log_file);
      g_log_file = nullptr;
    }
    return rc;
  }
  if (passthrough.size() >= 3 && passthrough[0] == "--spu-audio-test") {
    const int frames = std::max(1, std::atoi(passthrough[2].c_str()));
    int rc = 0;
    if (passthrough.size() >= 5) {
      rc = run_spu_audio_test(passthrough[1], frames, passthrough[3],
                              passthrough[4], wav_out_path);
    } else {
      rc = run_spu_audio_test(passthrough[1], frames, "", "", wav_out_path);
    }
    if (g_log_file) {
      log_flush_repeats();
      std::fclose(g_log_file);
      g_log_file = nullptr;
    }
    return rc;
  }

  SDL_SetMainReady(); // Tell SDL we handled main() ourselves

  printf("========================================\n");
  printf("  %s\n", VIBESTATION_FULL_VERSION_STRING);
  printf("========================================\n");
  printf("Starting up...\n");
  fflush(stdout);

  App app;
  app.set_input_recorder_config(g_input_recorder_config);
  printf("App object created.\n");
  fflush(stdout);

  if (!app.init()) {
    fprintf(stderr, "FATAL: Failed to initialize VibeStation!\n");
    fflush(stderr);
    printf("Press Enter to exit...\n");
    fflush(stdout);
    getchar();
    return 1;
  }

  if (audio_target_latency_override_ms >= 0) {
    g_spu_audio_target_latency_ms =
        static_cast<u32>(audio_target_latency_override_ms);
  }
  if (audio_max_latency_override_ms >= 0) {
    g_spu_audio_max_latency_ms =
        static_cast<u32>(audio_max_latency_override_ms);
  }
  if (audio_soft_latency_override_ms >= 0) {
    g_spu_audio_soft_latency_ms =
        static_cast<u32>(audio_soft_latency_override_ms);
  }
  g_spu_audio_soft_latency_ms = std::clamp(g_spu_audio_soft_latency_ms,
      g_spu_audio_target_latency_ms, 750u);
  g_spu_audio_max_latency_ms = std::clamp(g_spu_audio_max_latency_ms,
      g_spu_audio_soft_latency_ms, 1000u);
  if (audio_queue_override >= 0) {
    g_spu_enable_audio_queue = audio_queue_override != 0;
  }
  if (audio_smooth_trim_override >= 0) {
    g_spu_enable_smooth_trim = audio_smooth_trim_override != 0;
  }
  if (audio_raw_drift_override >= 0) {
    g_spu_audio_raw_drift = audio_raw_drift_override != 0;
  }
  if (show_audio_stats_override >= 0) {
    g_spu_show_audio_stats = show_audio_stats_override != 0;
  }
  if (audio_stats_log_override >= 0) {
    g_spu_audio_stats_log = audio_stats_log_override != 0;
  }

  if (!windowed_disc_path.empty()) {
    printf("Launching disc from command line...\n");
    fflush(stdout);
    if (!app.launch_disc_from_cli(windowed_bios_path, windowed_disc_path,
                                  windowed_direct_boot)) {
      fprintf(stderr, "FATAL: Failed to launch command-line disc.\n");
      fflush(stderr);
      app.shutdown();
      if (g_log_file) {
        log_flush_repeats();
        std::fclose(g_log_file);
        g_log_file = nullptr;
      }
      return 1;
    }
  }
  else if (!windowed_bios_only_path.empty()) {
    printf("Launching BIOS-only (no disc) from command line...\n");
    fflush(stdout);
    if (!app.launch_bios_only_from_cli(windowed_bios_only_path)) {
      fprintf(stderr, "FATAL: Failed to launch BIOS-only.\n");
      fflush(stderr);
      app.shutdown();
      if (g_log_file) {
        log_flush_repeats();
        std::fclose(g_log_file);
        g_log_file = nullptr;
      }
      return 1;
    }
  }

  printf("Initialization complete! Running...\n");
  fflush(stdout);

  app.run();
  app.shutdown();

  return 0;
}
