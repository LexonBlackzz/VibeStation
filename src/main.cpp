// ── VibeStation — PS1 Emulator ──────────────────────────────────────
// Entry point

#define SDL_MAIN_HANDLED // Prevent SDL from redefining main()
#include "core/system.h"
#include "input/controller.h"
#include "ui/app.h"
#include <SDL.h>
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <array>
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

namespace {
constexpr u32 kCpuComparePc = 0x80010000u;

struct CpuCompareMemoryWord {
  u32 addr = 0;
  u32 value = 0;
};

struct CpuCompareCodeMutation {
  u32 after_instructions = 0;
  u32 addr = 0;
  u32 value = 0;
  bool invalidate_icache_line = true;
};

struct CpuCompareNativeTierMode {
  bool all_native = true;
  bool memory_native = true;
  bool alu_native = true;
};

struct CpuCompareCase {
  const char *name = "";
  std::vector<u32> program;
  std::vector<CpuCompareMemoryWord> memory;
  std::vector<u32> compare_memory_addresses;
  std::vector<CpuCompareCodeMutation> mutations;
  std::vector<u32> segment_instructions;
  std::vector<CpuCompareNativeTierMode> segment_native_tiers;
  std::array<u32, 32> initial_gpr{};
  u32 initial_cop0_sr_bits = 0;
  u32 initial_irq_mask = 0;
  bool initial_irq_pending = false;
  bool request_irq_on_branch = false;
  u32 instructions = 0;
  bool expect_final_control_state = false;
  u32 expected_pc = 0;
  u32 expected_next_pc = 0;
  u32 expected_current_pc = 0;
  u64 expected_cycles = 0;
  bool experimental_unknown_fallback = false;
  bool require_full_native_when_available = false;
  bool require_native_entry_when_available = false;
  bool require_native_memory_helper_when_available = false;
  bool require_native_memory_exception_when_available = false;
  bool require_native_helper_load_delay_entry_when_available = false;
  bool require_native_branch_tail_when_available = false;
  bool native_branch_should_be_taken = false;
  u8 native_branch_primary_op = 0xFFu;
  bool require_native_branch_delay_memory_helper_when_available = false;
  bool require_native_mmio_when_available = false;
  bool disable_branch_tail_for_x64 = false;
  bool blacklist_branch_tail_for_x64 = false;
  bool require_branch_tail_disabled_fallback_when_available = false;
  bool require_branch_tail_blacklisted_fallback_when_available = false;
  bool allow_partial_native_branch_tail = false;
  bool allow_partial_native_memory_helper = false;
  bool compare_segment_states = false;
  bool disable_all_native_for_x64 = false;
  bool disable_memory_native_for_x64 = false;
  bool disable_alu_native_for_x64 = false;
  bool require_all_native_disabled_fallback_when_available = false;
  bool require_memory_native_disabled_fallback_when_available = false;
  bool require_alu_native_disabled_fallback_when_available = false;
  bool require_native_memory_tier_entry_when_available = false;
  bool require_native_alu_tier_entry_when_available = false;
  bool enable_ram_load_fastpath_for_x64 = false;
  bool require_native_ram_load_fastpath_when_available = false;
  bool require_no_native_ram_load_fastpath = false;
  bool expect_x64_fallback = false;
};

struct CpuComparePeripheralState {
  u32 irq_stat = 0;
  u32 irq_mask = 0;
  u32 dma_dpcr = 0;
  u32 dma_dicr = 0;
  u64 cd_sector_count = 0;
  int cd_read_lba = 0;
  int cd_active_lba = 0;
  int cd_data_index = 0;
  int cd_busy_cycles = 0;
  size_t cd_pending_irqs = 0;
  size_t cd_response_size = 0;
  u8 cd_last_irq = 0;
  bool cd_data_ready = false;
  bool cd_data_request = false;
};

static bool cpu_compare_peripherals_equal(
    const CpuComparePeripheralState &a,
    const CpuComparePeripheralState &b) {
  return a.irq_stat == b.irq_stat && a.irq_mask == b.irq_mask &&
         a.dma_dpcr == b.dma_dpcr && a.dma_dicr == b.dma_dicr &&
         a.cd_sector_count == b.cd_sector_count &&
         a.cd_read_lba == b.cd_read_lba &&
         a.cd_active_lba == b.cd_active_lba &&
         a.cd_data_index == b.cd_data_index &&
         a.cd_busy_cycles == b.cd_busy_cycles &&
         a.cd_pending_irqs == b.cd_pending_irqs &&
         a.cd_response_size == b.cd_response_size &&
         a.cd_last_irq == b.cd_last_irq &&
         a.cd_data_ready == b.cd_data_ready &&
         a.cd_data_request == b.cd_data_request;
}

struct CpuCompareRunResult {
  CpuDebugState state{};
  CpuBackendStats stats{};
  CpuRunSliceResult run{};
  std::vector<CpuDebugState> segment_states;
  std::vector<CpuComparePeripheralState> segment_peripherals;
  CpuComparePeripheralState peripherals{};
  u32 irq_stat = 0;
  u32 irq_mask = 0;
  std::vector<u32> memory_values;
};

static CpuComparePeripheralState capture_cpu_compare_peripherals(
    System &sys) {
  const CdRom &cd = sys.cdrom();
  CpuComparePeripheralState out{};
  out.irq_stat = sys.irq().stat();
  out.irq_mask = sys.irq().mask();
  out.dma_dpcr = sys.debug_dma_read(0x70u);
  out.dma_dicr = sys.debug_dma_read(0x74u);
  out.cd_sector_count = cd.sector_count();
  out.cd_read_lba = cd.current_read_lba();
  out.cd_active_lba = cd.active_data_lba();
  out.cd_data_index = cd.dma_data_index();
  out.cd_busy_cycles = cd.busy_cycles_remaining();
  out.cd_pending_irqs = cd.pending_irq_count();
  out.cd_response_size = cd.response_fifo_size();
  out.cd_last_irq = cd.last_irq_code();
  out.cd_data_ready = cd.sector_data_ready();
  out.cd_data_request = cd.sector_data_request();
  return out;
}

static u32 enc_r(u32 rs, u32 rt, u32 rd, u32 shamt, u32 funct) {
  return ((rs & 31u) << 21) | ((rt & 31u) << 16) | ((rd & 31u) << 11) |
         ((shamt & 31u) << 6) | (funct & 63u);
}

static u32 enc_i(u32 op, u32 rs, u32 rt, u16 imm) {
  return ((op & 63u) << 26) | ((rs & 31u) << 21) | ((rt & 31u) << 16) |
         imm;
}

static u32 enc_j(u32 op, u32 target) {
  return ((op & 63u) << 26) | ((target >> 2) & 0x03FFFFFFu);
}

static const char *cpu_compare_mode_name(CpuExecutionMode mode) {
  switch (mode) {
  case CpuExecutionMode::DecodedBlockInterpreter:
    return "DecodedBlockInterpreter";
  case CpuExecutionMode::X64Jit:
    return "X64Jit";
  case CpuExecutionMode::Interpreter:
  default:
    return "Interpreter";
  }
}

static const char *cpu_compare_outcome(CpuExecutionMode mode,
                                       const CpuBackendStats &stats) {
  if (mode == CpuExecutionMode::Interpreter) {
    return "interpreter";
  }

  if (stats.native_block_entries != 0 || stats.native_instructions != 0) {
    if (stats.decoded_instructions != 0 || stats.fallback_instructions != 0 ||
        stats.interpreter_fallback_steps != 0) {
      return "native_with_fallback";
    }
    return "native";
  }

  if (mode == CpuExecutionMode::X64Jit && !stats.native_available) {
    if (stats.decoded_instructions != 0 || stats.decoded_block_entries != 0) {
      return "x64_unavailable_decoded_fallback";
    }
    if (stats.interpreter_fallback_steps != 0 ||
        stats.fallback_instructions != 0) {
      return "x64_unavailable_interpreter_fallback";
    }
    return "x64_unavailable_no_optimized_work";
  }

  if (stats.decoded_instructions != 0 || stats.decoded_block_entries != 0) {
    if (stats.interpreter_fallback_steps != 0 ||
        stats.fallback_instructions != 0) {
      return mode == CpuExecutionMode::X64Jit
                 ? "decoded_with_interpreter_fallback"
                 : "decoded_interpreter_fallback";
    }
    return mode == CpuExecutionMode::X64Jit ? "decoded_fallback"
                                            : "decoded";
  }

  if (stats.interpreter_fallback_steps != 0 || stats.fallback_instructions != 0) {
    return mode == CpuExecutionMode::X64Jit ? "interpreter_fallback"
                                            : "fallback";
  }

  return "no_optimized_work";
}

static void log_cpu_compare_program(const CpuCompareCase &test_case) {
  for (size_t i = 0; i < test_case.program.size(); ++i) {
    LOG_ERROR("CPU_COMPARE_PROGRAM name=%s index=%u pc=0x%08X opcode=0x%08X",
              test_case.name, static_cast<unsigned>(i),
              kCpuComparePc + static_cast<u32>(i * 4u),
              test_case.program[i]);
  }
}

static bool cpu_debug_states_equal(const CpuDebugState &a,
                                   const CpuDebugState &b) {
  for (u32 i = 0; i < 32u; ++i) {
    if (a.gpr[i] != b.gpr[i]) {
      return false;
    }
  }

  return a.pc == b.pc && a.next_pc == b.next_pc &&
         a.current_pc == b.current_pc && a.hi == b.hi && a.lo == b.lo &&
         a.load_reg == b.load_reg && a.load_value == b.load_value &&
         a.next_load_reg == b.next_load_reg &&
         a.next_load_value == b.next_load_value &&
         a.in_delay_slot == b.in_delay_slot &&
         a.pending_delay_slot == b.pending_delay_slot &&
         a.pending_branch_taken == b.pending_branch_taken &&
         a.pending_branch_pc == b.pending_branch_pc &&
         a.active_branch_pc == b.active_branch_pc &&
         a.exception_raised == b.exception_raised &&
         a.cop0_sr == b.cop0_sr && a.cop0_cause == b.cop0_cause &&
         a.cop0_epc == b.cop0_epc &&
         a.cop0_badvaddr == b.cop0_badvaddr && a.cycles == b.cycles;
}

static void log_cpu_debug_state_diff(const char *name,
                                     const char *actual_mode,
                                     const CpuDebugState &reference,
                                     const CpuDebugState &actual) {
  auto field = [&](const char *field_name, auto a, auto b) {
    if (a != b) {
      LOG_ERROR("CPU_COMPARE_DIFF name=%s mode=%s field=%s reference=0x%llX actual=0x%llX",
                name, actual_mode, field_name,
                static_cast<unsigned long long>(a),
                static_cast<unsigned long long>(b));
    }
  };

  field("pc", reference.pc, actual.pc);
  field("next_pc", reference.next_pc, actual.next_pc);
  field("current_pc", reference.current_pc, actual.current_pc);
  field("hi", reference.hi, actual.hi);
  field("lo", reference.lo, actual.lo);
  field("load_reg", reference.load_reg, actual.load_reg);
  field("load_value", reference.load_value, actual.load_value);
  field("next_load_reg", reference.next_load_reg, actual.next_load_reg);
  field("next_load_value", reference.next_load_value,
        actual.next_load_value);
  field("in_delay_slot", reference.in_delay_slot ? 1u : 0u,
        actual.in_delay_slot ? 1u : 0u);
  field("pending_delay_slot", reference.pending_delay_slot ? 1u : 0u,
        actual.pending_delay_slot ? 1u : 0u);
  field("pending_branch_taken", reference.pending_branch_taken ? 1u : 0u,
        actual.pending_branch_taken ? 1u : 0u);
  field("pending_branch_pc", reference.pending_branch_pc,
        actual.pending_branch_pc);
  field("active_branch_pc", reference.active_branch_pc,
        actual.active_branch_pc);
  field("exception_raised", reference.exception_raised ? 1u : 0u,
        actual.exception_raised ? 1u : 0u);
  field("cop0_sr", reference.cop0_sr, actual.cop0_sr);
  field("cop0_cause", reference.cop0_cause, actual.cop0_cause);
  field("cop0_epc", reference.cop0_epc, actual.cop0_epc);
  field("cop0_badvaddr", reference.cop0_badvaddr, actual.cop0_badvaddr);
  field("cycles", reference.cycles, actual.cycles);

  for (u32 i = 0; i < 32u; ++i) {
    if (reference.gpr[i] != actual.gpr[i]) {
      LOG_ERROR("CPU_COMPARE_DIFF name=%s mode=%s reg=r%u reference=0x%08X actual=0x%08X",
                name, actual_mode, static_cast<unsigned>(i), reference.gpr[i],
                actual.gpr[i]);
    }
  }
}

static bool cpu_compare_expected_state_pass(const CpuCompareCase &test_case,
                                            CpuExecutionMode mode,
                                            const CpuDebugState &state) {
  if (!test_case.expect_final_control_state) {
    return true;
  }

  bool pass = true;
  auto field = [&](const char *field_name, auto expected, auto actual) {
    if (expected == actual) {
      return;
    }
    pass = false;
    LOG_ERROR("CPU_COMPARE_EXPECTED_DIFF name=%s mode=%s field=%s expected=0x%llX actual=0x%llX",
              test_case.name, cpu_compare_mode_name(mode), field_name,
              static_cast<unsigned long long>(expected),
              static_cast<unsigned long long>(actual));
  };

  field("pc", test_case.expected_pc, state.pc);
  field("next_pc", test_case.expected_next_pc, state.next_pc);
  field("current_pc", test_case.expected_current_pc, state.current_pc);
  field("cycles", test_case.expected_cycles, state.cycles);
  return pass;
}

static CpuCompareRunResult run_cpu_compare_case_once(
    const CpuCompareCase &test_case, CpuExecutionMode mode) {
  auto sys = std::make_unique<System>();
  sys->init_hardware();
  sys->reset();

  for (size_t i = 0; i < test_case.program.size(); ++i) {
    sys->write32((kCpuComparePc & 0x1FFFFFFFu) + static_cast<u32>(i * 4u),
                 test_case.program[i]);
  }
  for (const CpuCompareMemoryWord &word : test_case.memory) {
    sys->write32(word.addr, word.value);
  }
  if (test_case.initial_irq_mask != 0u ||
      test_case.request_irq_on_branch || test_case.initial_irq_pending) {
    sys->irq().write(4u, test_case.initial_irq_mask);
  }
  if (test_case.initial_irq_pending) {
    sys->irq().request(Interrupt::VBlank);
  }

  CpuDebugState initial = sys->cpu().debug_state();
  initial.pc = kCpuComparePc;
  initial.next_pc = kCpuComparePc + 4u;
  initial.current_pc = 0;
  initial.cycles = 0;
  initial.load_reg = 0;
  initial.load_value = 0;
  initial.next_load_reg = 0;
  initial.next_load_value = 0;
  initial.in_delay_slot = false;
  initial.pending_delay_slot = false;
  initial.pending_branch_taken = false;
  initial.pending_branch_pc = 0;
  initial.active_branch_pc = 0;
  initial.exception_raised = false;
  initial.cop0_sr |= test_case.initial_cop0_sr_bits;
  for (u32 i = 0; i < 32u; ++i) {
    initial.gpr[i] = test_case.initial_gpr[i];
  }
  initial.gpr[0] = 0;
  sys->cpu().debug_set_state(initial);
  sys->cpu().flush_cpu_backend();
  sys->cpu().notify_cpu_backend_frame(1);

  g_cpu_execution_mode_cli_override = true;
  g_cpu_execution_mode_cli_value = mode;
  g_cpu_backend_compare_irq_on_branch = test_case.request_irq_on_branch;
  g_cpu_backend_compare_allow_partial_branch_tail =
      mode == CpuExecutionMode::X64Jit &&
      test_case.allow_partial_native_branch_tail;
  g_cpu_backend_compare_allow_partial_memory_helper =
      mode == CpuExecutionMode::X64Jit &&
      test_case.allow_partial_native_memory_helper;
  g_cpu_x64_jit_branch_tail_cli_override = true;
  g_cpu_x64_jit_branch_tail_cli_value =
      !(mode == CpuExecutionMode::X64Jit &&
        test_case.disable_branch_tail_for_x64);
  g_cpu_x64_jit_branch_tail_blacklist.clear();
  if (mode == CpuExecutionMode::X64Jit &&
      test_case.blacklist_branch_tail_for_x64) {
    g_cpu_x64_jit_branch_tail_blacklist.push_back(kCpuComparePc);
  }
  g_cpu_x64_jit_all_native_cli_override = true;
  g_cpu_x64_jit_all_native_cli_value =
      !(mode == CpuExecutionMode::X64Jit &&
        test_case.disable_all_native_for_x64);
  g_cpu_x64_jit_native_memory_cli_override = true;
  g_cpu_x64_jit_native_memory_cli_value =
      !(mode == CpuExecutionMode::X64Jit &&
        test_case.disable_memory_native_for_x64);
  g_cpu_x64_jit_native_alu_cli_override = true;
  g_cpu_x64_jit_native_alu_cli_value =
      !(mode == CpuExecutionMode::X64Jit &&
        test_case.disable_alu_native_for_x64);
  g_cpu_x64_jit_ram_load_fastpath_enabled =
      mode == CpuExecutionMode::X64Jit &&
      test_case.enable_ram_load_fastpath_for_x64;
  CpuCompareRunResult out{};
  u32 executed = 0;
  size_t segment_index = 0;
  auto run_segment = [&](u32 instruction_count) {
    if (instruction_count == 0) {
      return;
    }
    if (mode == CpuExecutionMode::X64Jit &&
        segment_index < test_case.segment_native_tiers.size()) {
      const CpuCompareNativeTierMode &tiers =
          test_case.segment_native_tiers[segment_index];
      g_cpu_x64_jit_all_native_cli_value = tiers.all_native;
      g_cpu_x64_jit_native_memory_cli_value = tiers.memory_native;
      g_cpu_x64_jit_native_alu_cli_value = tiers.alu_native;
    }
    CpuRunSliceResult segment = sys->cpu().run_slice(100000u, instruction_count);
    out.run.cycles += segment.cycles;
    out.run.instructions += segment.instructions;
    executed += segment.instructions;
    out.segment_states.push_back(sys->cpu().debug_state());
    out.segment_peripherals.push_back(
        capture_cpu_compare_peripherals(*sys));
    ++segment_index;
  };

  if (!test_case.segment_instructions.empty()) {
    for (u32 instruction_count : test_case.segment_instructions) {
      if (executed >= test_case.instructions) {
        break;
      }
      const u32 remaining = test_case.instructions - executed;
      run_segment(std::min(instruction_count, remaining));
    }
  } else {
    for (const CpuCompareCodeMutation &mutation : test_case.mutations) {
      const u32 target = std::min(mutation.after_instructions,
                                  test_case.instructions);
      if (target > executed) {
        run_segment(target - executed);
      }
      sys->write32(mutation.addr, mutation.value);
      if (mutation.invalidate_icache_line) {
        sys->cpu().debug_invalidate_icache_line(mutation.addr);
      }
    }
  }
  if (executed < test_case.instructions) {
    run_segment(test_case.instructions - executed);
  }
  out.state = sys->cpu().debug_state();
  out.stats = sys->cpu().cpu_backend_stats();
  out.peripherals = capture_cpu_compare_peripherals(*sys);
  out.irq_stat = sys->irq().stat();
  out.irq_mask = sys->irq().mask();
  for (u32 addr : test_case.compare_memory_addresses) {
    out.memory_values.push_back(sys->read32(addr));
  }
  return out;
}

static void pad_cpu_compare_program(CpuCompareCase &test_case,
                                    u32 instruction_count = 16u) {
  while (test_case.program.size() < instruction_count) {
    test_case.program.push_back(0);
  }
  test_case.instructions = instruction_count;
}

static std::vector<CpuCompareCase> make_cpu_compare_cases() {
  std::vector<CpuCompareCase> cases;

  CpuCompareCase native_control{};
  native_control.name = "native_control_state_icache_cycles";
  native_control.program.assign(16u, 0u);
  native_control.instructions = 16;
  native_control.expect_final_control_state = true;
  native_control.expected_pc = kCpuComparePc + 16u * 4u;
  native_control.expected_next_pc = native_control.expected_pc + 4u;
  native_control.expected_current_pc = kCpuComparePc + 15u * 4u;
  native_control.expected_cycles = 32u;
  native_control.require_full_native_when_available = true;
  cases.push_back(native_control);

  CpuCompareCase all_native_disabled{};
  all_native_disabled.name = "x64_all_native_disabled_gate";
  all_native_disabled.program.assign(16u, 0u);
  all_native_disabled.instructions = 16;
  all_native_disabled.disable_all_native_for_x64 = true;
  all_native_disabled.expect_x64_fallback = true;
  all_native_disabled.require_all_native_disabled_fallback_when_available =
      true;
  cases.push_back(all_native_disabled);

  CpuCompareCase native_memory_disabled{};
  native_memory_disabled.name = "x64_native_memory_disabled_gate";
  native_memory_disabled.initial_gpr[1] = 0x80011180u;
  native_memory_disabled.memory.push_back({0x00011180u, 0x12345678u});
  native_memory_disabled.program = {enc_i(0x23, 1, 2, 0), 0};
  pad_cpu_compare_program(native_memory_disabled);
  native_memory_disabled.disable_memory_native_for_x64 = true;
  native_memory_disabled.expect_x64_fallback = true;
  native_memory_disabled
      .require_memory_native_disabled_fallback_when_available = true;
  cases.push_back(native_memory_disabled);

  CpuCompareCase native_alu_disabled{};
  native_alu_disabled.name = "x64_native_alu_disabled_gate";
  native_alu_disabled.program.assign(16u, 0u);
  native_alu_disabled.instructions = 16;
  native_alu_disabled.disable_alu_native_for_x64 = true;
  native_alu_disabled.expect_x64_fallback = true;
  native_alu_disabled.require_alu_native_disabled_fallback_when_available =
      true;
  cases.push_back(native_alu_disabled);

  CpuCompareCase native_mixed{};
  native_mixed.name = "native_mixed_alu_immediate";
  native_mixed.program = {
      0,
      enc_i(0x09, 0, 1, 0x0001),
      enc_i(0x09, 1, 1, 0x0001),
      enc_i(0x0F, 0, 2, 0x1234),
      enc_i(0x0D, 2, 2, 0x5678),
      enc_i(0x0C, 2, 3, 0x00FF),
      enc_i(0x0E, 3, 4, 0x00AA),
      enc_r(1, 1, 5, 0, 0x21),
      enc_r(5, 1, 6, 0, 0x23),
      enc_r(2, 4, 7, 0, 0x24),
      enc_r(2, 4, 8, 0, 0x25),
      enc_r(2, 4, 9, 0, 0x26),
      enc_r(2, 4, 10, 0, 0x27),
      enc_i(0x0A, 10, 11, 0x0000),
      enc_i(0x0B, 10, 12, 0xFFFF),
      enc_r(12, 11, 13, 0, 0x21),
  };
  native_mixed.require_full_native_when_available = true;
  native_mixed.instructions = 16;
  cases.push_back(native_mixed);

  CpuCompareCase r0_writes{};
  r0_writes.name = "native_r0_writes_ignored";
  r0_writes.program = {
      enc_i(0x09, 0, 0, 0x1234),
      enc_i(0x0F, 0, 0, 0xFFFF),
      enc_i(0x0D, 0, 0, 0xFFFF),
      enc_r(0, 0, 0, 4, 0x00),
      enc_i(0x09, 0, 1, 0x0007),
  };
  r0_writes.require_full_native_when_available = true;
  pad_cpu_compare_program(r0_writes);
  cases.push_back(r0_writes);

  CpuCompareCase addiu_overlap{};
  addiu_overlap.name = "native_addiu_positive_negative_overlap";
  addiu_overlap.initial_gpr[1] = 0x7FFFFFFFu;
  addiu_overlap.initial_gpr[2] = 0x00000010u;
  addiu_overlap.program = {
      enc_i(0x09, 0, 3, 0x7FFF),
      enc_i(0x09, 3, 4, 0x8000),
      enc_i(0x09, 1, 1, 0x0001),
      enc_i(0x0D, 2, 2, 0x0001),
      enc_r(1, 1, 1, 0, 0x21),
  };
  addiu_overlap.require_full_native_when_available = true;
  pad_cpu_compare_program(addiu_overlap);
  cases.push_back(addiu_overlap);

  CpuCompareCase wrapping_logic{};
  wrapping_logic.name = "native_wrapping_logic_lui_zero_extend";
  wrapping_logic.program = {
      enc_i(0x0F, 0, 1, 0xFFFF),
      enc_i(0x0D, 1, 1, 0xFFFF),
      enc_i(0x09, 0, 2, 0x0001),
      enc_r(1, 2, 3, 0, 0x21),
      enc_r(2, 1, 4, 0, 0x23),
      enc_i(0x0F, 0, 5, 0x00FF),
      enc_i(0x0C, 5, 6, 0xF0F0),
      enc_i(0x0D, 6, 7, 0x0F0F),
      enc_i(0x0E, 7, 8, 0xFFFF),
      enc_r(7, 8, 9, 0, 0x24),
      enc_r(7, 8, 10, 0, 0x25),
      enc_r(7, 8, 11, 0, 0x26),
      enc_r(7, 8, 12, 0, 0x27),
  };
  wrapping_logic.require_full_native_when_available = true;
  pad_cpu_compare_program(wrapping_logic);
  cases.push_back(wrapping_logic);

  CpuCompareCase comparisons{};
  comparisons.name = "native_signed_unsigned_comparisons";
  comparisons.program = {
      enc_i(0x09, 0, 1, 0xFFFF),
      enc_i(0x09, 0, 2, 0x0001),
      enc_i(0x0F, 0, 3, 0x8000),
      enc_r(1, 2, 4, 0, 0x2A),
      enc_r(1, 2, 5, 0, 0x2B),
      enc_r(3, 2, 6, 0, 0x2A),
      enc_r(3, 2, 7, 0, 0x2B),
      enc_i(0x0A, 2, 8, 0xFFFF),
      enc_i(0x0A, 1, 9, 0x0001),
      enc_i(0x0B, 2, 10, 0xFFFF),
      enc_i(0x0B, 1, 11, 0x0001),
  };
  comparisons.require_full_native_when_available = true;
  pad_cpu_compare_program(comparisons);
  cases.push_back(comparisons);

  CpuCompareCase shifts{};
  shifts.name = "native_shift_immediate_and_variable";
  shifts.program = {
      enc_i(0x0F, 0, 1, 0x8000),
      enc_i(0x0D, 1, 1, 0x0001),
      enc_r(0, 1, 2, 0, 0x00),
      enc_r(0, 1, 3, 4, 0x00),
      enc_r(0, 1, 4, 0, 0x02),
      enc_r(0, 1, 5, 4, 0x02),
      enc_r(0, 1, 6, 0, 0x03),
      enc_r(0, 1, 7, 4, 0x03),
      enc_i(0x09, 0, 8, 0x0028),
      enc_r(8, 1, 9, 0, 0x04),
      enc_r(8, 1, 10, 0, 0x06),
      enc_r(8, 1, 11, 0, 0x07),
      enc_r(8, 1, 1, 0, 0x04),
  };
  shifts.require_full_native_when_available = true;
  pad_cpu_compare_program(shifts);
  cases.push_back(shifts);

  CpuCompareCase bne_taken{};
  bne_taken.name = "native_branch_tail_bne_taken_alu_delay";
  bne_taken.initial_gpr[1] = 1u;
  bne_taken.initial_gpr[2] = 2u;
  bne_taken.program = {
      enc_i(0x05, 1, 2, 2),
      enc_i(0x09, 0, 3, 0x0011),
      enc_i(0x09, 0, 4, 0x0022),
      enc_i(0x09, 0, 5, 0x0033),
  };
  bne_taken.instructions = 2;
  bne_taken.require_full_native_when_available = true;
  bne_taken.require_native_branch_tail_when_available = true;
  bne_taken.native_branch_should_be_taken = true;
  cases.push_back(bne_taken);

  CpuCompareCase bne_not_taken{};
  bne_not_taken.name = "native_branch_tail_bne_not_taken_alu_delay";
  bne_not_taken.initial_gpr[1] = 1u;
  bne_not_taken.initial_gpr[2] = 1u;
  bne_not_taken.program = bne_taken.program;
  bne_not_taken.instructions = 2;
  bne_not_taken.require_full_native_when_available = true;
  bne_not_taken.require_native_branch_tail_when_available = true;
  cases.push_back(bne_not_taken);

  CpuCompareCase beq_taken{};
  beq_taken.name = "native_branch_tail_beq_taken_alu_delay";
  beq_taken.initial_gpr[1] = 7u;
  beq_taken.initial_gpr[2] = 7u;
  beq_taken.program = {
      enc_i(0x04, 1, 2, 2),
      enc_i(0x09, 0, 3, 0x0011),
      enc_i(0x09, 0, 4, 0x0022),
      enc_i(0x09, 0, 5, 0x0033),
  };
  beq_taken.instructions = 2;
  beq_taken.require_full_native_when_available = true;
  beq_taken.require_native_branch_tail_when_available = true;
  beq_taken.native_branch_should_be_taken = true;
  cases.push_back(beq_taken);

  CpuCompareCase beq_not_taken{};
  beq_not_taken.name = "native_branch_tail_beq_not_taken_alu_delay";
  beq_not_taken.initial_gpr[1] = 7u;
  beq_not_taken.initial_gpr[2] = 8u;
  beq_not_taken.program = beq_taken.program;
  beq_not_taken.instructions = 2;
  beq_not_taken.require_full_native_when_available = true;
  beq_not_taken.require_native_branch_tail_when_available = true;
  cases.push_back(beq_not_taken);

  CpuCompareCase bgtz_taken{};
  bgtz_taken.name = "native_branch_tail_bgtz_taken_alu_delay";
  bgtz_taken.initial_gpr[1] = 1u;
  bgtz_taken.program = {
      enc_i(0x07, 1, 0, 1),
      enc_i(0x09, 0, 2, 0x0031),
      0,
  };
  bgtz_taken.instructions = 2;
  bgtz_taken.require_full_native_when_available = true;
  bgtz_taken.require_native_branch_tail_when_available = true;
  bgtz_taken.native_branch_should_be_taken = true;
  bgtz_taken.native_branch_primary_op = 0x07u;
  cases.push_back(bgtz_taken);

  CpuCompareCase bgtz_not_taken{};
  bgtz_not_taken.name = "native_branch_tail_bgtz_not_taken_alu_delay";
  bgtz_not_taken.initial_gpr[1] = 0xFFFFFFFFu;
  bgtz_not_taken.program = bgtz_taken.program;
  bgtz_not_taken.instructions = 2;
  bgtz_not_taken.require_full_native_when_available = true;
  bgtz_not_taken.require_native_branch_tail_when_available = true;
  bgtz_not_taken.native_branch_primary_op = 0x07u;
  cases.push_back(bgtz_not_taken);

  CpuCompareCase blez_taken{};
  blez_taken.name = "native_branch_tail_blez_taken_alu_delay";
  blez_taken.initial_gpr[1] = 0u;
  blez_taken.program = {
      enc_i(0x06, 1, 0, 1),
      enc_i(0x09, 0, 2, 0x0032),
      0,
  };
  blez_taken.instructions = 2;
  blez_taken.require_full_native_when_available = true;
  blez_taken.require_native_branch_tail_when_available = true;
  blez_taken.native_branch_should_be_taken = true;
  blez_taken.native_branch_primary_op = 0x06u;
  cases.push_back(blez_taken);

  CpuCompareCase blez_not_taken{};
  blez_not_taken.name = "native_branch_tail_blez_not_taken_alu_delay";
  blez_not_taken.initial_gpr[1] = 1u;
  blez_not_taken.program = blez_taken.program;
  blez_not_taken.instructions = 2;
  blez_not_taken.require_full_native_when_available = true;
  blez_not_taken.require_native_branch_tail_when_available = true;
  blez_not_taken.native_branch_primary_op = 0x06u;
  cases.push_back(blez_not_taken);

  CpuCompareCase bgtz_memory_loop{};
  bgtz_memory_loop.name = "native_branch_tail_bgtz_memory_store_delay";
  bgtz_memory_loop.initial_gpr[1] = 0x80011240u;
  bgtz_memory_loop.memory.push_back({0x00011240u, 2u});
  bgtz_memory_loop.program = {
      enc_i(0x24, 1, 2, 0),
      0,
      enc_i(0x09, 2, 2, 0xFFFF),
      enc_i(0x07, 2, 0, 0xFFFD),
      enc_i(0x28, 1, 2, 1),
  };
  bgtz_memory_loop.instructions = 5;
  bgtz_memory_loop.require_full_native_when_available = true;
  bgtz_memory_loop.require_native_memory_helper_when_available = true;
  bgtz_memory_loop.require_native_branch_tail_when_available = true;
  bgtz_memory_loop.native_branch_should_be_taken = true;
  bgtz_memory_loop.native_branch_primary_op = 0x07u;
  bgtz_memory_loop
      .require_native_branch_delay_memory_helper_when_available = true;
  cases.push_back(bgtz_memory_loop);

  CpuCompareCase blez_memory_delay{};
  blez_memory_delay.name = "native_branch_tail_blez_memory_load_delay";
  blez_memory_delay.initial_gpr[1] = 0u;
  blez_memory_delay.initial_gpr[6] = 0x80011250u;
  blez_memory_delay.memory.push_back({0x00011250u, 0x55667788u});
  blez_memory_delay.program = {
      enc_i(0x06, 1, 0, 1),
      enc_i(0x23, 6, 5, 0),
      0,
  };
  blez_memory_delay.instructions = 2;
  blez_memory_delay.require_full_native_when_available = true;
  blez_memory_delay.require_native_memory_helper_when_available = true;
  blez_memory_delay.require_native_branch_tail_when_available = true;
  blez_memory_delay.native_branch_should_be_taken = true;
  blez_memory_delay.native_branch_primary_op = 0x06u;
  blez_memory_delay
      .require_native_branch_delay_memory_helper_when_available = true;
  cases.push_back(blez_memory_delay);

  CpuCompareCase beq_memory_body_delay{};
  beq_memory_body_delay.name = "native_branch_tail_beq_memory_body_delay";
  beq_memory_body_delay.initial_gpr[1] = 0x80011260u;
  beq_memory_body_delay.initial_gpr[2] = 5u;
  beq_memory_body_delay.memory.push_back({0x00011264u, 0x11223344u});
  beq_memory_body_delay.program = {
      enc_i(0x2B, 1, 2, 0),
      enc_i(0x23, 1, 4, 0),
      0,
      enc_i(0x04, 4, 2, 1),
      enc_i(0x23, 1, 5, 4),
      0,
  };
  beq_memory_body_delay.instructions = 5;
  beq_memory_body_delay.require_full_native_when_available = true;
  beq_memory_body_delay.require_native_memory_helper_when_available = true;
  beq_memory_body_delay.require_native_branch_tail_when_available = true;
  beq_memory_body_delay.native_branch_should_be_taken = true;
  beq_memory_body_delay
      .require_native_branch_delay_memory_helper_when_available = true;
  cases.push_back(beq_memory_body_delay);

  CpuCompareCase bne_memory_body{};
  bne_memory_body.name = "native_branch_tail_bne_memory_body";
  bne_memory_body.initial_gpr[1] = 0x80011100u;
  bne_memory_body.initial_gpr[2] = 0x12345678u;
  bne_memory_body.program = {
      enc_i(0x2B, 1, 2, 0),
      enc_i(0x23, 1, 3, 0),
      0,
      enc_i(0x05, 3, 0, 1),
      enc_i(0x09, 3, 4, 1),
      0,
  };
  bne_memory_body.instructions = 5;
  bne_memory_body.require_full_native_when_available = true;
  bne_memory_body.require_native_memory_helper_when_available = true;
  bne_memory_body.require_native_branch_tail_when_available = true;
  bne_memory_body.native_branch_should_be_taken = true;
  cases.push_back(bne_memory_body);

  CpuCompareCase bne_lw_delay{};
  bne_lw_delay.name = "native_branch_tail_bne_lw_delay";
  bne_lw_delay.initial_gpr[1] = 1u;
  bne_lw_delay.initial_gpr[2] = 0u;
  bne_lw_delay.initial_gpr[6] = 0x80011120u;
  bne_lw_delay.memory.push_back({0x00011120u, 0xCAFEBABEu});
  bne_lw_delay.program = {
      enc_i(0x05, 1, 2, 1),
      enc_i(0x23, 6, 5, 0),
      0,
      0,
  };
  bne_lw_delay.instructions = 2;
  bne_lw_delay.require_full_native_when_available = true;
  bne_lw_delay.require_native_memory_helper_when_available = true;
  bne_lw_delay.require_native_branch_tail_when_available = true;
  bne_lw_delay.native_branch_should_be_taken = true;
  bne_lw_delay.require_native_branch_delay_memory_helper_when_available = true;
  cases.push_back(bne_lw_delay);

  CpuCompareCase branch_tail_then_decoded_consumer{};
  branch_tail_then_decoded_consumer.name =
      "native_branch_tail_then_decoded_load_consumer";
  branch_tail_then_decoded_consumer.initial_gpr[1] = 1u;
  branch_tail_then_decoded_consumer.initial_gpr[6] = 0x80011270u;
  branch_tail_then_decoded_consumer.memory.push_back(
      {0x00011270u, 0xABCDEF01u});
  branch_tail_then_decoded_consumer.program = {
      enc_i(0x05, 1, 0, 1),
      enc_i(0x23, 6, 5, 0),
      enc_r(5, 0, 7, 0, 0x21),
  };
  branch_tail_then_decoded_consumer.instructions = 3;
  branch_tail_then_decoded_consumer.segment_instructions = {2u, 1u};
  branch_tail_then_decoded_consumer.segment_native_tiers = {
      {true, true, true}, {false, true, true}};
  branch_tail_then_decoded_consumer.compare_segment_states = true;
  branch_tail_then_decoded_consumer
      .require_native_memory_helper_when_available = true;
  branch_tail_then_decoded_consumer
      .require_native_branch_tail_when_available = true;
  branch_tail_then_decoded_consumer.native_branch_should_be_taken = true;
  branch_tail_then_decoded_consumer
      .require_native_branch_delay_memory_helper_when_available = true;
  cases.push_back(branch_tail_then_decoded_consumer);

  CpuCompareCase decoded_load_then_branch_tail{};
  decoded_load_then_branch_tail.name =
      "decoded_load_then_native_branch_tail_cancel";
  decoded_load_then_branch_tail.initial_gpr[1] = 0x80011280u;
  decoded_load_then_branch_tail.initial_gpr[2] = 0x11111111u;
  decoded_load_then_branch_tail.initial_gpr[4] = 1u;
  decoded_load_then_branch_tail.memory.push_back(
      {0x00011280u, 0xDEADBEEFu});
  decoded_load_then_branch_tail.program = {
      enc_i(0x23, 1, 2, 0),
      enc_i(0x05, 4, 0, 1),
      enc_i(0x09, 0, 2, 7),
      0,
  };
  decoded_load_then_branch_tail.instructions = 3;
  decoded_load_then_branch_tail.segment_instructions = {1u, 2u};
  decoded_load_then_branch_tail.segment_native_tiers = {
      {false, true, true}, {true, true, true}};
  decoded_load_then_branch_tail.compare_segment_states = true;
  decoded_load_then_branch_tail
      .require_native_helper_load_delay_entry_when_available = true;
  decoded_load_then_branch_tail
      .require_native_branch_tail_when_available = true;
  decoded_load_then_branch_tail.native_branch_should_be_taken = true;
  cases.push_back(decoded_load_then_branch_tail);

  CpuCompareCase bne_delay_exception{};
  bne_delay_exception.name = "native_branch_tail_delay_memory_exception";
  bne_delay_exception.initial_gpr[1] = 1u;
  bne_delay_exception.initial_gpr[2] = 0u;
  bne_delay_exception.initial_gpr[6] = 0x80011122u;
  bne_delay_exception.program = {
      enc_i(0x05, 1, 2, 1),
      enc_i(0x23, 6, 5, 0),
      0,
  };
  bne_delay_exception.instructions = 2;
  bne_delay_exception.require_full_native_when_available = true;
  bne_delay_exception.require_native_memory_helper_when_available = true;
  bne_delay_exception.require_native_memory_exception_when_available = true;
  bne_delay_exception.require_native_branch_tail_when_available = true;
  bne_delay_exception.native_branch_should_be_taken = true;
  bne_delay_exception.require_native_branch_delay_memory_helper_when_available =
      true;
  cases.push_back(bne_delay_exception);

  CpuCompareCase bne_load_delay_crossing{};
  bne_load_delay_crossing.name = "native_branch_tail_load_delay_crossing";
  bne_load_delay_crossing.initial_gpr[1] = 0x80011130u;
  bne_load_delay_crossing.initial_gpr[2] = 0u;
  bne_load_delay_crossing.memory.push_back({0x00011130u, 0x01020304u});
  bne_load_delay_crossing.program = {
      enc_i(0x23, 1, 2, 0),
      enc_i(0x05, 2, 0, 1),
      enc_r(2, 0, 3, 0, 0x21),
      0,
  };
  bne_load_delay_crossing.instructions = 3;
  bne_load_delay_crossing.require_full_native_when_available = true;
  bne_load_delay_crossing.require_native_memory_helper_when_available = true;
  bne_load_delay_crossing.require_native_branch_tail_when_available = true;
  cases.push_back(bne_load_delay_crossing);

  CpuCompareCase bne_loop_shape{};
  bne_loop_shape.name = "native_branch_tail_atrain_loop_shape";
  bne_loop_shape.initial_gpr[1] = 0x80011140u;
  bne_loop_shape.initial_gpr[2] = 1u;
  bne_loop_shape.initial_gpr[4] = 10u;
  bne_loop_shape.program = {
      0,
      enc_r(0, 2, 2, 1, 0x00),
      enc_r(4, 2, 3, 0, 0x23),
      enc_i(0x2B, 1, 3, 0),
      enc_i(0x23, 1, 5, 0),
      0,
      enc_i(0x09, 6, 6, 1),
      enc_i(0x2B, 1, 6, 4),
      enc_i(0x23, 1, 7, 4),
      0,
      enc_i(0x0A, 6, 8, 2),
      enc_i(0x05, 8, 0, 0xFFF4),
      enc_i(0x23, 1, 9, 8),
  };
  bne_loop_shape.memory.push_back({0x00011148u, 0x0BADF00Du});
  bne_loop_shape.instructions = 26;
  bne_loop_shape.require_full_native_when_available = true;
  bne_loop_shape.require_native_memory_helper_when_available = true;
  bne_loop_shape.require_native_branch_tail_when_available = true;
  bne_loop_shape.native_branch_should_be_taken = true;
  bne_loop_shape.require_native_branch_delay_memory_helper_when_available = true;
  cases.push_back(bne_loop_shape);

  CpuCompareCase atrain_splash_poll_pair{};
  atrain_splash_poll_pair.name =
      "native_branch_tail_atrain_splash_poll_pair_segments";
  atrain_splash_poll_pair.initial_gpr[1] = 0x80011300u;
  atrain_splash_poll_pair.memory.push_back({0x00011300u, 3u});
  atrain_splash_poll_pair.program = {
      enc_i(0x09, 1, 1, 0),
      enc_i(0x23, 1, 3, 0),
      0,
      enc_i(0x09, 3, 4, 1),
      enc_i(0x2B, 1, 4, 4),
      enc_i(0x23, 1, 5, 4),
      0,
      enc_i(0x05, 5, 0, 1),
      0,
      enc_i(0x0F, 0, 6, 0x8001),
      enc_i(0x23, 6, 7, 0x1304),
      0,
      enc_r(0, 7, 8, 0, 0x2A),
      enc_i(0x05, 8, 0, 0xFFFB),
      0,
  };
  atrain_splash_poll_pair.instructions = 15;
  atrain_splash_poll_pair.segment_instructions = {9u, 6u};
  atrain_splash_poll_pair.compare_segment_states = true;
  atrain_splash_poll_pair.require_full_native_when_available = true;
  atrain_splash_poll_pair.require_native_memory_helper_when_available = true;
  atrain_splash_poll_pair.require_native_branch_tail_when_available = true;
  atrain_splash_poll_pair.native_branch_should_be_taken = true;
  atrain_splash_poll_pair.compare_memory_addresses.push_back(0x00011304u);
  cases.push_back(atrain_splash_poll_pair);

  CpuCompareCase branch_tail_disabled{};
  branch_tail_disabled.name = "native_branch_tail_disabled_gate";
  branch_tail_disabled.initial_gpr[1] = 1u;
  branch_tail_disabled.program = {
      enc_i(0x05, 1, 0, 1),
      enc_i(0x09, 0, 2, 1),
      0,
  };
  branch_tail_disabled.instructions = 2;
  branch_tail_disabled.disable_branch_tail_for_x64 = true;
  branch_tail_disabled.expect_x64_fallback = true;
  branch_tail_disabled.require_branch_tail_disabled_fallback_when_available =
      true;
  cases.push_back(branch_tail_disabled);

  CpuCompareCase branch_tail_blacklisted{};
  branch_tail_blacklisted.name = "native_branch_tail_pc_blacklist";
  branch_tail_blacklisted.initial_gpr[1] = 1u;
  branch_tail_blacklisted.program = branch_tail_disabled.program;
  branch_tail_blacklisted.instructions = 2;
  branch_tail_blacklisted.blacklist_branch_tail_for_x64 = true;
  branch_tail_blacklisted.expect_x64_fallback = true;
  branch_tail_blacklisted
      .require_branch_tail_blacklisted_fallback_when_available = true;
  cases.push_back(branch_tail_blacklisted);

  CpuCompareCase branch_irq_before{};
  branch_irq_before.name = "native_branch_tail_irq_pending_before_branch";
  branch_irq_before.initial_gpr[1] = 1u;
  branch_irq_before.initial_cop0_sr_bits = 1u | (1u << 10);
  branch_irq_before.initial_irq_mask = 1u;
  branch_irq_before.initial_irq_pending = true;
  branch_irq_before.program = {
      enc_i(0x05, 1, 0, 1),
      enc_i(0x09, 0, 2, 1),
      0,
  };
  branch_irq_before.instructions = 1;
  branch_irq_before.expect_x64_fallback = true;
  cases.push_back(branch_irq_before);

  CpuCompareCase branch_irq_delay{};
  branch_irq_delay.name = "native_branch_tail_irq_pending_before_delay";
  branch_irq_delay.initial_gpr[1] = 1u;
  branch_irq_delay.initial_cop0_sr_bits = 1u | (1u << 10);
  branch_irq_delay.initial_irq_mask = 1u;
  branch_irq_delay.request_irq_on_branch = true;
  branch_irq_delay.program = {
      enc_i(0x05, 1, 0, 1),
      enc_i(0x09, 0, 2, 1),
      0,
  };
  branch_irq_delay.instructions = 2;
  branch_irq_delay.segment_instructions = {1u, 1u};
  branch_irq_delay.allow_partial_native_branch_tail = true;
  branch_irq_delay.compare_segment_states = true;
  branch_irq_delay.require_native_branch_tail_when_available = true;
  branch_irq_delay.native_branch_should_be_taken = true;
  cases.push_back(branch_irq_delay);

  CpuCompareCase branch_mmio_body{};
  branch_mmio_body.name = "native_branch_tail_mmio_body_read_write";
  branch_mmio_body.initial_gpr[1] = 0x1F801070u;
  branch_mmio_body.initial_gpr[2] = 1u;
  branch_mmio_body.program = {
      enc_i(0x2B, 1, 2, 4),
      enc_i(0x23, 1, 3, 4),
      0,
      enc_i(0x05, 3, 0, 1),
      enc_i(0x09, 0, 4, 1),
      0,
  };
  branch_mmio_body.instructions = 5;
  branch_mmio_body.require_full_native_when_available = true;
  branch_mmio_body.require_native_memory_helper_when_available = true;
  branch_mmio_body.require_native_mmio_when_available = true;
  branch_mmio_body.require_native_branch_tail_when_available = true;
  branch_mmio_body.native_branch_should_be_taken = true;
  cases.push_back(branch_mmio_body);

  CpuCompareCase branch_mmio_load_delay{};
  branch_mmio_load_delay.name = "native_branch_tail_taken_mmio_load_delay";
  branch_mmio_load_delay.initial_gpr[1] = 1u;
  branch_mmio_load_delay.initial_gpr[6] = 0x1F801070u;
  branch_mmio_load_delay.initial_irq_mask = 1u;
  branch_mmio_load_delay.initial_irq_pending = true;
  branch_mmio_load_delay.program = {
      enc_i(0x05, 1, 0, 1),
      enc_i(0x23, 6, 5, 0),
      0,
  };
  branch_mmio_load_delay.instructions = 2;
  branch_mmio_load_delay.require_full_native_when_available = true;
  branch_mmio_load_delay.require_native_memory_helper_when_available = true;
  branch_mmio_load_delay.require_native_mmio_when_available = true;
  branch_mmio_load_delay.require_native_branch_tail_when_available = true;
  branch_mmio_load_delay.native_branch_should_be_taken = true;
  branch_mmio_load_delay
      .require_native_branch_delay_memory_helper_when_available = true;
  cases.push_back(branch_mmio_load_delay);

  CpuCompareCase branch_mmio_store_delay{};
  branch_mmio_store_delay.name = "native_branch_tail_taken_mmio_store_delay";
  branch_mmio_store_delay.initial_gpr[1] = 1u;
  branch_mmio_store_delay.initial_gpr[2] = 1u;
  branch_mmio_store_delay.initial_gpr[6] = 0x1F801070u;
  branch_mmio_store_delay.program = {
      enc_i(0x05, 1, 0, 1),
      enc_i(0x2B, 6, 2, 4),
      0,
  };
  branch_mmio_store_delay.instructions = 2;
  branch_mmio_store_delay.require_full_native_when_available = true;
  branch_mmio_store_delay.require_native_memory_helper_when_available = true;
  branch_mmio_store_delay.require_native_mmio_when_available = true;
  branch_mmio_store_delay.require_native_branch_tail_when_available = true;
  branch_mmio_store_delay.native_branch_should_be_taken = true;
  branch_mmio_store_delay
      .require_native_branch_delay_memory_helper_when_available = true;
  cases.push_back(branch_mmio_store_delay);

  CpuCompareCase branch_not_taken_lw_delay{};
  branch_not_taken_lw_delay.name =
      "native_branch_tail_not_taken_memory_delay";
  branch_not_taken_lw_delay.initial_gpr[1] = 1u;
  branch_not_taken_lw_delay.initial_gpr[2] = 1u;
  branch_not_taken_lw_delay.initial_gpr[6] = 0x80011160u;
  branch_not_taken_lw_delay.memory.push_back(
      {0x00011160u, 0x11223344u});
  branch_not_taken_lw_delay.program = {
      enc_i(0x05, 1, 2, 1),
      enc_i(0x23, 6, 5, 0),
      0,
  };
  branch_not_taken_lw_delay.instructions = 2;
  branch_not_taken_lw_delay.require_full_native_when_available = true;
  branch_not_taken_lw_delay.require_native_memory_helper_when_available = true;
  branch_not_taken_lw_delay.require_native_branch_tail_when_available = true;
  branch_not_taken_lw_delay
      .require_native_branch_delay_memory_helper_when_available = true;
  cases.push_back(branch_not_taken_lw_delay);

  CpuCompareCase repeated_mmio_branch{};
  repeated_mmio_branch.name = "native_repeated_mmio_status_reads_branch";
  repeated_mmio_branch.initial_gpr[1] = 0x1F801070u;
  repeated_mmio_branch.initial_irq_mask = 1u;
  repeated_mmio_branch.initial_irq_pending = true;
  repeated_mmio_branch.program = {
      enc_i(0x23, 1, 2, 0),
      0,
      enc_i(0x23, 1, 3, 0),
      0,
      enc_i(0x05, 2, 3, 1),
      0,
      0,
  };
  repeated_mmio_branch.instructions = 6;
  repeated_mmio_branch.require_full_native_when_available = true;
  repeated_mmio_branch.require_native_memory_helper_when_available = true;
  repeated_mmio_branch.require_native_mmio_when_available = true;
  repeated_mmio_branch.require_native_branch_tail_when_available = true;
  cases.push_back(repeated_mmio_branch);

  CpuCompareCase mmio_load_delay_branch{};
  mmio_load_delay_branch.name = "native_mmio_load_delay_then_branch";
  mmio_load_delay_branch.initial_gpr[1] = 0x1F801070u;
  mmio_load_delay_branch.initial_gpr[2] = 0u;
  mmio_load_delay_branch.initial_irq_mask = 1u;
  mmio_load_delay_branch.initial_irq_pending = true;
  mmio_load_delay_branch.program = {
      enc_i(0x23, 1, 2, 0),
      enc_i(0x05, 2, 0, 1),
      0,
      0,
  };
  mmio_load_delay_branch.instructions = 3;
  mmio_load_delay_branch.require_full_native_when_available = true;
  mmio_load_delay_branch.require_native_memory_helper_when_available = true;
  mmio_load_delay_branch.require_native_mmio_when_available = true;
  mmio_load_delay_branch.require_native_branch_tail_when_available = true;
  cases.push_back(mmio_load_delay_branch);

  CpuCompareCase mmio_write_branch{};
  mmio_write_branch.name = "native_mmio_write_then_branch";
  mmio_write_branch.initial_gpr[1] = 0x1F801070u;
  mmio_write_branch.initial_gpr[2] = 1u;
  mmio_write_branch.program = {
      enc_i(0x2B, 1, 2, 4),
      enc_i(0x05, 2, 0, 1),
      0,
      0,
  };
  mmio_write_branch.instructions = 3;
  mmio_write_branch.require_full_native_when_available = true;
  mmio_write_branch.require_native_memory_helper_when_available = true;
  mmio_write_branch.require_native_mmio_when_available = true;
  mmio_write_branch.require_native_branch_tail_when_available = true;
  mmio_write_branch.native_branch_should_be_taken = true;
  cases.push_back(mmio_write_branch);

  CpuCompareCase native_memory_mid_block_irq{};
  native_memory_mid_block_irq.name = "native_memory_mid_block_irq_state";
  native_memory_mid_block_irq.initial_gpr[1] = 0x1F801070u;
  native_memory_mid_block_irq.initial_gpr[2] = 1u;
  native_memory_mid_block_irq.initial_cop0_sr_bits = 0x401u;
  native_memory_mid_block_irq.initial_irq_pending = true;
  native_memory_mid_block_irq.program = {
      enc_i(0x2B, 1, 2, 4),
      enc_i(0x05, 0, 0, 1),
      0,
  };
  native_memory_mid_block_irq.instructions = 2;
  native_memory_mid_block_irq.allow_partial_native_branch_tail = true;
  native_memory_mid_block_irq.require_native_entry_when_available = true;
  native_memory_mid_block_irq.require_native_memory_helper_when_available =
      true;
  native_memory_mid_block_irq.require_native_mmio_when_available = true;
  cases.push_back(native_memory_mid_block_irq);

  CpuCompareCase native_memory_then_decoded_load{};
  native_memory_then_decoded_load.name =
      "native_memory_then_decoded_consumes_load";
  native_memory_then_decoded_load.initial_gpr[1] = 0x800111A0u;
  native_memory_then_decoded_load.initial_gpr[2] = 0x11111111u;
  native_memory_then_decoded_load.memory.push_back(
      {0x000111A0u, 0x22222222u});
  native_memory_then_decoded_load.program.assign(17u, 0u);
  native_memory_then_decoded_load.program[15] = enc_i(0x23, 1, 2, 0);
  native_memory_then_decoded_load.program[16] =
      enc_r(2, 0, 3, 0, 0x21);
  native_memory_then_decoded_load.instructions = 17;
  native_memory_then_decoded_load.segment_instructions = {16u, 1u};
  native_memory_then_decoded_load.segment_native_tiers = {
      {true, true, true}, {false, true, true}};
  native_memory_then_decoded_load.compare_segment_states = true;
  native_memory_then_decoded_load.enable_ram_load_fastpath_for_x64 = true;
  native_memory_then_decoded_load
      .require_native_ram_load_fastpath_when_available = true;
  native_memory_then_decoded_load
      .require_native_memory_tier_entry_when_available = true;
  cases.push_back(native_memory_then_decoded_load);

  CpuCompareCase decoded_then_native_fast_load{};
  decoded_then_native_fast_load.name =
      "decoded_load_then_native_ram_fast_load";
  decoded_then_native_fast_load.initial_gpr[1] = 0x80011420u;
  decoded_then_native_fast_load.initial_gpr[2] = 0x11111111u;
  decoded_then_native_fast_load.initial_gpr[3] = 0x33333333u;
  decoded_then_native_fast_load.memory.push_back(
      {0x00011420u, 0x22222222u});
  decoded_then_native_fast_load.memory.push_back(
      {0x00011424u, 0x44444444u});
  decoded_then_native_fast_load.program.assign(17u, 0u);
  decoded_then_native_fast_load.program[0] = enc_i(0x23, 1, 2, 0);
  decoded_then_native_fast_load.program[1] = enc_i(0x23, 1, 3, 4);
  decoded_then_native_fast_load.program[2] =
      enc_r(2, 0, 4, 0, 0x21);
  decoded_then_native_fast_load.instructions = 17;
  decoded_then_native_fast_load.segment_instructions = {1u, 16u};
  decoded_then_native_fast_load.segment_native_tiers = {
      {false, true, true}, {true, true, true}};
  decoded_then_native_fast_load.compare_segment_states = true;
  decoded_then_native_fast_load.enable_ram_load_fastpath_for_x64 = true;
  decoded_then_native_fast_load
      .require_native_ram_load_fastpath_when_available = true;
  decoded_then_native_fast_load
      .require_native_helper_load_delay_entry_when_available = true;
  decoded_then_native_fast_load
      .require_native_memory_tier_entry_when_available = true;
  cases.push_back(decoded_then_native_fast_load);

  CpuCompareCase decoded_then_native_memory_load{};
  decoded_then_native_memory_load.name =
      "decoded_load_then_native_memory_helper";
  decoded_then_native_memory_load.initial_gpr[1] = 0x800111B0u;
  decoded_then_native_memory_load.initial_gpr[2] = 0x11111111u;
  decoded_then_native_memory_load.memory.push_back(
      {0x000111B0u, 0x22222222u});
  decoded_then_native_memory_load.compare_memory_addresses.push_back(
      0x000111B4u);
  decoded_then_native_memory_load.program.assign(17u, 0u);
  decoded_then_native_memory_load.program[0] = enc_i(0x23, 1, 2, 0);
  decoded_then_native_memory_load.program[1] = enc_i(0x2B, 1, 2, 4);
  decoded_then_native_memory_load.program[3] =
      enc_r(2, 0, 3, 0, 0x21);
  decoded_then_native_memory_load.instructions = 17;
  decoded_then_native_memory_load.segment_instructions = {1u, 16u};
  decoded_then_native_memory_load.segment_native_tiers = {
      {false, true, true}, {true, true, true}};
  decoded_then_native_memory_load.compare_segment_states = true;
  decoded_then_native_memory_load.require_native_memory_helper_when_available =
      true;
  decoded_then_native_memory_load
      .require_native_memory_tier_entry_when_available = true;
  cases.push_back(decoded_then_native_memory_load);

  CpuCompareCase native_alu_then_memory{};
  native_alu_then_memory.name = "native_alu_then_native_memory_block";
  native_alu_then_memory.initial_gpr[1] = 0x800111C0u;
  native_alu_then_memory.program.assign(32u, 0u);
  native_alu_then_memory.program[0] = enc_i(0x09, 0, 2, 5);
  native_alu_then_memory.program[16] = enc_i(0x2B, 1, 2, 0);
  native_alu_then_memory.compare_memory_addresses.push_back(0x000111C0u);
  native_alu_then_memory.instructions = 32;
  native_alu_then_memory.segment_instructions = {16u, 16u};
  native_alu_then_memory.compare_segment_states = true;
  native_alu_then_memory.require_native_memory_helper_when_available = true;
  native_alu_then_memory.require_native_memory_tier_entry_when_available =
      true;
  native_alu_then_memory.require_native_alu_tier_entry_when_available = true;
  cases.push_back(native_alu_then_memory);

  CpuCompareCase native_memory_then_alu{};
  native_memory_then_alu.name = "native_memory_then_native_alu_block";
  native_memory_then_alu.initial_gpr[1] = 0x800111D0u;
  native_memory_then_alu.initial_gpr[2] = 0xA5A5A5A5u;
  native_memory_then_alu.program.assign(32u, 0u);
  native_memory_then_alu.program[0] = enc_i(0x2B, 1, 2, 0);
  native_memory_then_alu.program[16] = enc_i(0x09, 0, 3, 7);
  native_memory_then_alu.compare_memory_addresses.push_back(0x000111D0u);
  native_memory_then_alu.instructions = 32;
  native_memory_then_alu.segment_instructions = {16u, 16u};
  native_memory_then_alu.compare_segment_states = true;
  native_memory_then_alu.require_native_memory_helper_when_available = true;
  native_memory_then_alu.require_native_memory_tier_entry_when_available =
      true;
  native_memory_then_alu.require_native_alu_tier_entry_when_available = true;
  cases.push_back(native_memory_then_alu);

  CpuCompareCase jal{};
  jal.name = "jal_link_delay";
  jal.program = {
      enc_j(0x03, kCpuComparePc + 0x10u),
      enc_i(0x09, 0, 5, 0x0055),
      enc_i(0x09, 0, 6, 0x0066),
      0,
      enc_r(31, 0, 7, 0, 0x21),
      enc_i(0x09, 0, 8, 0x0088),
  };
  jal.instructions = 4;
  jal.expect_x64_fallback = true;
  cases.push_back(jal);

  CpuCompareCase jalr{};
  jalr.name = "jalr_link_delay";
  jalr.initial_gpr[8] = kCpuComparePc + 0x10u;
  jalr.program = {
      enc_r(8, 0, 9, 0, 0x09),
      enc_i(0x09, 0, 5, 0x0055),
      enc_i(0x09, 0, 6, 0x0066),
      0,
      enc_r(9, 0, 10, 0, 0x21),
      enc_i(0x09, 0, 11, 0x0077),
  };
  jalr.instructions = 4;
  jalr.expect_x64_fallback = true;
  cases.push_back(jalr);

  CpuCompareCase load_delay{};
  load_delay.name = "load_delay_lw";
  load_delay.initial_gpr[2] = 0x11111111u;
  load_delay.program = {
      enc_i(0x0F, 0, 1, 0x8001),
      enc_i(0x23, 1, 2, 0x1000),
      enc_r(2, 0, 3, 0, 0x21),
      enc_r(2, 0, 4, 0, 0x21),
  };
  load_delay.memory.push_back({0x00011000u, 0x12345678u});
  load_delay.require_full_native_when_available = true;
  load_delay.require_native_memory_helper_when_available = true;
  pad_cpu_compare_program(load_delay);
  cases.push_back(load_delay);

  CpuCompareCase ram_fast_load_delay = load_delay;
  ram_fast_load_delay.name = "native_ram_fast_load_delay";
  ram_fast_load_delay.enable_ram_load_fastpath_for_x64 = true;
  ram_fast_load_delay.require_native_memory_helper_when_available = false;
  ram_fast_load_delay.require_native_ram_load_fastpath_when_available = true;
  cases.push_back(ram_fast_load_delay);

  CpuCompareCase load_entry_alu{};
  load_entry_alu.name = "native_load_delay_entry_alu_then_memory";
  load_entry_alu.initial_gpr[1] = 0x80011080u;
  load_entry_alu.initial_gpr[2] = 0x11111111u;
  load_entry_alu.program = {
      enc_i(0x23, 1, 2, 0),
      enc_r(2, 0, 3, 0, 0x21),
      enc_i(0x2B, 1, 3, 4),
      enc_i(0x23, 1, 4, 4),
      0,
  };
  load_entry_alu.memory.push_back({0x00011080u, 0x22222222u});
  load_entry_alu.segment_instructions = {1u, 16u};
  load_entry_alu.require_native_memory_helper_when_available = true;
  load_entry_alu.require_native_helper_load_delay_entry_when_available = true;
  pad_cpu_compare_program(load_entry_alu, 17u);
  cases.push_back(load_entry_alu);

  CpuCompareCase load_entry_memory{};
  load_entry_memory.name = "native_load_delay_entry_memory_then_memory";
  load_entry_memory.initial_gpr[1] = 0x80011090u;
  load_entry_memory.initial_gpr[2] = 0x11111111u;
  load_entry_memory.program = {
      enc_i(0x23, 1, 2, 0),
      enc_i(0x2B, 1, 2, 4),
      enc_i(0x23, 1, 3, 4),
      0,
  };
  load_entry_memory.memory.push_back({0x00011090u, 0x22222222u});
  load_entry_memory.segment_instructions = {1u, 16u};
  load_entry_memory.require_native_memory_helper_when_available = true;
  load_entry_memory.require_native_helper_load_delay_entry_when_available =
      true;
  pad_cpu_compare_program(load_entry_memory, 17u);
  cases.push_back(load_entry_memory);

  CpuCompareCase memory_load_store{};
  memory_load_store.name = "native_memory_load_store";
  memory_load_store.initial_gpr[1] = 0x80011020u;
  memory_load_store.initial_gpr[2] = 0xCAFEBABEu;
  memory_load_store.program = {
      enc_i(0x2B, 1, 2, 0),
      enc_i(0x23, 1, 3, 0),
      0,
  };
  memory_load_store.require_full_native_when_available = true;
  memory_load_store.require_native_memory_helper_when_available = true;
  pad_cpu_compare_program(memory_load_store);
  cases.push_back(memory_load_store);

  CpuCompareCase sign_loads{};
  sign_loads.name = "native_memory_sign_zero_loads";
  sign_loads.initial_gpr[1] = 0x80011030u;
  sign_loads.program = {
      enc_i(0x20, 1, 2, 2),
      enc_i(0x24, 1, 3, 3),
      enc_i(0x21, 1, 4, 2),
      enc_i(0x25, 1, 5, 0),
      0,
  };
  sign_loads.memory.push_back({0x00011030u, 0x80FF7F01u});
  sign_loads.require_full_native_when_available = true;
  sign_loads.require_native_memory_helper_when_available = true;
  pad_cpu_compare_program(sign_loads);
  cases.push_back(sign_loads);

  CpuCompareCase ram_fast_load_widths{};
  ram_fast_load_widths.name = "native_ram_fast_load_widths_sign_zero";
  ram_fast_load_widths.initial_gpr[1] = 0x80011400u;
  ram_fast_load_widths.memory.push_back({0x00011400u, 0x8001FF80u});
  ram_fast_load_widths.program = {
      enc_i(0x20, 1, 2, 0),
      0,
      enc_i(0x24, 1, 3, 1),
      0,
      enc_i(0x21, 1, 4, 0),
      0,
      enc_i(0x25, 1, 5, 2),
      0,
      enc_i(0x23, 1, 6, 0),
      0,
  };
  ram_fast_load_widths.enable_ram_load_fastpath_for_x64 = true;
  ram_fast_load_widths.require_native_ram_load_fastpath_when_available = true;
  ram_fast_load_widths.require_full_native_when_available = true;
  pad_cpu_compare_program(ram_fast_load_widths);
  cases.push_back(ram_fast_load_widths);

  CpuCompareCase stores{};
  stores.name = "native_memory_byte_half_word_stores";
  stores.initial_gpr[1] = 0x80011040u;
  stores.initial_gpr[2] = 0xCAFEBABEu;
  stores.program = {
      enc_i(0x2B, 1, 2, 0),
      enc_i(0x28, 1, 2, 4),
      enc_i(0x29, 1, 2, 6),
      enc_i(0x23, 1, 3, 0),
      enc_i(0x24, 1, 4, 4),
      enc_i(0x25, 1, 5, 6),
      0,
  };
  stores.require_full_native_when_available = true;
  stores.require_native_memory_helper_when_available = true;
  pad_cpu_compare_program(stores);
  cases.push_back(stores);

  CpuCompareCase mixed_memory_alu{};
  mixed_memory_alu.name = "native_memory_mixed_alu_load_delay";
  mixed_memory_alu.initial_gpr[1] = 0x80011060u;
  mixed_memory_alu.program = {
      enc_i(0x09, 0, 2, 1),
      enc_i(0x23, 1, 3, 0),
      enc_r(3, 2, 4, 0, 0x21),
      enc_r(3, 2, 5, 0, 0x21),
      enc_i(0x2B, 1, 5, 4),
      enc_i(0x23, 1, 6, 4),
      0,
      enc_r(6, 2, 7, 0, 0x21),
  };
  mixed_memory_alu.memory.push_back({0x00011060u, 5u});
  mixed_memory_alu.require_full_native_when_available = true;
  mixed_memory_alu.require_native_memory_helper_when_available = true;
  pad_cpu_compare_program(mixed_memory_alu);
  cases.push_back(mixed_memory_alu);

  CpuCompareCase memory_load_alu_same_reg_store{};
  memory_load_alu_same_reg_store.name =
      "native_memory_load_alu_same_reg_cancels_delay";
  memory_load_alu_same_reg_store.initial_gpr[1] = 0x80011220u;
  memory_load_alu_same_reg_store.initial_gpr[2] = 0x11111111u;
  memory_load_alu_same_reg_store.memory.push_back(
      {0x00011220u, 0xA5A5A5A5u});
  memory_load_alu_same_reg_store.compare_memory_addresses.push_back(
      0x00011224u);
  memory_load_alu_same_reg_store.program = {
      enc_i(0x23, 1, 2, 0),
      enc_i(0x09, 0, 2, 7),
      enc_i(0x2B, 1, 2, 4),
  };
  pad_cpu_compare_program(memory_load_alu_same_reg_store);
  memory_load_alu_same_reg_store.instructions = 3u;
  memory_load_alu_same_reg_store.require_native_entry_when_available = true;
  memory_load_alu_same_reg_store
      .require_native_memory_helper_when_available = true;
  memory_load_alu_same_reg_store
      .require_native_memory_tier_entry_when_available = true;
  memory_load_alu_same_reg_store.enable_ram_load_fastpath_for_x64 = true;
  memory_load_alu_same_reg_store
      .require_native_ram_load_fastpath_when_available = true;
  memory_load_alu_same_reg_store.segment_instructions.assign(3u, 1u);
  memory_load_alu_same_reg_store.compare_segment_states = true;
  memory_load_alu_same_reg_store.allow_partial_native_memory_helper = true;
  cases.push_back(memory_load_alu_same_reg_store);

  CpuCompareCase mmio_helper{};
  mmio_helper.name = "native_mmio_safe_helper_store";
  mmio_helper.initial_gpr[1] = 0x1F801080u;
  mmio_helper.initial_gpr[2] = 0u;
  mmio_helper.program = {
      enc_i(0x2B, 1, 2, 0),
      0,
  };
  mmio_helper.require_full_native_when_available = true;
  mmio_helper.require_native_memory_helper_when_available = true;
  pad_cpu_compare_program(mmio_helper);
  cases.push_back(mmio_helper);

  CpuCompareCase cdrom_status_helper{};
  cdrom_status_helper.name = "native_memory_cdrom_status_read";
  cdrom_status_helper.initial_gpr[1] = 0x1F801800u;
  cdrom_status_helper.program = {
      enc_i(0x24, 1, 2, 0),
      0,
      enc_r(2, 0, 3, 0, 0x21),
  };
  cdrom_status_helper.require_full_native_when_available = true;
  cdrom_status_helper.require_native_memory_helper_when_available = true;
  cdrom_status_helper.require_native_mmio_when_available = true;
  cdrom_status_helper.enable_ram_load_fastpath_for_x64 = true;
  cdrom_status_helper.require_no_native_ram_load_fastpath = true;
  pad_cpu_compare_program(cdrom_status_helper);
  cases.push_back(cdrom_status_helper);

  CpuCompareCase scratchpad_slow_load{};
  scratchpad_slow_load.name = "native_scratchpad_load_stays_helper";
  scratchpad_slow_load.initial_gpr[1] = 0x1F800000u;
  scratchpad_slow_load.memory.push_back({0x1F800000u, 0x55667788u});
  scratchpad_slow_load.program = {
      enc_i(0x23, 1, 2, 0),
      0,
  };
  scratchpad_slow_load.enable_ram_load_fastpath_for_x64 = true;
  scratchpad_slow_load.require_no_native_ram_load_fastpath = true;
  scratchpad_slow_load.require_native_memory_helper_when_available = true;
  scratchpad_slow_load.require_full_native_when_available = true;
  pad_cpu_compare_program(scratchpad_slow_load);
  cases.push_back(scratchpad_slow_load);

  CpuCompareCase dma_status_helper{};
  dma_status_helper.name = "native_memory_dma_status_read_write";
  dma_status_helper.initial_gpr[1] = 0x1F801080u;
  dma_status_helper.program = {
      enc_i(0x23, 1, 2, 0x70),
      0,
      enc_i(0x2B, 1, 2, 0x70),
  };
  dma_status_helper.require_full_native_when_available = true;
  dma_status_helper.require_native_memory_helper_when_available = true;
  dma_status_helper.require_native_mmio_when_available = true;
  pad_cpu_compare_program(dma_status_helper);
  cases.push_back(dma_status_helper);

  CpuCompareCase syscall_exception{};
  syscall_exception.name = "exception_syscall";
  syscall_exception.program = {
      enc_i(0x09, 0, 1, 5),
      0x0000000Cu,
      enc_i(0x09, 0, 2, 6),
  };
  syscall_exception.instructions = 2;
  syscall_exception.expect_x64_fallback = true;
  cases.push_back(syscall_exception);

  CpuCompareCase break_exception{};
  break_exception.name = "exception_break";
  break_exception.program = {
      enc_i(0x09, 0, 1, 5),
      0x0000000Du,
      enc_i(0x09, 0, 2, 6),
  };
  break_exception.instructions = 2;
  break_exception.expect_x64_fallback = true;
  cases.push_back(break_exception);

  CpuCompareCase unaligned_lw{};
  unaligned_lw.name = "exception_unaligned_lw";
  unaligned_lw.initial_gpr[1] = 0x80011002u;
  unaligned_lw.program = {
      enc_i(0x23, 1, 2, 0),
      enc_i(0x09, 0, 3, 3),
  };
  unaligned_lw.require_native_entry_when_available = true;
  unaligned_lw.require_native_memory_helper_when_available = true;
  unaligned_lw.require_native_memory_exception_when_available = true;
  unaligned_lw.enable_ram_load_fastpath_for_x64 = true;
  unaligned_lw.require_no_native_ram_load_fastpath = true;
  pad_cpu_compare_program(unaligned_lw);
  cases.push_back(unaligned_lw);

  CpuCompareCase unaligned_lh{};
  unaligned_lh.name = "exception_unaligned_lh";
  unaligned_lh.initial_gpr[1] = 0x80011001u;
  unaligned_lh.program = {
      enc_i(0x21, 1, 2, 0),
      enc_i(0x09, 0, 3, 3),
  };
  unaligned_lh.require_native_entry_when_available = true;
  unaligned_lh.require_native_memory_helper_when_available = true;
  unaligned_lh.require_native_memory_exception_when_available = true;
  unaligned_lh.enable_ram_load_fastpath_for_x64 = true;
  unaligned_lh.require_no_native_ram_load_fastpath = true;
  pad_cpu_compare_program(unaligned_lh);
  cases.push_back(unaligned_lh);

  CpuCompareCase unaligned_sw{};
  unaligned_sw.name = "exception_unaligned_sw";
  unaligned_sw.initial_gpr[1] = 0x80011002u;
  unaligned_sw.initial_gpr[2] = 0x12345678u;
  unaligned_sw.program = {
      enc_i(0x2B, 1, 2, 0),
      enc_i(0x09, 0, 3, 3),
  };
  unaligned_sw.require_native_entry_when_available = true;
  unaligned_sw.require_native_memory_helper_when_available = true;
  unaligned_sw.require_native_memory_exception_when_available = true;
  pad_cpu_compare_program(unaligned_sw);
  cases.push_back(unaligned_sw);

  CpuCompareCase unaligned_sh{};
  unaligned_sh.name = "exception_unaligned_sh";
  unaligned_sh.initial_gpr[1] = 0x80011001u;
  unaligned_sh.initial_gpr[2] = 0x12345678u;
  unaligned_sh.program = {
      enc_i(0x29, 1, 2, 0),
      enc_i(0x09, 0, 3, 3),
  };
  unaligned_sh.require_native_entry_when_available = true;
  unaligned_sh.require_native_memory_helper_when_available = true;
  unaligned_sh.require_native_memory_exception_when_available = true;
  pad_cpu_compare_program(unaligned_sh);
  cases.push_back(unaligned_sh);

  CpuCompareCase cop0{};
  cop0.name = "unsafe_cop0_fallback";
  cop0.program = {
      (0x10u << 26) | (0u << 21) | (2u << 16) | (12u << 11),
      0,
  };
  cop0.instructions = 2;
  cop0.expect_x64_fallback = true;
  cases.push_back(cop0);

  CpuCompareCase cop2{};
  cop2.name = "unsafe_cop2_gte_fallback";
  cop2.program = {
      (0x12u << 26) | (0u << 21) | (2u << 16) | (0u << 11),
      0,
  };
  cop2.instructions = 2;
  cop2.expect_x64_fallback = true;
  cases.push_back(cop2);

  CpuCompareCase unsupported_strict{};
  unsupported_strict.name = "unsafe_unsupported_opcode_exception";
  unsupported_strict.program = {
      0xFC000000u,
      enc_i(0x09, 0, 2, 2),
  };
  unsupported_strict.instructions = 1;
  unsupported_strict.expect_x64_fallback = true;
  cases.push_back(unsupported_strict);

  CpuCompareCase unknown_primary{};
  unknown_primary.name = "unknown_primary_fallback_nop";
  unknown_primary.program = {
      0xFC000000u,
      enc_i(0x09, 0, 2, 2),
  };
  unknown_primary.instructions = 2;
  unknown_primary.experimental_unknown_fallback = true;
  unknown_primary.expect_x64_fallback = true;
  cases.push_back(unknown_primary);

  CpuCompareCase unknown_special{};
  unknown_special.name = "unknown_special_fallback_rd_zero";
  unknown_special.initial_gpr[5] = 0x12345678u;
  unknown_special.program = {
      enc_r(0, 0, 5, 0, 0x3F),
      enc_i(0x09, 0, 6, 6),
  };
  unknown_special.instructions = 2;
  unknown_special.experimental_unknown_fallback = true;
  unknown_special.expect_x64_fallback = true;
  cases.push_back(unknown_special);

  CpuCompareCase ram_invalidation{};
  ram_invalidation.name = "ram_code_invalidation";
  ram_invalidation.program = {
      enc_i(0x09, 0, 1, 1),
      enc_i(0x09, 0, 2, 2),
      enc_i(0x09, 0, 3, 3),
  };
  ram_invalidation.mutations.push_back(
      {1u, kCpuComparePc + 4u, enc_i(0x09, 0, 2, 0x0022)});
  ram_invalidation.instructions = 3;
  cases.push_back(ram_invalidation);

  return cases;
}

static int run_cpu_backend_compare_test(bool memory_only = false) {
  LOG_INFO("=== CPU Backend Compare Test ===");
  const bool saved_override = g_cpu_execution_mode_cli_override;
  const CpuExecutionMode saved_override_value = g_cpu_execution_mode_cli_value;
  const bool saved_unknown_fallback =
      g_experimental_unhandled_special_returns_zero;
  const bool saved_force_native = g_cpu_x64_jit_force_compile;
  const bool saved_branch_tail_enabled = g_cpu_x64_jit_branch_tail_enabled;
  const bool saved_branch_tail_cli_override =
      g_cpu_x64_jit_branch_tail_cli_override;
  const bool saved_branch_tail_cli_value =
      g_cpu_x64_jit_branch_tail_cli_value;
  const std::vector<u32> saved_branch_tail_blacklist =
      g_cpu_x64_jit_branch_tail_blacklist;
  const bool saved_compare_irq_on_branch =
      g_cpu_backend_compare_irq_on_branch;
  const bool saved_compare_partial_branch_tail =
      g_cpu_backend_compare_allow_partial_branch_tail;
  const bool saved_compare_partial_memory =
      g_cpu_backend_compare_allow_partial_memory_helper;
  const bool saved_compare_test_active = g_cpu_backend_compare_test_active;
  const bool saved_all_native_cli_override =
      g_cpu_x64_jit_all_native_cli_override;
  const bool saved_all_native_cli_value =
      g_cpu_x64_jit_all_native_cli_value;
  const bool saved_memory_native_cli_override =
      g_cpu_x64_jit_native_memory_cli_override;
  const bool saved_memory_native_cli_value =
      g_cpu_x64_jit_native_memory_cli_value;
  const bool saved_alu_native_cli_override =
      g_cpu_x64_jit_native_alu_cli_override;
  const bool saved_alu_native_cli_value =
      g_cpu_x64_jit_native_alu_cli_value;
  const bool saved_ram_load_fastpath =
      g_cpu_x64_jit_ram_load_fastpath_enabled;
  g_cpu_backend_compare_test_active = true;
  g_cpu_x64_jit_force_compile = true;

  int failures = 0;
  const std::array<CpuExecutionMode, 3> modes = {
      CpuExecutionMode::Interpreter,
      CpuExecutionMode::DecodedBlockInterpreter,
      CpuExecutionMode::X64Jit,
  };

  for (const CpuCompareCase &test_case : make_cpu_compare_cases()) {
    if (memory_only) {
      const std::string_view name(test_case.name);
      const bool memory_case =
          test_case.require_native_memory_helper_when_available ||
          test_case.require_native_memory_exception_when_available ||
          test_case.require_native_memory_tier_entry_when_available ||
          test_case.disable_memory_native_for_x64 ||
          name.find("memory") != std::string_view::npos ||
          name.find("mmio") != std::string_view::npos;
      if (!memory_case) {
        continue;
      }
    }
    g_experimental_unhandled_special_returns_zero =
        test_case.experimental_unknown_fallback;

    CpuCompareRunResult reference =
        run_cpu_compare_case_once(test_case, CpuExecutionMode::Interpreter);

    for (CpuExecutionMode mode : modes) {
      CpuCompareRunResult result =
          (mode == CpuExecutionMode::Interpreter)
              ? reference
              : run_cpu_compare_case_once(test_case, mode);

      bool pass = cpu_debug_states_equal(reference.state, result.state);
      const bool state_pass = pass;
      bool segment_state_pass = true;
      bool segment_peripheral_pass = true;
      if (test_case.compare_segment_states) {
        segment_state_pass =
            reference.segment_states.size() == result.segment_states.size();
        const size_t segment_count = std::min(reference.segment_states.size(),
                                              result.segment_states.size());
        for (size_t segment = 0; segment < segment_count; ++segment) {
          if (!cpu_debug_states_equal(reference.segment_states[segment],
                                      result.segment_states[segment])) {
            segment_state_pass = false;
            log_cpu_debug_state_diff(
                test_case.name, cpu_compare_mode_name(mode),
                reference.segment_states[segment],
                result.segment_states[segment]);
          }
        }
        segment_peripheral_pass =
            reference.segment_peripherals.size() ==
            result.segment_peripherals.size();
        const size_t peripheral_segment_count =
            std::min(reference.segment_peripherals.size(),
                     result.segment_peripherals.size());
        for (size_t segment = 0; segment < peripheral_segment_count;
             ++segment) {
          if (!cpu_compare_peripherals_equal(
                  reference.segment_peripherals[segment],
                  result.segment_peripherals[segment])) {
            segment_peripheral_pass = false;
          }
        }
      }
      const bool irq_state_pass =
          reference.irq_stat == result.irq_stat &&
          reference.irq_mask == result.irq_mask;
      const bool memory_state_pass =
          reference.memory_values == result.memory_values;
      const bool peripheral_state_pass = cpu_compare_peripherals_equal(
          reference.peripherals, result.peripherals);
      const bool expected_state_pass =
          cpu_compare_expected_state_pass(test_case, mode, result.state);
      bool native_check_pass = true;
      const char *native_check = "not_required";

      if (mode == CpuExecutionMode::X64Jit) {
        if (test_case.require_full_native_when_available) {
          if (!result.stats.native_available) {
            native_check = "skip_native_unavailable";
          } else {
            const bool fully_native =
                result.stats.native_blocks_compiled != 0 &&
                result.stats.native_block_entries != 0 &&
                result.stats.native_instructions >= test_case.instructions &&
                result.stats.native_code_bytes != 0 &&
                result.stats.decoded_instructions == 0 &&
                result.stats.fallback_instructions == 0 &&
                result.stats.interpreter_fallback_steps == 0;
            native_check = fully_native ? "native_full" : "native_missing";
            native_check_pass = fully_native;
          }
        } else if (test_case.require_native_entry_when_available) {
          if (!result.stats.native_available) {
            native_check = "skip_native_unavailable";
          } else {
            const bool allow_post_exception_interpreter =
                test_case.require_native_memory_exception_when_available;
            const bool native_entered =
                result.stats.native_blocks_compiled != 0 &&
                result.stats.native_block_entries != 0 &&
                result.stats.native_instructions != 0 &&
                result.stats.native_code_bytes != 0 &&
                result.stats.decoded_instructions == 0;
            const bool fallback_ok =
                allow_post_exception_interpreter ||
                (result.stats.fallback_instructions == 0 &&
                 result.stats.interpreter_fallback_steps == 0);
            native_check = native_entered ? "native_entered"
                                          : "native_missing";
            native_check_pass = native_entered && fallback_ok;
            if (native_entered && !fallback_ok) {
              native_check = "unexpected_fallback";
            }
          }
        } else if (test_case.expect_x64_fallback) {
          if (!result.stats.native_available) {
            native_check = "skip_native_unavailable";
          } else {
            const bool clean_fallback =
                result.stats.native_block_entries == 0 &&
                result.stats.native_instructions == 0;
            native_check = clean_fallback ? "clean_fallback"
                                          : "unexpected_native";
            native_check_pass = clean_fallback;
          }
        }

        if (native_check_pass && result.stats.native_available &&
            test_case.require_native_memory_helper_when_available &&
            result.stats.native_memory_helper_calls == 0) {
          native_check = "native_memory_helper_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_native_memory_exception_when_available &&
            result.stats.native_memory_exception_exits == 0) {
          native_check = "native_memory_exception_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_native_helper_load_delay_entry_when_available) {
          const bool load_delay_native =
              result.stats.native_helper_load_delay_entries != 0 &&
              result.stats.native_helper_load_delay_passes != 0 &&
              result.stats.native_block_entries != 0 &&
              result.stats.native_instructions != 0;
          native_check = load_delay_native ? "native_load_delay_entry"
                                           : "native_load_delay_entry_missing";
          native_check_pass = load_delay_native;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_native_branch_tail_when_available) {
          const u64 expected_branch_count =
              test_case.native_branch_should_be_taken
                  ? result.stats.native_branch_taken
                  : result.stats.native_branch_not_taken;
          u64 expected_opcode_entries = 1u;
          u64 expected_opcode_outcomes = 1u;
          if (test_case.native_branch_primary_op == 0x07u) {
            expected_opcode_entries =
                result.stats.native_branch_tail_bgtz_entries;
            expected_opcode_outcomes =
                test_case.native_branch_should_be_taken
                    ? result.stats.native_branch_tail_bgtz_taken
                    : result.stats.native_branch_tail_bgtz_not_taken;
          } else if (test_case.native_branch_primary_op == 0x06u) {
            expected_opcode_entries =
                result.stats.native_branch_tail_blez_entries;
            expected_opcode_outcomes =
                test_case.native_branch_should_be_taken
                    ? result.stats.native_branch_tail_blez_taken
                    : result.stats.native_branch_tail_blez_not_taken;
          }
          const bool branch_tail_native =
              result.stats.native_branch_tail_blocks_compiled != 0 &&
              result.stats.native_branch_tail_entries != 0 &&
              expected_branch_count != 0 && expected_opcode_entries != 0 &&
              expected_opcode_outcomes != 0;
          native_check = branch_tail_native ? "native_branch_tail"
                                             : "native_branch_tail_missing";
          native_check_pass = branch_tail_native;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case
                .require_native_branch_delay_memory_helper_when_available &&
            result.stats.native_branch_delay_slot_memory_helpers == 0) {
          native_check = "native_branch_delay_memory_helper_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_native_mmio_when_available &&
            result.stats.mmio_accesses == 0) {
          native_check = "native_mmio_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_branch_tail_disabled_fallback_when_available &&
            result.stats.native_branch_tail_disabled_fallbacks == 0) {
          native_check = "branch_tail_disabled_fallback_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_branch_tail_blacklisted_fallback_when_available &&
            result.stats.native_branch_tail_blacklisted_fallbacks == 0) {
          native_check = "branch_tail_blacklist_fallback_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_all_native_disabled_fallback_when_available &&
            result.stats.native_all_disabled_fallbacks == 0) {
          native_check = "all_native_disabled_fallback_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_memory_native_disabled_fallback_when_available &&
            result.stats.native_memory_disabled_fallbacks == 0) {
          native_check = "memory_native_disabled_fallback_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_alu_native_disabled_fallback_when_available &&
            result.stats.native_alu_disabled_fallbacks == 0) {
          native_check = "alu_native_disabled_fallback_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_native_memory_tier_entry_when_available &&
            result.stats.native_memory_block_entries == 0) {
          native_check = "native_memory_tier_entry_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_native_alu_tier_entry_when_available &&
            result.stats.native_alu_block_entries == 0) {
          native_check = "native_alu_tier_entry_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_native_ram_load_fastpath_when_available &&
            result.stats.native_memory_fastpath_loads == 0) {
          native_check = "native_ram_load_fastpath_missing";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            test_case.require_no_native_ram_load_fastpath &&
            result.stats.native_memory_fastpath_loads != 0) {
          native_check = "unexpected_native_ram_load_fastpath";
          native_check_pass = false;
        }
        if (native_check_pass && result.stats.native_available &&
            result.stats.native_memory_fastpath_mmio_loads != 0) {
          native_check = "native_mmio_fastpath_forbidden";
          native_check_pass = false;
        }
      }

      pass = pass && segment_state_pass && segment_peripheral_pass &&
             irq_state_pass && peripheral_state_pass && memory_state_pass &&
             expected_state_pass && native_check_pass;
      const char *outcome = cpu_compare_outcome(mode, result.stats);
      LOG_INFO(
          "CPU_COMPARE name=%s mode=%s result=%s outcome=%s native_check=%s pc=0x%08X next_pc=0x%08X current_pc=0x%08X instr=%u cycles=%llu decoded_instr=%llu native_instr=%llu fallback_instr=%llu native_mem_helpers=%llu native_mem_exits=%llu helper_ld_entries=%llu helper_ld_passes=%llu helper_ld_fallbacks=%llu forced_reason=%s forced_slices=%llu forced_instr=%llu native_blocks=%llu native_attempts=%llu native_successes=%llu native_compiled=%llu native_entries=%llu native_code_bytes=%llu native_available=%u",
          test_case.name, cpu_compare_mode_name(mode),
          pass ? "PASS" : "FAIL", outcome, native_check, result.state.pc,
          result.state.next_pc, result.state.current_pc, result.run.instructions,
          static_cast<unsigned long long>(result.state.cycles),
          static_cast<unsigned long long>(result.stats.decoded_instructions),
          static_cast<unsigned long long>(result.stats.native_instructions),
          static_cast<unsigned long long>(result.stats.fallback_instructions),
          static_cast<unsigned long long>(
              result.stats.native_memory_helper_calls),
          static_cast<unsigned long long>(
              result.stats.native_memory_exception_exits),
          static_cast<unsigned long long>(
              result.stats.native_helper_load_delay_entries),
          static_cast<unsigned long long>(
              result.stats.native_helper_load_delay_passes),
          static_cast<unsigned long long>(
              result.stats.native_helper_load_delay_fallbacks),
          cpu_forced_interpreter_reason_name(
              result.stats.forced_interpreter_last_reason),
          static_cast<unsigned long long>(
              result.stats.forced_interpreter_slices),
          static_cast<unsigned long long>(
              result.stats.forced_interpreter_instructions),
          static_cast<unsigned long long>(result.stats.native_blocks),
          static_cast<unsigned long long>(result.stats.native_compile_attempts),
          static_cast<unsigned long long>(result.stats.native_compile_successes),
          static_cast<unsigned long long>(result.stats.native_blocks_compiled),
          static_cast<unsigned long long>(result.stats.native_block_entries),
          static_cast<unsigned long long>(result.stats.native_code_bytes),
          result.stats.native_available ? 1u : 0u);

      if (mode == CpuExecutionMode::X64Jit &&
          (test_case.require_full_native_when_available ||
           test_case.require_native_entry_when_available ||
           test_case.require_native_memory_helper_when_available ||
           test_case.require_native_memory_exception_when_available ||
           test_case.require_native_helper_load_delay_entry_when_available ||
           test_case.require_native_branch_tail_when_available ||
           test_case
               .require_native_branch_delay_memory_helper_when_available)) {
        LOG_INFO(
            "CPU_COMPARE_NATIVE name=%s required=1 available=%u compiled=%u entered=%u native_instr=%llu decoded_instr=%llu fallback_instr=%llu native_mem_helpers=%llu native_mem_exits=%llu helper_ld_entries=%llu helper_ld_passes=%llu helper_ld_fallbacks=%llu code_bytes=%llu attempts=%llu successes=%llu force=%u hot_threshold=%u min_block=%u",
            test_case.name, result.stats.native_available ? 1u : 0u,
            result.stats.native_blocks_compiled != 0 ? 1u : 0u,
            result.stats.native_block_entries != 0 ? 1u : 0u,
            static_cast<unsigned long long>(result.stats.native_instructions),
            static_cast<unsigned long long>(result.stats.decoded_instructions),
            static_cast<unsigned long long>(result.stats.fallback_instructions),
            static_cast<unsigned long long>(
                result.stats.native_memory_helper_calls),
            static_cast<unsigned long long>(
                result.stats.native_memory_exception_exits),
            static_cast<unsigned long long>(
                result.stats.native_helper_load_delay_entries),
            static_cast<unsigned long long>(
                result.stats.native_helper_load_delay_passes),
            static_cast<unsigned long long>(
                result.stats.native_helper_load_delay_fallbacks),
            static_cast<unsigned long long>(result.stats.native_code_bytes),
            static_cast<unsigned long long>(result.stats.native_compile_attempts),
            static_cast<unsigned long long>(
                result.stats.native_compile_successes),
            g_cpu_x64_jit_force_compile ? 1u : 0u,
            g_cpu_x64_jit_hot_block_threshold,
            g_cpu_x64_jit_min_block_instructions);
      }

      if (mode == CpuExecutionMode::X64Jit &&
          test_case.require_native_branch_tail_when_available) {
        LOG_INFO(
            "CPU_COMPARE_BRANCH_TAIL name=%s blocks_compiled=%llu entries=%llu taken=%llu not_taken=%llu rejects=%llu delay_memory_helpers=%llu",
            test_case.name,
            static_cast<unsigned long long>(
                result.stats.native_branch_tail_blocks_compiled),
            static_cast<unsigned long long>(
                result.stats.native_branch_tail_entries),
            static_cast<unsigned long long>(result.stats.native_branch_taken),
            static_cast<unsigned long long>(
                result.stats.native_branch_not_taken),
            static_cast<unsigned long long>(
                result.stats.native_branch_tail_rejects),
            static_cast<unsigned long long>(
                result.stats.native_branch_delay_slot_memory_helpers));
      }

      if (mode == CpuExecutionMode::X64Jit &&
          (test_case.enable_ram_load_fastpath_for_x64 ||
           test_case.require_native_ram_load_fastpath_when_available ||
           test_case.require_no_native_ram_load_fastpath)) {
        LOG_INFO("CPU_COMPARE_RAM_FASTPATH name=%s enabled=%u fast_loads=%llu memory_helpers=%llu ram_helpers=%llu scratch_helpers=%llu bios_helpers=%llu mmio_helpers=%llu unknown_helpers=%llu unaligned_helpers=%llu mmio_fast_loads=%llu",
                 test_case.name,
                 test_case.enable_ram_load_fastpath_for_x64 ? 1u : 0u,
                 static_cast<unsigned long long>(
                     result.stats.native_memory_fastpath_loads),
                 static_cast<unsigned long long>(
                     result.stats.native_memory_helper_calls),
                 static_cast<unsigned long long>(
                     result.stats.native_memory_helper_ram_calls),
                 static_cast<unsigned long long>(
                     result.stats.native_memory_helper_scratchpad_calls),
                 static_cast<unsigned long long>(
                     result.stats.native_memory_helper_bios_calls),
                 static_cast<unsigned long long>(
                     result.stats.native_memory_helper_mmio_calls),
                 static_cast<unsigned long long>(
                     result.stats.native_memory_helper_unknown_calls),
                 static_cast<unsigned long long>(
                     result.stats.native_memory_helper_unaligned_calls),
                 static_cast<unsigned long long>(
                     result.stats.native_memory_fastpath_mmio_loads));
      }

      if (!pass) {
        ++failures;
        if (!state_pass) {
          log_cpu_debug_state_diff(test_case.name, cpu_compare_mode_name(mode),
                                   reference.state, result.state);
        }
        if (!irq_state_pass) {
          LOG_ERROR("CPU_COMPARE_IRQ_DIFF name=%s mode=%s reference_stat=0x%08X actual_stat=0x%08X reference_mask=0x%08X actual_mask=0x%08X",
                    test_case.name, cpu_compare_mode_name(mode),
                    reference.irq_stat, result.irq_stat,
                    reference.irq_mask, result.irq_mask);
        }
        if (!memory_state_pass) {
          LOG_ERROR("CPU_COMPARE_MEMORY_DIFF name=%s mode=%s reference_count=%zu actual_count=%zu",
                    test_case.name, cpu_compare_mode_name(mode),
                    reference.memory_values.size(),
                    result.memory_values.size());
        }
        if (!peripheral_state_pass || !segment_peripheral_pass) {
          LOG_ERROR("CPU_COMPARE_PERIPHERAL_DIFF name=%s mode=%s final=%u segment=%u reference_dma=0x%08X/0x%08X actual_dma=0x%08X/0x%08X reference_cd=%llu/%d/%d actual_cd=%llu/%d/%d",
                    test_case.name, cpu_compare_mode_name(mode),
                    peripheral_state_pass ? 1u : 0u,
                    segment_peripheral_pass ? 1u : 0u,
                    reference.peripherals.dma_dpcr,
                    reference.peripherals.dma_dicr,
                    result.peripherals.dma_dpcr,
                    result.peripherals.dma_dicr,
                    static_cast<unsigned long long>(
                        reference.peripherals.cd_sector_count),
                    reference.peripherals.cd_read_lba,
                    reference.peripherals.cd_active_lba,
                    static_cast<unsigned long long>(
                        result.peripherals.cd_sector_count),
                    result.peripherals.cd_read_lba,
                    result.peripherals.cd_active_lba);
        }
        if (!native_check_pass) {
          LOG_ERROR(
              "CPU_COMPARE_BACKEND_DIFF name=%s mode=%s expected=%s outcome=%s native_available=%u native_blocks_compiled=%llu native_entries=%llu native_instr=%llu native_mem_helpers=%llu native_mem_exits=%llu helper_ld_entries=%llu helper_ld_passes=%llu helper_ld_fallbacks=%llu native_code_bytes=%llu decoded_instr=%llu fallback_instr=%llu interpreter_fallback_steps=%llu",
              test_case.name, cpu_compare_mode_name(mode), native_check,
              outcome, result.stats.native_available ? 1u : 0u,
              static_cast<unsigned long long>(
                  result.stats.native_blocks_compiled),
              static_cast<unsigned long long>(
                  result.stats.native_block_entries),
              static_cast<unsigned long long>(result.stats.native_instructions),
              static_cast<unsigned long long>(
                  result.stats.native_memory_helper_calls),
              static_cast<unsigned long long>(
                  result.stats.native_memory_exception_exits),
              static_cast<unsigned long long>(
                  result.stats.native_helper_load_delay_entries),
              static_cast<unsigned long long>(
                  result.stats.native_helper_load_delay_passes),
              static_cast<unsigned long long>(
                  result.stats.native_helper_load_delay_fallbacks),
              static_cast<unsigned long long>(result.stats.native_code_bytes),
              static_cast<unsigned long long>(result.stats.decoded_instructions),
              static_cast<unsigned long long>(result.stats.fallback_instructions),
              static_cast<unsigned long long>(
                  result.stats.interpreter_fallback_steps));
        }
        log_cpu_compare_program(test_case);
      }
    }
  }

  g_experimental_unhandled_special_returns_zero = saved_unknown_fallback;
  g_cpu_x64_jit_force_compile = saved_force_native;
  g_cpu_x64_jit_branch_tail_enabled = saved_branch_tail_enabled;
  g_cpu_x64_jit_branch_tail_cli_override =
      saved_branch_tail_cli_override;
  g_cpu_x64_jit_branch_tail_cli_value = saved_branch_tail_cli_value;
  g_cpu_x64_jit_branch_tail_blacklist = saved_branch_tail_blacklist;
  g_cpu_backend_compare_irq_on_branch = saved_compare_irq_on_branch;
  g_cpu_backend_compare_allow_partial_branch_tail =
      saved_compare_partial_branch_tail;
  g_cpu_backend_compare_allow_partial_memory_helper =
      saved_compare_partial_memory;
  g_cpu_x64_jit_all_native_cli_override = saved_all_native_cli_override;
  g_cpu_x64_jit_all_native_cli_value = saved_all_native_cli_value;
  g_cpu_x64_jit_native_memory_cli_override =
      saved_memory_native_cli_override;
  g_cpu_x64_jit_native_memory_cli_value = saved_memory_native_cli_value;
  g_cpu_x64_jit_native_alu_cli_override = saved_alu_native_cli_override;
  g_cpu_x64_jit_native_alu_cli_value = saved_alu_native_cli_value;
  g_cpu_x64_jit_ram_load_fastpath_enabled = saved_ram_load_fastpath;
  g_cpu_backend_compare_test_active = saved_compare_test_active;
  g_cpu_execution_mode_cli_override = saved_override;
  g_cpu_execution_mode_cli_value = saved_override_value;

  if (failures != 0) {
    LOG_ERROR("CPU backend compare test failed: %d case(s)", failures);
    return 1;
  }
  LOG_INFO("CPU backend compare test passed");
  return 0;
}
} // namespace

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
  sys->cpu().notify_cpu_backend_frame(0u);

  for (int i = 0; i < frames; ++i) {
    sys->sio().set_button_state(auto_input_buttons_for_frame(i + 1));
    sys->run_frame();
    sys->cpu().notify_cpu_backend_frame(static_cast<u32>(i + 1));
    frame_test_cpu_ms_total += sys->profiling_stats().cpu_ms;
    frame_test_core_ms_total += sys->profiling_stats().total_ms;
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
  LOG_INFO("FRAME_PERF cpu_core_ms=%.3f core_ms=%.3f slowdown_percent=%.2f target_fps=%.3f detailed=%u",
           cpu_core_ms_avg, core_ms_avg, slowdown_percent, target_fps,
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
      g_cpu_x64_jit_ram_load_fastpath_enabled = true;
      continue;
    }
    if (a == "--jit-disable-ram-load-fastpath") {
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
    if (memory_only) {
      g_cpu_x64_jit_memory_trace = true;
    }
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
  printf("  VibeStation - PS1 Emulator v0.5.2\n");
  printf("========================================\n");
  printf("Starting up...\n");
  fflush(stdout);

  App app;
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
