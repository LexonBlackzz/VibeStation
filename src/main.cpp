// ── VibeStation — PS1 Emulator ──────────────────────────────────────
// Entry point

#define SDL_MAIN_HANDLED // Prevent SDL from redefining main()
#include "core/system.h"
#include "ui/app.h"
#include <SDL.h>
#include <cctype>
#include <cstdlib>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <vector>

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
                          const std::string &cue_path = "") {
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
  const bool owns_log = (g_log_file == nullptr);
  if (owns_log) {
    g_log_file = std::fopen("bios_test.log", "w");
  }
  LOG_INFO("=== VibeStation Frame Test ===");
  LOG_INFO("BIOS path: %s", bios_path.c_str());
  LOG_INFO("Frames: %d", frames);

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

  sys->reset();
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

  for (int i = 0; i < frames; ++i) {
    // Diagnostic probe: hold Start pressed (active-low bit3) for boot-test runs.
    sys->sio().set_button_state(static_cast<u16>(0xFFFFu & ~0x0008u));
    sys->run_frame();
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

    if ((i + 1) % 60 == 0) {
      dump_vram_ppm(*sys, i + 1);
    }
    if ((i + 1) <= 10 || ((i + 1) % 30 == 0)) {
      const u32 instr = sys->read32(pc);
      LOG_INFO(
          "frame=%d pc=0x%08X instr=0x%08X cycles=%llu i_stat=0x%08X i_mask=0x%08X",
          i + 1, pc, instr,
          static_cast<unsigned long long>(sys->cpu().cycle_count()),
          sys->irq().stat(), sys->irq().mask());
    }
  }
  const System::BootDiagnostics &diag = sys->boot_diag();
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
      "cdread_count=%llu cd_irq1=%llu cd_irq3=%llu "
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
  LOG_INFO("Frame test complete");
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

  std::string wav_out_path;
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
      ++i;
      continue;
    }
    if (a == "--wav-out" && (i + 1) < args.size()) {
      wav_out_path = args[i + 1];
      ++i;
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
    passthrough.push_back(a);
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
  if (passthrough.size() >= 5 && passthrough[0] == "--boot-disc-test") {
    const int frames = std::max(1, std::atoi(passthrough[2].c_str()));
    const int rc =
        run_boot_disc_test(passthrough[1], frames, passthrough[3], passthrough[4]);
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
                          passthrough[4]);
    } else {
      rc = run_frame_test(passthrough[1], frames);
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
  printf("  VibeStation - PS1 Emulator v0.4.1\n");
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

  printf("Initialization complete! Running...\n");
  fflush(stdout);

  app.run();
  app.shutdown();

  return 0;
}


