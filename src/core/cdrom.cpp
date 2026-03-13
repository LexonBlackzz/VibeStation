#include "cdrom.h"
#include "interrupt.h"
#include "system.h"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <filesystem>
#include <sstream>
#include <unordered_map>

namespace {
constexpr u8 kHintTypeMask = 0x07u;
constexpr u8 kHintMaskAll = 0x1Fu;
constexpr size_t kFifoCapacity = 16u;

std::string trim_copy(const std::string &text) {
  const size_t begin = text.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return {};
  }
  const size_t end = text.find_last_not_of(" \t\r\n");
  return text.substr(begin, end - begin + 1u);
}

std::string upper_copy(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return text;
}

std::string normalize_path_key(std::string path) {
  std::replace(path.begin(), path.end(), '\\', '/');
  return upper_copy(path);
}

u64 file_size_bytes(const std::string &path) {
  std::error_code ec;
  const u64 size = std::filesystem::file_size(std::filesystem::path(path), ec);
  return ec ? 0 : size;
}

bool resolve_track_layout(std::vector<CdTrack> &tracks,
                          const std::string &default_file,
                          bool &single_file, bool &monotonic_abs,
                          std::string &error) {
  if (tracks.empty()) {
    error = "No tracks found in cue";
    return false;
  }

  std::vector<std::string> file_order;
  std::unordered_map<std::string, std::string> file_path;
  std::unordered_map<std::string, int> file_sector_size;
  std::unordered_map<std::string, u64> file_bytes;

  for (CdTrack &track : tracks) {
    if (track.filename.empty()) {
      track.filename = default_file;
    }
    if (track.filename.empty()) {
      error = "Track file path is empty";
      return false;
    }
    if (track.sector_size <= 0) {
      track.sector_size = 2352;
    }

    const std::string key = normalize_path_key(track.filename);
    if (file_path.find(key) == file_path.end()) {
      const u64 bytes = file_size_bytes(track.filename);
      if (bytes == 0) {
        error = "Missing or empty track file: " + track.filename;
        return false;
      }
      file_order.push_back(key);
      file_path[key] = track.filename;
      file_sector_size[key] = track.sector_size;
      file_bytes[key] = bytes;
    }
  }

  std::unordered_map<std::string, int> file_base_abs;
  int next_base_abs = 150;
  for (const std::string &key : file_order) {
    const int sector_size = std::max(1, file_sector_size[key]);
    const int sectors =
        static_cast<int>(file_bytes[key] / static_cast<u64>(sector_size));
    if (sectors <= 0) {
      error = "Track file has no readable sectors: " + file_path[key];
      return false;
    }
    file_base_abs[key] = next_base_abs;
    next_base_abs += sectors;
  }

  single_file = (file_order.size() == 1u);
  monotonic_abs = true;
  int prev_abs = -1;
  for (CdTrack &track : tracks) {
    const std::string key = normalize_path_key(track.filename);
    const int base_abs = file_base_abs[key];
    track.index01_abs_lba = base_abs + std::max(0, track.index01_file_lba);
    track.index01_file_offset =
        static_cast<u64>(std::max(0, track.index01_file_lba)) *
        static_cast<u64>(std::max(1, track.sector_size));
    if (prev_abs > track.index01_abs_lba) {
      monotonic_abs = false;
    }
    prev_abs = track.index01_abs_lba;
  }

  return true;
}

bool starts_with_ci(const std::string &line, const char *prefix) {
  for (size_t i = 0; prefix[i] != '\0'; ++i) {
    if (i >= line.size()) {
      return false;
    }
    const unsigned char a = static_cast<unsigned char>(line[i]);
    const unsigned char b = static_cast<unsigned char>(prefix[i]);
    if (std::tolower(a) != std::tolower(b)) {
      return false;
    }
  }
  return true;
}

std::string parse_file_name(const std::string &line) {
  const size_t q0 = line.find('"');
  if (q0 != std::string::npos) {
    const size_t q1 = line.find('"', q0 + 1u);
    if (q1 != std::string::npos && q1 > q0 + 1u) {
      return line.substr(q0 + 1u, q1 - q0 - 1u);
    }
  }
  std::istringstream iss(line);
  std::string token;
  std::string name;
  iss >> token >> name;
  return name;
}

int parse_msf_text(const std::string &msf) {
  int mm = 0;
  int ss = 0;
  int ff = 0;
  char c0 = '\0';
  char c1 = '\0';
  std::istringstream iss(msf);
  if (!(iss >> mm >> c0 >> ss >> c1 >> ff) || c0 != ':' || c1 != ':') {
    return -1;
  }
  if (mm < 0 || ss < 0 || ff < 0) {
    return -1;
  }
  return mm * 60 * 75 + ss * 75 + ff;
}

bool is_audio_track(const CdTrack *track) {
  return track != nullptr && upper_copy(track->type) == "AUDIO";
}

bool is_mode2_track(const CdTrack *track) {
  if (track == nullptr) {
    return false;
  }
  const std::string type = upper_copy(track->type);
  return starts_with_ci(type, "MODE2/") || type == "CDI/2336";
}

struct SectorLayout {
  bool mode2 = false;
  bool has_subheader = false;
  size_t subheader_offset = 0;
  size_t user_data_offset = 0;
  size_t user_data_size = 0;
};

SectorLayout describe_sector_layout(const std::vector<u8> &raw_sector,
                                    const CdTrack *track) {
  SectorLayout layout{};
  layout.user_data_size = raw_sector.size();

  auto clamp_layout = [&]() {
    if (layout.user_data_offset > raw_sector.size()) {
      layout.user_data_offset = raw_sector.size();
    }
    if (layout.user_data_offset + layout.user_data_size > raw_sector.size()) {
      layout.user_data_size = raw_sector.size() - layout.user_data_offset;
    }
  };

  const bool mode2_track = is_mode2_track(track);
  if (raw_sector.size() >= 2352u) {
    const bool mode2_from_header = raw_sector.size() >= 16u && raw_sector[15] == 0x02u;
    layout.mode2 = mode2_track || mode2_from_header;
    if (layout.mode2) {
      layout.user_data_offset = 24u;
      layout.user_data_size = 0x800u;
      layout.has_subheader = raw_sector.size() >= 24u;
      layout.subheader_offset = 16u;
    } else {
      layout.user_data_offset = 16u;
      layout.user_data_size = 0x800u;
    }
    clamp_layout();
    return layout;
  }

  if (raw_sector.size() >= 2336u && (mode2_track || raw_sector.size() == 2336u)) {
    layout.mode2 = true;
    layout.user_data_offset = 8u;
    layout.user_data_size = 0x800u;
    layout.has_subheader = raw_sector.size() >= 8u;
    layout.subheader_offset = 0u;
    clamp_layout();
    return layout;
  }

  if (raw_sector.size() >= 2048u) {
    layout.mode2 = mode2_track;
    layout.user_data_offset = 0u;
    layout.user_data_size = 0x800u;
    clamp_layout();
    return layout;
  }

  clamp_layout();
  return layout;
}

bool read_sector_subheader(const std::vector<u8> &raw_sector,
                           const SectorLayout &layout, u8 &file, u8 &channel,
                           u8 &submode, u8 &codinginfo) {
  if (!layout.has_subheader || layout.subheader_offset + 4u > raw_sector.size()) {
    return false;
  }
  file = raw_sector[layout.subheader_offset + 0u];
  channel = raw_sector[layout.subheader_offset + 1u];
  submode = raw_sector[layout.subheader_offset + 2u];
  codinginfo = raw_sector[layout.subheader_offset + 3u];
  return true;
}

u8 clip_u8(int value) {
  return static_cast<u8>(std::clamp(value, 0, 255));
}

u8 to_bcd8(u8 value) { return static_cast<u8>(((value / 10) << 4) | (value % 10)); }

s16 sat16(s32 value) {
  if (value < -32768) {
    return -32768;
  }
  if (value > 32767) {
    return 32767;
  }
  return static_cast<s16>(value);
}

u8 irq_to_hint(u8 irq) { return static_cast<u8>(irq & kHintTypeMask); }

u8 hint_to_mask(u8 hint_type) {
  if (hint_type >= 1u && hint_type <= 5u) {
    return static_cast<u8>(1u << (hint_type - 1u));
  }
  return 0u;
}

u16 read_le16(const std::vector<u8> &data, size_t offset) {
  if (offset + 1u >= data.size()) {
    return 0;
  }
  return static_cast<u16>(static_cast<u16>(data[offset]) |
                          (static_cast<u16>(data[offset + 1u]) << 8));
}

bool looks_like_str_header_at(const std::vector<u8> &raw_sector,
                              size_t user_offset) {
  if (user_offset + 8u > raw_sector.size()) {
    return false;
  }

  // Standard STR stream header starts with 0160h at user-data offset.
  if (read_le16(raw_sector, user_offset + 0u) != 0x0160u) {
    return false;
  }

  // Validate chunk counters to reduce false positives.
  const u16 chunk_no = read_le16(raw_sector, user_offset + 4u);
  const u16 chunks_per_frame = read_le16(raw_sector, user_offset + 6u);
  if (chunks_per_frame == 0u || chunks_per_frame > 1024u) {
    return false;
  }
  if (chunk_no >= chunks_per_frame) {
    return false;
  }
  return true;
}

bool is_probable_str_video_sector(const std::vector<u8> &raw_sector) {
  // Typical user-data offsets:
  // - MODE2/2352 : +24 (12 sync + 4 header + 8 subheader)
  // - MODE2/2336 : +8  (4 subheader + 4 copy)
  // - 2048 dumps  : +0
  static constexpr std::array<size_t, 4> kCandidateOffsets = {24u, 8u, 0u, 16u};
  for (size_t offset : kCandidateOffsets) {
    if (looks_like_str_header_at(raw_sector, offset)) {
      return true;
    }
  }
  return false;
}

std::vector<u8> make_error_response(u8 stat, u8 error_code) {
  return {static_cast<u8>(stat | 0x01u), error_code};
}

void lba_to_msf_bcd(int abs_lba, u8 &mm, u8 &ss, u8 &ff) {
  const int lba = std::max(0, abs_lba);
  const int m = lba / (60 * 75);
  const int s = (lba / 75) % 60;
  const int f = lba % 75;
  mm = to_bcd8(clip_u8(m % 100));
  ss = to_bcd8(clip_u8(s));
  ff = to_bcd8(clip_u8(f));
}

constexpr std::array<s32, 4> kXaPosFilter = {0, 60, 115, 98};
constexpr std::array<s32, 4> kXaNegFilter = {0, 0, -52, -55};

int xa_shift_from_header(u8 header) {
  int shift = static_cast<int>(header & 0x0Fu);
  // Reserved values 13..15 behave like 9 on real hardware.
  if (shift > 12) {
    shift = 9;
  }
  return shift;
}

int xa_sign_extend_4(u8 value) {
  int sample = static_cast<int>(value & 0x0Fu);
  if ((sample & 0x08) != 0) {
    sample -= 0x10;
  }
  return sample;
}

int xa_sign_extend_8(u8 value) {
  return static_cast<int>(static_cast<s8>(value));
}

s16 xa_decode_sample(int packed_sample, bool eight_bit, int shift,
                     int filter, s32 &old, s32 &older) {
  const int clamped_filter = std::clamp(filter, 0, 3);
  const s32 f0 = kXaPosFilter[static_cast<size_t>(clamped_filter)];
  const s32 f1 = kXaNegFilter[static_cast<size_t>(clamped_filter)];

  s32 sample = static_cast<s32>(packed_sample) << (eight_bit ? 8 : 12);
  sample >>= shift;
  sample += ((old * f0) + (older * f1) + 32) >> 6;
  sample = std::clamp<s32>(sample, -32768, 32767);

  older = old;
  old = sample;
  return static_cast<s16>(sample);
}
} // namespace

void CdRom::reset() {
  index_reg_ = 0;
  interrupt_enable_ = 0;
  interrupt_flag_ = 0;

  param_fifo_.clear();
  response_fifo_.clear();
  response_index_ = 0;

  data_buffer_.clear();
  data_index_ = 0;
  data_ready_ = false;
  data_request_ = false;

  state_ = State::Idle;
  pending_second_ = {};
  pending_irqs_.clear();

  seek_mm_ = 0;
  seek_ss_ = 0;
  seek_ff_ = 0;
  read_lba_ = 0;
  pending_cycles_ = 0;
  read_period_cycles_ = read_period_for_mode();
  command_busy_ = false;
  command_busy_cycles_ = 0;
  last_command_ = 0;
  last_irq_code_ = 0;
  mode_ = 0x20u;
  filter_file_ = 0;
  filter_channel_ = 0;
  host_audio_regs_.fill(0);
  host_audio_apply_ = 0;

  command_counter_ = 0;
  command_hist_.fill(0);
  sector_counter_ = 0;
  saw_read_command_ = false;
  saw_getid_ = false;
  saw_setloc_ = false;
  saw_seekl_ = false;
  saw_readn_or_reads_ = false;
  saw_sector_visible_ = false;
  read_command_count_ = 0;
  irq_int1_count_ = 0;
  irq_int2_count_ = 0;
  irq_int3_count_ = 0;
  irq_int4_count_ = 0;
  irq_int5_count_ = 0;
  read_buffer_stall_count_ = 0;
  response_promotion_count_ = 0;
  status_e0_poll_count_ = 0;
  status_e0_streak_max_ = 0;
  status_e0_streak_current_ = 0;

  seek_target_lba_ = 0;
  seek_target_valid_ = false;
  seek_complete_ = false;
  read_whole_sector_ = true;
  pending_read_start_ = false;
  pending_reads_mode_ = false;
  cdda_playing_ = false;
  cdda_cmd_muted_ = false;
  cdda_adp_muted_ = false;
  adpcm_busy_cycles_ = 0;

  atv_pending_ = {0x80u, 0x00u, 0x80u, 0x00u};
  atv_active_ = atv_pending_;
  xa_hist1_ = {};
  xa_hist2_ = {};
  xa_stream_valid_ = false;
  xa_stream_file_ = 0;
  xa_stream_channel_ = 0;
  insert_probe_active_ = false;
  insert_probe_delay_cycles_ = 0;
  insert_probe_stage_ = 0;

  motor_on_ = false;
  shell_open_ = false;
  seek_error_ = false;
  id_error_ = false;
}

bool CdRom::load_bin_cue(const std::string &bin_path,
                         const std::string &cue_path) {
  if (bin_file_.is_open()) {
    bin_file_.close();
  }
  tracks_.clear();
  disc_loaded_ = false;
  track_map_valid_ = false;
  resolved_disc_path_.clear();
  bin_size_ = 0;

  std::filesystem::path cue_dir;
  if (!cue_path.empty()) {
    const std::filesystem::path cue_fs(cue_path);
    cue_dir = cue_fs.parent_path();
    if (!parse_cue(cue_path, cue_dir.string())) {
      LOG_ERROR("CDROM: Failed to parse CUE file: %s", cue_path.c_str());
      return false;
    }
  }

  std::filesystem::path bin_fs(bin_path);
  if (bin_fs.empty() && !tracks_.empty() && !tracks_.front().filename.empty()) {
    bin_fs = tracks_.front().filename;
  }
  if (!bin_fs.empty() && !bin_fs.is_absolute() && !cue_dir.empty()) {
    bin_fs = cue_dir / bin_fs;
  }

  if (bin_fs.empty()) {
    LOG_ERROR("CDROM: No BIN image specified");
    return false;
  }

  if (tracks_.empty()) {
    CdTrack track;
    track.number = 1;
    track.type = "MODE2/2352";
    track.filename = bin_fs.string();
    track.sector_size = 2352;
    track.index01_file_lba = 0;
    track.index01_abs_lba = 150;
    track.index01_file_offset = 0;
    tracks_.push_back(track);
  }

  std::string layout_error;
  bool monotonic = true;
  const std::string default_file =
      !tracks_.front().filename.empty() ? tracks_.front().filename : bin_fs.string();
  bool unused_single_file = true;
  if (!resolve_track_layout(tracks_, default_file, unused_single_file, monotonic,
                            layout_error)) {
    LOG_ERROR("CDROM: Invalid CUE track layout: %s", layout_error.c_str());
    return false;
  }

  const std::filesystem::path primary_fs = tracks_.front().filename.empty()
                                               ? bin_fs
                                               : std::filesystem::path(tracks_.front().filename);
  bin_file_.open(primary_fs, std::ios::binary);
  if (!bin_file_.is_open()) {
    LOG_ERROR("CDROM: Failed opening BIN image: %s", primary_fs.string().c_str());
    return false;
  }
  bin_size_ = file_size_bytes(primary_fs.string());

  resolved_disc_path_ = primary_fs.string();
  disc_loaded_ = true;
  track_map_valid_ = monotonic;

  reset();
  notify_disc_inserted();
  LOG_INFO("CDROM: Loaded disc - %zu track(s) from %s", tracks_.size(),
           resolved_disc_path_.c_str());
  if (!track_map_valid_) {
    LOG_WARN("CDROM: Track map is non-monotonic; compatibility may be reduced");
  }
  return true;
}

bool CdRom::swap_disc_image(const std::string &bin_path,
                            const std::string &cue_path) {
  const std::vector<CdTrack> old_tracks = tracks_;
  const std::string old_resolved_disc_path = resolved_disc_path_;
  const bool old_track_map_valid = track_map_valid_;
  const bool old_disc_loaded = disc_loaded_;
  const u64 old_bin_size = bin_size_;

  auto restore_previous_disc = [&]() {
    tracks_ = old_tracks;
    resolved_disc_path_ = old_resolved_disc_path;
    track_map_valid_ = old_track_map_valid;
    disc_loaded_ = old_disc_loaded;
    bin_size_ = old_bin_size;
    if (!old_resolved_disc_path.empty()) {
      bin_file_.clear();
      bin_file_.open(old_resolved_disc_path, std::ios::binary);
    }
  };

  if (bin_file_.is_open()) {
    bin_file_.close();
  }

  std::vector<CdTrack> parsed_tracks;
  std::filesystem::path cue_dir;
  std::filesystem::path bin_fs(bin_path);
  tracks_.clear();

  if (!cue_path.empty()) {
    const std::filesystem::path cue_fs(cue_path);
    cue_dir = cue_fs.parent_path();
    if (!parse_cue(cue_path, cue_dir.string())) {
      restore_previous_disc();
      LOG_ERROR("CDROM: Failed to parse CUE file for live insert: %s",
                cue_path.c_str());
      return false;
    }
    parsed_tracks = tracks_;
  }

  if (bin_fs.empty() && !parsed_tracks.empty() &&
      !parsed_tracks.front().filename.empty()) {
    bin_fs = parsed_tracks.front().filename;
  }
  if (!bin_fs.empty() && !bin_fs.is_absolute() && !cue_dir.empty()) {
    bin_fs = cue_dir / bin_fs;
  }
  if (bin_fs.empty()) {
    restore_previous_disc();
    LOG_ERROR("CDROM: No BIN image specified for live insert");
    return false;
  }

  if (parsed_tracks.empty()) {
    CdTrack track;
    track.number = 1;
    track.type = "MODE2/2352";
    track.filename = bin_fs.string();
    track.sector_size = 2352;
    track.index01_file_lba = 0;
    track.index01_abs_lba = 150;
    track.index01_file_offset = 0;
    parsed_tracks.push_back(track);
  }

  std::string layout_error;
  bool monotonic = true;
  bool unused_single_file = true;
  const std::string default_file = !parsed_tracks.front().filename.empty()
                                       ? parsed_tracks.front().filename
                                       : bin_fs.string();
  if (!resolve_track_layout(parsed_tracks, default_file, unused_single_file, monotonic,
                            layout_error)) {
    restore_previous_disc();
    LOG_ERROR("CDROM: Invalid CUE track layout for live insert: %s",
              layout_error.c_str());
    return false;
  }

  const std::filesystem::path primary_fs =
      parsed_tracks.front().filename.empty()
          ? bin_fs
          : std::filesystem::path(parsed_tracks.front().filename);
  std::ifstream replacement_bin(primary_fs, std::ios::binary);
  if (!replacement_bin.is_open()) {
    restore_previous_disc();
    LOG_ERROR("CDROM: Failed opening BIN image for live insert: %s",
              primary_fs.string().c_str());
    return false;
  }

  tracks_ = std::move(parsed_tracks);
  bin_file_ = std::move(replacement_bin);
  resolved_disc_path_ = primary_fs.string();
  bin_size_ = file_size_bytes(primary_fs.string());
  disc_loaded_ = true;
  track_map_valid_ = monotonic;

  // Keep controller configuration (notably IRQ enable) intact for hot-swap,
  // but clear in-flight transfer/decoder state from the previous disc.
  state_ = State::Idle;
  pending_second_ = {};
  pending_irqs_.clear();
  param_fifo_.clear();
  response_fifo_.clear();
  response_index_ = 0;
  data_buffer_.clear();
  data_index_ = 0;
  data_ready_ = false;
  data_request_ = false;
  command_busy_ = false;
  command_busy_cycles_ = 0;
  seek_mm_ = 0;
  seek_ss_ = 0;
  seek_ff_ = 0;
  read_lba_ = 0;
  pending_cycles_ = 0;
  seek_target_lba_ = 0;
  seek_target_valid_ = false;
  seek_complete_ = false;
  pending_read_start_ = false;
  pending_reads_mode_ = false;
  cdda_playing_ = false;
  seek_error_ = false;
  id_error_ = false;
  shell_open_ = false;
  motor_on_ = true;
  adpcm_busy_cycles_ = 0;
  xa_hist1_ = {};
  xa_hist2_ = {};
  xa_stream_valid_ = false;
  xa_stream_file_ = 0;
  xa_stream_channel_ = 0;
  interrupt_flag_ = 0;
  refresh_irq_line();

  LOG_INFO("CDROM: Live inserted disc - %zu track(s) from %s", tracks_.size(),
           resolved_disc_path_.c_str());
  if (!track_map_valid_) {
    LOG_WARN("CDROM: Track map is non-monotonic; compatibility may be reduced");
  }
  return true;
}

void CdRom::notify_disc_inserted() {
  if (!disc_loaded_) {
    return;
  }
  shell_open_ = false;
  seek_error_ = false;
  id_error_ = false;
  motor_on_ = true;
  // Issue a light-weight probe sequence after hot-insert so BIOS/game-side
  // polling sees the same command cadence as on hardware.
  insert_probe_active_ = true;
  insert_probe_delay_cycles_ = 1;
  insert_probe_stage_ = 0;
}

bool CdRom::parse_cue(const std::string &cue_path, const std::string &bin_dir) {
  std::ifstream cue(cue_path);
  if (!cue.is_open()) {
    return false;
  }

  tracks_.clear();
  std::string current_file;
  CdTrack *current_track = nullptr;
  std::string line;
  while (std::getline(cue, line)) {
    line = trim_copy(line);
    if (line.empty()) {
      continue;
    }

    if (starts_with_ci(line, "FILE")) {
      current_file = parse_file_name(line);
      if (!current_file.empty() && !bin_dir.empty()) {
        std::filesystem::path fp(current_file);
        if (!fp.is_absolute()) {
          fp = std::filesystem::path(bin_dir) / fp;
          current_file = fp.string();
        }
      }
      continue;
    }

    if (starts_with_ci(line, "TRACK")) {
      std::istringstream iss(line);
      std::string token;
      int number = 0;
      std::string type;
      iss >> token >> number >> type;
      if (number <= 0 || type.empty()) {
        current_track = nullptr;
        continue;
      }

      CdTrack track;
      track.number = number;
      track.type = upper_copy(type);
      track.filename = current_file;
      if (track.type == "MODE1/2048" || track.type == "MODE2/2048") {
        track.sector_size = 2048;
      } else if (track.type == "MODE2/2336" || track.type == "CDI/2336") {
        track.sector_size = 2336;
      } else {
        track.sector_size = 2352;
      }
      tracks_.push_back(track);
      current_track = &tracks_.back();
      continue;
    }

    if (current_track == nullptr) {
      continue;
    }

    if (starts_with_ci(line, "PREGAP")) {
      std::istringstream iss(line);
      std::string token;
      std::string msf;
      iss >> token >> msf;
      const int pregap = parse_msf_text(msf);
      if (pregap >= 0) {
        current_track->pregap_sectors = pregap;
      }
      continue;
    }

    if (starts_with_ci(line, "INDEX")) {
      std::istringstream iss(line);
      std::string token;
      int index_num = 0;
      std::string msf;
      iss >> token >> index_num >> msf;
      if (index_num == 1) {
        const int lba = parse_msf_text(msf);
        if (lba >= 0) {
          current_track->index01_file_lba = lba;
        }
      }
      continue;
    }
  }

  return !tracks_.empty();
}

u8 CdRom::stat_byte() const {
  u8 stat = 0;
  if (cdda_playing_) {
    stat |= 0x80u;
  } else if (state_ == State::Seeking) {
    stat |= 0x40u;
  } else if (state_ == State::Reading) {
    stat |= 0x20u;
  }
  if (shell_open_ || !disc_loaded_) {
    stat |= 0x10u;
  }
  if (id_error_) {
    stat |= 0x08u;
  }
  if (seek_error_) {
    stat |= 0x04u;
  }
  if (motor_on_) {
    stat |= 0x02u;
  }
  return stat;
}

int CdRom::msf_to_lba(u8 mm, u8 ss, u8 ff) const {
  const int m = from_bcd(mm);
  const int s = from_bcd(ss);
  const int f = from_bcd(ff);
  if (m < 0 || s < 0 || s >= 60 || f < 0 || f >= 75) {
    return -1;
  }
  return (m * 60 * 75 + s * 75 + f) - 150;
}

int CdRom::read_period_for_mode() const {
  const bool double_speed = (mode_ & 0x80u) != 0;
  return static_cast<int>(psx::CPU_CLOCK_HZ / (double_speed ? 150u : 75u));
}

int CdRom::command_busy_for(u8 cmd) const {
  switch (cmd) {
  case 0x01: // GetStat
  case 0x10: // GetlocL
  case 0x11: // GetlocP
    return 1200;
  case 0x02: // Setloc
  case 0x0E: // Setmode
    return 2000;
  case 0x03: // Play
  case 0x06: // ReadN
  case 0x1B: // ReadS
    return 2400;
  case 0x15: // SeekL
  case 0x16: // SeekP
    return 10000;
  case 0x1A: // GetID
  case 0x12: // Setsession
  case 0x1E: // ReadTOC
    return 4000;
  case 0x0A: // Init
    return 6000;
  case 0x09: // Pause
    return 4000;
  default:
    return 2000;
  }
}

void CdRom::refresh_read_period() {
  read_period_cycles_ = std::max(1, read_period_for_mode());
}

void CdRom::schedule_second_response(int delay_cycles, u8 irq,
                                     std::vector<u8> response) {
  pending_second_.active = true;
  pending_second_.delay = std::max(1, delay_cycles);
  pending_second_.irq = irq;
  pending_second_.response = std::move(response);
}

void CdRom::start_read_stream(bool reads_mode) {
  pending_reads_mode_ = reads_mode;
  if (seek_target_valid_ && seek_target_lba_ != read_lba_) {
    state_ = State::Seeking;
    pending_read_start_ = true;
    pending_cycles_ = std::max(1, read_period_cycles_ / 2);
  } else {
    state_ = State::Reading;
    pending_read_start_ = false;
    pending_cycles_ = std::max(1, read_period_cycles_);
  }
  cdda_playing_ = false;
  seek_complete_ = false;
  saw_read_command_ = true;
  saw_readn_or_reads_ = true;
}

const CdTrack *CdRom::track_for_lba(int lba) const {
  if (tracks_.empty()) {
    return nullptr;
  }
  const int abs_lba = lba + 150;
  const CdTrack *track = &tracks_.front();
  for (const CdTrack &candidate : tracks_) {
    if (abs_lba >= candidate.index01_abs_lba) {
      track = &candidate;
    } else {
      break;
    }
  }
  return track;
}

int CdRom::track_end_lba(const CdTrack *track) const {
  if (track == nullptr) {
    return -1;
  }
  auto it = std::find_if(tracks_.begin(), tracks_.end(),
                         [track](const CdTrack &entry) { return &entry == track; });
  if (it == tracks_.end()) {
    return -1;
  }
  const size_t idx = static_cast<size_t>(std::distance(tracks_.begin(), it));
  if (idx + 1u < tracks_.size()) {
    return tracks_[idx + 1u].index01_abs_lba - 151;
  }
  const u64 file_bytes = !track->filename.empty() ? file_size_bytes(track->filename)
                                                   : bin_size_;
  const int sectors =
      static_cast<int>(file_bytes / static_cast<u64>(std::max(1, track->sector_size)));
  const int rel_len = std::max(1, sectors - track->index01_file_lba);
  return (track->index01_abs_lba - 150) + rel_len - 1;
}

bool CdRom::read_raw_sector_for_lba(int psx_lba, std::vector<u8> &raw_sector,
                                    const CdTrack **track_out) {
  raw_sector.clear();
  if (!disc_loaded_) {
    return false;
  }

  const CdTrack *track = track_for_lba(psx_lba);
  if (track_out != nullptr) {
    *track_out = track;
  }
  if (track == nullptr) {
    return false;
  }

  const int rel = psx_lba - (track->index01_abs_lba - 150);
  if (rel < 0) {
    return false;
  }

  const int file_lba = track->index01_file_lba + rel;
  const int sector_size = std::max(1, track->sector_size);
  const u64 offset = static_cast<u64>(file_lba) * static_cast<u64>(sector_size);
  const std::string target_path =
      !track->filename.empty() ? track->filename : resolved_disc_path_;
  if (target_path.empty()) {
    return false;
  }

  std::ifstream alt_file;
  std::ifstream *stream = nullptr;
  u64 target_size = 0;

  const bool use_primary =
      !resolved_disc_path_.empty() &&
      normalize_path_key(target_path) == normalize_path_key(resolved_disc_path_) &&
      bin_file_.is_open();
  if (use_primary) {
    stream = &bin_file_;
    target_size = bin_size_;
  } else {
    alt_file.open(std::filesystem::path(target_path), std::ios::binary);
    if (!alt_file.is_open()) {
      return false;
    }
    stream = &alt_file;
    target_size = file_size_bytes(target_path);
  }

  if (target_size > 0 &&
      offset + static_cast<u64>(sector_size) > target_size) {
    return false;
  }

  raw_sector.resize(static_cast<size_t>(sector_size));
  stream->clear();
  stream->seekg(static_cast<std::streamoff>(offset), std::ios::beg);
  if (!stream->good()) {
    return false;
  }
  stream->read(reinterpret_cast<char *>(raw_sector.data()), sector_size);
  return stream->gcount() == sector_size;
}

bool CdRom::cd_audio_muted() const { return cdda_cmd_muted_ || cdda_adp_muted_; }

void CdRom::apply_host_audio_matrix(std::vector<s16> &samples) const {
  if (samples.size() < 2u) {
    return;
  }

  const s32 ll = static_cast<s32>(atv_active_[0]);
  const s32 lr = static_cast<s32>(atv_active_[3]);
  const s32 rl = static_cast<s32>(atv_active_[1]);
  const s32 rr = static_cast<s32>(atv_active_[2]);
  for (size_t i = 0; i + 1u < samples.size(); i += 2u) {
    const s32 l = samples[i + 0u];
    const s32 r = samples[i + 1u];
    const s32 out_l = ((l * ll) + (r * lr)) / 0x80;
    const s32 out_r = ((l * rl) + (r * rr)) / 0x80;
    samples[i + 0u] = sat16(out_l);
    samples[i + 1u] = sat16(out_r);
  }
}

bool CdRom::stream_cdda_sector() {
  std::vector<u8> raw;
  const CdTrack *track = nullptr;
  if (!read_raw_sector_for_lba(read_lba_, raw, &track)) {
    return false;
  }
  if (raw.size() < 2352u || !is_audio_track(track)) {
    return true;
  }

  if (!cd_audio_muted() && sys_ != nullptr) {
    std::vector<s16> samples;
    samples.reserve(raw.size() / 2u);
    for (size_t i = 0; i + 3u < raw.size(); i += 4u) {
      const s16 l = static_cast<s16>(static_cast<u16>(raw[i + 0u]) |
                                     (static_cast<u16>(raw[i + 1u]) << 8));
      const s16 r = static_cast<s16>(static_cast<u16>(raw[i + 2u]) |
                                     (static_cast<u16>(raw[i + 3u]) << 8));
      samples.push_back(l);
      samples.push_back(r);
    }
    apply_host_audio_matrix(samples);
    sys_->push_cd_audio_samples(samples, 44100u);
  }
  return true;
}

void CdRom::maybe_decode_xa_audio(const std::vector<u8> &raw_sector,
                                  const CdTrack *track) {
  if ((mode_ & 0x40u) == 0) {
    return;
  }
  const SectorLayout layout = describe_sector_layout(raw_sector, track);
  if (!layout.mode2) {
    return;
  }
  if (is_probable_str_video_sector(raw_sector)) {
    return;
  }

  u8 file = 0;
  u8 channel = 0;
  u8 submode = 0;
  u8 codinginfo = 0;
  if (!read_sector_subheader(raw_sector, layout, file, channel, submode, codinginfo)) {
    return;
  }
  // Treat sectors tagged as Video as non-audio, even if bit2 is noisy.
  // Some streams carry imperfect submode flags; decoding those as XA can
  // consume video payload and make MDEC appear to stall.
  if ((submode & 0x04u) == 0 || (submode & 0x02u) != 0) {
    return;
  }

  if ((mode_ & 0x08u) != 0 &&
      (file != filter_file_ || channel != filter_channel_)) {
    return;
  }

  const bool stream_changed =
      !xa_stream_valid_ || xa_stream_file_ != file || xa_stream_channel_ != channel;
  if (stream_changed) {
    xa_hist1_ = {};
    xa_hist2_ = {};
  }
  xa_stream_valid_ = true;
  xa_stream_file_ = file;
  xa_stream_channel_ = channel;
  adpcm_busy_cycles_ = std::max(adpcm_busy_cycles_, read_period_cycles_);

  const bool stereo = (codinginfo & 0x03u) == 0x01u;
  const bool sample_rate_18900 = (codinginfo & 0x04u) != 0;
  const u8 bits_per_sample = static_cast<u8>((codinginfo >> 4) & 0x03u);
  const bool eight_bit = (bits_per_sample == 0x01u);
  const bool four_bit = (bits_per_sample == 0x00u);
  const u32 sample_rate = sample_rate_18900 ? 18900u : 37800u;

  if (!four_bit && !eight_bit) {
    static bool warned_bad_bps = false;
    if (!warned_bad_bps && g_log_fmv_diagnostics) {
      LOG_WARN("CDROM: XA unsupported bits-per-sample coding=%u", bits_per_sample);
      warned_bad_bps = true;
    }
    return;
  }

  const size_t decode_base = layout.user_data_offset;
  if (!cd_audio_muted() && sys_ != nullptr &&
      raw_sector.size() >= (decode_base + 18u * 128u)) {
    std::vector<s16> samples;

    if (stereo) {
      const size_t blocks_per_group = four_bit ? 4u : 2u;
      samples.reserve(18u * blocks_per_group * 28u * 2u);

      s32 old_l = xa_hist1_[0];
      s32 older_l = xa_hist2_[0];
      s32 old_r = xa_hist1_[1];
      s32 older_r = xa_hist2_[1];

      for (size_t group = 0; group < 18u; ++group) {
        const size_t base = decode_base + group * 128u;
        for (size_t blk = 0; blk < blocks_per_group; ++blk) {
          if (four_bit) {
            const u8 header_l = raw_sector[base + 4u + blk * 2u + 0u];
            const u8 header_r = raw_sector[base + 4u + blk * 2u + 1u];
            const int shift_l = xa_shift_from_header(header_l);
            const int shift_r = xa_shift_from_header(header_r);
            const int filter_l = static_cast<int>((header_l >> 4) & 0x03u);
            const int filter_r = static_cast<int>((header_r >> 4) & 0x03u);

            for (size_t j = 0; j < 28u; ++j) {
              const u8 packed = raw_sector[base + 16u + blk + j * 4u];
              const int t_l = xa_sign_extend_4(static_cast<u8>(packed & 0x0Fu));
              const int t_r = xa_sign_extend_4(static_cast<u8>((packed >> 4) & 0x0Fu));
              const s16 s_l = xa_decode_sample(t_l, false, shift_l, filter_l,
                                               old_l, older_l);
              const s16 s_r = xa_decode_sample(t_r, false, shift_r, filter_r,
                                               old_r, older_r);
              samples.push_back(s_l);
              samples.push_back(s_r);
            }
          } else {
            const u8 header_l = raw_sector[base + 4u + blk * 2u + 0u];
            const u8 header_r = raw_sector[base + 4u + blk * 2u + 1u];
            const int shift_l = xa_shift_from_header(header_l);
            const int shift_r = xa_shift_from_header(header_r);
            const int filter_l = static_cast<int>((header_l >> 4) & 0x03u);
            const int filter_r = static_cast<int>((header_r >> 4) & 0x03u);

            for (size_t j = 0; j < 28u; ++j) {
              const u32 packed = static_cast<u32>(raw_sector[base + 16u + j * 4u + 0u]) |
                                 (static_cast<u32>(raw_sector[base + 16u + j * 4u + 1u]) << 8) |
                                 (static_cast<u32>(raw_sector[base + 16u + j * 4u + 2u]) << 16) |
                                 (static_cast<u32>(raw_sector[base + 16u + j * 4u + 3u]) << 24);
              const int t_l = xa_sign_extend_8(
                  static_cast<u8>((packed >> (blk * 16u + 0u)) & 0xFFu));
              const int t_r = xa_sign_extend_8(
                  static_cast<u8>((packed >> (blk * 16u + 8u)) & 0xFFu));
              const s16 s_l = xa_decode_sample(t_l, true, shift_l, filter_l,
                                               old_l, older_l);
              const s16 s_r = xa_decode_sample(t_r, true, shift_r, filter_r,
                                               old_r, older_r);
              samples.push_back(s_l);
              samples.push_back(s_r);
            }
          }
        }
      }

      xa_hist1_[0] = sat16(old_l);
      xa_hist2_[0] = sat16(older_l);
      xa_hist1_[1] = sat16(old_r);
      xa_hist2_[1] = sat16(older_r);
    } else {
      // FIX: mono 4-bit uses 4 blocks/group, mono 8-bit uses 4 blocks/group.
      // The original code had `four_bit ? 4u : 4u` (dead ternary). This is
      // actually numerically correct for both cases, but clarified for intent.
      const size_t blocks_per_group = 4u;
      samples.reserve(18u * blocks_per_group * 28u * 2u);

      s32 old = xa_hist1_[0];
      s32 older = xa_hist2_[0];

      for (size_t group = 0; group < 18u; ++group) {
        const size_t base = decode_base + group * 128u;
        for (size_t blk = 0; blk < blocks_per_group; ++blk) {
          if (four_bit) {
            for (size_t nibble = 0; nibble < 2u; ++nibble) {
              const u8 header = raw_sector[base + 4u + blk * 2u + nibble];
              const int shift = xa_shift_from_header(header);
              const int filter = static_cast<int>((header >> 4) & 0x03u);
              for (size_t j = 0; j < 28u; ++j) {
                const u8 packed = raw_sector[base + 16u + blk + j * 4u];
                const int t = xa_sign_extend_4(
                    static_cast<u8>((packed >> (nibble * 4u)) & 0x0Fu));
                const s16 s = xa_decode_sample(t, false, shift, filter, old, older);
                samples.push_back(s);
                samples.push_back(s);
              }
            }
          } else {
            const u8 header = raw_sector[base + 4u + blk];
            const int shift = xa_shift_from_header(header);
            const int filter = static_cast<int>((header >> 4) & 0x03u);
            for (size_t j = 0; j < 28u; ++j) {
              const int t = xa_sign_extend_8(raw_sector[base + 16u + j * 4u + blk]);
              const s16 s = xa_decode_sample(t, true, shift, filter, old, older);
              samples.push_back(s);
              samples.push_back(s);
            }
          }
        }
      }

      xa_hist1_[0] = sat16(old);
      xa_hist2_[0] = sat16(older);
      xa_hist1_[1] = xa_hist1_[0];
      xa_hist2_[1] = xa_hist2_[0];
    }

    apply_host_audio_matrix(samples);
    sys_->push_cd_audio_samples(samples, sample_rate);
  }
}

bool CdRom::read_sector() {
  std::vector<u8> raw;
  const CdTrack *track = nullptr;
  if (!read_raw_sector_for_lba(read_lba_, raw, &track)) {
    return false;
  }

  if (is_audio_track(track) && (mode_ & 0x01u) == 0) {
    return false;
  }

  const SectorLayout layout = describe_sector_layout(raw, track);
  u8 file = 0;
  u8 channel = 0;
  u8 submode = 0;
  u8 ignored_codinginfo = 0;
  const bool has_subheader =
      read_sector_subheader(raw, layout, file, channel, submode, ignored_codinginfo);
  const bool looks_like_str_video = is_probable_str_video_sector(raw);
  const bool is_video_sector =
      looks_like_str_video || (has_subheader && ((submode & 0x02u) != 0));
  const bool is_audio_sector =
      has_subheader && ((submode & 0x04u) != 0) && !is_video_sector;
  const bool filter_on = (mode_ & 0x08u) != 0;
  const bool filter_match =
      !filter_on || (file == filter_file_ && channel == filter_channel_);

  bool delivered_to_adpcm = false;
  if ((mode_ & 0x40u) != 0 && is_audio_sector && filter_match) {
    maybe_decode_xa_audio(raw, track);
    delivered_to_adpcm = true;
  }

  bool deliver_data = true;
  if (delivered_to_adpcm) {
    deliver_data = false;
  }
  if (filter_on && is_audio_sector && !filter_match) {
    deliver_data = false;
  }
  if (!deliver_data) {
    return true;
  }

  std::vector<u8> payload;
  // ReadN (06h) and ReadS (1Bh) differ in retry policy/timing, not in how the
  // host data FIFO is formatted.
  //
  // Host payload width is selected by Setmode bits:
  // - bit5=1 -> raw sector payload (0x924 bytes after sync)
  // - bit5=0 -> user data only (0x800 bytes)
  //
  // Setmode bit4 is the CD-ROM "ignore bit", not a host payload-size selector.
  if (read_whole_sector_) {
    if (raw.size() >= (12u + 0x924u)) {
      payload.assign(raw.begin() + 12, raw.begin() + 12 + 0x924u);
    } else {
      payload = raw;
    }
  } else {
    size_t data_offset = layout.user_data_offset;
    size_t data_size = layout.user_data_size;
    if (data_size == 0u && !raw.empty()) {
      data_offset = 0u;
      data_size = raw.size();
    }
    if (data_offset + data_size > raw.size()) {
      data_size = raw.size() - data_offset;
    }
    payload.assign(raw.begin() + static_cast<ptrdiff_t>(data_offset),
                   raw.begin() + static_cast<ptrdiff_t>(data_offset + data_size));
  }

  if (data_ready_ && data_index_ < static_cast<int>(data_buffer_.size())) {
    ++read_buffer_stall_count_;
  }
  data_buffer_ = std::move(payload);
  data_index_ = 0;
  data_ready_ = !data_buffer_.empty();
  if (data_ready_) {
    saw_sector_visible_ = true;
  }
  return true;
}

void CdRom::fire_irq(u8 irq_num) {
  interrupt_flag_ = static_cast<u8>((interrupt_flag_ & 0xF8u) | irq_to_hint(irq_num));
  last_irq_code_ = irq_num;
  if (irq_num == 1u) {
    ++irq_int1_count_;
  } else if (irq_num == 2u) {
    ++irq_int2_count_;
  } else if (irq_num == 3u) {
    ++irq_int3_count_;
  } else if (irq_num == 4u) {
    ++irq_int4_count_;
  } else if (irq_num == 5u) {
    ++irq_int5_count_;
  }
  refresh_irq_line();
}

void CdRom::enqueue_irq(u8 irq_num, std::vector<u8> response,
                        bool wait_for_command_idle) {
  const bool pending = (interrupt_flag_ & kHintTypeMask) != 0;
  const bool response_pending =
      response_index_ < static_cast<int>(response_fifo_.size());

  if (pending || response_pending || (wait_for_command_idle && command_busy_)) {
    pending_irqs_.push_back(PendingIrq{
        irq_num,
        wait_for_command_idle,
        std::move(response),
    });
    return;
  }

  response_fifo_ = std::move(response);
  if (response_fifo_.size() > kFifoCapacity) {
    response_fifo_.resize(kFifoCapacity);
  }
  response_index_ = 0;
  fire_irq(irq_num);
}

void CdRom::service_pending_irq() {
  if (pending_irqs_.empty()) {
    return;
  }
  if ((interrupt_flag_ & kHintTypeMask) != 0) {
    return;
  }
  if (response_index_ < static_cast<int>(response_fifo_.size())) {
    return;
  }

  PendingIrq &front = pending_irqs_.front();
  if (front.wait_for_command_idle && command_busy_) {
    return;
  }

  response_fifo_ = std::move(front.response);
  if (response_fifo_.size() > kFifoCapacity) {
    response_fifo_.resize(kFifoCapacity);
  }
  response_index_ = 0;
  const u8 irq = front.irq;
  pending_irqs_.pop_front();
  ++response_promotion_count_;
  fire_irq(irq);
}

void CdRom::refresh_irq_line() {
  if (sys_ == nullptr) {
    return;
  }
  const u8 active_type = static_cast<u8>(interrupt_flag_ & kHintTypeMask);
  const u8 active_mask = hint_to_mask(active_type);
  if ((interrupt_enable_ & active_mask & kHintMaskAll) != 0) {
    sys_->irq().request(Interrupt::CDROM);
  }
}

void CdRom::cmd_getstat() {
  enqueue_irq(3, {stat_byte()});
}

void CdRom::cmd_setloc() {
  saw_setloc_ = true;
  if (param_fifo_.size() != 3u) {
    enqueue_irq(5, make_error_response(stat_byte(), 0x20u));
    return;
  }

  const int target = msf_to_lba(param_fifo_[0], param_fifo_[1], param_fifo_[2]);
  if (target < 0) {
    enqueue_irq(5, make_error_response(stat_byte(), 0x10u));
    return;
  }

  seek_mm_ = param_fifo_[0];
  seek_ss_ = param_fifo_[1];
  seek_ff_ = param_fifo_[2];
  seek_target_lba_ = target;
  seek_target_valid_ = true;
  seek_error_ = false;
  enqueue_irq(3, {stat_byte()});
}

void CdRom::cmd_play() {
  if (!disc_loaded_) {
    enqueue_irq(5, make_error_response(stat_byte(), 0x80u));
    return;
  }

  if (!param_fifo_.empty()) {
    const int track_no = from_bcd(param_fifo_[0]);
    if (track_no > 0) {
      auto it = std::find_if(tracks_.begin(), tracks_.end(),
                             [track_no](const CdTrack &track) {
                               return track.number == track_no;
                             });
      if (it != tracks_.end()) {
        read_lba_ = it->index01_abs_lba - 150;
        seek_target_lba_ = read_lba_;
        seek_target_valid_ = true;
      }
    }
  } else if (seek_target_valid_) {
    read_lba_ = seek_target_lba_;
  }

  motor_on_ = true;
  cdda_playing_ = true;
  state_ = State::Reading;
  pending_cycles_ = std::max(1, read_period_cycles_);
  enqueue_irq(3, {stat_byte()});
}

void CdRom::cmd_readn() {
  if (!disc_loaded_) {
    enqueue_irq(5, make_error_response(stat_byte(), 0x80u));
    return;
  }

  if (seek_target_valid_) {
    const CdTrack *target_track = track_for_lba(seek_target_lba_);
    if (is_audio_track(target_track) && (mode_ & 0x01u) == 0) {
      enqueue_irq(5, make_error_response(stat_byte(), 0x40u));
      return;
    }
  }

  ++read_command_count_;
  motor_on_ = true;
  start_read_stream(false);
  enqueue_irq(3, {stat_byte()});
}

void CdRom::cmd_pause() {
  const u8 first = stat_byte();
  cdda_playing_ = false;
  pending_read_start_ = false;
  pending_reads_mode_ = false;
  state_ = State::Idle;
  seek_complete_ = true;
  enqueue_irq(3, {first});
  schedule_second_response(25000, 2, {stat_byte()});
}

void CdRom::cmd_init() {
  mode_ = 0x20u;
  filter_file_ = 0;
  filter_channel_ = 0;
  read_whole_sector_ = true;
  motor_on_ = true;
  cdda_playing_ = false;
  cdda_cmd_muted_ = false;
  cdda_adp_muted_ = false;
  seek_error_ = false;
  id_error_ = false;
  pending_read_start_ = false;
  pending_reads_mode_ = false;
  state_ = State::Idle;
  refresh_read_period();
  enqueue_irq(3, {stat_byte()});
  schedule_second_response(50000, 2, {stat_byte()});
}

void CdRom::cmd_setmode() {
  if (param_fifo_.size() != 1u) {
    enqueue_irq(5, make_error_response(stat_byte(), 0x20u));
    return;
  }

  const u8 new_mode = param_fifo_[0];
  // Setmode bit5 selects raw sector reads (0x924 bytes after sync).
  // Bit4 is the CD-ROM "ignore bit" and does not change host FIFO width.
  read_whole_sector_ = (new_mode & 0x20u) != 0;
  mode_ = new_mode;
  // id_error_ = (mode_ & 0x10u) != 0;
  refresh_read_period();
  enqueue_irq(3, {stat_byte()});
}

void CdRom::cmd_seekl() {
  saw_seekl_ = true;
  if (!seek_target_valid_) {
    enqueue_irq(5, make_error_response(stat_byte(), 0x10u));
    return;
  }
  motor_on_ = true;
  cdda_playing_ = false;
  pending_read_start_ = false;
  state_ = State::Seeking;
  pending_cycles_ = std::max(1, read_period_cycles_ / 2);
  seek_error_ = false;
  enqueue_irq(3, {stat_byte()});
  schedule_second_response(33868, 2, {stat_byte()});
}

void CdRom::cmd_getid() {
  saw_getid_ = true;
  if (shell_open_ || !disc_loaded_) {
    enqueue_irq(5, {0x11u, 0x80u}); // 0x11 = Shell Open (0x10) | Error (0x01)
    return;
  }

  const bool audio_disk = !tracks_.empty() && is_audio_track(&tracks_.front());
  enqueue_irq(3, {stat_byte()});
  if (audio_disk) {
    id_error_ = true;
    schedule_second_response(33868, 5,
                             {static_cast<u8>(stat_byte() | 0x01u), 0x90u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u});
  } else {
    id_error_ = false;
    schedule_second_response(33868, 2,
                             {stat_byte(), 0x00u, 0x20u, 0x00u,
                              static_cast<u8>('S'), static_cast<u8>('C'),
                              static_cast<u8>('E'), static_cast<u8>('A')});
    if (g_trace_cdrom) {
      LOG_INFO("CDROM: GetID queued region=SCEA type=0x20");
    }
  }
}

void CdRom::cmd_reads() {
  if (!disc_loaded_) {
    enqueue_irq(5, make_error_response(stat_byte(), 0x80u));
    return;
  }
  ++read_command_count_;
  motor_on_ = true;
  start_read_stream(true);
  enqueue_irq(3, {stat_byte()});
}

void CdRom::cmd_test() {
  if (param_fifo_.empty()) {
    enqueue_irq(5, make_error_response(stat_byte(), 0x20u));
    return;
  }

  switch (param_fifo_[0]) {
  case 0x20:
    enqueue_irq(3, {0x96u, 0x08u, 0x24u, 0xC1u});
    break;
  case 0x21:
    enqueue_irq(3, {0x00u});
    break;
  case 0x22:
    enqueue_irq(3, {static_cast<u8>('f'), static_cast<u8>('o'),
                    static_cast<u8>('r'), static_cast<u8>(' '),
                    static_cast<u8>('U'), static_cast<u8>('/'),
                    static_cast<u8>('C')});
    break;
  default:
    enqueue_irq(5, make_error_response(stat_byte(), 0x10u));
    break;
  }
}

void CdRom::cmd_gettn() {
  if (!disc_loaded_ || tracks_.empty()) {
    enqueue_irq(5, make_error_response(stat_byte(), 0x80u));
    return;
  }
  const u8 first = to_bcd(clip_u8(std::clamp(tracks_.front().number, 1, 99)));
  const u8 last = to_bcd(clip_u8(std::clamp(tracks_.back().number, 1, 99)));
  enqueue_irq(3, {stat_byte(), first, last});
}

void CdRom::cmd_gettd() {
  if (!disc_loaded_ || tracks_.empty()) {
    enqueue_irq(5, make_error_response(stat_byte(), 0x80u));
    return;
  }
  if (param_fifo_.size() != 1u) {
    enqueue_irq(5, make_error_response(stat_byte(), 0x20u));
    return;
  }

  const int track_no = from_bcd(param_fifo_[0]);
  int abs_lba = 0;
  if (track_no == 0) {
    const CdTrack *last = &tracks_.back();
    abs_lba = track_end_lba(last) + 150 + 1;
  } else {
    auto it =
        std::find_if(tracks_.begin(), tracks_.end(), [track_no](const CdTrack &t) {
          return t.number == track_no;
        });
    if (it == tracks_.end()) {
      enqueue_irq(5, make_error_response(stat_byte(), 0x10u));
      return;
    }
    abs_lba = it->index01_abs_lba;
  }

  u8 mm = 0;
  u8 ss = 0;
  u8 ff = 0;
  lba_to_msf_bcd(abs_lba, mm, ss, ff);
  enqueue_irq(3, {stat_byte(), mm, ss});
}

void CdRom::cmd_stop() {
  const u8 first = stat_byte();
  cdda_playing_ = false;
  state_ = State::Idle;
  pending_read_start_ = false;
  pending_reads_mode_ = false;
  motor_on_ = false;
  data_ready_ = false;
  data_buffer_.clear();
  data_index_ = 0;
  enqueue_irq(3, {first});
  schedule_second_response(33868, 2, {stat_byte()});
}

// FIX: cmd_standby had inverted motor_on_ guard. Standby spins the motor up
// from a stopped state; it should error if the motor is ALREADY on, not if
// it is off. The original `if (motor_on_)` caused GT2's boot sequence to
// receive INT5 instead of INT3 whenever Standby was called while the drive
// was already spinning (the normal case), hanging the loader.
void CdRom::cmd_standby() {
  if (!motor_on_) {
    enqueue_irq(5, make_error_response(stat_byte(), 0x20u));
    return;
  }
  motor_on_ = true;
  cdda_playing_ = false;
  state_ = State::Idle;
  enqueue_irq(3, {stat_byte()});
  schedule_second_response(40000, 2, {stat_byte()});
}

void CdRom::cmd_mute() {
  cdda_cmd_muted_ = true;
  enqueue_irq(3, {stat_byte()});
}

void CdRom::cmd_demute() {
  cdda_cmd_muted_ = false;
  enqueue_irq(3, {stat_byte()});
}

void CdRom::cmd_setfilter() {
  if (param_fifo_.size() != 2u) {
    enqueue_irq(5, make_error_response(stat_byte(), 0x20u));
    return;
  }
  filter_file_ = param_fifo_[0];
  filter_channel_ = param_fifo_[1];
  enqueue_irq(3, {stat_byte()});
}

void CdRom::cmd_setsession() {
  if (param_fifo_.size() != 1u) {
    enqueue_irq(5, make_error_response(stat_byte(), 0x20u));
    return;
  }
  const u8 session = param_fifo_[0];
  if (session == 0) {
    enqueue_irq(5, make_error_response(stat_byte(), 0x10u));
    return;
  }
  enqueue_irq(3, {stat_byte()});
  schedule_second_response(33868, 2, {stat_byte()});
}

void CdRom::cmd_getloc_l() {
  if (!disc_loaded_) {
    enqueue_irq(5, make_error_response(stat_byte(), 0x80u));
    return;
  }
  if (state_ == State::Seeking) {
    enqueue_irq(5, make_error_response(stat_byte(), 0x80u));
    return;
  }

  const int probe_lba = std::max(0, read_lba_ - 1);
  std::vector<u8> raw;
  const CdTrack *track = nullptr;
  if (!read_raw_sector_for_lba(probe_lba, raw, &track) || raw.empty() ||
      is_audio_track(track)) {
    enqueue_irq(5, make_error_response(stat_byte(), 0x80u));
    return;
  }

  if (raw.size() >= 2352u && raw.size() >= 20u) {
    enqueue_irq(3, {raw[12], raw[13], raw[14], raw[15], raw[16], raw[17], raw[18],
                    raw[19]});
    return;
  }

  const SectorLayout layout = describe_sector_layout(raw, track);
  u8 file = 0;
  u8 channel = 0;
  u8 submode = 0;
  u8 codinginfo = 0;
  (void)read_sector_subheader(raw, layout, file, channel, submode, codinginfo);

  const int abs_lba = std::max(0, probe_lba + 150);
  u8 mm = 0;
  u8 ss = 0;
  u8 ff = 0;
  lba_to_msf_bcd(abs_lba, mm, ss, ff);
  enqueue_irq(3, {mm, ss, ff, static_cast<u8>(layout.mode2 ? 0x02u : 0x01u), file,
                  channel, submode, codinginfo});
}

void CdRom::cmd_getloc_p() {
  if (!disc_loaded_) {
    enqueue_irq(5, make_error_response(stat_byte(), 0x80u));
    return;
  }

  const CdTrack *track = track_for_lba(read_lba_);
  const int abs_lba = std::max(0, read_lba_ + 150);
  const int track_start = (track != nullptr) ? track->index01_abs_lba : 150;
  const int rel_lba = std::max(0, abs_lba - track_start);

  u8 mm = 0;
  u8 ss = 0;
  u8 ff = 0;
  u8 amm = 0;
  u8 ass = 0;
  u8 aff = 0;
  lba_to_msf_bcd(rel_lba, mm, ss, ff);
  lba_to_msf_bcd(abs_lba, amm, ass, aff);

  const int track_no = (track != nullptr) ? track->number : 1;
  enqueue_irq(3, {to_bcd(clip_u8(std::clamp(track_no, 1, 99))), 0x01u, mm, ss, ff,
                  amm, ass, aff});
}

void CdRom::cmd_readtoc() {
  motor_on_ = true;
  // ReadTOC handshake is INT3 (command accepted) followed by delayed INT2
  // (command completion).
  enqueue_irq(3, {stat_byte()});
  schedule_second_response(33868, 2, {stat_byte()});
}

void CdRom::execute_internal_command(u8 cmd,
                                     std::initializer_list<u8> params) {
  if (command_busy_ || command_busy_cycles_ > 0) {
    return;
  }
  if ((interrupt_flag_ & kHintTypeMask) != 0) {
    return;
  }
  if (response_index_ < static_cast<int>(response_fifo_.size())) {
    return;
  }

  last_command_ = cmd;
  command_busy_cycles_ = command_busy_for(cmd);
  command_busy_ = command_busy_cycles_ > 0;
  param_fifo_.assign(params.begin(), params.end());
  execute_command(cmd);
  param_fifo_.clear();
}

void CdRom::execute_command(u8 cmd) {
  switch (cmd) {
  case 0x01:
    cmd_getstat();
    break;
  case 0x02:
    cmd_setloc();
    break;
  case 0x03:
    cmd_play();
    break;
  case 0x04:
  case 0x05:
    enqueue_irq(3, {stat_byte()});
    break;
  case 0x06:
    cmd_readn();
    break;
  case 0x07:
    cmd_standby();
    break;
  case 0x08:
    cmd_stop();
    break;
  case 0x09:
    cmd_pause();
    break;
  case 0x0A:
    cmd_init();
    break;
  case 0x0B:
    cmd_mute();
    break;
  case 0x0C:
    cmd_demute();
    break;
  case 0x0D:
    cmd_setfilter();
    break;
  case 0x0E:
    cmd_setmode();
    break;
  case 0x0F:
    enqueue_irq(3, {stat_byte(), mode_, 0x00u, filter_file_, filter_channel_});
    break;
  case 0x10:
    cmd_getloc_l();
    break;
  case 0x11:
    cmd_getloc_p();
    break;
  case 0x12:
    cmd_setsession();
    break;
  case 0x13:
    cmd_gettn();
    break;
  case 0x14:
    cmd_gettd();
    break;
  case 0x15:
  case 0x16:
    cmd_seekl();
    break;
  case 0x19:
    cmd_test();
    break;
  case 0x1A:
    cmd_getid();
    break;
  case 0x1B:
    cmd_reads();
    break;
  case 0x1C:
    enqueue_irq(3, {stat_byte()});
    break;
  case 0x1E:
    cmd_readtoc();
    break;
  default:
    enqueue_irq(5, {0x11u, 0x40u});
    break;
  }
}

u8 CdRom::read8(u32 offset) {
  offset &= 0x3u;
  if (g_trace_cdrom) {
    static u64 cdrom_read_counter = 0;
    if (trace_should_log(cdrom_read_counter, g_trace_burst_cdrom,
                         g_trace_stride_cdrom)) {
      LOG_DEBUG("CDROM: read8 off=%u bank=%u", static_cast<unsigned>(offset),
                static_cast<unsigned>(index_reg_));
    }
  }

  switch (offset) {
  case 0: {
    u8 status = static_cast<u8>(index_reg_ & 0x03u);
    if (adpcm_busy_cycles_ > 0) {
      status |= 0x04u;
    }
    if (param_fifo_.empty()) {
      status |= 0x08u;
    }
    if (param_fifo_.size() < kFifoCapacity) {
      status |= 0x10u;
    }
    if (response_index_ < static_cast<int>(response_fifo_.size())) {
      status |= 0x20u;
    }
    if (data_ready_ && data_request_) {
      status |= 0x40u;
    }
    if (command_busy_) {
      status |= 0x80u;
    }

    if (status == 0xE0u) {
      ++status_e0_poll_count_;
      ++status_e0_streak_current_;
      status_e0_streak_max_ =
          std::max(status_e0_streak_max_, status_e0_streak_current_);
    } else {
      status_e0_streak_current_ = 0;
    }
    return status;
  }

  case 1:
    if (response_index_ < static_cast<int>(response_fifo_.size())) {
      const u8 value = response_fifo_[response_index_++];
      if (response_index_ >= static_cast<int>(response_fifo_.size())) {
        response_fifo_.clear();
        response_index_ = 0;
        service_pending_irq();
      }
      return value;
    }
    return 0;

  case 2: {
    if (data_ready_ && data_index_ < static_cast<int>(data_buffer_.size())) {
      const u8 value = data_buffer_[static_cast<size_t>(data_index_++)];
      if (data_index_ >= static_cast<int>(data_buffer_.size())) {
        data_ready_ = false;
      }
      return value;
    }
    return 0;
  }

  case 3:
    if (index_reg_ == 0 || index_reg_ == 2) {
      return static_cast<u8>(interrupt_enable_ | 0xE0u);
    }
    return static_cast<u8>(interrupt_flag_ | 0xE0u);
  }

  return 0xFFu;
}

void CdRom::write8(u32 offset, u8 value) {
  // FIX: offset masking was incorrectly commented out. Without this mask,
  // writes with stray high bits in the offset fall through all switch cases
  // silently, losing commands and parameters entirely.
  offset &= 0x3u;
  if (g_trace_cdrom) {
    static u64 cdrom_write_counter = 0;
    if (trace_should_log(cdrom_write_counter, g_trace_burst_cdrom,
                         g_trace_stride_cdrom)) {
      LOG_DEBUG("CDROM: write8 off=%u bank=%u value=%02X",
                static_cast<unsigned>(offset), static_cast<unsigned>(index_reg_),
                value);
    }
  }

  switch (offset) {
  case 0:
    index_reg_ = value & 0x3;
    return;

  case 1:
    switch (index_reg_) {
    case 0:
      last_command_ = value;
      ++command_counter_;
      ++command_hist_[static_cast<size_t>(value)];
      command_busy_cycles_ = command_busy_for(value);
      command_busy_ = command_busy_cycles_ > 0;
      execute_command(value);
      param_fifo_.clear();
      return;
    case 1:
      host_audio_regs_[6] = value;
      return;
    case 2:
      host_audio_regs_[1] = value;
      return;
    case 3:
      atv_pending_[2] = value;
      return;
    default:
      LOG_WARN("CDROM: Write to port 1 index %u = 0x%02X", index_reg_, value);
      return;
    }

  case 2:
    switch (index_reg_) {
    case 0:
      if (param_fifo_.size() < kFifoCapacity) {
        param_fifo_.push_back(value);
      }
      return;
    case 1:
      interrupt_enable_ = static_cast<u8>(value & kHintMaskAll);
      refresh_irq_line();
      return;
    case 2:
      atv_pending_[0] = value;
      return;
    case 3:
      atv_pending_[3] = value;
      return;
    default:
      return;
    }

  case 3:
    switch (index_reg_) {
    case 0:
      host_audio_regs_[0] = value;
      data_request_ = (value & 0x80u) != 0;
      // FIX D: clearing data_request_ (BFRD=0) must reset the read position
      // so that a subsequent set re-delivers the sector from the beginning.
      // Metal Gear Solid: Special Missions clears BFRD between two DMAs and
      // needs the buffer pointer reset; matches DuckStation's sb.position = 0.
      if (!data_request_) {
        data_index_ = 0;
      }
      if ((value & 0x20u) != 0) {
        host_audio_regs_[7] = 1;
      }
      return;

    case 1:
      if ((value & 0x40u) != 0) {
        param_fifo_.clear();
      }
      if ((value & 0x20u) != 0) {
        adpcm_busy_cycles_ = 0;
      }
      if ((value & 0x1Fu) != 0) {
        const u8 ack_mask = static_cast<u8>(value & 0x1Fu);
        const u8 old_type = static_cast<u8>(interrupt_flag_ & kHintTypeMask);
        const u8 old_mask = hint_to_mask(old_type);
        if ((old_mask & ack_mask) != 0u) {
          interrupt_flag_ =
              static_cast<u8>(interrupt_flag_ & static_cast<u8>(~kHintTypeMask));
          response_fifo_.clear();
          response_index_ = 0;
        }
      }
      if ((value & 0x80u) != 0) {
        response_fifo_.clear();
        response_index_ = 0;
      }
      refresh_irq_line();
      service_pending_irq();
      return;

    case 2:
      atv_pending_[1] = value;
      return;

    case 3:
      host_audio_apply_ = value;
      cdda_adp_muted_ = (value & 0x01u) != 0;
      if ((value & 0x20u) != 0) {
        atv_active_ = atv_pending_;
      }
      return;
    default:
      return;
    }
  }
}

void CdRom::tick(u32 cycles) {
  const bool profile_detailed = g_profile_detailed_timing;
  std::chrono::high_resolution_clock::time_point t0{};
  if (profile_detailed) {
    t0 = std::chrono::high_resolution_clock::now();
  }

  const int step = static_cast<int>(cycles);

  if (command_busy_) {
    command_busy_cycles_ -= step;
    if (command_busy_cycles_ <= 0) {
      command_busy_cycles_ = 0;
      command_busy_ = false;
      service_pending_irq();
    }
  }

  if (adpcm_busy_cycles_ > 0) {
    adpcm_busy_cycles_ -= step;
    if (adpcm_busy_cycles_ < 0) {
      adpcm_busy_cycles_ = 0;
    }
  }

  if (pending_second_.active) {
    pending_second_.delay -= step;
    if (pending_second_.delay <= 0) {
      pending_second_.active = false;
      enqueue_irq(pending_second_.irq, std::move(pending_second_.response), false);
      pending_second_.response.clear();
    }
  }

  if (insert_probe_active_) {
    insert_probe_delay_cycles_ -= step;
    if (insert_probe_delay_cycles_ <= 0) {
      switch (insert_probe_stage_) {
      case 0:
        execute_internal_command(0x19u, {0x20u});
        break;
      case 1:
        execute_internal_command(0x01u, {});
        break;
      case 2:
        execute_internal_command(0x01u, {});
        break;
      default:
        insert_probe_active_ = false;
        break;
      }

      if (insert_probe_active_) {
        ++insert_probe_stage_;
        insert_probe_delay_cycles_ = 4000;
        if (insert_probe_stage_ > 2) {
          insert_probe_active_ = false;
        }
      }
    }
  }

  if (state_ == State::Seeking) {
    pending_cycles_ -= step;
    if (pending_cycles_ <= 0) {
      read_lba_ = seek_target_valid_ ? seek_target_lba_ : read_lba_;
      seek_complete_ = true;
      if (pending_read_start_) {
        pending_read_start_ = false;
        state_ = State::Reading;
        pending_cycles_ = std::max(1, read_period_cycles_);
      } else {
        state_ = State::Idle;
      }
    }
  }

  if (state_ == State::Reading) {
    int remaining = step;
    while (remaining > 0 && state_ == State::Reading) {
      if (pending_cycles_ > remaining) {
        pending_cycles_ -= remaining;
        break;
      }
      remaining -= std::max(0, pending_cycles_);
      pending_cycles_ = std::max(1, read_period_cycles_);

      bool ok = cdda_playing_ ? stream_cdda_sector() : read_sector();
      if (!ok) {
        seek_error_ = true;
        state_ = State::Idle;
        cdda_playing_ = false;
        enqueue_irq(5, make_error_response(stat_byte(), 0x04u), false);
        break;
      }

      ++sector_counter_;
      if (!cdda_playing_) {
        // Only fire INT1 when a real host-data sector landed in the buffer.
        // XA ADPCM sectors are consumed silently by maybe_decode_xa_audio()
        // and leave data_ready_ false. On real hardware those sectors do NOT
        // generate INT1; firing it with an empty buffer corrupts the game's
        // sector count and causes DMA reads to return zeros.
        if (data_ready_) {
          enqueue_irq(1, {stat_byte()}, false);
        }
      } else if ((mode_ & 0x04u) != 0) {
        const int abs_lba = read_lba_ + 150;
        if ((abs_lba % 10) == 0) {
          const CdTrack *track = track_for_lba(read_lba_);
          const int track_start = (track != nullptr) ? track->index01_abs_lba : 150;
          const int rel_lba = std::max(0, abs_lba - track_start);
          u8 rel_m = 0;
          u8 rel_s = 0;
          u8 rel_f = 0;
          u8 abs_m = 0;
          u8 abs_s = 0;
          u8 abs_f = 0;
          lba_to_msf_bcd(rel_lba, rel_m, rel_s, rel_f);
          lba_to_msf_bcd(abs_lba, abs_m, abs_s, abs_f);
          const bool rel = (abs_lba % 32) >= 16;
          const u8 track_no = to_bcd(clip_u8(
              std::clamp((track != nullptr) ? track->number : 1, 1, 99)));
          const u8 ss = rel ? static_cast<u8>(rel_s | 0x80u) : abs_s;
          const u8 mm = rel ? rel_m : abs_m;
          const u8 ff = rel ? rel_f : abs_f;
          enqueue_irq(1, {stat_byte(), track_no, 0x01u, mm, ss, ff, 0x00u, 0x00u},
                      false);
        }
      }

      const CdTrack *track = track_for_lba(read_lba_);
      const int end_lba = track_end_lba(track);
      if (end_lba >= 0 && read_lba_ >= end_lba) {
        if (cdda_playing_) {
          cdda_playing_ = false;
          state_ = State::Idle;
          if ((mode_ & 0x02u) == 0) {
            motor_on_ = false;
          }
          enqueue_irq(4, {stat_byte()}, false);
        } else {
          state_ = State::Idle;
        }
        break;
      }

      ++read_lba_;
    }
  }

  service_pending_irq();

  if (profile_detailed && sys_ != nullptr) {
    const auto t1 = std::chrono::high_resolution_clock::now();
    sys_->add_cdrom_time(
        std::chrono::duration<double, std::milli>(t1 - t0).count());
  }
}

u32 CdRom::dma_read() {
  if (!data_ready_ || !data_request_ ||
      data_index_ >= static_cast<int>(data_buffer_.size())) {
    return 0;
  }

  u32 value = 0;
  for (int i = 0; i < 4; ++i) {
    u8 byte = 0;
    if (data_index_ < static_cast<int>(data_buffer_.size())) {
      byte = data_buffer_[static_cast<size_t>(data_index_++)];
    }
    value |= static_cast<u32>(byte) << (i * 8u);
  }

  if (data_index_ >= static_cast<int>(data_buffer_.size())) {
    data_ready_ = false;
  }
  return value;
}
