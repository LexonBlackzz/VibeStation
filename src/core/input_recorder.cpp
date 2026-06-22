#include "input_recorder.h"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace {
constexpr u32 kFormatVersion = 1;
constexpr char kMagicHeader[] = "VIBESTATION_INPUT_V1";
constexpr u32 kMaxFramesInMemory = 1000000; // ~5.5 hours at 60fps

std::string get_timestamp_string() {
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf;
#ifdef _WIN32
  localtime_s(&tm_buf, &time_t_now);
#else
  localtime_r(&time_t_now, &tm_buf);
#endif
  std::ostringstream oss;
  oss << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S");
  return oss.str();
}

std::string get_filename_from_path(const std::string& path) {
  return std::filesystem::path(path).filename().string();
}

std::filesystem::path find_repository_root() {
  std::error_code ec;
  std::filesystem::path current = std::filesystem::absolute(
      std::filesystem::current_path(), ec);
  if (ec) {
    current = std::filesystem::current_path();
  }

  for (std::filesystem::path candidate = current; !candidate.empty();
       candidate = candidate.parent_path()) {
    if (std::filesystem::exists(candidate / ".git") &&
        std::filesystem::exists(candidate / "CMakeLists.txt")) {
      return candidate;
    }
    if (candidate == candidate.parent_path()) {
      break;
    }
  }
  return current;
}

std::string default_replay_folder() {
  const std::filesystem::path repository_root = find_repository_root();
  return std::filesystem::absolute(repository_root.parent_path() / "replays")
      .lexically_normal().string();
}

std::string resolve_replay_path(const std::string& path_or_name,
                                const std::string& replay_folder) {
  if (path_or_name.empty()) {
    return {};
  }

  std::filesystem::path path(path_or_name);
  if (!path.is_absolute()) {
    path = std::filesystem::path(replay_folder) / path;
  }
  if (!path.has_extension()) {
    path += ".vimovie";
  }
  return std::filesystem::absolute(path).lexically_normal().string();
}
}

InputRecorder::InputRecorder()
    : replay_folder_path_(default_replay_folder()) {}

void InputRecorder::set_config(const Config& config) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode_ != Mode::Idle) {
    last_error_ = "Stop the active input movie before changing its configuration.";
    status_message_ = last_error_;
    fprintf(stderr, "[InputRecorder] %s\n", last_error_.c_str());
    return;
  }
  config_ = config;
  config_.record_path =
      resolve_replay_path(config.record_path, replay_folder_path_);
  config_.playback_path =
      resolve_replay_path(config.playback_path, replay_folder_path_);
  if (!config_.record_path.empty()) {
    fprintf(stdout, "[InputRecorder] Resolved record path: %s\n",
            config_.record_path.c_str());
  }
  if (!config_.playback_path.empty()) {
    fprintf(stdout, "[InputRecorder] Resolved playback path: %s\n",
            config_.playback_path.c_str());
  }
}

InputRecorder::Config InputRecorder::config() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return config_;
}

void InputRecorder::set_end_behavior(PlaybackEndBehavior behavior) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_.end_behavior = behavior;
}

InputRecorder::~InputRecorder() {
  shutdown();
}

bool InputRecorder::init(const std::string& disc_path, 
                         const std::string& emulator_version) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode_ != Mode::Idle) {
    last_error_ = "Stop the active input movie before starting another one.";
    status_message_ = last_error_;
    fprintf(stderr, "[InputRecorder] %s\n", last_error_.c_str());
    return false;
  }

  close_files();
  playback_frames_.clear();
  reset_session_state();

  if (config_.recording_enabled() && config_.playback_enabled()) {
    set_error("Cannot record and playback simultaneously.");
    return false;
  }

  if (config_.recording_enabled()) {
    if (!open_recording_file(disc_path, emulator_version)) {
      return false;
    }
    mode_ = Mode::Recording;
    state_ = State::Recording;
    status_message_ = "Recording input movie.";
    fprintf(stdout, "[InputRecorder] Recording started: %s\n", 
            config_.record_path.c_str());
    return true;
  }

  if (config_.playback_enabled()) {
    if (!open_playback_file()) {
      return false;
    }
    mode_ = Mode::Playing;
    state_ = State::Playing;
    status_message_ = "Playing input movie.";
    fprintf(stdout, "[InputRecorder] Playback started: %s (%llu frames loaded)\n",
            config_.playback_path.c_str(), 
            static_cast<unsigned long long>(playback_total_frames_));
    return true;
  }

  state_ = State::Idle;
  status_message_ = "Idle";
  return true;
}

void InputRecorder::shutdown() {
  std::lock_guard<std::mutex> lock(mutex_);
  shutdown_locked();
}

void InputRecorder::shutdown_locked() {
  const Mode previous_mode = mode_;
  if (mode_ == Mode::Recording && recorded_frame_count_ > 0) {
    fprintf(stdout, "[InputRecorder] Recording finished: %llu frames saved to %s\n",
            static_cast<unsigned long long>(recorded_frame_count_),
            config_.record_path.c_str());
  }
  
  if (mode_ == Mode::Playing && playback_frame_count_ > 0) {
    fprintf(stdout, "[InputRecorder] Playback finished: %llu frames replayed\n",
            static_cast<unsigned long long>(playback_frame_count_));
  }

  if (snapshots_saved_ > 0) {
    fprintf(stdout, "[InputRecorder] Saved %u snapshots to %s\n",
            snapshots_saved_, config_.snapshot_output_dir.c_str());
  }

  close_files();
  playback_frames_.clear();
  mode_ = Mode::Idle;
  if (previous_mode != Mode::Idle) {
    state_ = State::Finished;
    status_message_ = previous_mode == Mode::Recording
        ? "Recording stopped."
        : "Playback stopped.";
  }
}

void InputRecorder::reset_session_state() {
  recorded_frame_count_ = 0;
  playback_frame_count_ = 0;
  playback_current_index_ = 0;
  playback_total_frames_ = 0;
  playback_finished_ = false;
  playback_eof_reached_ = false;
  current_emulated_frame_ = 0;
  next_snapshot_index_ = 0;
  snapshots_saved_ = 0;
  last_error_.clear();
  status_message_ = "Idle";
  state_ = State::Idle;
}

void InputRecorder::set_error(const std::string& message) {
  state_ = State::Error;
  last_error_ = message;
  status_message_ = message;
  fprintf(stderr, "[InputRecorder] %s\n", message.c_str());
}

bool InputRecorder::open_recording_file(const std::string& disc_path,
                                        const std::string& emulator_version) {
  std::error_code ec;
  const std::filesystem::path parent =
      std::filesystem::path(config_.record_path).parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent, ec);
  }
  if (ec) {
    set_error("Failed to create recording directory: " + parent.string());
    return false;
  }

  record_file_ = fopen(config_.record_path.c_str(), "w");
  if (!record_file_) {
    set_error("Failed to open recording file: " + config_.record_path);
    return false;
  }

  if (!write_header(disc_path, emulator_version)) {
    fclose(record_file_);
    record_file_ = nullptr;
    return false;
  }

  return true;
}

bool InputRecorder::open_playback_file() {
  playback_file_ = fopen(config_.playback_path.c_str(), "r");
  if (!playback_file_) {
    set_error("Failed to open playback file: " + config_.playback_path);
    return false;
  }

  if (!read_header()) {
    fclose(playback_file_);
    playback_file_ = nullptr;
    return false;
  }

  // Read all frames into memory for fast access
  playback_frames_.clear();
  playback_frames_.reserve(10000); // Reserve for ~3 minutes at 60fps

  char line[256];
  while (fgets(line, sizeof(line), playback_file_)) {
    // Skip comments and empty lines
    if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
      continue;
    }

    unsigned long long frame_index = 0;
    u32 button_state = 0;
    u32 lx = 0x80;
    u32 ly = 0x80;
    u32 rx = 0x80;
    u32 ry = 0x80;
    const int parsed = sscanf(line, "%llu %x %x %x %x %x",
                              &frame_index, &button_state,
                              &lx, &ly, &rx, &ry);
    if (parsed == 2 || parsed == 6) {
      if (frame_index != playback_frames_.size()) {
        set_error("Non-sequential playback frame " +
                  std::to_string(frame_index) + " (expected " +
                  std::to_string(playback_frames_.size()) + ").");
        close_files();
        playback_frames_.clear();
        return false;
      }
      
      if (playback_frames_.size() >= kMaxFramesInMemory) {
        set_error("Playback file exceeds the frame limit of " +
                  std::to_string(kMaxFramesInMemory) + ".");
        close_files();
        playback_frames_.clear();
        return false;
      }

      if (button_state > 0xFFFFu || lx > 0xFFu || ly > 0xFFu ||
          rx > 0xFFu || ry > 0xFFu) {
        set_error("Playback frame contains an out-of-range pad value.");
        close_files();
        playback_frames_.clear();
        return false;
      }

      ControllerState state{};
      state.buttons = static_cast<u16>(button_state);
      state.lx = static_cast<u8>(lx);
      state.ly = static_cast<u8>(ly);
      state.rx = static_cast<u8>(rx);
      state.ry = static_cast<u8>(ry);
      playback_frames_.push_back(state);
    } else {
      set_error("Malformed playback frame line.");
      close_files();
      playback_frames_.clear();
      return false;
    }
  }

  if (playback_frames_.empty()) {
    set_error("No valid frames found in playback file.");
    fclose(playback_file_);
    playback_file_ = nullptr;
    return false;
  }

  playback_total_frames_ = playback_frames_.size();
  fclose(playback_file_);
  playback_file_ = nullptr;

  return true;
}

void InputRecorder::close_files() {
  if (record_file_) {
    fflush(record_file_);
    fclose(record_file_);
    record_file_ = nullptr;
  }
  
  if (playback_file_) {
    fclose(playback_file_);
    playback_file_ = nullptr;
  }
}

bool InputRecorder::write_header(const std::string& disc_path,
                                 const std::string& emulator_version) {
  if (!record_file_) {
    return false;
  }

  // Write magic header
  fprintf(record_file_, "%s\n", kMagicHeader);
  
  // Write metadata
  fprintf(record_file_, "# Format Version: %u\n", kFormatVersion);
  fprintf(record_file_, "# Timestamp: %s\n", get_timestamp_string().c_str());
  
  if (!disc_path.empty()) {
    fprintf(record_file_, "# Disc: %s\n", get_filename_from_path(disc_path).c_str());
  }
  
  if (!emulator_version.empty()) {
    fprintf(record_file_, "# Emulator: %s\n", emulator_version.c_str());
  }
  
  fprintf(record_file_,
          "# Format: frame_index buttons_hex lx_hex ly_hex rx_hex ry_hex\n");
  fprintf(record_file_,
          "# Pad state format: controller 1 raw active-low PS1 buttons plus raw 8-bit analog bytes\n");
  fprintf(record_file_, "# Neutral/all released: 0xFFFF\n");
  fprintf(record_file_, "#\n");
  
  if (fflush(record_file_) != 0 || ferror(record_file_)) {
    set_error("Failed to write replay header: " + config_.record_path);
    return false;
  }
  return true;
}

bool InputRecorder::read_header() {
  if (!playback_file_) {
    return false;
  }

  char line[256];
  
  // Read and verify magic header
  if (!fgets(line, sizeof(line), playback_file_)) {
    set_error("Failed to read replay magic header.");
    return false;
  }

  // Remove trailing newline
  size_t len = strlen(line);
  if (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
    line[len - 1] = '\0';
    if (len > 1 && line[len - 2] == '\r') {
      line[len - 2] = '\0';
    }
  }

  if (strcmp(line, kMagicHeader) != 0) {
    set_error("Invalid replay file format (bad magic header).");
    return false;
  }

  // The frame reader skips human-readable metadata lines. Leaving the stream
  // immediately after the magic avoids text-mode seek ambiguity on Windows.
  return true;
}
void InputRecorder::record_frame_locked(
    u64 frame_index, const ControllerState& controller_state) {
  if (mode_ != Mode::Recording || !record_file_) {
    return;
  }

  if (fprintf(record_file_, "%llu %04X %02X %02X %02X %02X\n",
              static_cast<unsigned long long>(frame_index),
              controller_state.buttons, controller_state.lx,
              controller_state.ly, controller_state.rx,
              controller_state.ry) < 0) {
    close_files();
    mode_ = Mode::Idle;
    set_error("Failed while writing replay frame data.");
    return;
  }
  ++recorded_frame_count_;

  // Flush every 60 frames (~1 second at 60fps) for safety
  if (recorded_frame_count_ % 60 == 0) {
    if (fflush(record_file_) != 0) {
      close_files();
      mode_ = Mode::Idle;
      set_error("Failed while flushing replay frame data.");
    }
  }
}

void InputRecorder::record_frame(u64 frame_index, u16 button_state) {
  ControllerState state{};
  state.buttons = button_state;
  record_frame(frame_index, state);
}

void InputRecorder::record_frame(
    u64 frame_index, const ControllerState& controller_state) {
  std::lock_guard<std::mutex> lock(mutex_);
  current_emulated_frame_ = frame_index;
  record_frame_locked(frame_index, controller_state);
}

InputRecorder::FrameResult InputRecorder::playback_frame_locked(u64 frame_index) {
  FrameResult result{};
  if (mode_ != Mode::Playing) {
    return result;
  }
  result.override_input = true;

  if (playback_finished_) {
    switch (config_.end_behavior) {
      case PlaybackEndBehavior::HoldNeutral:
        result.controller = {};
        return result;
      case PlaybackEndBehavior::Stop:
        result.controller = {};
        result.stop_requested = true;
        return result;
      case PlaybackEndBehavior::Loop:
        playback_finished_ = false;
        break;
    }
  }

  // Check if we've reached the end of recorded frames
  if (playback_current_index_ >= playback_frames_.size()) {
    if (!playback_eof_reached_) {
      playback_eof_reached_ = true;
      fprintf(stdout, "[InputRecorder] Playback reached end at frame %llu\n",
              static_cast<unsigned long long>(frame_index));
    }

    switch (config_.end_behavior) {
      case PlaybackEndBehavior::HoldNeutral:
        playback_finished_ = true;
        state_ = State::Eof;
        status_message_ = "Playback reached EOF; holding neutral input.";
        result.controller = {};
        return result;
        
      case PlaybackEndBehavior::Stop:
        playback_finished_ = true;
        state_ = State::Finished;
        status_message_ = "Playback reached EOF; stopping emulation.";
        result.controller = {};
        result.stop_requested = true;
        return result;
        
      case PlaybackEndBehavior::Loop:
        fprintf(stdout, "[InputRecorder] Looping playback from frame 0\n");
        playback_current_index_ = 0;
        playback_eof_reached_ = false;
        state_ = State::Playing;
        status_message_ = "Playback looped to frame 0.";
        break;
    }
  }

  result.controller = playback_frames_[playback_current_index_];
  ++playback_current_index_;
  ++playback_frame_count_;
  return result;
}

InputRecorder::FrameResult InputRecorder::process_frame(
    u64 frame_index, const ControllerState& current_state) {
  std::lock_guard<std::mutex> lock(mutex_);
  current_emulated_frame_ = frame_index;
  if (mode_ == Mode::Recording) {
    record_frame_locked(frame_index, current_state);
    return {};
  }
  return playback_frame_locked(frame_index);
}

InputRecorder::FrameResult InputRecorder::process_frame(
    u64 frame_index, u16 current_button_state) {
  ControllerState state{};
  state.buttons = current_button_state;
  return process_frame(frame_index, state);
}

u16 InputRecorder::get_playback_input(u64 frame_index) {
  std::lock_guard<std::mutex> lock(mutex_);
  current_emulated_frame_ = frame_index;
  return playback_frame_locked(frame_index).controller.buttons;
}

InputRecorder::Status InputRecorder::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  Status result{};
  result.mode = mode_;
  result.state = state_;
  result.record_path = config_.record_path;
  result.playback_path = config_.playback_path;
  result.replay_folder_path = replay_folder_path_;
  result.current_emulated_frame = current_emulated_frame_;
  result.recorded_frame_count = recorded_frame_count_;
  result.playback_frame_count = playback_frame_count_;
  result.playback_current_index = playback_current_index_;
  result.playback_total_frames = playback_total_frames_;
  result.playback_finished = playback_finished_;
  result.playback_eof_reached = playback_eof_reached_;
  result.end_behavior = config_.end_behavior;
  result.last_error = last_error_;
  result.status_message = status_message_;
  return result;
}

InputRecorder::Mode InputRecorder::mode() const {
  return status().mode;
}

bool InputRecorder::is_recording() const {
  return mode() == Mode::Recording;
}

bool InputRecorder::is_playing() const {
  return mode() == Mode::Playing;
}

bool InputRecorder::playback_finished() const {
  return status().playback_finished;
}

u64 InputRecorder::recorded_frame_count() const {
  return status().recorded_frame_count;
}

u64 InputRecorder::playback_frame_count() const {
  return status().playback_frame_count;
}

const char* InputRecorder::state_name(State state) {
  switch (state) {
    case State::Idle: return "Idle";
    case State::Recording: return "Recording";
    case State::Playing: return "Playing";
    case State::Eof: return "EOF";
    case State::Finished: return "Finished";
    case State::Error: return "Error";
  }
  return "Unknown";
}

const char* InputRecorder::end_behavior_name(PlaybackEndBehavior behavior) {
  switch (behavior) {
    case PlaybackEndBehavior::HoldNeutral: return "Hold neutral after EOF";
    case PlaybackEndBehavior::Stop: return "Stop emulation at EOF";
    case PlaybackEndBehavior::Loop: return "Loop to frame 0";
  }
  return "Unknown";
}

void InputRecorder::check_snapshot(u32 frame_index, const void* framebuffer,
                                   u32 width, u32 height, bool is_24bit) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!config_.snapshots_enabled() || !framebuffer) {
    return;
  }

  bool should_snapshot = false;

  // Check interval-based snapshots
  if (config_.snapshot_interval_frames > 0) {
    if (frame_index > 0 && frame_index % config_.snapshot_interval_frames == 0) {
      should_snapshot = true;
    }
  }

  // Check specific frame snapshots
  if (!config_.snapshot_at_frames.empty()) {
    if (next_snapshot_index_ < config_.snapshot_at_frames.size()) {
      u32 target_frame = config_.snapshot_at_frames[next_snapshot_index_];
      if (frame_index == target_frame) {
        should_snapshot = true;
        ++next_snapshot_index_;
      }
    }
  }

  if (should_snapshot) {
    save_snapshot(frame_index, framebuffer, width, height, is_24bit);
  }
}

void InputRecorder::save_snapshot(u32 frame_index, const void* framebuffer,
                                  u32 width, u32 height, bool is_24bit) {
  if (!framebuffer || width == 0 || height == 0) {
    return;
  }

  // Create output directory if it doesn't exist
  try {
    std::filesystem::create_directories(config_.snapshot_output_dir);
  } catch (const std::exception& e) {
    fprintf(stderr, "[InputRecorder] Failed to create snapshot directory: %s\n", e.what());
    return;
  }

  // Generate filename: snapshot_frame_NNNNNN.ppm
  std::ostringstream filename;
  filename << "snapshot_frame_" << std::setfill('0') << std::setw(6) << frame_index << ".ppm";
  
  std::filesystem::path output_path = 
    std::filesystem::path(config_.snapshot_output_dir) / filename.str();

  FILE* file = fopen(output_path.string().c_str(), "wb");
  if (!file) {
    fprintf(stderr, "[InputRecorder] Failed to open snapshot file: %s\n", 
            output_path.string().c_str());
    return;
  }

  // Write PPM header (P6 format - binary RGB)
  fprintf(file, "P6\n%u %u\n255\n", width, height);

  // Convert and write pixel data
  const u8* src = static_cast<const u8*>(framebuffer);
  
  if (is_24bit) {
    // 24-bit RGB - write directly (assuming BGR order, convert to RGB)
    std::vector<u8> row_buffer(width * 3);
    for (u32 y = 0; y < height; ++y) {
      for (u32 x = 0; x < width; ++x) {
        u32 pixel_offset = (y * width + x) * 3;
        // Convert BGR to RGB
        row_buffer[x * 3 + 0] = src[pixel_offset + 2]; // R
        row_buffer[x * 3 + 1] = src[pixel_offset + 1]; // G
        row_buffer[x * 3 + 2] = src[pixel_offset + 0]; // B
      }
      fwrite(row_buffer.data(), 1, width * 3, file);
    }
  } else {
    // 16-bit RGB555 - convert to 24-bit RGB
    const u16* src16 = static_cast<const u16*>(framebuffer);
    std::vector<u8> row_buffer(width * 3);
    
    for (u32 y = 0; y < height; ++y) {
      for (u32 x = 0; x < width; ++x) {
        u16 pixel = src16[y * width + x];
        
        // Extract RGB555 components (assuming little endian)
        u8 r = static_cast<u8>((pixel & 0x1F) << 3);
        u8 g = static_cast<u8>(((pixel >> 5) & 0x1F) << 3);
        u8 b = static_cast<u8>(((pixel >> 10) & 0x1F) << 3);
        
        // Expand 5-bit to 8-bit (replicate top bits to bottom)
        r |= (r >> 5);
        g |= (g >> 5);
        b |= (b >> 5);
        
        row_buffer[x * 3 + 0] = r;
        row_buffer[x * 3 + 1] = g;
        row_buffer[x * 3 + 2] = b;
      }
      fwrite(row_buffer.data(), 1, width * 3, file);
    }
  }

  fclose(file);
  ++snapshots_saved_;

  fprintf(stdout, "[InputRecorder] Saved snapshot: %s\n", 
          output_path.string().c_str());
}
