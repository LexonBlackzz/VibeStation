#pragma once

#include "types.h"
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>

// Frame-based input recording and playback for deterministic testing.
// Records controller 1 state once per emulated video frame.

class InputRecorder {
public:
  enum class Mode {
    Idle,
    Recording,
    Playing
  };

  enum class PlaybackEndBehavior {
    HoldNeutral,
    Stop,
    Loop
  };

  enum class State {
    Idle,
    Recording,
    Playing,
    Eof,
    Finished,
    Error
  };

  struct Config {
    std::string record_path;
    std::string playback_path;
    PlaybackEndBehavior end_behavior = PlaybackEndBehavior::HoldNeutral;
    
    // Snapshot configuration
    std::string snapshot_output_dir;
    std::vector<u32> snapshot_at_frames;
    u32 snapshot_interval_frames = 0;
    
    bool recording_enabled() const { return !record_path.empty(); }
    bool playback_enabled() const { return !playback_path.empty(); }
    bool snapshots_enabled() const {
      return !snapshot_output_dir.empty() &&
             (snapshot_interval_frames > 0 || !snapshot_at_frames.empty());
    }
  };

  struct FrameResult {
    bool override_input = false;
    struct ControllerState {
      u16 buttons = 0xFFFF;
      u8 lx = 0x80;
      u8 ly = 0x80;
      u8 rx = 0x80;
      u8 ry = 0x80;
    } controller;
    bool stop_requested = false;
  };

  using ControllerState = FrameResult::ControllerState;

  struct Status {
    Mode mode = Mode::Idle;
    State state = State::Idle;
    std::string record_path;
    std::string playback_path;
    std::string replay_folder_path;
    u64 current_emulated_frame = 0;
    u64 recorded_frame_count = 0;
    u64 playback_frame_count = 0;
    u64 playback_current_index = 0;
    u64 playback_total_frames = 0;
    bool playback_finished = false;
    bool playback_eof_reached = false;
    PlaybackEndBehavior end_behavior = PlaybackEndBehavior::HoldNeutral;
    std::string last_error;
    std::string status_message;
  };

  InputRecorder();
  ~InputRecorder();

  // Configuration (call before init)
  void set_config(const Config& config);
  Config config() const;
  void set_end_behavior(PlaybackEndBehavior behavior);

  // Initialize recording or playback
  bool init(const std::string& disc_path, const std::string& emulator_version);
  
  // Shutdown and flush
  void shutdown();

  // Called exactly once at the start of each emulated frame. Recording stores
  // the raw controller 1 state supplied by SIO. Playback sets override_input
  // even when the active-low button word is neutral (0xFFFF).
  FrameResult process_frame(u64 frame_index,
                            const ControllerState& current_state);
  FrameResult process_frame(u64 frame_index, u16 current_button_state);

  // Recording API - call once per emulated frame
  void record_frame(u64 frame_index, u16 button_state);
  void record_frame(u64 frame_index, const ControllerState& controller_state);

  // Playback API - call once per emulated frame
  // Compatibility helper. Callers must use is_playing() to decide whether the
  // returned value overrides live input; 0xFFFF is a valid movie frame.
  u16 get_playback_input(u64 frame_index);

  // Snapshot API - call after frame rendering
  void check_snapshot(u32 frame_index, const void* framebuffer, 
                      u32 width, u32 height, bool is_24bit);

  // State queries
  Status status() const;
  Mode mode() const;
  bool is_recording() const;
  bool is_playing() const;
  bool playback_finished() const;
  u64 recorded_frame_count() const;
  u64 playback_frame_count() const;
  static const char* state_name(State state);
  static const char* end_behavior_name(PlaybackEndBehavior behavior);

private:
  bool open_recording_file(const std::string& disc_path, 
                           const std::string& emulator_version);
  bool open_playback_file();
  void close_files();
  void reset_session_state();
  void set_error(const std::string& message);
  void shutdown_locked();
  void record_frame_locked(u64 frame_index,
                           const ControllerState& controller_state);
  FrameResult playback_frame_locked(u64 frame_index);
  
  bool write_header(const std::string& disc_path, 
                    const std::string& emulator_version);
  bool read_header();
  
  void save_snapshot(u32 frame_index, const void* framebuffer,
                     u32 width, u32 height, bool is_24bit);

  Config config_;
  std::string replay_folder_path_;
  mutable std::mutex mutex_;
  Mode mode_ = Mode::Idle;
  State state_ = State::Idle;
  u64 current_emulated_frame_ = 0;
  std::string last_error_;
  std::string status_message_ = "Idle";
  
  // Recording state
  FILE* record_file_ = nullptr;
  u64 recorded_frame_count_ = 0;
  
  // Playback state  
  FILE* playback_file_ = nullptr;
  std::vector<ControllerState> playback_frames_;
  u64 playback_frame_count_ = 0;
  u64 playback_current_index_ = 0;
  u64 playback_total_frames_ = 0;
  bool playback_finished_ = false;
  bool playback_eof_reached_ = false;
  
  // Snapshot state
  u32 next_snapshot_index_ = 0;
  u32 snapshots_saved_ = 0;
};
