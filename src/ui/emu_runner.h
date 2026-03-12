#pragma once

#include "../core/system.h"
#include "../core/types.h"
#include <array>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct FrameSnapshot {
  u64 frame_id = 0;
  int width = 0;
  int height = 0;
  std::vector<u32> rgba;
};

class EmuRunner {
public:
  struct RuntimeSnapshot {
    u64 frame_id = 0;
    bool running = false;
    System::BootDiagnostics boot_diag{};
    System::ProfilingStats profiling{};
    double core_frame_ms = 0.0;
    Spu::AudioDiag spu_audio{};
    std::array<s16, 24> spu_voice_level_l{};
    std::array<s16, 24> spu_voice_level_r{};
    std::array<bool, 24> spu_voice_active{};
    u32 spu_endx_mask = 0;
    std::array<bool, 2> memory_card_inserted{};
    std::array<bool, 2> memory_card_dirty{};
    std::array<std::string, 2> memory_card_path{};
  };

  ~EmuRunner();

  bool start(System *system);
  void stop();

  void set_running(bool running);
  void pause_and_wait_idle();
  bool is_running() const { return running_.load(std::memory_order_acquire); }
  void set_vram_debug_capture_enabled(bool enabled) {
    capture_vram_debug_.store(enabled, std::memory_order_release);
  }

  void set_speed(double speed);
  double speed() const { return speed_.load(std::memory_order_acquire); }

  void set_input_state(u16 buttons, u8 lx, u8 ly, u8 rx, u8 ry);
  void request_live_disc_insert(const std::string &bin_path,
                                const std::string &cue_path);
  void request_memory_card_paths(const std::array<std::string, 2> &slot_paths);
  bool consume_latest_frame(FrameSnapshot &out_frame);
  bool consume_latest_vram_snapshot(std::vector<u16> &out_vram);
  RuntimeSnapshot runtime_snapshot() const;

private:
  static u64 pack_input(u16 buttons, u8 lx, u8 ly, u8 rx, u8 ry);
  static void unpack_input(u64 packed, u16 &buttons, u8 &lx, u8 &ly, u8 &rx,
                           u8 &ry);
  void apply_input_state(System &system);
  void apply_pending_memory_card_paths();
  void apply_pending_disc_insert();
  bool should_capture_frame() const;
  void publish_frame(FrameSnapshot &&frame, const RuntimeSnapshot &snapshot);
  void publish_snapshot(const RuntimeSnapshot &snapshot);
  void worker_main();
  void wait_until_idle();

  System *system_ = nullptr;
  std::thread worker_;

  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> frame_active_{false};
  std::atomic<double> speed_{1.0};
  std::atomic<u64> input_mailbox_{0};

  mutable std::mutex control_mutex_;
  std::condition_variable control_cv_;

  mutable std::mutex idle_mutex_;
  std::condition_variable idle_cv_;

  mutable std::mutex frame_mutex_;
  FrameSnapshot pending_frame_{};
  bool has_pending_frame_ = false;
  mutable std::atomic<u32> fast_mode_capture_counter_{0};

  mutable std::mutex snapshot_mutex_;
  RuntimeSnapshot latest_snapshot_{};
  std::atomic<bool> capture_vram_debug_{false};
  mutable std::mutex vram_mutex_;
  std::vector<u16> latest_vram_snapshot_{};
  bool has_latest_vram_snapshot_ = false;

  mutable std::mutex disc_request_mutex_;
  bool has_pending_disc_request_ = false;
  std::string pending_disc_bin_path_;
  std::string pending_disc_cue_path_;

  mutable std::mutex memcard_request_mutex_;
  bool has_pending_memcard_request_ = false;
  std::array<std::string, 2> pending_memcard_paths_{};
};
