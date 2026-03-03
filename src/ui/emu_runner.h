#pragma once

#include "../core/system.h"
#include "../core/types.h"
#include <atomic>
#include <condition_variable>
#include <mutex>
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
  };

  ~EmuRunner();

  bool start(System *system);
  void stop();

  void set_running(bool running);
  void pause_and_wait_idle();
  bool is_running() const { return running_.load(std::memory_order_acquire); }

  void set_speed(double speed);
  double speed() const { return speed_.load(std::memory_order_acquire); }

  void set_input_state(u16 buttons, u8 lx, u8 ly, u8 rx, u8 ry);
  bool consume_latest_frame(FrameSnapshot &out_frame);
  RuntimeSnapshot runtime_snapshot() const;

private:
  static u64 pack_input(u16 buttons, u8 lx, u8 ly, u8 rx, u8 ry);
  static void unpack_input(u64 packed, u16 &buttons, u8 &lx, u8 &ly, u8 &rx,
                           u8 &ry);
  void apply_input_state(System &system);
  void publish_frame(FrameSnapshot &&frame, const RuntimeSnapshot &snapshot);
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

  mutable std::mutex snapshot_mutex_;
  RuntimeSnapshot latest_snapshot_{};
};

