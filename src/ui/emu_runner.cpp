#include "emu_runner.h"
#include <algorithm>
#include <chrono>

EmuRunner::~EmuRunner() { stop(); }

bool EmuRunner::start(System *system) {
  if (worker_.joinable() || system == nullptr) {
    return false;
  }

  system_ = system;
  stop_requested_.store(false, std::memory_order_release);
  running_.store(false, std::memory_order_release);
  frame_active_.store(false, std::memory_order_release);
  speed_.store(1.0, std::memory_order_release);
  input_mailbox_.store(pack_input(0xFFFFu, 0x80, 0x80, 0x80, 0x80),
                       std::memory_order_release);
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    pending_frame_ = {};
    has_pending_frame_ = false;
  }
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    latest_snapshot_ = {};
  }

  worker_ = std::thread(&EmuRunner::worker_main, this);
  return true;
}

void EmuRunner::stop() {
  if (!worker_.joinable()) {
    return;
  }

  running_.store(false, std::memory_order_release);
  stop_requested_.store(true, std::memory_order_release);
  control_cv_.notify_all();
  idle_cv_.notify_all();

  worker_.join();
  system_ = nullptr;

  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    pending_frame_ = {};
    has_pending_frame_ = false;
  }
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    latest_snapshot_.running = false;
  }
}

void EmuRunner::set_running(bool running) {
  running_.store(running, std::memory_order_release);
  if (system_ != nullptr) {
    system_->set_running(running);
  }
  control_cv_.notify_all();
  if (!running) {
    idle_cv_.notify_all();
  }
}

void EmuRunner::wait_until_idle() {
  std::unique_lock<std::mutex> lock(idle_mutex_);
  idle_cv_.wait(lock, [this] {
    return !frame_active_.load(std::memory_order_acquire);
  });
}

void EmuRunner::pause_and_wait_idle() {
  set_running(false);
  wait_until_idle();
}

void EmuRunner::set_speed(double speed) {
  const double clamped = std::max(0.25, std::min(speed, 4.0));
  speed_.store(clamped, std::memory_order_release);
}

void EmuRunner::set_input_state(u16 buttons, u8 lx, u8 ly, u8 rx, u8 ry) {
  input_mailbox_.store(pack_input(buttons, lx, ly, rx, ry),
                       std::memory_order_release);
}

bool EmuRunner::consume_latest_frame(FrameSnapshot &out_frame) {
  std::lock_guard<std::mutex> lock(frame_mutex_);
  if (!has_pending_frame_) {
    return false;
  }
  out_frame = std::move(pending_frame_);
  pending_frame_ = {};
  has_pending_frame_ = false;
  return true;
}

EmuRunner::RuntimeSnapshot EmuRunner::runtime_snapshot() const {
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return latest_snapshot_;
}

u64 EmuRunner::pack_input(u16 buttons, u8 lx, u8 ly, u8 rx, u8 ry) {
  u64 packed = 0;
  packed |= static_cast<u64>(buttons);
  packed |= static_cast<u64>(lx) << 16;
  packed |= static_cast<u64>(ly) << 24;
  packed |= static_cast<u64>(rx) << 32;
  packed |= static_cast<u64>(ry) << 40;
  return packed;
}

void EmuRunner::unpack_input(u64 packed, u16 &buttons, u8 &lx, u8 &ly, u8 &rx,
                             u8 &ry) {
  buttons = static_cast<u16>(packed & 0xFFFFu);
  lx = static_cast<u8>((packed >> 16) & 0xFFu);
  ly = static_cast<u8>((packed >> 24) & 0xFFu);
  rx = static_cast<u8>((packed >> 32) & 0xFFu);
  ry = static_cast<u8>((packed >> 40) & 0xFFu);
}

void EmuRunner::apply_input_state(System &system) {
  u16 buttons = 0xFFFFu;
  u8 lx = 0x80;
  u8 ly = 0x80;
  u8 rx = 0x80;
  u8 ry = 0x80;
  unpack_input(input_mailbox_.load(std::memory_order_acquire), buttons, lx, ly,
               rx, ry);
  system.sio().set_button_state(buttons);
  system.sio().set_analog_state(lx, ly, rx, ry);
}

void EmuRunner::publish_frame(FrameSnapshot &&frame,
                              const RuntimeSnapshot &snapshot) {
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    pending_frame_ = std::move(frame);
    has_pending_frame_ = true;
  }
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    latest_snapshot_ = snapshot;
  }
}

void EmuRunner::worker_main() {
  using steady_clock = std::chrono::steady_clock;
  auto next_tick = steady_clock::now();

  auto run_one_frame = [&]() {
    frame_active_.store(true, std::memory_order_release);

    FrameSnapshot frame{};
    RuntimeSnapshot snapshot{};

    apply_input_state(*system_);
    system_->run_frame(false);

    std::vector<u32> rgba;
    const DisplaySampleInfo sample = system_->gpu().build_display_rgba(rgba);
    system_->update_display_diag(sample);

    frame.frame_id = static_cast<u64>(system_->boot_diag().frame_counter);
    frame.width = std::max(1, sample.width);
    frame.height = std::max(1, sample.height);
    frame.rgba = std::move(rgba);

    snapshot.frame_id = frame.frame_id;
    snapshot.running = running_.load(std::memory_order_acquire);
    snapshot.boot_diag = system_->boot_diag();
    snapshot.profiling = system_->profiling_stats();
    snapshot.core_frame_ms = snapshot.profiling.total_ms;

    publish_frame(std::move(frame), snapshot);

    frame_active_.store(false, std::memory_order_release);
    idle_cv_.notify_all();
  };

  while (!stop_requested_.load(std::memory_order_acquire)) {
    std::unique_lock<std::mutex> control_lock(control_mutex_);
    control_cv_.wait(control_lock, [this] {
      return stop_requested_.load(std::memory_order_acquire) ||
             running_.load(std::memory_order_acquire);
    });
    control_lock.unlock();

    if (stop_requested_.load(std::memory_order_acquire)) {
      break;
    }
    if (!running_.load(std::memory_order_acquire) || system_ == nullptr) {
      continue;
    }
    next_tick = steady_clock::now();

    while (running_.load(std::memory_order_acquire) &&
           !stop_requested_.load(std::memory_order_acquire)) {
      const double speed = std::max(0.25, speed_.load(std::memory_order_acquire));
      const auto frame_period = std::chrono::duration_cast<steady_clock::duration>(
          std::chrono::duration<double>((1.0 / system_->target_fps()) / speed));
      const auto now = steady_clock::now();
      if (now < next_tick) {
        const auto remain = next_tick - now;
        if (remain > std::chrono::milliseconds(1)) {
          std::this_thread::sleep_for(remain - std::chrono::microseconds(500));
        } else {
          std::this_thread::yield();
        }
        continue;
      }

      int frames_to_run = 1;
      const double lag_sec =
          std::chrono::duration<double>(now - next_tick).count();
      const double period_sec = std::chrono::duration<double>(frame_period).count();
      if (period_sec > 0.0) {
        const int lag_frames = static_cast<int>(lag_sec / period_sec);
        frames_to_run = std::min(2, std::max(1, lag_frames + 1));
      }

      for (int i = 0;
           i < frames_to_run && running_.load(std::memory_order_acquire) &&
           !stop_requested_.load(std::memory_order_acquire);
           ++i) {
        run_one_frame();
        next_tick += frame_period;
      }

      const auto after = steady_clock::now();
      if (after > next_tick + frame_period * 4) {
        // Clamp long hitches to avoid long catch-up spirals.
        next_tick = after;
      }
    }
  }

  frame_active_.store(false, std::memory_order_release);
  idle_cv_.notify_all();
}
