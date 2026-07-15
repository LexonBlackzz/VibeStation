#pragma once
#include "types.h"
#include <deque>
#include <mutex>
#include <vector>

class System;

// A serialized emulator state snapshot, stored as a flat byte buffer.
// System::save_state() populates it; System::restore_state() consumes it.
struct SystemSnapshot {
  u64 frame_id = 0;
  std::vector<u8> data;
};

// Ring-buffer of SystemSnapshots for the rewind feature.
// Thread-safe: capture (push) from the emu thread, consume (pop) from the same
// thread during rewind-hold. External query (snapshot_count) is lock-free.
class RewindManager {
public:
  void init(int buffer_seconds, int fps);
  void clear();
  void push(SystemSnapshot &&snap);
  bool pop(SystemSnapshot &out);
  bool peek(SystemSnapshot &out) const;
  std::size_t snapshot_count() const;
  std::size_t memory_bytes() const;
  int buffer_seconds() const { return buffer_seconds_; }
  void set_buffer_seconds(int seconds, int fps);

private:
  mutable std::mutex mutex_;
  std::deque<SystemSnapshot> ring_;
  int buffer_seconds_ = 5;
  int fps_ = 60;
  std::size_t max_snapshots_ = 300; // buffer_seconds * fps
};
