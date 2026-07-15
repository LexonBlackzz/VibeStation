#include "rewind.h"
#include <algorithm>

void RewindManager::init(int buffer_seconds, int fps) {
  std::lock_guard<std::mutex> lock(mutex_);
  buffer_seconds_ = std::max(1, std::min(10, buffer_seconds));
  fps_ = std::max(1, fps);
  max_snapshots_ = static_cast<std::size_t>(buffer_seconds_) *
                   static_cast<std::size_t>(fps_);
  ring_.clear();
}

void RewindManager::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  ring_.clear();
}

void RewindManager::push(SystemSnapshot &&snap) {
  std::lock_guard<std::mutex> lock(mutex_);
  ring_.push_back(std::move(snap));
  while (ring_.size() > max_snapshots_) {
    ring_.pop_front();
  }
}

bool RewindManager::pop(SystemSnapshot &out) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (ring_.empty()) {
    return false;
  }
  out = std::move(ring_.back());
  ring_.pop_back();
  return true;
}

bool RewindManager::peek(SystemSnapshot &out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (ring_.empty()) {
    return false;
  }
  out = ring_.back();
  return true;
}

std::size_t RewindManager::snapshot_count() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return ring_.size();
}

std::size_t RewindManager::memory_bytes() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::size_t total = 0;
  for (const auto &snap : ring_) {
    total += snap.data.size() + sizeof(SystemSnapshot);
  }
  return total;
}

void RewindManager::set_buffer_seconds(int seconds, int fps) {
  std::lock_guard<std::mutex> lock(mutex_);
  buffer_seconds_ = std::max(1, std::min(10, seconds));
  fps_ = std::max(1, fps);
  max_snapshots_ = static_cast<std::size_t>(buffer_seconds_) *
                   static_cast<std::size_t>(fps);
  while (ring_.size() > max_snapshots_) {
    ring_.pop_front();
  }
}
