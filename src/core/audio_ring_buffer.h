#pragma once

#include "types.h"

#include <atomic>
#include <cstddef>
#include <mutex>
#include <vector>

// ============================================================================
// AudioRingBuffer
// ============================================================================
//
// Thread-safe ring buffer bridging the PS1 SPU emulation thread (producer)
// and the host SDL audio callback thread (consumer).
//
// Features:
//   - Configurable buffer duration and sample rate via constructor / init().
//   - Hardcoded to stereo 16-bit (s16) matching the PS1 SPU output format.
//   - "Source Engine" lag-stutter: when the emulator drops frames and the
//     buffer is starved, a rolling history buffer of the last ~400 ms of
//     audio is looped/repeated to produce a glitchy retro stutter instead
//     of dead silence.
//   - Push never blocks: the SPU thread can push even 1 sample at a time
//     during a massive frame freeze without overflowing.
//   - set_buffer_duration() changes the capacity threshold at runtime
//     WITHOUT dropping pending audio data.
// ============================================================================

class AudioRingBuffer {
public:
  // Defaults tuned for PS1: 44.1 kHz, stereo, 400 ms buffer.
  static constexpr u32 DEFAULT_SAMPLE_RATE = 44100;
  static constexpr u32 DEFAULT_CHANNELS = 2;
  static constexpr double DEFAULT_BUFFER_SECONDS = 0.4; // 400 ms

  // Duration of rolling audio history kept for stutter looping.
  // 400 ms of stereo at 44100 Hz ≈ 35 280 s16 values — enough to
  // avoid a high-pitched whine when stuttering from a tiny drain.
  static constexpr double HISTORY_BUFFER_SECONDS = 0.4;

  // --------------------------------------------------------------------------
  // Construction / Initialization
  // --------------------------------------------------------------------------

  AudioRingBuffer();
  AudioRingBuffer(double buffer_seconds,
                  u32 sample_rate = DEFAULT_SAMPLE_RATE);

  // Configure the buffer. Allocates storage and resets all state.
  void init(double buffer_seconds, u32 sample_rate = DEFAULT_SAMPLE_RATE,
            u32 channels = DEFAULT_CHANNELS);

  // --------------------------------------------------------------------------
  // Dynamic Configuration (no data loss)
  // --------------------------------------------------------------------------

  // Change the target buffer duration at runtime. Resizes the backing
  // storage but preserves all currently pending samples — no pop or
  // click when the configuration changes mid-playback.
  void set_buffer_duration(double seconds);

  // --------------------------------------------------------------------------
  // Producer (SPU thread) — push_samples
  // --------------------------------------------------------------------------
  //
  // Append `count` interleaved stereo samples (L, R, L, R, …) to the ring
  // buffer. This function NEVER blocks: if the buffer is full it discards
  // the oldest samples to make room, ensuring the SPU thread can push at
  // any rate — even a single sample per second during a 7-second frame drop.
  //
  // @param samples  Pointer to interleaved s16 sample data.
  // @param count    Number of samples to push (even 1 is valid).
  void push_samples(const s16 *samples, size_t count);

  // --------------------------------------------------------------------------
  // Consumer (Host Audio thread) — read_samples
  // --------------------------------------------------------------------------
  //
  // Fill `out_buffer` with `requested_count` interleaved stereo samples.
  //
  // Normal path: if enough fresh samples are available, copy them and advance
  // the read pointer while recording them in the rolling history buffer.
  //
  // Stutter path: if fewer than `requested_count` samples are available,
  // drain what exists, then fill the remaining slots by looping the rolling
  // history buffer — a continuously-updated copy of the last ~400 ms of
  // audio. This produces the glitchy "Source Engine" lag stutter.
  //
  // @param out_buffer       Destination buffer (must hold at least
  //                         `requested_count` s16 values).
  // @param requested_count  Number of samples to produce.
  void read_samples(s16 *out_buffer, size_t requested_count);

  // --------------------------------------------------------------------------
  // Query helpers
  // --------------------------------------------------------------------------

  size_t available_samples() const;
  size_t capacity_samples() const { return capacity_; }
  double capacity_seconds() const;

  // Whether the stutter loop is currently active.
  bool is_stuttering() const {
    return stutter_active_.load(std::memory_order_acquire);
  }

  // Reset the buffer to empty without deallocating.
  void clear();

private:
  size_t samples_from_seconds(double seconds) const;

  // Resize the backing storage to `new_capacity + 1` slots, preserving
  // the currently valid samples. Must be called under mutex_.
  void resize_buffer(size_t new_capacity);

  size_t used_samples() const {
    // Standard ring buffer: one-slot waste sentinel.
    if (write_pos_ >= read_pos_) {
      return write_pos_ - read_pos_;
    }
    return buffer_.size() - (read_pos_ - write_pos_);
  }

  // Copy `count` samples starting at `ring_pos` from the ring buffer
  // into `dst`. Handles wrap-around. Must be called under mutex_.
  void copy_from_ring(size_t ring_pos, s16 *dst, size_t count) const;

  // Advance `ring_pos` by `count` positions, wrapping at buffer_.size().
  static size_t advance_pos(size_t ring_pos, size_t count, size_t buf_size) {
    ring_pos += count;
    if (ring_pos >= buf_size) ring_pos -= buf_size;
    return ring_pos;
  }

  // Append samples to the rolling history buffer. Must be called under
  // mutex_.
  void history_push(const s16 *samples, size_t count);

  // Synchronization.
  mutable std::mutex mutex_;

  // Ring buffer storage.
  std::vector<s16> buffer_;
  size_t capacity_ = 0; // Max usable samples (buffer_.size() - 1).
  size_t write_pos_ = 0;
  size_t read_pos_ = 0;

  // Format.
  u32 sample_rate_ = DEFAULT_SAMPLE_RATE;
  u32 channels_ = DEFAULT_CHANNELS;

  // Rolling history buffer — always contains the most recent ~400 ms of
  // audio consumed from the ring buffer, used as the source for stutter
  // looping.
  std::vector<s16> history_buffer_;
  size_t history_write_ = 0;
  size_t history_valid_ = 0; // How many samples are actually filled.

  // Stutter state.
  std::atomic<bool> stutter_active_{false};
  size_t stutter_index_ = 0; // Current position in the history loop.
};
