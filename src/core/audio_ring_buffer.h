#pragma once

#include "types.h"

#include <array>
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
//     buffer is starved, a fixed snapshot of the last ~400 ms of audio is
//     looped until enough fresh audio has accumulated to resume cleanly.
//   - Callback reads use try-lock and never wait; a contested callback emits
//     silence while the producer retains normal mutex-protected writes.
//   - set_buffer_duration() changes the capacity threshold at runtime
//     WITHOUT dropping pending audio data.
// ============================================================================

class AudioRingBuffer {
public:
  struct PushResult {
    size_t pushed_samples = 0;
    size_t dropped_samples = 0;
    size_t peak_samples = 0;
    size_t queued_samples = 0;
  };

  struct ReadResult {
    size_t consumed_samples = 0;
    size_t discarded_samples = 0;
    size_t queue_samples_before = 0;
    size_t queue_samples_after = 0;
    size_t history_drained_samples = 0;
    u32 peak_sample_before_clamp = 0;
    size_t clipped_samples = 0;
    size_t read_pointer_discontinuities = 0;
    bool lock_contended = false;
  };

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
  // buffer. If full, it discards oldest samples to make room. Producer writes
  // may take the mutex; the SDL callback always uses a non-waiting try-lock.
  //
  // @param samples  Pointer to interleaved s16 sample data.
  // @param count    Number of samples to push (even 1 is valid).
  void push_samples(const s16 *samples, size_t count);

  // Append samples and enforce a latency watermark in one producer-side
  // critical section. If the queue crosses max_samples, oldest audio is
  // discarded until target_samples remains.
  PushResult push_samples_bounded(const s16 *samples, size_t count,
                                  size_t target_samples,
                                  size_t max_samples);

  // Drop the oldest unread live samples without invoking stutter synthesis.
  // Dropped audio is still recorded into the rolling history so the optional
  // stutter loop can keep evolving without adding playback latency.
  size_t discard_oldest_samples(size_t count);

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
  // latch a fixed loop from the rolling history buffer and keep outputting
  // that loop until enough fresh audio has accumulated to resume cleanly.
  //
  // @param out_buffer       Destination buffer (must hold at least
  //                         `requested_count` s16 values).
  // @param requested_count  Number of samples to produce.
  void read_samples(s16 *out_buffer, size_t requested_count);

  // DuckStation-style normal playback read. This never enters the stutter
  // loop: it consumes fresh frames when available, stretches a short partial
  // read over the requested output to avoid a hard silence splice, and emits
  // silence only when completely empty.
  void read_live_samples(s16 *out_buffer, size_t requested_count);

  // Force Source-style stutter output while consuming any newly generated
  // live audio into the rolling history. This keeps the loop evolving during
  // sustained emulation slowdown without playing delayed live audio.
  void read_stutter_samples(s16 *out_buffer, size_t requested_count);

  // Real-time callback read. This never waits for the producer: if the ring
  // lock is busy, or fewer samples are available, the missing output remains
  // silence and the result reports how many fresh samples were consumed.
  ReadResult try_read_samples(s16 *out_buffer, size_t requested_count,
                              size_t discard_before_read = 0);

  // Play the recent-output history without consuming live queued samples.
  // Used only during real starvation; like try_read_samples(), it never waits.
  ReadResult try_read_history_samples(s16 *out_buffer,
                                      size_t requested_count,
                                      size_t drain_live_samples = 0);

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

  void append_samples_locked(const s16 *samples, size_t count,
                             size_t &dropped_samples,
                             bool preserve_stutter_history = true);
  size_t discard_oldest_locked(size_t count,
                               bool preserve_stutter_history = true);
  void publish_available_locked() {
    available_samples_snapshot_.store(used_samples(),
                                      std::memory_order_release);
  }

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

  // Copy `count` samples from the rolling history buffer starting at
  // `history_pos`. Handles wrap-around. Must be called under mutex_.
  void copy_from_history(size_t history_pos, s16 *dst, size_t count) const;

  // Copy `count` samples from the latched stutter loop and advance the
  // current loop position. Must be called under mutex_.
  void copy_from_stutter_loop(s16 *dst, size_t count);

  // Choose a rotation point in the rolling history that minimizes the jump
  // from the last played frame into the latched stutter loop.
  size_t choose_stutter_start_pos() const;

  // Rebuild the stutter loop from rolling history, optionally previewing
  // unread ring-buffer samples into the newest tail so heavy spiking can
  // keep evolving instead of freezing on a single snapshot.
  void refresh_stutter_loop(size_t preview_samples, bool prefer_clean_entry);

  // Track the last complete stereo frame written to the output so stutter
  // recovery can crossfade back into live audio without a hard edge.
  void update_last_output_frame(const s16 *samples, size_t count);

  // Blend the start of freshly resumed audio with the tail of the latched
  // stutter loop. Must be called under mutex_.
  void crossfade_from_stutter(s16 *samples, size_t count);

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
  std::atomic<size_t> available_samples_snapshot_{0};

  // Format.
  u32 sample_rate_ = DEFAULT_SAMPLE_RATE;
  u32 channels_ = DEFAULT_CHANNELS;

  // Rolling history buffer — always contains the most recent ~400 ms of
  // audio consumed from the ring buffer, used as the source for stutter
  // loop snapshots.
  std::vector<s16> history_buffer_;
  size_t history_write_ = 0;
  size_t history_valid_ = 0; // How many samples are actually filled.

  // Stutter state.
  std::atomic<bool> stutter_active_{false};
  std::vector<s16> stutter_loop_buffer_;
  size_t stutter_loop_pos_ = 0;
  size_t stutter_resume_threshold_ = 0;
  std::array<s16, DEFAULT_CHANNELS> last_output_frame_ = {};
  bool has_last_output_frame_ = false;
  bool history_loop_active_ = false;
  size_t history_loop_start_ = 0;
  size_t history_loop_length_ = 0;
  size_t history_loop_offset_ = 0;
};
