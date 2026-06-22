#include "audio_ring_buffer.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

// ============================================================================
// AudioRingBuffer — Implementation
// ============================================================================

AudioRingBuffer::AudioRingBuffer() = default;

AudioRingBuffer::AudioRingBuffer(double buffer_seconds, u32 sample_rate) {
  init(buffer_seconds, sample_rate);
}

// ---------------------------------------------------------------------------
// init()
// ---------------------------------------------------------------------------
//
// Allocate storage and reset all state. Any pending samples are lost.
// ---------------------------------------------------------------------------
void AudioRingBuffer::init(double buffer_seconds, u32 sample_rate,
                           u32 channels) {
  std::lock_guard<std::mutex> lock(mutex_);

  sample_rate_ = sample_rate;
  channels_ = channels;
  capacity_ = samples_from_seconds(buffer_seconds);

  // One extra slot for the empty/full sentinel.
  buffer_.resize(capacity_ + 1);
  write_pos_ = 0;
  read_pos_ = 0;
  available_samples_snapshot_.store(0, std::memory_order_release);

  // Allocate the rolling history buffer.
  history_buffer_.resize(
      std::max<size_t>(samples_from_seconds(HISTORY_BUFFER_SECONDS), 1));
  history_write_ = 0;
  history_valid_ = 0;

  // Reset stutter state.
  stutter_active_ = false;
  stutter_loop_buffer_.clear();
  stutter_loop_pos_ = 0;
  stutter_resume_threshold_ = 0;
  last_output_frame_.fill(0);
  has_last_output_frame_ = false;
  history_loop_active_ = false;
  history_loop_start_ = 0;
  history_loop_length_ = 0;
  history_loop_offset_ = 0;
}

// ---------------------------------------------------------------------------
// set_buffer_duration()
// ---------------------------------------------------------------------------
//
// Change the capacity target at runtime WITHOUT destroying pending audio.
// The backing vector is reallocated, but all currently valid samples
// between read_pos_ and write_pos_ are preserved and re-packed from
// position 0 in the new buffer. The stutter history is left untouched.
// ---------------------------------------------------------------------------
void AudioRingBuffer::set_buffer_duration(double seconds) {
  std::lock_guard<std::mutex> lock(mutex_);
  const size_t new_capacity = samples_from_seconds(seconds);
  if (new_capacity == capacity_) return;

  resize_buffer(new_capacity);
}

// ---------------------------------------------------------------------------
// resize_buffer() — internal helper
// ---------------------------------------------------------------------------
//
// Reallocates the backing vector to `new_capacity + 1` while preserving
// every currently-valid sample.  Safe to call with new_capacity smaller
// than the current used count — excess oldest samples are silently dropped
// (the newest ones are kept).
// ---------------------------------------------------------------------------
void AudioRingBuffer::resize_buffer(size_t new_capacity) {
  if (new_capacity == 0) return;

  const size_t old_size = buffer_.size();
  if (old_size == 0) {
    // Brand-new buffer, nothing to preserve.
    buffer_.resize(new_capacity + 1);
    capacity_ = new_capacity;
    write_pos_ = 0;
    read_pos_ = 0;
    return;
  }

  const size_t used = used_samples();
  const size_t keep = std::min(used, new_capacity);

  // Extract the samples we want to keep (the newest ones at the write end).
  std::vector<s16> saved(keep);
  if (keep > 0) {
    // The newest `keep` samples start `keep` positions before write_pos_.
    size_t src_pos;
    if (write_pos_ >= keep) {
      src_pos = write_pos_ - keep;
    } else {
      src_pos = old_size - (keep - write_pos_);
    }
    copy_from_ring(src_pos, saved.data(), keep);
  }

  // Resize and repack from position 0.
  buffer_.resize(new_capacity + 1);
  capacity_ = new_capacity;
  if (keep > 0) {
    std::memcpy(buffer_.data(), saved.data(), keep * sizeof(s16));
  }
  read_pos_ = 0;
  write_pos_ = keep;
  publish_available_locked();
}

// ---------------------------------------------------------------------------
// push_samples() — Producer (SPU thread)
// ---------------------------------------------------------------------------
//
// Append `count` interleaved s16 samples.  Zero minimum — even a single
// sample is accepted and immediately available for reading.
//
// Non-blocking guarantee: if the buffer is full the oldest unread samples
// are discarded by advancing read_pos_.
// ---------------------------------------------------------------------------
void AudioRingBuffer::push_samples(const s16 *samples, size_t count) {
  if (count == 0 || samples == nullptr) return;
  if (capacity_ == 0) return;

  std::lock_guard<std::mutex> lock(mutex_);

  size_t dropped_samples = 0;
  append_samples_locked(samples, count, dropped_samples);
  publish_available_locked();
}

AudioRingBuffer::PushResult AudioRingBuffer::push_samples_bounded(
    const s16 *samples, size_t count, size_t target_samples,
    size_t max_samples) {
  PushResult result{};
  if (count == 0 || samples == nullptr || capacity_ == 0) {
    return result;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  result.pushed_samples = count;

  target_samples = std::min(target_samples, capacity_);
  max_samples = std::min(std::max(max_samples, target_samples), capacity_);
  const size_t used_before_push = used_samples();
  const size_t logical_peak = used_before_push >
          (std::numeric_limits<size_t>::max() - count)
      ? std::numeric_limits<size_t>::max()
      : used_before_push + count;
  append_samples_locked(samples, count, result.dropped_samples, false);

  result.peak_samples = logical_peak;
  if (result.peak_samples > max_samples) {
    result.dropped_samples += discard_oldest_locked(
        used_samples() > target_samples ? used_samples() - target_samples : 0u,
        false);
  }
  result.queued_samples = used_samples();
  publish_available_locked();
  return result;
}

void AudioRingBuffer::append_samples_locked(const s16 *samples, size_t count,
                                            size_t &dropped_samples,
                                            bool preserve_stutter_history) {

  if (count > capacity_) {
    const size_t drop = count - capacity_;
    samples += drop;
    count = capacity_;
    dropped_samples += drop;
  }

  // If pushing would overflow, discard oldest to make room.
  const size_t used = used_samples();
  if (used + count > capacity_) {
    const size_t discard = (used + count) - capacity_;
    if (preserve_stutter_history) {
      std::vector<s16> stale(discard);
      copy_from_ring(read_pos_, stale.data(), discard);
      history_push(stale.data(), stale.size());
    }
    read_pos_ = advance_pos(read_pos_, discard, buffer_.size());
    dropped_samples += discard;
  }

  // Fast path: no wrap.
  const size_t write_end = write_pos_ + count;
  if (write_end <= buffer_.size()) {
    std::memcpy(buffer_.data() + write_pos_, samples,
                count * sizeof(s16));
    write_pos_ = write_end;
    if (write_pos_ == buffer_.size()) write_pos_ = 0;
  } else {
    const size_t first_chunk = buffer_.size() - write_pos_;
    std::memcpy(buffer_.data() + write_pos_, samples,
                first_chunk * sizeof(s16));
    const size_t second_chunk = count - first_chunk;
    std::memcpy(buffer_.data(), samples + first_chunk,
                second_chunk * sizeof(s16));
    write_pos_ = second_chunk;
  }
}

size_t AudioRingBuffer::discard_oldest_samples(size_t count) {
  if (count == 0 || capacity_ == 0) return 0;

  std::lock_guard<std::mutex> lock(mutex_);
  const size_t discard = discard_oldest_locked(count);
  publish_available_locked();
  return discard;
}

size_t AudioRingBuffer::discard_oldest_locked(size_t count,
                                              bool preserve_stutter_history) {
  const size_t discard = std::min(count, used_samples());
  if (discard == 0) return 0;

  if (preserve_stutter_history) {
    std::vector<s16> stale(discard);
    copy_from_ring(read_pos_, stale.data(), discard);
    history_push(stale.data(), stale.size());
  }
  read_pos_ = advance_pos(read_pos_, discard, buffer_.size());
  return discard;
}

// ---------------------------------------------------------------------------
// read_samples() — Consumer (Host Audio thread) + Stutter Logic
// ---------------------------------------------------------------------------
//
// Fill `out_buffer` with exactly `requested_count` interleaved s16 samples.
//
// ── Normal Path ──────────────────────────────────────────────────────────
//   available >= requested_count
//   Copy the samples, advance read_pos_, and feed them into the rolling
//   history buffer so the stutter source is always fresh.
//
// ── Stutter Path ─────────────────────────────────────────────────────────
//   available < requested_count
//   1. Drain whatever fresh samples exist into the start of out_buffer.
//      (These are also recorded into the history buffer.)
//   2. Fill the remaining slots by looping the rolling history buffer —
//      which holds the last ~400 ms of audio from normal playback.
//      This produces a glitchy, retro "Source Engine" lag stutter instead
//      of the horrible digital whine you'd get from looping 1-2 samples.
//   3. If the history buffer is empty (cold start), fill with silence.
//
//   The stutter exits automatically the next time enough samples are
//   available — the SPU thread caught up and we return to the normal path.
// ---------------------------------------------------------------------------
void AudioRingBuffer::read_samples(s16 *out_buffer, size_t requested_count) {
  if (requested_count == 0 || out_buffer == nullptr) return;
  if (capacity_ == 0) {
    std::memset(out_buffer, 0, requested_count * sizeof(s16));
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  const size_t available = used_samples();
  bool was_stuttering = stutter_active_.load(std::memory_order_acquire);
  if (!g_spu_enable_lag_stutter && was_stuttering) {
    stutter_active_ = false;
    stutter_resume_threshold_ = 0;
    stutter_loop_buffer_.clear();
    stutter_loop_pos_ = 0;
    was_stuttering = false;
  }

  if (available >= requested_count &&
      (!was_stuttering || available >= stutter_resume_threshold_)) {
    stutter_active_ = false;
    stutter_resume_threshold_ = 0;
    stutter_loop_buffer_.clear();
    stutter_loop_pos_ = 0;

    copy_from_ring(read_pos_, out_buffer, requested_count);
    read_pos_ = advance_pos(read_pos_, requested_count, buffer_.size());
    publish_available_locked();
    history_push(out_buffer, requested_count);
    if (was_stuttering) {
      crossfade_from_stutter(out_buffer, requested_count);
    }
    update_last_output_frame(out_buffer, requested_count);
    return;
  }

  if (!was_stuttering && history_valid_ > 0) {
    if (g_spu_enable_lag_stutter) {
      refresh_stutter_loop(available, true);
      stutter_resume_threshold_ =
          std::min(capacity_, std::max(requested_count * 2u, requested_count));
      stutter_active_ = true;
    }
  } else if (was_stuttering && history_valid_ > 0 && available >= channels_) {
    refresh_stutter_loop(available, false);
  }

  size_t drained = 0;
  if (!was_stuttering && available > 0) {
    drained = std::min(available, requested_count);
    copy_from_ring(read_pos_, out_buffer, drained);
    read_pos_ = advance_pos(read_pos_, drained, buffer_.size());
    publish_available_locked();
    history_push(out_buffer, drained);
    update_last_output_frame(out_buffer, drained);
  }

  const size_t remaining = requested_count - drained;
  if (remaining == 0) {
    update_last_output_frame(out_buffer, requested_count);
    return;
  }

  if (stutter_loop_buffer_.empty() && !was_stuttering && history_valid_ > 0 &&
      g_spu_enable_lag_stutter) {
    refresh_stutter_loop(0, true);
    stutter_resume_threshold_ =
        std::min(capacity_, std::max(requested_count * 2u, requested_count));
    stutter_active_ = true;
  }

  if (!stutter_loop_buffer_.empty()) {
    copy_from_stutter_loop(out_buffer + drained, remaining);
    if (!was_stuttering) {
      crossfade_from_stutter(out_buffer + drained, remaining);
    }
    update_last_output_frame(out_buffer, requested_count);
    return;
  }

  for (size_t i = drained; i < requested_count; ++i) {
    out_buffer[i] = has_last_output_frame_
        ? last_output_frame_[i % std::max<u32>(channels_, 1u)]
        : 0;
  }
  stutter_active_ = false;
  stutter_resume_threshold_ = 0;
  stutter_loop_pos_ = 0;
  update_last_output_frame(out_buffer, requested_count);
}

void AudioRingBuffer::read_live_samples(s16 *out_buffer,
                                        size_t requested_count) {
  if (requested_count == 0 || out_buffer == nullptr) return;
  if (capacity_ == 0) {
    std::memset(out_buffer, 0, requested_count * sizeof(s16));
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  stutter_active_ = false;
  stutter_resume_threshold_ = 0;
  stutter_loop_buffer_.clear();
  stutter_loop_pos_ = 0;

  const size_t frame_samples = std::max<u32>(channels_, 1u);
  const size_t requested_aligned =
      requested_count - (requested_count % frame_samples);
  const size_t available = used_samples();
  const size_t available_aligned =
      available - (available % frame_samples);

  if (requested_aligned == 0) {
    std::memset(out_buffer, 0, requested_count * sizeof(s16));
    return;
  }

  if (available_aligned >= requested_aligned) {
    copy_from_ring(read_pos_, out_buffer, requested_aligned);
    read_pos_ = advance_pos(read_pos_, requested_aligned, buffer_.size());
    publish_available_locked();
    history_push(out_buffer, requested_aligned);
    update_last_output_frame(out_buffer, requested_aligned);
    if (requested_aligned < requested_count) {
      std::memset(out_buffer + requested_aligned, 0,
                  (requested_count - requested_aligned) * sizeof(s16));
    }
    return;
  }

  if (available_aligned == 0) {
    std::memset(out_buffer, 0, requested_count * sizeof(s16));
    has_last_output_frame_ = false;
    last_output_frame_.fill(0);
    return;
  }

  std::vector<s16> live(available_aligned);
  copy_from_ring(read_pos_, live.data(), available_aligned);
  read_pos_ = advance_pos(read_pos_, available_aligned, buffer_.size());
  publish_available_locked();
  history_push(live.data(), live.size());

  const size_t live_frames = available_aligned / frame_samples;
  const size_t requested_frames = requested_aligned / frame_samples;
  for (size_t frame = 0; frame < requested_frames; ++frame) {
    const size_t src_frame =
        std::min((frame * live_frames) / requested_frames, live_frames - 1u);
    for (size_t ch = 0; ch < frame_samples; ++ch) {
      out_buffer[(frame * frame_samples) + ch] =
          live[(src_frame * frame_samples) + ch];
    }
  }

  if (requested_aligned < requested_count) {
    std::memset(out_buffer + requested_aligned, 0,
                (requested_count - requested_aligned) * sizeof(s16));
  }
  update_last_output_frame(out_buffer, requested_aligned);
}

void AudioRingBuffer::read_stutter_samples(s16 *out_buffer,
                                           size_t requested_count) {
  if (requested_count == 0 || out_buffer == nullptr) return;
  if (capacity_ == 0) {
    std::memset(out_buffer, 0, requested_count * sizeof(s16));
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  const size_t available = used_samples();
  if (available > 0) {
    std::vector<s16> fresh(available);
    copy_from_ring(read_pos_, fresh.data(), available);
    read_pos_ = advance_pos(read_pos_, available, buffer_.size());
    publish_available_locked();
    history_push(fresh.data(), fresh.size());
  }

  if (!g_spu_enable_lag_stutter) {
    for (size_t i = 0; i < requested_count; ++i) {
      out_buffer[i] = has_last_output_frame_
          ? last_output_frame_[i % std::max<u32>(channels_, 1u)]
          : 0;
    }
    stutter_active_ = false;
    update_last_output_frame(out_buffer, requested_count);
    return;
  }

  if (history_valid_ > 0) {
    refresh_stutter_loop(0, !stutter_active_.load(std::memory_order_acquire));
    stutter_resume_threshold_ = std::min(capacity_, history_buffer_.size());
    stutter_active_ = true;
  }

  if (!stutter_loop_buffer_.empty()) {
    copy_from_stutter_loop(out_buffer, requested_count);
  } else {
    for (size_t i = 0; i < requested_count; ++i) {
      out_buffer[i] = has_last_output_frame_
          ? last_output_frame_[i % std::max<u32>(channels_, 1u)]
          : 0;
    }
  }
  update_last_output_frame(out_buffer, requested_count);
}

AudioRingBuffer::ReadResult AudioRingBuffer::try_read_samples(
    s16 *out_buffer, size_t requested_count, size_t discard_before_read) {
  ReadResult result{};
  if (requested_count == 0 || out_buffer == nullptr) {
    return result;
  }

  std::memset(out_buffer, 0, requested_count * sizeof(s16));
  if (capacity_ == 0) {
    return result;
  }

  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    result.lock_contended = true;
    return result;
  }

  const size_t frame_samples = std::max<u32>(channels_, 1u);
  const bool resuming_from_history = history_loop_active_;
  const size_t requested_aligned =
      requested_count - (requested_count % frame_samples);
  const size_t available = used_samples();
  const size_t available_aligned = available - (available % frame_samples);
  result.queue_samples_before = available_aligned;
  discard_before_read -= discard_before_read % frame_samples;
  const size_t max_discard = available_aligned > requested_aligned
      ? available_aligned - requested_aligned
      : 0u;
  result.discarded_samples = std::min(discard_before_read, max_discard);
  const size_t available_after_discard =
      available_aligned - result.discarded_samples;
  result.consumed_samples =
      std::min(requested_aligned, available_after_discard);
  if (result.consumed_samples == 0) {
    result.queue_samples_after = available_after_discard;
    publish_available_locked();
    return result;
  }

  if (result.discarded_samples > 0) {
    static constexpr size_t kSmoothTrimCrossfadeFrames = 64u;
    const size_t fade_samples = std::min(
        result.consumed_samples, kSmoothTrimCrossfadeFrames * frame_samples);
    for (size_t i = 0; i < fade_samples; ++i) {
      const size_t old_pos =
          (read_pos_ + i) % buffer_.size();
      const size_t new_pos =
          (read_pos_ + result.discarded_samples + i) % buffer_.size();
      const size_t fade_frame = i / frame_samples;
      const size_t fade_frames = fade_samples / frame_samples;
      const s64 old_weight = static_cast<s64>(fade_frames - fade_frame);
      const s64 new_weight = static_cast<s64>(fade_frame + 1u);
      const s64 denominator = static_cast<s64>(fade_frames + 1u);
      const s64 mixed =
          (static_cast<s64>(buffer_[old_pos]) * old_weight +
           static_cast<s64>(buffer_[new_pos]) * new_weight) /
          denominator;
      const s64 magnitude = mixed < 0 ? -mixed : mixed;
      result.peak_sample_before_clamp = std::max<u32>(
          result.peak_sample_before_clamp,
          static_cast<u32>(std::min<s64>(magnitude,
                                         std::numeric_limits<u32>::max())));
      if (mixed < -32768 || mixed > 32767) {
        ++result.clipped_samples;
      }
      out_buffer[i] = static_cast<s16>(std::clamp<s64>(mixed, -32768, 32767));
    }
    if (fade_samples < result.consumed_samples) {
      const size_t remainder_pos = advance_pos(
          read_pos_, result.discarded_samples + fade_samples, buffer_.size());
      copy_from_ring(remainder_pos, out_buffer + fade_samples,
                     result.consumed_samples - fade_samples);
    }
    read_pos_ = advance_pos(
        read_pos_, result.discarded_samples + result.consumed_samples,
        buffer_.size());
    result.read_pointer_discontinuities = 1;
  } else {
    copy_from_ring(read_pos_, out_buffer, result.consumed_samples);
    read_pos_ = advance_pos(read_pos_, result.consumed_samples, buffer_.size());
  }
  for (size_t i = 0; i < result.consumed_samples; ++i) {
    const s32 sample = static_cast<s32>(out_buffer[i]);
    result.peak_sample_before_clamp = std::max<u32>(
        result.peak_sample_before_clamp,
        static_cast<u32>(sample < 0 ? -sample : sample));
  }
  if (resuming_from_history) {
    crossfade_from_stutter(out_buffer, result.consumed_samples);
  }
  history_push(out_buffer, result.consumed_samples);
  update_last_output_frame(out_buffer, result.consumed_samples);
  history_loop_active_ = false;
  result.queue_samples_after = used_samples();
  publish_available_locked();
  return result;
}

AudioRingBuffer::ReadResult AudioRingBuffer::try_read_history_samples(
    s16 *out_buffer, size_t requested_count, size_t drain_live_samples) {
  ReadResult result{};
  if (requested_count == 0 || out_buffer == nullptr) {
    return result;
  }

  std::memset(out_buffer, 0, requested_count * sizeof(s16));
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    result.lock_contended = true;
    return result;
  }

  const size_t frame_samples = std::max<u32>(channels_, 1u);
  const size_t requested_aligned =
      requested_count - (requested_count % frame_samples);
  result.queue_samples_before = used_samples();
  drain_live_samples -= drain_live_samples % frame_samples;
  result.history_drained_samples =
      std::min(drain_live_samples,
               result.queue_samples_before -
                   (result.queue_samples_before % frame_samples));
  size_t drain_remaining = result.history_drained_samples;
  while (drain_remaining > 0) {
    const size_t chunk = std::min(drain_remaining,
                                  buffer_.size() - read_pos_);
    history_push(buffer_.data() + read_pos_, chunk);
    read_pos_ = advance_pos(read_pos_, chunk, buffer_.size());
    drain_remaining -= chunk;
  }
  result.queue_samples_after = used_samples();
  publish_available_locked();
  if (requested_aligned == 0 || history_valid_ < frame_samples ||
      history_buffer_.empty()) {
    return result;
  }

  if (!history_loop_active_) {
    history_loop_length_ = history_valid_ - (history_valid_ % frame_samples);
    history_loop_start_ = history_valid_ == history_buffer_.size()
        ? choose_stutter_start_pos()
        : 0u;
    history_loop_offset_ = 0;
    history_loop_active_ = true;
  }

  size_t written = 0;
  while (written < requested_aligned && history_loop_length_ > 0) {
    const size_t physical =
        (history_loop_start_ + history_loop_offset_) % history_buffer_.size();
    const size_t until_loop_end =
        history_loop_length_ - history_loop_offset_;
    const size_t until_buffer_end = history_buffer_.size() - physical;
    const size_t chunk = std::min(
        requested_aligned - written,
        std::min(until_loop_end, until_buffer_end));
    std::memcpy(out_buffer + written, history_buffer_.data() + physical,
                chunk * sizeof(s16));
    written += chunk;
    history_loop_offset_ += chunk;
    if (history_loop_offset_ >= history_loop_length_) {
      history_loop_offset_ = 0;
    }
  }
  result.consumed_samples = written;
  for (size_t i = 0; i < written; ++i) {
    const s32 sample = static_cast<s32>(out_buffer[i]);
    result.peak_sample_before_clamp = std::max<u32>(
        result.peak_sample_before_clamp,
        static_cast<u32>(sample < 0 ? -sample : sample));
  }
  update_last_output_frame(out_buffer, written);
  return result;
}

// ---------------------------------------------------------------------------
// samples_from_seconds()
// ---------------------------------------------------------------------------
//
// Convert a time duration to the equivalent sample count.
// NO minimum clamp — if the math says 1, the result is 1.
// ---------------------------------------------------------------------------
size_t AudioRingBuffer::samples_from_seconds(double seconds) const {
  return static_cast<size_t>(
      std::max(0.0, std::ceil(
          seconds * static_cast<double>(sample_rate_) *
          static_cast<double>(channels_))));
}

// ---------------------------------------------------------------------------
// available_samples()
// ---------------------------------------------------------------------------
size_t AudioRingBuffer::available_samples() const {
  return available_samples_snapshot_.load(std::memory_order_acquire);
}

// ---------------------------------------------------------------------------
// capacity_seconds()
// ---------------------------------------------------------------------------
double AudioRingBuffer::capacity_seconds() const {
  if (sample_rate_ == 0 || channels_ == 0) return 0.0;
  return static_cast<double>(capacity_) /
         (static_cast<double>(sample_rate_) * static_cast<double>(channels_));
}

// ---------------------------------------------------------------------------
// clear()
// ---------------------------------------------------------------------------
void AudioRingBuffer::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  write_pos_ = 0;
  read_pos_ = 0;
  available_samples_snapshot_.store(0, std::memory_order_release);
  stutter_active_ = false;
  stutter_loop_buffer_.clear();
  stutter_loop_pos_ = 0;
  stutter_resume_threshold_ = 0;
  history_write_ = 0;
  history_valid_ = 0;
  last_output_frame_.fill(0);
  has_last_output_frame_ = false;
  history_loop_active_ = false;
  history_loop_start_ = 0;
  history_loop_length_ = 0;
  history_loop_offset_ = 0;
}

// ---------------------------------------------------------------------------
// copy_from_ring() — internal helper
// ---------------------------------------------------------------------------
//
// Copy `count` samples from the ring buffer starting at `ring_pos` into
// `dst`.  Handles wrap-around with either a single memcpy or two.
// ---------------------------------------------------------------------------
void AudioRingBuffer::copy_from_ring(size_t ring_pos, s16 *dst,
                                     size_t count) const {
  if (count == 0) return;
  const size_t buf_size = buffer_.size();
  const size_t first_chunk = std::min(count, buf_size - ring_pos);

  std::memcpy(dst, buffer_.data() + ring_pos,
              first_chunk * sizeof(s16));

  if (first_chunk < count) {
    std::memcpy(dst + first_chunk, buffer_.data(),
                (count - first_chunk) * sizeof(s16));
  }
}

void AudioRingBuffer::copy_from_history(size_t history_pos, s16 *dst,
                                        size_t count) const {
  if (count == 0) return;
  const size_t hist_size = history_buffer_.size();
  const size_t first_chunk = std::min(count, hist_size - history_pos);

  std::memcpy(dst, history_buffer_.data() + history_pos,
              first_chunk * sizeof(s16));

  if (first_chunk < count) {
    std::memcpy(dst + first_chunk, history_buffer_.data(),
                (count - first_chunk) * sizeof(s16));
  }
}

void AudioRingBuffer::copy_from_stutter_loop(s16 *dst, size_t count) {
  if (count == 0 || stutter_loop_buffer_.empty()) return;

  const size_t loop_size = stutter_loop_buffer_.size();
  size_t copied = 0;
  while (copied < count) {
    const size_t chunk =
        std::min(count - copied, loop_size - stutter_loop_pos_);
    std::memcpy(dst + copied, stutter_loop_buffer_.data() + stutter_loop_pos_,
                chunk * sizeof(s16));
    copied += chunk;
    stutter_loop_pos_ += chunk;
    if (stutter_loop_pos_ >= loop_size) {
      stutter_loop_pos_ = 0;
    }
  }
}

size_t AudioRingBuffer::choose_stutter_start_pos() const {
  const size_t hist_size = history_buffer_.size();
  if (history_valid_ == 0 || hist_size == 0 || channels_ == 0) {
    return 0;
  }

  const size_t hist_read_start =
      (history_write_ + hist_size - history_valid_) % hist_size;
  if (!has_last_output_frame_ || history_valid_ <= channels_) {
    return hist_read_start;
  }

  size_t best_pos = hist_read_start;
  s64 best_score = std::numeric_limits<s64>::max();
  for (size_t offset = 0; offset + channels_ <= history_valid_;
       offset += channels_) {
    const size_t pos = (hist_read_start + offset) % hist_size;
    s64 score = 0;
    for (u32 ch = 0; ch < channels_ && ch < last_output_frame_.size(); ++ch) {
      const s32 sample = static_cast<s32>(history_buffer_[(pos + ch) % hist_size]);
      const s32 last = static_cast<s32>(last_output_frame_[ch]);
      score += static_cast<s64>(std::abs(sample - last));
    }
    if (score < best_score) {
      best_score = score;
      best_pos = pos;
    }
  }

  return best_pos;
}

void AudioRingBuffer::refresh_stutter_loop(size_t preview_samples,
                                           bool prefer_clean_entry) {
  if (history_valid_ == 0 || history_buffer_.empty()) {
    stutter_loop_buffer_.clear();
    stutter_loop_pos_ = 0;
    return;
  }

  preview_samples = std::min(preview_samples, history_valid_);
  preview_samples -= (preview_samples % std::max<u32>(channels_, 1u));

  const size_t old_size = stutter_loop_buffer_.size();
  const size_t old_pos = stutter_loop_pos_;
  const size_t hist_read_start = prefer_clean_entry
      ? choose_stutter_start_pos()
      : (history_write_ + history_buffer_.size() - history_valid_) %
            history_buffer_.size();

  std::vector<s16> refreshed(history_valid_);
  copy_from_history(hist_read_start, refreshed.data(), history_valid_);

  if (preview_samples > 0) {
    const size_t tail_offset = refreshed.size() - preview_samples;
    copy_from_ring(read_pos_, refreshed.data() + tail_offset, preview_samples);
  }

  stutter_loop_buffer_.swap(refreshed);
  if (stutter_loop_buffer_.empty()) {
    stutter_loop_pos_ = 0;
    return;
  }

  if (prefer_clean_entry || old_size == 0) {
    stutter_loop_pos_ = 0;
    return;
  }

  const size_t mapped_pos =
      (old_pos * stutter_loop_buffer_.size()) / std::max<size_t>(old_size, 1u);
  const size_t frame_align = std::max<u32>(channels_, 1u);
  const size_t aligned_pos = mapped_pos - (mapped_pos % frame_align);
  if (stutter_loop_buffer_.size() <= frame_align) {
    stutter_loop_pos_ = 0;
  } else {
    stutter_loop_pos_ =
        std::min(aligned_pos, stutter_loop_buffer_.size() - frame_align);
  }
}

void AudioRingBuffer::update_last_output_frame(const s16 *samples, size_t count) {
  if (samples == nullptr || channels_ == 0 || count < channels_) return;
  const size_t last_frame = count - channels_;
  for (u32 ch = 0; ch < channels_ && ch < last_output_frame_.size(); ++ch) {
    last_output_frame_[ch] = samples[last_frame + ch];
  }
  has_last_output_frame_ = true;
}

void AudioRingBuffer::crossfade_from_stutter(s16 *samples, size_t count) {
  if (samples == nullptr || !has_last_output_frame_ || channels_ == 0) return;

  static constexpr size_t kResumeCrossfadeFrames = 64;
  const size_t frame_count = count / channels_;
  const size_t blend_frames = std::min(frame_count, kResumeCrossfadeFrames);
  if (blend_frames == 0) return;

  for (size_t frame = 0; frame < blend_frames; ++frame) {
    const float t = static_cast<float>(frame + 1u) /
                    static_cast<float>(blend_frames + 1u);
    for (u32 ch = 0; ch < channels_ && ch < last_output_frame_.size(); ++ch) {
      const size_t idx = frame * channels_ + ch;
      const float blended =
          (static_cast<float>(last_output_frame_[ch]) * (1.0f - t)) +
          (static_cast<float>(samples[idx]) * t);
      samples[idx] = static_cast<s16>(std::clamp(
          static_cast<int>(std::lround(blended)), -32768, 32767));
    }
  }
}

// ---------------------------------------------------------------------------
// history_push() — internal helper
// ---------------------------------------------------------------------------
//
// Append samples into the rolling circular history buffer.  The history
// always contains the most recent ~400 ms of audio; when it wraps the
// oldest entries are overwritten.
// ---------------------------------------------------------------------------
void AudioRingBuffer::history_push(const s16 *samples, size_t count) {
  if (count == 0 || history_buffer_.empty()) return;

  const size_t hist_size = history_buffer_.size();

  if (count >= hist_size) {
    // New data exceeds history size — just take the tail portion so the
    // most recent samples fill the entire buffer.
    const size_t offset = count - hist_size;
    std::memcpy(history_buffer_.data(), samples + offset,
                hist_size * sizeof(s16));
    history_write_ = 0;
    history_valid_ = hist_size;
  } else {
    const size_t first_chunk = std::min(count, hist_size - history_write_);
    std::memcpy(history_buffer_.data() + history_write_, samples,
                first_chunk * sizeof(s16));

    if (first_chunk < count) {
      std::memcpy(history_buffer_.data(), samples + first_chunk,
                  (count - first_chunk) * sizeof(s16));
      history_write_ = (count - first_chunk);
    } else {
      history_write_ += first_chunk;
      if (history_write_ == hist_size) history_write_ = 0;
    }

    history_valid_ = std::min(history_valid_ + count, hist_size);
  }
}
