#include "audio_ring_buffer.h"

#include <algorithm>
#include <cstring>

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

  // Allocate the rolling history buffer.
  history_buffer_.resize(
      std::max<size_t>(samples_from_seconds(HISTORY_BUFFER_SECONDS), 1));
  history_write_ = 0;
  history_valid_ = 0;

  // Reset stutter state.
  stutter_active_ = false;
  stutter_index_ = 0;
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

  // If pushing would overflow, discard oldest to make room.
  const size_t used = used_samples();
  if (used + count > capacity_) {
    const size_t discard = (used + count) - capacity_;
    read_pos_ = advance_pos(read_pos_, discard, buffer_.size());
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

  if (available >= requested_count) {
    // ---- Normal path: enough data, copy and advance. ----
    stutter_active_ = false;

    // Copy from ring.
    copy_from_ring(read_pos_, out_buffer, requested_count);
    read_pos_ = advance_pos(read_pos_, requested_count, buffer_.size());

    // Record into rolling history so stutter source stays fresh.
    history_push(out_buffer, requested_count);
  } else {
    // ---- Stutter path: starved, loop the rolling history. ----

    // 1. Drain what we have (may be zero).
    size_t drained = 0;
    if (available > 0) {
      copy_from_ring(read_pos_, out_buffer, available);
      // Feed drained samples into history as well.
      history_push(out_buffer, available);
      read_pos_ = advance_pos(read_pos_, available, buffer_.size());
      drained = available;
    }

    // 2. Fill the remainder by looping the rolling history buffer.
    //    The oldest valid sample in the circular history is at:
    //      (history_write_ - history_valid_ + hist_size) % hist_size
    //    Each successive output sample reads the next position in the
    //    circular history, wrapping at history_valid_.
    size_t out_idx = drained;
    if (history_valid_ > 0) {
      stutter_active_ = true;
      const size_t hist_size = history_buffer_.size();
      const size_t hist_read_start =
          (history_write_ + hist_size - history_valid_) % hist_size;

      while (out_idx < requested_count) {
        const size_t hist_pos =
            (hist_read_start + stutter_index_) % hist_size;
        out_buffer[out_idx++] = history_buffer_[hist_pos];
        ++stutter_index_;
      }
    } else {
      // Complete cold start — no history yet. Fill with silence.
      std::memset(out_buffer + out_idx, 0,
                  (requested_count - out_idx) * sizeof(s16));
      stutter_active_ = false;
    }
  }
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
  std::lock_guard<std::mutex> lock(mutex_);
  return used_samples();
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
  stutter_active_ = false;
  stutter_index_ = 0;
  history_write_ = 0;
  history_valid_ = 0;
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
