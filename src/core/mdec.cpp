#include "mdec.h"

void Mdec::reset() {
  control_ = 0;
  command_busy_ = false;
  expect_command_word_ = true;
  command_id_ = 0;
  command_word_ = 0;
  in_words_remaining_ = 0;
  decode_seed_ = 0x1A2B3C4Du;
  out_fifo_.clear();
}

void Mdec::begin_command(u32 value) {
  command_word_ = value;
  command_id_ = static_cast<u8>((value >> 29) & 0x7u);
  expect_command_word_ = false;
  command_busy_ = true;

  switch (command_id_) {
  case 0: // Decode macroblocks.
    in_words_remaining_ = value & 0xFFFFu;
    if (in_words_remaining_ == 0) {
      in_words_remaining_ = 1;
    }
    break;
  case 1: // Set quantization table.
    in_words_remaining_ = (value & 0x1u) ? 32u : 16u;
    break;
  case 2: // Set scale table.
    in_words_remaining_ = 32u;
    break;
  default:
    in_words_remaining_ = value & 0xFFFFu;
    break;
  }

  if (in_words_remaining_ == 0) {
    finish_command();
  }
}

void Mdec::finish_command() {
  expect_command_word_ = true;
  command_busy_ = false;
  in_words_remaining_ = 0;
}

void Mdec::push_decode_word(u32 value) {
  decode_seed_ = decode_seed_ * 1664525u + 1013904223u + value;
  const u32 pseudo =
      ((value ^ decode_seed_) & 0x00FFFFFFu) | ((value >> 24) << 24);
  out_fifo_.push_back(pseudo);
}

void Mdec::write_command(u32 value) {
  if (expect_command_word_) {
    begin_command(value);
    return;
  }

  if (in_words_remaining_ > 0) {
    --in_words_remaining_;
  }

  if (command_id_ == 0) {
    push_decode_word(value);
  }

  if (in_words_remaining_ == 0) {
    finish_command();
  }
}

void Mdec::write_control(u32 value) {
  control_ = value;
  if (value & 0x80000000u) {
    reset();
    return;
  }

  if (value & 0x40000000u) {
    out_fifo_.clear();
  }
}

u32 Mdec::read_data() {
  if (out_fifo_.empty()) {
    return 0;
  }
  const u32 value = out_fifo_.front();
  out_fifo_.pop_front();
  return value;
}

u32 Mdec::read_status() const {
  u32 status = 0;
  if (command_busy_) {
    status |= 1u << 29;
  }
  if (dma_in_request()) {
    status |= 1u << 28;
  }
  if (dma_out_request()) {
    status |= 1u << 27;
  }
  if (out_fifo_.empty()) {
    status |= 1u << 31;
  }
  if (!expect_command_word_ && in_words_remaining_ > 0) {
    status |= (in_words_remaining_ & 0xFFFFu);
  }
  return status;
}

bool Mdec::dma_in_request() const {
  const bool dma_in_enabled = (control_ & 0x40000000u) != 0;
  return dma_in_enabled && !expect_command_word_ && (in_words_remaining_ > 0);
}

bool Mdec::dma_out_request() const {
  const bool dma_out_enabled = (control_ & 0x20000000u) != 0;
  return dma_out_enabled && !out_fifo_.empty();
}
