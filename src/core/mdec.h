#pragma once
#include "types.h"
#include <deque>

class Mdec {
public:
  void reset();

  void write_command(u32 value);
  void write_control(u32 value);
  u32 read_data();
  u32 read_status() const;

  void dma_write(u32 value) { write_command(value); }
  u32 dma_read() { return read_data(); }
  bool dma_in_request() const;
  bool dma_out_request() const;

private:
  void begin_command(u32 value);
  void finish_command();
  void push_decode_word(u32 value);

  u32 control_ = 0;
  bool command_busy_ = false;
  bool expect_command_word_ = true;
  u8 command_id_ = 0;
  u32 command_word_ = 0;
  u32 in_words_remaining_ = 0;
  u32 decode_seed_ = 0x1A2B3C4Du;
  std::deque<u32> out_fifo_{};
};

