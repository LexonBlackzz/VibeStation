#pragma once
#include "types.h"
#include <array>
#include <deque>
#include <vector>

class Mdec {
public:
  void reset();

  void write_command(u32 value);
  void write_control(u32 value);
  u32 read_data();
  u32 read_status() const;
  u8 dma_out_block() const;
  u8 dma_out_depth() const { return out_depth_latched_; }

  void dma_write(u32 value) { write_command(value); }
  u32 dma_read() { return read_data(); }
  bool dma_in_request() const;
  bool dma_out_request() const;
  u32 dma_out_words_available() const {
    return static_cast<u32>(out_fifo_.size());
  }

private:
  static constexpr size_t kBlockSize = 64;
  using Block = std::array<int, kBlockSize>;

  void begin_command(u32 value);
  void execute_command();
  void execute_decode();
  void execute_set_quant_table();
  void execute_set_scale_table();
  bool decode_block(Block &block, const std::array<u8, kBlockSize> &quant_table);
  void idct(const Block &coeffs, Block &pixels) const;
  void emit_colored_macroblock(const Block &cr, const Block &cb,
                               const Block &y1, const Block &y2,
                               const Block &y3, const Block &y4);
  void emit_monochrome_macroblock(const Block &y);
  void push_output_byte(u8 value);
  static int sign_extend_10(u16 value);
  static int clamp_s11(int value);
  u8 encode_component(int value) const;
  u16 encode_rgb15(int r, int g, int b) const;

  u32 control_ = 0;
  bool command_busy_ = false;
  bool expect_command_word_ = true;
  u8 command_id_ = 0;
  u32 command_word_ = 0;
  u32 in_words_remaining_ = 0;
  bool in_unlimited_ = false;
  
  std::deque<u16> in_halfword_fifo_{};

  std::array<u8, kBlockSize> quant_luma_{};
  std::array<u8, kBlockSize> quant_chroma_{};
  std::array<s16, kBlockSize> scale_table_{};
  u8 status_command_bits_ = 0;
  u8 current_block_ = 4;
  u8 output_depth_ = 2;
  bool output_signed_ = false;
  bool output_set_bit15_ = false;
  u32 output_pack_word_ = 0;
  u32 output_pack_bytes_ = 0;
  u8 output_word_block_id_ = 4;
  u8 out_depth_latched_ = 2;
  std::deque<u32> out_fifo_{};
  std::deque<u8> out_block_fifo_{};
};
