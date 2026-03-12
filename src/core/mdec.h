#pragma once
#include "types.h"
#include <array>
#include <deque>
#include <vector>

class Mdec {
public:
  struct DebugStats {
    static constexpr size_t kCommandHistory = 16;
    static constexpr size_t kWriteHistory = 32;
    u64 decode_commands = 0;
    u64 set_quant_commands = 0;
    u64 set_scale_commands = 0;
    u64 blocks_decoded = 0;
    u64 dc_only_blocks = 0;
    u64 qscale_zero_blocks = 0;
    u64 qscale_sum = 0;
    u32 qscale_max = 0;
    u64 nonzero_coeff_count = 0;
    u64 eob_markers = 0;
    u64 overflow_breaks = 0;
    u32 quant_luma0 = 0;
    u32 quant_chroma0 = 0;
    u32 quant_luma_avg = 0;
    u32 quant_chroma_avg = 0;
    u32 command_history_count = 0;
    std::array<u32, kCommandHistory> command_history_words{};
    std::array<u8, kCommandHistory> command_history_ids{};
    u32 write_history_count = 0;
    std::array<u32, kWriteHistory> write_history_words{};
    std::array<u8, kWriteHistory> write_history_expect_command{};
    std::array<u8, kWriteHistory> write_history_active_command{};
    u32 control_history_count = 0;
    std::array<u32, kCommandHistory> control_history_words{};
  };

  struct DebugCompare {
    static constexpr size_t kBlocksPerMacroblock = 6;
    static constexpr size_t kInputSampleHalfwords = 16;

    enum class Stage : u8 {
      None = 0,
      Coeff = 1,
      Idct = 2,
      Rgb = 3,
      Match = 4,
    };

    bool captured = false;
    bool color_macroblock = false;
    u64 macroblocks_compared = 0;
    Stage stage = Stage::None;
    u8 mismatch_block = 0xFFu;
    u8 mismatch_index = 0xFFu;
    s32 current_value = 0;
    s32 reference_value = 0;
    u32 input_halfword_count = 0;
    std::array<u16, kInputSampleHalfwords> input_halfwords{};
    std::array<u8, kBlocksPerMacroblock> qscales{};
  };

  void reset();

  void write_command(u32 value);
  void write_control(u32 value);
  u32 read_data();
  u32 read_status() const;
  u8 dma_out_block() const;
  u8 dma_out_depth() const { return out_depth_latched_; }
  u32 dma_out_macroblock_seq() const;

  void dma_write(u32 value) { write_command(value); }
  u32 dma_read() { return read_data(); }
  bool dma_in_request() const;
  bool dma_out_request() const;
  u32 dma_out_words_available() const {
    return static_cast<u32>(out_fifo_.size());
  }
  const DebugStats &debug_stats() const { return debug_stats_; }
  void reset_debug_stats();
  const DebugCompare &debug_compare() const { return debug_compare_; }
  void reset_debug_compare() { debug_compare_ = {}; }

private:
  static constexpr size_t kBlockSize = 64;
  using Block = std::array<int, kBlockSize>;
  using MacroblockBlocks = std::array<Block, DebugCompare::kBlocksPerMacroblock>;
  using MacroblockRgb = std::array<u32, 16u * 16u>;

  enum class DecodeVariant : u8 {
    Current = 0,
    Reference = 1,
  };

  void begin_command(u32 value);
  void soft_reset_state();
  void execute_command();
  void execute_decode();
  void execute_set_quant_table();
  void execute_set_scale_table();
  bool scan_block(size_t &cursor) const;
  bool scan_macroblock(size_t block_count, size_t &consumed_halfwords) const;
  bool decode_block(Block &block, const std::array<u8, kBlockSize> &quant_table,
                    size_t &cursor);
  void idct(const Block &coeffs, Block &pixels) const;
  void compare_colored_macroblock(const std::vector<u16> &input) ;
  bool decode_colored_macroblock_variant(const std::vector<u16> &input,
                                         DecodeVariant variant,
                                         MacroblockBlocks &coeff_blocks,
                                         MacroblockBlocks &pixel_blocks,
                                         std::array<u8, DebugCompare::kBlocksPerMacroblock> &qscales) const;
  bool decode_block_variant(const std::vector<u16> &input, size_t &cursor,
                            const std::array<u8, kBlockSize> &quant_table,
                            DecodeVariant variant, Block &coeffs,
                            Block &pixels, u8 &q_scale) const;
  void idct_variant(const Block &coeffs, Block &pixels,
                    DecodeVariant variant) const;
  void build_macroblock_rgb(const MacroblockBlocks &pixel_blocks,
                            MacroblockRgb &rgb_out) const;
  void pack_macroblock_words(const MacroblockRgb &rgb,
                             DecodeVariant variant,
                             std::vector<u32> &words_out) const;
  void emit_colored_macroblock(const Block &cr, const Block &cb,
                               const Block &y1, const Block &y2,
                               const Block &y3, const Block &y4);
  void emit_monochrome_macroblock(const Block &y);
  void push_output_word(u32 value);
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
  u32 output_macroblock_seq_ = 0;
  u32 current_output_macroblock_seq_ = 0;
  u8 out_depth_latched_ = 2;
  std::deque<u32> out_fifo_{};
  std::deque<u8> out_block_fifo_{};
  std::deque<u32> out_macroblock_fifo_{};
  DebugStats debug_stats_{};
  DebugCompare debug_compare_{};
  void refresh_debug_quant_stats();
};
