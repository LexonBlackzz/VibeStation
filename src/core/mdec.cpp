#include "mdec.h"
#include <algorithm>
#include <array>

namespace {
// Run-length stream index -> row-major 8x8 matrix index.
constexpr std::array<int, 64> kZigZag = {
  0 ,8 ,1 ,2 ,9 ,16,24,17,
  10,3 ,4 ,11,18,25,32,40,
  33,26,19,12,5 ,6 ,13,20,
  27,34,41,48,56,49,42,35,
  28,21,14,7 ,15,22,29,36,
  43,50,57,58,51,44,37,30,
  23,31,38,45,52,59,60,53,
  46,39,47,54,61,62,55,63,
};

constexpr std::array<s16, 64> kDefaultScaleTable = {
    static_cast<s16>(0x5A82), static_cast<s16>(0x5A82),
    static_cast<s16>(0x5A82), static_cast<s16>(0x5A82),
    static_cast<s16>(0x5A82), static_cast<s16>(0x5A82),
    static_cast<s16>(0x5A82), static_cast<s16>(0x5A82),
    static_cast<s16>(0x7D8A), static_cast<s16>(0x6A6D),
    static_cast<s16>(0x471C), static_cast<s16>(0x18F8),
    static_cast<s16>(0xE707), static_cast<s16>(0xB8E3),
    static_cast<s16>(0x9592), static_cast<s16>(0x8275),
    static_cast<s16>(0x7641), static_cast<s16>(0x30FB),
    static_cast<s16>(0xCF04), static_cast<s16>(0x89BE),
    static_cast<s16>(0x89BE), static_cast<s16>(0xCF04),
    static_cast<s16>(0x30FB), static_cast<s16>(0x7641),
    static_cast<s16>(0x6A6D), static_cast<s16>(0xE707),
    static_cast<s16>(0x8275), static_cast<s16>(0xB8E3),
    static_cast<s16>(0x471C), static_cast<s16>(0x7D8A),
    static_cast<s16>(0x18F8), static_cast<s16>(0x9592),
    static_cast<s16>(0x5A82), static_cast<s16>(0xA57D),
    static_cast<s16>(0xA57D), static_cast<s16>(0x5A82),
    static_cast<s16>(0x5A82), static_cast<s16>(0xA57D),
    static_cast<s16>(0xA57D), static_cast<s16>(0x5A82),
    static_cast<s16>(0x471C), static_cast<s16>(0x8275),
    static_cast<s16>(0x18F8), static_cast<s16>(0x6A6D),
    static_cast<s16>(0x9592), static_cast<s16>(0xE707),
    static_cast<s16>(0x7D8A), static_cast<s16>(0xB8E3),
    static_cast<s16>(0x30FB), static_cast<s16>(0x89BE),
    static_cast<s16>(0x7641), static_cast<s16>(0xCF04),
    static_cast<s16>(0xCF04), static_cast<s16>(0x7641),
    static_cast<s16>(0x89BE), static_cast<s16>(0x30FB),
    static_cast<s16>(0x18F8), static_cast<s16>(0xB8E3),
    static_cast<s16>(0x6A6D), static_cast<s16>(0x8275),
    static_cast<s16>(0x7D8A), static_cast<s16>(0x9592),
    static_cast<s16>(0x471C), static_cast<s16>(0xE707),
};

} // namespace

void Mdec::refresh_debug_quant_stats() {
  u32 luma_sum = 0;
  u32 chroma_sum = 0;
  for (size_t i = 0; i < 64u; ++i) {
    luma_sum += quant_luma_[i];
    chroma_sum += quant_chroma_[i];
  }
  debug_stats_.quant_luma0 = quant_luma_[0];
  debug_stats_.quant_chroma0 = quant_chroma_[0];
  debug_stats_.quant_luma_avg = luma_sum / 64u;
  debug_stats_.quant_chroma_avg = chroma_sum / 64u;
}

void Mdec::reset_debug_stats() {
  debug_stats_.decode_commands = 0;
  debug_stats_.set_quant_commands = 0;
  debug_stats_.set_scale_commands = 0;
  debug_stats_.blocks_decoded = 0;
  debug_stats_.dc_only_blocks = 0;
  debug_stats_.qscale_zero_blocks = 0;
  debug_stats_.qscale_sum = 0;
  debug_stats_.qscale_max = 0;
  debug_stats_.nonzero_coeff_count = 0;
  debug_stats_.eob_markers = 0;
  debug_stats_.overflow_breaks = 0;
  debug_stats_.command_history_count = 0;
  debug_stats_.command_history_words.fill(0);
  debug_stats_.command_history_ids.fill(0);
  debug_stats_.write_history_count = 0;
  debug_stats_.write_history_words.fill(0);
  debug_stats_.write_history_expect_command.fill(0);
  debug_stats_.write_history_active_command.fill(0);
  debug_stats_.control_history_count = 0;
  debug_stats_.control_history_words.fill(0);
  refresh_debug_quant_stats();
}

void Mdec::soft_reset_state() {
  control_ = 0;
  command_busy_ = false;
  expect_command_word_ = true;
  command_id_ = 0;
  command_word_ = 0;
  in_words_remaining_ = 0;
  in_unlimited_ = false;
  in_halfword_fifo_.clear();
  status_command_bits_ = 0;
  current_block_ = 4;
  output_depth_ = 2;
  output_signed_ = false;
  output_set_bit15_ = false;
  output_pack_word_ = 0;
  output_pack_bytes_ = 0;
  output_word_block_id_ = 4;
  output_macroblock_seq_ = 0;
  current_output_macroblock_seq_ = 0;
  out_depth_latched_ = 2;
  out_fifo_.clear();
  out_block_fifo_.clear();
  out_macroblock_fifo_.clear();
}

void Mdec::reset() {
  soft_reset_state();
  quant_luma_.fill(1);
  quant_chroma_.fill(1);
  // Scale table is consumed transposed by the IDCT path.
  for (size_t y = 0; y < 8u; ++y) {
    for (size_t x = 0; x < 8u; ++x) {
      scale_table_[y * 8u + x] = kDefaultScaleTable[x * 8u + y];
    }
  }
  debug_stats_ = {};
  refresh_debug_quant_stats();
}

void Mdec::begin_command(u32 value) {
  command_word_ = value;
  command_id_ = static_cast<u8>((value >> 29) & 0x7u);
  const u32 hist_index =
      debug_stats_.command_history_count % static_cast<u32>(DebugStats::kCommandHistory);
  debug_stats_.command_history_words[hist_index] = value;
  debug_stats_.command_history_ids[hist_index] = command_id_;
  ++debug_stats_.command_history_count;
  status_command_bits_ = static_cast<u8>((value >> 25) & 0x0Fu);
  output_depth_ = static_cast<u8>((value >> 27) & 0x3u);
  output_signed_ = (value & (1u << 26)) != 0;
  output_set_bit15_ = (value & (1u << 25)) != 0;
  in_halfword_fifo_.clear();
  // A fresh command resets pending output words from the previous command.
  out_fifo_.clear();
  out_block_fifo_.clear();
  out_macroblock_fifo_.clear();
  output_pack_word_ = 0;
  output_pack_bytes_ = 0;

  const u32 length_param = value & 0xFFFFu;
  in_words_remaining_ = length_param;
  in_unlimited_ = false;

  switch (command_id_) {
  case 0: // No-op / Reset?
    expect_command_word_ = true;
    command_busy_ = false;
    break;
  case 1: // Decode macroblocks
    ++debug_stats_.decode_commands;
    in_words_remaining_ = (value & 0xFFFFu);
    in_unlimited_ = false;
    command_busy_ = true;
    expect_command_word_ = false;
    out_depth_latched_ = output_depth_;
    output_pack_word_ = 0;
    output_pack_bytes_ = 0;
    break;
  case 2: // Set quant table
    ++debug_stats_.set_quant_commands;
    in_words_remaining_ = (value & 0x1u) ? 32u : 16u;
    in_unlimited_ = false;
    command_busy_ = true;
    expect_command_word_ = false;
    break;
  case 3: // Set scale table
    ++debug_stats_.set_scale_commands;
    in_words_remaining_ = 32u;
    in_unlimited_ = false;
    command_busy_ = true;
    expect_command_word_ = false;
    break;
  default:
    expect_command_word_ = true;
    command_busy_ = false;
    break;
  }

  if (!in_unlimited_ && in_words_remaining_ == 0 && command_id_ != 0) {
    execute_command();
    expect_command_word_ = true;
    command_busy_ = false;
  }
}

void Mdec::write_command(u32 value) {
  const u32 write_hist_index =
      debug_stats_.write_history_count % static_cast<u32>(DebugStats::kWriteHistory);
  debug_stats_.write_history_words[write_hist_index] = value;
  debug_stats_.write_history_expect_command[write_hist_index] =
      expect_command_word_ ? 1u : 0u;
  debug_stats_.write_history_active_command[write_hist_index] = command_id_;
  ++debug_stats_.write_history_count;

  if (expect_command_word_) {
    begin_command(value);
    return;
  }

  const u16 lo = static_cast<u16>(value & 0xFFFFu);
  const u16 hi = static_cast<u16>(value >> 16);
  if (g_mdec_debug_swap_input_halfwords) {
    in_halfword_fifo_.push_back(hi);
    in_halfword_fifo_.push_back(lo);
  } else {
    in_halfword_fifo_.push_back(lo);
    in_halfword_fifo_.push_back(hi);
  }

  if (!in_unlimited_ && in_words_remaining_ > 0) {
    --in_words_remaining_;
  }

  if (command_id_ == 1) {
    execute_decode();
  }

  if (!in_unlimited_ && in_words_remaining_ == 0) {
    if (command_id_ != 1) {
      execute_command();
    }
    expect_command_word_ = true;
    command_busy_ = false;
  }
}

void Mdec::write_control(u32 value) {
  const u32 control_hist_index =
      debug_stats_.control_history_count % static_cast<u32>(DebugStats::kCommandHistory);
  debug_stats_.control_history_words[control_hist_index] = value;
  ++debug_stats_.control_history_count;
  if (value & 0x80000000u) {
    soft_reset_state();
    // Reset is edge-triggered, but the same write still carries the DMA enable bits.
    control_ = value & 0x7FFFFFFFu;
    return;
  }
  control_ = value;
}

u32 Mdec::read_data() {
  if (out_fifo_.empty()) {
    return 0;
  }
  const u32 value = out_fifo_.front();
  out_fifo_.pop_front();
  if (!out_block_fifo_.empty()) {
    out_block_fifo_.pop_front();
  }
  if (!out_macroblock_fifo_.empty()) {
    out_macroblock_fifo_.pop_front();
  }
  return value;
}

u32 Mdec::dma_out_macroblock_seq() const {
  if (!out_macroblock_fifo_.empty()) {
    return out_macroblock_fifo_.front();
  }
  return current_output_macroblock_seq_;
}

u8 Mdec::dma_out_block() const {
  if (!out_block_fifo_.empty()) {
    return out_block_fifo_.front();
  }
  return current_block_;
}

u32 Mdec::read_status() const {
  u32 status = 0;
  if (out_fifo_.empty()) {
    status |= 1u << 31;
  }
  // bit 30: Data In Full - we use dynamic buffer, so report full if huge
  if (in_halfword_fifo_.size() > 4096) {
      status |= 1u << 30;
  }
  if (command_busy_) {
    status |= 1u << 29;
  }
  if (dma_in_request()) {
    status |= 1u << 28;
  }
  if (dma_out_request()) {
    status |= 1u << 27;
  }

  // Bits 26-23: latched depth, signed, bit15
  status |= (static_cast<u32>(status_command_bits_ & 0x0Fu) << 23);
  
  const u8 block =
      out_block_fifo_.empty() ? current_block_ : out_block_fifo_.front();
  status |= (static_cast<u32>(block & 0x7u) << 16);

  if (!expect_command_word_) {
    if (in_words_remaining_ == 0) {
      status |= 0xFFFFu;
    } else {
      status |= ((in_words_remaining_ - 1u) & 0xFFFFu);
    }
  } else {
    status |= 0xFFFFu;
  }

  return status;
}

bool Mdec::dma_in_request() const {
  const bool dma_in_enabled = (control_ & 0x40000000u) != 0;
  if (!dma_in_enabled) {
    return false;
  }
  if (expect_command_word_) {
    // Channel 0 request mode must be able to deliver command words.
    return true;
  }
  if (in_unlimited_) {
    return true;
  }
  return in_words_remaining_ > 0;
}

bool Mdec::dma_out_request() const {
  const bool dma_out_enabled = (control_ & 0x20000000u) != 0;
  return dma_out_enabled && !out_fifo_.empty();
}

void Mdec::execute_command() {
  switch (command_id_) {
  case 1:
    execute_decode();
    break;
  case 2:
    execute_set_quant_table();
    break;
  case 3:
    execute_set_scale_table();
    break;
  default:
    break;
  }
}

void Mdec::execute_set_quant_table() {
  const bool has_chroma_table = (command_word_ & 0x1u) != 0;
  const size_t target_bytes = has_chroma_table ? 128u : 64u;

  std::array<u8, 128> bytes{};
  size_t byte_count = 0;
  while (!in_halfword_fifo_.empty() && byte_count < target_bytes) {
    const u16 hw = in_halfword_fifo_.front();
    in_halfword_fifo_.pop_front();
    bytes[byte_count++] = static_cast<u8>(hw & 0xFFu);
    if (byte_count < target_bytes) {
      bytes[byte_count++] = static_cast<u8>((hw >> 8) & 0xFFu);
    }
  }

  const size_t luma_bytes = std::min<size_t>(64u, byte_count);
  for (size_t i = 0; i < luma_bytes; ++i) {
    quant_luma_[i] = bytes[i];
  }

  if (has_chroma_table && byte_count > 64u) {
    const size_t chroma_bytes = std::min<size_t>(64u, byte_count - 64u);
    for (size_t i = 0; i < chroma_bytes; ++i) {
      quant_chroma_[i] = bytes[64u + i];
    }
  }

  refresh_debug_quant_stats();
}

void Mdec::execute_set_scale_table() {
  std::array<s16, 64> values{};
  size_t entry = 0;
  while (!in_halfword_fifo_.empty() && entry < values.size()) {
    values[entry++] = static_cast<s16>(in_halfword_fifo_.front());
    in_halfword_fifo_.pop_front();
  }

  // MDEC scale table is uploaded transposed relative to this IDCT indexing.
  for (size_t y = 0; y < 8u; ++y) {
    for (size_t x = 0; x < 8u; ++x) {
      scale_table_[y * 8u + x] = values[x * 8u + y];
    }
  }
}

void Mdec::execute_decode() {
  if (out_depth_latched_ == 2 || out_depth_latched_ == 3) {
    while (true) {
      // Peek for padding
      while (!in_halfword_fifo_.empty() && in_halfword_fifo_.front() == 0xFE00u) {
          in_halfword_fifo_.pop_front();
      }
      if (in_halfword_fifo_.empty()) break;
      size_t available_halfwords = 0;
      if (!scan_macroblock(6u, available_halfwords)) {
        break;
      }

      Block cr{}, cb{}, y1{}, y2{}, y3{}, y4{};
      size_t cursor = 0;

      current_block_ = 4;
      if (!decode_block(cr, quant_chroma_, cursor)) {
        break;
      }
      current_block_ = 5;
      if (!decode_block(cb, quant_chroma_, cursor)) {
        break;
      }
      current_block_ = 0;
      if (!decode_block(y1, quant_luma_, cursor)) {
        break;
      }
      current_block_ = 1;
      if (!decode_block(y2, quant_luma_, cursor)) {
        break;
      }
      current_block_ = 2;
      if (!decode_block(y3, quant_luma_, cursor)) {
        break;
      }
      current_block_ = 3;
      if (!decode_block(y4, quant_luma_, cursor)) {
        break;
      }

      if (g_mdec_debug_compare_macroblocks) {
        std::vector<u16> compare_input;
        compare_input.reserve(cursor);
        for (size_t i = 0; i < cursor; ++i) {
          compare_input.push_back(in_halfword_fifo_[i]);
        }
        compare_colored_macroblock(compare_input);
      }

      emit_colored_macroblock(cr, cb, y1, y2, y3, y4);
      while (available_halfwords-- > 0) {
        in_halfword_fifo_.pop_front();
      }
    }
  } else {
    while (true) {
      while (!in_halfword_fifo_.empty() && in_halfword_fifo_.front() == 0xFE00u) {
          in_halfword_fifo_.pop_front();
      }
      if (in_halfword_fifo_.empty()) break;
      size_t available_halfwords = 0;
      if (!scan_macroblock(1u, available_halfwords)) {
        break;
      }

      Block y{};
      size_t cursor = 0;
      current_block_ = 0;
      if (!decode_block(y, quant_luma_, cursor)) {
        break;
      }
      emit_monochrome_macroblock(y);
      while (available_halfwords-- > 0) {
        in_halfword_fifo_.pop_front();
      }
    }
  }
}

bool Mdec::scan_block(size_t &cursor) const {
  u16 first = 0;
  for (;;) {
    if (cursor >= in_halfword_fifo_.size()) {
      return false;
    }
    first = in_halfword_fifo_[cursor++];
    if (first != 0xFE00u) {
      break;
    }
  }

  int k = 0;
  while (true) {
    if (cursor >= in_halfword_fifo_.size()) {
      return false;
    }
    const u16 word = in_halfword_fifo_[cursor++];
    k += static_cast<int>((word >> 10) & 0x3Fu) + 1;
    if (k >= 63) {
      return true;
    }
  }
}

bool Mdec::scan_macroblock(size_t block_count, size_t &consumed_halfwords) const {
  size_t cursor = 0;
  for (size_t block = 0; block < block_count; ++block) {
    if (!scan_block(cursor)) {
      return false;
    }
  }
  consumed_halfwords = cursor;
  return true;
}

bool Mdec::decode_block(Block &block, const std::array<u8, kBlockSize> &quant_table,
                        size_t &cursor) {
  block.fill(0);

  u16 first = 0;
  for (;;) {
    if (cursor >= in_halfword_fifo_.size()) {
      return false;
    }
    first = in_halfword_fifo_[cursor++];
    if (first != 0xFE00u) {
      break;
    }
    ++debug_stats_.eob_markers;
  }

  const int q_scale = static_cast<int>((first >> 10) & 0x3Fu);
  int k = 0;
  bool has_nonzero_ac = false;
  u32 nonzero_coeffs = 0;

  auto dequantize = [&](u16 word, int index, bool is_first) {
    const int level = sign_extend_10(word);
    const int bias = level ? ((level < 0) ? 8 : -8) : 0;
    if (is_first) {
      const int coeff =
          (q_scale == 0) ? (level << 5)
                         : ((level * static_cast<int>(quant_table[0]) << 4) + bias);
      return std::clamp(coeff, -0x4000, 0x3FFF);
    }

    const int scale = q_scale * static_cast<int>(quant_table[index]);
    const int coeff =
        (scale == 0) ? (level << 5)
                     : ((((level * scale) >> 3) << 4) + bias);
    return std::clamp(coeff, -0x4000, 0x3FFF);
  };

  // q_scale==0 uses linear coefficient placement (no zigzag remap).
  const int dc = dequantize(first, 0, true);
  block[(q_scale == 0) ? 0 : kZigZag[0]] = dc;
  if (dc != 0) {
    ++nonzero_coeffs;
  }

  while (true) {
    if (cursor >= in_halfword_fifo_.size()) {
      return false; // Need more data for EOB
    }
    const u16 word = in_halfword_fifo_[cursor++];

    k += static_cast<int>((word >> 10) & 0x3Fu) + 1;
    if (k < 64) {
      const int coeff = dequantize(word, k, false);
      block[(q_scale == 0) ? k : kZigZag[k]] = coeff;
      if (coeff != 0) {
        has_nonzero_ac = true;
        ++nonzero_coeffs;
      }
    }

    if (k >= 63) {
      break;
    }
  }

  Block spatial{};
  idct(block, spatial);
  for (int &sample : spatial) {
    int signed9 = sample & 0x1FF;
    if ((signed9 & 0x100) != 0) {
      signed9 -= 0x200;
    }
    sample = std::clamp(signed9, -128, 127);
  }
  block = spatial;

  ++debug_stats_.blocks_decoded;
  if (!has_nonzero_ac) {
    ++debug_stats_.dc_only_blocks;
  }
  if (q_scale == 0) {
    ++debug_stats_.qscale_zero_blocks;
  }
  debug_stats_.qscale_sum += static_cast<u64>(q_scale);
  debug_stats_.qscale_max = std::max<u32>(debug_stats_.qscale_max,
                                          static_cast<u32>(q_scale));
  debug_stats_.nonzero_coeff_count += static_cast<u64>(nonzero_coeffs);
  return true;
}

void Mdec::idct(const Block &coeffs, Block &pixels) const {
  auto idct_row = [&](const int *row, const s16 *matrix_row) {
    s64 sum = 0;
    for (u32 i = 0; i < 8u; ++i) {
      sum += static_cast<s64>(row[i]) * static_cast<s32>(matrix_row[i]);
    }
    return static_cast<int>((sum + 0x20000LL) >> 18);
  };

  std::array<int, 64> temp{};
  for (u32 x = 0; x < 8u; ++x) {
    int row[8]{};
    for (u32 i = 0; i < 8u; ++i) {
      row[i] = coeffs[x * 8u + i];
    }
    for (u32 y = 0; y < 8u; ++y) {
      temp[y * 8u + x] = idct_row(row, &scale_table_[y * 8u]);
    }
  }

  for (u32 x = 0; x < 8u; ++x) {
    for (u32 y = 0; y < 8u; ++y) {
      const int sum = idct_row(&temp[x * 8u], &scale_table_[y * 8u]);
      int signed9 = sum & 0x1FF;
      if ((signed9 & 0x100) != 0) {
        signed9 -= 0x200;
      }
      pixels[x * 8u + y] = std::clamp(signed9, -128, 127);
    }
  }
}

void Mdec::compare_colored_macroblock(const std::vector<u16> &input) {
  const u64 compare_count = debug_compare_.macroblocks_compared + 1u;

  DebugCompare probe{};
  probe.captured = true;
  probe.color_macroblock = true;
  probe.macroblocks_compared = compare_count;
  probe.input_halfword_count = static_cast<u32>(input.size());
  const size_t input_sample_count =
      std::min(input.size(), DebugCompare::kInputSampleHalfwords);
  for (size_t i = 0; i < input_sample_count; ++i) {
    probe.input_halfwords[i] = input[i];
  }

  MacroblockBlocks current_coeffs{};
  MacroblockBlocks current_pixels{};
  MacroblockBlocks reference_coeffs{};
  MacroblockBlocks reference_pixels{};
  std::array<u8, DebugCompare::kBlocksPerMacroblock> current_qscales{};
  std::array<u8, DebugCompare::kBlocksPerMacroblock> reference_qscales{};

  if (!decode_colored_macroblock_variant(input, DecodeVariant::Current,
                                         current_coeffs, current_pixels,
                                         current_qscales) ||
      !decode_colored_macroblock_variant(input, DecodeVariant::Reference,
                                         reference_coeffs, reference_pixels,
                                         reference_qscales)) {
    debug_compare_ = probe;
    return;
  }

  probe.qscales = current_qscales;

  for (size_t block = 0; block < DebugCompare::kBlocksPerMacroblock; ++block) {
    for (size_t i = 0; i < kBlockSize; ++i) {
      if (current_coeffs[block][i] != reference_coeffs[block][i]) {
        probe.stage = DebugCompare::Stage::Coeff;
        probe.mismatch_block = static_cast<u8>(block);
        probe.mismatch_index = static_cast<u8>(i);
        probe.current_value = current_coeffs[block][i];
        probe.reference_value = reference_coeffs[block][i];
        debug_compare_ = probe;
        return;
      }
    }
  }

  for (size_t block = 0; block < DebugCompare::kBlocksPerMacroblock; ++block) {
    for (size_t i = 0; i < kBlockSize; ++i) {
      if (current_pixels[block][i] != reference_pixels[block][i]) {
        probe.stage = DebugCompare::Stage::Idct;
        probe.mismatch_block = static_cast<u8>(block);
        probe.mismatch_index = static_cast<u8>(i);
        probe.current_value = current_pixels[block][i];
        probe.reference_value = reference_pixels[block][i];
        debug_compare_ = probe;
        return;
      }
    }
  }

  MacroblockRgb current_rgb{};
  MacroblockRgb reference_rgb{};
  build_macroblock_rgb(current_pixels, current_rgb);
  build_macroblock_rgb(reference_pixels, reference_rgb);

  std::vector<u32> current_words;
  std::vector<u32> reference_words;
  pack_macroblock_words(current_rgb, DecodeVariant::Current, current_words);
  pack_macroblock_words(reference_rgb, DecodeVariant::Reference,
                        reference_words);

  const size_t word_count = std::min(current_words.size(), reference_words.size());
  for (size_t i = 0; i < word_count; ++i) {
    if (current_words[i] != reference_words[i]) {
      probe.stage = DebugCompare::Stage::Rgb;
      probe.mismatch_block = static_cast<u8>(i / 48u);
      probe.mismatch_index = static_cast<u8>(i & 0xFFu);
      probe.current_value = static_cast<s32>(current_words[i]);
      probe.reference_value = static_cast<s32>(reference_words[i]);
      debug_compare_ = probe;
      return;
    }
  }

  if (current_words.size() != reference_words.size()) {
    probe.stage = DebugCompare::Stage::Rgb;
    probe.mismatch_block = 0xFEu;
    probe.mismatch_index = 0u;
    probe.current_value = static_cast<s32>(current_words.size());
    probe.reference_value = static_cast<s32>(reference_words.size());
    debug_compare_ = probe;
    return;
  }

  probe.stage = DebugCompare::Stage::Match;
  debug_compare_ = probe;
}

bool Mdec::decode_colored_macroblock_variant(
    const std::vector<u16> &input, DecodeVariant variant,
    MacroblockBlocks &coeff_blocks, MacroblockBlocks &pixel_blocks,
    std::array<u8, DebugCompare::kBlocksPerMacroblock> &qscales) const {
  size_t cursor = 0;
  if (!decode_block_variant(input, cursor, quant_chroma_, variant,
                            coeff_blocks[0], pixel_blocks[0], qscales[0])) {
    return false;
  }
  if (!decode_block_variant(input, cursor, quant_chroma_, variant,
                            coeff_blocks[1], pixel_blocks[1], qscales[1])) {
    return false;
  }
  if (!decode_block_variant(input, cursor, quant_luma_, variant,
                            coeff_blocks[2], pixel_blocks[2], qscales[2])) {
    return false;
  }
  if (!decode_block_variant(input, cursor, quant_luma_, variant,
                            coeff_blocks[3], pixel_blocks[3], qscales[3])) {
    return false;
  }
  if (!decode_block_variant(input, cursor, quant_luma_, variant,
                            coeff_blocks[4], pixel_blocks[4], qscales[4])) {
    return false;
  }
  if (!decode_block_variant(input, cursor, quant_luma_, variant,
                            coeff_blocks[5], pixel_blocks[5], qscales[5])) {
    return false;
  }
  return true;
}

bool Mdec::decode_block_variant(const std::vector<u16> &input, size_t &cursor,
                                const std::array<u8, kBlockSize> &quant_table,
                                DecodeVariant variant, Block &coeffs,
                                Block &pixels, u8 &q_scale) const {
  coeffs.fill(0);
  pixels.fill(0);

  u16 first = 0;
  for (;;) {
    if (cursor >= input.size()) {
      return false;
    }
    first = input[cursor++];
    if (first != 0xFE00u) {
      break;
    }
  }

  const int q_scale_value =
      (variant == DecodeVariant::Reference)
          ? static_cast<int>(first >> 10)
          : static_cast<int>((first >> 10) & 0x3Fu);
  q_scale = static_cast<u8>(q_scale_value & 0x3Fu);
  int k = 0;

  auto dequantize = [&](u16 word, int index, bool is_first) {
    const int level = sign_extend_10(word);
    const int bias = level ? ((level < 0) ? 8 : -8) : 0;
    if (is_first) {
      const int coeff =
          (q_scale_value == 0)
              ? (level << 5)
              : ((level * static_cast<int>(quant_table[0]) << 4) + bias);
      return std::clamp(coeff, -0x4000, 0x3FFF);
    }

    const int scale = q_scale_value * static_cast<int>(quant_table[index]);
    const int coeff =
        (scale == 0) ? (level << 5)
                     : ((((level * scale) >> 3) << 4) + bias);
    return std::clamp(coeff, -0x4000, 0x3FFF);
  };

  coeffs[(q_scale_value == 0) ? 0 : kZigZag[0]] = dequantize(first, 0, true);

  while (true) {
    if (cursor >= input.size()) {
      return false;
    }
    const u16 word = input[cursor++];
    k += static_cast<int>((word >> 10) & 0x3Fu) + 1;
    if (k < 64) {
      coeffs[(q_scale_value == 0) ? k : kZigZag[k]] =
          dequantize(word, k, false);
    }
    if (k >= 63) {
      break;
    }
  }

  idct_variant(coeffs, pixels, variant);
  return true;
}

void Mdec::idct_variant(const Block &coeffs, Block &pixels,
                        DecodeVariant variant) const {
  if (variant == DecodeVariant::Current) {
    idct(coeffs, pixels);
    return;
  }

  auto idct_row = [&](const int *row, const s16 *matrix_row) {
    s64 sum = 0;
    for (u32 i = 0; i < 8u; ++i) {
      sum += static_cast<s64>(row[i]) * static_cast<s32>(matrix_row[i]);
    }
    return static_cast<int>((sum + 0x20000LL) >> 18);
  };

  std::array<int, 64> temp{};
  for (u32 x = 0; x < 8u; ++x) {
    int row[8]{};
    for (u32 i = 0; i < 8u; ++i) {
      row[i] = coeffs[x * 8u + i];
    }
    for (u32 y = 0; y < 8u; ++y) {
      temp[y * 8u + x] = idct_row(row, &scale_table_[y * 8u]);
    }
  }

  for (u32 x = 0; x < 8u; ++x) {
    for (u32 y = 0; y < 8u; ++y) {
      const int sum = idct_row(&temp[x * 8u], &scale_table_[y * 8u]);
      int signed9 = sum & 0x1FF;
      if ((signed9 & 0x100) != 0) {
        signed9 -= 0x200;
      }
      pixels[x * 8u + y] = std::clamp(signed9, -128, 127);
    }
  }
}

void Mdec::build_macroblock_rgb(const MacroblockBlocks &pixel_blocks,
                                MacroblockRgb &rgb_out) const {
  auto sign_extend_9 = [](int value) {
    int signed9 = value & 0x1FF;
    if ((signed9 & 0x100) != 0) {
      signed9 -= 0x200;
    }
    return signed9;
  };

  const Block &cr = pixel_blocks[0];
  const Block &cb = pixel_blocks[1];
  const std::array<const Block *, 4> y_blocks = {
      &pixel_blocks[2], &pixel_blocks[3], &pixel_blocks[4], &pixel_blocks[5]};

  rgb_out.fill(0);
  for (int block = 0; block < 4; ++block) {
    const int bx = (block & 1) * 8;
    const int by = (block & 2) * 4;
    const Block &y_block = *y_blocks[static_cast<size_t>(block)];
    for (int y = 0; y < 8; ++y) {
      for (int x = 0; x < 8; ++x) {
        const int yy = y_block[y * 8 + x];
        const int cx = (bx + x) >> 1;
        const int cy = (by + y) >> 1;
        const int crv = cr[cy * 8 + cx];
        const int cbv = cb[cy * 8 + cx];

        const int r =
            std::clamp(sign_extend_9(yy + (((359 * crv) + 0x80) >> 8)),
                       -128, 127);
        const int g = std::clamp(
            sign_extend_9(yy + ((((-88 * cbv) & ~0x1F) +
                                  ((-183 * crv) & ~0x07) + 0x80) >> 8)),
            -128, 127);
        const int b =
            std::clamp(sign_extend_9(yy + (((454 * cbv) + 0x80) >> 8)),
                       -128, 127);

        rgb_out[static_cast<size_t>((by + y) * 16 + (bx + x))] =
            static_cast<u32>(encode_component(r)) |
            (static_cast<u32>(encode_component(g)) << 8) |
            (static_cast<u32>(encode_component(b)) << 16);
      }
    }
  }
}

void Mdec::pack_macroblock_words(const MacroblockRgb &rgb, DecodeVariant variant,
                                 std::vector<u32> &words_out) const {
  words_out.clear();
  (void)variant;

  if (out_depth_latched_ == 3u) {
    const auto to15 = [&](u32 color) {
      const auto to5 = [](u32 component) {
        return std::min<u32>(((component + 4u) >> 3), 0x1Fu);
      };
      return static_cast<u16>(
          to5(color & 0xFFu) | (to5((color >> 8) & 0xFFu) << 5) |
          (to5((color >> 16) & 0xFFu) << 10) |
          (output_set_bit15_ ? 0x8000u : 0u));
    };

    for (size_t i = 0; i + 1u < rgb.size(); i += 2u) {
      const u16 p0 = to15(rgb[i]);
      const u16 p1 = to15(rgb[i + 1u]);
      words_out.push_back(static_cast<u32>(p0) |
                          (static_cast<u32>(p1) << 16));
    }
    return;
  }

  u32 word = 0;
  u32 byte_index = 0;
  for (u32 color : rgb) {
    for (u32 shift = 0; shift < 24u; shift += 8u) {
      word |= ((color >> shift) & 0xFFu) << (byte_index * 8u);
      ++byte_index;
      if (byte_index == 4u) {
        words_out.push_back(word);
        word = 0;
        byte_index = 0;
      }
    }
  }
  if (byte_index != 0u) {
    words_out.push_back(word);
  }
}

u8 Mdec::encode_component(int value) const {
  // MDEC color/mono output is based on signed 8-bit components.
  // Unsigned mode is that signed result biased by +128 (aka xor 0x80).
  const int signed_clamped = std::clamp(value, -128, 127);
  if (output_signed_) {
    return static_cast<u8>(static_cast<s8>(signed_clamped));
  }
  return static_cast<u8>(signed_clamped + 128);
}

u16 Mdec::encode_rgb15(int r, int g, int b) const {
  const u8 rr = encode_component(r);
  const u8 rg = encode_component(g);
  const u8 rb = encode_component(b);
  const u16 bit15 = output_set_bit15_ ? 0x8000u : 0u;
  return static_cast<u16>((rr >> 3) | ((rg >> 3) << 5) | ((rb >> 3) << 10) | bit15);
}

void Mdec::emit_colored_macroblock(const Block &cr, const Block &cb,
                                   const Block &y1, const Block &y2,
                                   const Block &y3, const Block &y4) {
  auto sign_extend_9 = [](int value) {
    int signed9 = value & 0x1FF;
    if ((signed9 & 0x100) != 0) {
      signed9 -= 0x200;
    }
    return signed9;
  };

  const std::array<const Block *, 4> y_blocks = {&y1, &y2, &y3, &y4};
  output_word_block_id_ = 4;
  current_output_macroblock_seq_ = output_macroblock_seq_++;

  auto pixel_rgb = [&](int px, int py) {
    const int block = ((py >> 3) << 1) | (px >> 3);
    const Block &y_block = *y_blocks[static_cast<size_t>(block)];
    const bool block_enabled =
        ((g_mdec_debug_color_block_mask >> static_cast<u32>(block)) & 0x1u) != 0u;
    const int local_x = px & 7;
    const int local_y = py & 7;
    const int yy =
        (!block_enabled || g_mdec_debug_disable_luma) ? 0 : y_block[local_y * 8 + local_x];
    const int cx = px >> 1;
    const int cy = py >> 1;
    const int crv =
        (!block_enabled || g_mdec_debug_disable_chroma) ? 0 : cr[cy * 8 + cx];
    const int cbv =
        (!block_enabled || g_mdec_debug_disable_chroma) ? 0 : cb[cy * 8 + cx];

    const int r =
        std::clamp(sign_extend_9(yy + (((359 * crv) + 0x80) >> 8)), -128, 127);
    const int g = std::clamp(
        sign_extend_9(yy + ((((-88 * cbv) & ~0x1F) +
                              ((-183 * crv) & ~0x07) + 0x80) >> 8)),
        -128, 127);
    const int b =
        std::clamp(sign_extend_9(yy + (((454 * cbv) + 0x80) >> 8)), -128, 127);
    return std::array<int, 3>{r, g, b};
  };

  if (out_depth_latched_ == 3) {
    bool have_low = false;
    u16 low_pixel = 0;
    for (int py = 0; py < 16; ++py) {
      for (int px = 0; px < 16; ++px) {
        const auto rgb = pixel_rgb(px, py);
        const u16 rgb15 = encode_rgb15(rgb[0], rgb[1], rgb[2]);
        if (!have_low) {
          low_pixel = rgb15;
          have_low = true;
        } else {
          push_output_word(static_cast<u32>(low_pixel) |
                           (static_cast<u32>(rgb15) << 16));
          have_low = false;
        }
      }
    }
    return;
  }

  for (int py = 0; py < 16; ++py) {
    for (int px = 0; px < 16; ++px) {
      const auto rgb = pixel_rgb(px, py);
      push_output_byte(encode_component(rgb[0]));
      push_output_byte(encode_component(rgb[1]));
      push_output_byte(encode_component(rgb[2]));
    }
  }
}

void Mdec::emit_monochrome_macroblock(const Block &y) {
  output_word_block_id_ = 4;
  current_output_macroblock_seq_ = output_macroblock_seq_++;
  if (out_depth_latched_ == 0) { // 4-bit monochrome
    for (size_t i = 0; i < y.size(); i += 2) {
      const int y0 = g_mdec_debug_disable_luma ? 0 : y[i];
      const int y1 = g_mdec_debug_disable_luma ? 0 : y[i + 1];
      const u8 lo = static_cast<u8>(encode_component(y0) >> 4);
      const u8 hi = static_cast<u8>(encode_component(y1) & 0xF0u);
      push_output_byte(static_cast<u8>(lo | hi));
    }
    return;
  }

  for (int value : y) {
    const int yy = g_mdec_debug_disable_luma ? 0 : value;
    if (out_depth_latched_ == 3) {
      const u16 rgb15 = encode_rgb15(yy, yy, yy);
      push_output_byte(static_cast<u8>(rgb15 & 0xFFu));
      push_output_byte(static_cast<u8>(rgb15 >> 8));
    } else { // 8-bit monochrome (Depth 1)
      push_output_byte(encode_component(yy));
    }
  }
}

void Mdec::push_output_word(u32 value) {
  out_fifo_.push_back(value);
  out_block_fifo_.push_back(output_word_block_id_);
  out_macroblock_fifo_.push_back(current_output_macroblock_seq_);
}

void Mdec::push_output_byte(u8 value) {
  if (g_mdec_debug_force_solid_output) {
    value = 0x80u;
  }
  output_pack_word_ |= static_cast<u32>(value) << (output_pack_bytes_ * 8);
  ++output_pack_bytes_;
  if (output_pack_bytes_ == 4) {
    push_output_word(output_pack_word_);
    output_pack_word_ = 0;
    output_pack_bytes_ = 0;
  }
}

int Mdec::sign_extend_10(u16 value) {
  int result = static_cast<int>(value & 0x03FFu);
  if ((result & 0x0200) != 0) {
    result -= 0x0400;
  }
  return result;
}

int Mdec::clamp_s11(int value) {
  return std::clamp(value, -1024, 1023);
}
