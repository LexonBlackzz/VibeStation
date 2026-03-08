#include "mdec.h"
#include <algorithm>
#include <array>

namespace {
// Run-length stream index -> 8x8 matrix index (PSX MDEC zagzig order).
constexpr std::array<int, 64> kZagZig = {
  0 ,1 ,8 ,16,9 ,2 ,3 ,10,
  17,24,32,25,18,11,4 ,5 ,
  12,19,26,33,40,48,41,34,
  27,20,13,6 ,7 ,14,21,28,
  35,42,49,56,57,50,43,36,
  29,22,15,23,30,37,44,51,
  58,59,52,45,38,31,39,46,
  53,60,61,54,47,55,62,63,
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

void Mdec::reset() {
  control_ = 0;
  command_busy_ = false;
  expect_command_word_ = true;
  command_id_ = 0;
  command_word_ = 0;
  in_words_remaining_ = 0;
  input_words_.clear();
  quant_luma_.fill(1);
  quant_chroma_.fill(1);
  scale_table_ = kDefaultScaleTable;
  status_command_bits_ = 0;
  current_block_ = 4;
  output_depth_ = 2;
  output_signed_ = false;
  output_set_bit15_ = false;
  output_pack_word_ = 0;
  output_pack_bytes_ = 0;
  output_word_block_id_ = 4;
  out_depth_latched_ = 2;
  out_fifo_.clear();
  out_block_fifo_.clear();
}

void Mdec::begin_command(u32 value) {
  command_word_ = value;
  command_id_ = static_cast<u8>((value >> 29) & 0x7u);
  status_command_bits_ = static_cast<u8>((value >> 25) & 0x0Fu);
  output_depth_ = static_cast<u8>((value >> 27) & 0x3u);
  output_signed_ = (value & (1u << 26)) != 0;
  output_set_bit15_ = (value & (1u << 25)) != 0;
  input_words_.clear();

  switch (command_id_) {
  case 1: // Decode macroblocks
    in_words_remaining_ = value & 0xFFFFu;
    command_busy_ = true;
    expect_command_word_ = false;
    break;
  case 2: // Set quant table
    in_words_remaining_ = (value & 0x1u) ? 32u : 16u;
    command_busy_ = true;
    expect_command_word_ = false;
    break;
  case 3: // Set scale table
    in_words_remaining_ = 32u;
    command_busy_ = true;
    expect_command_word_ = false;
    break;
  default:
    finish_command();
    break;
  }

  if (in_words_remaining_ == 0) {
    execute_command();
    finish_command();
  }

  LOG_DEBUG("MDEC: begin cmd=0x%08X id=%u words=%u depth=%u signed=%u bit15=%u",
            command_word_, static_cast<unsigned>(command_id_),
            static_cast<unsigned>(in_words_remaining_),
            static_cast<unsigned>(output_depth_), output_signed_ ? 1u : 0u,
            output_set_bit15_ ? 1u : 0u);
}

void Mdec::finish_command() {
  expect_command_word_ = true;
  command_busy_ = false;
  in_words_remaining_ = 0;
  input_words_.clear();
  current_block_ = 4;
}

void Mdec::write_command(u32 value) {
  if (expect_command_word_) {
    begin_command(value);
    return;
  }

  input_words_.push_back(value);
  if (in_words_remaining_ > 0) {
    --in_words_remaining_;
  }

  if (in_words_remaining_ == 0) {
    execute_command();
    finish_command();
  }
}

void Mdec::write_control(u32 value) {
  if (value & 0x80000000u) {
    reset();
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
  return value;
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
  if (command_busy_) {
    status |= 1u << 29;
  }
  if (dma_in_request()) {
    status |= 1u << 28;
  }
  if (dma_out_request()) {
    status |= 1u << 27;
  }

  status |= (static_cast<u32>(status_command_bits_ & 0x0Fu) << 23);
  const u8 block =
      out_block_fifo_.empty() ? current_block_ : out_block_fifo_.front();
  status |= (static_cast<u32>(block & 0x7u) << 16);

  if (!expect_command_word_ && in_words_remaining_ > 0) {
    status |= ((in_words_remaining_ - 1u) & 0xFFFFu);
  } else {
    status |= 0xFFFFu;
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
  size_t byte_index = 0;
  for (u32 word : input_words_) {
    for (int shift = 0; shift < 32 && byte_index < quant_luma_.size(); shift += 8) {
      quant_luma_[byte_index++] = static_cast<u8>((word >> shift) & 0xFFu);
    }
  }

  if ((command_word_ & 0x1u) == 0) {
    return;
  }

  byte_index = 0;
  for (size_t i = 16; i < input_words_.size() && byte_index < quant_chroma_.size();
       ++i) {
    const u32 word = input_words_[i];
    for (int shift = 0; shift < 32 && byte_index < quant_chroma_.size(); shift += 8) {
      quant_chroma_[byte_index++] = static_cast<u8>((word >> shift) & 0xFFu);
    }
  }
}

void Mdec::execute_set_scale_table() {
  size_t entry = 0;
  for (u32 word : input_words_) {
    if (entry < scale_table_.size()) {
      scale_table_[entry++] = static_cast<s16>(word & 0xFFFFu);
    }
    if (entry < scale_table_.size()) {
      scale_table_[entry++] = static_cast<s16>((word >> 16) & 0xFFFFu);
    }
  }
}

void Mdec::execute_decode() {
  out_depth_latched_ = output_depth_;
  const size_t out_before = out_fifo_.size();
  std::vector<u16> halfwords;
  halfwords.reserve(input_words_.size() * 2);
  for (u32 word : input_words_) {
    halfwords.push_back(static_cast<u16>(word & 0xFFFFu));
    halfwords.push_back(static_cast<u16>((word >> 16) & 0xFFFFu));
  }

  std::array<u16, 8> src_preview = {0, 0, 0, 0, 0, 0, 0, 0};
  for (size_t i = 0; i < src_preview.size() && i < halfwords.size(); ++i) {
    src_preview[i] = halfwords[i];
  }

  size_t pos = 0;
  size_t macroblock_count = 0;
  output_pack_word_ = 0;
  output_pack_bytes_ = 0;
  static bool logged_first_decode = false;

  auto next_non_padding = [&](size_t start) {
    while (start < halfwords.size() && halfwords[start] == 0xFE00u) {
      ++start;
    }
    return start;
  };

  if (output_depth_ == 2 || output_depth_ == 3) {
    while (true) {
      pos = next_non_padding(pos);
      if (pos >= halfwords.size()) {
        break;
      }

      Block cr{}, cb{}, y1{}, y2{}, y3{}, y4{};
      current_block_ = 4;
      if (!decode_block(halfwords, pos, cr, quant_chroma_)) {
        break;
      }
      current_block_ = 5;
      if (!decode_block(halfwords, pos, cb, quant_chroma_)) {
        break;
      }
      current_block_ = 0;
      if (!decode_block(halfwords, pos, y1, quant_luma_)) {
        break;
      }
      current_block_ = 1;
      if (!decode_block(halfwords, pos, y2, quant_luma_)) {
        break;
      }
      current_block_ = 2;
      if (!decode_block(halfwords, pos, y3, quant_luma_)) {
        break;
      }
      current_block_ = 3;
      if (!decode_block(halfwords, pos, y4, quant_luma_)) {
        break;
      }

      if (!logged_first_decode) {
        auto block_minmax = [](const Block &block) {
          auto [min_it, max_it] = std::minmax_element(block.begin(), block.end());
          return std::pair<int, int>(*min_it, *max_it);
        };
        const auto [cr_min, cr_max] = block_minmax(cr);
        const auto [cb_min, cb_max] = block_minmax(cb);
        const auto [y1_min, y1_max] = block_minmax(y1);
        const auto [y2_min, y2_max] = block_minmax(y2);
        const auto [y3_min, y3_max] = block_minmax(y3);
        const auto [y4_min, y4_max] = block_minmax(y4);

        const int yy = y1[0];
        const int cbv = cb[0];
        const int crv = cr[0];
        const int r = yy + ((359 * crv + 0x80) >> 8);
        const int g = yy - ((88 * cbv + 183 * crv + 0x80) >> 8);
        const int b = yy + ((454 * cbv + 0x80) >> 8);

        LOG_DEBUG(
            "MDEC: mb0 qlum=%u qchr=%u cr0=%d cb0=%d y00=%d blocks="
            "cr[%d,%d] cb[%d,%d] y1[%d,%d] y2[%d,%d] y3[%d,%d] y4[%d,%d] "
            "rgb0=%d,%d,%d enc=%02X,%02X,%02X",
            static_cast<unsigned>(quant_luma_[0]),
            static_cast<unsigned>(quant_chroma_[0]), cr[0], cb[0], y1[0],
            cr_min, cr_max, cb_min, cb_max, y1_min, y1_max, y2_min, y2_max,
            y3_min, y3_max, y4_min, y4_max, r, g, b, encode_component(r),
            encode_component(g), encode_component(b));
        logged_first_decode = true;
      }

      emit_colored_macroblock(cr, cb, y1, y2, y3, y4);
      ++macroblock_count;
    }
  } else {
    while (true) {
      pos = next_non_padding(pos);
      if (pos >= halfwords.size()) {
        break;
      }
      Block y{};
      current_block_ = 4;
      if (!decode_block(halfwords, pos, y, quant_luma_)) {
        break;
      }
      emit_monochrome_macroblock(y);
      ++macroblock_count;
    }
  }

  if (output_pack_bytes_ != 0) {
    out_fifo_.push_back(output_pack_word_);
    out_block_fifo_.push_back(output_word_block_id_);
    output_pack_word_ = 0;
    output_pack_bytes_ = 0;
  }

  const size_t produced_words = out_fifo_.size() - out_before;
  size_t nonzero_words = 0;
  std::array<u32, 4> first_words = {0, 0, 0, 0};
  for (size_t i = 0; i < produced_words; ++i) {
    const u32 value = out_fifo_[out_before + i];
    if (i < first_words.size()) {
      first_words[i] = value;
    }
    if (value != 0) {
      ++nonzero_words;
    }
  }

  LOG_DEBUG(
      "MDEC: decode words_in=%zu halfwords=%zu src=%04X,%04X,%04X,%04X,"
      "%04X,%04X,%04X,%04X macroblocks=%zu words_out=%zu nonzero=%zu "
      "first=%08X,%08X,%08X,%08X depth=%u",
      input_words_.size(), halfwords.size(), src_preview[0], src_preview[1],
      src_preview[2], src_preview[3], src_preview[4], src_preview[5],
      src_preview[6], src_preview[7], macroblock_count, produced_words,
      nonzero_words,
      first_words[0], first_words[1], first_words[2], first_words[3],
      static_cast<unsigned>(output_depth_));
}

bool Mdec::decode_block(const std::vector<u16> &src, size_t &pos, Block &block,
                        const std::array<u8, kBlockSize> &quant_table) {
  block.fill(0);

  while (pos < src.size() && src[pos] == 0xFE00u) {
    ++pos;
  }
  if (pos >= src.size()) {
    return false;
  }

  const u16 first = src[pos++];
  const int q_scale = static_cast<int>((first >> 10) & 0x3Fu);
  int k = 0;

  auto dequantize = [&](u16 word, int index, bool first_coeff) {
    const int level = sign_extend_10(word);
    int value = 0;
    if (q_scale == 0) {
      value = level * 2;
    } else if (first_coeff) {
      value = level * static_cast<int>(quant_table[0]);
    } else {
      value = (level * static_cast<int>(quant_table[index]) * q_scale + 4) / 8;
    }
    return clamp_s11(value);
  };

  if (q_scale == 0) {
    block[0] = dequantize(first, 0, true);
  } else {
    block[kZagZig[0]] = dequantize(first, 0, true);
  }

  while (pos < src.size()) {
    const u16 word = src[pos++];
    if (word == 0xFE00u) {
      break;
    }

    k += static_cast<int>((word >> 10) & 0x3Fu) + 1;
    if (k > 63) {
      break;
    }

    const int value = dequantize(word, k, false);
    block[(q_scale == 0) ? k : kZagZig[static_cast<size_t>(k)]] = value;
  }

  Block spatial{};
  idct(block, spatial);
  block = spatial;
  return true;
}

void Mdec::idct(const Block &coeffs, Block &pixels) const {
  auto idct_pass = [&](const Block &src, Block &dst) {
    for (int x = 0; x < 8; ++x) {
      for (int y = 0; y < 8; ++y) {
        s64 sum = 0;
        for (int z = 0; z < 8; ++z) {
          const int sample =
              src[static_cast<size_t>(y) + static_cast<size_t>(z) * 8u];
          const int scale =
              static_cast<int>(scale_table_[static_cast<size_t>(x) +
                                            static_cast<size_t>(z) * 8u]) >>
              3;
          sum += static_cast<s64>(sample) * static_cast<s64>(scale);
        }
        dst[static_cast<size_t>(x) + static_cast<size_t>(y) * 8u] =
            static_cast<int>((sum + 0x0FFF) >> 13);
      }
    }
  };

  Block temp{};
  idct_pass(coeffs, temp);
  idct_pass(temp, pixels);
}

u8 Mdec::encode_component(int value) const {
  // PSX MDEC standard output is 8-bit unsigned.
  // Internal values are typically centered around 0 if signed, or 128 if unsigned.
  int clamped = value;
  if (!output_signed_) {
    clamped += 128;
  }
  return static_cast<u8>(std::clamp(clamped, 0, 255));
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
  // A macroblock is 16x16 pixels composed of four 8x8 luma blocks (Y1-Y4).
  // The Cr and Cb blocks are 8x8 and cover the entire 16x16 area.
  auto get_y = [&](int x, int y) {
    if (y < 8) {
      return (x < 8) ? y1[y * 8 + x] : y2[y * 8 + (x - 8)];
    } else {
      return (x < 8) ? y3[(y - 8) * 8 + x] : y4[(y - 8) * 8 + (x - 8)];
    }
  };

  for (int y = 0; y < 16; ++y) {
    for (int x = 0; x < 16; ++x) {
      if (y < 8) {
        output_word_block_id_ = (x < 8) ? 0 : 1;
      } else {
        output_word_block_id_ = (x < 8) ? 2 : 3;
      }

      const int yy = get_y(x, y);
      const int cx = x >> 1;
      const int cy = y >> 1;
      const int crv = cr[cy * 8 + cx];
      const int cbv = cb[cy * 8 + cx];

      // CCIR 601 integer math (fixed point 12-bit):
      const int r = yy + ((crv * 5743 + 2048) >> 12);
      const int g = yy - ((cbv * 1410 + crv * 2925 + 2048) >> 12);
      const int b = yy + ((cbv * 7258 + 2048) >> 12);

      if (output_depth_ == 3) {
        const u16 rgb15 = encode_rgb15(r, g, b);
        push_output_byte(static_cast<u8>(rgb15 & 0xFFu));
        push_output_byte(static_cast<u8>(rgb15 >> 8));
      } else {
        // 24-bit RGB is stored as (R, G, B) in bytes.
        push_output_byte(encode_component(r));
        push_output_byte(encode_component(g));
        push_output_byte(encode_component(b));
      }
    }
  }
}

void Mdec::emit_monochrome_macroblock(const Block &y) {
  output_word_block_id_ = 4;
  if (output_depth_ == 0) { // 4-bit monochrome
    for (size_t i = 0; i < y.size(); i += 2) {
      const u8 lo = static_cast<u8>(encode_component(y[i]) >> 4);
      const u8 hi = static_cast<u8>(encode_component(y[i + 1]) & 0xF0u);
      push_output_byte(static_cast<u8>(lo | hi));
    }
    return;
  }

  for (int value : y) {
    if (output_depth_ == 3) {
      const u16 rgb15 = encode_rgb15(value, value, value);
      push_output_byte(static_cast<u8>(rgb15 & 0xFFu));
      push_output_byte(static_cast<u8>(rgb15 >> 8));
    } else if (output_depth_ == 2) { // 8-bit monochrome
      push_output_byte(encode_component(value));
    }
  }
}

void Mdec::push_output_byte(u8 value) {
  output_pack_word_ |= static_cast<u32>(value) << (output_pack_bytes_ * 8);
  ++output_pack_bytes_;
  if (output_pack_bytes_ == 4) {
    out_fifo_.push_back(output_pack_word_);
    out_block_fifo_.push_back(output_word_block_id_);
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

int Mdec::clamp_s8(int value) {
  return std::clamp(value, -128, 127);
}
