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
  in_unlimited_ = false;
  in_buffer_.clear();
  in_buffer_pos_ = 0;
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
  in_buffer_.clear();
  in_buffer_pos_ = 0;

  in_words_remaining_ = value & 0xFFFFu;
  in_unlimited_ = (in_words_remaining_ == 0xFFFFu);

  switch (command_id_) {
  case 1: // Decode macroblocks
    command_busy_ = true;
    expect_command_word_ = false;
    out_depth_latched_ = output_depth_;
    output_pack_word_ = 0;
    output_pack_bytes_ = 0;
    break;
  case 2: // Set quant table
    in_words_remaining_ = (value & 0x1u) ? 32u : 16u;
    in_unlimited_ = false;
    command_busy_ = true;
    expect_command_word_ = false;
    break;
  case 3: // Set scale table
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

  if (!in_unlimited_ && in_words_remaining_ == 0) {
    execute_command();
    expect_command_word_ = true;
    command_busy_ = false;
  }
}

void Mdec::write_command(u32 value) {
  if (expect_command_word_) {
    begin_command(value);
    return;
  }

  in_buffer_.push_back(value);
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

  if (!expect_command_word_) {
    if (in_unlimited_) {
      status |= 0xFFFFu;
    } else if (in_words_remaining_ > 0) {
      status |= ((in_words_remaining_ - 1u) & 0xFFFFu);
    } else {
      status |= 0xFFFFu;
    }
  } else {
    status |= 0xFFFFu;
  }

  return status;
}

bool Mdec::dma_in_request() const {
  const bool dma_in_enabled = (control_ & 0x40000000u) != 0;
  return dma_in_enabled && !expect_command_word_ && (in_unlimited_ || in_words_remaining_ > 0);
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
  for (u32 word : in_buffer_) {
    for (int shift = 0; shift < 32 && byte_index < quant_luma_.size(); shift += 8) {
      quant_luma_[byte_index++] = static_cast<u8>((word >> shift) & 0xFFu);
    }
  }

  if ((command_word_ & 0x1u) == 0) {
    return;
  }

  byte_index = 0;
  for (size_t i = 16; i < in_buffer_.size() && byte_index < quant_chroma_.size();
       ++i) {
    const u32 word = in_buffer_[i];
    for (int shift = 0; shift < 32 && byte_index < quant_chroma_.size(); shift += 8) {
      quant_chroma_[byte_index++] = static_cast<u8>((word >> shift) & 0xFFu);
    }
  }
}

void Mdec::execute_set_scale_table() {
  size_t entry = 0;
  for (u32 word : in_buffer_) {
    if (entry < scale_table_.size()) {
      scale_table_[entry++] = static_cast<s16>(word & 0xFFFFu);
    }
    if (entry < scale_table_.size()) {
      scale_table_[entry++] = static_cast<s16>((word >> 16) & 0xFFFFu);
    }
  }
}

void Mdec::execute_decode() {
  std::vector<u16> halfwords;
  halfwords.reserve((in_buffer_.size() - in_buffer_pos_) * 2);
  for (size_t i = in_buffer_pos_; i < in_buffer_.size(); ++i) {
    u32 word = in_buffer_[i];
    halfwords.push_back(static_cast<u16>(word & 0xFFFFu));
    halfwords.push_back(static_cast<u16>((word >> 16) & 0xFFFFu));
  }

  size_t pos = 0;
  auto next_non_padding = [&](size_t start) {
    while (start < halfwords.size() && halfwords[start] == 0xFE00u) {
      ++start;
    }
    return start;
  };

  if (out_depth_latched_ == 2 || out_depth_latched_ == 3) {
    while (true) {
      size_t mb_start_pos = next_non_padding(pos);
      size_t current_pos = mb_start_pos;
      
      Block cr{}, cb{}, y1{}, y2{}, y3{}, y4{};
      
      current_block_ = 4;
      if (!decode_block(halfwords, current_pos, cr, quant_chroma_)) break;
      current_block_ = 5;
      if (!decode_block(halfwords, current_pos, cb, quant_chroma_)) break;
      current_block_ = 0;
      if (!decode_block(halfwords, current_pos, y1, quant_luma_)) break;
      current_block_ = 1;
      if (!decode_block(halfwords, current_pos, y2, quant_luma_)) break;
      current_block_ = 2;
      if (!decode_block(halfwords, current_pos, y3, quant_luma_)) break;
      current_block_ = 3;
      if (!decode_block(halfwords, current_pos, y4, quant_luma_)) break;

      emit_colored_macroblock(cr, cb, y1, y2, y3, y4);
      pos = current_pos;
    }
  } else {
    while (true) {
      size_t block_start_pos = next_non_padding(pos);
      size_t current_pos = block_start_pos;
      Block y{};
      current_block_ = 0;
      if (!decode_block(halfwords, current_pos, y, quant_luma_)) break;
      emit_monochrome_macroblock(y);
      pos = current_pos;
    }
  }

  size_t words_consumed = pos / 2;
  in_buffer_pos_ += words_consumed;
  
  if (in_buffer_pos_ >= in_buffer_.size()) {
    in_buffer_.clear();
    in_buffer_pos_ = 0;
  } else if (in_buffer_pos_ > 1024) {
      in_buffer_.erase(in_buffer_.begin(), in_buffer_.begin() + in_buffer_pos_);
      in_buffer_pos_ = 0;
  }
}

bool Mdec::decode_block(const std::vector<u16> &src, size_t &pos, Block &block,
                        const std::array<u8, kBlockSize> &quant_table) {
  size_t temp_pos = pos;
  while (temp_pos < src.size() && src[temp_pos] == 0xFE00u) {
    ++temp_pos;
  }
  if (temp_pos >= src.size()) {
    return false;
  }

  const u16 first = src[temp_pos++];
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

  Block temp_block{};
  temp_block.fill(0);
  if (q_scale == 0) {
    temp_block[0] = dequantize(first, 0, true);
  } else {
    temp_block[kZagZig[0]] = dequantize(first, 0, true);
  }

  bool found_eob = false;
  while (temp_pos < src.size()) {
    const u16 word = src[temp_pos++];
    if (word == 0xFE00u) {
      found_eob = true;
      break;
    }

    k += static_cast<int>((word >> 10) & 0x3Fu) + 1;
    if (k > 63) {
      break;
    }

    const int value = dequantize(word, k, false);
    temp_block[(q_scale == 0) ? k : kZagZig[static_cast<size_t>(k)]] = value;
  }

  if (!found_eob) {
    return false;
  }

  pos = temp_pos;
  idct(temp_block, block);
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
                                            static_cast<size_t>(z) * 8u]) >> 3;
          sum += static_cast<s64>(sample) * static_cast<s64>(scale);
        }
        dst[static_cast<size_t>(x) + static_cast<size_t>(y) * 8u] =
            static_cast<int>((sum + 0x1000) >> 13);
      }
    }
  };

  Block temp{};
  idct_pass(coeffs, temp);
  idct_pass(temp, pixels);
}

u8 Mdec::encode_component(int value) const {
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
  for (int block = 0; block < 4; ++block) {
    const int bx = (block & 1) * 8;
    const int by = (block & 2) * 4;
    output_word_block_id_ = static_cast<u8>(block);
    
    const Block &y_block = (block == 0) ? y1 : (block == 1) ? y2 : (block == 2) ? y3 : y4;

    for (int y = 0; y < 8; ++y) {
      for (int x = 0; x < 8; ++x) {
        const int yy = y_block[y * 8 + x];
        const int cx = (bx + x) >> 1;
        const int cy = (by + y) >> 1;
        const int crv = cr[cy * 8 + cx];
        const int cbv = cb[cy * 8 + cx];

        const int r = yy + ((crv * 5743 + 2048) >> 12);
        const int g = yy - ((cbv * 1410 + crv * 2925 + 2048) >> 12);
        const int b = yy + ((cbv * 7258 + 2048) >> 12);

        if (out_depth_latched_ == 3) {
          const u16 rgb15 = encode_rgb15(r, g, b);
          push_output_byte(static_cast<u8>(rgb15 & 0xFFu));
          push_output_byte(static_cast<u8>(rgb15 >> 8));
        } else {
          push_output_byte(encode_component(r));
          push_output_byte(encode_component(g));
          push_output_byte(encode_component(b));
        }
      }
    }
  }
}

void Mdec::emit_monochrome_macroblock(const Block &y) {
  output_word_block_id_ = 4;
  if (out_depth_latched_ == 0) { // 4-bit monochrome
    for (size_t i = 0; i < y.size(); i += 2) {
      const u8 lo = static_cast<u8>(encode_component(y[i]) >> 4);
      const u8 hi = static_cast<u8>(encode_component(y[i + 1]) & 0xF0u);
      push_output_byte(static_cast<u8>(lo | hi));
    }
    return;
  }

  for (int value : y) {
    if (out_depth_latched_ == 3) {
      const u16 rgb15 = encode_rgb15(value, value, value);
      push_output_byte(static_cast<u8>(rgb15 & 0xFFu));
      push_output_byte(static_cast<u8>(rgb15 >> 8));
    } else if (out_depth_latched_ == 2) { // 8-bit monochrome
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
