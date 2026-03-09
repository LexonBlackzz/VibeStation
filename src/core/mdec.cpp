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
  in_halfword_fifo_.clear();
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
  in_halfword_fifo_.clear();

  const u32 length_param = value & 0xFFFFu;
  in_words_remaining_ = length_param;
  in_unlimited_ = false;

  switch (command_id_) {
  case 0: // No-op / Reset?
    expect_command_word_ = true;
    command_busy_ = false;
    break;
  case 1: // Decode macroblocks
    in_words_remaining_ = (value & 0xFFFFu);
    in_unlimited_ = false;
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

  if (!in_unlimited_ && in_words_remaining_ == 0 && command_id_ != 0) {
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

  in_halfword_fifo_.push_back(static_cast<u16>(value & 0xFFFFu));
  in_halfword_fifo_.push_back(static_cast<u16>(value >> 16));

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
}

void Mdec::execute_set_scale_table() {
  size_t entry = 0;
  while (!in_halfword_fifo_.empty() && entry < 64) {
    scale_table_[entry++] = static_cast<s16>(in_halfword_fifo_.front());
    in_halfword_fifo_.pop_front();
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

      Block cr{}, cb{}, y1{}, y2{}, y3{}, y4{};
      
      // Need a way to backtrack if decode_block fails due to missing data.
      // We'll use a temporary copy of the FIFO state.
      auto backup_fifo = in_halfword_fifo_;
      
      current_block_ = 4;
      if (!decode_block(cr, quant_chroma_)) { in_halfword_fifo_ = backup_fifo; break; }
      current_block_ = 5;
      if (!decode_block(cb, quant_chroma_)) { in_halfword_fifo_ = backup_fifo; break; }
      current_block_ = 0;
      if (!decode_block(y1, quant_luma_)) { in_halfword_fifo_ = backup_fifo; break; }
      current_block_ = 1;
      if (!decode_block(y2, quant_luma_)) { in_halfword_fifo_ = backup_fifo; break; }
      current_block_ = 2;
      if (!decode_block(y3, quant_luma_)) { in_halfword_fifo_ = backup_fifo; break; }
      current_block_ = 3;
      if (!decode_block(y4, quant_luma_)) { in_halfword_fifo_ = backup_fifo; break; }

      emit_colored_macroblock(cr, cb, y1, y2, y3, y4);
    }
  } else {
    while (true) {
      while (!in_halfword_fifo_.empty() && in_halfword_fifo_.front() == 0xFE00u) {
          in_halfword_fifo_.pop_front();
      }
      if (in_halfword_fifo_.empty()) break;

      Block y{};
      auto backup_fifo = in_halfword_fifo_;
      current_block_ = 0;
      if (!decode_block(y, quant_luma_)) { in_halfword_fifo_ = backup_fifo; break; }
      emit_monochrome_macroblock(y);
    }
  }
}

bool Mdec::decode_block(Block &block, const std::array<u8, kBlockSize> &quant_table) {
  if (in_halfword_fifo_.empty()) return false;

  const u16 first = in_halfword_fifo_.front();
  in_halfword_fifo_.pop_front();
  const int q_scale = static_cast<int>((first >> 10) & 0x3Fu);
  int k = 0;

  auto dequantize = [&](u16 word, int index, bool is_first) {
    const int level = sign_extend_10(word);
    int val = 0;
    if (q_scale == 0) {
      val = level * 2;
    } else if (is_first) {
      val = level * static_cast<int>(quant_table[0]);
    } else {
      val = (level * static_cast<int>(quant_table[index]) * q_scale + 4) / 8;
    }
    return std::clamp(val, -1024, 1023);
  };

  block.fill(0);
  block[q_scale == 0 ? 0 : kZagZig[0]] = dequantize(first, 0, true);

  while (true) {
    if (in_halfword_fifo_.empty()) return false; // Need more data for EOB
    const u16 word = in_halfword_fifo_.front();
    in_halfword_fifo_.pop_front();

    if (word == 0xFE00u) break; // EOB

    k += static_cast<int>((word >> 10) & 0x3Fu) + 1;
    if (k > 63) break; // Stream error

    block[q_scale == 0 ? k : kZagZig[k]] = dequantize(word, k, false);
  }

  Block spatial{};
  idct(block, spatial);
  block = spatial;
  return true;
}

void Mdec::idct(const Block &coeffs, Block &pixels) const {
  // PSX standard IDCT: Pass 1 >> 15, Pass 2 >> 16.
  // This correctly biases mid-point (512) to 128.
  auto idct_pass = [&](const Block &src, Block &dst, int shift) {
    for (int x = 0; x < 8; ++x) {
      for (int y = 0; y < 8; ++y) {
        s64 sum = 0;
        for (int z = 0; z < 8; ++z) {
          sum += (s64)src[y + z * 8] * scale_table_[x + z * 8];
        }
        dst[x + y * 8] = static_cast<int>((sum + (1LL << (shift - 1))) >> shift);
      }
    }
  };

  Block temp{};
  idct_pass(coeffs, temp, 16);
  idct_pass(temp, pixels, 16);
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

                // CCIR 601 integer math (fixed point 12-bit):
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
    } else { // 8-bit monochrome (Depth 1)
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
