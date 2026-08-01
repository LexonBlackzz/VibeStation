#include "memory_card.h"

#include <algorithm>
#include <filesystem>
#include <fstream>

namespace {
constexpr u8 kOpenBusByte = 0xFF;
constexpr u8 kCardID1 = 0x5A;
constexpr u8 kCardID2 = 0x5D;
constexpr u8 kAck1 = 0x5C;
constexpr u8 kAck2 = 0x5D;
constexpr u8 kEndByte = 0x47;
} // namespace

MemoryCard::MemoryCard() { format_data(data_); }

void MemoryCard::format_data(std::array<u8, kSizeBytes> &data) {
  data.fill(0xFF);

  // Header frame 0.
  {
    u8 *frame = data.data();
    std::fill_n(frame, kFrameSize, static_cast<u8>(0));
    frame[0] = 'M';
    frame[1] = 'C';
    frame[0x7F] = frame_checksum(frame);
  }

  // Directory frames 1..15 (all free).
  for (u32 frame_index = 1; frame_index < 16; ++frame_index) {
    u8 *frame = data.data() + (frame_index * kFrameSize);
    std::fill_n(frame, kFrameSize, static_cast<u8>(0));
    frame[0] = 0xA0;
    frame[8] = 0xFF;
    frame[9] = 0xFF;
    frame[0x7F] = frame_checksum(frame);
  }

  // Broken-sector list frames 16..35.
  for (u32 frame_index = 16; frame_index < 36; ++frame_index) {
    u8 *frame = data.data() + (frame_index * kFrameSize);
    std::fill_n(frame, kFrameSize, static_cast<u8>(0));
    frame[0] = 0xFF;
    frame[1] = 0xFF;
    frame[2] = 0xFF;
    frame[3] = 0xFF;
    frame[8] = 0xFF;
    frame[9] = 0xFF;
    frame[0x7F] = frame_checksum(frame);
  }

  // Broken-sector replacement area frames 36..55.
  for (u32 frame_index = 36; frame_index < 56; ++frame_index) {
    u8 *frame = data.data() + (frame_index * kFrameSize);
    std::fill_n(frame, kFrameSize, static_cast<u8>(0));
  }

  // Unused frames 56..62.
  for (u32 frame_index = 56; frame_index < 63; ++frame_index) {
    u8 *frame = data.data() + (frame_index * kFrameSize);
    std::fill_n(frame, kFrameSize, static_cast<u8>(0));
  }

  // Test frame 63 copies frame 0.
  std::copy_n(data.data(), kFrameSize, data.data() + (63 * kFrameSize));
}

u8 MemoryCard::frame_checksum(const u8 *frame) {
  u8 checksum = 0;
  for (size_t i = 0; i < (kFrameSize - 1); ++i) {
    checksum ^= frame[i];
  }
  return checksum;
}

bool MemoryCard::load_raw_file(const std::string &path,
                               std::array<u8, kSizeBytes> &out_data) {
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    LOG_ERROR("SIO: Failed to open memory card file: %s", path.c_str());
    return false;
  }

  in.seekg(0, std::ios::end);
  const std::streamoff size = in.tellg();
  if (size != static_cast<std::streamoff>(kSizeBytes)) {
    LOG_ERROR("SIO: Memory card size mismatch (%lld bytes): %s",
              static_cast<long long>(size), path.c_str());
    return false;
  }
  in.seekg(0, std::ios::beg);

  in.read(reinterpret_cast<char *>(out_data.data()),
          static_cast<std::streamsize>(out_data.size()));
  if (!in) {
    LOG_ERROR("SIO: Failed reading memory card file: %s", path.c_str());
    return false;
  }
  return true;
}

bool MemoryCard::save_raw_file(const std::string &path,
                               const std::array<u8, kSizeBytes> &data) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out.is_open()) {
    LOG_ERROR("SIO: Failed to write memory card file: %s", path.c_str());
    return false;
  }

  out.write(reinterpret_cast<const char *>(data.data()),
            static_cast<std::streamsize>(data.size()));
  if (!out) {
    LOG_ERROR("SIO: Write error for memory card file: %s", path.c_str());
    return false;
  }
  return true;
}

bool MemoryCard::load_or_create(const std::string &path) {
  if (path.empty()) {
    eject();
    return true;
  }

  if (inserted_ && path_ == path) {
    reset_transfer_state();
    return true;
  }

  std::error_code ec;
  const std::filesystem::path fs_path(path);
  if (!fs_path.parent_path().empty()) {
    std::filesystem::create_directories(fs_path.parent_path(), ec);
    if (ec) {
      LOG_ERROR("SIO: Failed to create memory card directory: %s",
                fs_path.parent_path().string().c_str());
      return false;
    }
  }

  std::array<u8, kSizeBytes> new_data{};
  const bool exists = std::filesystem::exists(fs_path, ec) && !ec;
  if (exists) {
    if (!load_raw_file(path, new_data)) {
      return false;
    }
  } else {
    format_data(new_data);
    if (!save_raw_file(path, new_data)) {
      return false;
    }
  }

  if (!path_.empty() && path_ != path) {
    save_if_dirty();
  }

  data_ = new_data;
  path_ = path;
  inserted_ = true;
  dirty_ = false;
  flags_ = kFlagNoWriteYet;
  reset_transfer_state();

  LOG_INFO("SIO: Memory card mounted: %s", path_.c_str());
  return true;
}

void MemoryCard::eject() {
  save_if_dirty();
  inserted_ = false;
  dirty_ = false;
  path_.clear();
  flags_ = kFlagNoWriteYet;
  format_data(data_);
  reset_transfer_state();
}

bool MemoryCard::save_if_dirty() {
  if (!inserted_ || !dirty_ || path_.empty()) {
    return true;
  }
  if (!save_raw_file(path_, data_)) {
    flags_ |= kFlagWriteError;
    return false;
  }

  dirty_ = false;
  flags_ &= static_cast<u8>(~kFlagWriteError);
  return true;
}

void MemoryCard::reset() {
  reset_transfer_state();
  flags_ = static_cast<u8>((flags_ & kFlagWriteError) | kFlagNoWriteYet);
}

void MemoryCard::reset_transfer_state() {
  state_ = State::Idle;
  address_ = 0;
  sector_offset_ = 0;
  checksum_ = 0;
  last_byte_ = 0;
}

bool MemoryCard::write_byte(u16 address, u8 offset, u8 value) {
  const size_t index =
      (static_cast<size_t>(address) * kFrameSize) + static_cast<size_t>(offset);
  if (index >= data_.size()) {
    return false;
  }
  if (data_[index] != value) {
    dirty_ = true;
  }
  data_[index] = value;
  return true;
}

u8 MemoryCard::read_byte(u16 address, u8 offset) const {
  const size_t index =
      (static_cast<size_t>(address) * kFrameSize) + static_cast<size_t>(offset);
  if (index >= data_.size()) {
    return kOpenBusByte;
  }
  return data_[index];
}

MemoryCard::TransferResult MemoryCard::transfer(u8 data_in) {
  TransferResult result{};
  result.data_out = kOpenBusByte;
  result.ack = false;

  if (!inserted_) {
    last_byte_ = data_in;
    return result;
  }

  auto set_address_msb = [&](u8 value) {
    address_ =
        static_cast<u16>(((address_ & 0x00FFu) | (static_cast<u16>(value) << 8)) &
                         0x03FFu);
  };

  auto set_address_lsb = [&](u8 value) {
    address_ = static_cast<u16>(((address_ & 0xFF00u) | static_cast<u16>(value)) &
                                0x03FFu);
    sector_offset_ = 0;
  };

  switch (state_) {
  case State::Idle:
    if (data_in == 0x81) {
      result.data_out = kOpenBusByte;
      result.ack = true;
      state_ = State::Command;
    }
    break;

  case State::Command:
    result.data_out = flags_;
    switch (data_in) {
    case 0x52: // Read data.
      result.ack = true;
      state_ = State::ReadCardID1;
      break;
    case 0x57: // Write data.
      result.ack = true;
      state_ = State::WriteCardID1;
      break;
    case 0x53: // Get card ID.
      result.ack = true;
      state_ = State::GetIDCardID1;
      break;
    default:
      state_ = State::Idle;
      break;
    }
    break;

  case State::ReadCardID1:
    result.data_out = kCardID1;
    result.ack = true;
    state_ = State::ReadCardID2;
    break;
  case State::ReadCardID2:
    result.data_out = kCardID2;
    result.ack = true;
    state_ = State::ReadAddressMSB;
    break;
  case State::ReadAddressMSB:
    result.data_out = 0x00;
    result.ack = true;
    set_address_msb(data_in);
    state_ = State::ReadAddressLSB;
    break;
  case State::ReadAddressLSB:
    result.data_out = last_byte_;
    result.ack = true;
    set_address_lsb(data_in);
    state_ = State::ReadACK1;
    break;
  case State::ReadACK1:
    result.data_out = kAck1;
    result.ack = true;
    state_ = State::ReadACK2;
    break;
  case State::ReadACK2:
    result.data_out = kAck2;
    result.ack = true;
    state_ = State::ReadConfirmAddressMSB;
    break;
  case State::ReadConfirmAddressMSB:
    result.data_out = static_cast<u8>((address_ >> 8) & 0xFFu);
    result.ack = true;
    state_ = State::ReadConfirmAddressLSB;
    break;
  case State::ReadConfirmAddressLSB:
    result.data_out = static_cast<u8>(address_ & 0xFFu);
    result.ack = true;
    state_ = State::ReadData;
    break;
  case State::ReadData: {
    const u8 value = read_byte(address_, sector_offset_);
    if (sector_offset_ == 0) {
      checksum_ = static_cast<u8>((address_ >> 8) ^ (address_ & 0xFFu) ^ value);
    } else {
      checksum_ ^= value;
    }
    result.data_out = value;
    result.ack = true;
    ++sector_offset_;
    if (sector_offset_ >= kFrameSize) {
      sector_offset_ = 0;
      state_ = State::ReadChecksum;
    }
    break;
  }
  case State::ReadChecksum:
    result.data_out = checksum_;
    result.ack = true;
    state_ = State::ReadEnd;
    break;
  case State::ReadEnd:
    result.data_out = kEndByte;
    // The terminal 0x47 is sent without a DSR/ACK pulse.  Acknowledging it
    // keeps the serial transaction alive for one extra phase and can make a
    // following controller/card selection consume the wrong response byte.
    result.ack = false;
    state_ = State::Idle;
    break;

  case State::WriteCardID1:
    result.data_out = kCardID1;
    result.ack = true;
    state_ = State::WriteCardID2;
    break;
  case State::WriteCardID2:
    result.data_out = kCardID2;
    result.ack = true;
    state_ = State::WriteAddressMSB;
    break;
  case State::WriteAddressMSB:
    result.data_out = 0x00;
    result.ack = true;
    set_address_msb(data_in);
    state_ = State::WriteAddressLSB;
    break;
  case State::WriteAddressLSB:
    result.data_out = last_byte_;
    result.ack = true;
    set_address_lsb(data_in);
    state_ = State::WriteData;
    break;
  case State::WriteData:
    if (sector_offset_ == 0) {
      checksum_ = static_cast<u8>((address_ >> 8) ^ (address_ & 0xFFu) ^ data_in);
      flags_ &= static_cast<u8>(~kFlagNoWriteYet);
    } else {
      checksum_ ^= data_in;
    }

    write_byte(address_, sector_offset_, data_in);
    result.data_out = last_byte_;
    result.ack = true;
    ++sector_offset_;
    if (sector_offset_ >= kFrameSize) {
      sector_offset_ = 0;
      state_ = State::WriteChecksum;
    }
    break;
  case State::WriteChecksum:
    // The card returns the last byte sent by the host at this point.  The
    // checksum is consumed internally and is not echoed back.
    result.data_out = last_byte_;
    result.ack = true;
    state_ = State::WriteACK1;
    break;
  case State::WriteACK1:
    result.data_out = kAck1;
    result.ack = true;
    state_ = State::WriteACK2;
    break;
  case State::WriteACK2:
    result.data_out = kAck2;
    result.ack = true;
    state_ = State::WriteEnd;
    break;
  case State::WriteEnd:
    result.data_out = kEndByte;
    result.ack = false;
    state_ = State::Idle;
    save_if_dirty();
    break;

  case State::GetIDCardID1:
    result.data_out = kCardID1;
    result.ack = true;
    state_ = State::GetIDCardID2;
    break;
  case State::GetIDCardID2:
    result.data_out = kCardID2;
    result.ack = true;
    state_ = State::GetIDACK1;
    break;
  case State::GetIDACK1:
    result.data_out = kAck1;
    result.ack = true;
    state_ = State::GetIDACK2;
    break;
  case State::GetIDACK2:
    result.data_out = kAck2;
    result.ack = true;
    state_ = State::GetID1;
    break;
  case State::GetID1:
    result.data_out = 0x04;
    result.ack = true;
    state_ = State::GetID2;
    break;
  case State::GetID2:
    result.data_out = 0x00;
    result.ack = true;
    state_ = State::GetID3;
    break;
  case State::GetID3:
    result.data_out = 0x00;
    result.ack = true;
    state_ = State::GetID4;
    break;
  case State::GetID4:
    result.data_out = 0x80;
    // Get-ID terminates with 0x80 and no DSR/ACK pulse.  Keeping the card
    // selected here lets the next controller poll consume a card response.
    result.ack = false;
    state_ = State::Idle;
    break;
  }

  last_byte_ = data_in;
  return result;
}
