#include "bios.h"
#include <algorithm>
#include <cstring>
#include <fstream>
#include <vector>

bool Bios::load(const std::string &path) {
  loaded_ = false;
  fast_boot_patched_ = false;
  info_.clear();
  mapped_size_ = psx::BIOS_SIZE;
  data_.clear();
  original_data_.clear();

  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file.is_open()) {
    LOG_ERROR("BIOS: Failed to open file: %s", path.c_str());
    return false;
  }

  auto size = file.tellg();
  const bool size_is_standard =
      (size == static_cast<std::streamoff>(psx::BIOS_SIZE));
  if (!size_is_standard) {
    const bool size_is_kb_aligned =
        (size > 0) && ((static_cast<u64>(size) % 1024u) == 0u);
    const bool size_covers_window =
        size >= static_cast<std::streamoff>(psx::BIOS_SIZE);
    if (!g_experimental_bios_size_mode || !size_is_kb_aligned ||
        !size_covers_window) {
      LOG_ERROR("BIOS: Invalid size %lld (expected %u)", (long long)size,
                psx::BIOS_SIZE);
      return false;
    }
  }

  file.seekg(0);
  const size_t file_size = static_cast<size_t>(size);
  data_.resize(file_size);
  file.read(reinterpret_cast<char *>(data_.data()), size);
  if (!file || file.gcount() != size) {
    LOG_ERROR("BIOS: Failed to read image (%lld/%lld bytes)",
              (long long)file.gcount(), (long long)size);
    return false;
  }

  if (size_is_standard) {
    mapped_size_ = psx::BIOS_SIZE;
  } else {
    if (g_unsafe_ps2_bios_mode) {
      mapped_size_ = static_cast<u32>(file_size);
      LOG_WARN("BIOS: UNSAFE PS2 BIOS mode active (%lld bytes mapped).",
               (long long)size);
    } else {
      mapped_size_ = psx::BIOS_SIZE;
      LOG_WARN(
          "BIOS: Experimental size mode active (%lld bytes). Using first %u bytes.",
          (long long)size, psx::BIOS_SIZE);
    }
  }

  loaded_ = true;
  original_data_ = data_;

  identify();
  LOG_INFO("BIOS: Loaded successfully — %s", info_.c_str());
  return true;
}

bool Bios::apply_fast_boot_patch() {
  if (!loaded_ || data_.empty() || original_data_.empty()) {
    return false;
  }

  // Always patch from a clean BIOS image, so toggling the mode is deterministic.
  data_ = original_data_;

  // Type1B signature used by many retail PS1 BIOSes.
  struct PatternByte {
    u8 value;
    bool match;
  };
  static constexpr PatternByte kType1bPattern[] = {
      {0xE0, true}, {0xFF, true}, {0xBD, true}, {0x27, true},
      {0x1C, true}, {0x00, true}, {0xBF, true}, {0xAF, true},
      {0x20, true}, {0x00, true}, {0xA4, true}, {0xAF, true},
      {0x00, false}, {0x00, false}, {0x05, true}, {0x3C, true},
      {0x00, false}, {0x00, false}, {0x06, true}, {0x3C, true},
      {0x00, false}, {0x00, false}, {0xC6, true}, {0x34, true},
      {0x00, false}, {0x00, false}, {0xA5, true}, {0x34, true},
      {0x00, false}, {0x00, false}, {0x00, false}, {0x0F, true},
  };

  auto pattern_matches = [](const std::vector<u8> &blob, size_t offset) {
    if (offset + std::size(kType1bPattern) > blob.size()) {
      return false;
    }
    for (size_t i = 0; i < std::size(kType1bPattern); ++i) {
      if (kType1bPattern[i].match && blob[offset + i] != kType1bPattern[i].value) {
        return false;
      }
    }
    return true;
  };

  u32 patch_offset = 0x00018000u; // Type1A fallback (works on most retail PS1 BIOS images)
  bool found_type1b = false;
  if (data_.size() >= std::size(kType1bPattern)) {
    const size_t max_offset = data_.size() - std::size(kType1bPattern);
    for (size_t i = 0; i <= max_offset; ++i) {
      if (pattern_matches(data_, i)) {
        patch_offset = static_cast<u32>(i);
        found_type1b = true;
        break;
      }
    }
  }

  static constexpr u32 kShellReplacement[] = {
      0x3C011F80u, // lui at, 0x1f80
      0x3C0A0300u, // lui t2, 0x0300
      0xAC2A1814u, // sw t2, 0x1814(at) (enable display)
      0x03E00008u, // jr ra
      0x00000000u, // nop
  };

  const size_t replacement_size = sizeof(kShellReplacement);
  if (static_cast<size_t>(patch_offset) + replacement_size > data_.size()) {
    LOG_WARN("BIOS: Fast-boot patch offset out of range (offset=0x%08X, size=%zu).",
             patch_offset, data_.size());
    return false;
  }

  auto write32_le = [this](size_t offset, u32 value) {
    data_[offset + 0] = static_cast<u8>((value >> 0) & 0xFFu);
    data_[offset + 1] = static_cast<u8>((value >> 8) & 0xFFu);
    data_[offset + 2] = static_cast<u8>((value >> 16) & 0xFFu);
    data_[offset + 3] = static_cast<u8>((value >> 24) & 0xFFu);
  };
  for (size_t i = 0; i < std::size(kShellReplacement); ++i) {
    write32_le(static_cast<size_t>(patch_offset) + (i * 4u), kShellReplacement[i]);
  }

  fast_boot_patched_ = true;
  if (found_type1b) {
    LOG_INFO("BIOS: Applied direct disc boot patch at Type1B offset 0x%08X.", patch_offset);
  } else {
    LOG_INFO("BIOS: Applied direct disc boot patch at fallback Type1A offset 0x%08X.",
             patch_offset);
  }
  return true;
}

void Bios::restore_original_image() {
  if (!loaded_ || original_data_.empty()) {
    return;
  }
  if (fast_boot_patched_) {
    data_ = original_data_;
    fast_boot_patched_ = false;
  }
}

void Bios::identify() {
  // Try to identify the BIOS version by scanning for known strings
  const size_t scan_size =
      std::min<size_t>(data_.size(), static_cast<size_t>(psx::BIOS_SIZE));
  std::string content(reinterpret_cast<const char *>(data_.data()), scan_size);

  struct BiosInfo {
    const char *search;
    const char *name;
  };
  static const BiosInfo known_bioses[] = {
      {"SCPH-1000", "SCPH-1000 (Japan v1.0)"},
      {"SCPH-1001", "SCPH-1001 (North America v2.0)"},
      {"SCPH-1002", "SCPH-1002 (Europe v2.0)"},
      {"SCPH-3000", "SCPH-3000 (Japan v1.1)"},
      {"SCPH-3500", "SCPH-3500 (Japan v2.1)"},
      {"SCPH-5000", "SCPH-5000 (Japan v3.0)"},
      {"SCPH-5500", "SCPH-5500 (Japan v3.0)"},
      {"SCPH-5501", "SCPH-5501 (North America v3.0)"},
      {"SCPH-5502", "SCPH-5502 (Europe v3.0)"},
      {"SCPH-7000", "SCPH-7000 (Japan v4.0)"},
      {"SCPH-7001", "SCPH-7001 (North America v4.1)"},
      {"SCPH-7002", "SCPH-7002 (Europe v4.1)"},
      {"SCPH-7501", "SCPH-7501 (North America v4.4)"},
      {"SCPH-7502", "SCPH-7502 (Europe v4.4)"},
      {"SCPH-9001", "SCPH-9001 (North America v4.5)"},
      {"SCPH-9002", "SCPH-9002 (Europe v4.5)"},
      {"SCPH-100", "PSone SCPH-100 (Japan v4.3)"},
      {"SCPH-101", "PSone SCPH-101 (North America v4.5)"},
      {"SCPH-102", "PSone SCPH-102 (Europe v4.5)"},
  };

  for (const auto &bios : known_bioses) {
    if (content.find(bios.search) != std::string::npos) {
      info_ = bios.name;
      return;
    }
  }

  // Many retail dumps do not include SCPH strings; fall back to ROM version.
  const char *ver_tag = "System ROM Version ";
  std::string best_ver;
  size_t pos = 0;
  while ((pos = content.find(ver_tag, pos)) != std::string::npos) {
    size_t ver_start = pos + std::strlen(ver_tag);
    std::string ver;
    for (size_t i = ver_start; i < content.size() && ver.size() < 48; ++i) {
      const unsigned char ch = static_cast<unsigned char>(content[i]);
      if (ch < 32 || ch > 126) {
        break;
      }
      ver.push_back(static_cast<char>(ch));
    }
    if (!ver.empty()) {
      // Prefer entries with date stamps (e.g. "2.2 12/04/95 A").
      if (ver.find('/') != std::string::npos) {
        best_ver = ver;
      } else if (best_ver.empty()) {
        best_ver = ver;
      }
    }
    pos = ver_start;
  }

  if (!best_ver.empty()) {
    if (best_ver.find("2.2 12/04/95 A") != std::string::npos) {
      info_ = "SCPH-1001 compatible (ROM " + best_ver + ")";
    } else {
      info_ = "PS1 ROM " + best_ver;
    }
    return;
  }

  info_ = "Unknown BIOS (valid size, unknown signature)";
}

u8 Bios::read8(u32 offset) const {
  const u32 size = mapped_size_ == 0 ? psx::BIOS_SIZE : mapped_size_;
  return data_[static_cast<size_t>(offset % size)];
}

u16 Bios::read16(u32 offset) const {
  const u32 size = mapped_size_ == 0 ? psx::BIOS_SIZE : mapped_size_;
  const u16 lo = data_[static_cast<size_t>(offset % size)];
  const u16 hi = data_[static_cast<size_t>((offset + 1) % size)];
  return static_cast<u16>(lo | (hi << 8));
}

u32 Bios::read32(u32 offset) const {
  const u32 size = mapped_size_ == 0 ? psx::BIOS_SIZE : mapped_size_;
  const u32 b0 = data_[static_cast<size_t>(offset % size)];
  const u32 b1 = data_[static_cast<size_t>((offset + 1) % size)];
  const u32 b2 = data_[static_cast<size_t>((offset + 2) % size)];
  const u32 b3 = data_[static_cast<size_t>((offset + 3) % size)];
  return b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
}
