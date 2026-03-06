#include "bios.h"
#include <algorithm>
#include <fstream>
#include <vector>

bool Bios::load(const std::string &path) {
  loaded_ = false;
  info_.clear();
  mapped_size_ = psx::BIOS_SIZE;
  data_.clear();

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

  identify();
  LOG_INFO("BIOS: Loaded successfully — %s", info_.c_str());
  return true;
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
