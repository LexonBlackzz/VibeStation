#pragma once
#include "types.h"
#include <string>
#include <vector>


// ── BIOS ROM (512KB) ───────────────────────────────────────────────
// Loads and provides read-only access to the PS1 BIOS.
// Supports all known BIOS versions (SCPH-1001, 5501, 7001, etc.)

class Bios {
public:
  bool load(const std::string &path);
  bool is_loaded() const { return loaded_; }
  bool apply_fast_boot_patch();
  void restore_original_image();
  bool fast_boot_patched() const { return fast_boot_patched_; }

  u8 read8(u32 offset) const;
  u16 read16(u32 offset) const;
  u32 read32(u32 offset) const;
  u32 mapped_size() const { return mapped_size_; }

  const std::string &get_info() const { return info_; }

private:
  std::vector<u8> data_{};
  std::vector<u8> original_data_{};
  bool loaded_ = false;
  bool fast_boot_patched_ = false;
  std::string info_;
  u32 mapped_size_ = psx::BIOS_SIZE;

  void identify();
};
