#pragma once

#include "common/types.h"

#include <cstddef>
#include <string>
#include <vector>

namespace ps2 {

class Bios {
public:
    static constexpr std::size_t kSize = 4u * 1024u * 1024u;
    static constexpr u32 kPhysicalBase = 0x1FC00000u;
    static constexpr u32 kCachedBase = 0x9FC00000u;
    static constexpr u32 kUncachedBase = 0xBFC00000u;
    static constexpr u32 kResetVector = kUncachedBase;

    bool load_file(const std::string& path, std::string& error);
    void clear();

    [[nodiscard]] bool loaded() const { return data_.size() == kSize; }
    [[nodiscard]] const std::string& path() const { return path_; }
    [[nodiscard]] const std::string& romver() const { return romver_; }

    [[nodiscard]] bool contains_physical(u32 address, std::size_t width) const;
    [[nodiscard]] bool read8_physical(u32 address, u8& value) const;
    [[nodiscard]] bool read16_physical(u32 address, u16& value) const;
    [[nodiscard]] bool read32_physical(u32 address, u32& value) const;
    [[nodiscard]] bool read64_physical(u32 address, u64& value) const;

private:
    static std::string extract_romver(const std::vector<u8>& data);

    std::vector<u8> data_;
    std::string path_;
    std::string romver_;
};

} // namespace ps2
