#pragma once

#include "common/types.h"

#include <fstream>
#include <memory>
#include <string>

namespace ps2 {

// CDVD media types as reported by the drive (CDVD register 0x0F).
enum class DiscType : u8 {
    None = 0x00,
    Ps1Cd = 0x10,
    Ps2Cd = 0x12,
    Ps2Dvd = 0x14,
    DvdVideo = 0xFE,
    Illegal = 0xFF,
};

// Read-only sector image. Supports plain 2048-byte ISOs and raw 2352-byte
// MODE1 images; sectors are always handed out as 2048 bytes of user data.
class DiscImage {
public:
    bool open(const std::string& path, std::string& error);
    void close();

    [[nodiscard]] bool is_open() const { return sector_count_ != 0u; }
    [[nodiscard]] u32 sector_count() const { return sector_count_; }
    [[nodiscard]] const std::string& path() const { return path_; }
    [[nodiscard]] DiscType type() const { return type_; }
    [[nodiscard]] bool is_dvd() const { return sector_count_ > 452849u; }
    // Executable named by SYSTEM.CNF BOOT2/BOOT (empty if none).
    [[nodiscard]] const std::string& boot_path() const { return boot_path_; }

    [[nodiscard]] bool read_sector(u32 lsn, u8* out2048);

private:
    void detect_type();
    [[nodiscard]] bool find_file(
        const char* name, u32& extent, u32& size);

    // Heap-allocated: an ifstream would push Ps2System past the stack budget.
    std::unique_ptr<std::ifstream> file_ = std::make_unique<std::ifstream>();
    std::string path_;
    std::string boot_path_;
    u32 sector_count_ = 0;
    u32 sector_size_ = 2048;
    u32 data_offset_ = 0;
    DiscType type_ = DiscType::None;
};

} // namespace ps2
