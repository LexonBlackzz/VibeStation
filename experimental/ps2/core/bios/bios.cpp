#include "core/bios/bios.h"

#include <algorithm>
#include <array>
#include <fstream>

namespace ps2 {
namespace {

constexpr std::size_t align16(std::size_t value) {
    return (value + 15u) & ~std::size_t{15u};
}

u16 read_le16(const std::vector<u8>& data, std::size_t offset) {
    return static_cast<u16>(data[offset]) |
           (static_cast<u16>(data[offset + 1]) << 8);
}

u32 read_le32(const std::vector<u8>& data, std::size_t offset) {
    return static_cast<u32>(data[offset]) |
           (static_cast<u32>(data[offset + 1]) << 8) |
           (static_cast<u32>(data[offset + 2]) << 16) |
           (static_cast<u32>(data[offset + 3]) << 24);
}

} // namespace

bool Bios::load_file(const std::string& path, std::string& error) {
    error.clear();

    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        error = "Could not open BIOS file.";
        return false;
    }

    const std::streamoff file_size = file.tellg();
    if (file_size != static_cast<std::streamoff>(kSize)) {
        error = "PS2 BIOS must be exactly 4 MiB.";
        return false;
    }

    std::vector<u8> data(kSize);
    file.seekg(0, std::ios::beg);
    if (!file.read(reinterpret_cast<char*>(data.data()),
                   static_cast<std::streamsize>(data.size()))) {
        error = "Failed to read complete BIOS file.";
        return false;
    }

    data_ = std::move(data);
    path_ = path;
    romver_ = extract_romver(data_);
    return true;
}

void Bios::clear() {
    data_.clear();
    data_.shrink_to_fit();
    path_.clear();
    romver_.clear();
}

bool Bios::contains_physical(u32 address, std::size_t width) const {
    if (!loaded() || address < kPhysicalBase) {
        return false;
    }

    const std::size_t offset =
        static_cast<std::size_t>(address - kPhysicalBase);
    return offset <= kSize && width <= (kSize - offset);
}

bool Bios::read8_physical(u32 address, u8& value) const {
    if (!contains_physical(address, 1)) {
        return false;
    }
    value = data_[address - kPhysicalBase];
    return true;
}

bool Bios::read16_physical(u32 address, u16& value) const {
    if (!contains_physical(address, 2)) {
        return false;
    }
    const std::size_t offset = address - kPhysicalBase;
    value = static_cast<u16>(data_[offset]) |
            (static_cast<u16>(data_[offset + 1]) << 8);
    return true;
}

bool Bios::read32_physical(u32 address, u32& value) const {
    if (!contains_physical(address, 4)) {
        return false;
    }
    const std::size_t offset = address - kPhysicalBase;
    value = read_le32(data_, offset);
    return true;
}

bool Bios::read64_physical(u32 address, u64& value) const {
    if (!contains_physical(address, 8)) {
        return false;
    }

    const std::size_t offset = address - kPhysicalBase;
    value = 0;
    for (u32 i = 0; i < 8; ++i) {
        value |= static_cast<u64>(data_[offset + i]) << (i * 8);
    }
    return true;
}

std::string Bios::extract_romver(const std::vector<u8>& data) {
    constexpr std::array<char, 6> kResetName =
        {'R', 'E', 'S', 'E', 'T', '\0'};

    const auto it = std::search(
        data.begin(), data.end(), kResetName.begin(), kResetName.end());
    if (it == data.end()) {
        return {};
    }

    const std::size_t romdir_offset =
        static_cast<std::size_t>(std::distance(data.begin(), it));
    if (romdir_offset + 16 > data.size()) {
        return {};
    }

    // RESET is the first ROMDIR file and occupies all bytes before ROMDIR.
    // Checking this relation avoids treating an unrelated RESET string as
    // the directory.
    const u32 reset_size = read_le32(data, romdir_offset + 12);
    if (reset_size != romdir_offset) {
        return {};
    }

    std::size_t entry_offset = romdir_offset;
    std::size_t file_offset = 0;

    while (entry_offset + 16 <= data.size()) {
        const char* raw_name =
            reinterpret_cast<const char*>(data.data() + entry_offset);

        std::size_t name_length = 0;
        while (name_length < 10 && raw_name[name_length] != '\0') {
            ++name_length;
        }
        if (name_length == 0) {
            break;
        }

        const std::string name(raw_name, name_length);
        const u16 ext_info_size = read_le16(data, entry_offset + 10);
        (void)ext_info_size;
        const u32 file_size = read_le32(data, entry_offset + 12);

        if (name == "ROMVER") {
            if (file_offset + file_size > data.size()) {
                return {};
            }

            std::string result;
            const std::size_t max_length =
                std::min<std::size_t>(file_size, 32);
            for (std::size_t i = 0; i < max_length; ++i) {
                const char c = static_cast<char>(data[file_offset + i]);
                if (c == '\0' || c == '\r' || c == '\n') {
                    break;
                }
                if (c < 0x20 || c > 0x7E) {
                    return {};
                }
                result.push_back(c);
            }
            return result;
        }

        file_offset += align16(file_size);
        if (file_offset > data.size()) {
            return {};
        }

        entry_offset += 16;
    }

    return {};
}

} // namespace ps2
