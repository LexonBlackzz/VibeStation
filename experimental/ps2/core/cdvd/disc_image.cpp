#include "core/cdvd/disc_image.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstring>
#include <fstream>
#include <vector>

namespace ps2 {
namespace {

u32 le32(const u8* p) {
    return static_cast<u32>(p[0]) | (static_cast<u32>(p[1]) << 8) |
           (static_cast<u32>(p[2]) << 16) | (static_cast<u32>(p[3]) << 24);
}

bool same_name(const u8* name, u32 length, const char* want) {
    // Directory records carry names like "SYSTEM.CNF;1".
    u32 i = 0;
    for (; i < length && name[i] != ';'; ++i) {
        if (want[i] == '\0' ||
            std::toupper(name[i]) != std::toupper(static_cast<u8>(want[i]))) {
            return false;
        }
    }
    return want[i] == '\0';
}

} // namespace

bool DiscImage::open(const std::string& cue_or_image, std::string& error) {
    close();
    std::string path = cue_or_image;
    std::string ext = path.size() > 4u ? path.substr(path.size() - 4u) : "";
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (ext == ".cue") {
        // Only the first FILE (the data track) is mapped; audio tracks are not.
        std::ifstream cue(path);
        std::string line;
        path.clear();
        while (std::getline(cue, line)) {
            const auto q1 = line.find('"');
            const auto q2 = line.find('"', q1 + 1u);
            if (line.find("FILE") != std::string::npos &&
                q1 != std::string::npos && q2 != std::string::npos) {
                const auto dir = cue_or_image.find_last_of("/\\");
                path = (dir == std::string::npos
                            ? std::string()
                            : cue_or_image.substr(0, dir + 1u)) +
                       line.substr(q1 + 1u, q2 - q1 - 1u);
                break;
            }
        }
        if (path.empty()) {
            error = "No FILE entry in cue sheet: " + cue_or_image;
            return false;
        }
    }
    file_->open(path, std::ios::binary | std::ios::ate);
    if (!*file_) {
        error = "Cannot open disc image: " + path;
        return false;
    }
    const auto bytes = static_cast<u64>(file_->tellg());

    // Plain ISOs are a multiple of 2048; raw images use 2352-byte sectors with
    // user data at +16 (MODE1) or +24 (MODE2 XA, i.e. PS1 discs). A raw image
    // starts with a sync pattern, which tells it apart from an ISO of a size
    // divisible by both.
    std::array<u8, 16> head{};
    file_->seekg(0);
    file_->read(reinterpret_cast<char*>(head.data()), 16);
    const bool raw = bytes % 2352u == 0u && head[0] == 0x00u &&
                     head[1] == 0xFFu && head[11] == 0x00u;
    if (raw) {
        sector_size_ = 2352u;
        data_offset_ = head[15] == 2u ? 24u : 16u;
    } else if (bytes % 2048u == 0u) {
        sector_size_ = 2048u;
        data_offset_ = 0u;
    } else {
        error = "Unsupported disc image size: " + path;
        file_->close();
        return false;
    }
    sector_count_ = static_cast<u32>(bytes / sector_size_);
    path_ = cue_or_image;
    detect_type();
    return true;
}

void DiscImage::close() {
    file_->close();
    file_->clear();
    path_.clear();
    boot_path_.clear();
    sector_count_ = 0;
    type_ = DiscType::None;
}

bool DiscImage::read_sector(u32 lsn, u8* out2048) {
    if (lsn >= sector_count_) return false;
    file_->clear();
    file_->seekg(
        static_cast<std::streamoff>(lsn) * sector_size_ + data_offset_);
    file_->read(reinterpret_cast<char*>(out2048), 2048);
    return static_cast<bool>(*file_);
}

// Looks a file up in the ISO9660 root directory.
bool DiscImage::find_file(const char* name, u32& extent, u32& size) {
    std::array<u8, 2048> sector{};
    if (!read_sector(16u, sector.data()) ||
        std::memcmp(sector.data() + 1, "CD001", 5) != 0) {
        return false;
    }
    const u32 root_extent = le32(sector.data() + 156 + 2);
    const u32 root_size = le32(sector.data() + 156 + 10);
    for (u32 offset = 0; offset < root_size; offset += 2048u) {
        if (!read_sector(root_extent + offset / 2048u, sector.data())) {
            return false;
        }
        u32 pos = 0;
        while (pos < 2048u && sector[pos] != 0u) {
            const u8* record = sector.data() + pos;
            const u32 name_length = record[32];
            if (same_name(record + 33, name_length, name)) {
                extent = le32(record + 2);
                size = le32(record + 10);
                return true;
            }
            pos += record[0];
        }
    }
    return false;
}

void DiscImage::detect_type() {
    type_ = DiscType::Illegal;
    boot_path_.clear();

    u32 extent = 0;
    u32 size = 0;
    if (find_file("SYSTEM.CNF", extent, size) && size != 0u) {
        std::vector<u8> text(((size + 2047u) / 2048u) * 2048u);
        for (u32 i = 0; i < text.size() / 2048u; ++i) {
            if (!read_sector(extent + i, text.data() + i * 2048u)) return;
        }
        const std::string cnf(reinterpret_cast<char*>(text.data()), size);
        const bool ps2 = cnf.find("BOOT2") != std::string::npos;
        const std::size_t key = cnf.find(ps2 ? "BOOT2" : "BOOT");
        if (key != std::string::npos) {
            const std::size_t eq = cnf.find('=', key);
            const std::size_t end = cnf.find_first_of("\r\n", eq);
            if (eq != std::string::npos) {
                boot_path_ = cnf.substr(eq + 1, end - eq - 1);
                const auto first = boot_path_.find_first_not_of(" \t");
                const auto last = boot_path_.find_last_not_of(" \t");
                boot_path_ = first == std::string::npos
                    ? std::string()
                    : boot_path_.substr(first, last - first + 1);
            }
            type_ = ps2 ? (is_dvd() ? DiscType::Ps2Dvd : DiscType::Ps2Cd)
                        : DiscType::Ps1Cd;
        }
        return;
    }
    u32 ignored = 0;
    if (find_file("VIDEO_TS", extent, ignored)) type_ = DiscType::DvdVideo;
}

} // namespace ps2
