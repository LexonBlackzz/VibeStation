#include "ui/screenshot_utils.h"
#include <algorithm>
#include <cstddef>

namespace {
    void append_be32(std::vector<u8>& out, u32 value) {
        out.push_back(static_cast<u8>((value >> 24) & 0xFFu));
        out.push_back(static_cast<u8>((value >> 16) & 0xFFu));
        out.push_back(static_cast<u8>((value >> 8) & 0xFFu));
        out.push_back(static_cast<u8>(value & 0xFFu));
    }

    u32 crc32_png(const u8* data, size_t size) {
        u32 crc = 0xFFFFFFFFu;
        for (size_t i = 0; i < size; ++i) {
            crc ^= data[i];
            for (int bit = 0; bit < 8; ++bit) {
                const u32 mask = static_cast<u32>(-(static_cast<int>(crc & 1u)));
                crc = (crc >> 1) ^ (0xEDB88320u & mask);
            }
        }
        return ~crc;
    }

    u32 adler32_zlib(const u8* data, size_t size) {
        constexpr u32 kMod = 65521u;
        u32 a = 1u;
        u32 b = 0u;
        for (size_t i = 0; i < size; ++i) {
            a = (a + data[i]) % kMod;
            b = (b + a) % kMod;
        }
        return (b << 16) | a;
    }

    void append_png_chunk(std::vector<u8>& out, const char type[4],
        const std::vector<u8>& data) {
        append_be32(out, static_cast<u32>(data.size()));
        const size_t type_offset = out.size();
        out.push_back(static_cast<u8>(type[0]));
        out.push_back(static_cast<u8>(type[1]));
        out.push_back(static_cast<u8>(type[2]));
        out.push_back(static_cast<u8>(type[3]));
        out.insert(out.end(), data.begin(), data.end());
        const u32 crc =
            crc32_png(out.data() + type_offset, static_cast<size_t>(4u + data.size()));
        append_be32(out, crc);
    }
}

void resample_rgba_nearest(const std::vector<u32>& src, int src_width, int src_height,
    std::vector<u32>& dst, int dst_width, int dst_height) {
    const int safe_src_width = std::max(1, src_width);
    const int safe_src_height = std::max(1, src_height);
    const int safe_dst_width = std::max(1, dst_width);
    const int safe_dst_height = std::max(1, dst_height);
    dst.resize(static_cast<size_t>(safe_dst_width) * static_cast<size_t>(safe_dst_height));
    for (int y = 0; y < safe_dst_height; ++y) {
        const int src_y = (y * safe_src_height) / safe_dst_height;
        const size_t dst_row = static_cast<size_t>(y) * static_cast<size_t>(safe_dst_width);
        const size_t src_row = static_cast<size_t>(src_y) * static_cast<size_t>(safe_src_width);
        for (int x = 0; x < safe_dst_width; ++x) {
            const int src_x = (x * safe_src_width) / safe_dst_width;
            dst[dst_row + static_cast<size_t>(x)] =
                src[src_row + static_cast<size_t>(src_x)];
        }
    }
}

bool encode_rgba_png(const std::vector<u32>& rgba, int width, int height,
    std::vector<u8>& out_png) {
    if (width <= 0 || height <= 0) {
        return false;
    }
    const size_t pixel_count =
        static_cast<size_t>(width) * static_cast<size_t>(height);
    if (rgba.size() < pixel_count) {
        return false;
    }

    const size_t row_bytes = static_cast<size_t>(width) * 4u;
    std::vector<u8> filtered;
    filtered.reserve(static_cast<size_t>(height) * (row_bytes + 1u));
    for (int y = 0; y < height; ++y) {
        filtered.push_back(0u); // PNG filter type: None.
        const size_t row_base = static_cast<size_t>(y) * static_cast<size_t>(width);
        for (int x = 0; x < width; ++x) {
            const u32 px = rgba[row_base + static_cast<size_t>(x)];
            filtered.push_back(static_cast<u8>(px & 0xFFu));
            filtered.push_back(static_cast<u8>((px >> 8) & 0xFFu));
            filtered.push_back(static_cast<u8>((px >> 16) & 0xFFu));
            filtered.push_back(static_cast<u8>((px >> 24) & 0xFFu));
        }
    }

    // Minimal zlib stream using uncompressed DEFLATE blocks.
    std::vector<u8> idat;
    idat.reserve(filtered.size() + 6u +
        ((filtered.size() / 65535u) + 1u) * 5u);
    idat.push_back(0x78u);
    idat.push_back(0x01u); // Low compression/fastest profile header.

    size_t offset = 0;
    size_t remaining = filtered.size();
    while (remaining > 0) {
        const u16 block_len =
            static_cast<u16>(std::min<size_t>(remaining, 65535u));
        const bool is_final = (remaining <= 65535u);
        idat.push_back(static_cast<u8>(is_final ? 1u : 0u)); // BFINAL + BTYPE=00
        idat.push_back(static_cast<u8>(block_len & 0xFFu));
        idat.push_back(static_cast<u8>((block_len >> 8) & 0xFFu));
        const u16 nlen = static_cast<u16>(~block_len);
        idat.push_back(static_cast<u8>(nlen & 0xFFu));
        idat.push_back(static_cast<u8>((nlen >> 8) & 0xFFu));
        const u8* block_data = filtered.data() + offset;
        idat.insert(idat.end(), block_data, block_data + block_len);
        offset += block_len;
        remaining -= block_len;
    }
    append_be32(idat, adler32_zlib(filtered.data(), filtered.size()));

    out_png.clear();
    out_png.reserve(idat.size() + 128u);
    static constexpr u8 kPngSignature[8] = {
        0x89u, 0x50u, 0x4Eu, 0x47u, 0x0Du, 0x0Au, 0x1Au, 0x0Au
    };
    out_png.insert(out_png.end(), kPngSignature, kPngSignature + 8);

    std::vector<u8> ihdr;
    ihdr.reserve(13u);
    append_be32(ihdr, static_cast<u32>(width));
    append_be32(ihdr, static_cast<u32>(height));
    ihdr.push_back(8u); // bit depth
    ihdr.push_back(6u); // color type RGBA
    ihdr.push_back(0u); // compression method
    ihdr.push_back(0u); // filter method
    ihdr.push_back(0u); // interlace method
    append_png_chunk(out_png, "IHDR", ihdr);
    append_png_chunk(out_png, "IDAT", idat);
    append_png_chunk(out_png, "IEND", {});
    return true;
}
