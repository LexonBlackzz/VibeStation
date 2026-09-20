#pragma once

#include "common/types.h"

#include <vector>

namespace ps2 {

class GsVram {
public:
    static constexpr u32 kSize = 4u * 1024u * 1024u;

    GsVram();

    void reset();

    [[nodiscard]] static bool supported_color_psm(u32 psm);
    [[nodiscard]] static bool supported_depth_psm(u32 psm);
    [[nodiscard]] static u32 pixel_address_bytes(
        u32 psm, u32 x, u32 y, u32 bp, u32 bw);
    [[nodiscard]] static u32 depth_address_bytes(
        u32 psm, u32 x, u32 y, u32 bp, u32 bw);

    bool write_pixel(u32 psm, u32 x, u32 y, u32 bp, u32 bw, u32 value);
    [[nodiscard]] u32 read_pixel(
        u32 psm, u32 x, u32 y, u32 bp, u32 bw) const;
    bool write_depth(u32 psm, u32 x, u32 y, u32 bp, u32 bw, u32 value);
    [[nodiscard]] u32 read_depth(
        u32 psm, u32 x, u32 y, u32 bp, u32 bw) const;

    [[nodiscard]] u8 byte_at(u32 offset) const {
        return data_[offset & (kSize - 1u)];
    }
    [[nodiscard]] const std::vector<u8>& data() const { return data_; }

private:
    std::vector<u8> data_;
};

} // namespace ps2
