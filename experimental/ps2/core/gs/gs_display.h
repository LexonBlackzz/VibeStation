#pragma once

#include "common/types.h"

#include <array>
#include <vector>

namespace ps2 {

class GsPrivileged;
class GsVram;

class GsDisplay {
public:
    void reset();
    void update(const GsPrivileged& regs, const GsVram& vram);

    [[nodiscard]] bool valid() const { return valid_; }
    [[nodiscard]] u32 width() const { return width_; }
    [[nodiscard]] u32 height() const { return height_; }
    [[nodiscard]] u32 circuit() const { return circuit_; }
    [[nodiscard]] u32 psm() const { return psm_; }
    [[nodiscard]] u64 generation() const { return generation_; }
    [[nodiscard]] u64 nonzero_pixel_count() const {
        return nonzero_pixel_count_;
    }
    [[nodiscard]] bool has_visible_pixels() const {
        return valid_ && nonzero_pixel_count_ != 0;
    }
    [[nodiscard]] const std::vector<u32>& rgba8() const { return rgba8_; }

private:
    bool valid_ = false;
    u32 width_ = 0;
    u32 height_ = 0;
    u32 circuit_ = 0;
    u32 psm_ = 0;
    u64 generation_ = 0;
    u64 nonzero_pixel_count_ = 0;
    bool scanout_cache_valid_ = false;
    u64 cached_vram_generation_ = 0;
    std::array<u64, 7> cached_scanout_registers_{};
    std::vector<u32> rgba8_;
};

} // namespace ps2
