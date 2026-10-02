#pragma once

#include "common/types.h"

#include <array>
#include <atomic>
#include <mutex>
#include <vector>

namespace ps2 {

class GsPrivileged;
class GsVram;

class GsDisplay {
public:
    void reset();
    void update(const GsPrivileged& regs, const GsVram& vram);

    // Scanout may run on the GS raster worker. Hold this while reading
    // rgba8() or several fields that must belong to the same frame.
    [[nodiscard]] std::unique_lock<std::mutex> lock() const {
        return std::unique_lock<std::mutex>(mutex_);
    }

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
        return visible_.load(std::memory_order_acquire);
    }
    [[nodiscard]] const std::vector<u32>& rgba8() const { return rgba8_; }

private:
    void reset_locked();
    void update_locked(const GsPrivileged& regs, const GsVram& vram);

    mutable std::mutex mutex_;
    std::atomic<bool> visible_{false};
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
