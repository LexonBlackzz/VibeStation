#pragma once

#include "common/types.h"

#include <array>
#include <atomic>
#include <mutex>
#include <vector>

namespace ps2 {

class GsPrivileged;
class GsVram;

// PCRTC scanout. update() may run on the GS raster worker while other threads
// read the result: the scalar accessors are safe from any thread, and callers
// reading rgba8() concurrently with an update must hold lock_image().
class GsDisplay {
public:
    void reset();
    void update(const GsPrivileged& regs, const GsVram& vram);

    [[nodiscard]] bool valid() const { return valid_.load(); }
    [[nodiscard]] u32 width() const { return width_.load(); }
    [[nodiscard]] u32 height() const { return height_.load(); }
    [[nodiscard]] u32 circuit() const { return circuit_.load(); }
    [[nodiscard]] u32 psm() const { return psm_.load(); }
    [[nodiscard]] u64 generation() const { return generation_.load(); }
    [[nodiscard]] u64 nonzero_pixel_count() const {
        return nonzero_pixel_count_.load();
    }
    [[nodiscard]] bool has_visible_pixels() const {
        return valid() && nonzero_pixel_count() != 0;
    }
    [[nodiscard]] const std::vector<u32>& rgba8() const { return rgba8_; }
    [[nodiscard]] std::unique_lock<std::mutex> lock_image() const {
        return std::unique_lock<std::mutex>(image_mutex_);
    }

private:
    std::atomic<bool> valid_{false};
    std::atomic<u32> width_{0};
    std::atomic<u32> height_{0};
    std::atomic<u32> circuit_{0};
    std::atomic<u32> psm_{0};
    std::atomic<u64> generation_{0};
    std::atomic<u64> nonzero_pixel_count_{0};
    // Scanout cache state is only touched by the thread performing updates.
    bool scanout_cache_valid_ = false;
    u64 cached_vram_generation_ = 0;
    std::array<u64, 7> cached_scanout_registers_{};
    mutable std::mutex image_mutex_;
    std::vector<u32> rgba8_;
};

} // namespace ps2
