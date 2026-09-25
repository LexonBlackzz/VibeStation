#pragma once

#include "core/gs/gs_gpu_backend.h"
#include "core/gs/gs_vram.h"

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

struct SDL_Window;
typedef void* SDL_GLContext;

namespace ps2::ui {

class Ps2GlGsBackend final : public GsGpuBackend {
public:
    Ps2GlGsBackend(
        SDL_Window* window,
        SDL_GLContext context);
    ~Ps2GlGsBackend() override;

    [[nodiscard]] bool available() const override {
        return available_.load(std::memory_order_acquire);
    }
    [[nodiscard]] const char* name() const override {
        return "OpenGL 4.3 compute";
    }

    bool submit_sprite(
        const GsVram& vram,
        const GsRasterContext& ctx,
        const GsRasterVertex& a,
        const GsRasterVertex& b,
        s32 top,
        s32 bottom,
        u64 area) override;
    bool synchronize_to_cpu(GsVram& vram) override;
    void invalidate_cpu_source() override;

private:
    enum class JobType {
        Upload,
        Sprite,
        Sync,
        Stop,
    };

    struct SpriteJob {
        GsRasterContext ctx{};
        GsRasterVertex a{};
        GsRasterVertex b{};
        s32 left = 0;
        s32 right = 0;
        s32 top = 0;
        s32 bottom = 0;
        u32 dirty_begin = 0;
        u32 dirty_end = 0;
    };

    struct Job {
        JobType type = JobType::Stop;
        std::vector<u8> upload{};
        u64 cpu_generation = 0;
        SpriteJob sprite{};
        GsVram* sync_vram = nullptr;
        u64 sync_serial = 0;
    };

    bool sprite_supported(
        const GsRasterContext& ctx,
        const GsRasterVertex& a,
        const GsRasterVertex& b,
        s32 top,
        s32 bottom,
        SpriteJob& job) const;
    void worker_main();
    bool initialize_gl();
    void destroy_gl();
    bool execute_sprite(const SpriteJob& job);

    SDL_Window* window_ = nullptr;
    SDL_GLContext context_ = nullptr;
    std::atomic<bool> available_{false};
    std::thread worker_{};

    std::mutex mutex_{};
    std::condition_variable condition_{};
    std::condition_variable ready_condition_{};
    std::condition_variable sync_condition_{};
    std::deque<Job> jobs_{};
    bool init_finished_ = false;
    bool stop_ = false;
    bool gpu_dirty_ = false;
    u64 outstanding_draws_ = 0;
    u32 dirty_begin_ = GsVram::kSize;
    u32 dirty_end_ = 0;
    u64 queued_cpu_generation_ = ~u64{0};
    u64 synced_cpu_generation_ = ~u64{0};
    u64 next_sync_serial_ = 1;
    u64 completed_sync_serial_ = 0;

    unsigned int vram_buffer_ = 0;
    unsigned int program_ = 0;
};

} // namespace ps2::ui
