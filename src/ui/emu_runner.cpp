#include "emu_runner.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <SDL.h>

EmuRunner::~EmuRunner() { stop(); }

bool EmuRunner::start(System* system) {
    if (worker_.joinable() || system == nullptr) {
        return false;
    }

    system_ = system;
    stop_requested_.store(false, std::memory_order_release);
    running_.store(false, std::memory_order_release);
    frame_active_.store(false, std::memory_order_release);
    speed_.store(1.0, std::memory_order_release);
    input_mailbox_.store(pack_input(0xFFFFu, 0x80, 0x80, 0x80, 0x80),
        std::memory_order_release);
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        pending_frame_ = {};
        has_pending_frame_ = false;
    }
    {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        latest_snapshot_ = {};
    }
    {
        std::lock_guard<std::mutex> lock(vram_mutex_);
        latest_vram_snapshot_.clear();
        has_latest_vram_snapshot_ = false;
    }
    {
        std::lock_guard<std::mutex> lock(memcard_request_mutex_);
        has_pending_memcard_request_ = false;
        pending_memcard_paths_ = {};
    }
    capture_vram_debug_.store(false, std::memory_order_release);

    worker_ = std::thread(&EmuRunner::worker_main, this);
    return true;
}

void EmuRunner::stop() {
    if (!worker_.joinable()) {
        return;
    }

    running_.store(false, std::memory_order_release);
    stop_requested_.store(true, std::memory_order_release);
    control_cv_.notify_all();
    idle_cv_.notify_all();

    worker_.join();
    system_ = nullptr;

    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        pending_frame_ = {};
        has_pending_frame_ = false;
    }
    {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        latest_snapshot_.running = false;
    }
    {
        std::lock_guard<std::mutex> lock(vram_mutex_);
        latest_vram_snapshot_.clear();
        has_latest_vram_snapshot_ = false;
    }
    {
        std::lock_guard<std::mutex> lock(memcard_request_mutex_);
        has_pending_memcard_request_ = false;
        pending_memcard_paths_ = {};
    }
    capture_vram_debug_.store(false, std::memory_order_release);
}

void EmuRunner::set_running(bool running) {
    running_.store(running, std::memory_order_release);
    if (system_ != nullptr) {
        system_->set_running(running);
    }
    control_cv_.notify_all();
    if (!running) {
        idle_cv_.notify_all();
    }
}

void EmuRunner::wait_until_idle() {
    std::unique_lock<std::mutex> lock(idle_mutex_);
    idle_cv_.wait(lock, [this] {
        return !frame_active_.load(std::memory_order_acquire);
        });
}

void EmuRunner::pause_and_wait_idle() {
    set_running(false);
    wait_until_idle();
}

void EmuRunner::set_speed(double speed) {
    const double clamped = std::max(0.25, std::min(speed, 4.0));
    speed_.store(clamped, std::memory_order_release);
    if (system_ != nullptr) {
        system_->set_audio_output_speed(clamped);
    }
}

void EmuRunner::set_input_state(u16 buttons, u8 lx, u8 ly, u8 rx, u8 ry) {
    input_mailbox_.store(pack_input(buttons, lx, ly, rx, ry),
        std::memory_order_release);
}

void EmuRunner::request_live_disc_insert(const std::string& bin_path,
    const std::string& cue_path) {
    std::lock_guard<std::mutex> lock(disc_request_mutex_);
    pending_disc_bin_path_ = bin_path;
    pending_disc_cue_path_ = cue_path;
    has_pending_disc_request_ = true;
}

void EmuRunner::request_memory_card_paths(
    const std::array<std::string, 2>& slot_paths) {
    std::lock_guard<std::mutex> lock(memcard_request_mutex_);
    pending_memcard_paths_ = slot_paths;
    has_pending_memcard_request_ = true;
}

bool EmuRunner::consume_latest_frame(FrameSnapshot& out_frame) {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!has_pending_frame_) {
        return false;
    }
    out_frame = std::move(pending_frame_);
    pending_frame_ = {};
    has_pending_frame_ = false;
    return true;
}

bool EmuRunner::consume_latest_vram_snapshot(std::vector<u16>& out_vram) {
    std::lock_guard<std::mutex> lock(vram_mutex_);
    if (!has_latest_vram_snapshot_) {
        return false;
    }
    out_vram = latest_vram_snapshot_;
    has_latest_vram_snapshot_ = false;
    return true;
}

EmuRunner::RuntimeSnapshot EmuRunner::runtime_snapshot() const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    return latest_snapshot_;
}

u64 EmuRunner::pack_input(u16 buttons, u8 lx, u8 ly, u8 rx, u8 ry) {
    u64 packed = 0;
    packed |= static_cast<u64>(buttons);
    packed |= static_cast<u64>(lx) << 16;
    packed |= static_cast<u64>(ly) << 24;
    packed |= static_cast<u64>(rx) << 32;
    packed |= static_cast<u64>(ry) << 40;
    return packed;
}

void EmuRunner::unpack_input(u64 packed, u16& buttons, u8& lx, u8& ly, u8& rx,
    u8& ry) {
    buttons = static_cast<u16>(packed & 0xFFFFu);
    lx = static_cast<u8>((packed >> 16) & 0xFFu);
    ly = static_cast<u8>((packed >> 24) & 0xFFu);
    rx = static_cast<u8>((packed >> 32) & 0xFFu);
    ry = static_cast<u8>((packed >> 40) & 0xFFu);
}

void EmuRunner::apply_input_state(System& system) {
    u16 buttons = 0xFFFFu;
    u8 lx = 0x80;
    u8 ly = 0x80;
    u8 rx = 0x80;
    u8 ry = 0x80;
    unpack_input(input_mailbox_.load(std::memory_order_acquire), buttons, lx, ly,
        rx, ry);
    system.sio().set_button_state(buttons);
    system.sio().set_analog_state(lx, ly, rx, ry);
}

bool EmuRunner::should_capture_frame() const {
    if (!g_gpu_fast_mode) {
        return true;
    }
    std::lock_guard<std::mutex> lock(frame_mutex_);
    return !has_pending_frame_;
}

void EmuRunner::publish_frame(FrameSnapshot&& frame,
    const RuntimeSnapshot& snapshot) {
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            pending_frame_ = std::move(frame);
            has_pending_frame_ = true;
        }
        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            latest_snapshot_ = snapshot;
        }
}

void EmuRunner::publish_snapshot(const RuntimeSnapshot& snapshot) {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    latest_snapshot_ = snapshot;
}

void EmuRunner::apply_pending_memory_card_paths() {
    std::array<std::string, 2> slot_paths{};
    {
        std::lock_guard<std::mutex> lock(memcard_request_mutex_);
        if (!has_pending_memcard_request_) {
            return;
        }
        slot_paths = pending_memcard_paths_;
        has_pending_memcard_request_ = false;
    }

    if (system_ == nullptr) {
        return;
    }

    for (u32 slot = 0; slot < slot_paths.size(); ++slot) {
        system_->set_memory_card_slot(slot, slot_paths[slot]);
    }
}

void EmuRunner::apply_pending_disc_insert() {
    std::string bin_path;
    std::string cue_path;
    {
        std::lock_guard<std::mutex> lock(disc_request_mutex_);
        if (!has_pending_disc_request_) {
            return;
        }
        bin_path = pending_disc_bin_path_;
        cue_path = pending_disc_cue_path_;
        has_pending_disc_request_ = false;
        pending_disc_bin_path_.clear();
        pending_disc_cue_path_.clear();
    }

    if (system_ == nullptr) {
        return;
    }

    if (system_->swap_disc_image(bin_path, cue_path)) {
        system_->notify_disc_inserted();
    }
}

void EmuRunner::worker_main() {
    SDL_SetThreadPriority(SDL_THREAD_PRIORITY_HIGH);

    using steady_clock = std::chrono::steady_clock;
    auto next_tick = steady_clock::now();

    auto run_one_frame = [&](bool publish_outputs, bool skip_audio_for_turbo,
                             bool force_capture) {
        frame_active_.store(true, std::memory_order_release);

        apply_pending_memory_card_paths();
        apply_input_state(*system_);
        apply_pending_disc_insert();
        system_->run_frame(false, skip_audio_for_turbo);

        if (!publish_outputs) {
            frame_active_.store(false, std::memory_order_release);
            idle_cv_.notify_all();
            return;
        }

        RuntimeSnapshot snapshot{};
        snapshot.frame_id = static_cast<u64>(system_->boot_diag().frame_counter);
        snapshot.running = running_.load(std::memory_order_acquire);
        snapshot.boot_diag = system_->boot_diag();
        snapshot.profiling = system_->profiling_stats();
        snapshot.core_frame_ms = snapshot.profiling.total_ms;
        snapshot.spu_audio = system_->spu_audio_diag();

        const auto& spu = system_->spu();
        const auto abs16 = [](s16 value) -> int {
            const int v = static_cast<int>(value);
            return (v < 0) ? -v : v;
            };
        for (size_t voice = 0; voice < snapshot.spu_voice_level_l.size(); ++voice) {
            const u32 base = 0x200u + (static_cast<u32>(voice) * 4u);
            const s16 level_l = static_cast<s16>(spu.read16(base + 0u));
            const s16 level_r = static_cast<s16>(spu.read16(base + 2u));
            snapshot.spu_voice_level_l[voice] = level_l;
            snapshot.spu_voice_level_r[voice] = level_r;
            snapshot.spu_voice_active[voice] =
                (abs16(level_l) != 0) || (abs16(level_r) != 0);
        }
        const u32 endx_lo = static_cast<u32>(spu.read16(0x19C));
        const u32 endx_hi = static_cast<u32>(spu.read16(0x19E) & 0x00FFu);
        snapshot.spu_endx_mask = endx_lo | (endx_hi << 16);
        for (u32 slot = 0; slot < snapshot.memory_card_inserted.size(); ++slot) {
            snapshot.memory_card_inserted[slot] =
                system_->memory_card_inserted(slot);
            snapshot.memory_card_dirty[slot] = system_->memory_card_dirty(slot);
            snapshot.memory_card_path[slot] = system_->memory_card_path(slot);
        }

        if (force_capture || should_capture_frame()) {
            FrameSnapshot frame{};
            std::vector<u32> rgba;
            const DisplaySampleInfo sample =
                // UI output should always use the stable present path.
                // include_stats switches to a slower/alternate conversion path
                // that can drop BIOS menu cursor pixels on some scenes.
                system_->gpu().build_display_rgba(rgba, false);

            frame.frame_id = snapshot.frame_id;
            frame.width = std::max(1, sample.width);
            frame.height = std::max(1, sample.height);
            frame.rgba = std::move(rgba);

            publish_frame(std::move(frame), snapshot);
        }
        else {
            publish_snapshot(snapshot);
        }

        if (capture_vram_debug_.load(std::memory_order_acquire)) {
            const size_t pixel_count =
                static_cast<size_t>(psx::VRAM_WIDTH) * psx::VRAM_HEIGHT;
            const u16* vram = system_->gpu().vram();
            std::lock_guard<std::mutex> lock(vram_mutex_);
            if (latest_vram_snapshot_.size() != pixel_count) {
                latest_vram_snapshot_.resize(pixel_count);
            }
            std::memcpy(latest_vram_snapshot_.data(), vram,
                pixel_count * sizeof(u16));
            has_latest_vram_snapshot_ = true;
        }

        frame_active_.store(false, std::memory_order_release);
        idle_cv_.notify_all();
        };

    while (!stop_requested_.load(std::memory_order_acquire)) {
        std::unique_lock<std::mutex> control_lock(control_mutex_);
        control_cv_.wait(control_lock, [this] {
            return stop_requested_.load(std::memory_order_acquire) ||
                running_.load(std::memory_order_acquire);
            });
        control_lock.unlock();

        if (stop_requested_.load(std::memory_order_acquire)) {
            break;
        }
        if (!running_.load(std::memory_order_acquire) || system_ == nullptr) {
            continue;
        }
        next_tick = steady_clock::now();

        while (running_.load(std::memory_order_acquire) &&
            !stop_requested_.load(std::memory_order_acquire)) {
            const double speed = std::max(0.25, speed_.load(std::memory_order_acquire));
            const auto frame_period = std::chrono::duration_cast<steady_clock::duration>(
                std::chrono::duration<double>((1.0 / system_->target_fps()) / speed));
            const auto now = steady_clock::now();
            if (now < next_tick) {
                const auto remain = next_tick - now;
                if (remain > std::chrono::milliseconds(1)) {
                    std::this_thread::sleep_for(remain - std::chrono::microseconds(500));
                }
                else {
                    std::this_thread::yield();
                }
                continue;
            }

            int frames_to_run = 1;
            const double lag_sec =
                std::chrono::duration<double>(now - next_tick).count();
            const double period_sec = std::chrono::duration<double>(frame_period).count();
            if (period_sec > 0.0) {
                const int lag_frames = static_cast<int>(lag_sec / period_sec);
                const int max_catchup_frames =
                    (speed > 1.001)
                    ? std::clamp(static_cast<int>(std::ceil(speed * 2.0)), 2, 8)
                    : 2;
                frames_to_run = std::min(max_catchup_frames, std::max(1, lag_frames + 1));
            }

            for (int i = 0;
                i < frames_to_run && running_.load(std::memory_order_acquire) &&
                !stop_requested_.load(std::memory_order_acquire);
                ++i) {
                run_one_frame(true, false, false);
                next_tick += frame_period;
            }

            const auto after = steady_clock::now();
            if (after > next_tick + frame_period * 4) {
                // Clamp long hitches to avoid long catch-up spirals.
                next_tick = after;
            }
        }
    }

    frame_active_.store(false, std::memory_order_release);
    idle_cv_.notify_all();
}
