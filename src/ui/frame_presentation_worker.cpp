#include "ui/frame_presentation_worker.h"

#include "ui/definitive/definitive_shared.h"
#include "ui/screenshot_utils.h"

#include <SDL.h>

#include <algorithm>
#include <utility>

FramePresentationWorker::~FramePresentationWorker() {
    stop();
}

bool FramePresentationWorker::start(EmuRunner* runner) {
    if (started_.load(std::memory_order_acquire)) {
        return true;
    }
    if (runner == nullptr) {
        return false;
    }

    runner_ = runner;
    stop_requested_.store(false, std::memory_order_release);

    try {
        worker_ =
            std::thread(
                &FramePresentationWorker::worker_main,
                this);
    }
    catch (...) {
        runner_ = nullptr;
        return false;
    }

    started_.store(true, std::memory_order_release);
    return true;
}

void FramePresentationWorker::stop() {
    if (!started_.exchange(false, std::memory_order_acq_rel)) {
        return;
    }

    stop_requested_.store(true, std::memory_order_release);
    input_cv_.notify_all();

    if (worker_.joinable()) {
        worker_.join();
    }

    Job pending{};
    {
        std::lock_guard<std::mutex> lock(input_mutex_);
        if (has_pending_job_) {
            pending = std::move(pending_job_);
            pending_job_ = {};
            has_pending_job_ = false;
        }
    }
    if (!pending.frame.rgba.empty()) {
        recycle_raw(std::move(pending.frame));
    }

    {
        std::lock_guard<std::mutex> lock(output_mutex_);
        pending_output_ = {};
        has_pending_output_ = false;
    }
    {
        std::lock_guard<std::mutex> lock(recycled_mutex_);
        recycled_output_rgba_.clear();
    }

    runner_ = nullptr;
}

void FramePresentationWorker::submit(
    FrameSnapshot&& frame,
    int target_width,
    int target_height) {
    if (!started_.load(std::memory_order_acquire) ||
        runner_ == nullptr) {
        recycle_raw(std::move(frame));
        return;
    }

    Job dropped{};
    {
        std::lock_guard<std::mutex> lock(input_mutex_);
        if (has_pending_job_) {
            dropped = std::move(pending_job_);
        }

        pending_job_.frame = std::move(frame);
        pending_job_.target_width =
            std::max(1, target_width);
        pending_job_.target_height =
            std::max(1, target_height);
        has_pending_job_ = true;
    }

    if (!dropped.frame.rgba.empty()) {
        recycle_raw(std::move(dropped.frame));
    }

    input_cv_.notify_one();
}

bool FramePresentationWorker::consume_latest(
    PreparedUiFrame& out_frame) {
    std::lock_guard<std::mutex> lock(output_mutex_);
    if (!has_pending_output_) {
        return false;
    }

    out_frame = std::move(pending_output_);
    pending_output_ = {};
    has_pending_output_ = false;
    return true;
}

void FramePresentationWorker::recycle_consumed(
    PreparedUiFrame&& frame) {
    frame.frame_id = 0;
    frame.width = 0;
    frame.height = 0;

    std::lock_guard<std::mutex> lock(recycled_mutex_);
    if (frame.rgba.capacity() >=
        recycled_output_rgba_.capacity()) {
        recycled_output_rgba_ =
            std::move(frame.rgba);
    }
}

void FramePresentationWorker::recycle_raw(
    FrameSnapshot&& frame) {
    if (runner_ != nullptr) {
        runner_->recycle_consumed_frame(
            std::move(frame));
    }
}

void FramePresentationWorker::worker_main() {
    // Presentation work is deliberately lower priority than emulation and the
    // host audio callback. It may drop visual frames under load, but it must
    // never steal timing budget from game logic or audio production.
    SDL_SetThreadPriority(SDL_THREAD_PRIORITY_LOW);

    while (!stop_requested_.load(std::memory_order_acquire)) {
        Job job{};

        {
            std::unique_lock<std::mutex> lock(input_mutex_);
            input_cv_.wait(
                lock,
                [this] {
                    return stop_requested_.load(
                               std::memory_order_acquire) ||
                        has_pending_job_;
                });

            if (stop_requested_.load(
                    std::memory_order_acquire)) {
                break;
            }

            job = std::move(pending_job_);
            pending_job_ = {};
            has_pending_job_ = false;
        }

        if (job.frame.rgba.empty() ||
            job.frame.width <= 0 ||
            job.frame.height <= 0) {
            recycle_raw(std::move(job.frame));
            continue;
        }

        PreparedUiFrame prepared{};
        prepared.frame_id = job.frame.frame_id;
        prepared.width =
            std::max(1, job.target_width);
        prepared.height =
            std::max(1, job.target_height);

        {
            std::lock_guard<std::mutex> lock(recycled_mutex_);
            if (!recycled_output_rgba_.empty()) {
                prepared.rgba.swap(
                    recycled_output_rgba_);
            }
        }

        // Always prepare a dedicated presentation buffer, even when source and
        // destination dimensions match. That keeps EmuRunner's capture buffer
        // ownership entirely on the emulation side and makes the UI mailbox
        // independent of the game thread.
        resample_rgba_nearest(
            job.frame.rgba,
            job.frame.width,
            job.frame.height,
            prepared.rgba,
            prepared.width,
            prepared.height);

        // Ambilight edge analysis lives on this presentation worker too, so the
        // ImGui/main thread only draws already-prepared state.
        definitive_ui::update_gameplay_ambient(
            prepared.rgba,
            prepared.width,
            prepared.height);

        recycle_raw(std::move(job.frame));

        PreparedUiFrame stale{};
        {
            std::lock_guard<std::mutex> lock(output_mutex_);
            if (has_pending_output_) {
                stale = std::move(pending_output_);
            }

            pending_output_ = std::move(prepared);
            has_pending_output_ = true;
        }

        if (!stale.rgba.empty()) {
            recycle_consumed(std::move(stale));
        }
    }
}
