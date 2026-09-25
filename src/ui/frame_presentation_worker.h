#pragma once

#include "ui/emu_runner.h"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

struct PreparedUiFrame {
    u64 frame_id = 0;
    int width = 0;
    int height = 0;
    std::vector<u32> rgba;
};

class FramePresentationWorker {
public:
    ~FramePresentationWorker();

    bool start(EmuRunner* runner);
    void stop();

    // Latest-only submission. If UI presentation falls behind, stale raw
    // frames are dropped/recycled instead of building latency.
    void submit(
        FrameSnapshot&& frame,
        int target_width,
        int target_height);

    bool consume_latest(PreparedUiFrame& out_frame);

    // Return the previously displayed UI buffer for reuse by the worker.
    void recycle_consumed(PreparedUiFrame&& frame);

private:
    struct Job {
        FrameSnapshot frame;
        int target_width = 0;
        int target_height = 0;
    };

    void worker_main();
    void recycle_raw(FrameSnapshot&& frame);

    EmuRunner* runner_ = nullptr;
    std::thread worker_;
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> started_{false};

    std::mutex input_mutex_;
    std::condition_variable input_cv_;
    Job pending_job_{};
    bool has_pending_job_ = false;

    std::mutex output_mutex_;
    PreparedUiFrame pending_output_{};
    bool has_pending_output_ = false;

    std::mutex recycled_mutex_;
    std::vector<u32> recycled_output_rgba_{};
};
