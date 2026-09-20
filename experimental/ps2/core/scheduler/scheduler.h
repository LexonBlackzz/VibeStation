#pragma once

#include "common/types.h"

#include <functional>
#include <queue>
#include <unordered_set>
#include <vector>

namespace ps2 {

enum class EventType {
    EeTimer,
    EeDmac,
    Vif0,
    Vif1,
    Vu0,
    Vu1,
    Gif,
    Gs,
    Iop,
    Sif,
    Spu2,
};

class Scheduler {
public:
    using Tick = u64;
    using EventId = u64;

    struct Event {
        Tick time = 0;
        EventType type = EventType::EeTimer;
        EventId id = 0;
    };

    using Handler = std::function<void(const Event&)>;

    void reset();

    [[nodiscard]] Tick now() const { return now_; }
    [[nodiscard]] bool empty() const { return events_.empty(); }

    EventId schedule(EventType type, Tick delay);
    void cancel(EventId id);
    void run_until(Tick target, const Handler& handler);

private:
    struct QueuedEvent {
        Event event{};
        u64 sequence = 0;
    };

    struct Earlier {
        bool operator()(const QueuedEvent& lhs, const QueuedEvent& rhs) const {
            if (lhs.event.time != rhs.event.time) {
                return lhs.event.time > rhs.event.time;
            }
            return lhs.sequence > rhs.sequence;
        }
    };

    Tick now_ = 0;
    EventId next_id_ = 1;
    u64 next_sequence_ = 0;
    std::priority_queue<QueuedEvent, std::vector<QueuedEvent>, Earlier> events_;
    std::unordered_set<EventId> cancelled_;
};

} // namespace ps2
