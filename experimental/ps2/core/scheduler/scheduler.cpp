#include "core/scheduler/scheduler.h"

#include <limits>
#include <stdexcept>

namespace ps2 {

void Scheduler::reset() {
    now_ = 0;
    next_id_ = 1;
    next_sequence_ = 0;
    events_ = {};
    cancelled_.clear();
}

Scheduler::EventId Scheduler::schedule(EventType type, Tick delay) {
    if (delay > std::numeric_limits<Tick>::max() - now_) {
        throw std::overflow_error("PS2 scheduler tick overflow");
    }

    const EventId id = next_id_++;
    events_.push(QueuedEvent{
        Event{now_ + delay, type, id},
        next_sequence_++,
    });
    return id;
}

void Scheduler::cancel(EventId id) {
    if (id != 0) {
        cancelled_.insert(id);
    }
}

void Scheduler::run_until(Tick target, const Handler& handler) {
    if (target < now_) {
        throw std::invalid_argument("PS2 scheduler cannot run backwards");
    }

    while (!events_.empty() && events_.top().event.time <= target) {
        const QueuedEvent queued = events_.top();
        events_.pop();

        if (cancelled_.erase(queued.event.id) != 0) {
            continue;
        }

        now_ = queued.event.time;
        if (handler) {
            handler(queued.event);
        }
    }

    now_ = target;
}

} // namespace ps2
