#include "lvins_common/time/time_task.h"
#include "lvins_common/logger.h"

namespace lvins {

TimeTask::TimeTask(uint64_t id, int64_t expire_time, uint32_t period, const Callback &callback)
    : id_(id), expire_time_(expire_time), period_(period), is_periodic_(period > 0), callback_(callback) {}

uint64_t TimeTask::id() const {
    return id_;
}

int64_t TimeTask::expireTime() const {
    return expire_time_;
}

void TimeTask::updateExpireTime() {
    if (is_periodic_) {
        expire_time_ += period_;
    } else {
        LVINS_WARN("Time task is not periodic, update expire time will not take any effect!");
    }
}

bool TimeTask::isPeriodic() const {
    return is_periodic_;
}

void TimeTask::run() const {
    if (callback_) {
        callback_();
    } else {
        LVINS_WARN("Time task callback is not set!");
    }
}

} // namespace lvins
