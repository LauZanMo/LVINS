#include "lvins_common/time/time_wheel.h"
#include "lvins_common/time/timer.h"

namespace lvins {

TimeWheel::TimeWheel(uint32_t scales, uint32_t scale_unit) : slots_(scales), scales_(scales), scale_unit_(scale_unit) {}

void TimeWheel::setNextWheel(TimeWheel *next) {
    next_wheel_ = next;
}

void TimeWheel::setLastWheel(TimeWheel *last) {
    last_wheel_ = last;
}

void TimeWheel::addTimeTask(const TimeTask::sPtr &task) {
    int64_t next_wheel_time = 0;
    if (next_wheel_) {
        next_wheel_time = next_wheel_->currentTime();
    }
    const auto diff = task->expireTime() + next_wheel_time - Timer::currentTimeInMsec();

    // 如果差值大于当前时间轮的刻度单位，则将任务添加当前时间轮
    if (diff >= scale_unit_) {
        const size_t n = (current_index_ + diff / scale_unit_) % scales_;
        slots_[n].push_back(task);
        return;
    }

    // 如果差值小于当前时间轮的刻度单位，则将任务添加到下一级时间轮
    if (next_wheel_) {
        next_wheel_->addTimeTask(task);
        return;
    }

    // 当前时间轮为最后一级时间轮，直接添加到当前时间轮
    slots_[current_index_].push_back(task);
}

int64_t TimeWheel::currentTime() const {
    int64_t time = current_index_ * scale_unit_;
    if (next_wheel_) {
        time += next_wheel_->currentTime();
    }

    return time;
}

void TimeWheel::increase() {
    ++current_index_;
    if (current_index_ < scales_) {
        return;
    }

    // 如果当前时间轮已经过了一个周期，则上一级时间轮也需要增长，增长后上一级时间轮的当前槽的时间任务移动到下一级时间轮
    current_index_ = current_index_ % scales_;
    if (last_wheel_) {
        last_wheel_->increase();
        const std::list<TimeTask::sPtr> slot = std::move(last_wheel_->getAndClearCurrentSlot());
        for (const auto &timer: slot) {
            addTimeTask(timer);
        }
    }
}

std::list<TimeTask::sPtr> TimeWheel::getAndClearCurrentSlot() {
    return std::move(slots_[current_index_]);
}

uint32_t TimeWheel::scales() const {
    return scales_;
}

uint32_t TimeWheel::scaleUnit() const {
    return scale_unit_;
}

} // namespace lvins
