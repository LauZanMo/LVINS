#include "lvins_common/time/time_wheel_scheduler.h"
#include "lvins_common/time/timer.h"

namespace lvins {

TimeWheelScheduler::TimeWheelScheduler(uint32_t step_ms) : step_ms_(step_ms) {
    LVINS_CHECK(step_ms_ > 0 && step_ms_ < 1000, "Step should be in range [0, 1000]!");
    appendWheel(24, 60 * 60 * 1000);        ///< 小时级时间轮
    appendWheel(60, 60 * 1000);             ///< 分钟级时间轮
    appendWheel(60, 1000);                  ///< 秒级时间轮
    appendWheel(1000 / step_ms_, step_ms_); ///< 毫秒级时间轮
}

TimeWheelScheduler::~TimeWheelScheduler() {
    stop();
}

uint64_t TimeWheelScheduler::addOneShotTask(int64_t delay, const TimeTask::Callback &callback) {
    auto expire_time = Timer::currentTimeInMsec() + delay;
    auto id          = next_id_++;
    { // 添加任务到时间轮
        std::lock_guard lock(wheel_mutex_);
        greatestWheel()->addTimeTask(std::make_shared<TimeTask>(id, expire_time, 0, callback));
    }
    return id;
}

uint64_t TimeWheelScheduler::addPeriodicTask(int64_t interval, const TimeTask::Callback &callback) {
    auto expire_time = Timer::currentTimeInMsec() + interval;
    auto id          = next_id_++;
    { // 添加任务到时间轮
        std::lock_guard lock(wheel_mutex_);
        greatestWheel()->addTimeTask(std::make_shared<TimeTask>(id, expire_time, interval, callback));
    }
    return id;
}

void TimeWheelScheduler::cancelTask(uint64_t id) {
    std::lock_guard lock(wheel_mutex_);
    cancel_task_ids_.insert(id);
}

void TimeWheelScheduler::start() {
    if (running_.exchange(true)) {
        return;
    }

    thread_ = std::thread(&TimeWheelScheduler::run, this);
}

void TimeWheelScheduler::stop() {
    if (!running_.exchange(false)) {
        return;
    }

    { // 唤醒线程
        std::lock_guard lock(sleep_mutex_);
        sleep_cv_.notify_one();
    }

    thread_.join();
}

void TimeWheelScheduler::run() {
    while (true) {
        { // 等待下一个时间片
            std::unique_lock lock(sleep_mutex_);
            sleep_cv_.wait_for(lock, std::chrono::milliseconds(step_ms_));
        }

        // 不在运行状态则退出
        if (!running_) {
            break;
        }

        { // 运行时间轮
            std::lock_guard lock(wheel_mutex_);
            leastWheel()->increase();
            auto slot = leastWheel()->getAndClearCurrentSlot();
            for (const auto &task: slot) {
                // 如果任务ID在取消列表中，则跳过该任务
                if (cancel_task_ids_.find(task->id()) != cancel_task_ids_.end()) {
                    continue;
                }

                // 执行任务
                task->run();

                // 如果任务是周期性任务，则更新过期时间并添加到时间轮
                if (task->isPeriodic()) {
                    task->updateExpireTime();
                    greatestWheel()->addTimeTask(task);
                }
            }
        }
    }
}

void TimeWheelScheduler::appendWheel(uint32_t scales, uint32_t scale_unit) {
    const auto wheel = std::make_shared<TimeWheel>(scales, scale_unit);

    // 时间轮链表为空则直接加入
    if (wheels_.empty()) {
        wheels_.push_back(wheel);
        return;
    }

    // 时间轮链表不为空则还需要设置前后指针
    const auto &last_wheel = wheels_.back();
    last_wheel->setNextWheel(wheel.get());
    wheel->setLastWheel(last_wheel.get());
    wheels_.push_back(wheel);
}

TimeWheel::Ptr &TimeWheelScheduler::greatestWheel() {
    return wheels_.front();
}

TimeWheel::Ptr &TimeWheelScheduler::leastWheel() {
    return wheels_.back();
}

} // namespace lvins
