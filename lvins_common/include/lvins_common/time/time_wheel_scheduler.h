#pragma once

#include "lvins_common/async/mutex_types.h"
#include "lvins_common/non_copyable.h"
#include "lvins_common/time/time_wheel.h"

#include <atomic>
#include <condition_variable>
#include <thread>
#include <unordered_set>

namespace lvins {

/**
 * @brief 时间轮调度器类
 * @details 时间轮调度器类用于管理时间任务的调度，总共有4个时间轮：由大到小分别是：小时级、分钟级、秒级和毫秒级
 */
class TimeWheelScheduler : public NonCopyable {
public:
    using Ptr = std::shared_ptr<TimeWheelScheduler>;

    /**
     * @brief 构造函数
     * @param step_ms 时间片步长（ms）
     */
    explicit TimeWheelScheduler(uint32_t step_ms = 50);

    /**
     * @brief 析构函数
     */
    ~TimeWheelScheduler();

    /**
     * @brief 添加一次性任务
     * @param delay 延迟时间（ms）
     * @param callback 回调函数
     * @return 任务ID
     */
    uint64_t addOneShotTask(int64_t delay, const TimeTask::Callback &callback);

    /**
     * @brief 添加周期性任务
     * @param interval 周期时间（ms）
     * @param callback 回调函数
     * @return 任务ID
     */
    uint64_t addPeriodicTask(int64_t interval, const TimeTask::Callback &callback);

    /**
     * @brief 取消任务
     * @param id 任务ID
     */
    void cancelTask(uint64_t id);

    /**
     * @brief 启动调度器
     */
    void start();

    /**
     * @brief 停止调度器
     */
    void stop();

private:
    /**
     * @brief 调度器运行主循环
     */
    void run();

    /**
     * @brief 添加时间轮
     * @param scales 时间轮的刻度数
     * @param scale_unit 每个刻度的时间单位（ms）
     */
    void appendWheel(uint32_t scales, uint32_t scale_unit);

    /**
     * @brief 获取当前最大的时间轮
     * @return 当前最大的时间轮
     */
    TimeWheel::Ptr &greatestWheel();

    /**
     * @brief 获取当前最小的时间轮
     * @return 当前最小的时间轮
     */
    TimeWheel::Ptr &leastWheel();

    std::atomic<bool> running_{false};  ///< 线程运行标志位
    mutex_t wheel_mutex_, sleep_mutex_; ///< 互斥锁
    std::condition_variable sleep_cv_;  ///< 条件变量
    std::thread thread_;                ///< 线程对象

    std::vector<TimeWheel::Ptr> wheels_;           ///< 时间轮集合
    std::unordered_set<uint64_t> cancel_task_ids_; ///< 取消的任务ID集合

    std::atomic<uint64_t> next_id_{1}; ///< 下一个任务ID
    uint32_t step_ms_{50};             ///< 时间片步长（ms）
};

} // namespace lvins
