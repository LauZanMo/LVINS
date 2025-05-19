#pragma once

#include "lvins_common/time/time_task.h"

#include <list>

namespace lvins {

/**
 * @brief 时间轮类
 */
class TimeWheel {
public:
    using Ptr = std::shared_ptr<TimeWheel>;

    /**
     * @brief 构造函数
     * @param scales 时间轮的刻度数
     * @param scale_unit 每个刻度的时间单位（ms）
     */
    TimeWheel(uint32_t scales, uint32_t scale_unit);

    /**
     * @brief 设置下一级时间轮
     * @param next 下一级时间轮
     */
    void setNextWheel(TimeWheel *next);

    /**
     * @brief 设置上一级时间轮
     * @param last 上一级时间轮
     */
    void setLastWheel(TimeWheel *last);

    /**
     * @brief 添加时间任务
     * @param task 时间任务
     */
    void addTimeTask(const TimeTask::Ptr &task);

    /**
     * @brief 获取当前槽对应的时间（ms）
     * @return 当前槽对应的时间（ms）
     */
    [[nodiscard]] int64_t currentTime() const;

    /**
     * @brief 增加当前时间轮的槽索引
     */
    void increase();

    /**
     * @brief 获取当前槽的时间任务并清空当前槽
     * @return 当前槽的时间任务列表
     */
    std::list<TimeTask::Ptr> getAndClearCurrentSlot();

    /**
     * @brief 获取当前时间轮的刻度数
     * @return 当前时间轮的刻度数
     */
    [[nodiscard]] uint32_t scales() const;

    /**
     * @brief 获取当前时间轮的刻度单位（ms）
     * @return 当前时间轮的刻度单位（ms）
     */
    [[nodiscard]] uint32_t scaleUnit() const;

private:
    uint32_t current_index_{0}; ///< 当前时间轮槽索引

    std::vector<std::list<TimeTask::Ptr>> slots_; ///< 时间轮槽，刻度与槽一一对应，槽中存储时间任务

    TimeWheel *next_wheel_{nullptr}; ///< 下一级时间轮（更小尺度）
    TimeWheel *last_wheel_{nullptr}; ///< 上一级时间轮（更大尺度）

    uint32_t scales_;     ///< 时间轮的刻度数
    uint32_t scale_unit_; ///< 每个刻度的时间单位（ms）
};

} // namespace lvins
