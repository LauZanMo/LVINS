#pragma once

#include <functional>
#include <memory>

namespace lvins {

/**
 * @brief 时间任务类
 */
class TimeTask {
public:
    using sPtr     = std::shared_ptr<TimeTask>;
    using Callback = std::function<void()>;

    /**
     * @brief 构造函数
     * @param id 任务ID
     * @param expire_time 过期时间（ms）
     * @param period 周期（ms）
     * @param callback 回调函数
     */
    TimeTask(uint64_t id, int64_t expire_time, uint32_t period, const Callback &callback);

    /**
     * @brief 获取任务ID
     * @return 任务ID
     */
    [[nodiscard]] uint64_t id() const;

    /**
     * @brief 获取过期时间（ms）
     * @return 过期时间（ms）
     */
    [[nodiscard]] int64_t expireTime() const;

    /**
     * @brief 更新过期时间
     * @warning 仅在周期性任务中有效
     */
    void updateExpireTime();

    /**
     * @brief 获取是否为周期性任务
     * @return 是否为周期性任务
     */
    [[nodiscard]] bool isPeriodic() const;

    /**
     * @brief 执行任务
     * @details 执行任务的回调函数
     */
    void run() const;

private:
    uint64_t id_;         ///< 任务ID
    int64_t expire_time_; ///< 过期时间（ms）
    uint32_t period_;     ///< 周期（ms）
    bool is_periodic_;    ///< 是否为周期性任务
    Callback callback_;   ///< 回调函数
};

} // namespace lvins
