#pragma once

#include "lvins_common/async/async_queue.h"
#include "lvins_common/non_copyable.h"
#include "lvins_odometry/base/lidar_frame_bundle.h"

#include <atomic>
#include <functional>
#include <thread>

namespace lvins {

/**
 * @brief 绘制器类
 * @details 绘制器类用于可视化里程计系统中的各种数据
 */
class DrawerBase : public NonCopyable {
public:
    using uPtr = std::unique_ptr<DrawerBase>;

    /**
     * @brief 构造函数
     * @details 构造函数会启动一个内部线程，用于执行绘制任务
     */
    DrawerBase();

    /**
     * @brief 析构函数
     */
    virtual ~DrawerBase();

    /**
     * @brief 重置绘制器
     */
    void reset();

    /**
     * @brief 更新雷达帧束
     * @param timestamp 雷达帧束时间戳
     * @param bundle 雷达帧束
     */
    void updateLidarFrameBundle(int64_t timestamp, const LidarFrameBundle::sPtr &bundle);

    /**
     * @brief 更新重置次数
     * @param times 重置次数
     */
    void updateResetTimes(size_t times);

protected:
    /**
     * @brief 绘制雷达帧束
     * @param timestamp 雷达帧束时间戳
     * @param bundle 雷达帧束
     */
    virtual void drawLidarFrameBundle(int64_t timestamp, const LidarFrameBundle::sPtr &bundle) = 0;

    /**
     * @brief 发布重置次数
     * @param times 重置次数
     */
    virtual void publishResetTimes(size_t times) = 0;

private:
    using DrawTask = std::function<void()>;

    /**
     * @brief 绘制线程，不断从缓冲区中取出任务进行绘制
     */
    void drawLoop();

    /**
     * @brief 重置绘制器的内部实现
     */
    void internalReset();

    // 多线程
    std::atomic<bool> running_{false};         ///< 运行标志位
    std::atomic<bool> reset_{false};           ///< 重置标志位
    std::unique_ptr<std::thread> draw_thread_; ///< 绘制线程

    // 缓冲区
    AsyncQueue<DrawTask> draw_task_buffer_; ///< 绘制任务缓冲区
};

} // namespace lvins
