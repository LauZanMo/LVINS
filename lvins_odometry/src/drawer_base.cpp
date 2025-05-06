#include "lvins_odometry/drawer_base.h"
#include "lvins_common/logger.h"

namespace lvins {

DrawerBase::DrawerBase() {
    LVINS_INFO("Starting drawer...");

    // 启动绘制线程
    running_     = true;
    draw_thread_ = std::make_unique<std::thread>(&DrawerBase::drawLoop, this);

    LVINS_INFO("Drawer started!");
}

DrawerBase::~DrawerBase() {
    LVINS_INFO("Closing drawer...");

    // 结束绘制线程
    running_ = false;
    draw_task_buffer_.push(nullptr);
    draw_thread_->join();

    LVINS_INFO("Drawer closed!");
}

void DrawerBase::reset() {
    LVINS_INFO("Resetting drawer...");

    // 使能标志位并发布重置消息
    reset_ = true;
    publishReset();
}

void DrawerBase::updateLidarFrameBundle(int64_t timestamp, const LidarFrameBundle::sPtr &bundle) {
    draw_task_buffer_.push([this, timestamp, bundle]() {
        drawLidarFrameBundle(timestamp, bundle);
    });
}

void DrawerBase::drawLoop() {
    while (true) {
        // 从缓冲区获取任务
        const auto task = draw_task_buffer_.pop();

        // 如果不在运行状态则退出
        if (!running_) {
            break;
        }

        // 执行绘制任务
        task();

        // 检查是否需要重置
        if (reset_) {
            internalReset();
        }
    }
}

void DrawerBase::internalReset() {
    // 清空缓冲区
    draw_task_buffer_.clear();

    // 重置标志位
    reset_ = false;

    LVINS_INFO("Drawer reset!");
}

} // namespace lvins
