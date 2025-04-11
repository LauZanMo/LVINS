#pragma once

#include "lvins_common/logger.h"

#include <chrono>

#if LVINS_LOG_LEVEL <= LVINS_LOG_LEVEL_TRACE
#define LVINS_DECLARE_TIMER(name) lvins::Timer name;
#define LVINS_START_TIMER(name) name.restart()
#define LVINS_STOP_TIMER(name) name.stop()
#else
#define LVINS_DECLARE_TIMER(name)
#define LVINS_START_TIMER(name) (void) 0
#define LVINS_STOP_TIMER(name) (void) 0
#endif

#define LVINS_PRINT_TIMER_S(name, timer) LVINS_TRACE("{} period: {:.6f}s", name, timer.costInSec())
#define LVINS_PRINT_TIMER_MS(name, timer) LVINS_TRACE("{} period: {:.3f}ms", name, timer.costInMsec())
#define LVINS_PRINT_TIMER_NS(name, timer) LVINS_TRACE("{} period: {}ns", name, LVINS_GROUP_DIGITS(timer.costInNsec()))

namespace lvins {

/**
 * @brief 计时器类
 * @details 该类用于计时，支持秒/毫秒计时与当前时间获取
 * @warning 该类线程不安全
 */
class Timer {
public:
    using Clock = std::chrono::steady_clock;

    /**
     * @brief 构造函数
     * @details 构造后会自动开始计时
     */
    Timer();

    /**
     * @brief 重新开始计时
     */
    void restart();

    /**
     * @brief 停止计时
     * @details 该函数会在costInSec和costInNsec中调用
     */
    void stop();

    /**
     * @brief 停止计时并获取计时时间（秒）
     * @return 计时时间（秒）
     */
    double costInSec();

    /**
     * @brief 停止计时并获取计时时间（毫秒）
     * @return 计时时间（毫秒）
     */
    double costInMsec();

    /**
     * @brief 停止计时并获取计时时间（纳秒）
     * @return 计时时间（纳秒）
     */
    int64_t costInNsec();

    /**
     * @brief 获取当前时间（年-月-日 时:分:秒）
     * @return 当前时间字符串
     */
    static std::string currentTime();

private:
    Clock::time_point start_;
    Clock::duration duration_{};
    bool stop_{false};
};

} // namespace lvins
