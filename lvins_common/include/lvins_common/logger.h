#pragma once

#include "lvins_common/string_helper.h"
#include "lvins_config/setup.h"

#include <Eigen/Core>
#include <spdlog/async_logger.h>
#include <spdlog/spdlog.h>

#define LVINS_TRACE(...) SPDLOG_LOGGER_TRACE(lvins::Logger::async_logger, __VA_ARGS__)
#define LVINS_DEBUG(...) SPDLOG_LOGGER_DEBUG(lvins::Logger::async_logger, __VA_ARGS__)
#define LVINS_INFO(...) SPDLOG_LOGGER_INFO(lvins::Logger::async_logger, __VA_ARGS__)
#define LVINS_WARN(...) SPDLOG_LOGGER_WARN(lvins::Logger::async_logger, __VA_ARGS__)
#define LVINS_ERROR(...) SPDLOG_LOGGER_ERROR(lvins::Logger::async_logger, __VA_ARGS__)
#define LVINS_FATAL(...)                                                                                               \
    SPDLOG_LOGGER_CRITICAL(lvins::Logger::sync_logger, __VA_ARGS__);                                                   \
    std::abort()
#if LVINS_LOG_LEVEL < LVINS_LOG_LEVEL_INFO
#define LVINS_CHECK(condition, ...)                                                                                    \
    do {                                                                                                               \
        if (!(condition)) {                                                                                            \
            LVINS_FATAL("Check failed: " __VA_ARGS__);                                                                 \
        }                                                                                                              \
    } while (0)
#else
#define LVINS_CHECK(condition, ...) (void) 0
#endif

namespace lvins {

/**
 * @brief Logger类，用于记录系统运行日志
 * @warning 需要在程序开始和结束时调用initialize和shutdown，这两个函数线程不安全!
 */
class Logger {
public:
    /**
     * @brief 初始化日志系统
     * @param log_to_screen 是否输出到屏幕
     * @param file_path 日志文件路径
     * @param program_name 程序名，日志文件名将以此为前缀
     */
    static void initialize(bool log_to_screen, const std::string &file_path, const std::string &program_name);

    /**
     * @brief 关闭日志系统
     */
    static void shutdown();

    /**
     * @brief Eigen矩阵输出格式
     * @note 请使用LVINS_MATRIX_FMT宏输出矩阵
     */
    static const Eigen::IOFormat matrix_fmt;

    /**
     * @brief 需要使用的logger
     * @details critical信息使用同步logger，其余使用异步logger
     */
    static std::shared_ptr<spdlog::async_logger> async_logger;
    static std::shared_ptr<spdlog::logger> sync_logger;
};

} // namespace lvins

/// @note 若输出矩阵出错，可能时fmt版本过高，请参考: https://github.com/gabime/spdlog/issues/2746
template<typename MatrixType>
auto LVINS_MATRIX_FMT(const MatrixType &mat) {
    return mat.format(lvins::Logger::matrix_fmt);
}

template<typename VectorType>
auto LVINS_VECTOR_FMT(const VectorType &vec) {
    return vec.transpose().format(lvins::Logger::matrix_fmt);
}
