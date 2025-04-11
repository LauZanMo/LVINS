#pragma once

#include "lvins_common/eigen_types.h"
#include "lvins_common/logger.h"

#include <deque>

namespace lvins {

/**
 * @brief IMU数据类
 * @details 该类用于存储IMU数据，包括时间戳、陀螺仪数据和加速度计数据
 */
struct Imu {
    /**
     * @brief 构造函数
     * @details 构造一个时间戳为-1，值全为0的IMU数据
     */
    Imu();

    /**
     * @brief 构造函数
     * @param timestamp 时间戳（单位：ns）
     * @param gyr 陀螺仪数据
     * @param acc 加速度计数据
     */
    Imu(int64_t timestamp, Vec3f gyr, Vec3f acc);

    /**
     * @brief 默认析构函数
     */
    ~Imu() = default;

    /**
     * @brief 判断IMU数据是否已初始化
     * @return IMU数据是否已初始化
     * @note 通过默认构造函数构造的IMU数据是未初始化的
     */
    explicit operator bool() const;

    /**
     * @brief 对两个IMU数据进行线性插值
     * @param imu0 第一个IMU数据
     * @param imu1 第二个IMU数据
     * @param timestamp 插值时刻的时间戳（单位：ns）
     * @return 插值后的IMU数据
     */
    static Imu interpolate(const Imu &imu0, const Imu &imu1, int64_t timestamp);

    int64_t timestamp; ///< 时间戳（ns）
    Vec3f gyr;         ///< 陀螺仪数据（rad/s）
    Vec3f acc;         ///< 加速度计数据（m/s^2）
};

using Imus      = std::vector<Imu, Eigen::aligned_allocator<Imu>>;
using ImuBuffer = std::deque<Imu, Eigen::aligned_allocator<Imu>>;

} // namespace lvins

/**
 * @brief IMU数据格式化器
 */
template<>
struct LVINS_FORMATTER<lvins::Imu> {
    /**
     * @brief 从文本中解析格式化字符
     * @param ctx 文本
     * @return 格式化字符尾部迭代器
     */
    constexpr auto parse(LVINS_FORMAT_PARSE_CONTEXT &ctx) {
        return ctx.begin();
    }

    /**
     * @brief 格式化
     * @param imu IMU数据
     * @param ctx 输出的格式化文本
     * @return 输出格式化文本的尾部迭代器
     */
    auto format(const lvins::Imu &imu, LVINS_FORMAT_CONTEXT &ctx) const {
        return LVINS_FORMAT_TO(ctx.out(), "timestamp: {}\ngyr: {}\nacc: {}", LVINS_GROUP_DIGITS(imu.timestamp),
                            LVINS_VECTOR_FMT(imu.gyr), LVINS_VECTOR_FMT(imu.acc));
    }
};
