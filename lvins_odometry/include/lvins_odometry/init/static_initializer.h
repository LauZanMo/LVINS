#pragma once

#include "lvins_odometry/init/initializer_base.h"

namespace lvins {

/**
 * @brief 静态初始化器类
 * @details 静态初始化器实现逻辑：<br/>
 *          1. 保持静止至达到初始化时长<br/>
 *          2. 通过静止时IMU的数据计算姿态和零偏，位置和速度置零，由此得到初始导航状态<br/>
 *          3. 根据初始导航状态构建帧束容器和导航状态容器
 */
class StaticInitializer final : public InitializerBase {
public:
    /**
     * @brief 构造函数
     * @param parameters 输入参数，顺序：初始化周期、陀螺仪零速阈值、加速度零速阈值
     * @param g_w 世界坐标系下的重力向量
     */
    StaticInitializer(const VecXf &parameters, Vec3f g_w);

    /**
     * @brief 添加IMU数据
     * @param imu IMU数据
     */
    void addImu(const Imu &imu) override;

    /**
     * @brief 添加雷达帧束
     * @param bundle 雷达帧束
     */
    void addLidarFrameBundle(const LidarFrameBundle::sPtr &bundle) override;

    /**
     * @brief 尝试初始化
     * @return 是否初始化成功
     */
    [[nodiscard]] bool tryInitialize() override;

    /**
     * @brief 获取IMU数据容器
     * @return IMU数据容器
     * @warning 该方法在初始化完成后才可调用
     */
    [[nodiscard]] const Imus &imus() const override;

    /**
     * @brief 获取雷达帧束容器
     * @return 雷达帧束容器
     * @warning 该方法在初始化完成后才可调用
     */
    [[nodiscard]] const std::vector<LidarFrameBundle::sPtr> &lidarFrameBundles() const override;

    /**
     * @brief 获取导航状态容器
     * @return 导航状态容器
     * @warning 该方法在初始化完成后才可调用
     */
    [[nodiscard]] const NavStates &navStates() const override;

    /**
     * @brief 重置初始化器
     */
    void reset() override;

    /**
     * @brief 获取初始化器类型
     * @return 初始化器类型
     */
    [[nodiscard]] std::string type() const override;

    /**
     * @brief 获取初始化器参数
     * @return 初始化器参数
     */
    [[nodiscard]] VecXf parameters() const override;

    /**
     * @brief 打印初始化器参数
     * @return 初始化器参数
     */
    [[nodiscard]] std::string print() const override;

private:
    Imus imus_;                                               ///< 用于初始化的IMU数据容器
    std::vector<LidarFrameBundle::sPtr> lidar_frame_bundles_; ///< 用于初始化的雷达帧束容器
    NavStates nav_states_;                                    ///< 用于初始化的导航状态容器

    Float zero_gyr_thresh_; ///< 陀螺仪零速阈值
    Float zero_acc_thresh_; ///< 加速度零速阈值
    int64_t init_period_;   ///< 初始化时长（ns）
};

} // namespace lvins
