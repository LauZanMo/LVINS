#pragma once

#include "lvins_odometry/init/initializer_base.h"

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>

namespace lvins {

/**
 * @brief 动态初始化器类
 * @details 动态初始化器实现逻辑：<br/>
 *          1. 对相邻帧束间的IMU进行预积分，加入缓冲区<br/>
 *          2. 对非首个帧束进行点云配准，计算增量位姿，加入缓冲区<br/>
 *          3. 等待缓冲区满<br/>
 *          4. 求解陀螺仪零偏，再使用陀螺仪零偏重新进行预积分<br/>
 *          5. 求解缓冲区中首个帧束的重力向量和所有帧束相对于自身的速度<br/>
 *          6. 以首个帧束位置为原点，利用求解所得构建缓冲区中所有帧束状态，完成初始化
 * @note 主要参考VINS-Fusion实现：https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/vins_estimator/src/initial/initial_aligment.cpp
 */
class DynamicInitializer final : public InitializerBase {
public:
    /**
     * @brief 构造函数
     * @param parameters 输入参数，顺序：缓冲区阈值
     * @param noise_params 噪声参数
     * @param aligner 点云配准器
     * @param g_w 世界坐标系下的重力向量
     */
    DynamicInitializer(const VecXf &parameters, const NoiseParameters &noise_params, const PointCloudAligner &aligner,
                       Vec3f g_w);

    /**
     * @brief 尝试初始化
     * @note 初始化成功会改变帧束导航状态
     * @param lidar_frame_bundle 新的雷达帧束
     * @param imus 新的IMU数据
     * @return 是否初始化成功
     */
    [[nodiscard]] bool tryInitialize(const LidarFrameBundle::Ptr &lidar_frame_bundle, const Imus &imus) override;

    /**
     * @brief 获取雷达帧束容器
     * @return 雷达帧束容器
     * @warning 该方法在初始化完成后才可调用
     */
    [[nodiscard]] const std::vector<LidarFrameBundle::Ptr> &lidarFrameBundles() const override;

    /**
     * @brief 获取导航状态
     * @return 导航状态
     * @warning 该方法在初始化完成后才可调用
     */
    [[nodiscard]] const NavState &navState() const override;

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
    using ImuParams = gtsam::PreintegratedCombinedMeasurements::Params;

    /**
     * @brief 求解陀螺仪零偏
     * @details 主要思路：利用相邻雷达帧束的预积分和点云配准计算的增量位姿，构建关于陀螺仪零偏的线性求解问题，并使用Cholosky分解求解
     * @param preintegrated 预积分集合
     * @param imus IMU集合
     * @param T_delta 增量位姿集合
     * @return 求解所得陀螺仪零偏
     * @warning 求解后会使用所得陀螺仪零偏对预积分缓冲区进行重积分
     */
    static Vec3f solveGyroscopeBias(std::deque<gtsam::PreintegratedImuMeasurements> &preintegrated,
                                    const std::deque<Imus> &imus, const std::deque<SE3d> &T_delta);

    /**
     * @brief 运动情况下求解缓冲区中首个帧束的重力向量和所有帧束相对于自身的速度
     * @details 主要思路：<br/>
     *          1. 利用相邻雷达帧束的预积分和点云配准计算的增量位姿，构建关于重力向量和速度的线性求解问题，并使用Cholosky分解求解<br/>
     *          2. 对重力向量进行优化，替代重力向量（三维）为重力优化向量（二维），利用模长的约束重新构建线性求解问题并求解。
     * @param preintegrated 预积分集合
     * @param T_delta 增量位姿集合
     * @param vels_b 每个帧束相对于自身的速度集合
     * @param g_b0 首个帧束的重力向量
     * @return 是否求解成功
     */
    [[nodiscard]] bool linearAlign(const std::deque<gtsam::PreintegratedImuMeasurements> &preintegrated,
                                   const std::deque<SE3d> &T_delta, std::vector<Vec3f> &vels_b, Vec3f &g_b0) const;

    /**
     * @brief 优化重力向量
     * @details 该函数为linearAlign函数的子函数，用于优化重力向量
     * @param preintegrated 预积分集合
     * @param T_delta 增量位姿集合
     * @param x 问题的解，（帧束速度+重力向量）
     * @note 问题的解中重力向量为三维，在向量的最末尾
     */
    void refineGravity(const std::deque<gtsam::PreintegratedImuMeasurements> &preintegrated,
                       const std::deque<SE3d> &T_delta, VecXd &x) const;

    /**
     * @brief 获取重力向量的球面切向标准正交基
     * @param g0 重力向量
     * @return 重力向量的球面切向标准正交基
     */
    static Mat32d getTangentBasis(const Vec3d &g0);

    std::deque<gtsam::PreintegratedImuMeasurements> preintegrated_buffer_; ///< 用于初始化的预积分缓冲区
    std::deque<Imus> imu_buffer_;                                          ///< 用于初始化的IMU数据缓冲区
    std::deque<LidarFrameBundle::Ptr> lidar_frame_bundle_buffer_;          ///< 用于初始化的帧束缓冲区
    std::deque<SE3d> T_delta_buffer_;                                      ///< 用于初始化的增量位姿缓冲区
    LidarFrameBundle::Ptr last_lidar_frame_bundle_;                        ///< 上一帧束

    std::vector<LidarFrameBundle::Ptr> lidar_frame_bundles_; ///< 初始化后的帧束集合
    NavState nav_state_;                                     ///< 初始化后的导航状态

    size_t buffer_size_;                      ///< 缓冲区阈值，超过则尝试初始化
    boost::shared_ptr<ImuParams> imu_params_; ///< IMU预积分参数
};

} // namespace lvins
