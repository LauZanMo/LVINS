#pragma once

#include "lvins_camera/camera_rig.h"
#include "lvins_common/noise_parameters.h"
#include "lvins_common/sensor/imu.h"
#include "lvins_common/time/time_wheel_scheduler.h"
#include "lvins_common/time/timer.h"
#include "lvins_lidar/lidar_rig.h"
#include "lvins_odometry/base/lidar_frame_bundle.h"
#include "lvins_odometry/drawer_base.h"
#include "lvins_odometry/init/initializer_base.h"
#include "lvins_odometry/preprocessor.h"

namespace lvins {

/**
 * @brief 估计器状态枚举类
 */
enum class EstimatorStatus {
    INITIALIZING = 0, ///< 初始化
    ESTIMATING   = 1, ///< 估计中
};

/**
 * @brief 估计器类
 * @details 估计器类是里程计系统的核心，负责接收点云、图像和IMU数据，进行状态估计，并将关键帧束输入后端优化
 */
class Estimator {
public:
    using Ptr = std::unique_ptr<Estimator>;

    /**
     * @brief 构造函数
     * @param config YAML配置节点
     * @param drawer 绘制器
     */
    Estimator(const YAML::Node &config, DrawerBase::Ptr drawer);

    /**
     * @brief 析构函数
     */
    ~Estimator();

    /**
     * @brief 重置估计器
     */
    void reset();

    /**
     * @brief 输入IMU数据
     * @param imu IMU数据
     */
    void addImu(const Imu &imu);

    /**
     * @brief 输入原始点云集合
     * @param timestamp 时间戳（ns）
     * @param raw_point_clouds 同一时间戳下的原始点云集合
     * @note 需要按照lidar_rig_的顺序输入原始点云集合
     */
    void addPointClouds(int64_t timestamp, const std::vector<RawPointCloud::Ptr> &raw_point_clouds);

    /**
     * @brief 输入原始图像集合
     * @param timestamp 时间戳（ns）
     * @param raw_images 同一时间戳下的原始图像集合
     * @note 需要按照camera_rig_的顺序输入原始图像集合
     */
    void addImages(int64_t timestamp, const std::vector<cv::Mat> &raw_images);

private:
    // 系统
    LidarRig::Ptr lidar_rig_;          ///< 激光雷达组
    CameraRig::Ptr camera_rig_;        ///< 相机组
    DrawerBase::Ptr drawer_;           ///< 绘制器
    Preprocessor::Ptr preprocessor_;   ///< 预处理器
    InitializerBase::Ptr initializer_; ///< 初始化器

    EstimatorStatus status_{EstimatorStatus::INITIALIZING}; ///< 估计器状态
    std::atomic<size_t> reset_count_{0};                    ///< 重置次数

    // 缓冲区
    AsyncQueue<LidarFrameBundle::Ptr> lidar_frame_bundle_buffer_; ///< 雷达帧束缓冲区

    // 计时器
    TimeWheelScheduler::Ptr wheel_scheduler_;    ///< 时间轮调度器
    LVINS_DECLARE_TIMER(lidar_preprocess_timer_) ///< 雷达预处理计时器

    // 参数
    bool acc_in_g_;                     ///< 加速度是否以g为单位
    Float gravity_mag_;                 ///< 重力向量模长
    NoiseParameters::Ptr noise_params_; ///< 噪声参数
    size_t cov_estimation_neighbors_;   ///< 点云协方差估计的最近邻数
    size_t min_icp_points_;             ///< 进行ICP的最小点数
    bool estimate_extrinsic_;           ///< 是否估计外参
};

} // namespace lvins
