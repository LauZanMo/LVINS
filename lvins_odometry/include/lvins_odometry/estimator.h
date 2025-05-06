#pragma once

#include "lvins_camera/camera_rig.h"
#include "lvins_common/sensor/imu.h"
#include "lvins_lidar/lidar_rig.h"
#include "lvins_odometry/drawer_base.h"
#include "lvins_odometry/fusion/eskf.h"

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
    using uPtr = std::unique_ptr<Estimator>;

    /**
     * @brief 构造函数
     * @param config YAML配置节点
     * @param drawer 绘制器
     */
    Estimator(const YAML::Node &config, DrawerBase::uPtr drawer);

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
    LidarRig::sPtr lidar_rig_;   ///< 激光雷达组
    CameraRig::sPtr camera_rig_; ///< 相机组
    DrawerBase::uPtr drawer_;    ///< 绘制器
};

} // namespace lvins
