#pragma once

#include "lvins_camera/camera_rig.h"
#include "lvins_common/noise_parameters.h"
#include "lvins_common/sensor/imu.h"
#include "lvins_common/time/time_wheel_scheduler.h"
#include "lvins_common/time/timer.h"
#include "lvins_icp/align/nearest_neighbor_search.h"
#include "lvins_icp/align/point_cloud_aligner.h"
#include "lvins_lidar/lidar_rig.h"
#include "lvins_odometry/base/lidar_frame_bundle.h"
#include "lvins_odometry/drawer_base.h"
#include "lvins_odometry/drift_detector.h"
#include "lvins_odometry/fusion/eskf.h"
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
    /**
     * @brief 估计线程
     */
    void estimateLoop();

    /**
     * @brief 重置估计器的内部实现
     */
    void internalReset();

    /**
     * @brief 同步缓冲区，使缓冲区头部元素时间戳对齐
     * @details 阻塞地同步缓冲区，使缓冲区头部元素时间戳对齐，若系统关闭，则返回失败
     * @return 是否成功同步缓冲区
     */
    bool syncBuffer();

    /**
     * @brief 从缓冲区中获取测量值
     * @details 阻塞地从缓冲区分别获取当前帧束及其时间戳之前的IMU，若系统关闭，则返回失败
     * @return 是否成功获取测量值
     * @note 更新的帧束可在cur_frame_bundle_中获取，IMU可在cur_imus_中获取
     */
    bool getMeasurementFromBuffer();

    /**
     * @brief 从缓冲区中获取指定时间戳之前的IMU
     * @details 阻塞地从缓冲区获取指定时间戳之前的IMU，若系统关闭，则返回失败
     * @param timestamp 时间戳
     * @param imus 时间戳之前的IMU
     * @return 是否成功获取IMU
     */
    bool getImusFromBuffer(int64_t timestamp, Imus &imus);

    // 系统
    LidarRig::Ptr lidar_rig_;                    ///< 激光雷达组
    CameraRig::Ptr camera_rig_;                  ///< 相机组
    DrawerBase::Ptr drawer_;                     ///< 绘制器
    Preprocessor::Ptr preprocessor_;             ///< 预处理器
    PointCloudAligner::Ptr point_cloud_aligner_; ///< 点云配准器
    ESKF::Ptr eskf_;                             ///< 扩展卡尔曼滤波器
    NearestNeighborSearch::Ptr local_mapper_;    ///< 局部地图（通过最近邻搜索器实现）
    InitializerBase::Ptr initializer_;           ///< 初始化器
    DriftDetector::Ptr drift_detector_;          ///< 系统漂移检测器

    EstimatorStatus status_{EstimatorStatus::INITIALIZING}; ///< 估计器状态

    // 多线程
    std::atomic<bool> running_{false};             ///< 运行标志位
    std::atomic<bool> reset_{false};               ///< 重置标志位
    std::atomic<size_t> reset_count_{0};           ///< 重置次数
    std::unique_ptr<std::thread> estimate_thread_; ///< 估计线程

    // 缓冲区
    AsyncQueue<LidarFrameBundle::Ptr> lidar_frame_bundle_buffer_;            ///< 雷达帧束缓冲区
    AsyncQueue<Imu> imu_buffer_;                                             ///< IMU缓冲区
    LidarFrameBundle::Ptr cur_lidar_frame_bundle_, last_lidar_frame_bundle_; ///< 当前帧束和上一帧束
    Imus cur_imus_;                                                          ///< 上一帧束到当前帧束间
    Imus preintegrate_imus_;                                                 ///< 用于预积分的所有的IMU
    Imu last_imu_;                                                           ///< 上一IMU

    // 计时器
    TimeWheelScheduler::Ptr wheel_scheduler_;    ///< 时间轮调度器
    LVINS_DECLARE_TIMER(lidar_preprocess_timer_) ///< 雷达预处理计时器
    LVINS_DECLARE_TIMER(pop_timer_)              ///< 获取测量值计时器
    LVINS_DECLARE_TIMER(deskew_timer_)           ///< 点云矫正计时器
    LVINS_DECLARE_TIMER(update_timer_)           ///< ESKF更新计时器

    // 参数
    bool acc_in_g_;                     ///< 加速度是否以g为单位
    Float gravity_mag_;                 ///< 重力向量模长
    NoiseParameters::Ptr noise_params_; ///< 噪声参数
    size_t cov_estimation_neighbors_;   ///< 点云协方差估计的最近邻数
    size_t min_icp_points_;             ///< 进行ICP的最小点数
    bool use_imu_prior_;                ///< 是否使用IMU先验
    bool estimate_extrinsic_;           ///< 是否估计外参
};

} // namespace lvins
