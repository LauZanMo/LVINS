#include "lvins_odometry/estimator.h"
#include "lvins_icp/preprocess/copy.h"
#include "lvins_icp/preprocess/covariance_estimation.h"

namespace lvins {

Estimator::Estimator(const YAML::Node &config, DrawerBase::Ptr drawer) : drawer_(std::move(drawer)) {
    LVINS_INFO("Starting estimator...");

#if LVINS_LICENSE_CHECK
    LVINS_INFO("Checking SLAM license...");

    if (std::time(nullptr) > LVINS_PRELOAD_UTC || 1 != LVINS_PRELOAD_CPUID) {
        LVINS_FATAL("SLAM license check failed! Please contact sales for more information.");
    } else {
        LVINS_INFO("SLAM license verified!");
    }
#endif

    // 重力相关参数
    acc_in_g_    = YAML::get<bool>(config, "acc_in_g");
    gravity_mag_ = YAML::get<Float>(config, "gravity_mag");
    Vec3f g_w(0.0, 0.0, gravity_mag_);

    // 噪声参数
    noise_params_ = std::make_shared<NoiseParameters>(config["noise_params"]);
    LVINS_INFO("Printing noise parameters:\n{}", *noise_params_);

    // 读取雷达文件路径，并初始化雷达组
    const auto lidar_rig_file = YAML::get<std::string>(config, "lidar_rig_file");
    if (!lidar_rig_file.empty()) {
        lidar_rig_ = LidarRig::loadFromYaml(lidar_rig_file);
        LVINS_INFO("Printing lidar rig parameters:\n{}", *lidar_rig_);
    } else {
        LVINS_INFO("No lidar in this odometry.");
    }

    // 读取相机文件路径，并初始化相机组
    const auto camera_rig_file = YAML::get<std::string>(config, "camera_rig_file");
    if (!camera_rig_file.empty()) {
        camera_rig_ = CameraRig::loadFromYaml(camera_rig_file);
        LVINS_INFO("Printing camera rig parameters:\n{}", *camera_rig_);
    } else {
        LVINS_INFO("No camera in this odometry.");
    }

    // 初始化点云预处理器
    preprocessor_ = std::make_unique<Preprocessor>(config["preprocessor"]);
    LVINS_INFO("Printing preprocessor parameters:\n{}", *preprocessor_);

    // 初始化点云搜索器
    point_cloud_searcher_ = NearestNeighborSearcher::loadFromYaml(config["point_cloud_searcher"]);
    LVINS_INFO("Printing point cloud searcher parameters:\n{}", *point_cloud_searcher_);

    // 初始化ESKF
    std::vector<SE3f> T_bs;
    if (lidar_rig_)
        T_bs.insert(T_bs.end(), lidar_rig_->Tbs().begin(), lidar_rig_->Tbs().end());
    if (camera_rig_)
        T_bs.insert(T_bs.end(), camera_rig_->Tbs().begin(), camera_rig_->Tbs().end());
    eskf_ = std::make_unique<ESKF>(config["eskf"], *noise_params_, g_w, T_bs);
    LVINS_INFO("Printing ESKF parameters:\n{}", *eskf_);

    // 初始化局部建图器
    local_mapper_ = NearestNeighborSearcher::loadFromYaml(config["local_mapper"]);
    LVINS_INFO("Printing local mapper parameters:\n{}", *local_mapper_);

    // 加载初始化器
    initializer_ = InitializerBase::loadFromYaml(config["initializer"], g_w);
    LVINS_INFO("Printing initializer parameters:\n{}", *initializer_);

    // 初始化时间轮调度器
    wheel_scheduler_ = std::make_shared<TimeWheelScheduler>();
    wheel_scheduler_->start();
    wheel_scheduler_->addPeriodicTask(1000, [this] {
        drawer_->updateResetTimes(reset_count_); // 以1s为周期更新重置次数
    });

    // 读取估计器配置
    const auto estimator_config = config["estimator"];
    cov_estimation_neighbors_   = YAML::get<size_t>(estimator_config, "cov_estimation_neighbors");
    min_icp_points_             = YAML::get<size_t>(estimator_config, "min_icp_points");
}

Estimator::~Estimator() {
    LVINS_INFO("Closing estimator...");

    // 打印外参
    if (lidar_rig_)
        LVINS_INFO("Printing optimized lidar rig parameters:\n{}", *lidar_rig_);
    if (camera_rig_)
        LVINS_INFO("Printing optimized camera rig parameters:\n{}", *camera_rig_);

    LVINS_INFO("Estimator closed!");
}

void Estimator::reset() {
    LVINS_INFO("Resetting estimator...");

    // 重置系统
    status_ = EstimatorStatus::INITIALIZING;
    drawer_->reset();
    eskf_->reset();
    local_mapper_->clear();

    // 清空缓冲区
    lidar_frame_bundle_buffer_.clear();

    ++reset_count_;
    LVINS_INFO("Estimator reset!");
}

void Estimator::addImu(const Imu &imu) {
    // 若加速度计输出单位为g，则转换为m/s2
    Imu input(imu);
    if (acc_in_g_) {
        input.acc *= gravity_mag_;
    }

    if (status_ == EstimatorStatus::INITIALIZING) {
        // 初始化器添加IMU数据
        initializer_->addImu(input);

        // 初始化器添加雷达帧束
        LidarFrameBundle::Ptr lidar_frame_bundle;
        while (lidar_frame_bundle_buffer_.tryPop(lidar_frame_bundle)) {
            initializer_->addLidarFrameBundle(lidar_frame_bundle);
        }

        // 尝试初始化
        if (initializer_->tryInitialize()) {
            // 初始化ESKF
            eskf_->initialize(initializer_->navStates(), initializer_->imus());

            // 初始化局部地图并绘制
            for (const auto &bundle: initializer_->lidarFrameBundles()) {
                for (size_t i = 0; i < bundle->size(); ++i) {
                    const auto &frame = bundle->frame(i);
                    local_mapper_->insert(frame.pointCloud(), frame.Twf());
                }
                drawer_->updateLidarFrameBundle(bundle->timestamp(), bundle);
            }

            // 更新状态标志位
            status_ = EstimatorStatus::ESTIMATING;
        }
    } else if (status_ == EstimatorStatus::ESTIMATING) {
        if (eskf_->propagate(input)) {
            // 更新对应updateState时间戳的帧束，并向增量地图嵌入点云
        }

        LidarFrameBundle::Ptr lidar_frame_bundle;
        while (lidar_frame_bundle_buffer_.tryPop(lidar_frame_bundle)) {
            // 构建观测函数，并添加到ESKF
        }
    }
}

void Estimator::addPointClouds(int64_t timestamp, const std::vector<RawPointCloud::Ptr> &raw_point_clouds) {
    LVINS_CHECK(raw_point_clouds.size() == lidar_rig_->size(),
                "The number of raw point clouds should match the number of lidars!");

    // 预处理点云并生成帧
    std::vector<LidarFrame::Ptr> lidar_frames(raw_point_clouds.size());
    size_t icp_points = 0;
    LVINS_START_TIMER(lidar_preprocess_timer_);
    for (size_t i = 0; i < raw_point_clouds.size(); ++i) {
        auto raw_point_cloud = preprocessor_->process(raw_point_clouds[i]);
        lidar_frames[i] = std::make_shared<LidarFrame>(timestamp, lidar_rig_->lidar(i), std::move(raw_point_cloud));
        // TODO: 根据IMU数据进行点云矫正
        copy(lidar_frames[i]->rawPointCloud(), lidar_frames[i]->pointCloud());
        estimateCovariance(lidar_frames[i]->pointCloud(), *point_cloud_searcher_, cov_estimation_neighbors_);
        icp_points += lidar_frames[i]->pointCloud().size();
    }
    LVINS_STOP_TIMER(lidar_preprocess_timer_);
    LVINS_PRINT_TIMER_MS("Lidar preprocess", lidar_preprocess_timer_);

    // 若点云点数太少，则跳过
    if (icp_points < min_icp_points_) {
        LVINS_WARN("Not enough points for registration! Skipping this lidar frame bundle at {}.",
                   LVINS_GROUP_DIGITS(timestamp));
        return;
    }

    // 将帧束加入缓冲区
    lidar_frame_bundle_buffer_.push(std::make_shared<LidarFrameBundle>(lidar_frames));
}

void Estimator::addImages(int64_t /*timestamp*/, const std::vector<cv::Mat> & /*raw_images*/) {
    LVINS_WARN("Image processing is not implemented!");
}

} // namespace lvins
