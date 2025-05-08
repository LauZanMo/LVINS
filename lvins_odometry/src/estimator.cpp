#include "lvins_odometry/estimator.h"

namespace lvins {

Estimator::Estimator(const YAML::Node &config, DrawerBase::uPtr drawer) : drawer_(std::move(drawer)) {
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
    const auto noise_params = std::make_shared<NoiseParameters>(config["noise_params"]);
    LVINS_INFO("Printing noise parameters:\n{}", *noise_params);

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

    // 加载点云预处理器
    preprocessor_ = std::make_unique<Preprocessor>(config["preprocessor"]);
    LVINS_INFO("Printing preprocessor parameters:\n{}", *preprocessor_);

    // 初始化ESKF
    std::vector<SE3f> T_bs;
    if (lidar_rig_)
        T_bs.insert(T_bs.end(), lidar_rig_->Tbs().begin(), lidar_rig_->Tbs().end());
    if (camera_rig_)
        T_bs.insert(T_bs.end(), camera_rig_->Tbs().begin(), camera_rig_->Tbs().end());
    eskf_ = std::make_unique<ESKF>(config["eskf"], noise_params, g_w, T_bs);
    LVINS_INFO("Printing ESKF parameters:\n{}", *eskf_);
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

    // 清空缓冲区
    lidar_frame_bundle_buffer_.clear();

    LVINS_INFO("Estimator reset!");
}

void Estimator::addImu(const Imu &imu) {
    // 若加速度计输出单位为g，则转换为m/s2
    Imu input(imu);
    if (acc_in_g_) {
        input.acc *= gravity_mag_;
    }

    LVINS_INFO("Get IMU data: {}.", imu);
}

void Estimator::addPointClouds(int64_t timestamp, const std::vector<RawPointCloud::Ptr> &raw_point_clouds) {
    LVINS_CHECK(raw_point_clouds.size() == lidar_rig_->size(),
                "The number of raw point clouds should match the number of lidars!");

    // 预处理点云并生成帧
    std::vector<LidarFrame::sPtr> lidar_frames(raw_point_clouds.size());
    size_t icp_points = 0;
    LVINS_START_TIMER(lidar_preprocess_timer_);
    for (size_t i = 0; i < raw_point_clouds.size(); ++i) {
        auto raw_point_cloud = preprocessor_->process(raw_point_clouds[i]);
        lidar_frames[i] = std::make_shared<LidarFrame>(timestamp, lidar_rig_->lidar(i), std::move(raw_point_cloud));
        // TODO: 根据IMU数据进行点云矫正
        icp_points += lidar_frames[i]->rawPointCloud()->size();
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
