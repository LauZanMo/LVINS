#include "lvins_odometry/estimator.h"
#include "lvins_icp/preprocess/copy.h"
#include "lvins_icp/preprocess/covariance_estimation.h"
#include "lvins_odometry/fusion/update_lidar.h"

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
    const Vec3f g_w(0.0, 0.0, gravity_mag_);

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

    // 初始化点云配准器
    point_cloud_aligner_ = std::make_shared<PointCloudAligner>(config["point_cloud_aligner"]);
    LVINS_INFO("Printing point cloud aligner parameters:\n{}", *point_cloud_aligner_);

    // 初始化ESKF
    std::vector<SE3f> T_bs;
    if (lidar_rig_)
        T_bs.insert(T_bs.end(), lidar_rig_->Tbs().begin(), lidar_rig_->Tbs().end());
    if (camera_rig_)
        T_bs.insert(T_bs.end(), camera_rig_->Tbs().begin(), camera_rig_->Tbs().end());
    eskf_ = std::make_unique<ESKF>(config["eskf"], *noise_params_, g_w, T_bs);
    LVINS_INFO("Printing ESKF parameters:\n{}", *eskf_);

    // 初始化局部建图器
    local_mapper_ = std::make_shared<NearestNeighborSearch>(config["local_mapper"]);
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
    use_imu_prior_              = YAML::get<bool>(estimator_config, "use_imu_prior");
    estimate_extrinsic_         = YAML::get<bool>(estimator_config, "estimate_extrinsic");

    // 启动估计线程
    running_         = true;
    estimate_thread_ = std::make_unique<std::thread>(&Estimator::estimateLoop, this);

    LVINS_INFO("Estimator started!");
}

Estimator::~Estimator() {
    LVINS_INFO("Closing estimator...");

    // 打印外参
    if (lidar_rig_)
        LVINS_INFO("Printing optimized lidar rig parameters:\n{}", *lidar_rig_);
    if (camera_rig_)
        LVINS_INFO("Printing optimized camera rig parameters:\n{}", *camera_rig_);

    // 结束估计线程
    running_ = false;
    lidar_frame_bundle_buffer_.push(nullptr);
    imu_buffer_.push(Imu());
    estimate_thread_->join();

    LVINS_INFO("Estimator closed!");
}

void Estimator::reset() {
    LVINS_INFO("Resetting estimator...");

    // 使能标志位
    reset_ = true;
}

void Estimator::addImu(const Imu &imu) {
    // 若加速度计输出单位为g，则转换为m/s2
    Imu input(imu);
    if (acc_in_g_) {
        input.acc *= gravity_mag_;
    }

    // 将IMU加入缓冲区
    imu_buffer_.push(std::move(input));
}

void Estimator::addPointClouds(int64_t timestamp, const std::vector<RawPointCloud::Ptr> &raw_point_clouds) {
    LVINS_CHECK(raw_point_clouds.size() == lidar_rig_->size(),
                "The number of raw point clouds should match the number of lidars!");

    // 预处理点云并生成帧
    std::vector<LidarFrame::Ptr> lidar_frames(raw_point_clouds.size());
    size_t icp_points = 0;
    LVINS_START_TIMER(lidar_preprocess_timer_);
    for (size_t i = 0; i < raw_point_clouds.size(); ++i) {
        lidar_frames[i] = std::make_shared<LidarFrame>(timestamp, lidar_rig_->lidar(i), raw_point_clouds[i]);
        lidar_frames[i]->preprocessPointCloud() = preprocessor_->process(lidar_frames[i]->rawPointCloud());
        estimateCovariance(*lidar_frames[i]->preprocessPointCloud(), cov_estimation_neighbors_);
        icp_points += lidar_frames[i]->preprocessPointCloud()->size();
    }
    LVINS_STOP_TIMER(lidar_preprocess_timer_);
    LVINS_PRINT_TIMER_MS("Lidar preprocess", lidar_preprocess_timer_);

    // 若点云点数太少，则跳过
    if (icp_points < min_icp_points_) {
        LVINS_WARN("Not enough points for registration! Skipping lidar frame bundle at {}.",
                   LVINS_GROUP_DIGITS(timestamp));
        return;
    }

    // 将帧束加入缓冲区
    lidar_frame_bundle_buffer_.push(std::make_shared<LidarFrameBundle>(lidar_frames));
}

void Estimator::addImages(int64_t /*timestamp*/, const std::vector<cv::Mat> & /*raw_images*/) {
    LVINS_WARN("Image processing is not implemented!");
}

void Estimator::estimateLoop() {
    // 同步缓冲区
    if (!syncBuffer())
        return;

    // 估计循环
    while (true) {
        // 从缓冲区中获取测量值
        LVINS_START_TIMER(pop_timer_);
        if (!getMeasurementFromBuffer()) {
            return;
        }
        LVINS_STOP_TIMER(pop_timer_);
        LVINS_PRINT_TIMER_MS("Get measurement", pop_timer_);

        LVINS_INFO("Lidar frame bundle timestamp range: [{} - {}]",
                   LVINS_GROUP_DIGITS(last_lidar_frame_bundle_->timestamp()),
                   LVINS_GROUP_DIGITS(cur_lidar_frame_bundle_->timestamp()));
        LVINS_INFO("IMUs timestamp range: [{} - {}]", LVINS_GROUP_DIGITS(cur_imus_.front().timestamp),
                   LVINS_GROUP_DIGITS(cur_imus_.back().timestamp));

        if (status_ == EstimatorStatus::INITIALIZING) {
            // 初始化时不进行点云矫正
            for (size_t i = 0; i < cur_lidar_frame_bundle_->size(); ++i) {
                const auto &frame   = cur_lidar_frame_bundle_->frame(i);
                frame->pointCloud() = copy(*frame->preprocessPointCloud());
            }

            // 尝试初始化
            if (initializer_->tryInitialize(cur_lidar_frame_bundle_, cur_imus_)) {
                // 初始化ESKF
                eskf_->initialize(initializer_->navState());

                // 初始化局部地图并绘制
                for (const auto &bundle: initializer_->lidarFrameBundles()) {
                    for (size_t i = 0; i < bundle->size(); ++i) {
                        const auto &frame = bundle->frame(i);
                        local_mapper_->insert(*frame->pointCloud(), frame->Twf());
                    }
                    drawer_->updateLidarFrameBundle(bundle->timestamp(), bundle);
                }

                // 绘制初始导航状态
                drawer_->updateNavState(eskf_->state().timestamp, eskf_->state());

                // 更新状态标志位
                status_ = EstimatorStatus::ESTIMATING;
            }
        } else if (status_ == EstimatorStatus::ESTIMATING) {
            // IMU预测
            const auto states = eskf_->propagate(cur_imus_);
            cur_lidar_frame_bundle_->setState(eskf_->state());
            // const auto predict_state = eskf_->state();

            // 如果使用IMU先验，则进行点云矫正
            LVINS_START_TIMER(deskew_timer_);
            for (size_t i = 0; i < cur_lidar_frame_bundle_->size(); ++i) {
                const auto &frame = cur_lidar_frame_bundle_->frame(i);
                if (use_imu_prior_) {
                    // frame->pointCloud() = deskew(*frame->preprocessPointCloud(), frame->lidar(), frame->Tbs(), states);
                    frame->pointCloud() = copy(*frame->preprocessPointCloud());
                } else {
                    frame->pointCloud() = copy(*frame->preprocessPointCloud());
                }
            }
            LVINS_STOP_TIMER(deskew_timer_);
            LVINS_PRINT_TIMER_MS("Deskew", deskew_timer_);

            // 量测更新
            if (lidar_rig_ && !camera_rig_) {
                const auto task = std::make_shared<eskf::UpdateLidar>(
                        cur_lidar_frame_bundle_->timestamp(), cur_lidar_frame_bundle_, *lidar_rig_,
                        *point_cloud_aligner_, *local_mapper_, *drawer_, 0, estimate_extrinsic_);
                eskf_->update(task);
            }
            // const auto update_state = eskf_->state();

            /*static NavState state(initializer_->navState());
            state.timestamp = cur_lidar_frame_bundle_->timestamp();

            for (size_t i = 0; i < cur_lidar_frame_bundle_->size(); ++i) {
                cur_lidar_frame_bundle_->frame(i)->pointCloud() =
                        copy(*cur_lidar_frame_bundle_->frame(i)->preprocessPointCloud());
            }

            const auto result = point_cloud_aligner_->align(
                    local_mapper_->getSearch(), cur_lidar_frame_bundle_->pointClouds(), *noise_params_, state.T,
                    cur_lidar_frame_bundle_->Tbs(), estimate_extrinsic_);

            state.T = result.T_tb;
            cur_lidar_frame_bundle_->setState(state);

            for (size_t i = 0; i < cur_lidar_frame_bundle_->size(); ++i) {
                const auto &frame = cur_lidar_frame_bundle_->frame(i);
                local_mapper_->insert(*frame->pointCloud(), frame->Twf());
            }

            drawer_->updateLidarFrameBundle(cur_lidar_frame_bundle_->timestamp(), cur_lidar_frame_bundle_);
            drawer_->updateNavState(state.timestamp, state);*/
        }

        // 检查是否需要重置，重置后需要同步缓冲区
        if (reset_) {
            internalReset();
            if (!syncBuffer())
                return;
        }
    }
}

void Estimator::internalReset() {
    // 重置系统
    status_ = EstimatorStatus::INITIALIZING;
    drawer_->reset();
    local_mapper_->reset();
    initializer_->reset();

    // 清空缓冲区
    lidar_frame_bundle_buffer_.clear();
    imu_buffer_.clear();
    cur_lidar_frame_bundle_.reset();
    last_lidar_frame_bundle_.reset();
    preintegrate_imus_.clear();
    last_imu_ = Imu();

    // 重置标志位
    reset_ = false;
    ++reset_count_;

    LVINS_INFO("Estimator reset!");
}

bool Estimator::syncBuffer() {
    // 从缓冲区中获取一个历元的测量值，用于比较
    last_lidar_frame_bundle_ = lidar_frame_bundle_buffer_.pop();
    last_imu_                = imu_buffer_.pop();

    if (!running_) {
        return false;
    }

    // 对IMU和帧束进行对齐
    while (last_imu_.timestamp >= last_lidar_frame_bundle_->timestamp()) {
        last_lidar_frame_bundle_ = lidar_frame_bundle_buffer_.pop();

        if (!running_) {
            return false;
        }
    }
    Imus imus;
    cur_lidar_frame_bundle_ = last_lidar_frame_bundle_;
    return getImusFromBuffer(cur_lidar_frame_bundle_->timestamp(), imus);
}

bool Estimator::getMeasurementFromBuffer() {
    LVINS_CHECK(cur_lidar_frame_bundle_, "Current frame bundle should be initialized!");

    // 保存上一帧束，并从帧束缓冲区中取出新帧束
    last_lidar_frame_bundle_ = cur_lidar_frame_bundle_;
    cur_lidar_frame_bundle_  = lidar_frame_bundle_buffer_.pop();

    if (!running_) {
        return false;
    }

    // 设置当前帧束外参
    cur_lidar_frame_bundle_->setTbs(lidar_rig_->Tbs());

    // 从缓冲区中获取当前帧束之前的IMU
    return getImusFromBuffer(cur_lidar_frame_bundle_->timestamp(), cur_imus_);
}

bool Estimator::getImusFromBuffer(int64_t timestamp, Imus &imus) {
    LVINS_CHECK(last_imu_, "Last IMU should be initialized!");

    // 将上一个IMU加入容器
    imus.clear();
    imus.push_back(last_imu_);

    // 从IMU缓冲区中取出timestamp之前的IMU
    while (true) {
        auto imu = imu_buffer_.pop();

        if (!running_) {
            return false;
        }

        // 根据时间戳对IMU做对应处理
        if (imu.timestamp < timestamp) {
            imus.push_back(std::move(imu));
        } else {
            last_imu_ = Imu::interpolate(imus.back(), imu, timestamp);
            imus.push_back(last_imu_);
            return true;
        }
    }
}

} // namespace lvins
