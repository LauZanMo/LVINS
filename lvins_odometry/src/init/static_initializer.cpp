#include "lvins_odometry/init/static_initializer.h"
#include "lvins_common/rotation_helper.h"
#include "lvins_odometry/fusion/ins_helper.h"

namespace lvins {

StaticInitializer::StaticInitializer(const VecXf &parameters, const NoiseParameters &noise_params,
                                     const PointCloudAligner &aligner, Vec3f g_w)
    : InitializerBase(noise_params, aligner, std::move(g_w)) {
    LVINS_CHECK(parameters.size() == 3,
                "Parameters size should be 3! Order: [init_period, zero_gyr_thresh, zero_acc_thresh]");
    init_period_     = static_cast<int64_t>(parameters[0] * 1e9);
    zero_gyr_thresh_ = parameters[1];
    zero_acc_thresh_ = parameters[2];
}

bool StaticInitializer::tryInitialize(const LidarFrameBundle::Ptr &lidar_frame_bundle, const Imus &imus) {
    LVINS_CHECK(!imus.empty(), "Imus should not be empty!");

    if (!initialized_) {
        // 追加IMU并检测零速
        const long offset = imus_.empty() ? 0 : 1; ///< 上一IMU容器尾部元素与当前imu容器头部元素相等，需要剔除
        imus_.insert(imus_.end(), imus.begin() + offset, imus.end());
        lidar_frame_bundles_.push_back(lidar_frame_bundle);
        if (!ins_helper::detectZeroVelocity(imus_, zero_gyr_thresh_, zero_acc_thresh_)) {
            LVINS_WARN("Please keep static to initialize!");
            imus_.clear();
            lidar_frame_bundles_.clear();
            return false;
        }

        // 静止时间未超过阈值，直接退出
        if (imus_.back().timestamp - imus_.front().timestamp < init_period_) {
            return false;
        }

        // 计算平均值
        const auto size_inv = LVINS_FLOAT(1.0) / LVINS_FLOAT(imus_.size());
        Vec3f ave_gyr(Vec3f::Zero()), ave_acc(Vec3f::Zero());
        for (const auto &imu: imus_) {
            ave_gyr += imu.gyr;
            ave_acc += imu.acc;
        }
        ave_gyr *= size_inv;
        ave_acc *= size_inv;

        // 重力调平
        Vec3f att;
        att[0] = LVINS_FLOAT(atan2(-ave_acc[1], -ave_acc[2]));
        att[1] = LVINS_FLOAT(atan(ave_acc[0] / sqrt(ave_acc[1] * ave_acc[1] + ave_acc[2] * ave_acc[2])));
        att[2] = 0.0;

        // 构建初始导航状态
        const SE3f T_wb(rotation_helper::toSO3(att), Vec3f::Zero());
        nav_state_ =
                NavState(imus_.front().timestamp, T_wb, Vec3f::Zero(), ave_gyr, ave_acc + T_wb.so3().inverse() * g_w_);

        // 构件帧束导航状态
        for (const auto &bundle: lidar_frame_bundles_) {
            nav_state_.timestamp = bundle->timestamp();
            bundle->setState(nav_state_);
        }

        // 使能标志位并打印状态
        initialized_ = true;
        LVINS_INFO("Static initialize done!\n"
                   "  State:\n{}\n"
                   "  IMUs size: {}\n"
                   "  Lidar frame bundle size: {}",
                   nav_state_, imus_.size(), lidar_frame_bundles_.size());
    }

    return true;
}

const std::vector<LidarFrameBundle::Ptr> &StaticInitializer::lidarFrameBundles() const {
    LVINS_CHECK(initialized_, "Lidar frame bundles should be accessed after initialized!");
    return lidar_frame_bundles_;
}

const NavState &StaticInitializer::navState() const {
    LVINS_CHECK(initialized_, "Nav states should be accessed after initialized!");
    return nav_state_;
}

void StaticInitializer::reset() {
    LVINS_INFO("Resetting static initializer...");

    // 清空容器
    imus_.clear();
    lidar_frame_bundles_.clear();
    nav_state_ = NavState();

    // 重置标志位
    initialized_ = false;

    LVINS_INFO("Static initializer reset!");
}

std::string StaticInitializer::type() const {
    return "static";
}

VecXf StaticInitializer::parameters() const {
    VecXf parameters(3);
    parameters[0] = LVINS_FLOAT(static_cast<double>(init_period_) * 1e-9);
    parameters[1] = zero_gyr_thresh_;
    parameters[2] = zero_acc_thresh_;
    return parameters;
}

std::string StaticInitializer::print() const {
    return LVINS_FORMAT("Static initializer:\n"
                        "  initialize period = {:.3f}\n"
                        "  zero gyroscope threshold = {}\n"
                        "  zero accelerometer threshold = {}",
                        static_cast<double>(init_period_) * 1e-9, zero_gyr_thresh_, zero_acc_thresh_);
}

} // namespace lvins
