#include "lvins_odometry/init/static_initializer.h"
#include "lvins_common/rotation_helper.h"
#include "lvins_odometry/fusion/ins_helper.h"

namespace lvins {

StaticInitializer::StaticInitializer(const VecXf &parameters, Vec3f g_w) : InitializerBase(std::move(g_w)) {
    LVINS_CHECK(parameters.size() == 3,
                "Parameters size should be 3! Order: [init_period, zero_gyr_thresh, zero_acc_thresh]");
    init_period_     = static_cast<int64_t>(parameters[0] * 1e9);
    zero_gyr_thresh_ = parameters[1];
    zero_acc_thresh_ = parameters[2];
}

void StaticInitializer::addImu(const Imu &imu) {
    imus_.push_back(imu);
}

void StaticInitializer::addLidarFrameBundle(const LidarFrameBundle::sPtr &bundle) {
    lidar_frame_bundles_.push_back(bundle);
}

bool StaticInitializer::tryInitialize() {
    if (!initialized_) {
        // 检测零速
        if (!ins_helper::detectZeroVelocity(imus_, zero_gyr_thresh_, zero_acc_thresh_)) {
            LVINS_WARN("Please keep static to initialize!");
            imus_.clear();
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

        // 构建初始状态
        const SE3f T_wb(rotation_helper::toSO3(att), Vec3f::Zero());
        NavState state(imus_.front().timestamp, T_wb, Vec3f::Zero(), ave_gyr, ave_acc + T_wb.so3().inverse() * g_w_);

        // 重构帧束容器，仅保留在imus_时间戳范围内的帧束
        lidar_frame_bundles_.erase(std::remove_if(lidar_frame_bundles_.begin(), lidar_frame_bundles_.end(),
                                                  [this](const LidarFrameBundle::sPtr &bundle) {
                                                      return bundle->timestamp() < imus_.front().timestamp ||
                                                             bundle->timestamp() > imus_.back().timestamp;
                                                  }),
                                   lidar_frame_bundles_.end());
        for (const auto &bundle: lidar_frame_bundles_) {
            state.timestamp = bundle->timestamp();
            bundle->setState(state);
        }

        // 构建导航状态容器，时间戳和长度与IMU数据容器一致
        nav_states_ = NavStates(imus_.size(), state);
        for (size_t i = 0; i < imus_.size(); ++i) {
            nav_states_[i].timestamp = imus_[i].timestamp;
        }

        // 使能标志位并打印状态
        initialized_ = true;
        LVINS_INFO("Static initialize done!\nState:\n{}\nIMUs size: {}\nLidar frame bundle size: {}", state,
                   imus_.size(), lidar_frame_bundles_.size());
    }

    return true;
}

const Imus &StaticInitializer::imus() const {
    LVINS_CHECK(initialized_, "IMUs should be accessed after initialized!");
    return imus_;
}

const std::vector<LidarFrameBundle::sPtr> &StaticInitializer::lidarFrameBundles() const {
    LVINS_CHECK(initialized_, "Lidar frame bundles should be accessed after initialized!");
    return lidar_frame_bundles_;
}

const NavStates &StaticInitializer::navStates() const {
    LVINS_CHECK(initialized_, "Nav states should be accessed after initialized!");
    return nav_states_;
}

void StaticInitializer::reset() {
    LVINS_INFO("Resetting static initializer...");

    // 清空容器
    imus_.clear();
    lidar_frame_bundles_.clear();
    nav_states_.clear();

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
