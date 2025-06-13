#include "lvins_odometry/init/dynamic_initializer.h"
#include "lvins_common/gtsam_helper.h"
#include "lvins_common/rotation_helper.h"
#include "lvins_icp/preprocess/deskew.h"

namespace lvins {

DynamicInitializer::DynamicInitializer(const VecXf &parameters, const NoiseParameters &noise_params,
                                       const PointCloudAligner &aligner, Vec3f g_w)
    : InitializerBase(noise_params, aligner, std::move(g_w)) {
    LVINS_CHECK(parameters.size() == 1, "Parameters size should be 1! Order: [buffer_size]");
    buffer_size_ = static_cast<size_t>(parameters[0]);
    LVINS_CHECK(buffer_size_ > 0, "Buffer size should be positive!");

    // 预积分噪声模型
    const auto &gyr_std         = noise_params_.gyr_std;
    const auto &acc_std         = noise_params_.acc_std;
    const auto &gyr_bias_std    = noise_params_.gyr_bias_std;
    const auto &acc_bias_std    = noise_params_.acc_bias_std;
    const auto &integration_std = noise_params_.integration_std;
    const auto &init_bias_std   = noise_params_.init_bias_std;
    // clang-format off
    imu_params_ = ImuParams::MakeSharedD();
    imu_params_->setGyroscopeCovariance(gyr_std * gyr_std * gtsam::I_3x3);
    imu_params_->setAccelerometerCovariance(acc_std * acc_std * gtsam::I_3x3);
    imu_params_->setBiasOmegaCovariance(gyr_bias_std * gyr_bias_std * gtsam::I_3x3);
    imu_params_->setBiasAccCovariance(acc_bias_std * acc_bias_std * gtsam::I_3x3);
    imu_params_->setIntegrationCovariance(integration_std * integration_std * gtsam::I_3x3);
    imu_params_->setBiasAccOmegaInit(init_bias_std * init_bias_std * gtsam::I_6x6);
    // clang-format on
}

bool DynamicInitializer::tryInitialize(const LidarFrameBundle::Ptr &lidar_frame_bundle, const Imus &imus) {
    LVINS_CHECK(!imus.empty(), "Imus should not be empty!");

    if (!initialized_) {
        // 将新的IMU进行单独的预积分，分别加入IMU缓冲区（用于求解和求解完成后的优化）和预积分缓冲区（用于求解）
        gtsam::PreintegratedImuMeasurements preintegrated(imu_params_);
        NavState state(imus[0].timestamp, SE3f(Mat44f::Identity()), Vec3f::Zero(), Vec3f::Zero(), Vec3f::Zero());
        NavStates states{state};
        for (size_t i = 1; i < imus.size(); ++i) {
            const auto dt       = 1e-9 * static_cast<double>(imus[i].timestamp - imus[i - 1].timestamp);
            const Vec3f mid_gyr = 0.5 * (imus[i - 1].gyr + imus[i].gyr);
            const Vec3f mid_acc = 0.5 * (imus[i - 1].acc + imus[i].acc);
            preintegrated.integrateMeasurement(mid_acc.cast<double>(), mid_gyr.cast<double>(), dt);

            state.timestamp = imus[i].timestamp;
            state.T         = toSE3(preintegrated.deltaRij(), preintegrated.deltaPij());
            state.vel       = preintegrated.deltaVij().cast<Float>();
            states.push_back(state);
        }
        preintegrated_buffer_.push_back(std::move(preintegrated));
        imu_buffer_.push_back(imus);

        // 对帧束进行点云矫正并加入帧束缓冲区
        for (size_t i = 0; i < lidar_frame_bundle->size(); ++i) {
            const auto &frame   = lidar_frame_bundle->frame(i);
            frame->pointCloud() = deskew(*frame->preprocessPointCloud(), frame->lidar(), frame->Tbs(), states);
        }
        lidar_frame_bundle_buffer_.push_back(lidar_frame_bundle);

        // 首次输入，则不需要进行点云配准
        if (!last_lidar_frame_bundle_) {
            T_delta_buffer_.emplace_back(Eigen::Matrix4d::Identity());
            last_lidar_frame_bundle_ = lidar_frame_bundle;
            return false;
        } else { // 否则，需要进行点云配准和缓冲区维护（滑动窗口）
            const auto result = aligner_.align(last_lidar_frame_bundle_->pointClouds(),
                                               lidar_frame_bundle->pointClouds(), state.T, lidar_frame_bundle->Tbs());
            SE3f T_delta      = result.T_tb;
            T_delta_buffer_.push_back(T_delta.cast<double>());
            last_lidar_frame_bundle_ = lidar_frame_bundle;

            // 缓冲区不满，直接退出，否则需要弹出队首
            if (preintegrated_buffer_.size() < buffer_size_) {
                return false;
            } else if (preintegrated_buffer_.size() > buffer_size_) {
                preintegrated_buffer_.pop_front();
                imu_buffer_.pop_front();
                lidar_frame_bundle_buffer_.pop_front();
                T_delta_buffer_.pop_front();
            }

            // 尝试初始化
            std::vector<Vec3f> vels;
            Vec3f g_b0;
            const Vec3f bg = solveGyroscopeBias(preintegrated_buffer_, imu_buffer_, T_delta_buffer_);
            if (linearAlign(preintegrated_buffer_, T_delta_buffer_, vels, g_b0)) {
                // 重力调平
                Vec3f att0;
                att0[0] = LVINS_FLOAT(atan2(-g_b0[1], -g_b0[2]));
                att0[1] = LVINS_FLOAT(atan(g_b0[0] / sqrt(g_b0[1] * g_b0[1] + g_b0[2] * g_b0[2])));
                att0[2] = 0.0;

                // 构建初始帧束状态
                const SE3f T_wb0(rotation_helper::toSO3(att0), Vec3f::Zero());
                NavState state0(lidar_frame_bundle_buffer_[0]->timestamp(), T_wb0, T_wb0.so3() * vels[0], bg,
                                Vec3f::Zero());
                lidar_frame_bundle_buffer_[0]->setState(state0);

                // 构建缓冲区内剩余帧束状态
                for (size_t i = 1; i < lidar_frame_bundle_buffer_.size(); ++i) {
                    SE3f T_wb = lidar_frame_bundle_buffer_[i - 1]->Twb() * T_delta_buffer_[i].cast<Float>();
                    NavState state(lidar_frame_bundle_buffer_[i]->timestamp(), T_wb, T_wb.so3() * vels[i], bg,
                                   Vec3f::Zero());
                    lidar_frame_bundle_buffer_[i]->setState(state);
                }

                // 构建输出
                lidar_frame_bundles_ = {lidar_frame_bundle_buffer_.begin(), lidar_frame_bundle_buffer_.end()};
                nav_state_           = lidar_frame_bundles_.back()->state();

                // 使能标志位并打印状态
                initialized_ = true;
                LVINS_INFO("Dynamic initialize done!\n"
                           "  State:\n{}\n"
                           "  Lidar frame bundle size: {}",
                           nav_state_, lidar_frame_bundle_buffer_.size());
            } else {
                return false;
            }
        }
    }
    return true;
}

const std::vector<LidarFrameBundle::Ptr> &DynamicInitializer::lidarFrameBundles() const {
    return lidar_frame_bundles_;
}

const NavState &DynamicInitializer::navState() const {
    return nav_state_;
}

void DynamicInitializer::reset() {
    LVINS_INFO("Resetting dynamic initializer...");

    // 清空缓冲区
    preintegrated_buffer_.clear();
    imu_buffer_.clear();
    lidar_frame_bundle_buffer_.clear();
    T_delta_buffer_.clear();
    lidar_frame_bundles_.clear();
    nav_state_               = NavState();
    last_lidar_frame_bundle_ = nullptr;

    // 重置标志位
    initialized_ = false;

    LVINS_INFO("Dynamic initializer reset!");
}

std::string DynamicInitializer::type() const {
    return "dynamic";
}

VecXf DynamicInitializer::parameters() const {
    VecXf parameters(1);
    parameters[0] = LVINS_FLOAT(buffer_size_);
    return parameters;
}

std::string DynamicInitializer::print() const {
    return LVINS_FORMAT("Dynamic initializer:\n  buffer size = {}", buffer_size_);
}

Vec3f DynamicInitializer::solveGyroscopeBias(std::deque<gtsam::PreintegratedImuMeasurements> &preintegrated,
                                             const std::deque<Imus> &imus, const std::deque<SE3d> &T_delta) {
    LVINS_CHECK(!preintegrated.empty(), "preintegrated should not be empty!");
    LVINS_CHECK(preintegrated.size() == imus.size(), "preintegrated and imus should have same size!");
    LVINS_CHECK(preintegrated.size() == T_delta.size(), "preintegrated and T_delta should have same size!");

    // 遍历雷达位姿增量与预积分，构建线性求解问题
    Mat33d A = Mat33d::Zero();
    Vec3d b  = Vec3d::Zero();
    for (size_t i = 1; i < preintegrated.size(); ++i) {
        Mat33d sub_A = preintegrated[i].preintegrated_H_biasOmega().topRows<3>();
        Vec3d sub_b  = (SO3d::exp(preintegrated[i].theta()).inverse() * T_delta[i].so3()).log();
        A += sub_A.transpose() * sub_A;
        b += sub_A.transpose() * sub_b;
    }

    // Cholosky分解求解问题
    Vec3f bg = A.ldlt().solve(b).cast<Float>();
    LVINS_INFO("Estimate gyroscope bias: {}", LVINS_VECTOR_FMT(bg));

    // 重新预积分
    for (size_t i = 1; i < preintegrated.size(); ++i) {
        preintegrated[i].resetIntegrationAndSetBias(toImuBias(bg, Vec3f::Zero()));
        for (size_t j = 1; j < imus[i].size(); ++j) {
            const auto dt       = 1e-9 * static_cast<double>(imus[i][j].timestamp - imus[i][j - 1].timestamp);
            const Vec3f mid_gyr = 0.5 * (imus[i][j - 1].gyr + imus[i][j].gyr);
            const Vec3f mid_acc = 0.5 * (imus[i][j - 1].acc + imus[i][j].acc);
            preintegrated[i].integrateMeasurement(mid_acc.cast<double>(), mid_gyr.cast<double>(), dt);
        }
    }

    return bg;
}

bool DynamicInitializer::linearAlign(const std::deque<gtsam::PreintegratedImuMeasurements> &preintegrated,
                                     const std::deque<SE3d> &T_delta, std::vector<Vec3f> &vels_b, Vec3f &g_b0) const {
    LVINS_CHECK(!preintegrated.empty(), "preintegrated should not be empty!");
    LVINS_CHECK(preintegrated.size() == T_delta.size(), "preintegrated and T_delta should have same size!");

    // 准备数据
    const long dim = static_cast<long>(preintegrated.size()) * 3 + 3; ///< 速度+重力向量
    MatXd A        = MatXd::Zero(dim, dim);
    VecXd b        = VecXd::Zero(dim);
    SO3d R_b0bk(Mat33d::Identity());

    // 遍历雷达位姿增量与预积分，构建线性求解问题
    for (size_t i = 1; i < preintegrated.size(); ++i) {
        MatXd sub_A = MatXd::Zero(6, 9);
        VecXd sub_b = VecXd::Zero(6);

        const auto dt           = preintegrated[i].deltaTij();
        sub_A.block<3, 3>(0, 0) = -dt * Mat33d::Identity();
        sub_A.block<3, 3>(0, 6) = 0.5 * dt * dt * R_b0bk.inverse().matrix();
        sub_A.block<3, 3>(3, 0) = -Mat33d::Identity();
        sub_A.block<3, 3>(3, 3) = T_delta[i].so3().matrix();
        sub_A.block<3, 3>(3, 6) = dt * R_b0bk.inverse().matrix();
        sub_b.head<3>()         = preintegrated[i].deltaPij() - T_delta[i].translation();
        sub_b.tail<3>()         = preintegrated[i].deltaVij();

        MatXd resolve_sub_A = sub_A.transpose() * sub_A;
        VecXd resolve_sub_b = sub_A.transpose() * sub_b;

        const long idx = static_cast<long>(i - 1) * 3;
        A.block<6, 6>(idx, idx) += resolve_sub_A.topLeftCorner<6, 6>();
        A.bottomRightCorner<3, 3>() += resolve_sub_A.bottomRightCorner<3, 3>();
        A.block<6, 3>(idx, dim - 3) += resolve_sub_A.topRightCorner<6, 3>();
        A.block<3, 6>(dim - 3, idx) += resolve_sub_A.bottomLeftCorner<3, 6>();
        b.segment<6>(idx) += resolve_sub_b.head<6>();
        b.tail<3>() += resolve_sub_b.tail<3>();

        R_b0bk *= T_delta[i].so3();
    }

    // 防止求解数值不稳定
    A *= 1e3;
    b *= 1e3;

    // Cholosky分解求解问题
    VecXd x = A.ldlt().solve(b);

    // 检查重力向量模长
    const Vec3f g_coarse = x.tail<3>().cast<Float>();
    LVINS_INFO("Estimate gravity: {}, magnitude: {:.4f}", LVINS_VECTOR_FMT(g_coarse), g_coarse.norm());
    if (std::fabs(g_coarse.norm() - g_w_.norm()) > 0.5) {
        return false;
    }

    // 优化重力向量
    refineGravity(preintegrated, T_delta, x);
    g_b0 = x.tail<3>().cast<Float>();
    LVINS_INFO("Refine gravity: {}, magnitude: {:.4f}", LVINS_VECTOR_FMT(g_b0), g_b0.norm());

    // 获取速度
    for (long i = 0; i < dim - 3; i += 3) {
        vels_b.emplace_back(x.segment<3>(i).cast<Float>());
    }

    return true;
}

void DynamicInitializer::refineGravity(const std::deque<gtsam::PreintegratedImuMeasurements> &preintegrated,
                                       const std::deque<SE3d> &T_delta, VecXd &x) const {
    // 准备数据
    Vec3d g0       = x.tail<3>().normalized() * g_w_.norm();
    const long dim = static_cast<long>(preintegrated.size()) * 3 + 2; ///< 速度+重力优化向量
    MatXd A        = MatXd::Zero(dim, dim);
    VecXd b        = VecXd::Zero(dim);
    VecXd refine_x;
    SO3d R_b0bk(Mat33d::Identity());

    // 迭代优化
    for (size_t iter = 0; iter < 5; ++iter) {
        // 获取重力向量球面切向的标准正交基
        const Mat32d basis = getTangentBasis(g0);

        // 遍历雷达位姿增量与预积分，构建线性求解问题
        for (size_t i = 1; i < preintegrated.size(); ++i) {
            MatXd sub_A = MatXd::Zero(6, 8);
            VecXd sub_b = VecXd::Zero(6);

            const auto dt           = preintegrated[i].deltaTij();
            sub_A.block<3, 3>(0, 0) = -dt * Mat33d::Identity();
            sub_A.block<3, 2>(0, 6) = 0.5 * dt * dt * R_b0bk.inverse().matrix() * basis;
            sub_A.block<3, 3>(3, 0) = -Mat33d::Identity();
            sub_A.block<3, 3>(3, 3) = T_delta[i].so3().matrix();
            sub_A.block<3, 2>(3, 6) = dt * R_b0bk.inverse().matrix() * basis;
            sub_b.head<3>()         = preintegrated[i].deltaPij() - T_delta[i].translation() -
                              0.5 * dt * dt * R_b0bk.inverse().matrix() * g0;
            sub_b.tail<3>() = preintegrated[i].deltaVij() - dt * R_b0bk.inverse().matrix() * g0;

            MatXd resolve_sub_A = sub_A.transpose() * sub_A;
            VecXd resolve_sub_b = sub_A.transpose() * sub_b;

            const long idx = static_cast<long>(i - 1) * 3;
            A.block<6, 6>(idx, idx) += resolve_sub_A.topLeftCorner<6, 6>();
            A.bottomRightCorner<2, 2>() += resolve_sub_A.bottomRightCorner<2, 2>();
            A.block<6, 2>(idx, dim - 2) += resolve_sub_A.topRightCorner<6, 2>();
            A.block<2, 6>(dim - 2, idx) += resolve_sub_A.bottomLeftCorner<2, 6>();
            b.segment<6>(idx) += resolve_sub_b.head<6>();
            b.tail<2>() += resolve_sub_b.tail<2>();

            R_b0bk *= T_delta[i].so3();
        }

        // 防止求解数值不稳定
        A *= 1e3;
        b *= 1e3;

        // Cholosky分解求解问题，并更新重力向量
        refine_x      = A.ldlt().solve(b);
        const Vec2d w = refine_x.tail<2>();
        g0            = (g0 + basis * w).normalized() * g_w_.norm();
    }

    // 构建返回值
    x.head(dim - 2) = refine_x.head(dim - 2);
    x.tail<3>()     = g0;
}

Mat32d DynamicInitializer::getTangentBasis(const Vec3d &g0) {
    // 准备数据
    const Vec3d b0 = g0.normalized();
    Vec3d unit_vec(0, 0, 1);
    if ((b0 - unit_vec).norm() < 1e-5) {
        unit_vec << 1, 0, 0;
    }

    // 构建球面切向标准正交基
    const Vec3d b1 = (unit_vec - b0 * (b0.transpose() * unit_vec)).normalized();
    const Vec3d b2 = b0.cross(b1);
    Mat32d basis;
    basis << b1, b2;
    return basis;
}

} // namespace lvins
