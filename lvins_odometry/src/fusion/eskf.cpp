#include "lvins_odometry/fusion/eskf.h"

namespace lvins {

ESKF::ESKF(const YAML::Node &config, const NoiseParameters &noise_params, Vec3f g_w, std::vector<SE3f> T_bs)
    : g_w_(std::move(g_w)),
      origin_g_w_(g_w_),
      T_bs_(std::move(T_bs)),
      origin_T_bs_(T_bs_),
      noise_params_(noise_params) {
    // 初始化参数
    max_iterations_     = YAML::get<size_t>(config, "max_iterations");
    iteration_quit_eps_ = YAML::get<double>(config, "iteration_quit_eps");
    verbosity_          = YAML::get<bool>(config, "verbosity");
    dim_                = O_EXT_P + 6 * static_cast<long>(T_bs_.size());

    // 初始化传感器噪声协方差矩阵
    Q_                         = MatXd::Zero(dim_, dim_);
    Q_.block<3, 3>(O_V, O_V)   = std::pow(noise_params_.acc_std, 2) * Mat33d::Identity();
    Q_.block<3, 3>(O_Q, O_Q)   = std::pow(noise_params_.gyr_std, 2) * Mat33d::Identity();
    Q_.block<3, 3>(O_BG, O_BG) = std::pow(noise_params_.gyr_bias_std, 2) * Mat33d::Identity();
    Q_.block<3, 3>(O_BA, O_BA) = std::pow(noise_params_.acc_bias_std, 2) * Mat33d::Identity();
}

YAML::Node ESKF::writeToYaml() const {
    YAML::Node node;
    node["max_iterations"]     = max_iterations_;
    node["iteration_quit_eps"] = iteration_quit_eps_;
    node["verbosity"]          = verbosity_;
    return node;
}

void ESKF::reset() {
    g_w_  = origin_g_w_;
    T_bs_ = origin_T_bs_;
}

void ESKF::initialize(const NavState &state) {
    LVINS_CHECK(state, "Input state should be initialized!");

    // 初始化状态
    state_ = state;

    // 初始化误差协方差矩阵
    P_                       = MatXd::Zero(dim_, dim_);
    P_.block<3, 3>(O_P, O_P) = std::pow(noise_params_.prior_pos_std, 2) * Mat33d::Identity();
    P_.block<3, 3>(O_V, O_V) = std::pow(noise_params_.prior_vel_std, 2) * Mat33d::Identity();
    P_.block<3, 3>(O_Q, O_Q) =
            Diag3d(std::pow(noise_params_.prior_roll_pitch_std, 2), std::pow(noise_params_.prior_roll_pitch_std, 2),
                   std::pow(noise_params_.prior_yaw_std, 2));
    P_.block<3, 3>(O_BG, O_BG) = std::pow(noise_params_.prior_gyr_bias_std, 2) * Mat33d::Identity();
    P_.block<3, 3>(O_BA, O_BA) = std::pow(noise_params_.prior_acc_bias_std, 2) * Mat33d::Identity();
    P_.block<3, 3>(O_GW, O_GW) = 1e-4 * Mat33d::Identity();
    for (long i = 0; i < static_cast<long>(T_bs_.size()); ++i) {
        P_.block<3, 3>(O_EXT_P + 6 * i, O_EXT_P + 6 * i) =
                std::pow(noise_params_.ext_trans_std, 2) * Mat33d::Identity();
        P_.block<3, 3>(O_EXT_Q + 6 * i, O_EXT_Q + 6 * i) = std::pow(noise_params_.ext_rot_std, 2) * Mat33d::Identity();
    }
}

NavStates ESKF::propagate(const Imus &imus) {
    LVINS_CHECK(state_, "State should be initialized first!");
    LVINS_CHECK(imus.size() > 1, "Input IMUs size should be greater than 1!");
    LVINS_CHECK(state_.timestamp == imus[0].timestamp,
                "First IMU timestamp should be aligned with current state timestamp!");

    NavStates states{state_};
    for (size_t i = 1; i < imus.size(); ++i) {
        // 中间量计算
        const auto &imu0 = imus[i - 1];
        const auto &imu1 = imus[i];
        const auto dt    = static_cast<double>(imu1.timestamp - imu0.timestamp) * 1e-9;

        // IMU递推
        Vec3f mid_gyr = 0.5 * (imu0.gyr + imu1.gyr) - state_.bg;
        const SO3f q0 = state_.T.so3();
        state_.T.so3() *= SO3f::exp(mid_gyr * dt);
        state_.T.normalize();

        Vec3f mid_acc = 0.5 * (q0 * (imu0.acc - state_.ba) + state_.T.so3() * (imu1.acc - state_.ba)) + g_w_;
        state_.T.translation() += state_.vel * dt + 0.5 * mid_acc * dt * dt;
        state_.vel += mid_acc * dt;

        // 更新结果
        // 顺序：[p q v bg ba g_w ext_p ext_q]
        state_.timestamp = imu1.timestamp;
        states.push_back(state_);

        // ESKF预测
        const Mat33d q1_mat      = state_.T.so3().matrix().cast<double>();
        MatXd F                  = MatXd::Identity(dim_, dim_);
        F.block<3, 3>(O_P, O_V)  = Mat33d::Identity() * dt;
        F.block<3, 3>(O_V, O_Q)  = -q1_mat * SO3d::hat(mid_acc.cast<double>()) * dt;
        F.block<3, 3>(O_V, O_BA) = -q1_mat * dt;
        F.block<3, 3>(O_V, O_GW) = Mat33d::Identity() * dt;
        F.block<3, 3>(O_Q, O_Q)  = SO3d::exp(-mid_gyr.cast<double>() * dt).matrix();
        F.block<3, 3>(O_Q, O_BG) = -Mat33d::Identity() * dt;

        P_ = F * P_ * F.transpose() + Q_;
    }

    return states;
}

const NavState &ESKF::state() const {
    return state_;
}

const std::vector<SE3f> &ESKF::Tbs() const {
    return T_bs_;
}

std::string ESKF::print() const {
    return LVINS_FORMAT("ESKF:\n"
                        "  max iterations = {}\n"
                        "  iteration quit epsilon = {}\n"
                        "  verbosity = {}\n"
                        "  dim = {}",
                        max_iterations_, iteration_quit_eps_, verbosity_, dim_);
}

void ESKF::update(const UpdateTaskPtr &task) {
    LVINS_CHECK(state_.timestamp == task->timestamp(), "Task timestamp should match ESKF current state!");

    // 计算观测量
    MatXd H, V;
    VecXd r;
    task->observe(noise_params_, dim_, state_, T_bs_, H, V, r);

    // 计算卡尔曼增益和残差
    const MatXd K           = P_ * H.transpose() * (H * P_ * H.transpose() + V).inverse();
    const VecXd delta_state = K * r;

    // 打印调试信息
    if (verbosity_) {
        LVINS_DEBUG("H = {}", LVINS_MATRIX_FMT(H));
        LVINS_DEBUG("V = {}", LVINS_MATRIX_FMT(V));
        LVINS_DEBUG("r = {}", LVINS_VECTOR_FMT(r));
        LVINS_DEBUG("K = {}", LVINS_MATRIX_FMT(K));
        LVINS_DEBUG("delta_state = {}", LVINS_VECTOR_FMT(delta_state));
    }

    // 更新名义状态和误差协方差矩阵
    correctNominal(delta_state.cast<Float>());
    std::vector<Vec3d> q_bs_err;
    for (long i = 0; i < static_cast<long>(T_bs_.size()); ++i) {
        q_bs_err.emplace_back(delta_state.segment<3>(O_EXT_Q + 6 * i));
    }
    P_ = (MatXd::Identity(dim_, dim_) - K * H) * P_;
    P_ = projectCovariance(delta_state.segment<3>(O_Q), q_bs_err);

    // 结尾处理
    task->finalize(state_, T_bs_);
}

void ESKF::update(const iUpdateTaskPtr &task) {
    LVINS_CHECK(state_.timestamp == task->timestamp(), "Task timestamp should match ESKF current state!");

    // 记录初值
    const auto &state     = state_;
    const SO3d q_wb_begin = state.T.so3().cast<double>();
    std::vector<SO3d> q_bs_begin;
    for (auto &T_bs: T_bs_) {
        q_bs_begin.push_back(T_bs.so3().cast<double>());
    }

    // 迭代循环
    MatXd Ht_Vinv_H;
    VecXd Ht_Vinv_r;
    MatXd Pk, Qk;
    for (size_t iter = 0; iter < max_iterations_; ++iter) {
        // 计算观测量
        task->observe(noise_params_, dim_, state, T_bs_, Ht_Vinv_H, Ht_Vinv_r);

        // 投影此次迭代的误差协方差矩阵
        const Vec3d q_wb_err = (state.T.so3().inverse().cast<double>() * q_wb_begin).log();
        std::vector<Vec3d> q_bs_err;
        for (size_t i = 0; i < T_bs_.size(); ++i) {
            q_bs_err.push_back((T_bs_[i].so3().inverse().cast<double>() * q_bs_begin[i]).log());
        }
        Pk = projectCovariance(q_wb_err, q_bs_err);

        // 计算中间量和残差
        Qk                      = (Pk.inverse() + Ht_Vinv_H).inverse();
        const VecXd delta_state = Qk * Ht_Vinv_r;

        // 打印调试信息
        if (verbosity_) {
            LVINS_INFO("Ht_Vinv_H = {}", LVINS_MATRIX_FMT(Ht_Vinv_H));
            LVINS_INFO("Ht_Vinv_r = {}", LVINS_VECTOR_FMT(Ht_Vinv_r));
            LVINS_INFO("Qk = {}", LVINS_MATRIX_FMT(Qk));
            LVINS_INFO("delta_state = {}", LVINS_VECTOR_FMT(delta_state));
        }

        // 更新名义状态
        correctNominal(delta_state.cast<Float>());

        // 误差小于阈值则退出
        if (delta_state.norm() < iteration_quit_eps_) {
            break;
        }
    }

    // 更新误差协方差矩阵
    P_                   = (MatXd::Identity(dim_, dim_) - Qk * Ht_Vinv_H) * Pk;
    const Vec3d q_wb_err = (state.T.so3().inverse().cast<double>() * q_wb_begin).log();
    std::vector<Vec3d> q_bs_err;
    for (size_t i = 0; i < T_bs_.size(); ++i) {
        q_bs_err.push_back((T_bs_[i].so3().inverse().cast<double>() * q_bs_begin[i]).log());
    }
    P_ = projectCovariance(q_wb_err, q_bs_err);

    // 结尾处理
    task->finalize(state_, T_bs_);
}

void ESKF::correctNominal(const VecXf &delta_state) {
    // 修正状态
    state_.T.translation() += delta_state.segment<3>(O_P);
    state_.vel += delta_state.segment<3>(O_V);
    state_.T.so3() *= SO3f::exp(delta_state.segment<3>(O_Q));
    state_.bg += delta_state.segment<3>(O_BG);
    state_.ba += delta_state.segment<3>(O_BA);
    state_.T.normalize();

    // 修正重力向量
    g_w_ += delta_state.segment<3>(O_GW);

    // 修正外参
    for (long i = 0; i < static_cast<long>(T_bs_.size()); ++i) {
        T_bs_[i].translation() += delta_state.segment<3>(O_EXT_P + 6 * i);
        T_bs_[i].so3() *= SO3f::exp(delta_state.segment<3>(O_EXT_Q + 6 * i));
        T_bs_[i].normalize();
    }
}

MatXd ESKF::projectCovariance(const Eigen::Ref<const Vec3d> &q_wb_err, const std::vector<Vec3d> &q_bs_err) const {
    // 计算雅可比矩阵
    MatXd J = MatXd::Identity(dim_, dim_);
    J.block<3, 3>(O_Q, O_Q) -= 0.5 * SO3d::hat(q_wb_err);
    for (long i = 0; i < static_cast<long>(q_bs_err.size()); ++i) {
        J.block<3, 3>(O_EXT_Q + 6 * i, O_EXT_Q + 6 * i) -= 0.5 * SO3d::hat(q_bs_err[i]);
    }

    // 投影误差协方差矩阵
    return J * P_ * J.transpose();
}

} // namespace lvins
