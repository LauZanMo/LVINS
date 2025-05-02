#include "lvins_odometry/fusion/eskf.h"

namespace lvins {

ESKF::ESKF(const YAML::Node &config, NoiseParameters::sConstPtr noise_params, Vec3f g_w, std::vector<SE3f> T_bs)
    : g_w_(std::move(g_w)),
      origin_g_w_(g_w_),
      T_bs_(std::move(T_bs)),
      origin_T_bs_(T_bs_),
      noise_params_(std::move(noise_params)) {
    // 初始化参数
    buffer_len_         = static_cast<int64_t>(YAML::get<double>(config, "buffer_len") * 1e9);
    max_iterations_     = YAML::get<size_t>(config, "max_iterations");
    iteration_quit_eqs_ = YAML::get<Float>(config, "iteration_quit_eqs");
    dim_                = O_EXT_P + 6 * static_cast<long>(T_bs_.size());

    // 初始化传感器噪声协方差矩阵
    Q_                         = MatXf::Zero(dim_, dim_);
    Q_.block<3, 3>(O_V, O_V)   = std::pow(noise_params_->acc_std, 2) * Mat33f::Identity();
    Q_.block<3, 3>(O_Q, O_Q)   = std::pow(noise_params_->gyr_std, 2) * Mat33f::Identity();
    Q_.block<3, 3>(O_BG, O_BG) = std::pow(noise_params_->gyr_bias_std, 2) * Mat33f::Identity();
    Q_.block<3, 3>(O_BA, O_BA) = std::pow(noise_params_->acc_bias_std, 2) * Mat33f::Identity();
}

YAML::Node ESKF::writeToYaml() const {
    YAML::Node node;
    node["buffer_len"]         = static_cast<double>(buffer_len_) * 1e-9;
    node["max_iterations"]     = max_iterations_;
    node["iteration_quit_eqs"] = iteration_quit_eqs_;
    return node;
}

void ESKF::reset() {
    g_w_  = origin_g_w_;
    T_bs_ = origin_T_bs_;
    state_buffer_.clear();
    imu_buffer_.clear();
    update_task_buffer_.clear();
    iterative_update_task_buffer_.clear();
    update_task_           = {-1, nullptr};
    iterative_update_task_ = {-1, nullptr};
}

/**
 * @brief 检查容器所有元素是否已初始化
 * @tparam T 容器类型
 * @param obj 容器对象
 * @return 容器所有元素是否已初始化
 */
template<typename T>
bool checkInitialized(const T &obj) {
    return std::all_of(obj.begin(), obj.end(), [](const auto &item) {
        if (item)
            return true;
        return false;
    });
}

void ESKF::initialize(const NavStates &states, const Imus &imus) {
    LVINS_CHECK(checkInitialized(states) && checkInitialized(imus), "Input states or IMUs should be initialized!");
    LVINS_CHECK(states.size() == imus.size() && states.front().timestamp == imus.front().timestamp &&
                        states.back().timestamp == imus.back().timestamp,
                "IMUs timestamp should be aligned with states timestamp!");

    // 初始化缓冲区
    state_buffer_ = NavStateBuffer(states.begin(), states.end());
    imu_buffer_   = ImuBuffer(imus.begin(), imus.end());
    update_state_ = states.front();

    // 初始化误差协方差矩阵
    P_                       = MatXf::Zero(dim_, dim_);
    P_.block<3, 3>(O_P, O_P) = std::pow(noise_params_->prior_pos_std, 2) * Mat33f::Identity();
    P_.block<3, 3>(O_V, O_V) = std::pow(noise_params_->prior_vel_std, 2) * Mat33f::Identity();
    P_.block<3, 3>(O_Q, O_Q) = Vec3f(LVINS_FLOAT(std::pow(noise_params_->prior_roll_pitch_std, 2)),
                                     LVINS_FLOAT(std::pow(noise_params_->prior_roll_pitch_std, 2)),
                                     LVINS_FLOAT(std::pow(noise_params_->prior_yaw_std, 2)))
                                       .asDiagonal();
    P_.block<3, 3>(O_BG, O_BG) = std::pow(noise_params_->prior_gyr_bias_std, 2) * Mat33f::Identity();
    P_.block<3, 3>(O_BA, O_BA) = std::pow(noise_params_->prior_acc_bias_std, 2) * Mat33f::Identity();
    P_.block<3, 3>(O_GW, O_GW) = 1e-4 * Mat33f::Identity();
    for (long i = 0; i < static_cast<long>(T_bs_.size()); ++i) {
        P_.block<3, 3>(O_EXT_P + 6 * i, O_EXT_P + 6 * i) =
                std::pow(noise_params_->extrinsic_trans_std, 2) * Mat33f::Identity();
        P_.block<3, 3>(O_EXT_Q + 6 * i, O_EXT_Q + 6 * i) =
                std::pow(noise_params_->extrinsic_rot_std, 2) * Mat33f::Identity();
    }
}

/**
 * @brief 执行循环
 * @details 从缓冲区中循环弹出任务并执行，直到任务时间戳晚于当前状态或缓冲区为空
 * @tparam Task 任务类型
 * @tparam TaskBuffer 任务缓冲区类型
 * @tparam ExecuteFunc 执行函数类型
 * @param state0 上一状态
 * @param state1 当前状态
 * @param current_task 当前任务
 * @param task_buffer 任务缓冲区
 * @param exec_func 执行函数
 */
template<typename Task, typename TaskBuffer, typename ExecuteFunc>
bool executeLoop(const NavState &state0, const NavState &state1, Task &current_task, TaskBuffer &task_buffer,
                 const ExecuteFunc &exec_func) {
    bool exec_flag = false;
    while (true) {
        // 任务在当前状态区间则执行，晚于当前状态则退出
        const auto &[timestamp, task_func] = current_task;
        if (timestamp > state0.timestamp && timestamp <= state1.timestamp) {
            exec_func(task_func);
            exec_flag = true;
        } else if (timestamp > state1.timestamp) {
            break;
        }

        // 尝试弹出下一个任务，失败则退出
        if (!task_buffer.tryPop(current_task))
            break;
    }
    return exec_flag;
}

bool ESKF::propagate(const Imu &imu) {
    LVINS_CHECK(!state_buffer_.empty() && !imu_buffer_.empty(), "ESKF should be initialized first!");

    // 检查IMU时间戳
    if (imu.timestamp <= imu_buffer_.back().timestamp) {
        LVINS_WARN("IMU timestamp jump backward during prediction!");
        return false;
    }

    { // 最新IMU处理
        // 中间量计算
        const auto dt          = LVINS_FLOAT(imu.timestamp - imu_buffer_.back().timestamp) * LVINS_FLOAT(1e-9);
        const auto &last_state = state_buffer_.back();

        // IMU递推
        const Vec3f acc = last_state.T.so3() * (imu.acc - last_state.ba) + g_w_;
        NavState state(last_state);
        state.timestamp       = imu.timestamp;
        state.T.translation() = last_state.T.translation() + last_state.vel * dt + 0.5 * acc * dt * dt;
        state.vel             = last_state.vel + acc * dt;
        state.T.so3()         = last_state.T.so3() * SO3f::exp((imu.gyr - last_state.bg) * dt);
        state.T.normalize();

        // 更新缓冲区
        imu_buffer_.push_back(imu);
        state_buffer_.push_back(std::move(state));
    }

    { // 缓冲区IMU处理
        bool update_flag = false;
        while (state_buffer_.back().timestamp - state_buffer_.front().timestamp > buffer_len_) {
            const auto &imu0   = imu_buffer_[0];
            const auto &imu1   = imu_buffer_[1];
            const auto &state0 = state_buffer_[0];
            const auto &state1 = state_buffer_[1];
            const auto dt      = LVINS_FLOAT(imu1.timestamp - imu0.timestamp) * LVINS_FLOAT(1e-9);

            // ESKF预测
            const Vec3f acc          = state0.T.so3() * (imu1.acc - state0.ba) + g_w_;
            MatXf F                  = MatXf::Identity(dim_, dim_);
            F.block<3, 3>(O_P, O_V)  = Mat33f::Identity() * dt;
            F.block<3, 3>(O_V, O_Q)  = -state1.T.so3().matrix() * SO3f::hat(acc) * dt;
            F.block<3, 3>(O_V, O_BA) = -state1.T.so3().matrix() * dt;
            F.block<3, 3>(O_V, O_GW) = Mat33f::Identity() * dt;
            F.block<3, 3>(O_Q, O_Q)  = SO3f::exp(-(imu1.gyr - state0.bg) * dt).matrix();
            F.block<3, 3>(O_Q, O_BG) = -Mat33f::Identity() * dt;

            P_ = F * P_ * F.transpose() + Q_;

            // 更新
            update_flag |=
                    executeLoop(state0, state1, update_task_, update_task_buffer_, [this](const ObserveFunc &func) {
                        update(func);
                    });

            // 迭代更新
            update_flag |= executeLoop(state0, state1, iterative_update_task_, iterative_update_task_buffer_,
                                       [this](const IterativeObserveFunc &func) {
                                           update(func);
                                       });

            // 弹出缓冲区最老数据，若执行了更新函数，则重新积分并设置更新状态
            imu_buffer_.pop_front();
            state_buffer_.pop_front();
            if (update_flag) {
                repropagate();
                update_state_ = state_buffer_.front();
            }
        }
        return update_flag;
    }
}

void ESKF::addUpdateTask(int64_t timestamp, const ObserveFunc &obs) {
    LVINS_CHECK(obs != nullptr, "Update function should not be nullptr!");
    update_task_buffer_.push({timestamp, obs});
}

void ESKF::addUpdateTask(int64_t timestamp, const IterativeObserveFunc &obs) {
    LVINS_CHECK(obs != nullptr, "Update function should not be nullptr!");
    iterative_update_task_buffer_.push({timestamp, obs});
}

const NavState &ESKF::predictState() const {
    return state_buffer_.back();
}

const NavState &ESKF::updateState() const {
    return update_state_;
}

const std::vector<SE3f> &ESKF::Tbs() const {
    return T_bs_;
}

void ESKF::update(const ObserveFunc &obs) {
    // 计算观测量
    MatXf H, V;
    VecXf r;
    obs(noise_params_, dim_, state_buffer_[1], T_bs_, H, V, r);

    // 计算卡尔曼增益和残差
    const MatXf K           = P_ * H.transpose() * (H * P_ * H.transpose() + V).inverse();
    const VecXf delta_state = K * r;

    // 更新名义状态和误差协方差矩阵
    correctNominal(delta_state);
    std::vector<Vec3f> q_bs_err;
    for (long i = 0; i < static_cast<long>(T_bs_.size()); ++i) {
        q_bs_err.emplace_back(delta_state.segment<3>(O_EXT_Q + 6 * i));
    }
    P_ = (MatXf::Identity(dim_, dim_) - K * H) * P_;
    P_ = projectCovariance(delta_state.segment<3>(O_Q), q_bs_err);
}

void ESKF::update(const IterativeObserveFunc &obs) {
    // 记录初值
    const auto &state     = state_buffer_[1];
    const SO3f q_wb_begin = state.T.so3();
    std::vector<SO3f> q_bs_begin;
    for (auto &T_bs: T_bs_) {
        q_bs_begin.push_back(T_bs.so3());
    }

    // 迭代循环
    MatXf Ht_Vinv_H;
    VecXf Ht_Vinv_r;
    MatXf Pk, Qk;
    for (size_t iter = 0; iter < max_iterations_; ++iter) {
        // 计算观测量
        obs(noise_params_, dim_, state, T_bs_, Ht_Vinv_H, Ht_Vinv_r);

        // 投影此次迭代的误差协方差矩阵
        const Vec3f q_wb_err = (state.T.so3().inverse() * q_wb_begin).log();
        std::vector<Vec3f> q_bs_err;
        for (size_t i = 0; i < T_bs_.size(); ++i) {
            q_bs_err.push_back((T_bs_[i].so3().inverse() * q_bs_begin[i]).log());
        }
        Pk = projectCovariance(q_wb_err, q_bs_err);

        // 计算中间量和残差
        Qk                      = (Pk.inverse() + Ht_Vinv_H).inverse();
        const VecXf delta_state = Qk * Ht_Vinv_r;

        // 更新名义状态
        correctNominal(delta_state);

        // 误差小于阈值则退出
        if (delta_state.norm() < iteration_quit_eqs_) {
            break;
        }
    }

    // 更新误差协方差矩阵
    P_                   = (MatXf::Identity(dim_, dim_) - Qk * Ht_Vinv_H) * Pk;
    const Vec3f q_wb_err = (state.T.so3().inverse() * q_wb_begin).log();
    std::vector<Vec3f> q_bs_err;
    for (size_t i = 0; i < T_bs_.size(); ++i) {
        q_bs_err.push_back((T_bs_[i].so3().inverse() * q_bs_begin[i]).log());
    }
    P_ = projectCovariance(q_wb_err, q_bs_err);
}

void ESKF::correctNominal(const VecXf &delta_state) {
    // 修正状态
    auto &state = state_buffer_[1];
    state.T.translation() += delta_state.segment<3>(O_P);
    state.vel += delta_state.segment<3>(O_V);
    state.T.so3() *= SO3f::exp(delta_state.segment<3>(O_Q));
    state.bg += delta_state.segment<3>(O_BG);
    state.ba += delta_state.segment<3>(O_BA);
    state.T.normalize();

    // 修正重力向量
    g_w_ += delta_state.segment<3>(O_GW);

    // 修正外参
    for (long i = 0; i < static_cast<long>(T_bs_.size()); ++i) {
        T_bs_[i].translation() += delta_state.segment<3>(O_EXT_P + 6 * i);
        T_bs_[i].so3() *= SO3f::exp(delta_state.segment<3>(O_EXT_Q + 6 * i));
        T_bs_[i].normalize();
    }
}

MatXf ESKF::projectCovariance(const Eigen::Ref<const Vec3f> &q_wb_err, const std::vector<Vec3f> &q_bs_err) const {
    // 计算雅可比矩阵
    MatXf J = MatXf::Identity(dim_, dim_);
    J.block<3, 3>(O_Q, O_Q) -= 0.5 * SO3f::hat(q_wb_err);
    for (long i = 0; i < static_cast<long>(q_bs_err.size()); ++i) {
        J.block<3, 3>(O_EXT_Q + 6 * i, O_EXT_Q + 6 * i) -= 0.5 * SO3f::hat(q_bs_err[i]);
    }

    // 投影误差协方差矩阵
    return J * P_ * J.transpose();
}

void ESKF::repropagate() {
    for (size_t i = 1; i < imu_buffer_.size(); ++i) {
        // 中间量计算
        const auto &imu0   = imu_buffer_[i - 1];
        const auto &imu1   = imu_buffer_[i];
        const auto &state0 = state_buffer_[i - 1];
        auto &state1       = state_buffer_[i];
        const auto dt      = LVINS_FLOAT(imu1.timestamp - imu0.timestamp) * LVINS_FLOAT(1e-9);

        // IMU递推
        const Vec3f acc        = state0.T.so3() * (imu1.acc - state0.ba) + g_w_;
        state1.T.translation() = state0.T.translation() + state0.vel * dt + 0.5 * acc * dt * dt;
        state1.vel             = state0.vel + acc * dt;
        state1.T.so3()         = state0.T.so3() * SO3f::exp((imu1.gyr - state0.bg) * dt);
        state1.bg              = state0.bg;
        state1.ba              = state0.ba;
        state1.T.normalize();
    }
}

} // namespace lvins
