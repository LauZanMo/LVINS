#include "lvins_odometry/fusion/update_lidar.h"
#include "lvins_odometry/fusion/eskf.h"

namespace lvins::eskf {

UpdateLidar::UpdateLidar(int64_t timestamp, LidarFrameBundle::Ptr bundle, LidarRig &lidar_rig,
                         const PointCloudAligner &aligner, NearestNeighborSearch &mapper, DrawerBase &drawer,
                         size_t o_T_bs, bool estimate_extrinsic)
    : UpdateTask(timestamp),
      bundle_(std::move(bundle)),
      lidar_rig_(lidar_rig),
      aligner_(aligner),
      mapper_(mapper),
      drawer_(drawer),
      o_T_bs_(static_cast<long>(o_T_bs)),
      o_ext_(ESKF::O_EXT_P + 6 * o_T_bs_),
      estimate_extrinsic_(estimate_extrinsic && bundle_->size() > 1),
      observe_dim_(estimate_extrinsic_ ? 6 + 6 * static_cast<long>(bundle_->size()) : 6) {}

void UpdateLidar::observe(const NoiseParameters &noise_params, long dim, const NavState &state,
                          const std::vector<SE3f> &T_bs, MatXd &H, MatXd &V, VecXd &r) const {
    // 提取帧束对应的外参
    const auto bundle_T_bs_begin = T_bs.begin() + o_T_bs_;
    const std::vector<SE3f> bundle_T_bs(bundle_T_bs_begin, bundle_T_bs_begin + static_cast<long>(bundle_->size()));

    // 点云配准
    const auto result = aligner_.align(mapper_.getSearch(), bundle_->pointClouds(), noise_params, state.T, bundle_T_bs,
                                       estimate_extrinsic_);

    // 初始化观测矩阵
    H = MatXd::Zero(observe_dim_, dim);
    V = MatXd::Zero(observe_dim_, observe_dim_);
    r = VecXd::Zero(observe_dim_);

    // 组装并归一化观测矩阵
    H.block<6, 6>(0, ESKF::O_P).setIdentity();
    V.block<3, 3>(0, 0) = std::pow(noise_params.odom_trans_std, 2) * Mat33d::Identity();
    V.block<3, 3>(3, 3) = std::pow(noise_params.odom_rot_std, 2) * Mat33d::Identity();
    r.segment<3>(0)     = (result.T_tb.translation() - state.T.translation()).cast<double>();
    r.segment<3>(3)     = (state.T.so3().inverse() * result.T_tb.so3()).log().cast<double>();
    if (estimate_extrinsic_) {
        for (long i = 0; i < static_cast<long>(bundle_->size()); ++i) {
            const long o_ext_i = o_ext_ + 6 * i;
            const long o_obs_i = 6 + 6 * i;

            H.block<6, 6>(o_obs_i, o_ext_i).setIdentity();
            V.block<3, 3>(o_obs_i, o_obs_i)         = std::pow(noise_params.ext_rot_std, 2) * Mat33d::Identity();
            V.block<3, 3>(o_obs_i + 3, o_obs_i + 3) = std::pow(noise_params.ext_trans_std, 2) * Mat33d::Identity();
            r.segment<3>(o_obs_i)     = (result.T_bs[i].translation() - bundle_T_bs[i].translation()).cast<double>();
            r.segment<3>(o_obs_i + 3) = (bundle_T_bs[i].so3().inverse() * result.T_bs[i].so3()).log().cast<double>();
        }
    }
}

void UpdateLidar::finalize(const NavState &state, const std::vector<SE3f> &T_bs) {
    // 提取帧束对应的外参
    const auto bundle_T_bs_begin = T_bs.begin() + o_T_bs_;
    const std::vector<SE3f> bundle_T_bs(bundle_T_bs_begin, bundle_T_bs_begin + static_cast<long>(bundle_->size()));

    // 更新帧束
    bundle_->setState(state);
    bundle_->setTbs(bundle_T_bs);

    // 更新雷达组外参
    lidar_rig_.setTbs(bundle_T_bs);

    // 更新地图
    for (size_t i = 0; i < bundle_->size(); ++i) {
        const auto &frame = bundle_->frame(i);
        mapper_.insert(*frame->pointCloud(), frame->Twf());
    }

    // 绘制导航状态和帧束
    drawer_.updateNavState(state.timestamp, state);
    drawer_.updateLidarFrameBundle(bundle_->timestamp(), bundle_);
}

} // namespace lvins::eskf
