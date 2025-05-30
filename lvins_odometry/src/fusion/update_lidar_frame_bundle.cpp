#include "lvins_odometry/fusion/update_lidar_frame_bundle.h"
#include "lvins_odometry/fusion/eskf.h"

namespace lvins::eskf {

UpdateLidarFrameBundle::UpdateLidarFrameBundle(int64_t timestamp, LidarFrameBundle::Ptr bundle,
                                               const PointCloudAlignerBase &aligner, NearestNeighborSearcher &mapper,
                                               DrawerBase &drawer, size_t o_T_bs, double lidar_fusion_ratio,
                                               bool estimate_extrinsic)
    : IterativeUpdateTask(timestamp),
      bundle_(std::move(bundle)),
      aligner_(aligner),
      mapper_(mapper),
      drawer_(drawer),
      o_T_bs_(static_cast<long>(o_T_bs)),
      o_ext_(ESKF::O_EXT_P + 6 * o_T_bs_),
      lidar_fusion_ratio_(lidar_fusion_ratio),
      estimate_extrinsic_(estimate_extrinsic) {}

void UpdateLidarFrameBundle::observe(const NoiseParameters & /*noise_params*/, long dim, const NavState &state,
                                     const std::vector<SE3f> &T_bs, MatXd &Ht_Vinv_H, VecXd &Ht_Vinv_r) const {
    // 提取帧束对应的外参
    const auto bundle_T_bs_begin = T_bs.begin() + o_T_bs_;
    const std::vector<SE3f> bundle_T_bs(bundle_T_bs_begin, bundle_T_bs_begin + static_cast<long>(bundle_->size()));

    // 线性化
    std::vector<size_t> num_inliers;
    MatXd H;
    VecXd b;
    aligner_.linearize(mapper_, bundle_->pointClouds(), state.T, bundle_T_bs, estimate_extrinsic_, num_inliers, H, b);

    // 初始化观测矩阵
    Ht_Vinv_H = MatXd::Zero(dim, dim);
    Ht_Vinv_r = VecXd::Zero(dim);

    // 计算有效点数量，如果没有有效点，则直接返回
    size_t total_inliers = 0;
    for (const auto &inliers: num_inliers) {
        total_inliers += inliers;
    }
    if (total_inliers == 0) {
        return;
    }

    // 组装并归一化观测矩阵
    const double pose_scale                     = lidar_fusion_ratio_ / static_cast<double>(total_inliers);
    Ht_Vinv_H.block<6, 6>(ESKF::O_P, ESKF::O_P) = H.block<6, 6>(0, 0) * pose_scale;
    Ht_Vinv_r.segment<6>(ESKF::O_EXT_P)         = b.segment<6>(0) * pose_scale;
    if (estimate_extrinsic_) {
        for (long i = 0; i < static_cast<long>(bundle_->size()); ++i) {
            if (num_inliers[i] == 0) {
                continue;
            }

            const long o_ext_i     = o_ext_ + 6 * i;
            const long o_block_i   = 6 + 6 * i;
            const double ext_scale = lidar_fusion_ratio_ / static_cast<double>(num_inliers[i]);

            Ht_Vinv_H.block<6, 6>(ESKF::O_P, o_ext_i) = H.block<6, 6>(0, o_block_i) * ext_scale;
            Ht_Vinv_H.block<6, 6>(o_ext_i, ESKF::O_P) = H.block<6, 6>(o_block_i, 0) * ext_scale;
            Ht_Vinv_H.block<6, 6>(o_ext_i, o_ext_i)   = H.block<6, 6>(o_block_i, o_block_i) * ext_scale;
            Ht_Vinv_r.segment<6>(o_ext_i)             = b.segment<6>(o_block_i) * ext_scale;
        }
    }
}

void UpdateLidarFrameBundle::finalize(const NavState &state, const std::vector<SE3f> &T_bs) {
    // 提取帧束对应的外参
    const auto bundle_T_bs_begin = T_bs.begin() + o_T_bs_;
    const std::vector<SE3f> bundle_T_bs(bundle_T_bs_begin, bundle_T_bs_begin + static_cast<long>(bundle_->size()));

    // 更新帧束
    auto set_state      = state;
    set_state.timestamp = bundle_->timestamp();
    bundle_->setTbs(bundle_T_bs);
    bundle_->setState(set_state);

    // 更新地图
    for (size_t i = 0; i < bundle_->size(); ++i) {
        const auto &frame = bundle_->frame(i);
        mapper_.insert(frame.pointCloud(), frame.Twf());
    }

    // 绘制帧束
    drawer_.updateLidarFrameBundle(bundle_->timestamp(), bundle_);
}

} // namespace lvins::eskf
