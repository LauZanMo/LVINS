#pragma once

#include "lvins_icp/align/point_cloud_aligner.h"
#include "lvins_lidar/lidar_rig.h"
#include "lvins_odometry/base/lidar_frame_bundle.h"
#include "lvins_odometry/drawer_base.h"
#include "lvins_odometry/fusion/update_task.h"

namespace lvins::eskf {

/**
 * @brief 雷达帧束更新任务类
 */
class UpdateLidar final : public UpdateTask {
public:
    /**
     * @brief 构造函数
     * @param timestamp 时间戳
     * @param bundle 待更新的雷达帧束
     * @param lidar_rig 雷达组
     * @param aligner 点云配准器
     * @param mapper 地图
     * @param drawer 绘制器
     * @param o_T_bs 首个雷达外参在外参集合中的位置
     * @param estimate_extrinsic 是否估计外参
     */
    UpdateLidar(int64_t timestamp, LidarFrameBundle::Ptr bundle, LidarRig &lidar_rig, const PointCloudAligner &aligner,
                NearestNeighborSearch &mapper, DrawerBase &drawer, size_t o_T_bs, bool estimate_extrinsic);

    /**
     * @brief 观测函数
     * @param noise_params 噪声参数
     * @param dim ESKF状态维度
     * @param state 预测当前状态
     * @param T_bs 预测外参集合
     * @param H 观测矩阵
     * @param V 观测噪声协方差矩阵
     * @param r 观测残差向量
     */
    void observe(const NoiseParameters &noise_params, long dim, const NavState &state, const std::vector<SE3f> &T_bs,
                 MatXd &H, MatXd &V, VecXd &r) const override;

    /**
     * @brief 结尾处理函数
     * @details 结尾处理包括更新雷达帧束，更新地图，更新雷达组外参和绘制雷达帧束
     * @param state 融合状态
     * @param T_bs 融合外参集合
     */
    void finalize(const NavState &state, const std::vector<SE3f> &T_bs) override;

private:
    LidarFrameBundle::Ptr bundle_;     ///< 待更新的雷达帧束
    LidarRig &lidar_rig_;              ///< 雷达组
    const PointCloudAligner &aligner_; ///< 点云配准器
    NearestNeighborSearch &mapper_;    ///< 地图
    DrawerBase &drawer_;               ///< 绘制器

    long o_T_bs_;             ///< 首个雷达外参在外参集合中的位置
    long o_ext_;              ///< 首个雷达外参在状态向量中的起始位置
    bool estimate_extrinsic_; ///< 是否估计外参
    long observe_dim_;        ///< 观测维度
};

} // namespace lvins::eskf
