#pragma once

#include "lvins_icp/align/point_cloud_aligner_base.h"
#include "lvins_odometry/base/lidar_frame_bundle.h"
#include "lvins_odometry/drawer_base.h"
#include "lvins_odometry/fusion/iterative_update_task.h"

namespace lvins::eskf {

/**
 * @brief 雷达帧束更新任务类
 */
class UpdateLidarFrameBundle final : public IterativeUpdateTask {
public:
    /**
     * @brief 构造函数
     * @param timestamp 时间戳
     * @param bundle 待更新的雷达帧束
     * @param aligner 点云配准器
     * @param mapper 地图
     * @param drawer 绘制器
     * @param o_T_bs 首个雷达外参在外参集合中的位置
     * @param lidar_fusion_ratio 雷达融合比例
     * @param estimate_extrinsic 是否估计外参
     */
    UpdateLidarFrameBundle(int64_t timestamp, LidarFrameBundle::Ptr bundle, const PointCloudAlignerBase &aligner,
                           NearestNeighborSearcher &mapper, DrawerBase &drawer, size_t o_T_bs,
                           double lidar_fusion_ratio, bool estimate_extrinsic);

    /**
     * @brief 观测函数
     * @param noise_params 噪声参数
     * @param dim ESKF状态维度
     * @param state 预测当前状态
     * @param T_bs 预测外参集合
     * @param Ht_Vinv_H IESKF计算中间项，也是非线性优化的Hessian矩阵
     * @param Ht_Vinv_r IESKF计算中间项，也是非线性优化的b向量
     */
    void observe(const NoiseParameters &noise_params, long dim, const NavState &state, const std::vector<SE3f> &T_bs,
                 MatXd &Ht_Vinv_H, VecXd &Ht_Vinv_r) const override;

    /**
     * @brief 结尾处理函数
     * @param state 融合状态
     * @param T_bs 融合外参集合
     */
    void finalize(const NavState &state, const std::vector<SE3f> &T_bs) override;

private:
    LidarFrameBundle::Ptr bundle_;         ///< 待更新的雷达帧束
    const PointCloudAlignerBase &aligner_; ///< 点云配准器
    NearestNeighborSearcher &mapper_;      ///< 地图
    DrawerBase &drawer_;                   ///< 绘制器

    long o_T_bs_;               ///< 首个雷达外参在外参集合中的位置
    long o_ext_;                ///< 首个雷达外参在状态向量中的起始位置
    double lidar_fusion_ratio_; ///< 雷达融合比例
    bool estimate_extrinsic_;   ///< 是否估计外参
};

} // namespace lvins::eskf
