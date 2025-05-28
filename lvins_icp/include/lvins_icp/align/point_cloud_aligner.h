#pragma once

#include "lvins_icp/align/point_cloud_aligner_base.h"

namespace lvins {

/**
 * @brief 点云配准器类
 * @tparam Factor 点云配准因子类型
 */
template<typename Factor>
class PointCloudAligner final : public PointCloudAlignerBase {
public:
    using FactorSetting = typename Factor::Setting;

    /**
     * @brief 构造函数
     * @param optimizer 点云配准优化器
     * @param criteria 终止条件
     * @param factor_setting 点云配准因子设置
     */
    PointCloudAligner(const Optimizer &optimizer, const TerminateCriteria &criteria,
                      const FactorSetting &factor_setting);

    /**
     * @brief 配准点云
     * @param target_nn_searcher 目标点云的最近邻搜索器
     * @param source_point_clouds 源点云集合
     * @param init_T_tb 目标点云到载体的初始相对位姿
     * @param init_T_bs 初始雷达外参集合
     * @param estimate_extrinsic 是否估计雷达外参
     * @return 配准结果
     */
    Result align(const NearestNeighborSearcher &target_nn_searcher,
                 const std::vector<const PointCloud *> &source_point_clouds, const SE3f &init_T_tb,
                 const std::vector<SE3f> &init_T_bs, bool estimate_extrinsic) override;

    /**
     * @brief 线性化
     * @details 用于向外界（比如ESKF）提供指定状态下的线性化结果
     * @param target_nn_searcher 目标点云的最近邻搜索器
     * @param source_point_clouds 源点云集合
     * @param T_tb 目标点云到载体的相对位姿
     * @param T_bs 雷达外参集合
     * @param estimate_extrinsic 是否估计雷达外参
     * @param H 信息矩阵，顺序: [p_tb, q_tb, p_bs0, q_bs0, ...]
     * @param b 信息向量，顺序: [p_tb, q_tb, p_bs0, q_bs0, ...]
     */
    void linearize(const NearestNeighborSearcher &target_nn_searcher,
                   const std::vector<const PointCloud *> &source_point_clouds, const SE3f &T_tb,
                   const std::vector<SE3f> &T_bs, bool estimate_extrinsic, MatXd &H, VecXd &b) override;

    /**
     * @brief 获取点云配准因子设置
     * @return 点云配准因子设置
     */
    [[nodiscard]] const FactorSetting &factorSetting() const;

    /**
     * @brief 打印点云配准器参数
     * @return 点云配准器参数
     */
    [[nodiscard]] std::string print() const override;

private:
    FactorSetting factor_setting_; ///< 点云配准因子设置
};

} // namespace lvins

#include "lvins_icp/align/point_cloud_aligner.hpp"
