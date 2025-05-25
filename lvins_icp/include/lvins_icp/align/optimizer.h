#pragma once

#include "lvins_icp/align/reducer.h"
#include "lvins_icp/align/result.h"
#include "lvins_icp/align/terminate_criteria.h"

namespace lvins::point_cloud_align {

/**
 * @brief 点云配准优化器类
 * @details 该类使用Levenberg-Marquardt算法进行点云配准优化。
 */
class Optimizer {
public:
    /**
     * @brief 构造函数
     * @param max_iterations 最大迭代次数
     * @param max_inner_iterations 最大内部迭代次数（lambda迭代）
     * @param init_lambda 初始lambda值
     * @param lambda_factor lambda调整因子
     */
    Optimizer(size_t max_iterations, size_t max_inner_iterations, Float init_lambda, Float lambda_factor);

    /**
     * @brief 优化点云配准
     * @tparam Factor 点云配准因子类型
     * @param target_nn_searcher 目标点云的最近邻搜索器
     * @param source_point_clouds 源点云集合
     * @param init_T_tb 目标点云到载体的初始相对位姿
     * @param init_T_bs 初始雷达外参集合
     * @param criteria 终止条件
     * @param factors 点云配准因子集合
     * @return 点云配准结果
     */
    template<typename Factor>
    Result optimize(const NearestNeighborSearcher &target_nn_searcher,
                    const std::vector<const PointCloud *> &source_point_clouds, const SE3f &init_T_tb,
                    const std::vector<SE3f> &init_T_bs, const TerminateCriteria &criteria,
                    std::vector<std::vector<Factor>> &factors) const;

private:
    size_t max_iterations_;       ///< 最大迭代次数
    size_t max_inner_iterations_; ///< 最大内部迭代次数（lambda调整）
    Float init_lambda_;           ///< 初始lambda值
    Float lambda_factor_;         ///< lambda调整因子
};

} // namespace lvins::point_cloud_align

#include "lvins_icp/align/optimizer.hpp"
