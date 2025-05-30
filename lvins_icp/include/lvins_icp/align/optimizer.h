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
    Optimizer(size_t max_iterations, size_t max_inner_iterations, double init_lambda, double lambda_factor);

    /**
     * @brief 优化点云配准
     * @tparam Factor 点云配准因子类型
     * @param target_nn_searcher 目标点云的最近邻搜索器
     * @param source_point_clouds 源点云集合
     * @param init_T_tb 目标点云到载体的初始相对位姿
     * @param init_T_bs 初始雷达外参集合
     * @param estimate_extrinsic 是否估计雷达外参
     * @param criteria 终止条件
     * @param factors 点云配准因子集合
     * @return 点云配准结果
     */
    template<typename Factor>
    Result optimize(const NearestNeighborSearcher &target_nn_searcher,
                    const std::vector<const PointCloud *> &source_point_clouds, const SE3f &init_T_tb,
                    const std::vector<SE3f> &init_T_bs, bool estimate_extrinsic, const TerminateCriteria &criteria,
                    std::vector<std::vector<Factor>> &factors) const;

    /**
     * @brief 线性化
     * @tparam Factor 点云配准因子类型
     * @param target_nn_searcher 目标点云的最近邻搜索器
     * @param source_point_clouds 源点云集合
     * @param T_tb 目标点云到载体的相对位姿
     * @param T_bs 雷达外参集合
     * @param estimate_extrinsic 是否估计雷达外参
     * @param H 信息矩阵，顺序: [p_tb, q_tb, p_bs0, q_bs0, ...]
     * @param b 信息向量，顺序: [p_tb, q_tb, p_bs0, q_bs0, ...]
     * @param factors 点云配准因子集合
     */
    template<typename Factor>
    void linearize(const NearestNeighborSearcher &target_nn_searcher,
                   const std::vector<const PointCloud *> &source_point_clouds, const SE3f &T_tb,
                   const std::vector<SE3f> &T_bs, bool estimate_extrinsic, MatXd &H, VecXd &b,
                   std::vector<std::vector<Factor>> &factors) const;

    /**
     * @brief 获取最大迭代次数
     * @return 最大迭代次数
     */
    [[nodiscard]] size_t maxIterations() const;

    /**
     * @brief 获取最大内部迭代次数（lambda调整）
     * @return 最大内部迭代次数
     */
    [[nodiscard]] size_t maxInnerIterations() const;

    /**
     * @brief 获取初始lambda值
     * @return 初始lambda值
     */
    [[nodiscard]] double initLambda() const;

    /**
     * @brief 获取lambda调整因子
     * @return lambda调整因子
     */
    [[nodiscard]] double lambdaFactor() const;

    /**
     * @brief 打印点云配准优化器参数
     * @return 点云配准优化器参数
     */
    [[nodiscard]] std::string print() const;

private:
    size_t max_iterations_;       ///< 最大迭代次数
    size_t max_inner_iterations_; ///< 最大内部迭代次数（lambda调整）
    double init_lambda_;          ///< 初始lambda值
    double lambda_factor_;        ///< lambda调整因子
};

} // namespace lvins::point_cloud_align

#include "lvins_icp/align/optimizer.hpp"
