#pragma once

#include "lvins_common/eigen_types.h"
#include "lvins_icp/ann/nearest_neighbor_searcher.h"
#include "lvins_icp/point_cloud.h"

#include <tuple>

namespace lvins::point_cloud_align {

/**
 * @brief 归约器类
 * @details 内部使用TBB做并行化计算
 */
class Reducer {
public:
    /**
     * @brief 线性化点云配准因子
     * @tparam Factor 点云配准因子类型
     * @param target_nn_searcher 目标点云的最近邻搜索器
     * @param source 源点云
     * @param T_tb 目标点云到载体的相对位姿
     * @param T_bs 雷达外参
     * @param factors 点云配准因子集合
     * @return 线性化结果（H, b, e）
     */
    template<typename Factor>
    static std::tuple<MatXf, VecXf, Float> linearize(const NearestNeighborSearcher &target_nn_searcher,
                                                     const PointCloud &source, const SE3f &T_tb, const SE3f &T_bs,
                                                     std::vector<Factor> &factors);

    /**
     * @brief 计算点云配准因子的误差
     * @tparam Factor 点云配准因子类型
     * @param target_nn_searcher 目标点云的最近邻搜索器
     * @param source 源点云
     * @param T_tb 目标点云到载体的相对位姿
     * @param T_bs 雷达外参
     * @param factors 点云配准因子集合
     * @return 误差值
     */
    template<typename Factor>
    static Float error(const NearestNeighborSearcher &target_nn_searcher, const PointCloud &source, const SE3f &T_tb,
                       const SE3f &T_bs, std::vector<Factor> &factors);
};

} // namespace lvins::point_cloud_align

#include "lvins_icp/align/reducer.hpp"
