#pragma once

#include "lvins_common/eigen_types.h"

#include <vector>

namespace lvins::point_cloud_align {

/**
 * @brief 点云配准结果类
 */
struct Result {
    /**
     * @brief 构造函数
     * @param init_T_tb 目标点云到载体的相对位姿
     * @param init_T_bs 雷达外参集合
     */
    Result(const SE3f &init_T_tb, const std::vector<SE3f> &init_T_bs);

    /**
     * @brief 默认析构函数
     */
    ~Result() = default;

    SE3f T_tb;              ///< 目标点云到载体的相对位姿
    std::vector<SE3f> T_bs; ///< 雷达外参集合

    bool converged{false}; ///< 是否收敛
    size_t iterations{0};  ///< 迭代次数
    size_t num_inliers{0}; ///< 采用点数量

    MatXd H;       ///< 信息矩阵，顺序: [p_tb, q_tb, p_bs0, q_bs0, ...]
    VecXd b;       ///< 信息向量，顺序: [p_tb, q_tb, p_bs0, q_bs0, ...]
    double e{0.0}; ///< 最终误差
};

} // namespace lvins::point_cloud_align