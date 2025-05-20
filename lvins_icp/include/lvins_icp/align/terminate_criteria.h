#pragma once

#include "lvins_common/eigen_types.h"

namespace lvins::point_cloud_align {

/**
 * @brief 点云配准终止条件类
 */
class TerminateCriteria {
public:
    /**
     * @brief 构造函数
     * @param trans_eps 平移收敛阈值
     * @param rot_eps 旋转收敛阈值
     */
    TerminateCriteria(Float trans_eps, Float rot_eps);

    /**
     * @brief 默认析构函数
     */
    ~TerminateCriteria() = default;

    /**
     * @brief 判断是否收敛
     * @param delta 迭代增量
     * @return 是否收敛
     */
    [[nodiscard]] bool isConverged(const VecXf &delta) const;

private:
    Float trans_eps_; ///< 平移收敛阈值
    Float rot_eps_;   ///< 旋转收敛阈值
};

} // namespace lvins::point_cloud_align
