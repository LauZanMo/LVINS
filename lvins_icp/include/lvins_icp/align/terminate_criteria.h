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
    TerminateCriteria(double trans_eps, double rot_eps);

    /**
     * @brief 默认析构函数
     */
    ~TerminateCriteria() = default;

    /**
     * @brief 判断是否收敛
     * @param delta 迭代增量
     * @return 是否收敛
     */
    [[nodiscard]] bool isConverged(const VecXd &delta) const;

    /**
     * @brief 获取平移收敛阈值
     * @return 平移收敛阈值
     */
    [[nodiscard]] double transEpsilon() const;

    /**
     * @brief 获取旋转收敛阈值
     * @return 旋转收敛阈值
     */
    [[nodiscard]] double rotEpsilon() const;

    /**
     * @brief 打印点云配准终止条件参数
     * @return 点云配准终止条件参数
     */
    [[nodiscard]] std::string print() const;

private:
    double trans_eps_; ///< 平移收敛阈值
    double rot_eps_;   ///< 旋转收敛阈值
};

} // namespace lvins::point_cloud_align
