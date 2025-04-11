#pragma once

#include "lvins_common/eigen_types.h"

namespace lvins::rotation_helper {

/**
 * @brief 欧拉角（ZYX顺序）转换为李代数
 * @tparam Scalar_ 欧拉角元素类型
 * @param euler 欧拉角（ZYX顺序）
 * @return 李代数
 */
template<class Scalar_>
Sophus::SO3<Scalar_> toSO3(const Eigen::Vector3<Scalar_> &euler) {
    return Sophus::SO3<Scalar_>(Eigen::AngleAxis<Scalar_>(euler[2], Eigen::Vector3<Scalar_>::UnitZ()) *
                                Eigen::AngleAxis<Scalar_>(euler[1], Eigen::Vector3<Scalar_>::UnitY()) *
                                Eigen::AngleAxis<Scalar_>(euler[0], Eigen::Vector3<Scalar_>::UnitX()));
}

/**
 * @brief 李代数转换为欧拉角（ZYX顺序）
 * @tparam Scalar_ 欧拉角元素类型
 * @param so3 李代数
 * @return 欧拉角（ZYX顺序）
 */
template<class Scalar_>
Eigen::Vector3<Scalar_> toEuler(const Sophus::SO3<Scalar_> &so3) {
    const auto mat = so3.matrix();
    return Eigen::Vector3<Scalar_>(atan2(mat(2, 1), mat(2, 2)),
                                   atan(-mat(2, 0) / sqrt(mat(2, 1) * mat(2, 1) + mat(2, 2) * mat(2, 2))),
                                   atan2(mat(1, 0), mat(0, 0)));
}

} // namespace lvins::rotation_helper
