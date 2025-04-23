#pragma once

#include <Eigen/Core>
#include <functional>
#include <type_traits>

namespace std {

/**
 * @brief eigen矩阵的哈希特化
 * @tparam Scalar 矩阵元素类型
 * @tparam Rows 矩阵行数
 * @tparam Cols 矩阵列数
 * @tparam Options 矩阵选项
 * @tparam MaxRows 矩阵最大行数
 * @tparam MaxCols 矩阵最大列数
 * @note https://wjngkoh.wordpress.com/2015/03/04/c-hash-function-for-eigen-matrix-and-vector/
 */
template<typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
struct hash<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>> {
    size_t operator()(const Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> &matrix) const {
        size_t seed = 0;
        for (size_t i = 0; i < static_cast<size_t>(matrix.size()); ++i) {
            Scalar elem = *(matrix.data() + i);
            seed ^= std::hash<Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

} // namespace std

/**
 * @brief eigen矩阵向下取整
 * @tparam Derived 矩阵类型
 * @param matrix 输入矩阵
 * @return 向下取整后的eigen矩阵
 * @note https://stackoverflow.com/questions/824118/why-is-floor-so-slow
 */
template<typename Derived>
auto fastFloor(const Eigen::MatrixBase<Derived> &matrix)
        -> Eigen::Matrix<int, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> {
    using Scalar = typename Derived::Scalar;
    static_assert(std::is_floating_point_v<Scalar>, "fastFloor only supports floating point types");
    Eigen::Matrix<int, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> base = matrix.template cast<int>();
    return base - (matrix.array() < base.array().template cast<Scalar>()).matrix().template cast<int>();
}
