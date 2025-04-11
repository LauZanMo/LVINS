#pragma once

#include <Eigen/Core>
#include <functional>

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
