#pragma once

#include "lvins_config/setup.h"

#include <cstddef>
#include <vector>

namespace lvins {

/**
 * @brief 单位索引变换
 */
struct IdentityTransform {
    template<typename T>
    T operator()(const T &x) const {
        return x;
    }
};

/**
 * @brief KNN搜索结果类
 * @tparam IndexTransform 索引变换函数
 */
template<typename IndexTransform = IdentityTransform>
class KnnResult {
public:
    /**
     * @brief 构造函数
     * @param indices 索引集合
     * @param sq_dists 距离平方集合
     * @param capacity 容量
     * @param max_sq_dist 指定点与近邻点的最大距离平方
     * @param index_transform 索引变换函数
     */
    KnnResult(std::vector<size_t> &indices, std::vector<Float> &sq_dists, size_t capacity, Float max_sq_dist,
              const IndexTransform &index_transform = IndexTransform());

    /**
     * @brief 默认析构函数
     */
    ~KnnResult() = default;

    /**
     * @brief 获取结果容量
     * @return 容量
     */
    [[nodiscard]] size_t capacity() const;

    /**
     * @brief 获取已找到的结果数量
     * @return 已找到的结果数量
     */
    [[nodiscard]] size_t numFound() const;

    /**
     * @brief 将结果推入到结果容器中（结果容器为对距离平方升序排序）
     * @param index 索引
     * @param sq_dist 距离平方
     */
    void push(size_t index, Float sq_dist);

private:
    std::vector<size_t> &indices_;          ///< 索引集合
    std::vector<Float> &sq_dists_;          ///< 距离平方集合
    size_t capacity_;                       ///< 容量
    size_t num_found_{0};                   ///< 已找到的结果数量
    const IndexTransform &index_transform_; ///< 索引变换函数
};

} // namespace lvins

#include "lvins_icp/ann/knn_result.hpp"
