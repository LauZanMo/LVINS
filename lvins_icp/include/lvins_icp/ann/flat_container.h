#pragma once

#include "lvins_common/eigen_types.h"
#include "lvins_icp/ann/knn_result.h"
#include "lvins_icp/point_cloud.h"

namespace lvins {

/**
 * @brief 平凡容器类
 * @details 平凡容器类用于存储点云数据，包含点、法向量和协方差矩阵，最近邻搜索方法为暴力搜索
 */
class FlatContainer {
public:
    /**
     * @brief 平凡容器设置
     */
    struct Setting {
        /**
         * @brief 构造函数
         * @param min_sq_dist_in_cell 容器中最小的点间距离平方
         * @param max_num_points_in_cell 容器中最大点数
         */
        Setting(Float min_sq_dist_in_cell, size_t max_num_points_in_cell)
            : min_sq_dist_in_cell(min_sq_dist_in_cell), max_num_points_in_cell(max_num_points_in_cell) {}

        /**
         * @brief 打印平凡容器设置参数
         * @return 平凡容器设置参数
         */
        [[nodiscard]] std::string print() const;

        Float min_sq_dist_in_cell     = 0.1 * 0.1; ///< 容器中最小的点间距离平方
        size_t max_num_points_in_cell = 15;        ///< 容器中最大点数
    };

    /**
     * @brief 默认构造函数
     */
    FlatContainer() = default;

    /**
     * @brief 默认析构函数
     */
    ~FlatContainer() = default;

    /**
     * @brief 获取容器点集数量
     * @return 点集数量
     */
    [[nodiscard]] size_t size() const;

    /**
     * @brief 获取容器中的点集
     * @return 点集
     */
    [[nodiscard]] const std::vector<Vec3f> &points() const;

    /**
     * @brief 获取容器中的法向量集合
     * @return 法向量集合
     */
    [[nodiscard]] const std::vector<Vec3f> &normals() const;

    /**
     * @brief 获取容器中的协方差矩阵集合
     * @return 协方差矩阵集合
     */
    [[nodiscard]] const std::vector<Mat33f> &covariances() const;

    /**
     * @brief 将点云中指定索引的点经过变换后加入容器
     * @param setting 容器设置
     * @param point_cloud 点云
     * @param i 指定索引
     * @param T 变换矩阵
     */
    void add(const Setting &setting, const PointCloud &point_cloud, size_t i, const SE3f &T);

    /**
     * @brief 对指定点进行最近邻搜索
     * @tparam Result 搜索结果类型
     * @param point 指定点
     * @param result 搜索结果
     */
    template<typename Result>
    void knnSearch(const Vec3f &point, Result &result) const;

    /**
     * @brief 对容器进行最终化处理（无操作）
     */
    static void finalize() {}

private:
    std::vector<Vec3f> points_;       ///< 点集
    std::vector<Vec3f> normals_;      ///< 法向量集合
    std::vector<Mat33f> covariances_; ///< 协方差矩阵集合
};

} // namespace lvins

#include "lvins_icp/ann/flat_container.hpp"
