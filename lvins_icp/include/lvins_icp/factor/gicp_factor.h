#pragma once

#include "lvins_common/eigen_types.h"
#include "lvins_icp/ann/nearest_neighbor_searcher.h"
#include "lvins_icp/point_cloud.h"

namespace lvins {

/**
 * @brief GICP因子类
 */
class GICPFactor {
public:
    /**
     * @brief GICP因子设置
     */
    struct Setting {
        /**
         * @brief 构造函数
         * @param max_search_sq_dist 最大搜索平方距离
         */
        explicit Setting(float max_search_sq_dist) : max_search_sq_dist(max_search_sq_dist) {}

        /**
         * @brief 打印GICP因子设置参数
         * @return GICP因子设置参数
         */
        [[nodiscard]] std::string print() const;

        float max_search_sq_dist; ///< 最大搜索平方距离
    };

    /**
     * @brief 构造函数
     * @param setting GICP因子设置
     */
    explicit GICPFactor(const Setting *setting);

    /**
     * @brief 默认析构函数
     */
    ~GICPFactor() = default;

    /**
     * @brief 线性化点云配准因子
     * @param target_nn_searcher 目标点云的最近邻搜索器
     * @param source 源点云
     * @param T_tb 目标点云到载体的相对位姿
     * @param T_bs 雷达外参
     * @param source_index 源点索引
     * @param H 线性化信息矩阵
     * @param b 线性化信息向量
     * @param e 误差值
     * @return 是否为可用点
     */
    [[nodiscard]] bool linearize(const NearestNeighborSearcher &target_nn_searcher, const PointCloud &source,
                                 const SE3f &T_tb, const SE3f &T_bs, size_t source_index, MatXd &H, VecXd &b,
                                 double &e);

    /**
     * @brief 计算点云配准因子的误差
     * @param target_nn_searcher 目标点云的最近邻搜索器
     * @param source 源点云
     * @param T_tb 目标点云到载体的相对位姿
     * @param T_bs 雷达外参
     * @return 误差
     */
    [[nodiscard]] double error(const NearestNeighborSearcher &target_nn_searcher, const PointCloud &source,
                               const SE3f &T_tb, const SE3f &T_bs) const;

    /**
     * @brief 获取是否为可用点
     * @return 是否为可用点
     */
    [[nodiscard]] bool isInlier() const;

private:
    size_t target_index_{std::numeric_limits<size_t>::max()}; ///< 目标点索引
    size_t source_index_{std::numeric_limits<size_t>::max()}; ///< 源点索引
    Mat33d point_cov_{Mat33d::Zero()};                        ///< 协方差矩阵

    const Setting *setting_; ///< 设置
};

} // namespace lvins
