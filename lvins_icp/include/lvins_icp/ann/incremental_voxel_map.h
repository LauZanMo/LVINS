#pragma once

#include "lvins_common/eigen_helper.h"
#include "lvins_icp/ann/nearest_neighbor_searcher.h"

namespace lvins {

/**
 * @brief 体素信息类
 */
struct VoxelInfo {
    /**
     * @brief 构造函数
     * @param coord 体素坐标
     * @param lru 上一次使用计数
     */
    VoxelInfo(Vec3i coord, size_t lru) : coord(std::move(coord)), lru(lru) {}

    Vec3i coord{}; ///< 体素坐标
    size_t lru{};  ///< 上一次使用计数
};

/**
 * @brief 增量体素地图类
 * @tparam VoxelContent 体素内容类型
 */
template<typename VoxelContent>
class IncrementalVoxelMap final : public NearestNeighborSearcher {
public:
    /**
     * @brief 构造函数
     * @param voxel_setting 体素内容设置
     * @param leaf_size 体素尺寸
     * @param lru_horizon LRU删除阈值
     * @param lru_clear_cycle LRU清除周期
     * @param search_offsets 搜索偏移量（1，7，27）
     */
    IncrementalVoxelMap(const typename VoxelContent::Setting &voxel_setting, Float leaf_size, size_t lru_horizon,
                        size_t lru_clear_cycle, size_t search_offsets);

    /**
     * @brief 获取体素设置
     * @return 体素设置
     */
    [[nodiscard]] const typename VoxelContent::Setting &voxelSetting() const;

    /**
     * @brief 获取体素尺寸
     * @return 体素尺寸
     */
    [[nodiscard]] Float leafSize() const;

    /**
     * @brief 获取LRU删除阈值
     * @return LRU删除阈值
     */
    [[nodiscard]] size_t lruHorizon() const;

    /**
     * @brief 获取LRU清除周期
     * @return LRU清除周期
     */
    [[nodiscard]] size_t lruClearCycle() const;

    /**
     * @brief 获取搜索偏移量
     * @return 搜索偏移量
     */
    [[nodiscard]] size_t searchOffsets() const;

    /**
     * @brief 设置搜索偏移量
     * @param search_offsets 搜索偏移量
     */
    void setSearchOffsets(size_t search_offsets);

    /**
     * @brief 将点云经过变换后嵌入到最近邻搜索器中
     * @param point_cloud 点云
     * @param T 变换矩阵
     */
    void insert(const PointCloud &point_cloud, const SE3f &T) override;

    /**
     * @brief 对指定点进行指定数量的最近邻搜索
     * @param point 指定点
     * @param k 指定数量
     * @param k_indices 索引集合
     * @param k_sq_dists 距离平方集合
     * @param max_sq_dist 指定点与近邻点的最大距离平方
     * @return 实际找到的最近邻数量
     */
    [[nodiscard]] size_t knnSearch(const Vec3f &point, size_t k, std::vector<size_t> &k_indices,
                                   std::vector<Float> &k_sq_dists, Float max_sq_dist) const override;

    /**
     * @brief 检测点集是否为空
     * @return
     */
    [[nodiscard]] bool isPointsEmpty() const override;

    /**
     * @brief 获取指定索引的点
     * @param i 指定索引
     * @return 指定索引的点
     */
    [[nodiscard]] const Vec3f &point(size_t i) const override;

    /**
     * @brief 获取指定索引的法向量
     * @param i 指定索引
     * @return 指定索引的法向量
     */
    [[nodiscard]] const Vec3f &normal(size_t i) const override;

    /**
     * @brief 获取指定索引的协方差矩阵
     * @param i 指定索引
     * @return 指定索引的协方差矩阵
     */
    [[nodiscard]] const Mat33f &covariance(size_t i) const override;

    /**
     * @brief 打印最近邻搜索器参数
     * @return 最近邻搜索器参数
     */
    [[nodiscard]] std::string print() const override;

private:
    /**
     * @brief 合成地图索引
     * @param voxel_id 体素索引
     * @param point_id 体素点索引
     * @return 地图索引
     */
    [[nodiscard]] static size_t getIndex(size_t voxel_id, size_t point_id);

    /**
     * @brief 从地图索引中获取体素索引
     * @param map_id 地图索引
     * @return 体素索引
     */
    [[nodiscard]] static size_t voxelId(size_t map_id);

    /**
     * @brief 从地图索引中获取体素点索引
     * @param map_id 地图索引
     * @return 体素点索引
     */
    [[nodiscard]] static size_t pointId(size_t map_id);

    typename VoxelContent::Setting voxel_setting_;                            ///< 体素设置
    std::unordered_map<Vec3i, size_t> voxel_index_map_;                       ///< 体素坐标-体素索引映射
    std::vector<std::shared_ptr<std::pair<VoxelInfo, VoxelContent>>> voxels_; ///< 体素集合

    Float inv_leaf_size_;               ///< 体素尺寸的倒数
    size_t lru_horizon_;                ///< LRU删除阈值
    size_t lru_clear_cycle_;            ///< LRU清除周期
    size_t lru_counter_{0};             ///< LRU计数器
    std::vector<Vec3i> search_offsets_; ///< 体素搜索偏移量

    static_assert(sizeof(size_t) == 8, "size_t must be 64-bit");
    static constexpr int point_id_bits_ = 32;                  ///< 使用前32位用于点ID
    static constexpr int voxel_id_bits_ = 64 - point_id_bits_; ///< 将其余部分用于体素ID
};

} // namespace lvins

#include "lvins_icp/ann/incremental_voxel_map.hpp"
