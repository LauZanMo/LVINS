#pragma once

#include "lvins_common/yaml/yaml_serialization.h"
#include "lvins_icp/point_cloud.h"

#include <gtsam_points/ann/ivox.hpp>

namespace lvins {

/**
 * @brief 最近邻搜索器
 */
class NearestNeighborSearch {
public:
    using Ptr    = std::shared_ptr<NearestNeighborSearch>;
    using Search = gtsam_points::iVox;

    /**
     * @brief 构造函数
     * @param config YAML配置节点
     */
    explicit NearestNeighborSearch(const YAML::Node &config);

    /**
     * @brief 默认析构函数
     */
    ~NearestNeighborSearch() = default;

    /**
     * @brief 将最近邻搜索器参数写入YAML节点
     * @return YAML节点
     */
    [[nodiscard]] YAML::Node writeToYaml() const;

    /**
     * @brief 重置最近邻搜索器
     */
    void reset();

    /**
     * @brief 获取最近邻搜索数据结构
     * @return 最近邻搜索器数据结构
     */
    [[nodiscard]] Search::ConstPtr getSearch() const;

    /**
     * @brief 插入点云数据到最近邻搜索器
     * @param point_cloud 点云数据
     * @param T 点云的变换矩阵
     */
    void insert(const PointCloud &point_cloud, const SE3f &T);

    /**
     * @brief 打印最近邻搜索器参数
     * @return 最近邻搜索器参数
     */
    [[nodiscard]] std::string print() const;

private:
    Search::Ptr nn_search_; ///< 最近邻搜索器数据结构
    int lru_horizon_;       ///< LRU缓存周期
    int lru_clear_cycle_;   ///< LRU缓存清理周期
    int search_offset_;     ///< 搜索偏移量
};

} // namespace lvins

/**
 * @brief 最近邻搜索器格式化器
 */
template<>
struct LVINS_FORMATTER<lvins::NearestNeighborSearch> {
    /**
     * @brief 从文本中解析格式化字符
     * @param ctx 文本
     * @return 格式化字符尾部迭代器
     */
    static constexpr auto parse(const LVINS_FORMAT_PARSE_CONTEXT &ctx) {
        return ctx.begin();
    }

    /**
     * @brief 格式化
     * @param search 最近邻搜索器
     * @param ctx 输出的格式化文本
     * @return 输出格式化文本的尾部迭代器
     */
    static auto format(const lvins::NearestNeighborSearch &search, LVINS_FORMAT_CONTEXT &ctx) {
        return LVINS_FORMAT_TO(ctx.out(), "{}", search.print());
    }
};
