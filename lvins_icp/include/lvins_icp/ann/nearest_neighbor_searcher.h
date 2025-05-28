#pragma once

#include "lvins_common/eigen_types.h"
#include "lvins_common/yaml/yaml_serialization.h"
#include "lvins_icp/point_cloud.h"

namespace lvins {

/**
 * @brief 最近邻搜索器类接口（抽象类）
 * @details 该类为最近邻搜索器的接口类，所有最近邻搜索器都通过YAML配置节点实现动态加载
 */
class NearestNeighborSearcher {
public:
    using Ptr = std::shared_ptr<NearestNeighborSearcher>;

    /**
     * @brief 默认构造函数
     */
    NearestNeighborSearcher() = default;

    /**
     * @brief 默认析构函数
     */
    virtual ~NearestNeighborSearcher() = default;

    /**
     * @brief 从YAML节点中加载最近邻搜索器
     * @param config YAML节点
     * @return 加载最近邻搜索器
     * @warning 如果加载失败，则返回空指针
     */
    static Ptr loadFromYaml(const YAML::Node &config);

    /**
     * @brief 将最近邻搜索器参数写入YAML节点
     * @return YAML节点
     */
    [[nodiscard]] YAML::Node writeToYaml() const;

    /**
     * @brief 将点云经过变换后嵌入到最近邻搜索器中
     * @param point_cloud 点云
     * @param T 变换矩阵
     */
    virtual void insert(const PointCloud &point_cloud, const SE3f &T) = 0;

    /**
     * @brief 清空最近邻搜索器
     */
    virtual void clear() = 0;

    /**
     * @brief 对指定点进行指定数量的最近邻搜索
     * @param point 指定点
     * @param k 指定数量
     * @param k_indices 索引集合
     * @param k_sq_dists 距离平方集合
     * @param max_sq_dist 指定点与近邻点的最大距离平方
     * @return 实际找到的最近邻数量
     */
    [[nodiscard]] virtual size_t knnSearch(const Eigen::Vector3f &point, size_t k, std::vector<size_t> &k_indices,
                                           std::vector<float> &k_sq_dists, float max_sq_dist) const = 0;

    /**
     * @brief 检测最近邻搜索器是否为空
     * @return 最近邻搜索器是否为空
     */
    [[nodiscard]] virtual bool empty() const = 0;

    /**
     * @brief 获取最近邻搜索器大小
     * @return 最近邻搜索器大小
     */
    [[nodiscard]] virtual size_t size() const = 0;

    /**
     * @brief 获取指定索引的点
     * @param i 指定索引
     * @return 指定索引的点
     */
    [[nodiscard]] virtual const Eigen::Vector3f &point(size_t i) const = 0;

    /**
     * @brief 获取指定索引的法向量
     * @param i 指定索引
     * @return 指定索引的法向量
     */
    [[nodiscard]] virtual const Eigen::Vector3f &normal(size_t i) const = 0;

    /**
     * @brief 获取指定索引的协方差矩阵
     * @param i 指定索引
     * @return 指定索引的协方差矩阵
     */
    [[nodiscard]] virtual const Eigen::Matrix3f &covariance(size_t i) const = 0;

    /**
     * @brief 打印最近邻搜索器参数
     * @return 最近邻搜索器参数
     */
    [[nodiscard]] virtual std::string print() const = 0;
};

} // namespace lvins

/**
 * @brief 最近邻搜索器格式化器
 * @tparam T 最近邻搜索器派生类型
 * @tparam Char 格式化字符类型
 */
template<typename T, typename Char>
struct LVINS_FORMATTER<T, Char, std::enable_if_t<std::is_convertible_v<T *, lvins::NearestNeighborSearcher *>>> {
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
     * @param nn_search 最近邻搜索器
     * @param ctx 输出的格式化文本
     * @return 输出格式化文本的尾部迭代器
     */
    static auto format(const lvins::NearestNeighborSearcher &nn_search, LVINS_FORMAT_CONTEXT &ctx) {
        return LVINS_FORMAT_TO(ctx.out(), "{}", nn_search.print());
    }
};
