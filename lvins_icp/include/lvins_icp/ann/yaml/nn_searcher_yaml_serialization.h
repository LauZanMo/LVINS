#pragma once

#include "lvins_icp/ann/nearest_neighbor_searcher.h"

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
#include <yaml-cpp/yaml.h>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

using lvins::NearestNeighborSearcher;

namespace YAML {

/**
 * @brief YAML序列化中NearestNeighborSearcher类的实现
 * @details 该类实现了yaml-cpp库中NearestNeighborSearcher类的序列化与反序列化的接口<br/>
 *         用法示例：<br/>
 *         1. 读取（反序列化）：auto nn_searcher = YAML::get<NearestNeighborSearcher>(node, "nn_searcher0");<br/>
 *         2. 写入（序列化）：node["nn_searcher0"] = nn_searcher;<br/>
 *         详细实例可查看NearestNeighborSearcher::loadFromYaml的实现
 */
template<>
struct convert<NearestNeighborSearcher> {
    /**
     * @brief 序列化NearestNeighborSearcher
     * @param nn_searcher NearestNeighborSearcher的实例
     * @return 是否序列化成功
     */
    static Node encode(const NearestNeighborSearcher &nn_searcher);

    /**
     * @brief 反序列化NearestNeighborSearcher
     * @param node NearestNeighborSearcher反序列化所在的YAML节点
     * @param nn_searcher NearestNeighborSearcher的实例
     * @return 是否反序列化成功
     */
    static bool decode(const Node &node, NearestNeighborSearcher &nn_searcher);
};

/**
 * @brief YAML序列化中NearestNeighborSearcher类指针的实现
 * @details 该类实现了yaml-cpp库中NearestNeighborSearcher类指针的序列化与反序列化的接口<br/>
 *          用法示例：<br/>
 *          1. 读取（反序列化）：auto nn_searcher = YAML::get<NearestNeighborSearcher::Ptr>(node, "nn_searcher0");<br/>
 *          2. 写入（序列化）：node["nn_searcher0"] = nn_searcher;<br/>
 *          详细实例可查看NearestNeighborSearcher::loadFromYaml的实现
 */
template<>
struct convert<NearestNeighborSearcher::Ptr> {
    /**
     * @brief 序列化NearestNeighborSearcher类指针
     * @param nn_searcher NearestNeighborSearcher的实例指针
     * @return 是否序列化成功
     */
    static Node encode(const NearestNeighborSearcher::Ptr &nn_searcher);

    /**
     * @brief 反序列化NearestNeighborSearcher类指针
     * @param node NearestNeighborSearcher反序列化所在的YAML节点
     * @param nn_searcher NearestNeighborSearcher的实例指针
     * @return 是否反序列化成功
     */
    static bool decode(const Node &node, NearestNeighborSearcher::Ptr &nn_searcher);
};

namespace internal {

template<typename VoxelContent>
bool encodeIncrementalVoxelMap(const NearestNeighborSearcher &nn_searcher, Node *nn_searcher_node);

} // namespace internal
} // namespace YAML
