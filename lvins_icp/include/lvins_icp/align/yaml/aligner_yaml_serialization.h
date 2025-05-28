#pragma once

#include "lvins_icp/align/point_cloud_aligner_base.h"

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
#include <yaml-cpp/yaml.h>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

using lvins::PointCloudAlignerBase;

namespace YAML {

/**
 * @brief YAML序列化中PointCloudAlignerBase类的实现
 * @details 该类实现了yaml-cpp库中PointCloudAlignerBase类的序列化与反序列化的接口<br/>
 *         用法示例：<br/>
 *         1. 读取（反序列化）：auto aligner = YAML::get<PointCloudAlignerBase>(node, "aligner0");<br/>
 *         2. 写入（序列化）：node["aligner0"] = aligner;<br/>
 *         详细实例可查看PointCloudAlignerBase::loadFromYaml的实现
 */
template<>
struct convert<PointCloudAlignerBase> {
    /**
     * @brief 序列化PointCloudAlignerBase
     * @param aligner PointCloudAlignerBase的实例
     * @return 是否序列化成功
     */
    static Node encode(const PointCloudAlignerBase &aligner);

    /**
     * @brief 反序列化PointCloudAlignerBase
     * @param node PointCloudAlignerBase反序列化所在的YAML节点
     * @param aligner PointCloudAlignerBase的实例
     * @return 是否反序列化成功
     */
    static bool decode(const Node &node, PointCloudAlignerBase &aligner);
};

/**
 * @brief YAML序列化中PointCloudAlignerBase类指针的实现
 * @details 该类实现了yaml-cpp库中PointCloudAlignerBase类指针的序列化与反序列化的接口<br/>
 *          用法示例：<br/>
 *          1. 读取（反序列化）：auto aligner = YAML::get<PointCloudAlignerBase::Ptr>(node, "aligner0");<br/>
 *          2. 写入（序列化）：node["aligner0"] = aligner;<br/>
 *          详细实例可查看PointCloudAlignerBase::loadFromYaml的实现
 */
template<>
struct convert<PointCloudAlignerBase::Ptr> {
    /**
     * @brief 序列化PointCloudAlignerBase类指针
     * @param aligner PointCloudAlignerBase的实例指针
     * @return 是否序列化成功
     */
    static Node encode(const PointCloudAlignerBase::Ptr &aligner);

    /**
     * @brief 反序列化PointCloudAlignerBase类指针
     * @param node PointCloudAlignerBase反序列化所在的YAML节点
     * @param aligner PointCloudAlignerBase的实例指针
     * @return 是否反序列化成功
     */
    static bool decode(const Node &node, PointCloudAlignerBase::Ptr &aligner);
};

namespace internal {

template<typename RegistrateFactor>
bool encodePointCloudAligner(const PointCloudAlignerBase &aligner_base, Node *aligner_node);

} // namespace internal
} // namespace YAML
