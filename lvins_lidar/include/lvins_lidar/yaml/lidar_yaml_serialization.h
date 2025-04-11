#pragma once

#include "lvins_lidar/lidar_geometry_base.h"

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
#include <yaml-cpp/yaml.h>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

using lvins::LidarGeometryBase;

namespace YAML {

/**
 * @brief YAML序列化中LidarGeometryBase类的实现
 * @details 该类实现了yaml-cpp库中LidarGeometryBase类的序列化与反序列化的接口<br/>
 *         用法示例：<br/>
 *         1. 读取（反序列化）：auto lidar = YAML::get<LidarGeometryBase>(node, "lidar0");<br/>
 *         2. 写入（序列化）：node["lidar0"] = lidar;<br/>
 *         详细实例可查看LidarGeometryBase::loadFromYaml/writeToYaml的实现
 */
template<>
struct convert<LidarGeometryBase> {
    /**
     * @brief 序列化LidarGeometryBase
     * @param lidar LidarGeometryBase的实例
     * @return 是否序列化成功
     */
    static Node encode(const LidarGeometryBase &lidar);

    /**
     * @brief 反序列化LidarGeometryBase
     * @param node LidarGeometryBase反序列化所在的YAML节点
     * @param lidar LidarGeometryBase的实例
     * @return 是否反序列化成功
     */
    static bool decode(const Node &node, LidarGeometryBase &lidar);
};

/**
 * @brief YAML序列化中LidarGeometryBase类指针的实现
 * @details 该类实现了yaml-cpp库中LidarGeometryBase类指针的序列化与反序列化的接口<br/>
 *          用法示例：<br/>
 *          1. 读取（反序列化）：auto lidar = YAML::get<LidarGeometryBase::sPtr>(node, "lidar0");<br/>
 *          2. 写入（序列化）：node["lidar0"] = lidar;<br/>
 *          详细实例可查看LidarGeometryBase::loadFromYaml/writeToYaml的实现
 */
template<>
struct convert<LidarGeometryBase::sPtr> {
    /**
     * @brief 序列化LidarGeometryBase类指针
     * @param lidar LidarGeometryBase的实例指针
     * @return 是否序列化成功
     */
    static Node encode(const LidarGeometryBase::sPtr &lidar);

    /**
     * @brief 反序列化LidarGeometryBase类指针
     * @param node LidarGeometryBase反序列化所在的YAML节点
     * @param lidar LidarGeometryBase的实例指针
     * @return 是否反序列化成功
     */
    static bool decode(const Node &node, LidarGeometryBase::sPtr &lidar);
};

} // namespace YAML
