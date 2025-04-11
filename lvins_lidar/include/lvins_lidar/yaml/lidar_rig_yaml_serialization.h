#pragma once

#include "lvins_lidar/lidar_rig.h"

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
#include <yaml-cpp/yaml.h>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

using lvins::LidarRig;

namespace YAML {

/**
 * @brief YAML序列化中LidarRig类的实现
 * @details 该类实现了yaml-cpp库中LidarRig类的序列化与反序列化的接口<br/>
 *         用法示例：<br/>
 *         1. 读取（反序列化）：auto rig = YAML::get<LidarRig>(node, "rig");<br/>
 *         2. 写入（序列化）：node["rig"] = rig;<br/>
 *         详细实例可查看LidarRig::loadFromYaml/writeToYaml的实现
 */
template<>
struct convert<LidarRig> {
    /**
     * @brief 序列化LidarRig
     * @param lidar_rig LidarRig的实例
     * @return 是否序列化成功
     */
    static Node encode(const LidarRig &lidar_rig);

    /**
     * @brief 反序列化LidarRig
     * @param node LidarRig反序列化所在的YAML节点
     * @param lidar_rig LidarRig的实例
     * @return 是否反序列化成功
     */
    static bool decode(const Node &node, LidarRig &lidar_rig);
};

/**
 * @brief YAML序列化中LidarRig类指针的实现
 * @details 该类实现了yaml-cpp库中LidarRig类指针的序列化与反序列化的接口<br/>
 *         用法示例：<br/>
 *         1. 读取（反序列化）：auto rig = YAML::get<LidarRig::sPtr>(node, "rig");<br/>
 *         2. 写入（序列化）：node["rig"] = rig;<br/>
 *         详细实例可查看LidarRig::loadFromYaml/writeToYaml的实现
 */
template<>
struct convert<LidarRig::sPtr> {
    /**
     * @brief 序列化LidarRig
     * @param lidar_rig LidarRig的实例指针
     * @return 是否序列化成功
     */
    static Node encode(const LidarRig::sPtr &lidar_rig);

    /**
     * @brief 反序列化LidarRig
     * @param node LidarRig反序列化所在的YAML节点
     * @param lidar_rig LidarRig的实例指针
     * @return 是否反序列化成功
     */
    static bool decode(const Node &node, LidarRig::sPtr &lidar_rig);
};

} // namespace YAML
