#pragma once

#include "lvins_camera/camera_rig.h"

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
#include <yaml-cpp/yaml.h>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

using lvins::CameraRig;

namespace YAML {

/**
 * @brief YAML序列化中CameraRig类的实现
 * @details 该类实现了yaml-cpp库中CameraRig类的序列化与反序列化的接口<br/>
 *         用法示例：<br/>
 *         1. 读取（反序列化）：auto rig = YAML::get<CameraRig>(node, "rig");<br/>
 *         2. 写入（序列化）：node["rig"] = rig;<br/>
 *         详细实例可查看CameraRig::loadFromYaml的实现
 */
template<>
struct convert<CameraRig> {
    /**
     * @brief 序列化CameraRig
     * @param camera_rig CameraRig的实例
     * @return 是否序列化成功
     */
    static Node encode(const CameraRig &camera_rig);

    /**
     * @brief 反序列化CameraRig
     * @param node CameraRig反序列化所在的YAML节点
     * @param camera_rig CameraRig的实例
     * @return 是否反序列化成功
     */
    static bool decode(const Node &node, CameraRig &camera_rig);
};

/**
 * @brief YAML序列化中CameraRig类指针的实现
 * @details 该类实现了yaml-cpp库中CameraRig类指针的序列化与反序列化的接口<br/>
 *         用法示例：<br/>
 *         1. 读取（反序列化）：auto rig = YAML::get<CameraRig::sPtr>(node, "rig");<br/>
 *         2. 写入（序列化）：node["rig"] = rig;<br/>
 *         详细实例可查看CameraRig::loadFromYaml的实现
 */
template<>
struct convert<CameraRig::sPtr> {
    /**
     * @brief 序列化CameraRig
     * @param camera_rig CameraRig的实例指针
     * @return 是否序列化成功
     */
    static Node encode(const CameraRig::sPtr &camera_rig);

    /**
     * @brief 反序列化CameraRig
     * @param node CameraRig反序列化所在的YAML节点
     * @param camera_rig CameraRig的实例指针
     * @return 是否反序列化成功
     */
    static bool decode(const Node &node, CameraRig::sPtr &camera_rig);
};

} // namespace YAML
