#pragma once

#include "lvins_camera/camera_geometry_base.h"

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
#include <yaml-cpp/yaml.h>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

using lvins::CameraGeometryBase;

namespace YAML {

/**
 * @brief YAML序列化中CameraGeometryBase类的实现
 * @details 该类实现了yaml-cpp库中CameraGeometryBase类的序列化与反序列化的接口<br/>
 *         用法示例：<br/>
 *         1. 读取（反序列化）：auto cam = YAML::get<CameraGeometryBase>(node, "cam0");<br/>
 *         2. 写入（序列化）：node["cam0"] = cam;<br/>
 *         详细实例可查看CameraGeometryBase::loadFromYaml的实现
 */
template<>
struct convert<CameraGeometryBase> {
    /**
     * @brief 序列化CameraGeometryBase
     * @param camera CameraGeometryBase的实例
     * @return 是否序列化成功
     */
    static Node encode(const CameraGeometryBase &camera);

    /**
     * @brief 反序列化CameraGeometryBase
     * @param node CameraGeometryBase反序列化所在的YAML节点
     * @param camera CameraGeometryBase的实例
     * @return 是否反序列化成功
     */
    static bool decode(const Node &node, CameraGeometryBase &camera);
};

/**
 * @brief YAML序列化中CameraGeometryBase类指针的实现
 * @details 该类实现了yaml-cpp库中CameraGeometryBase类指针的序列化与反序列化的接口<br/>
 *          用法示例：<br/>
 *          1. 读取（反序列化）：auto cam = YAML::get<CameraGeometryBase::Ptr>(node, "cam0");<br/>
 *          2. 写入（序列化）：node["cam0"] = cam;<br/>
 *          详细实例可查看CameraGeometryBase::loadFromYaml的实现
 */
template<>
struct convert<CameraGeometryBase::Ptr> {
    /**
     * @brief 序列化CameraGeometryBase类指针
     * @param camera CameraGeometryBase的实例指针
     * @return 是否序列化成功
     */
    static Node encode(const CameraGeometryBase::Ptr &camera);

    /**
     * @brief 反序列化CameraGeometryBase类指针
     * @param node CameraGeometryBase反序列化所在的YAML节点
     * @param camera CameraGeometryBase的实例指针
     * @return 是否反序列化成功
     */
    static bool decode(const Node &node, CameraGeometryBase::Ptr &camera);
};

namespace internal {

/**
 * @brief 序列化CameraGeometryBase中针孔相机投影模型
 * @tparam Distortion 相机的畸变模型
 * @param camera 相机实例
 * @param camera_node 相机所在的YAML节点
 * @return 是否序列化成功
 */
template<typename Distortion>
bool encodePinhole(const CameraGeometryBase &camera, Node *camera_node);

/**
 * @brief 序列化CameraGeometryBase中全向相机投影模型
 * @tparam Distortion 相机的畸变模型
 * @param camera 相机实例
 * @param camera_node 相机所在的YAML节点
 * @return 是否序列化成功
 */
template<typename Distortion>
bool encodeOmni(const CameraGeometryBase &camera, Node *camera_node);

} // namespace internal
} // namespace YAML
