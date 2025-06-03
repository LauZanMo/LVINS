#pragma once

#include "lvins_common/logger.h"

#include <typeinfo>
#include <yaml-cpp/yaml.h>

namespace YAML {

/**
 * @brief 安全加载YAML文件
 * @details 若加载失败，则打印错误信息并返回空节点
 * @param file_name 文件名
 * @return YAML节点
 */
YAML::Node load(const std::string &file_name);

/**
 * @brief 安全保存YAML文件
 * @param node YAML节点
 * @param file_name 文件名
 */
void dump(const YAML::Node &node, const std::string &file_name);

/**
 * @brief 安全获取指定节点的键对应的值
 * @details 若键不存在或类型不匹配，则打印错误信息并返回默认值
 * @tparam T 值类型
 * @param node 节点
 * @param key 键（可以为空，即直接转换当前节点）
 * @return 值
 */
template<typename T>
T get(const YAML::Node &node, const std::string &key) {
    LVINS_CHECK(node.IsMap(), "Unable to get Node[\"{}\"] because the node is not a map!", key);

    // 获取子节点，若键为空则子节点即为当前节点
    Node sub_node;
    try {
        if (key.empty())
            sub_node = node;
        else
            sub_node = node[key];
    } catch (const YAML::Exception &e) {
        LVINS_FATAL("Error getting key \"{}\"!", key);
    }

    if (sub_node) {
        try {
            return sub_node.as<T>();
        } catch (const YAML::Exception &e) {
            LVINS_FATAL("Error getting key \"{}\" as type {}!", key, typeid(T).name());
        }
    } else {
        LVINS_FATAL("Key \"{}\" does not exist!", key);
    }
}

} // namespace YAML
