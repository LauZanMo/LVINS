#include "lvins_common/yaml/yaml_serialization.h"

#include <fstream>

namespace YAML {

YAML::Node load(const std::string &file_name) {
    try {
        return YAML::LoadFile(file_name);
    } catch (const YAML::Exception &e) {
        LVINS_FATAL("Fail to load configuration file \"{}\"!", file_name);
    }
}

/**
 * @brief 递归序列化YAML节点
 * @param node YAML节点
 * @param out YAML转换器，用于输出到fstream
 */
void dumpNode(const YAML::Node &node, YAML::Emitter &out) {
    switch (node.Type()) {
        case YAML::NodeType::Null:
            break;
        case YAML::NodeType::Scalar:
            // 对于空字符串做特殊处理
            out << (node.as<std::string>() == "null" ? "" : node.as<std::string>());
            break;
        case YAML::NodeType::Sequence:
            out << YAML::BeginSeq;
            for (const auto &sub_node: node) {
                dumpNode(sub_node, out);
            }
            out << YAML::EndSeq;
            break;
        case YAML::NodeType::Map:
            out << YAML::BeginMap;
            for (const auto &sub_node: node) {
                // 对于data字段做特殊处理
                const auto key = sub_node.first.as<std::string>();
                if (key == "data") {
                    out.SetSeqFormat(YAML::Flow);
                } else {
                    out.SetSeqFormat(YAML::Block);
                }
                out << YAML::Key << key;
                out << YAML::Value;
                dumpNode(sub_node.second, out);
            }
            out << YAML::EndMap;
            break;
        default:
            LVINS_FATAL("Unsupported YAML node type!");
    }
}

void dump(const YAML::Node &node, const std::string &file_name) {
    try {
        YAML::Emitter out;
        out.SetIndent(4);
        dumpNode(node, out);

        std::ofstream fout(file_name);
        fout << out.c_str();
        fout.close();
    } catch (const YAML::Exception &e) {
        LVINS_FATAL("Fail to dump configuration file \"{}\"!", file_name);
    }
}

} // namespace YAML
