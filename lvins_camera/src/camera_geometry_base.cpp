#include "lvins_camera/camera_geometry_base.h"
#include "lvins_camera/yaml/camera_yaml_serialization.h"
#include "lvins_common/path_helper.h"
#include "lvins_common/yaml/yaml_serialization.h"

namespace lvins {

CameraGeometryBase::sPtr CameraGeometryBase::loadFromYaml(const std::string &config_file) {
    // 检查并转换路径（如果有需要）
    LVINS_CHECK(!config_file.empty(), "config_file should not be empty!");

    // 根据配置文件加载相机
    const auto node = YAML::load(path_helper::completePath(config_file));
    return YAML::get<CameraGeometryBase::sPtr>(node, "");
}

void CameraGeometryBase::writeToYaml(const std::string &config_file) const {
    // 检查并转换路径（如果有需要）
    LVINS_CHECK(!config_file.empty(), "config_file should not be empty!");

    // 将相机写入配置文件
    YAML::Node node;
    node = *this;
    YAML::dump(node, path_helper::completePath(config_file));
}

} // namespace lvins
