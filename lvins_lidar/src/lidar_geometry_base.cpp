#include "lvins_lidar/lidar_geometry_base.h"
#include "lvins_common/logger.h"
#include "lvins_common/path_helper.h"
#include "lvins_common/yaml/yaml_serialization.h"
#include "lvins_lidar/yaml/lidar_yaml_serialization.h"

namespace lvins {

LidarGeometryBase::LidarGeometryBase(std::string label, uint32_t scan_line, Float nearest_distance,
                                     Float farthest_distance)
    : label_(std::move(label)),
      scan_line_(scan_line),
      nearest_dist_(nearest_distance),
      nearest_dist2_(nearest_distance * nearest_distance),
      farthest_dist_(farthest_distance),
      farthest_dist2_(farthest_distance * farthest_distance) {}

LidarGeometryBase::Ptr LidarGeometryBase::loadFromYaml(const std::string &config_file) {
    // 检查并转换路径（如果有需要）
    LVINS_CHECK(!config_file.empty(), "config_file should not be empty!");

    // 根据配置文件加载雷达
    const auto node = YAML::load(path_helper::completePath(config_file));
    return YAML::get<LidarGeometryBase::Ptr>(node, "");
}

void LidarGeometryBase::writeToYaml(const std::string &config_file) const {
    // 检查并转换路径（如果有需要）
    LVINS_CHECK(!config_file.empty(), "config_file should not be empty!");

    // 将雷达写入配置文件
    YAML::Node node;
    node = *this;
    YAML::dump(node, path_helper::completePath(config_file));
}

int LidarGeometryBase::id() const {
    return id_;
}

void LidarGeometryBase::setId(int id) {
    id_ = id;
}
const std::string &LidarGeometryBase::label() const {
    return label_;
}

uint32_t LidarGeometryBase::scanLine() const {
    return scan_line_;
}

Float LidarGeometryBase::nearestDistance() const {
    return nearest_dist_;
}

Float LidarGeometryBase::farthestDistance() const {
    return farthest_dist_;
}

std::string LidarGeometryBase::print() const {
    return LVINS_FORMAT("{}\n"
                        "  name = {}\n"
                        "  scan line = {}\n"
                        "  nearest distance = {:.2f}\n"
                        "  farthest distance = {:.2f}",
                        label_, label_, scan_line_, nearest_dist_, farthest_dist_);
}

} // namespace lvins
