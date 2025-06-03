#include "lvins_odometry/preprocessor.h"
#include "lvins_icp/preprocess/box_filter.h"
#include "lvins_icp/preprocess/voxel_grid_sample.h"

namespace lvins {

Preprocessor::Preprocessor(const YAML::Node &config) {
    const auto point_cloud_config = config["point_cloud"];
    crop_box_size_                = YAML::get<float>(point_cloud_config, "crop_box_size");
    voxel_grid_size_              = YAML::get<float>(point_cloud_config, "voxel_grid_size");
}

YAML::Node Preprocessor::writeToYaml() const {
    YAML::Node node;
    node["point_cloud"]["crop_box_size"]   = crop_box_size_;
    node["point_cloud"]["voxel_grid_size"] = voxel_grid_size_;
    return node;
}

RawPointCloud::Ptr Preprocessor::process(const RawPointCloud::ConstPtr &point_cloud) const {
    // 载体滤波
    auto ret = boxFilter(point_cloud, crop_box_size_);

    // 体素滤波
    ret = voxelGridSample(ret, voxel_grid_size_);

    return ret;
}

std::string Preprocessor::print() const {
    return LVINS_FORMAT("Preprocessor:\n"
                        "  Point cloud:\n"
                        "  crop box size = {}\n"
                        "  voxel grid size = {}",
                        crop_box_size_, voxel_grid_size_);
}

} // namespace lvins
