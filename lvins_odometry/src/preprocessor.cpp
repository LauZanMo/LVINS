#include "lvins_odometry/preprocessor.h"
#include "lvins_icp/preprocess/box_filter.h"
#include "lvins_icp/preprocess/voxel_grid_sample.h"

namespace lvins {

Preprocessor::Preprocessor(const YAML::Node &config) {
    const auto point_cloud_config = config["point_cloud"];
    crop_box_size_                = YAML::get<float>(point_cloud_config, "crop_box_size");
    voxel_grid_size_              = YAML::get<float>(point_cloud_config, "voxel_grid_size");
    min_voxel_size_               = YAML::get<float>(point_cloud_config, "min_voxel_size");
    max_voxel_size_               = YAML::get<float>(point_cloud_config, "max_voxel_size");
    desire_point_cloud_size_      = YAML::get<size_t>(point_cloud_config, "desire_point_cloud_size");
}

YAML::Node Preprocessor::writeToYaml() const {
    YAML::Node node;
    node["point_cloud"]["crop_box_size"]           = crop_box_size_;
    node["point_cloud"]["voxel_grid_size"]         = voxel_grid_size_;
    node["point_cloud"]["min_voxel_size"]          = min_voxel_size_;
    node["point_cloud"]["max_voxel_size"]          = max_voxel_size_;
    node["point_cloud"]["desire_point_cloud_size"] = desire_point_cloud_size_;
    return node;
}

RawPointCloud::Ptr Preprocessor::process(const RawPointCloud::ConstPtr &point_cloud) {
    // 载体滤波
    auto ret = boxFilter(point_cloud, crop_box_size_);

    // 自适应体素滤波
    ret = adaptiveVoxelGridSample(ret, voxel_grid_size_, min_voxel_size_, max_voxel_size_, desire_point_cloud_size_,
                                  &voxel_grid_size_);
    LVINS_DEBUG("Adaptive voxel filter grid size: {:.2f}", voxel_grid_size_);

    return ret;
}

std::string Preprocessor::print() const {
    return LVINS_FORMAT("Preprocessor:\n"
                        "  Point cloud:\n"
                        "  crop box size = {}\n"
                        "  voxel grid size = {}\n"
                        "  min voxel size = {}\n"
                        "  max voxel size = {}\n"
                        "  desire point cloud size = {}",
                        crop_box_size_, voxel_grid_size_, min_voxel_size_, max_voxel_size_, desire_point_cloud_size_);
}

} // namespace lvins
