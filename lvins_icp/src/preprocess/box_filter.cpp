#include "lvins_icp/preprocess/box_filter.h"

#include <pcl/filters/crop_box.h>

namespace lvins {

RawPointCloud::Ptr boxFilter(const RawPointCloud::ConstPtr &point_cloud, float crop_box_size) {
    pcl::CropBox<RawPoint> crop_box;
    crop_box.setNegative(true);
    crop_box.setMin(Eigen::Vector4f(-crop_box_size, -crop_box_size, -crop_box_size, 1.0f));
    crop_box.setMax(Eigen::Vector4f(crop_box_size, crop_box_size, crop_box_size, 1.0f));

    const auto ret = pcl::make_shared<RawPointCloud>();
    crop_box.setInputCloud(point_cloud);
    crop_box.filter(*ret);

    return ret;
}

} // namespace lvins
