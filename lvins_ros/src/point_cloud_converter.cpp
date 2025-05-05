#include "lvins_ros/point_cloud_converter.h"
#include "lvins_common/logger.h"
#include <pcl/filters/filter.h>

namespace lvins::point_cloud_converter {

std::string detectLidarType(const sensor_msgs::msg::PointCloud2 &msg) {
    for (const auto &field: msg.fields) {
        // TODO: 还需要考虑offset
        if (field.name == "t") {
            return "ouster";
        } else if (field.name == "time") {
            return "velodyne";
        } else if (field.name == "timestamp") {
            return "livox";
        }
    }
    LVINS_ERROR("Can not detect lidar type!");
    return {};
}

RawPointCloud::Ptr convert(OusterPointCloud &src) {
    // 去除无效点
    std::vector<int> idx;
    src.is_dense = false;
    pcl::removeNaNFromPointCloud(src, src, idx);

    // 点云转换，包头时间戳设置为最新点的时间戳
    auto ret    = std::make_shared<RawPointCloud>();
    ret->header = src.header;
    ret->header.stamp *= 1e3; // pcl定义为us，统一转换为ns
    ret->width    = src.width;
    ret->height   = src.height;
    ret->is_dense = src.is_dense;
    ret->resize(src.size());
    auto ref_timestamp = static_cast<double>(src.header.stamp) * 1e3;
    for (size_t i = 0; i < src.size(); ++i) {
        ret->points[i].x         = src.points[i].x;
        ret->points[i].y         = src.points[i].y;
        ret->points[i].z         = src.points[i].z;
        ret->points[i].intensity = src.points[i].intensity;
        ret->points[i].timestamp = ref_timestamp + src.points[i].t;

        if (static_cast<double>(ret->header.stamp) < ret->points[i].timestamp) {
            ret->header.stamp = std::ceil(ret->points[i].timestamp);
        }
    }

    return ret;
}

RawPointCloud::Ptr convert(VelodynePointCloud &src) {
    // 去除无效点
    std::vector<int> idx;
    src.is_dense = false;
    pcl::removeNaNFromPointCloud(src, src, idx);

    // 点云转换，包头时间戳设置为最新点的时间戳
    auto ret    = std::make_shared<RawPointCloud>();
    ret->header = src.header;
    ret->header.stamp *= 1e3; // pcl定义为us，统一转换为ns
    ret->width    = src.width;
    ret->height   = src.height;
    ret->is_dense = src.is_dense;
    ret->resize(src.size());
    auto ref_timestamp = static_cast<double>(src.header.stamp) * 1e3;
    for (size_t i = 0; i < src.size(); ++i) {
        ret->points[i].x         = src.points[i].x;
        ret->points[i].y         = src.points[i].y;
        ret->points[i].z         = src.points[i].z;
        ret->points[i].intensity = src.points[i].intensity;
        ret->points[i].timestamp = ref_timestamp + src.points[i].time * 1e9;

        if (static_cast<double>(ret->header.stamp) < ret->points[i].timestamp) {
            ret->header.stamp = std::ceil(ret->points[i].timestamp);
        }
    }

    return ret;
}

RawPointCloud::Ptr convert(LivoxPointCloud &src) {
    // 去除无效点
    auto ret = std::make_shared<RawPointCloud>();
    std::vector<int> idx;
    src.is_dense = false;
    pcl::removeNaNFromPointCloud(src, *ret, idx);
    ret->header.stamp *= 1e3; // pcl定义为us，统一转换为ns

    // 包头时间戳设置为最新点的时间戳
    for (size_t i = 0; i < src.size(); ++i) {
        if (static_cast<double>(ret->header.stamp) < ret->points[i].timestamp) {
            ret->header.stamp = std::ceil(ret->points[i].timestamp);
        }
    }

    return ret;
}

RawPointCloud::Ptr convert(RobosensePointCloud &src) {
    // 去除无效点
    std::vector<int> idx;
    src.is_dense = false;
    pcl::removeNaNFromPointCloud(src, src, idx);

    // 点云转换，包头时间戳设置为最新点的时间戳
    auto ret    = std::make_shared<RawPointCloud>();
    ret->header = src.header;
    ret->header.stamp *= 1e3; // pcl定义为us，统一转换为ns
    ret->width    = src.width;
    ret->height   = src.height;
    ret->is_dense = src.is_dense;
    ret->resize(src.size());
    for (size_t i = 0; i < src.size(); ++i) {
        ret->points[i].x         = src.points[i].x;
        ret->points[i].y         = src.points[i].y;
        ret->points[i].z         = src.points[i].z;
        ret->points[i].intensity = src.points[i].intensity;
        ret->points[i].timestamp = src.points[i].timestamp * 1e9;

        if (static_cast<double>(ret->header.stamp) < ret->points[i].timestamp) {
            ret->header.stamp = std::ceil(ret->points[i].timestamp);
        }
    }

    return ret;
}

} // namespace lvins::point_cloud_converter
