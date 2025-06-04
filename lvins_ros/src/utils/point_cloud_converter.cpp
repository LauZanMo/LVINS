#include "lvins_ros/utils/point_cloud_converter.h"
#include "lvins_common/logger.h"

#include <rclcpp/rclcpp.hpp>

namespace lvins::point_cloud_converter {

template<typename T>
Point getPoint(const uint8_t *x, const uint8_t *y, const uint8_t *z) {
    return Point(*reinterpret_cast<const T *>(x), *reinterpret_cast<const T *>(y), *reinterpret_cast<const T *>(z),
                 1.0);
}

std::string detectLidarType(const sensor_msgs::msg::PointCloud2 &msg) {
    for (const auto &field: msg.fields) {
        if (field.name == "t") {
            return "ouster";
        }

        if (field.name == "time") {
            return "velodyne";
        }

        // TODO: 还需要考虑timestamp的offset，用于区分livox和robosense
        if (field.name == "timestamp") {
            return "livox";
        }
    }
    LVINS_ERROR("Can not detect lidar type!");
    return {};
}

PointCloud::Ptr convert(const sensor_msgs::msg::PointCloud2 &msg, const std::string &lidar_type) {
    const size_t num_points = msg.width * msg.height;

    int x_type         = 0;
    int y_type         = 0;
    int z_type         = 0;
    int timestamp_type = 0;
    int intensity_type = 0;

    int x_offset         = -1;
    int y_offset         = -1;
    int z_offset         = -1;
    int timestamp_offset = -1;
    int intensity_offset = -1;

    std::unordered_map<std::string, std::pair<int *, int *>> fields;
    fields["x"]         = std::make_pair(&x_type, &x_offset);
    fields["y"]         = std::make_pair(&y_type, &y_offset);
    fields["z"]         = std::make_pair(&z_type, &z_offset);
    fields["t"]         = std::make_pair(&timestamp_type, &timestamp_offset);
    fields["time"]      = std::make_pair(&timestamp_type, &timestamp_offset);
    fields["timestamp"] = std::make_pair(&timestamp_type, &timestamp_offset);
    fields["intensity"] = std::make_pair(&intensity_type, &intensity_offset);

    for (const auto &field: msg.fields) {
        auto found = fields.find(field.name);
        if (found != fields.end()) {
            auto &[data_type, offset] = found->second;

            *data_type = field.datatype;
            *offset    = static_cast<int>(field.offset);
        }
    }

    using sensor_msgs::msg::PointField;
    LVINS_CHECK(x_offset != -1 && y_offset != -1 && z_offset != -1, "Message misses point coordinate fields: x, y, z");
    LVINS_CHECK((x_type == PointField::FLOAT32 || x_type == PointField::FLOAT64) && x_type == y_type &&
                        x_type == z_type,
                "Message point type should be FLOAT32 or FLOAT64!");

    const auto point_cloud = std::make_shared<PointCloud>();

    // 点云
    std::vector<Point> points;
    points.resize(num_points);
    if (x_type == PointField::FLOAT32 && y_offset == x_offset + static_cast<int>(sizeof(float)) &&
        z_offset == y_offset + static_cast<int>(sizeof(float))) {
        // 特殊情况：3个连续的单精度浮点数
        for (size_t i = 0; i < num_points; ++i) {
            const auto raw_ptr    = &msg.data[msg.point_step * i + x_offset];
            const auto casted_ptr = reinterpret_cast<const float *>(raw_ptr);
            points[i] << Eigen::Map<const Eigen::Vector3f>(casted_ptr).cast<double>(), 1.0;
        }
    } else if (x_type == PointField::FLOAT64 && y_offset == x_offset + static_cast<int>(sizeof(double)) &&
               z_offset == y_offset + static_cast<int>(sizeof(double))) {
        // 特殊情况：3个连续的双精度浮点数
        for (size_t i = 0; i < num_points; ++i) {
            const auto raw_ptr    = &msg.data[msg.point_step * i + x_offset];
            const auto casted_ptr = reinterpret_cast<const double *>(raw_ptr);
            points[i] << Eigen::Map<const Eigen::Vector3d>(casted_ptr), 1.0;
        }
    } else {
        for (size_t i = 0; i < num_points; ++i) {
            const auto x_ptr = &msg.data[msg.point_step * i + x_offset];
            const auto y_ptr = &msg.data[msg.point_step * i + y_offset];
            const auto z_ptr = &msg.data[msg.point_step * i + z_offset];

            if (x_type == PointField::FLOAT32) {
                points[i] = getPoint<float>(x_ptr, y_ptr, z_ptr);
            } else {
                points[i] = getPoint<double>(x_ptr, y_ptr, z_ptr);
            }
        }
    }
    point_cloud->add_points(points);

    // 时间戳
    if (timestamp_offset != -1) {
        std::vector<double> timestamps;
        timestamps.resize(num_points);

        for (size_t i = 0; i < num_points; ++i) {
            const auto timestamp_ptr = &msg.data[msg.point_step * i + timestamp_offset];
            if (lidar_type == "ouster") {
                // Ouster：相对于扫描开始时间的差值（ns）
                timestamps[i] = static_cast<double>(rclcpp::Time(msg.header.stamp).nanoseconds()) +
                                *reinterpret_cast<const uint32_t *>(timestamp_ptr);
            } else if (lidar_type == "velodyne") {
                // Velodyne：相对于扫描开始时间的差值（ns）
                timestamps[i] = static_cast<double>(rclcpp::Time(msg.header.stamp).nanoseconds()) +
                                *reinterpret_cast<const float *>(timestamp_ptr);
            } else if (lidar_type == "livox") {
                // Livox：绝对时间戳（ns）
                timestamps[i] = *reinterpret_cast<const double *>(timestamp_ptr);
            } else if (lidar_type == "robosense") {
                // Robosense：绝对时间戳（s）
                timestamps[i] = *reinterpret_cast<const double *>(timestamp_ptr) * 1e9;
            } else {
                LVINS_FATAL("Unsupported lidar type: {}", lidar_type);
            }
        }
        point_cloud->add_times(timestamps);
    }

    // 强度
    if (intensity_offset != -1) {
        std::vector<double> intensities;
        intensities.resize(num_points);

        for (size_t i = 0; i < num_points; ++i) {
            const auto *intensity_ptr = &msg.data[msg.point_step * i + intensity_offset];
            switch (intensity_type) {
                case PointField::UINT8:
                    intensities[i] = *intensity_ptr;
                    break;
                case PointField::UINT16:
                    intensities[i] = *reinterpret_cast<const uint16_t *>(intensity_ptr);
                    break;
                case PointField::UINT32:
                    intensities[i] = *reinterpret_cast<const uint32_t *>(intensity_ptr);
                    break;
                case PointField::FLOAT32:
                    intensities[i] = *reinterpret_cast<const float *>(intensity_ptr);
                    break;
                case PointField::FLOAT64:
                    intensities[i] = *reinterpret_cast<const double *>(intensity_ptr);
                    break;
                default:
                    LVINS_FATAL("Unsupported intensity type: {}", intensity_type);
            }
        }
        point_cloud->add_intensities(intensities);
    }

    return point_cloud;
}

PointCloud::Ptr convert(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, const std::string &lidar_type) {
    return convert(*msg, lidar_type);
}

sensor_msgs::msg::PointCloud2::SharedPtr convert(int64_t timestamp, const std::string &frame_id,
                                                 const PointCloud &point_cloud) {
    const auto msg       = std::make_shared<sensor_msgs::msg::PointCloud2>();
    msg->header.frame_id = frame_id;
    msg->header.stamp    = rclcpp::Time(timestamp);

    msg->width  = point_cloud.size();
    msg->height = 1;

    using sensor_msgs::msg::PointField;
    const std::vector<std::string> field_names = {"x", "y", "z", "intensity"};
    const size_t num_fields                    = point_cloud.intensities ? 4 : 3;
    msg->fields.resize(num_fields);
    for (size_t i = 0; i < num_fields; ++i) {
        msg->fields[i].name     = field_names[i];
        msg->fields[i].offset   = sizeof(float) * i;
        msg->fields[i].datatype = PointField::FLOAT32;
        msg->fields[i].count    = 1;
    }

    msg->is_bigendian = false;
    msg->point_step   = sizeof(float) * num_fields;
    msg->row_step     = sizeof(float) * num_fields * point_cloud.size();

    msg->data.resize(sizeof(float) * num_fields * point_cloud.size());
    for (size_t i = 0; i < point_cloud.size(); ++i) {
        auto *point = reinterpret_cast<float *>(msg->data.data() + msg->point_step * i);
        for (long j = 0; j < 3; ++j) {
            point[j] = static_cast<float>(point_cloud.points[i][j]);
        }

        if (point_cloud.intensities) {
            point[3] = static_cast<float>(point_cloud.intensities[i]);
        }
    }

    return msg;
}

sensor_msgs::msg::PointCloud2::SharedPtr convert(int64_t timestamp, const std::string &frame_id,
                                                 const PointCloud::ConstPtr &point_cloud) {
    return convert(timestamp, frame_id, *point_cloud);
}

} // namespace lvins::point_cloud_converter
