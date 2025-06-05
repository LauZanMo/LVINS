#pragma once

#include "lvins_common/eigen_types.h"
#include "lvins_common/string_helper.h"

#include <gtsam_points/types/point_cloud_cpu.hpp>

namespace lvins {

using Point         = Eigen::Matrix<double, 4, 1>;
using PointCloud    = gtsam_points::PointCloudCPU;
using RawPointCloud = PointCloud;

/**
 * @brief 点云变换
 * @param point_cloud 输入点云
 * @param T 变换矩阵
 * @return 变换后的点云
 */
PointCloud::Ptr transform(const PointCloud &point_cloud, const SE3f &T);

} // namespace lvins

/**
 * @brief 点云格式化器
 */
template<>
struct LVINS_FORMATTER<lvins::PointCloud> {
    /**
     * @brief 从文本中解析格式化字符
     * @param ctx 文本
     * @return 格式化字符尾部迭代器
     */
    static constexpr auto parse(const LVINS_FORMAT_PARSE_CONTEXT &ctx) {
        return ctx.begin();
    }

    /**
     * @brief 格式化
     * @param point_cloud 点云数据
     * @param ctx 输出的格式化文本
     * @return 输出格式化文本的尾部迭代器
     */
    static auto format(const lvins::PointCloud &point_cloud, LVINS_FORMAT_CONTEXT &ctx) {
        if (!point_cloud.has_points()) {
            return LVINS_FORMAT_TO(ctx.out(), "points size: empty");
        }

        if (!point_cloud.has_times()) {
            return LVINS_FORMAT_TO(ctx.out(), "points size: {}", point_cloud.num_points);
        }

        return LVINS_FORMAT_TO(ctx.out(), "timestamp range: [{} - {}], points size: {}",
                               LVINS_GROUP_DIGITS(static_cast<int64_t>(point_cloud.times[0])),
                               LVINS_GROUP_DIGITS(static_cast<int64_t>(point_cloud.times[point_cloud.size() - 1])),
                               point_cloud.num_points);
    }
};
