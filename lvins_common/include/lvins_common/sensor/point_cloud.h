#pragma once

#include "lvins_common/string_helper.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lvins {

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif

///< 内部点及点云类
using PointCovarianceMap      = Eigen::Map<Eigen::Matrix4f, Eigen::Aligned>;
using PointCovarianceMapConst = const Eigen::Map<const Eigen::Matrix4f, Eigen::Aligned>;
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;      ///< 三维点（齐次形式）
    float intensity;      ///< 强度
    union EIGEN_ALIGN16 { ///< 协方差矩阵
        float cov[16];
        struct {
            float cov00, cov10, cov20, cov30;
            float cov01, cov11, cov21, cov31;
            float cov02, cov12, cov22, cov32;
            float cov03, cov13, cov23, cov33;
        };
    };

    PointCovarianceMap getCovariance4fMap() {
        return PointCovarianceMap(cov);
    }

    [[nodiscard]] PointCovarianceMapConst getCovariance4fMap() const {
        return PointCovarianceMapConst(cov);
    }

    PCL_MAKE_ALIGNED_OPERATOR_NEW
};
using PointCloud = pcl::PointCloud<Point>;

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

///< 内部原始点及点云类
struct EIGEN_ALIGN16 RawPoint {
    PCL_ADD_POINT4D;  ///< 三维点（齐次形式）
    float intensity;  ///< 强度
    double timestamp; ///< 绝对时间戳（ns）
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
using RawPointCloud = pcl::PointCloud<RawPoint>;

///< Ouster原始点及点云类
struct EIGEN_ALIGN16 OusterPoint {
    PCL_ADD_POINT4D; ///< 三维点（齐次形式）
    float intensity; ///< 强度
    uint32_t t;      ///< Ouster：相对于扫描开始时间的差值（ns）
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
using OusterPointCloud = pcl::PointCloud<OusterPoint>;

///< Velodyne原始点及点云类
struct EIGEN_ALIGN16 VelodynePoint {
    PCL_ADD_POINT4D; ///< 三维点（齐次形式）
    float intensity; ///< 强度
    float time;      ///< Velodyne：相对于扫描开始时间的差值（s）
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
using VelodynePointCloud = pcl::PointCloud<VelodynePoint>;

///< Livox原始点及点云类
using LivoxPoint      = RawPoint;
using LivoxPointCloud = RawPointCloud;

} // namespace lvins

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(lvins::Point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, cov00, cov00)
                                  (float, cov10, cov10)
                                  (float, cov20, cov20)
                                  (float, cov30, cov30)
                                  (float, cov01, cov01)
                                  (float, cov11, cov11)
                                  (float, cov21, cov21)
                                  (float, cov31, cov31)
                                  (float, cov02, cov02)
                                  (float, cov12, cov12)
                                  (float, cov22, cov22)
                                  (float, cov32, cov32)
                                  (float, cov03, cov03)
                                  (float, cov13, cov13)
                                  (float, cov23, cov23)
                                  (float, cov33, cov33))

POINT_CLOUD_REGISTER_POINT_STRUCT(lvins::RawPoint,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (double, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(lvins::OusterPoint,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (uint32_t, t, t))

POINT_CLOUD_REGISTER_POINT_STRUCT(lvins::VelodynePoint,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (float, time, time))
// clang-format on

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
    constexpr auto parse(LVINS_FORMAT_PARSE_CONTEXT &ctx) {
        return ctx.begin();
    }

    /**
     * @brief 格式化
     * @param point_cloud 点云数据
     * @param ctx 输出的格式化文本
     * @return 输出格式化文本的尾部迭代器
     */
    auto format(const lvins::PointCloud &point_cloud, LVINS_FORMAT_CONTEXT &ctx) const {
        return LVINS_FORMAT_TO(ctx.out(),
                               "seq: {}\n"
                               "stamp: {}\n"
                               "frame id: {}\n"
                               "points size: {}\n"
                               "is dense: {}",
                               point_cloud.header.seq, LVINS_GROUP_DIGITS(point_cloud.header.stamp),
                               point_cloud.header.frame_id, LVINS_GROUP_DIGITS(point_cloud.size()),
                               point_cloud.is_dense);
    }
};

/**
 * @brief 原始点云格式化器
 */
template<>
struct LVINS_FORMATTER<lvins::RawPointCloud> {
    /**
     * @brief 从文本中解析格式化字符
     * @param ctx 文本
     * @return 格式化字符尾部迭代器
     */
    constexpr auto parse(LVINS_FORMAT_PARSE_CONTEXT &ctx) {
        return ctx.begin();
    }

    /**
     * @brief 格式化
     * @param point_cloud 原始点云数据
     * @param ctx 输出的格式化文本
     * @return 输出格式化文本的尾部迭代器
     */
    auto format(const lvins::RawPointCloud &point_cloud, LVINS_FORMAT_CONTEXT &ctx) const {
        if (point_cloud.empty()) {
            return LVINS_FORMAT_TO(ctx.out(),
                                   "seq: {}\n"
                                   "stamp: {}\n"
                                   "frame id: {}\n"
                                   "points size: {}\n"
                                   "is dense: {}",
                                   point_cloud.header.seq, LVINS_GROUP_DIGITS(point_cloud.header.stamp),
                                   point_cloud.header.frame_id, LVINS_GROUP_DIGITS(point_cloud.size()),
                                   point_cloud.is_dense);
        } else {
            return LVINS_FORMAT_TO(ctx.out(),
                                   "seq: {}\n"
                                   "stamp: {}\n"
                                   "frame id: {}\n"
                                   "timestamp range: [{} - {}]\n"
                                   "points size: {}\n"
                                   "is dense: {}",
                                   point_cloud.header.seq, LVINS_GROUP_DIGITS(point_cloud.header.stamp),
                                   point_cloud.header.frame_id, LVINS_GROUP_DIGITS(point_cloud.front().timestamp),
                                   LVINS_GROUP_DIGITS(point_cloud.back().timestamp),
                                   LVINS_GROUP_DIGITS(point_cloud.size()), point_cloud.is_dense);
        }
    }
};
