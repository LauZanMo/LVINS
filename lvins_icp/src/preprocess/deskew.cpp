#include "lvins_icp/preprocess/deskew.h"
#include "lvins_icp/preprocess/copy.h"
#include "lvins_icp/preprocess/remove_invalid_points.h"

#include <sophus/interpolate.hpp>
#include <tbb/parallel_for.h>

namespace lvins {

PointCloud::Ptr deskew(const RawPointCloud &point_cloud, const LidarGeometryBase &lidar, const SE3f &T_bs,
                       const NavStates &states) {
    LVINS_CHECK(point_cloud.points, "Point cloud should not be empty!");
    LVINS_CHECK(states.size() > 1, "States size should be larger than 1!");

    // 将导航状态转为以时间戳为基准均匀分布的位姿（20等分）
    std::vector<long> uniform_timestamp(21);
    std::vector<SE3f> uniform_pose(21);
    const auto dt     = (states.back().timestamp - states.front().timestamp) / 20;
    size_t state0_idx = 0;
    for (size_t i = 0; i < 20; ++i) {
        const long timestamp = states.front().timestamp + dt * i;
        uniform_timestamp[i] = timestamp;
        while (state0_idx + 1 < states.size() && states[state0_idx + 1].timestamp <= timestamp) {
            ++state0_idx;
        }

        const auto &state0 = states[state0_idx];
        const auto &state1 = states[state0_idx + 1];
        const auto ratio = LVINS_FLOAT(timestamp - state0.timestamp) / LVINS_FLOAT(state1.timestamp - state0.timestamp);
        uniform_pose[i]  = Sophus::interpolate(state0.T, state1.T, ratio);
    }
    uniform_timestamp.back() = states.back().timestamp;
    uniform_pose.back()      = states.back().T;

    // 检查雷达点深度并进行运动补偿
    const auto point_cloud_deskewed = copy(point_cloud);
    std::vector<uint8_t> valid(point_cloud.size(), 1);
    std::atomic<size_t> num_points_skipped{0};
    tbb::parallel_for(tbb::blocked_range<size_t>(0, point_cloud.size()), [&](const tbb::blocked_range<size_t> &r) {
        for (size_t i = r.begin(); i < r.end(); ++i) {
            const auto timestamp = static_cast<int64_t>(point_cloud_deskewed->times[i]);
            auto &point          = point_cloud_deskewed->points[i];

            // 雷达点时间戳在导航状态之外则跳过
            if (timestamp < uniform_timestamp.front() || timestamp >= uniform_timestamp.back()) {
                valid[i] = 0;
                ++num_points_skipped;
                continue;
            }

            // 雷达点不在探测范围内则跳过
            if (!lidar.isPointValid(point.head<3>())) {
                continue;
            }

            // 内插导航状态至雷达点时间戳处，对雷达点进行运动补偿
            const auto pose0_idx       = (timestamp - states.front().timestamp) / dt;
            const auto &timestamp0     = uniform_timestamp[pose0_idx];
            const auto &timestamp1     = uniform_timestamp[pose0_idx + 1];
            const auto &pose0          = uniform_pose[pose0_idx];
            const auto &pose1          = uniform_pose[pose0_idx + 1];
            const auto ratio           = LVINS_FLOAT(timestamp - timestamp0) / LVINS_FLOAT(timestamp1 - timestamp0);
            const SE3f T_ws_point      = Sophus::interpolate(pose0, pose1, ratio) * T_bs;
            const SE3f T_ws_frame      = uniform_pose.back() * T_bs;
            const Mat44d T_frame_point = (T_ws_frame.inverse() * T_ws_point).matrix().cast<double>();

            point = T_frame_point * point;
            if (point_cloud_deskewed->covs) {
                auto &cov = point_cloud_deskewed->covs[i];
                cov       = T_frame_point * cov * T_frame_point.transpose();
            }
            if (point_cloud_deskewed->normals) {
                auto &normal = point_cloud_deskewed->normals[i];
                normal       = T_frame_point * normal;
            }
        }
    });
    LVINS_DEBUG("{} points are skipped because of timestamp precision loss. Ratio: {:.2f}%", num_points_skipped,
                100.0 * num_points_skipped / point_cloud.size());

    // 删除无效点并返回
    return removeInvalidPoints(point_cloud_deskewed, valid);
}

} // namespace lvins
