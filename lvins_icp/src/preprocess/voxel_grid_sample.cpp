#include "lvins_icp/preprocess/voxel_grid_sample.h"
#include "lvins_common/eigen_helper.h"
#include "lvins_common/logger.h"

#include <tbb/parallel_sort.h>

namespace lvins {

RawPointCloud::Ptr voxelGridSample(const RawPointCloud::ConstPtr &point_cloud, float voxel_grid_size) {
    LVINS_CHECK(voxel_grid_size > 0.0f, "Voxel grid size should be greater than 0.0f!");

    // 点云为空直接返回
    if (point_cloud->empty()) {
        return pcl::make_shared<RawPointCloud>();
    }

    // 中间值
    constexpr auto invalid_coord    = std::numeric_limits<int64_t>::max(); // 无效坐标
    constexpr int coord_bit_width   = 21;            // 使用比特位表示体素坐标 (int64_t可以被拆分为3个21位的三维坐标)
    constexpr size_t coord_bit_mask = (1 << 21) - 1; // 比特掩模
    constexpr int coord_offset      = 1 << (coord_bit_width - 1); // 坐标偏置，使得带负数的坐标计算值为正数
    const auto resolution_inv       = 1.0f / voxel_grid_size;

    // 计算点坐标对应的比特位
    const auto getBit = [&](size_t i) -> uint64_t {
        const Eigen::Array3f point = (*point_cloud)[i].getArray3fMap();
        if (!point.isFinite().all()) {
            return invalid_coord;
        }

        const Eigen::Array3i coord = fastFloor(point * resolution_inv) + coord_offset;
        if ((coord < 0).any() || (coord > coord_bit_mask).any()) {
            LVINS_WARN("Voxel coordinate is out of range!");
            return invalid_coord;
        }

        // 计算体素坐标对应的比特位（0|1bit, z|21bit, y|21bit, x|21bit）
        const uint64_t bits = (coord[0] & coord_bit_mask) << (coord_bit_width * 0) |
                              (coord[1] & coord_bit_mask) << (coord_bit_width * 1) |
                              (coord[2] & coord_bit_mask) << (coord_bit_width * 2);
        return bits;
    };
    std::vector<std::pair<uint64_t, size_t>> bit_index(point_cloud->size());
    tbb::parallel_for(tbb::blocked_range<size_t>(0, point_cloud->size()), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t i = range.begin(); i < range.end(); ++i) {
            bit_index[i] = {getBit(i), i};
        }
    });

    // 通过比特位排序
    tbb::parallel_sort(bit_index.begin(), bit_index.end(), [](const auto &lhs, const auto &rhs) {
        return lhs.first < rhs.first;
    });

    // 体素滤波（求均值）
    constexpr int block_size       = 2048;
    std::atomic<size_t> num_points = 0;
    const auto ret                 = pcl::make_shared<RawPointCloud>();
    ret->header                    = point_cloud->header;
    ret->resize(point_cloud->size());
    const auto task = [&](const tbb::blocked_range<size_t> &range) {
        // 局部容器初始化
        std::vector<double> sub_timestamps;
        std::vector<Eigen::Vector3f> sub_points;
        std::vector<float> sub_intensities;
        sub_timestamps.reserve(block_size);
        sub_points.reserve(block_size);
        sub_intensities.reserve(block_size);

        // 体素内点云搜索
        size_t size               = 1;
        double sum_timestamp      = (*point_cloud)[bit_index[range.begin()].second].timestamp;
        Eigen::Vector3f sum_point = (*point_cloud)[bit_index[range.begin()].second].getVector3fMap();
        float sum_intensity       = (*point_cloud)[bit_index[range.begin()].second].intensity;
        for (size_t i = range.begin() + 1; i != range.end(); ++i) {
            const auto &[last_bit, last_index] = bit_index[i - 1];
            const auto &[cur_bit, cur_index]   = bit_index[i];
            if (cur_bit == invalid_coord) {
                continue;
            }

            // 同一体素内搜索完成，则求均值并加入局部容器
            if (cur_bit != last_bit) {
                sub_timestamps.emplace_back(sum_timestamp / static_cast<double>(size));
                sub_points.emplace_back(sum_point / static_cast<float>(size));
                sub_intensities.emplace_back(sum_intensity / static_cast<float>(size));

                size          = 0;
                sum_timestamp = 0.0;
                sum_point.setZero();
                sum_intensity = 0.0f;
            }

            // 同一体素内数值累积
            ++size;
            sum_timestamp += (*point_cloud)[cur_index].timestamp;
            sum_point += (*point_cloud)[cur_index].getVector3fMap();
            sum_intensity += (*point_cloud)[cur_index].intensity;
        }
        sub_timestamps.emplace_back(sum_timestamp / static_cast<double>(size));
        sub_points.emplace_back(sum_point / static_cast<float>(size));
        sub_intensities.emplace_back(sum_intensity / static_cast<float>(size));

        // 将结果写入点云
        const size_t index_begin = num_points.fetch_add(sub_points.size());
        for (size_t i = 0; i < sub_points.size(); ++i) {
            (*ret)[index_begin + i].timestamp        = sub_timestamps[i];
            (*ret)[index_begin + i].getVector3fMap() = sub_points[i];
            (*ret)[index_begin + i].intensity        = sub_intensities[i];
        }
    };
    tbb::parallel_for(tbb::blocked_range<size_t>(0, point_cloud->size(), block_size), task);
    ret->resize(num_points.load());

    return ret;
}

RawPointCloud::Ptr adaptiveVoxelGridSample(const RawPointCloud::ConstPtr &point_cloud, float init_voxel_grid_size,
                                           float min_voxel_grid_size, float max_voxel_grid_size,
                                           size_t desire_point_cloud_size, float *final_voxel_grid_size) {
    LVINS_CHECK(init_voxel_grid_size > 0.0f, "Initial voxel grid size should be greater than 0.0f!");
    LVINS_CHECK(init_voxel_grid_size < max_voxel_grid_size,
                "Max voxel grid size should be greater than initial voxel grid size!");

    // 循环调用体素网格采样，并根据情况调整体素网格尺寸，直到点云数量符合期望值
    auto voxel_grid_size0 = init_voxel_grid_size;
    auto voxel_grid_size1 = voxel_grid_size0;
    auto point_cloud0     = voxelGridSample(point_cloud, voxel_grid_size0);
    auto point_cloud1     = point_cloud0;

    while (true) {
        // 符合条件则跳出循环
        if (point_cloud0->size() < desire_point_cloud_size && point_cloud1->size() > desire_point_cloud_size) {
            break;
        }

        // 不符合条件则根据情况调整体素网格尺寸
        if (point_cloud0->size() < desire_point_cloud_size && point_cloud1->size() < desire_point_cloud_size) {
            point_cloud0     = point_cloud1;
            voxel_grid_size0 = voxel_grid_size1;
            voxel_grid_size1 /= 1.5f;
            point_cloud1 = voxelGridSample(point_cloud, voxel_grid_size1);
        } else if (point_cloud0->size() > desire_point_cloud_size && point_cloud1->size() > desire_point_cloud_size) {
            point_cloud1     = point_cloud0;
            voxel_grid_size1 = voxel_grid_size0;
            voxel_grid_size0 *= 1.5f;
            point_cloud0 = voxelGridSample(point_cloud, voxel_grid_size0);
        }

        // 体素网格尺寸超出临界条件则跳出循环
        if (voxel_grid_size1 < min_voxel_grid_size) {
            break;
        }

        if (voxel_grid_size0 > max_voxel_grid_size) {
            point_cloud0     = point_cloud1;
            voxel_grid_size0 = voxel_grid_size1;
            break;
        }
    }

    // 若需要返回最终体素网格尺寸，则赋值
    if (final_voxel_grid_size) {
        *final_voxel_grid_size = voxel_grid_size0;
    }

    return point_cloud0;
}

} // namespace lvins
