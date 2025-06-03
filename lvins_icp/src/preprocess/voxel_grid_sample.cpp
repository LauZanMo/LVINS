#include "lvins_icp/preprocess/voxel_grid_sample.h"
#include "lvins_common/logger.h"

namespace lvins {

RawPointCloud::Ptr voxelGridSample(const RawPointCloud::ConstPtr &point_cloud, float voxel_grid_size) {
    LVINS_CHECK(voxel_grid_size > 0.0f, "Voxel grid size should be greater than 0.0f!");

    return gtsam_points::voxelgrid_sampling(point_cloud, voxel_grid_size, 4);
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
