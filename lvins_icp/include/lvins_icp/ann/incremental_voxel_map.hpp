#include "lvins_icp/ann/knn_result.h"

namespace lvins {

template<typename VoxelContent>
IncrementalVoxelMap<VoxelContent>::IncrementalVoxelMap(const typename VoxelContent::Setting &voxel_setting,
                                                       Float leaf_size, size_t lru_horizon, size_t lru_clear_cycle,
                                                       size_t search_offsets)
    : voxel_setting_(voxel_setting),
      inv_leaf_size_(LVINS_FLOAT(1.0) / leaf_size),
      lru_horizon_(lru_horizon),
      lru_clear_cycle_(lru_clear_cycle) {
    setSearchOffsets(search_offsets);
}

template<typename VoxelContent>
const typename VoxelContent::Setting &IncrementalVoxelMap<VoxelContent>::voxelSetting() const {
    return voxel_setting_;
}

template<typename VoxelContent>
Float IncrementalVoxelMap<VoxelContent>::leafSize() const {
    return 1.0 / inv_leaf_size_;
}

template<typename VoxelContent>
size_t IncrementalVoxelMap<VoxelContent>::lruHorizon() const {
    return lru_horizon_;
}

template<typename VoxelContent>
size_t IncrementalVoxelMap<VoxelContent>::lruClearCycle() const {
    return lru_clear_cycle_;
}

template<typename VoxelContent>
size_t IncrementalVoxelMap<VoxelContent>::searchOffsets() const {
    return search_offsets_.size();
}

template<typename VoxelContent>
void IncrementalVoxelMap<VoxelContent>::setSearchOffsets(size_t search_offsets) {
    switch (search_offsets) {
        case 1:
            search_offsets_ = {Vec3i::Zero()};
            break;

        case 7:
            search_offsets_ = {Vec3i::Zero(),   Vec3i(1, 0, 0), Vec3i(-1, 0, 0), Vec3i(0, 1, 0),
                               Vec3i(0, -1, 0), Vec3i(0, 0, 1), Vec3i(0, 0, -1)};
            break;

        case 27:
            for (int i = -1; i <= 1; ++i) {
                for (int j = -1; j <= 1; ++j) {
                    for (int k = -1; k <= 1; ++k) {
                        search_offsets_.emplace_back(i, j, k);
                    }
                }
            }
            break;

        default:
            LVINS_ERROR("Invalid search offsets: {}, available search offset: 1, 7, 27", search_offsets);
            break;
    }
}

template<typename VoxelContent>
void IncrementalVoxelMap<VoxelContent>::insert(const PointCloud &point_cloud, const SE3f &T) {
    // 将点云分别嵌入对应体素
    for (size_t i = 0; i < point_cloud.size(); ++i) {
        const Vec3f point = T * point_cloud[i].getVector3fMap().cast<Float>();
        const Vec3i coord = fastFloor(point * inv_leaf_size_);

        // 查询体素是否存在，不存在则创建新的体素
        auto found = voxel_index_map_.find(coord);
        if (found == voxel_index_map_.end()) {
            const auto voxel = std::make_shared<std::pair<VoxelInfo, VoxelContent>>(VoxelInfo(coord, lru_counter_),
                                                                                    VoxelContent());
            found            = voxel_index_map_.emplace_hint(found, coord, voxels_.size());
            voxels_.emplace_back(voxel);
        }

        // 更新体素的LRU计数器并将点加入体素
        auto &[info, content] = *voxels_[found->second];
        info.lru              = lru_counter_;
        content.add(voxel_setting_, point_cloud, i, T);
    }

    // 周期性删除长时间未使用的体素
    if (++lru_counter_ % lru_clear_cycle_ == 0) {
        // 删除长时间未使用的体素
        const auto remove_counter = std::remove_if(voxels_.begin(), voxels_.end(), [this](const auto &voxel) {
            return voxel->first.lru + lru_horizon_ < lru_counter_;
        });
        voxels_.resize(std::distance(voxels_.begin(), remove_counter));

        // 重排索引（rehash）
        voxel_index_map_.clear();
        for (size_t i = 0; i < voxels_.size(); ++i) {
            auto &[info, content]        = *voxels_[i];
            voxel_index_map_[info.coord] = i;
        }
    }

    // 最终化处理
    for (auto &voxel: voxels_) {
        auto &[info, content] = *voxel;
        content.finalize();
    }
}

template<typename VoxelContent>
size_t IncrementalVoxelMap<VoxelContent>::knnSearch(const Vec3f &point, size_t k, std::vector<size_t> *k_indices,
                                                    std::vector<Float> *k_sq_dists, Float max_sq_dist) const {
    // 获取体素坐标中心
    const Vec3i center = fastFloor(point * inv_leaf_size_);

    // 定义索引变换函数和结果容器
    size_t voxel_index         = 0;
    const auto index_transform = [this, &voxel_index](const size_t point_index) {
        return getIndex(voxel_index, point_index);
    };
    KnnResult result(k_indices, k_sq_dists, k, max_sq_dist, index_transform);

    // 以中心体素为起点，遍历周围的体素
    for (const auto &offset: search_offsets_) {
        // 根据体素坐标获取体素索引，不存在则跳过
        const Eigen::Vector3i coord = center + offset;
        const auto found            = voxel_index_map_.find(coord);
        if (found == voxel_index_map_.end()) {
            continue;
        }

        // 根据体素索引获取体素
        voxel_index                 = found->second;
        const auto &[info, content] = *voxels_[voxel_index];

        // 在体素中进行KNN搜索
        content.knnSearch(point, result);
    }

    return result.numFound();
}

template<typename VoxelContent>
bool IncrementalVoxelMap<VoxelContent>::isPointsEmpty() const {
    return voxels_.empty() ? true : voxels_.front()->second.points().empty();
}

template<typename VoxelContent>
const Vec3f &IncrementalVoxelMap<VoxelContent>::point(size_t i) const {
    LVINS_CHECK(voxelId(i) < voxels_.size(), "Invalid voxel id: {}", voxelId(i));
    LVINS_CHECK(pointId(i) < voxels_[voxelId(i)]->second.points().size(), "Invalid point id: {}", pointId(i));
    return voxels_[voxelId(i)]->second.points()[pointId(i)];
}

template<typename VoxelContent>
const Vec3f &IncrementalVoxelMap<VoxelContent>::normal(size_t i) const {
    LVINS_CHECK(voxelId(i) < voxels_.size(), "Invalid voxel id: {}", voxelId(i));
    LVINS_CHECK(pointId(i) < voxels_[voxelId(i)]->second.points().size(), "Invalid point id: {}", pointId(i));
    return voxels_[voxelId(i)]->second.normals()[pointId(i)];
}

template<typename VoxelContent>
const Mat33f &IncrementalVoxelMap<VoxelContent>::covariance(size_t i) const {
    LVINS_CHECK(voxelId(i) < voxels_.size(), "Invalid voxel id: {}", voxelId(i));
    LVINS_CHECK(pointId(i) < voxels_[voxelId(i)]->second.points().size(), "Invalid point id: {}", pointId(i));
    return voxels_[voxelId(i)]->second.covariances()[pointId(i)];
}

template<typename VoxelContent>
std::string IncrementalVoxelMap<VoxelContent>::print() const {
    return LVINS_FORMAT("Incremental voxel map:\n"
                        "  voxel size = {}\n"
                        "  lru horizon = {}\n"
                        "  lru clear cycle = {}\n"
                        "  search offsets = {}\n"
                        "{}",
                        1.0 / inv_leaf_size_, lru_horizon_, lru_clear_cycle_, search_offsets_.size(),
                        voxel_setting_.print());
}

template<typename VoxelContent>
size_t IncrementalVoxelMap<VoxelContent>::getIndex(size_t voxel_id, size_t point_id) {
    return voxel_id << point_id_bits_ | point_id;
}

template<typename VoxelContent>
size_t IncrementalVoxelMap<VoxelContent>::voxelId(size_t map_id) {
    return map_id >> point_id_bits_;
}

template<typename VoxelContent>
size_t IncrementalVoxelMap<VoxelContent>::pointId(size_t map_id) {
    return map_id & ((1ull << point_id_bits_) - 1);
}

} // namespace lvins
