#include "lvins_icp/ann/yaml/nn_searcher_yaml_serialization.h"

#include "lvins_icp/ann/flat_container.h"
#include "lvins_icp/ann/incremental_voxel_map.h"

using namespace lvins;

namespace YAML {

Node convert<NearestNeighborSearcher>::encode(const NearestNeighborSearcher &nn_searcher) {
    Node node;
    if (internal::encodeIncrementalVoxelMap<FlatContainer>(nn_searcher, &node)) {
        LVINS_FATAL("Unsupported nearest neighbor searcher type to encode!");
    }

    return node;
}

bool convert<NearestNeighborSearcher>::decode(const Node & /*node*/, NearestNeighborSearcher & /*nn_searcher*/) {
    LVINS_ERROR("Unsupported action: Directly decode with NearestNeighborSearcher object, try to decode with "
                "NearestNeighborSearcher::Ptr!");
    return false;
}

Node convert<std::shared_ptr<NearestNeighborSearcher>>::encode(const NearestNeighborSearcher::Ptr &nn_searcher) {
    LVINS_CHECK(nullptr != nn_searcher, "The nearest neighbor searcher is nullptr!");
    return convert<NearestNeighborSearcher>::encode(*nn_searcher);
}

bool convert<std::shared_ptr<NearestNeighborSearcher>>::decode(const Node &node,
                                                               NearestNeighborSearcher::Ptr &nn_searcher) {
    LVINS_CHECK(node.IsMap(), "Unable to parse the nearest neighbor searcher because the node is not a map!");
    const auto type = YAML::get<std::string>(node, "type");

    // 实例化最近邻搜索器
    if (type == "ivox") {
        const auto leaf_size       = YAML::get<Float>(node, "leaf_size");
        const auto lru_horizon     = YAML::get<size_t>(node, "lru_horizon");
        const auto lru_clear_cycle = YAML::get<size_t>(node, "lru_clear_cycle");
        const auto search_offsets  = YAML::get<size_t>(node, "search_offsets");
        const auto content_type    = YAML::get<std::string>(node["voxel_content"], "type");

        if (content_type == "flat") {
            const auto min_sq_dist_in_cell    = YAML::get<Float>(node["voxel_content"], "min_sq_dist_in_cell");
            const auto max_num_points_in_cell = YAML::get<size_t>(node["voxel_content"], "max_num_points_in_cell");
            FlatContainer::Setting setting(min_sq_dist_in_cell, max_num_points_in_cell);
            nn_searcher = std::make_shared<IncrementalVoxelMap<FlatContainer>>(setting, leaf_size, lru_horizon,
                                                                               lru_clear_cycle, search_offsets);
        } else {
            LVINS_FATAL("Unsupported voxel content type: {}!", content_type);
        }
    } else {
        LVINS_FATAL("Unsupported nearest neighbor searcher type: {}!", type);
    }

    return true;
}

namespace internal {

template<typename VoxelContent>
void encodeVoxelContent(const VoxelContent &voxel_content, Node *voxel_content_node);

template<>
void encodeVoxelContent(const FlatContainer::Setting &voxel_content, Node *voxel_content_node) {
    LVINS_CHECK(voxel_content_node != nullptr, "The voxel content node is nullptr!");
    (*voxel_content_node)["type"]                   = "flat";
    (*voxel_content_node)["min_sq_dist_in_cell"]    = voxel_content.max_num_points_in_cell;
    (*voxel_content_node)["max_num_points_in_cell"] = voxel_content.min_sq_dist_in_cell;
}

template<typename VoxelContent>
bool encodeIncrementalVoxelMap(const NearestNeighborSearcher &nn_searcher, Node *nn_searcher_node) {
    using SearcherConstPtr = const IncrementalVoxelMap<VoxelContent> *;

    LVINS_CHECK(nn_searcher_node != nullptr, "The nearest neighbor searcher node should not be nullptr!");
    if (auto incremental_voxel_map = dynamic_cast<SearcherConstPtr>(&nn_searcher)) {
        (*nn_searcher_node)["type"]            = "ivox";
        (*nn_searcher_node)["leaf_size"]       = incremental_voxel_map->leafSize();
        (*nn_searcher_node)["lru_horizon"]     = incremental_voxel_map->lruHorizon();
        (*nn_searcher_node)["lru_clear_cycle"] = incremental_voxel_map->lruClearCycle();
        (*nn_searcher_node)["search_offsets"]  = incremental_voxel_map->searchOffsets();

        Node voxel_content_node;
        encodeVoxelContent(incremental_voxel_map->voxelSetting(), &voxel_content_node);
        (*nn_searcher_node)["voxel_content"] = voxel_content_node;

        return true;
    }

    return false;
}

} // namespace internal
} // namespace YAML
