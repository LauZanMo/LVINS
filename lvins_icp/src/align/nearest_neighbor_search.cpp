#include "lvins_icp/align/nearest_neighbor_search.h"

namespace lvins {

NearestNeighborSearch::NearestNeighborSearch(const YAML::Node &config) {
    const auto leaf_size              = YAML::get<double>(config, "leaf_size");
    lru_horizon_                      = YAML::get<int>(config, "lru_horizon");
    lru_clear_cycle_                  = YAML::get<int>(config, "lru_clear_cycle");
    search_offset_                    = YAML::get<int>(config, "search_offset");
    const auto min_dist_in_cell       = YAML::get<double>(config["voxel_content"], "min_dist_in_cell");
    const auto max_num_points_in_cell = YAML::get<size_t>(config["voxel_content"], "max_num_points_in_cell");
    LVINS_CHECK(leaf_size > 0.0, "Leaf size should be positive!");

    nn_search_ = std::make_shared<Search>(leaf_size);
    nn_search_->set_lru_horizon(lru_horizon_);
    nn_search_->set_lru_clear_cycle(lru_clear_cycle_);
    nn_search_->set_neighbor_voxel_mode(search_offset_);
    nn_search_->voxel_insertion_setting().set_min_dist_in_cell(min_dist_in_cell);
    nn_search_->voxel_insertion_setting().set_max_num_points_in_cell(max_num_points_in_cell);
}

YAML::Node NearestNeighborSearch::writeToYaml() const {
    YAML::Node node;
    node["leaf_size"]                         = nn_search_->leaf_size();
    node["lru_horizon"]                       = lru_horizon_;
    node["lru_clear_cycle"]                   = lru_clear_cycle_;
    node["search_offset"]                     = search_offset_;
    node["voxel_content"]["min_dist_in_cell"] = std::sqrt(nn_search_->voxel_insertion_setting().min_sq_dist_in_cell);
    node["voxel_content"]["max_num_points_in_cell"] = nn_search_->voxel_insertion_setting().max_num_points_in_cell;
    return node;
}
void NearestNeighborSearch::reset() {
    nn_search_->clear();
}

NearestNeighborSearch::Search::ConstPtr NearestNeighborSearch::getSearch() const {
    return nn_search_;
}

void NearestNeighborSearch::insert(const PointCloud &point_cloud, const SE3f &T) {
    if (!point_cloud.size()) {
        return;
    }

    const auto trans_point_cloud = transform(point_cloud, T);
    nn_search_->insert(*trans_point_cloud);
}

std::string NearestNeighborSearch::print() const {
    return LVINS_FORMAT("Nearest neighbor search:\n"
                        "  leaf size = {}\n"
                        "  lru horizon = {}\n"
                        "  lru clear cycle = {}\n"
                        "  search offset = {}\n"
                        "  voxel content min dist in cell = {}\n"
                        "  voxel content max num points in cell = {}",
                        nn_search_->leaf_size(), lru_horizon_, lru_clear_cycle_, search_offset_,
                        std::sqrt(nn_search_->voxel_insertion_setting().min_sq_dist_in_cell),
                        nn_search_->voxel_insertion_setting().max_num_points_in_cell);
}

} // namespace lvins
