#include "lvins_icp/ann/nearest_neighbor_searcher.h"
#include "lvins_icp/ann/yaml/nn_searcher_yaml_serialization.h"

namespace lvins {

NearestNeighborSearcher::Ptr NearestNeighborSearcher::loadFromYaml(const YAML::Node &config) {
    return YAML::get<NearestNeighborSearcher::Ptr>(config, "");
}

YAML::Node NearestNeighborSearcher::writeToYaml() const {
    YAML::Node node;
    node = *this;
    return node;
}

} // namespace lvins
