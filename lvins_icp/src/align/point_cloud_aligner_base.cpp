#include "lvins_icp/align/point_cloud_aligner_base.h"
#include "lvins_icp/align/yaml/aligner_yaml_serialization.h"

namespace lvins {

PointCloudAlignerBase::PointCloudAlignerBase(const Optimizer &optimizer, const TerminateCriteria &criteria)
    : optimizer_(optimizer), criteria_(criteria) {}

PointCloudAlignerBase::Ptr PointCloudAlignerBase::loadFromYaml(const YAML::Node &config) {
    return YAML::get<PointCloudAlignerBase::Ptr>(config, "");
}

YAML::Node PointCloudAlignerBase::writeToYaml() const {
    YAML::Node node;
    node = *this;
    return node;
}

const PointCloudAlignerBase::Optimizer &PointCloudAlignerBase::optimizer() const {
    return optimizer_;
}

const PointCloudAlignerBase::TerminateCriteria &PointCloudAlignerBase::criteria() const {
    return criteria_;
}

} // namespace lvins
