#include "lvins_odometry/init/initializer_base.h"
#include "lvins_common/yaml/yaml_eigen_serialization.h"
#include "lvins_odometry/init/dynamic_initializer.h"
#include "lvins_odometry/init/static_initializer.h"

namespace lvins {

InitializerBase::InitializerBase(const NoiseParameters &noise_params, const PointCloudAligner &aligner, Vec3f g_w)
    : noise_params_(noise_params), aligner_(aligner), g_w_(std::move(g_w)) {}

InitializerBase::Ptr InitializerBase::loadFromYaml(const YAML::Node &config, const NoiseParameters &noise_params,
                                                   const PointCloudAligner &aligner, Vec3f g_w) {
    const auto type       = YAML::get<std::string>(config, "type");
    const auto parameters = YAML::get<VecXf>(config, "parameters");

    if (type == "static") {
        return std::make_unique<StaticInitializer>(parameters, noise_params, aligner, std::move(g_w));
    }

    if (type == "dynamic") {
        return std::make_unique<DynamicInitializer>(parameters, noise_params, aligner, std::move(g_w));
    }

    LVINS_FATAL("Invalid initializer type: {}", type);
}

YAML::Node InitializerBase::writeToYaml() const {
    YAML::Node node;
    node["type"]       = type();
    node["parameters"] = parameters();
    return node;
}

} // namespace lvins
