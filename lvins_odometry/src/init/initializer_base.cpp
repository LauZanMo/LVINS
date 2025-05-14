#include "lvins_odometry/init/initializer_base.h"
#include "lvins_common/yaml/yaml_eigen_serialization.h"
#include "lvins_odometry/init/static_initializer.h"

namespace lvins {

InitializerBase::InitializerBase(Vec3f g_w) : g_w_(std::move(g_w)) {}

InitializerBase::uPtr InitializerBase::loadFromYaml(const YAML::Node &config, Vec3f g_w) {
    const auto type       = YAML::get<std::string>(config, "type");
    const auto parameters = YAML::get<VecXf>(config, "parameters");

    if (type == "static") {
        return std::make_unique<StaticInitializer>(parameters, std::move(g_w));
    }

    if (type == "dynamic") {
        // return std::make_unique<DynamicInitializer>(parameters, std::move(g_w));
        LVINS_FATAL("Not implemented!");
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
