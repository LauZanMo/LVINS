#include "lvins_icp/align/yaml/aligner_yaml_serialization.h"

#include "lvins_icp/align/point_cloud_aligner.h"
#include "lvins_icp/factor/gicp_factor.h"

using namespace lvins;

namespace YAML {

Node convert<PointCloudAlignerBase>::encode(const PointCloudAlignerBase &aligner) {
    Node node;
    if (!internal::encodePointCloudAligner<GICPFactor>(aligner, &node)) {
        LVINS_FATAL("Unsupported point cloud aligner type to encode!");
    }

    return node;
}

bool convert<PointCloudAlignerBase>::decode(const Node & /*node*/, PointCloudAlignerBase & /*aligner*/) {
    LVINS_ERROR("Unsupported action: Directly decode with PointCloudAlignerBase object, try to decode with "
                "PointCloudAlignerBase::Ptr!");
    return false;
}

Node convert<std::shared_ptr<PointCloudAlignerBase>>::encode(const PointCloudAlignerBase::Ptr &aligner) {
    LVINS_CHECK(aligner != nullptr, "The point cloud aligner is nullptr!");
    return convert<PointCloudAlignerBase>::encode(*aligner);
}

bool convert<std::shared_ptr<PointCloudAlignerBase>>::decode(const Node &node, PointCloudAlignerBase::Ptr &aligner) {
    LVINS_CHECK(node.IsMap(), "Unable to parse the point cloud aligner because the node is not a map!");

    const auto max_iterations       = YAML::get<size_t>(node["optimizer"], "max_iterations");
    const auto max_inner_iterations = YAML::get<size_t>(node["optimizer"], "max_inner_iterations");
    const auto init_lambda          = YAML::get<double>(node["optimizer"], "init_lambda");
    const auto lambda_factor        = YAML::get<double>(node["optimizer"], "lambda_factor");

    const auto trans_eps = YAML::get<double>(node["criteria"], "trans_eps");
    const auto rot_eps   = YAML::get<double>(node["criteria"], "rot_eps");

    PointCloudAlignerBase::Optimizer optimizer(max_iterations, max_inner_iterations, init_lambda, lambda_factor);
    PointCloudAlignerBase::TerminateCriteria criteria(trans_eps, rot_eps);

    // 实例化点云配准器
    const auto factor_type = YAML::get<std::string>(node["factor"], "type");
    if (factor_type == "gicp") {
        const auto max_search_sq_dist = YAML::get<float>(node["factor"], "max_search_sq_dist");

        GICPFactor::Setting factor_setting(max_search_sq_dist);
        aligner = std::make_shared<PointCloudAligner<GICPFactor>>(optimizer, criteria, factor_setting);
    } else {
        LVINS_FATAL("Unsupported point cloud aligner factor type: {}!", factor_type);
    }

    return true;
}

namespace internal {

template<typename RegistrateFactor>
void encodeRegistrateFactor(const RegistrateFactor &factor, Node *registrate_factor_node);

template<>
void encodeRegistrateFactor(const GICPFactor::Setting &factor, Node *registrate_factor_node) {
    LVINS_CHECK(registrate_factor_node != nullptr, "The registrate factor node is nullptr!");
    (*registrate_factor_node)["type"]               = "gicp";
    (*registrate_factor_node)["max_search_sq_dist"] = factor.max_search_sq_dist;
}

template<typename RegistrateFactor>
bool encodePointCloudAligner(const PointCloudAlignerBase &aligner_base, Node *aligner_node) {
    using AlignerConstPtr = const PointCloudAligner<RegistrateFactor> *;

    LVINS_CHECK(aligner_node != nullptr, "The point cloud aligner base node should not be nullptr!");

    if (auto aligner = dynamic_cast<AlignerConstPtr>(&aligner_base)) {
        Node optimizer_node;
        optimizer_node["max_iterations"]       = aligner->optimizer().maxIterations();
        optimizer_node["max_inner_iterations"] = aligner->optimizer().maxInnerIterations();
        optimizer_node["init_lambda"]          = aligner->optimizer().initLambda();
        optimizer_node["lambda_factor"]        = aligner->optimizer().lambdaFactor();
        (*aligner_node)["optimizer"]           = optimizer_node;

        Node criteria_node;
        criteria_node["trans_eps"]  = aligner->criteria().transEpsilon();
        criteria_node["rot_eps"]    = aligner->criteria().rotEpsilon();
        (*aligner_node)["criteria"] = criteria_node;

        Node registrate_factor_node;
        encodeRegistrateFactor(aligner->factorSetting(), &registrate_factor_node);
        (*aligner_node)["factor"] = registrate_factor_node;

        return true;
    }

    return false;
}

} // namespace internal
} // namespace YAML
