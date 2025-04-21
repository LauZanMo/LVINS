#include "lvins_camera/yaml/camera_yaml_serialization.h"
#include "lvins_camera/camera_geometry.h"
#include "lvins_common/path_helper.h"
#include "lvins_common/yaml/yaml_eigen_serialization.h"

#include "lvins_camera/projection/omni_projection.h"
#include "lvins_camera/projection/pinhole_projection.h"

#include "lvins_camera/distortion/equidistant_distortion.h"
#include "lvins_camera/distortion/no_distortion.h"
#include "lvins_camera/distortion/radial_tangential_distortion.h"

#include <opencv2/imgcodecs.hpp>

using namespace lvins;

namespace YAML {

Node convert<CameraGeometryBase>::encode(const CameraGeometryBase &camera) {
    Node node;
    node["label"]  = camera.label();
    node["width"]  = camera.width();
    node["height"] = camera.height();

    if (!internal::encodePinhole<NoDistortion>(camera, &node) &&
        !internal::encodePinhole<RadialTangentialDistortion>(camera, &node) &&
        !internal::encodePinhole<EquidistantDistortion>(camera, &node) &&
        !internal::encodeOmni<NoDistortion>(camera, &node) &&
        !internal::encodeOmni<RadialTangentialDistortion>(camera, &node) &&
        !internal::encodeOmni<EquidistantDistortion>(camera, &node)) {
        LVINS_FATAL("Unsupported camera geometry type to encode!");
    }

    node["mask"] = camera.maskFileName();

    return node;
}

bool convert<CameraGeometryBase>::decode(const Node & /*node*/, CameraGeometryBase & /*camera*/) {
    LVINS_ERROR("Unsupported action: Directly decode with CameraGeometryBase object, try to decode with "
                "CameraGeometryBase::sPtr!");
    return false;
}

Node convert<CameraGeometryBase::sPtr>::encode(const CameraGeometryBase::sPtr &camera) {
    LVINS_CHECK(camera != nullptr, "The camera is nullptr!");
    return convert<CameraGeometryBase>::encode(*camera);
}

bool convert<CameraGeometryBase::sPtr>::decode(const Node &node, CameraGeometryBase::sPtr &camera) {
    LVINS_CHECK(node.IsMap(), "Unable to parse the camera because the node is not a map!");

    // 加载常规参数
    const auto label             = YAML::get<std::string>(node, "label");
    const auto width             = YAML::get<int>(node, "width");
    const auto height            = YAML::get<int>(node, "height");
    const auto projection_type   = YAML::get<std::string>(node["projection"], "type");
    const auto distortion_type   = YAML::get<std::string>(node["distortion"], "type");
    const auto projection_params = YAML::get<VecXf>(node["projection"], "parameters");
    const auto distortion_params = YAML::get<VecXf>(node["distortion"], "parameters");

    // 加载掩模
    cv::Mat mask;
    const auto mask_file_name = YAML::get<std::string>(node, "mask");
    if (!mask_file_name.empty()) {
        mask = cv::imread(path_helper::completePath(mask_file_name), cv::IMREAD_GRAYSCALE);
    }

    // 实例化相机
    if (projection_type == "pinhole" && distortion_type == "none") {
        camera = std::make_shared<CameraGeometry<PinholeProjection<NoDistortion>>>(
                label, width, height, mask, mask_file_name, PinholeProjection(projection_params, NoDistortion()));
    } else if (projection_type == "pinhole" && distortion_type == "radial-tangential") {
        camera = std::make_shared<CameraGeometry<PinholeProjection<RadialTangentialDistortion>>>(
                label, width, height, mask, mask_file_name,
                PinholeProjection(projection_params, RadialTangentialDistortion(distortion_params)));
    } else if (projection_type == "pinhole" && distortion_type == "equidistant") {
        camera = std::make_shared<CameraGeometry<PinholeProjection<EquidistantDistortion>>>(
                label, width, height, mask, mask_file_name,
                PinholeProjection(projection_params, EquidistantDistortion(distortion_params)));
    } else if (projection_type == "omni" && distortion_type == "none") {
        camera = std::make_shared<CameraGeometry<OmniProjection<NoDistortion>>>(
                label, width, height, mask, mask_file_name, OmniProjection(projection_params, NoDistortion()));
    } else if (projection_type == "omni" && distortion_type == "radial-tangential") {
        camera = std::make_shared<CameraGeometry<OmniProjection<RadialTangentialDistortion>>>(
                label, width, height, mask, mask_file_name,
                OmniProjection(projection_params, RadialTangentialDistortion(distortion_params)));
    } else if (projection_type == "omni" && distortion_type == "equidistant") {
        camera = std::make_shared<CameraGeometry<OmniProjection<EquidistantDistortion>>>(
                label, width, height, mask, mask_file_name,
                OmniProjection(projection_params, EquidistantDistortion(distortion_params)));
    } else {
        LVINS_FATAL("Unsupported combination of projection and distortion types: {} and {}!", projection_type,
                    distortion_type);
    }

    return true;
}

namespace internal {

template<typename Distortion>
void encodeDistortion(const Distortion &distortion, Node *distortion_node);

template<>
void encodeDistortion(const NoDistortion & /*distortion*/, Node *distortion_node) {
    LVINS_CHECK(distortion_node != nullptr, "The distortion node is nullptr!");
    (*distortion_node)["type"]       = "none";
    (*distortion_node)["parameters"] = VecXf(0, 1);
}

template<>
void encodeDistortion(const RadialTangentialDistortion &distortion, Node *distortion_node) {
    LVINS_CHECK(distortion_node != nullptr, "The distortion node is nullptr!");
    (*distortion_node)["type"]       = "radial-tangential";
    (*distortion_node)["parameters"] = distortion.distortionParameters();
}

template<>
void encodeDistortion(const EquidistantDistortion &distortion, Node *distortion_node) {
    LVINS_CHECK(distortion_node != nullptr, "The distortion node is nullptr!");
    (*distortion_node)["type"]       = "equidistant";
    (*distortion_node)["parameters"] = distortion.distortionParameters();
}

template<typename Distortion>
bool encodePinhole(const CameraGeometryBase &camera, Node *camera_node) {
    using GeometryConstPtr = const CameraGeometry<PinholeProjection<Distortion>> *;

    LVINS_CHECK(camera_node != nullptr, "The camera node should not be nullptr!");
    if (auto pinhole_camera = dynamic_cast<GeometryConstPtr>(&camera)) {
        (*camera_node)["projection"]["type"]       = "pinhole";
        (*camera_node)["projection"]["parameters"] = pinhole_camera->intrinsicParameters();

        Node distortion_node;
        encodeDistortion(pinhole_camera->projection().distortion(), &distortion_node);
        (*camera_node)["distortion"] = distortion_node;

        return true;
    }

    return false;
}

template<typename Distortion>
bool encodeOmni(const CameraGeometryBase &camera, Node *camera_node) {
    using GeometryConstPtr = const CameraGeometry<OmniProjection<Distortion>> *;

    LVINS_CHECK(camera_node != nullptr, "The camera node should not be nullptr!");
    if (auto omni_camera = dynamic_cast<GeometryConstPtr>(&camera)) {
        (*camera_node)["projection"]["type"]       = "omni";
        (*camera_node)["projection"]["parameters"] = omni_camera->intrinsicParameters();

        Node distortion_node;
        encodeDistortion(omni_camera->projection().distortion(), &distortion_node);
        (*camera_node)["distortion"] = distortion_node;

        return true;
    }

    return false;
}

} // namespace internal
} // namespace YAML
