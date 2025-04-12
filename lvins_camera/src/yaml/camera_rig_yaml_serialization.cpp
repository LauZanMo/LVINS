#include "lvins_camera/yaml/camera_rig_yaml_serialization.h"
#include "lvins_camera/yaml/camera_yaml_serialization.h"
#include "lvins_common/yaml/yaml_eigen_serialization.h"

using namespace lvins;

namespace YAML {

Node convert<CameraRig>::encode(const CameraRig &camera_rig) {
    Node rig_node;
    rig_node["label"] = camera_rig.label();

    Node cameras_node;
    for (size_t camera_idx = 0; camera_idx < camera_rig.size(); ++camera_idx) {
        Node camera_node;
        camera_node["camera"] = camera_rig.camera(camera_idx);
        camera_node["T_bs"]   = camera_rig.Tbs(camera_idx).matrix();
        cameras_node.push_back(camera_node);
    }

    rig_node["cameras"] = cameras_node;

    return rig_node;
}

bool convert<CameraRig>::decode(const Node & /*node*/, CameraRig & /*camera_rig*/) {
    LVINS_ERROR("Unsupported action: Directly decode with CameraRig object, try to decode with "
                "CameraRig::sPtr!");
    return false;
}

Node convert<CameraRig::sPtr>::encode(const CameraRig::sPtr &camera_rig) {
    LVINS_CHECK(camera_rig != nullptr, "The camera rig is nullptr!");
    return convert<CameraRig>::encode(*camera_rig);
}

bool convert<CameraRig::sPtr>::decode(const Node &node, CameraRig::sPtr &camera_rig) {
    LVINS_CHECK(node.IsMap(), "Unable to parse the camera rig because the node is not a map!");

    // 加载常规参数
    const auto label = YAML::get<std::string>(node, "label");

    // 检查相机队列节点
    const auto cameras_node = node["cameras"];
    LVINS_CHECK(cameras_node.IsSequence(), "Cameras node should be a sequence!");
    const auto num_cameras = cameras_node.size();
    LVINS_CHECK(num_cameras > 0, "Number of cameras should be greater than 0!");

    // 加载所有相机及其外参
    std::vector<CameraGeometryBase::sPtr> cameras;
    std::vector<SE3f> T_bs_vec;
    for (size_t camera_idx = 0; camera_idx < num_cameras; ++camera_idx) {
        const auto camera_node = cameras_node[camera_idx];
        LVINS_CHECK(camera_node && camera_node.IsMap(), "Unable to get camera node for camera #{}!", camera_idx);

        auto camera   = YAML::get<CameraGeometryBase::sPtr>(camera_node, "camera");
        auto T_bs_raw = YAML::get<Mat44f>(camera_node, "T_bs");
        camera->setId(static_cast<int>(camera_idx));

        // 此操作是为了防止输入旋转矩阵非正交
        Quatf q_bs(T_bs_raw.block<3, 3>(0, 0));
        q_bs.normalize();
        T_bs_raw.block<3, 3>(0, 0) = q_bs.toRotationMatrix();
        SE3f T_bs(T_bs_raw);

        cameras.push_back(std::move(camera));
        T_bs_vec.push_back(std::move(T_bs));
    }

    // 实例化相机组
    camera_rig = std::make_shared<CameraRig>(label, cameras, T_bs_vec);

    return true;
}

} // namespace YAML
