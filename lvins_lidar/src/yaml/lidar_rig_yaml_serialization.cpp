#include "lvins_lidar/yaml/lidar_rig_yaml_serialization.h"
#include "lvins_common/yaml/yaml_eigen_serialization.h"
#include "lvins_lidar/yaml/lidar_yaml_serialization.h"

using namespace lvins;

namespace YAML {

Node convert<LidarRig>::encode(const LidarRig &lidar_rig) {
    Node rig_node;
    rig_node["label"] = lidar_rig.label();

    Node lidars_node;
    for (size_t lidar_idx = 0; lidar_idx < lidar_rig.size(); ++lidar_idx) {
        Node lidar_node;
        lidar_node["lidar"] = lidar_rig.lidar(lidar_idx);
        lidar_node["T_bs"]  = lidar_rig.Tbs(lidar_idx).matrix();
        lidars_node.push_back(lidar_node);
    }

    rig_node["lidars"] = lidars_node;

    return rig_node;
}

bool convert<LidarRig>::decode(const Node & /*node*/, LidarRig & /*lidar_rig*/) {
    LVINS_ERROR("Unsupported action: Directly decode with LidarRig object, try to decode with "
                "LidarRig::Ptr!");
    return false;
}

Node convert<LidarRig::Ptr>::encode(const LidarRig::Ptr &lidar_rig) {
    LVINS_CHECK(lidar_rig != nullptr, "The lidar rig is nullptr!");
    return convert<LidarRig>::encode(*lidar_rig);
}

bool convert<LidarRig::Ptr>::decode(const Node &node, LidarRig::Ptr &lidar_rig) {
    LVINS_CHECK(node.IsMap(), "Unable to parse the lidar rig because the node is not a map!");

    // 加载常规参数
    const auto label = YAML::get<std::string>(node, "label");

    // 检查雷达队列节点
    const auto lidars_node = node["lidars"];
    LVINS_CHECK(lidars_node.IsSequence(), "Lidars node should be a sequence!");
    const auto num_lidars = lidars_node.size();
    LVINS_CHECK(num_lidars > 0, "Number of lidars should be greater than 0!");

    // 加载所有雷达及其外参
    std::vector<LidarGeometryBase::Ptr> lidars;
    std::vector<SE3f> T_bs_vec;
    for (size_t lidar_idx = 0; lidar_idx < num_lidars; ++lidar_idx) {
        const auto lidar_node = lidars_node[lidar_idx];
        LVINS_CHECK(lidar_node && lidar_node.IsMap(), "Unable to get lidar node for lidar #{}!", lidar_idx);

        auto lidar    = YAML::get<LidarGeometryBase::Ptr>(lidar_node, "lidar");
        auto T_bs_raw = YAML::get<Mat44f>(lidar_node, "T_bs");
        lidar->setId(static_cast<int>(lidar_idx));

        // 此操作是为了防止输入旋转矩阵非正交
        Quatf q_bs(T_bs_raw.block<3, 3>(0, 0));
        q_bs.normalize();
        T_bs_raw.block<3, 3>(0, 0) = q_bs.toRotationMatrix();
        SE3f T_bs(T_bs_raw);

        lidars.push_back(std::move(lidar));
        T_bs_vec.push_back(std::move(T_bs));
    }

    // 实例化雷达组
    lidar_rig = std::make_shared<LidarRig>(label, lidars, T_bs_vec);

    return true;
}

} // namespace YAML
