#include "lvins_lidar/lidar_rig.h"
#include "lvins_common/logger.h"
#include "lvins_common/path_helper.h"
#include "lvins_common/yaml/yaml_serialization.h"
#include "lvins_lidar/yaml/lidar_rig_yaml_serialization.h"

namespace lvins {

LidarRig::LidarRig(std::string label, const std::vector<LidarGeometryBase::sPtr> &lidars, std::vector<SE3f> T_bs_vec)
    : label_(std::move(label)), lidars_(lidars), T_bs_vec_(std::move(T_bs_vec)) {
    LVINS_CHECK(lidars_.size() == T_bs_vec_.size(), "Lidars size should be equal to T_bs_vec size!");
}

LidarRig::sPtr LidarRig::loadFromYaml(const std::string &config_file) {
    // 检查并转换路径（如果有需要）
    LVINS_CHECK(!config_file.empty(), "config_file should not be empty!");

    // 根据配置文件加载雷达
    const auto node = YAML::load(path_helper::completePath(config_file));
    return YAML::get<LidarRig::sPtr>(node, "");
}

void LidarRig::writeToYaml(const std::string &config_file) const {
    // 检查并转换路径（如果有需要）
    LVINS_CHECK(!config_file.empty(), "config_file should not be empty!");

    // 将雷达写入配置文件
    YAML::Node node;
    node = *this;
    YAML::dump(node, path_helper::completePath(config_file));
}

const std::string &LidarRig::label() const {
    return label_;
}

const LidarGeometryBase::sPtr &LidarRig::lidar(size_t idx) const {
    LVINS_CHECK(idx < lidars_.size(), "Index should be less than lidars size!");
    return lidars_[idx];
}

size_t LidarRig::size() const {
    return lidars_.size();
}

const SE3f &LidarRig::Tbs(size_t idx) const {
    LVINS_CHECK(idx < T_bs_vec_.size(), "Index should be less than T_bs_vec size!");
    return T_bs_vec_[idx];
}

void LidarRig::setTbs(size_t idx, const SE3f &T_bs) {
    LVINS_CHECK(idx < T_bs_vec_.size(), "Index should be less than T_bs_vec size!");
    T_bs_vec_[idx] = T_bs;
}

const std::vector<SE3f> &LidarRig::Tbs() const {
    return T_bs_vec_;
}

void LidarRig::setTbs(const std::vector<SE3f> &T_bs) {
    LVINS_CHECK(T_bs_vec_.size() == T_bs.size(), "T_bs size and T_bs_vec should have the same size!");
    T_bs_vec_ = T_bs;
}

std::string LidarRig::print() const {
    std::string ret;
    auto end = LVINS_FORMAT_TO(std::back_inserter(ret), "Lidar rig {}:", label_);
    for (size_t i = 0; i < lidars_.size(); ++i) {
        end = LVINS_FORMAT_TO(end, "\nLidar #{}:\n{}\nT_bs[{}] = {}", i, lidars_[i]->print(), i,
                              LVINS_MATRIX_FMT(T_bs_vec_[i].matrix()));
    }
    return ret;
}

} // namespace lvins
