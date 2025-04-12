#include "lvins_camera/camera_rig.h"
#include "lvins_camera/yaml/camera_rig_yaml_serialization.h"
#include "lvins_common/logger.h"
#include "lvins_common/path_helper.h"
#include "lvins_common/yaml/yaml_serialization.h"

namespace lvins {

CameraRig::CameraRig(std::string label, const std::vector<CameraGeometryBase::sPtr> &cameras,
                     std::vector<SE3f> T_bs_vec)
    : label_(std::move(label)), cameras_(cameras), T_bs_vec_(std::move(T_bs_vec)) {
    LVINS_CHECK(cameras_.size() == T_bs_vec_.size(), "Cameras size should be equal to T_bs_vec size!");
}

CameraRig::sPtr CameraRig::loadFromYaml(const std::string &config_file) {
    // 检查并转换路径（如果有需要）
    LVINS_CHECK(!config_file.empty(), "config_file should not be empty!");

    // 根据配置文件加载相机
    const auto node = YAML::load(path_helper::completePath(config_file));
    return YAML::get<CameraRig::sPtr>(node, "");
}

void CameraRig::writeToYaml(const std::string &config_file) const {
    // 检查并转换路径（如果有需要）
    LVINS_CHECK(!config_file.empty(), "config_file should not be empty!");

    // 将相机写入配置文件
    YAML::Node node;
    node = *this;
    YAML::dump(node, path_helper::completePath(config_file));
}

const std::string &CameraRig::label() const {
    return label_;
}

const CameraGeometryBase::sPtr &CameraRig::camera(size_t idx) const {
    LVINS_CHECK(idx < cameras_.size(), "Index should be less than cameras size!");
    return cameras_[idx];
}

size_t CameraRig::size() const {
    return cameras_.size();
}

const SE3f &CameraRig::Tbs(size_t idx) const {
    LVINS_CHECK(idx < T_bs_vec_.size(), "Index should be less than T_bs_vec size!");
    return T_bs_vec_[idx];
}

void CameraRig::setTbs(size_t idx, const SE3f &T_bs) {
    LVINS_CHECK(idx < T_bs_vec_.size(), "Index should be less than T_bs_vec size!");
    T_bs_vec_[idx] = T_bs;
}

const std::vector<SE3f> &CameraRig::Tbs() const {
    return T_bs_vec_;
}

void CameraRig::setTbs(const std::vector<SE3f> &T_bs) {
    LVINS_CHECK(T_bs_vec_.size() == T_bs.size(), "T_bs size and T_bs_vec should have the same size!");
    T_bs_vec_ = T_bs;
}

std::string CameraRig::print() const {
    std::string ret;
    auto end = LVINS_FORMAT_TO(std::back_inserter(ret), "Camera rig {}:", label_);
    for (size_t i = 0; i < cameras_.size(); ++i) {
        end = LVINS_FORMAT_TO(end, "\nCamera #{}:\n{}\nT_bs[{}] = {}", i, cameras_[i]->print(), i,
                              LVINS_MATRIX_FMT(T_bs_vec_[i].matrix()));
    }
    return ret;
}

} // namespace lvins
