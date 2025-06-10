#include "lvins_odometry/drift_detector.h"

namespace lvins {

DriftDetector::DriftDetector(const YAML::Node &config) {
    max_rot_err_   = YAML::get<Float>(config, "max_rot_err");
    max_trans_err_ = YAML::get<Float>(config, "max_trans_err");
    max_vel_err_   = YAML::get<Float>(config, "max_vel_err");
    verbosity_     = YAML::get<bool>(config, "verbosity");
}

YAML::Node DriftDetector::writeToYaml() const {
    YAML::Node node;
    node["max_rot_err"]   = max_rot_err_;
    node["max_trans_err"] = max_trans_err_;
    node["max_vel_err"]   = max_vel_err_;
    node["verbosity"]     = verbosity_;
    return node;
}

bool DriftDetector::detect(const NavState &predict, const NavState &measure) const {
    const Float rot_err   = (predict.T.so3().inverse() * measure.T.so3()).log().norm();
    const Float trans_err = (predict.T.translation() - measure.T.translation()).norm();
    const Float vel_err   = (predict.vel - measure.vel).norm();

    // 打印调试信息
    if (verbosity_) {
        LVINS_DEBUG("Drift detector:\n"
                    "  rotation error = {:.6f}\n"
                    "  translation error = {:.6f}\n"
                    "  velocity error = {:.6f}",
                    rot_err, trans_err, vel_err);
    }

    // 检测系统漂移
    if (rot_err > max_rot_err_ || trans_err > max_trans_err_ || vel_err > max_vel_err_) {
        LVINS_WARN("System drift detected!\n"
                   "  rotation error = {:.6f}\n"
                   "  translation error = {:.6f}\n"
                   "  velocity error = {:.6f}",
                   rot_err, trans_err, vel_err);
        return true;
    }

    return false;
}

std::string DriftDetector::print() const {
    return LVINS_FORMAT("Drift detector:\n"
                        "  max rotation error = {:.3f}\n"
                        "  max translation error = {:.3f}\n"
                        "  max velocity error = {:.3f}\n"
                        "  verbosity = {}",
                        max_rot_err_, max_trans_err_, max_vel_err_, verbosity_);
}

} // namespace lvins
