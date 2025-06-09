#include "lvins_common/noise_parameters.h"

namespace lvins {

NoiseParameters::NoiseParameters(const YAML::Node &config) {
    // 先验参数
    prior_roll_pitch_std = YAML::get<double>(config, "prior_roll_pitch_std");
    prior_yaw_std        = YAML::get<double>(config, "prior_yaw_std");
    prior_pos_std        = YAML::get<double>(config, "prior_pos_std");
    prior_vel_std        = YAML::get<double>(config, "prior_vel_std");
    prior_gyr_bias_std   = YAML::get<double>(config, "prior_gyr_bias_std");
    prior_acc_bias_std   = YAML::get<double>(config, "prior_acc_bias_std");

    // 积分参数
    gyr_std         = YAML::get<double>(config, "gyr_std");
    acc_std         = YAML::get<double>(config, "acc_std");
    gyr_bias_std    = YAML::get<double>(config, "gyr_bias_std");
    acc_bias_std    = YAML::get<double>(config, "acc_bias_std");
    integration_std = YAML::get<double>(config, "integration_std");
    init_bias_std   = YAML::get<double>(config, "init_bias_std");

    // 里程计参数
    odom_rot_std   = YAML::get<double>(config, "odom_rot_std");
    odom_trans_std = YAML::get<double>(config, "odom_trans_std");

    // 外参参数
    ext_rot_std   = YAML::get<double>(config, "ext_rot_std");
    ext_trans_std = YAML::get<double>(config, "ext_trans_std");
}

YAML::Node NoiseParameters::writeToYaml() const {
    YAML::Node node;

    node["prior_roll_pitch_std"] = prior_roll_pitch_std;
    node["prior_yaw_std"]        = prior_yaw_std;
    node["prior_pos_std"]        = prior_pos_std;
    node["prior_vel_std"]        = prior_vel_std;
    node["prior_gyr_bias_std"]   = prior_gyr_bias_std;
    node["prior_acc_bias_std"]   = prior_acc_bias_std;

    node["gyr_std"]         = gyr_std;
    node["acc_std"]         = acc_std;
    node["gyr_bias_std"]    = gyr_bias_std;
    node["acc_bias_std"]    = acc_bias_std;
    node["integration_std"] = integration_std;
    node["init_bias_std"]   = init_bias_std;

    node["odom_rot_std"]   = odom_rot_std;
    node["odom_trans_std"] = odom_trans_std;

    node["ext_rot_std"]   = ext_rot_std;
    node["ext_trans_std"] = ext_trans_std;

    return node;
}

std::string NoiseParameters::print() const {
    return LVINS_FORMAT("Noise parameters:\n"
                        "  prior roll pitch std = {}\n"
                        "  prior yaw std = {}\n"
                        "  prior position std = {}\n"
                        "  prior velocity std = {}\n"
                        "  prior gyroscope bias std = {}\n"
                        "  prior accelerometer bias std = {}\n\n"
                        "  gyroscope std = {}\n"
                        "  accelerometer std = {}\n"
                        "  gyroscope bias std = {}\n"
                        "  accelerometer bias std = {}\n"
                        "  integration std = {}\n"
                        "  initial bias std = {}\n\n"
                        "  odometer rotation std = {}\n"
                        "  odometer translation std = {}\n\n"
                        "  extrinsic rotation std = {}\n"
                        "  extrinsic translation std = {}",
                        prior_roll_pitch_std, prior_yaw_std, prior_pos_std, prior_vel_std, prior_gyr_bias_std,
                        prior_acc_bias_std, gyr_std, acc_std, gyr_bias_std, acc_bias_std, integration_std,
                        init_bias_std, odom_rot_std, odom_trans_std, ext_rot_std, ext_trans_std);
}

} // namespace lvins
