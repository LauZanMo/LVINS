#pragma once

#include "lvins_common/string_helper.h"
#include "lvins_common/yaml/yaml_serialization.h"

namespace lvins {

/**
 * @brief 噪声参数类
 */
struct NoiseParameters {
    using Ptr = std::shared_ptr<NoiseParameters>;

    /**
     * @brief 构造函数
     * @param config YAML配置节点
     */
    explicit NoiseParameters(const YAML::Node &config);

    /**
     * @brief 将噪声参数参数写入YAML节点
     * @return YAML节点
     */
    [[nodiscard]] YAML::Node writeToYaml() const;

    /**
     * @brief 打印噪声参数
     * @return 噪声参数
     */
    [[nodiscard]] std::string print() const;

    double prior_roll_pitch_std; ///< 先验俯仰角/横滚角标准差
    double prior_yaw_std;        ///< 先验航向角标准差
    double prior_pos_std;        ///< 先验位置标准差
    double prior_vel_std;        ///< 先验速度标准差
    double prior_gyr_bias_std;   ///< 先验陀螺仪零偏标准差
    double prior_acc_bias_std;   ///< 先验加速度计零偏标准差

    double gyr_std;         ///< 陀螺仪标准差
    double acc_std;         ///< 加速度计标准差
    double gyr_bias_std;    ///< 陀螺仪零偏标准差
    double acc_bias_std;    ///< 加速度计零偏标准差
    double integration_std; ///< 积分标准差
    double init_bias_std;   ///< 初始零偏标准差

    double odom_rot_std;   ///< 里程计姿态角标准差
    double odom_trans_std; ///< 里程计位移标准差

    double extrinsic_rot_std;   ///< 外参姿态角标准差
    double extrinsic_trans_std; ///< 外参位移标准差
};

} // namespace lvins

/**
 * @brief 噪声参数格式化器
 */
template<>
struct LVINS_FORMATTER<lvins::NoiseParameters> {
    /**
     * @brief 从文本中解析格式化字符
     * @param ctx 文本
     * @return 格式化字符尾部迭代器
     */
    static constexpr auto parse(const LVINS_FORMAT_PARSE_CONTEXT &ctx) {
        return ctx.begin();
    }

    /**
     * @brief 格式化
     * @param noise_params 噪声参数
     * @param ctx 输出的格式化文本
     * @return 输出格式化文本的尾部迭代器
     */
    static auto format(const lvins::NoiseParameters &noise_params, LVINS_FORMAT_CONTEXT &ctx) {
        return LVINS_FORMAT_TO(ctx.out(), "{}", noise_params.print());
    }
};
