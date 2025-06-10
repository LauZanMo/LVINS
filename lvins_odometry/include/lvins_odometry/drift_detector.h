#pragma once

#include "lvins_common/nav_state.h"
#include "lvins_common/non_copyable.h"
#include "lvins_common/yaml/yaml_serialization.h"

namespace lvins {

/**
 * @brief 漂移检测器类
 * @details 该类通过计算预测状态（IMU积分状态）和测量状态的位姿和速度偏差来检测系统是否出现漂移
 */
class DriftDetector : public NonCopyable {
public:
    using Ptr = std::unique_ptr<DriftDetector>;

    /**
     * @brief 构造函数
     * @param config YAML配置节点
     */
    explicit DriftDetector(const YAML::Node &config);

    /**
     * @brief 将漂移检测器参数写入YAML节点
     * @return YAML节点
     */
    [[nodiscard]] YAML::Node writeToYaml() const;

    /**
     * @brief 检测系统漂移
     * @details 该函数计算预测状态（IMU积分状态）和测量状态的位姿和速度偏差，若超出给定阈值，则认为系统出现漂移
     * @param predict 预测状态
     * @param measure 测量状态
     * @return 是否检测到系统漂移
     */
    [[nodiscard]] bool detect(const NavState &predict, const NavState &measure) const;

    /**
     * @brief 打印漂移检测器参数
     * @return 漂移检测器参数
     */
    [[nodiscard]] std::string print() const;

private:
    Float max_rot_err_;   ///< 最大旋转偏差
    Float max_trans_err_; ///< 最大位移偏差
    Float max_vel_err_;   ///< 最大速度偏差
    bool verbosity_;      ///< 是否打印调试信息
};

} // namespace lvins

/**
 * @brief 漂移检测器格式化器
 */
template<>
struct LVINS_FORMATTER<lvins::DriftDetector> {
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
     * @param drift_detector 漂移检测器
     * @param ctx 输出的格式化文本
     * @return 输出格式化文本的尾部迭代器
     */
    static auto format(const lvins::DriftDetector &drift_detector, LVINS_FORMAT_CONTEXT &ctx) {
        return LVINS_FORMAT_TO(ctx.out(), "{}", drift_detector.print());
    }
};
