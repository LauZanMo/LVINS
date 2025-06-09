#pragma once

#include "lvins_common/nav_state.h"
#include "lvins_common/noise_parameters.h"
#include "lvins_common/sensor/imu.h"
#include "lvins_common/yaml/yaml_serialization.h"
#include "lvins_odometry/fusion/iterative_update_task.h"
#include "lvins_odometry/fusion/update_task.h"

namespace lvins {

/**
 * @brief 扩展卡尔曼滤波器（ESKF）类
 * @details 该类实现了扩展卡尔曼滤波器（ESKF）算法，用于惯性导航系统的状态估计，该类有以下特性<br/>
 *          1. 支持多传感器融合<br/>
 *          2. 支持单次更新和迭代更新<br/>
 *          3. 延迟更新，以解决实时处理时观测数据延迟到达的问题
 */
class ESKF {
public:
    using Ptr            = std::unique_ptr<ESKF>;
    using UpdateTaskPtr  = eskf::UpdateTask::Ptr;
    using iUpdateTaskPtr = eskf::IterativeUpdateTask::Ptr;

    /**
     * @brief 构造函数
     * @param config YAML配置节点
     * @param noise_params 噪声参数
     * @param g_w 世界坐标系下的重力向量
     * @param T_bs 外参集合
     */
    ESKF(const YAML::Node &config, const NoiseParameters &noise_params, Vec3f g_w, std::vector<SE3f> T_bs);

    /**
     * @brief 默认析构函数
     */
    ~ESKF() = default;

    /**
     * @brief 将ESKF参数写入YAML节点
     * @return YAML节点
     */
    [[nodiscard]] YAML::Node writeToYaml() const;

    /**
     * @brief 重置ESKF
     */
    void reset();

    /**
     * @brief 初始化
     * @param state 导航初始状态
     */
    void initialize(const NavState &state);

    /**
     * @brief 惯性递推
     * @param imus IMU数据容器
     * @return 每一次积分的导航状态集合（含起点）
     */
    NavStates propagate(const Imus &imus);

    /**
     * @brief 单次更新
     * @param task 单次更新任务
     */
    void update(const UpdateTaskPtr &task);

    /**
     * @brief 迭代更新
     * @param task 迭代更新任务
     */
    void update(const iUpdateTaskPtr &task);

    /**
     * @brief 获取当前时刻状态
     * @return 当前时刻状态
     */
    [[nodiscard]] const NavState &state() const;

    /**
     * @brief 获取外参集合
     * @return 外参集合
     */
    [[nodiscard]] const std::vector<SE3f> &Tbs() const;

    /**
     * @brief 打印ESKF参数
     * @return ESKF参数
     */
    [[nodiscard]] std::string print() const;

    // 各状态量在状态向量中的起点索引
    static constexpr long O_P     = 0;
    static constexpr long O_Q     = O_P + 3;
    static constexpr long O_V     = 6;
    static constexpr long O_BG    = 9;
    static constexpr long O_BA    = 12;
    static constexpr long O_GW    = 15;
    static constexpr long O_EXT_P = 18;
    static constexpr long O_EXT_Q = O_EXT_P + 3;

private:
    /**
     * @brief 更新名义状态
     * @param delta_state 状态修正量
     */
    void correctNominal(const VecXf &delta_state);

    /**
     * @brief 投影误差协方差矩阵
     * @param q_wb_err 世界坐标系下的位姿残差
     * @param q_bs_err 外参位姿残差集合
     * @return 投影后的误差协方差矩阵
     */
    [[nodiscard]] MatXd projectCovariance(const Eigen::Ref<const Vec3d> &q_wb_err,
                                          const std::vector<Vec3d> &q_bs_err) const;

    Vec3f g_w_, origin_g_w_;               ///< 世界坐标系下的重力向量
    std::vector<SE3f> T_bs_, origin_T_bs_; ///< 外参集合
    NavState state_;                       ///< 当前时刻状态
    MatXd P_;                              ///< 误差协方差矩阵
    MatXd Q_;                              ///< 传感器噪声协方差矩阵

    const NoiseParameters &noise_params_; ///< 噪声参数
    size_t max_iterations_;               ///< 最大迭代次数
    double iteration_quit_eps_;           ///< 迭代退出阈值
    bool verbosity_;                      ///< 是否打印调试信息
    long dim_;                            ///< 状态维度 [p q v bg ba g_w ext_p ext_q]
};

} // namespace lvins

/**
 * @brief ESKF格式化器
 */
template<>
struct LVINS_FORMATTER<lvins::ESKF> {
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
     * @param eskf ESKF
     * @param ctx 输出的格式化文本
     * @return 输出格式化文本的尾部迭代器
     */
    static auto format(const lvins::ESKF &eskf, LVINS_FORMAT_CONTEXT &ctx) {
        return LVINS_FORMAT_TO(ctx.out(), "{}", eskf.print());
    }
};
