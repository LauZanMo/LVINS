#pragma once

#include "lvins_common/async/async_priority_queue.h"
#include "lvins_common/nav_state.h"
#include "lvins_common/noise_parameters.h"
#include "lvins_common/sensor/imu.h"
#include "lvins_common/yaml/yaml_serialization.h"
#include "lvins_odometry/fusion/iterative_update_task.h"
#include "lvins_odometry/fusion/update_task.h"

namespace lvins {

/**
 * @brief 任务指针比较器
 * @tparam T 任务指针类型
 * @note 该比较器主要用于任务缓冲区的排序，以实现队首的任务时间戳最小
 */
template<typename T = void>
struct TaskPtrGreater {
    template<typename U = T, std::enable_if_t<!std::is_same_v<U, void>, int> = 0>
    bool operator()(const U &lhs, const U &rhs) const {
        return *lhs > *rhs;
    }
};

/**
 * @brief 特化void类型
 */
template<>
struct TaskPtrGreater<void> {
    template<typename U>
    bool operator()(const U &lhs, const U &rhs) const {
        return *lhs > *rhs;
    }
};

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
     * @param states 导航初始状态集合
     * @param imus IMU数据集合
     * @note IMU数据集合的时间戳必须与导航初始状态集合的时间戳一致
     */
    void initialize(const NavStates &states, const Imus &imus);

    /**
     * @brief 惯性递推
     * @param imu IMU数据
     * @return 此次递推是否有更新
     */
    [[nodiscard]] bool propagate(const Imu &imu);

    /**
     * @brief 添加单次更新任务
     * @param task 单次更新任务
     */
    void addUpdateTask(const UpdateTaskPtr &task);

    /**
     * @brief 添加迭代更新任务
     * @param task 迭代更新任务
     */
    void addUpdateTask(const iUpdateTaskPtr &task);

    /**
     * @brief 获取预测状态
     * @return 预测状态
     */
    [[nodiscard]] const NavState &predictState() const;

    /**
     * @brief 获取更新状态
     * @return 更新状态
     */
    [[nodiscard]] const NavState &updateState() const;

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
    static constexpr long O_V     = 3;
    static constexpr long O_Q     = 6;
    static constexpr long O_BG    = 9;
    static constexpr long O_BA    = 12;
    static constexpr long O_GW    = 15;
    static constexpr long O_EXT_P = 18;
    static constexpr long O_EXT_Q = O_EXT_P + 3;

private:
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
    [[nodiscard]] MatXf projectCovariance(const Eigen::Ref<const Vec3f> &q_wb_err,
                                          const std::vector<Vec3f> &q_bs_err) const;

    /**
     * @brief 重新积分
     * @note 该函数会在更新后调用，用于将更新后的状态重新积分到当前时刻
     */
    void repropagate();

    Vec3f g_w_, origin_g_w_;               ///< 世界坐标系下的重力向量
    std::vector<SE3f> T_bs_, origin_T_bs_; ///< 外参集合
    NavState update_state_;                ///< 当前更新状态

    MatXf P_; ///< 误差协方差矩阵
    MatXf Q_; ///< 传感器噪声协方差矩阵

    NavStateBuffer state_buffer_; ///< 状态数据缓冲区
    ImuBuffer imu_buffer_;        ///< IMU数据缓冲区

    UpdateTaskPtr update_task_;                                                         ///< 更新任务
    iUpdateTaskPtr iterative_update_task_;                                              ///< 迭代更新任务
    AsyncPriorityQueue<UpdateTaskPtr, TaskPtrGreater<>> update_task_buffer_;            ///< 更新任务缓冲区
    AsyncPriorityQueue<iUpdateTaskPtr, TaskPtrGreater<>> iterative_update_task_buffer_; ///< 迭代更新任务缓冲区
    std::vector<UpdateTaskPtr> exec_update_tasks_;                                      ///< 已执行的更新任务集合
    std::vector<iUpdateTaskPtr> exec_iterative_update_tasks_;                           ///< 已执行的更新任务集合

    const NoiseParameters &noise_params_; ///< 噪声参数
    int64_t buffer_len_;                  ///< 缓冲区长度（ns）
    size_t max_iterations_;               ///< 最大迭代次数
    Float iteration_quit_eps_;            ///< 迭代退出阈值
    long dim_;                            ///< 状态维度（p v q bg ba g_w ext_p ext_q）
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
