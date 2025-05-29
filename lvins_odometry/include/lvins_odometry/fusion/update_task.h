#pragma once

#include "lvins_common/nav_state.h"
#include "lvins_common/noise_parameters.h"

namespace lvins::eskf {

/**
 * @brief 单次更新任务类
 * @details 该类用于ESKF单次更新，包含时间戳、观测函数和结尾处理函数
 */
class UpdateTask {
public:
    using Ptr = std::shared_ptr<UpdateTask>;

    /**
     * @brief 构造函数
     * @param timestamp 时间戳（ns）
     */
    explicit UpdateTask(int64_t timestamp);

    /**
     * @brief 默认析构函数
     */
    virtual ~UpdateTask() = default;

    /**
     * @brief 获取时间戳
     * @return 时间戳（ns）
     */
    [[nodiscard]] int64_t timestamp() const;

    /**
     * @brief 观测函数
     * @param noise_params 噪声参数
     * @param dim ESKF状态维度
     * @param state 预测当前状态
     * @param T_bs 预测外参集合
     * @param H 观测矩阵
     * @param V 观测噪声协方差矩阵
     * @param r 观测残差向量
     */
    virtual void observe(const NoiseParameters &noise_params, long dim, const NavState &state,
                         const std::vector<SE3f> &T_bs, MatXd &H, MatXd &V, VecXd &r) const = 0;

    /**
     * @brief 结尾处理函数
     * @param state 融合状态
     * @param T_bs 融合外参集合
     */
    virtual void finalize(const NavState &state, const std::vector<SE3f> &T_bs) = 0;

    /**
     * @brief 比较运算符
     * @param other 其他单次更新任务
     * @return 是否小于
     */
    bool operator<(const UpdateTask &other) const;

    /**
     * @brief 比较运算符
     * @param other 其他单次更新任务
     * @return 是否大于
     */
    bool operator>(const UpdateTask &other) const;

protected:
    int64_t timestamp_; ///< 时间戳（ns）
};

} // namespace lvins::eskf
