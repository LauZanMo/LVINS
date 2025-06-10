#pragma once

#include "lvins_common/noise_parameters.h"
#include "lvins_icp/align/nearest_neighbor_search.h"
#include "lvins_icp/align/result.h"
#include "lvins_icp/point_cloud.h"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_points/optimizers/levenberg_marquardt_ext_params.hpp>

namespace lvins {

/**
 * @brief 点云配准器基类
 * @details 该类为点云配准器基类，所有点云配准器都通过YAML配置文件实现动态加载
 */
class PointCloudAligner {
public:
    using Ptr             = std::shared_ptr<PointCloudAligner>;
    using OptimizerParams = gtsam_points::LevenbergMarquardtExtParams;
    using Result          = point_cloud_align::Result;

    /**
     * @brief 构造函数
     */
    explicit PointCloudAligner(const YAML::Node &config);

    /**
     * @brief 默认析构函数
     */
    ~PointCloudAligner() = default;

    /**
     * @brief 将点云配准器参数写入YAML节点
     * @return YAML节点
     */
    [[nodiscard]] YAML::Node writeToYaml() const;

    /**
     * @brief 配准点云（不估计外参，用于动态初始化）
     * @param target_point_clouds 目标点云集合
     * @param source_point_clouds 源点云集合
     * @param init_T_tb 目标点云到载体的初始相对位姿
     * @param init_T_bs 初始雷达外参集合
     * @return 配准结果
     */
    [[nodiscard]] Result align(const std::vector<PointCloud::ConstPtr> &target_point_clouds,
                               const std::vector<PointCloud::ConstPtr> &source_point_clouds, const SE3f &init_T_tb,
                               const std::vector<SE3f> &init_T_bs) const;

    /**
     * @brief 配准点云
     * @param target_nn_search 目标点云的最近邻搜索器
     * @param source_point_clouds 源点云集合
     * @param noise_params 噪声参数
     * @param init_T_tb 目标点云到载体的初始相对位姿
     * @param init_T_bs 初始雷达外参集合
     * @param estimate_extrinsic 是否估计雷达外参
     * @return 配准结果
     */
    [[nodiscard]] Result align(const NearestNeighborSearch::Search::ConstPtr &target_nn_search,
                               const std::vector<PointCloud::ConstPtr> &source_point_clouds,
                               const NoiseParameters &noise_params, const SE3f &init_T_tb,
                               const std::vector<SE3f> &init_T_bs, bool estimate_extrinsic) const;

    /**
     * @brief 将点云配准因子加入因子图
     * @param target_nn_search 目标点云的最近邻搜索器
     * @param source_point_clouds 源点云集合
     * @param noise_params 噪声参数
     * @param init_T_tb 目标点云到载体的初始相对位姿
     * @param init_T_bs 初始雷达外参集合
     * @param estimate_extrinsic 是否估计雷达外参
     * @param graph 因子图
     * @param values 因子图变量值集合
     */
    void addFactor(const NearestNeighborSearch::Search::ConstPtr &target_nn_search,
                   const std::vector<PointCloud::ConstPtr> &source_point_clouds, const NoiseParameters &noise_params,
                   const SE3f &init_T_tb, const std::vector<SE3f> &init_T_bs, bool estimate_extrinsic,
                   gtsam::NonlinearFactorGraph &graph, gtsam::Values &values) const;

    /**
     * @brief 打印点云配准器参数
     * @return 点云配准器参数
     */
    [[nodiscard]] std::string print() const;

private:
    std::shared_ptr<OptimizerParams> params_; ///< 优化器参数
    double trans_eps_;                        ///< 平移收敛阈值
    double rot_eps_;                          ///< 旋转收敛阈值
    int num_threads_;                         ///< 求解线程数
};

} // namespace lvins

/**
 * @brief 点云配准器格式化器
 */
template<>
struct LVINS_FORMATTER<lvins::PointCloudAligner> {
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
     * @param aligner 点云配准器
     * @param ctx 输出的格式化文本
     * @return 输出格式化文本的尾部迭代器
     */
    static auto format(const lvins::PointCloudAligner &aligner, LVINS_FORMAT_CONTEXT &ctx) {
        return LVINS_FORMAT_TO(ctx.out(), "{}", aligner.print());
    }
};
