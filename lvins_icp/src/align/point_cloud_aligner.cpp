#include "lvins_icp/align/point_cloud_aligner.h"
#include "lvins_common/gtsam_helper.h"

#include "gtsam/slam/BetweenFactor.h"
#include <gtsam/inference/Symbol.h>
#include <gtsam_points/factors/integrated_ct_gicp_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::W;

namespace lvins {

PointCloudAligner::PointCloudAligner(const YAML::Node &config) {
    const auto max_iterations     = YAML::get<int>(config, "max_iterations");
    const auto absolute_error_eps = YAML::get<double>(config, "absolute_error_eps");
    const auto relative_error_eps = YAML::get<double>(config, "relative_error_eps");
    const auto verbosity          = YAML::get<bool>(config, "verbosity");
    trans_eps_                    = YAML::get<double>(config, "trans_eps");
    rot_eps_                      = YAML::get<double>(config, "rot_eps");
    num_threads_                  = YAML::get<int>(config, "num_threads");

    params_ = std::make_shared<OptimizerParams>();
    params_->setMaxIterations(max_iterations);
    params_->setAbsoluteErrorTol(absolute_error_eps);
    params_->setRelativeErrorTol(relative_error_eps);
    if (verbosity) {
        params_->callback = [](auto &&status, auto && /*values*/) {
            LVINS_DEBUG("Printing optimization status:\n{}", status.to_string());
        };
    }
}

YAML::Node PointCloudAligner::writeToYaml() const {
    YAML::Node node;
    node["max_iterations"]     = params_->maxIterations;
    node["absolute_error_eps"] = params_->absoluteErrorTol;
    node["relative_error_eps"] = params_->relativeErrorTol;
    node["verbosity"]          = params_->callback ? true : false;
    node["trans_eps"]          = trans_eps_;
    node["rot_eps"]            = rot_eps_;
    node["num_threads"]        = num_threads_;
    return node;
}

PointCloudAligner::Result PointCloudAligner::align(const std::vector<PointCloud::ConstPtr> &target_point_clouds,
                                                   const std::vector<PointCloud::ConstPtr> &source_point_clouds,
                                                   const SE3f &init_T_tb, const std::vector<SE3f> &init_T_bs) const {
    // 初始化因子图
    gtsam::Values values;
    gtsam::NonlinearFactorGraph graph;

    // 先验因子
    values.insert(W(0), gtsam::Pose3());
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(W(0), gtsam::Pose3(),
                                                           gtsam::noiseModel::Isotropic::Precision(6, 1e6));

    // 点云配准因子
    gtsam::Pose3 last_T_tb = toPose3(init_T_tb);
    values.insert(B(0), toPose3(init_T_tb));
    for (size_t i = 0; i < source_point_clouds.size(); ++i) {
        const auto transform_target_point_cloud = transform(*target_point_clouds[i], init_T_bs[i]);
        const auto transform_source_point_cloud = transform(*source_point_clouds[i], init_T_bs[i]);

        const auto icp_factor = gtsam::make_shared<gtsam_points::IntegratedCT_GICPFactor>(
                W(0), B(0), transform_target_point_cloud, transform_source_point_cloud);
        icp_factor->set_num_threads(num_threads_);
        graph.add(icp_factor);
    }

    // 优化
    params_->termination_criteria = [&](const gtsam::Values &optimized_values) {
        bool converged = false;

        // 位姿收敛判断
        const auto current_T_tb       = optimized_values.at<gtsam::Pose3>(B(0));
        const gtsam::Pose3 delta_T_tb = last_T_tb.inverse() * current_T_tb;
        last_T_tb                     = current_T_tb;

        const double dt_T_tb = current_T_tb.translation().norm();
        const double dr_T_tb = gtsam::Rot3::Logmap(delta_T_tb.rotation()).norm();

        converged &= dt_T_tb < trans_eps_ && dr_T_tb < rot_eps_;

        // 求解失败的情况
        if (dt_T_tb < 1e-10 && dr_T_tb < 1e-10) {
            return false;
        }

        return converged;
    };
    gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, values, *params_);
    values = optimizer.optimize();

    // 获取优化结果
    Result result;
    result.T_tb       = toSE3(values.at<gtsam::Pose3>(B(0)));
    result.T_bs       = init_T_bs;
    result.iterations = optimizer.iterations();

    return result;
}

PointCloudAligner::Result PointCloudAligner::align(const NearestNeighborSearch::Search::ConstPtr &target_nn_search,
                                                   const std::vector<PointCloud::ConstPtr> &source_point_clouds,
                                                   const NoiseParameters &noise_params, const SE3f &init_T_tb,
                                                   const std::vector<SE3f> &init_T_bs, bool estimate_extrinsic) const {
    // 点云数量不足时外参估计欠定，矫正标志位
    estimate_extrinsic &= source_point_clouds.size() > 1;

    // 初始化因子图
    gtsam::Values values;
    gtsam::NonlinearFactorGraph graph;

    // 先验因子
    values.insert(W(0), gtsam::Pose3());
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(W(0), gtsam::Pose3(),
                                                           gtsam::noiseModel::Isotropic::Precision(6, 1e6));

    gtsam::Pose3 last_T_tb = toPose3(init_T_tb);
    std::vector<gtsam::Pose3> last_T_ts;
    values.insert(B(0), toPose3(init_T_tb));
    if (estimate_extrinsic) {
        // 点云配准因子
        for (size_t i = 0; i < source_point_clouds.size(); ++i) {
            SE3f init_T_ts = init_T_tb * init_T_bs[i];
            last_T_ts.push_back(toPose3(init_T_ts));
            values.insert(L(i), toPose3(init_T_ts));

            const auto icp_factor =
                    gtsam::make_shared<gtsam_points::IntegratedCT_GICPFactor_<NearestNeighborSearch::Search>>(
                            W(0), L(i), target_nn_search, source_point_clouds[i], target_nn_search);
            icp_factor->set_num_threads(num_threads_);
            graph.add(icp_factor);
        }

        // 外参因子
        const auto &ext_rot_std   = noise_params.ext_rot_std;
        const auto &ext_trans_std = noise_params.ext_trans_std;
        // clang-format off
        const auto ext_noise_model = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << ext_rot_std, ext_rot_std, ext_rot_std,
                                        ext_trans_std, ext_trans_std, ext_trans_std).finished());
        // clang-format on
        for (size_t i = 0; i < init_T_bs.size(); ++i) {
            const auto ext_factor = gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                    B(0), L(i), toPose3(init_T_bs[i]), ext_noise_model);
            graph.add(ext_factor);
        }
    } else {
        // 点云配准因子
        for (size_t i = 0; i < source_point_clouds.size(); ++i) {
            const auto transform_point_cloud = transform(*source_point_clouds[i], init_T_bs[i]);

            const auto icp_factor =
                    gtsam::make_shared<gtsam_points::IntegratedCT_GICPFactor_<NearestNeighborSearch::Search>>(
                            W(0), B(0), target_nn_search, transform_point_cloud, target_nn_search);
            icp_factor->set_num_threads(num_threads_);
            graph.add(icp_factor);
        }
    }

    // 优化
    params_->termination_criteria = [&](const gtsam::Values &optimized_values) {
        bool converged = false;

        // 位姿收敛判断
        const auto current_T_tb       = optimized_values.at<gtsam::Pose3>(B(0));
        const gtsam::Pose3 delta_T_tb = last_T_tb.inverse() * current_T_tb;
        last_T_tb                     = current_T_tb;

        const double dt_T_tb = current_T_tb.translation().norm();
        const double dr_T_tb = gtsam::Rot3::Logmap(delta_T_tb.rotation()).norm();

        converged &= dt_T_tb < trans_eps_ && dr_T_tb < rot_eps_;

        // 外参收敛判断
        if (estimate_extrinsic) {
            for (size_t i = 0; i < last_T_ts.size(); ++i) {
                const auto current_T_ts       = optimized_values.at<gtsam::Pose3>(L(i));
                const gtsam::Pose3 delta_T_ts = last_T_ts[i].inverse() * current_T_ts;
                last_T_ts[i]                  = current_T_ts;

                const double dt_T_ts = current_T_ts.translation().norm();
                const double dr_T_ts = gtsam::Rot3::Logmap(delta_T_ts.rotation()).norm();

                converged &= dt_T_ts < trans_eps_ && dr_T_ts < rot_eps_;
            }
        }

        // 求解失败的情况
        if (dt_T_tb < 1e-10 && dr_T_tb < 1e-10) {
            return false;
        }

        return converged;
    };
    gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, values, *params_);
    values = optimizer.optimize();

    // 获取优化结果
    Result result;
    result.T_tb = toSE3(values.at<gtsam::Pose3>(B(0)));
    if (estimate_extrinsic) {
        for (size_t i = 0; i < init_T_bs.size(); ++i) {
            SE3f T_bs = result.T_tb.inverse() * toSE3(values.at<gtsam::Pose3>(L(i)));
            result.T_bs.push_back(T_bs);
        }
    } else {
        result.T_bs = init_T_bs;
    }
    result.iterations = optimizer.iterations();

    return result;
}

void PointCloudAligner::addFactor(const NearestNeighborSearch::Search::ConstPtr &target_nn_search,
                                  const std::vector<PointCloud::ConstPtr> &source_point_clouds,
                                  const NoiseParameters &noise_params, const SE3f &init_T_tb,
                                  const std::vector<SE3f> &init_T_bs, bool estimate_extrinsic,
                                  gtsam::NonlinearFactorGraph &graph, gtsam::Values &values) const {
    if (estimate_extrinsic) {
        // 点云配准因子
        for (size_t i = 0; i < source_point_clouds.size(); ++i) {
            SE3f init_T_ts = init_T_tb * init_T_bs[i];
            values.insert(L(i), toPose3(init_T_ts));

            const auto icp_factor =
                    gtsam::make_shared<gtsam_points::IntegratedGICPFactor_<NearestNeighborSearch::Search>>(
                            W(0), L(i), target_nn_search, source_point_clouds[i], target_nn_search);
            icp_factor->set_num_threads(num_threads_);
            graph.add(icp_factor);
        }

        // 外参因子
        const auto &ext_rot_std   = noise_params.ext_rot_std;
        const auto &ext_trans_std = noise_params.ext_trans_std;
        // clang-format off
        const auto ext_noise_model = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << ext_rot_std, ext_rot_std, ext_rot_std,
                                        ext_trans_std, ext_trans_std, ext_trans_std).finished());
        // clang-format on
        for (size_t i = 0; i < init_T_bs.size(); ++i) {
            const auto ext_factor = gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                    B(0), L(i), toPose3(init_T_bs[i]), ext_noise_model);
            graph.add(ext_factor);
        }
    } else {
        // 点云配准因子
        for (size_t i = 0; i < source_point_clouds.size(); ++i) {
            const auto transform_point_cloud = transform(*source_point_clouds[i], init_T_bs[i]);
            const auto icp_factor =
                    gtsam::make_shared<gtsam_points::IntegratedGICPFactor_<NearestNeighborSearch::Search>>(
                            W(0), B(0), target_nn_search, transform_point_cloud, target_nn_search);
            icp_factor->set_num_threads(num_threads_);
            graph.add(icp_factor);
        }
    }
}

std::string PointCloudAligner::print() const {
    return LVINS_FORMAT("Point cloud aligner:\n"
                        "  max iterations = {}\n"
                        "  absolute error epsilon = {}\n"
                        "  relative error epsilon = {}\n"
                        "  verbosity = {}\n"
                        "  translation epsilon = {}\n"
                        "  rotation epsilon = {}\n"
                        "  num threads = {}",
                        params_->maxIterations, params_->absoluteErrorTol, params_->relativeErrorTol,
                        params_->callback ? "true" : "false", trans_eps_, rot_eps_, num_threads_);
}

} // namespace lvins
