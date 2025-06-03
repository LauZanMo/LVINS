#include "lvins_common/path_helper.h"
#include "lvins_common/time/time_wheel_scheduler.h"
#include "lvins_common/time/timer.h"
#include "lvins_icp/align/point_cloud_aligner.h"
#include "lvins_icp/ann/yaml/nn_searcher_yaml_serialization.h"
#include "lvins_icp/factor/gicp_factor.h"
#include "lvins_icp/preprocess/covariance_estimation.h"

#include <pcl/io/pcd_io.h>
#include <thread>

#include "bfnn.h"

using namespace lvins;

#include "pcl/filters/voxel_grid.h"
void VoxelGrid(PointCloud::Ptr cloud, float voxel_size) {
    pcl::VoxelGrid<Point> voxel;
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel.setInputCloud(cloud);

    PointCloud::Ptr output(new PointCloud);
    voxel.filter(*output);
    cloud->swap(*output);
}

void EvaluateMatches(const std::vector<std::pair<size_t, size_t>> &truth,
                     const std::vector<std::pair<size_t, size_t>> &esti) {
    int fp = 0; // false-positive，esti存在但truth中不存在
    int fn = 0; // false-negative, truth存在但esti不存在

    // LOG(INFO) << "truth: " << truth.size() << ", esti: " << esti.size();
    LVINS_INFO("truth: {}, esti: {}", truth.size(), esti.size());

    /// 检查某个匹配在另一个容器中存不存在
    auto exist = [](const std::pair<size_t, size_t> &data, const std::vector<std::pair<size_t, size_t>> &vec) -> bool {
        return std::find(vec.begin(), vec.end(), data) != vec.end();
    };

    int effective_esti           = 0;
    constexpr size_t kINVALID_ID = std::numeric_limits<size_t>::max();
    for (const auto &d: esti) {
        if (d.first != kINVALID_ID && d.second != kINVALID_ID) {
            effective_esti++;

            if (!exist(d, truth)) {
                fp++;
            }
        }
    }

    for (const auto &d: truth) {
        if (!exist(d, esti)) {
            fn++;
        }
    }

    float precision = 1.0 - float(fp) / effective_esti;
    float recall    = 1.0 - float(fn) / truth.size();
    // LOG(INFO) << "precision: " << precision << ", recall: " << recall << ", fp: " << fp << ", fn: " << fn;
    LVINS_INFO("precision: {}, recall: {}, fp: {}, fn: {}", precision, recall, fp, fn);
}

int main(int /*argc*/, char **argv) {
    const auto program_name = path_helper::getFileName(argv[0]);
    const auto log_path     = "/home/ubuntu/logs";

    // 初始化Logger
    Logger::initialize(true, log_path, program_name);

    const auto test_config  = YAML::load(path_helper::completePath("config/test/test_nn_searcher.yaml"));
    const auto nn_searcher0 = NearestNeighborSearcher::loadFromYaml(test_config["nn_searcher"]);
    // const auto nn_searcher1 = NearestNeighborSearcher::loadFromYaml(test_config["nn_searcher"]);
    LVINS_INFO("Nearest neighbor searcher:\n{}", *nn_searcher0);

    PointCloud::Ptr first(new PointCloud), second(new PointCloud);
    pcl::io::loadPCDFile(path_helper::completePath("config/test/first.pcd"), *first);
    pcl::io::loadPCDFile(path_helper::completePath("config/test/second.pcd"), *second);

    VoxelGrid(first, 0.05);
    VoxelGrid(second, 0.05);

    std::vector<std::pair<size_t, size_t>> matches;
    bfnn_cloud_mt(first, second, matches);
    LVINS_INFO("Total matches found: {}", matches.size());
    // for (size_t i = 0; i < 10; ++i) {
    //     LVINS_INFO("Match {}: first point index: {}, second point index: {}", i, matches[i].first, matches[i].second);
    // }

    std::vector<std::pair<size_t, size_t>> matches_ivox;
    nn_searcher0->insert(*first, SE3f(Mat44f::Identity()));
    for (size_t i = 0; i < second->size(); ++i) {
        auto pt = second->points[i].getVector3fMap();
        std::vector<size_t> index;
        std::vector<Float> sq_dist;
        if (nn_searcher0->knnSearch(pt, 1, index, sq_dist, 1.0f)) {
            auto search = nn_searcher0->point(index[0]);
            for (size_t j = 0; j < first->size(); ++j) {
                if (search == first->points[j].getVector3fMap()) {
                    LVINS_INFO("Search point: {}, first point: {}", pt.transpose(),
                               first->points[j].getVector3fMap().transpose());
                    matches_ivox.emplace_back(j, i);
                    break;
                }
            }
        }
    }

    for (size_t i = 0; i < 100; ++i) {
        LVINS_INFO("Match {}: first point index: {}, second point index: {}", i, matches_ivox[i].first,
                   matches_ivox[i].second);
    }

    EvaluateMatches(matches, matches_ivox);

    // LVINS_DECLARE_TIMER(a);
    // estimateCovariance(*first, *nn_searcher0, 5);
    // estimateCovariance(*second, *nn_searcher1, 5);
    // LVINS_PRINT_TIMER_MS("Estimate covariance", a);
    // LVINS_INFO("First point cloud size: {}, second point cloud size: {}", first->size(), second->size());
    //
    // const auto aligner = PointCloudAlignerBase::loadFromYaml(test_config["point_cloud_aligner"]);
    // LVINS_INFO("{}", *aligner);
    //
    // LVINS_DECLARE_TIMER(b);
    // const auto result =
    //         aligner->align(*nn_searcher0, {second.get()}, SE3f(Mat44f::Identity()), {SE3f(Mat44f::Identity())}, true);
    // LVINS_PRINT_TIMER_MS("Align result", b);
    // LVINS_INFO("Final result:\n"
    //            "   T_tb:\n{}\n"
    //            "   T_bs:\n{}\n"
    //            "   converged: {}\n"
    //            "   error: {}\n"
    //            "   iterations: {}\n"
    //            "   num_inliers: {}",
    //            result.T_tb.params(), result.T_bs[0].matrix(), result.converged, result.e, result.iterations,
    //            result.num_inliers);
    // LVINS_INFO("Alignment completed!");

    // std::vector<size_t> indices;
    // std::vector<Float> sq_dists;
    // const Vec3f point(0.0f, 0.0f, 0.0f);
    // auto size = nn_searcher->knnSearch(point, 5, indices, sq_dists, 100.0f);

    // const auto tws = std::make_shared<TimeWheelScheduler>();
    // tws->start();
    // LVINS_INFO("Current time: {}", Timer::currentTime());
    // tws->addOneShotTask(1500, [] {
    //     LVINS_INFO("Current time: {}", Timer::currentTime());
    //     LVINS_INFO("One shot task executed!");
    // });
    //
    // auto id = tws->addPeriodicTask(1000, [] {
    //     LVINS_INFO("Current time: {}", Timer::currentTime());
    //     LVINS_INFO("Periodic task executed!");
    // });
    //
    // std::this_thread::sleep_for(std::chrono::seconds(3));
    // tws->cancelTask(id);
    // LVINS_INFO("Task {} canceled!", id);
    //
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    // tws->stop();

    // 关闭Logger
    Logger::shutdown();
}
