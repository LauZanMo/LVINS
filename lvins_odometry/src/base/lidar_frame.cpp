#include "lvins_odometry/base/lidar_frame.h"
#include "lvins_common/logger.h"

namespace lvins {

static std::atomic<long> frame_counter{0};

LidarFrame::LidarFrame(int64_t timestamp, const LidarGeometryBase &lidar, RawPointCloud::Ptr raw_point_cloud)
    : timestamp_(timestamp),
      id_(frame_counter++),
      lidar_(lidar),
      raw_point_cloud_(std::move(raw_point_cloud)),
      point_cloud_(std::make_shared<PointCloud>()) {}

int64_t LidarFrame::timestamp() const {
    return timestamp_;
}

long LidarFrame::id() const {
    return id_;
}

long LidarFrame::bundleId() const {
    return bundle_id_;
}

void LidarFrame::setBundleId(long bundle_id) {
    bundle_id_ = bundle_id;
}

const LidarGeometryBase &LidarFrame::lidar() const {
    return lidar_;
}

const SE3f &LidarFrame::Twf() const {
    return T_wf_;
}

void LidarFrame::setTwf(const SE3f &T_wf) {
    T_wf_ = T_wf;
}

const SE3f &LidarFrame::Tbs() const {
    return T_bs_;
}

void LidarFrame::setTbs(const SE3f &T_bs) {
    T_bs_ = T_bs;
}

RawPointCloud &LidarFrame::rawPointCloud() {
    return *raw_point_cloud_;
}

const RawPointCloud &LidarFrame::rawPointCloud() const {
    return *raw_point_cloud_;
}

PointCloud &LidarFrame::pointCloud() {
    return *point_cloud_;
}

const PointCloud &LidarFrame::pointCloud() const {
    return *point_cloud_;
}

std::string LidarFrame::print() const {
    return LVINS_FORMAT("Lidar frame #{}:\n"
                        "  timestamp: {}\n"
                        "  bundle id: {}\n"
                        "  lidar: {}\n"
                        "  T_wf: {}\n"
                        "  T_bs: {}\n"
                        "  raw point cloud: {}\n"
                        "  point cloud: {}",
                        LVINS_GROUP_DIGITS(id_), LVINS_GROUP_DIGITS(timestamp_), LVINS_GROUP_DIGITS(bundle_id_),
                        lidar_.label(), LVINS_VECTOR_FMT(T_wf_.params()), LVINS_VECTOR_FMT(T_bs_.params()),
                        *raw_point_cloud_, *point_cloud_);
}

} // namespace lvins
