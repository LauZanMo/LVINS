#include "lvins_odometry/base/lidar_frame_bundle.h"
#include "lvins_common/logger.h"

namespace lvins {

static std::atomic<long> bundle_counter{0};

LidarFrameBundle::LidarFrameBundle(const std::vector<LidarFrame::Ptr> &frames)
    : id_(bundle_counter++), frames_(frames) {
    LVINS_CHECK(!frames.empty(), "Frame bundle should contain at least one frame!");
    for (const auto &frame: frames_) {
        LVINS_CHECK(timestamp() == frame->timestamp(), "All frames in frame bundle should have the same timestamp!");
        frame->setBundleId(id_);
    }
}

int64_t LidarFrameBundle::timestamp() const {
    return frames_.front()->timestamp();
}

long LidarFrameBundle::id() const {
    return id_;
}

const NavState &LidarFrameBundle::state() const {
    LVINS_CHECK(state_, "State should be initialized before access!");
    return state_;
}

void LidarFrameBundle::setState(const NavState &state) {
    LVINS_CHECK(!state_ || timestamp() == state.timestamp,
                "State timestamp should be equal to the frame bundle timestamp!");
    state_ = state;
    for (const auto &frame: frames_) {
        frame->setTwf(state_.T * frame->Tbs());
    }
}

const SE3f &LidarFrameBundle::Twb() const {
    LVINS_CHECK(state_, "State should be initialized before access!");
    return state_.T;
}

void LidarFrameBundle::setTwb(const SE3f &T_wb) {
    state_.T = T_wb;
    for (const auto &frame: frames_) {
        frame->setTwf(T_wb * frame->Tbs());
    }
}

std::vector<SE3f> LidarFrameBundle::Tbs() const {
    std::vector<SE3f> T_bs;
    for (const auto &frame: frames_) {
        T_bs.push_back(frame->Tbs());
    }
    return T_bs;
}

void LidarFrameBundle::setTbs(const std::vector<SE3f> &T_bs_vec) {
    LVINS_CHECK(T_bs_vec.size() == frames_.size(), "T_bs_vec size should be equal to the number of frames!");
    for (size_t i = 0; i < T_bs_vec.size(); ++i) {
        frames_[i]->setTbs(T_bs_vec[i]);
    }
}

size_t LidarFrameBundle::size() const {
    return frames_.size();
}

LidarFrame::Ptr &LidarFrameBundle::frame(size_t idx) {
    LVINS_CHECK(idx < frames_.size(), "Index should be less than the number of frames!");
    return frames_[idx];
}

LidarFrame::ConstPtr LidarFrameBundle::frame(size_t idx) const {
    LVINS_CHECK(idx < frames_.size(), "Index should be less than the number of frames!");
    return frames_[idx];
}

std::vector<PointCloud::ConstPtr> LidarFrameBundle::pointClouds() const {
    std::vector<PointCloud::ConstPtr> point_clouds;
    for (const auto &frame: frames_) {
        point_clouds.push_back(frame->pointCloud());
    }
    return point_clouds;
}

std::string LidarFrameBundle::print() const {
    std::string ret;
    auto end = LVINS_FORMAT_TO(std::back_inserter(ret), "Lidar frame bundle #{}:", LVINS_GROUP_DIGITS(id_));
    for (const auto &frame: frames_) {
        end = LVINS_FORMAT_TO(end, "\n{}", frame->print());
    }
    return ret;
}

} // namespace lvins
