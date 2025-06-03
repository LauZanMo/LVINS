#include "lvins_common/nav_state.h"

namespace lvins {

NavState::NavState() : timestamp(-1), T(Mat44f::Identity()), vel(Vec3f::Zero()), bg(Vec3f::Zero()), ba(Vec3f::Zero()) {}

NavState::NavState(int64_t timestamp, const SE3f &T, Vec3f vel, Vec3f bg, Vec3f ba)
    : timestamp(timestamp), T(T), vel(std::move(vel)), bg(std::move(bg)), ba(std::move(ba)) {}

NavState::operator bool() const {
    return timestamp >= 0;
}

} // namespace lvins
