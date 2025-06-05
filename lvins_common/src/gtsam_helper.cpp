#include "lvins_common/gtsam_helper.h"

namespace lvins {

gtsam::Pose3 toPose3(const SE3f &T) {
    return {T.matrix().cast<double>()};
}

gtsam::imuBias::ConstantBias toImuBias(const Vec3f &bg, const Vec3f &ba) {
    return {ba.cast<double>(), bg.cast<double>()};
}

SE3f toSE3(const gtsam::Pose3 &pose) {
    return SE3f(pose.matrix().cast<Float>());
}

} // namespace lvins
