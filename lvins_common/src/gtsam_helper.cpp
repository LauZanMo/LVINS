#include "lvins_common/gtsam_helper.h"

namespace lvins {

gtsam::Pose3 toPose3(const SE3f &T) {
    return {T.matrix().cast<double>()};
}

gtsam::imuBias::ConstantBias toImuBias(const Vec3f &bg, const Vec3f &ba) {
    return {ba.cast<double>(), bg.cast<double>()};
}

SE3f toSE3(const gtsam::Pose3 &pose) {
    // 此操作是为了防止输入旋转矩阵非正交
    Mat44f T = pose.matrix().cast<Float>();
    Quatf q(T.block<3, 3>(0, 0));
    q.normalize();
    T.block<3, 3>(0, 0) = q.toRotationMatrix();

    return SE3f(T);
}

} // namespace lvins
