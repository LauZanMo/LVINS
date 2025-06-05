#pragma once

#include "lvins_common/eigen_types.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>

namespace lvins {

/**
 * @brief 将位姿从内部形式转换为gtsam形式
 * @param T 内部形式位姿
 * @return gtsam形式位姿
 */
gtsam::Pose3 toPose3(const SE3f &T);

/**
 * @brief 将IMU零偏从内部形式转换为gtsam形式
 * @param bg 陀螺仪零偏
 * @param ba 加速度计零偏
 * @return gtsam形式IMU零偏
 */
gtsam::imuBias::ConstantBias toImuBias(const Vec3f &bg, const Vec3f &ba);

/**
 * @brief 将位姿从gtsam形式位姿转换为内部形式
 * @param pose gtsam形式位姿
 * @return 内部形式位姿
 */
SE3f toSE3(const gtsam::Pose3 &pose);

} // namespace lvins
