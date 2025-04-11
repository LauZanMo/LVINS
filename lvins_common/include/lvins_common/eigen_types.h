#pragma once

#include "lvins_config/setup.h"

#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace lvins {

using Vec3f = Eigen::Matrix<Float, 3, 1>;
using Vec4f = Eigen::Matrix<Float, 4, 1>;
using Vec6f = Eigen::Matrix<Float, 6, 1>;
using VecXf = Eigen::Matrix<Float, Eigen::Dynamic, 1>;

using Mat33f = Eigen::Matrix<Float, 3, 3>;
using Mat44f = Eigen::Matrix<Float, 4, 4>;
using MatXf  = Eigen::Matrix<Float, Eigen::Dynamic, Eigen::Dynamic>;

using Quatf = Eigen::Quaternion<Float>;
using SO3f  = Sophus::SO3<Float>;
using SE3f  = Sophus::SE3<Float>;

} // namespace lvins
