#pragma once

#include "lvins_config/setup.h"

#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace lvins {

using Vec2f = Eigen::Matrix<Float, 2, 1>;
using Vec2d = Eigen::Matrix<double, 2, 1>;
using Vec3f = Eigen::Matrix<Float, 3, 1>;
using Vec3d = Eigen::Matrix<double, 3, 1>;
using Vec4f = Eigen::Matrix<Float, 4, 1>;
using Vec6f = Eigen::Matrix<Float, 6, 1>;
using VecXf = Eigen::Matrix<Float, Eigen::Dynamic, 1>;
using VecXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;

using Mat22f = Eigen::Matrix<Float, 2, 2>;
using Mat23f = Eigen::Matrix<Float, 2, 3>;
using Mat32f = Eigen::Matrix<Float, 3, 2>;
using Mat32d = Eigen::Matrix<double, 3, 2>;
using Mat33f = Eigen::Matrix<Float, 3, 3>;
using Mat33d = Eigen::Matrix<double, 3, 3>;
using Mat44f = Eigen::Matrix<Float, 4, 4>;
using Mat44d = Eigen::Matrix<double, 4, 4>;
using MatXf  = Eigen::Matrix<Float, Eigen::Dynamic, Eigen::Dynamic>;
using MatXd  = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

using Diag2f = Eigen::DiagonalMatrix<Float, 2>;
using Diag3d = Eigen::DiagonalMatrix<double, 3>;

using Quatf = Eigen::Quaternion<Float>;
using SO3f  = Sophus::SO3<Float>;
using SO3d  = Sophus::SO3<double>;
using SE3f  = Sophus::SE3<Float>;
using SE3d  = Sophus::SE3<double>;

} // namespace lvins
