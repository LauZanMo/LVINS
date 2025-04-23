#include "lvins_common/eigen_helper.h"

template Eigen::Vector3i fastFloor(const Eigen::MatrixBase<Eigen::Vector3f> &matrix);
template Eigen::Vector3i fastFloor(const Eigen::MatrixBase<Eigen::Vector3d> &matrix);
