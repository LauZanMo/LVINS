#include "lvins_common/logger.h"

namespace lvins {

template<typename DerivedKeypoint>
bool CameraGeometryBase::isKeypointVisible(const Eigen::MatrixBase<DerivedKeypoint> &keypoint) const {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedKeypoint, 2, 1);
    typedef typename DerivedKeypoint::Scalar Scalar;
    const auto ret = keypoint[0] >= static_cast<Scalar>(0.0) && keypoint[1] >= static_cast<Scalar>(0.0) &&
                     keypoint[0] < static_cast<Scalar>(width()) && keypoint[1] < static_cast<Scalar>(height());
    if (hasMask())
        return ret && !isMasked(keypoint);
    return ret;
}

template<typename DerivedKeypoint>
bool CameraGeometryBase::isKeypointVisible(const Eigen::MatrixBase<DerivedKeypoint> &keypoint,
                                           typename DerivedKeypoint::Scalar border) const {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedKeypoint, 2, 1);
    typedef typename DerivedKeypoint::Scalar Scalar;
    LVINS_CHECK(2 * border < static_cast<Scalar>(width()), "2 * margin should be less than camera width!");
    LVINS_CHECK(2 * border < static_cast<Scalar>(height()), "2 * margin should be less than camera height!");
    const auto ret = keypoint[0] >= border && keypoint[1] >= border &&
                     keypoint[0] < static_cast<Scalar>(width()) - border &&
                     keypoint[1] < static_cast<Scalar>(height()) - border;
    if (hasMask())
        return ret && !isMasked(keypoint);
    return ret;
}

} // namespace lvins
