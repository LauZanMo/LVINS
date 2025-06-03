namespace lvins {

template<typename DerivedPoint>
bool LidarGeometryBase::isPointValid(const Eigen::MatrixBase<DerivedPoint> &point) const {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedPoint, 3, 1)
    typedef typename DerivedPoint::Scalar Scalar;
    const auto dist2 = point.squaredNorm();
    return dist2 > static_cast<Scalar>(nearest_dist2_) && dist2 < static_cast<Scalar>(farthest_dist2_);
}

} // namespace lvins