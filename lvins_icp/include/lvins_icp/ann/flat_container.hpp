namespace lvins {

template<typename Result>
void FlatContainer::knnSearch(const Vec3f &point, Result &result) const {
    if (points_.empty()) {
        return;
    }

    for (size_t i = 0; i < points_.size(); ++i) {
        const auto sq_dist = (points_[i] - point).squaredNorm();
        result.push(i, sq_dist);
    }
}

} // namespace lvins
