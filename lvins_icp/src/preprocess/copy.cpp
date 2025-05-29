#include "lvins_icp/preprocess/copy.h"

namespace lvins {

void copy(const RawPointCloud &src, PointCloud &output) {
    // 初始化点云
    output.header   = src.header;
    output.width    = src.width;
    output.height   = src.height;
    output.is_dense = src.is_dense;
    output.resize(src.size());

    // 丢弃时间信息，进行值拷贝
    for (size_t i = 0; i < src.size(); ++i) {
        output[i].getVector3fMap() = src[i].getVector3fMap();
        output[i].intensity        = src[i].intensity;
    }
}

} // namespace lvins
