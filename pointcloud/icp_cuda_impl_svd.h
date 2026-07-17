#ifndef ICP_CUDA_IMPL_SVD_H
#define ICP_CUDA_IMPL_SVD_H

#include "icp_cuda_impl.h"
#include <vector>

namespace ICP_CUDA_Impl {

// SVD 求解变换矩阵（在 CPU 上使用 Eigen）
void computeTransformationSVD(
    const std::vector<Point3D>& src_points,
    const std::vector<Point3D>& tgt_points,
    float* transform_out); // 16 个元素的 4x4 矩阵

} // namespace ICP_CUDA_Impl

#endif // ICP_CUDA_IMPL_SVD_H
