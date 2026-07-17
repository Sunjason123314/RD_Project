#ifndef ICP_CUDA_IMPL_KERNELS_H
#define ICP_CUDA_IMPL_KERNELS_H

#include "icp_cuda_impl.h"

namespace ICP_CUDA_Impl {

// CUDA kernel 接口
void launchFindNearestNeighbors(
    const Point3D* d_source, int source_size,
    const Point3D* d_target, int target_size,
    int* d_correspondences,
    float* d_distances,
    float max_distance_sq,
    int threads, int blocks);

void launchApplyTransform(
    const Point3D* d_input,
    Point3D* d_output,
    int size,
    const float* d_transform_matrix,
    int threads, int blocks);

} // namespace ICP_CUDA_Impl

#endif // ICP_CUDA_IMPL_KERNELS_H
