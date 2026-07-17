#include "icp_cuda_impl_kernels.h"
#include <device_launch_parameters.h>
#include <cuda_runtime.h>

namespace ICP_CUDA_Impl {

// CUDA kernel: 为每个源点找到最近的目标点
__global__ void findNearestNeighborsKernel(
    const Point3D* source, int source_size,
    const Point3D* target, int target_size,
    int* correspondences,
    float* distances,
    float max_distance_sq)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(idx >= source_size) return;

    Point3D src_pt = source[idx];
    float min_dist_sq = max_distance_sq;
    int best_match = -1;

    for(int i = 0; i < target_size; ++i)
    {
        Point3D tgt_pt = target[i];
        float dx = src_pt.x - tgt_pt.x;
        float dy = src_pt.y - tgt_pt.y;
        float dz = src_pt.z - tgt_pt.z;
        float dist_sq = dx*dx + dy*dy + dz*dz;

        if(dist_sq < min_dist_sq)
        {
            min_dist_sq = dist_sq;
            best_match = i;
        }
    }

    correspondences[idx] = best_match;
    distances[idx] = (best_match >= 0) ? sqrtf(min_dist_sq) : -1.0f;
}

// CUDA kernel: 应用变换到点云
__global__ void applyTransformKernel(
    const Point3D* input,
    Point3D* output,
    int size,
    const float* transform_matrix)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(idx >= size) return;

    Point3D pt = input[idx];

    output[idx].x = transform_matrix[0] * pt.x + transform_matrix[1] * pt.y + transform_matrix[2] * pt.z + transform_matrix[3];
    output[idx].y = transform_matrix[4] * pt.x + transform_matrix[5] * pt.y + transform_matrix[6] * pt.z + transform_matrix[7];
    output[idx].z = transform_matrix[8] * pt.x + transform_matrix[9] * pt.y + transform_matrix[10] * pt.z + transform_matrix[11];
}

void launchFindNearestNeighbors(
    const Point3D* d_source, int source_size,
    const Point3D* d_target, int target_size,
    int* d_correspondences,
    float* d_distances,
    float max_distance_sq,
    int threads, int blocks)
{
    findNearestNeighborsKernel<<<blocks, threads>>>(
        d_source, source_size,
        d_target, target_size,
        d_correspondences,
        d_distances,
        max_distance_sq
    );
}

void launchApplyTransform(
    const Point3D* d_input,
    Point3D* d_output,
    int size,
    const float* d_transform_matrix,
    int threads, int blocks)
{
    applyTransformKernel<<<blocks, threads>>>(
        d_input,
        d_output,
        size,
        d_transform_matrix
    );
}

} // namespace ICP_CUDA_Impl
