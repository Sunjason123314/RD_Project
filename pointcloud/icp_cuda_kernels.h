#ifndef ICP_CUDA_KERNELS_H
#define ICP_CUDA_KERNELS_H

#ifdef __cplusplus
extern "C" {
#endif

void launchFindNearestNeighbors(
    const void* d_source, int source_size,
    const void* d_target, int target_size,
    int* d_correspondences,
    float* d_distances,
    float max_distance_sq,
    int threads, int blocks);

void launchApplyTransform(
    const void* d_input,
    void* d_output,
    int size,
    const float* d_transform_matrix,
    int threads, int blocks);

#ifdef __cplusplus
}
#endif

#endif // ICP_CUDA_KERNELS_H
