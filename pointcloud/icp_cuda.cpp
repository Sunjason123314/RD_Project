#include "icp_cuda.h"
#include "icp_cuda_kernels.h"
#include "icp_svd.h"
#include <cuda_runtime.h>
#include <Eigen/Dense>
#include <vector>
#include <limits>
#include <cmath>

namespace ICP_CUDA {

ICPResult performICP(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    int max_iterations,
    float max_correspondence_distance,
    float transformation_epsilon,
    float fitness_epsilon)
{
    ICPResult result;
    result.transformation = Eigen::Matrix4f::Identity();
    result.fitness_score = std::numeric_limits<float>::max();
    result.converged = false;
    result.iterations = 0;

    if(!source || !target || source->empty() || target->empty())
        return result;

    int source_size = static_cast<int>(source->size());
    int target_size = static_cast<int>(target->size());

    // 分配 GPU 内存
    void *d_source, *d_target, *d_source_transformed;
    int *d_correspondences;
    float *d_distances, *d_transform;

    cudaMalloc(&d_source, source_size * 3 * sizeof(float));
    cudaMalloc(&d_target, target_size * 3 * sizeof(float));
    cudaMalloc(&d_source_transformed, source_size * 3 * sizeof(float));
    cudaMalloc(&d_correspondences, source_size * sizeof(int));
    cudaMalloc(&d_distances, source_size * sizeof(float));
    cudaMalloc(&d_transform, 16 * sizeof(float));

    // 拷贝源点云到 GPU
    std::vector<ICP_Point3D> h_source(source_size);
    std::vector<ICP_Point3D> h_target(target_size);

    for(int i = 0; i < source_size; ++i)
    {
        h_source[i].x = source->points[i].x;
        h_source[i].y = source->points[i].y;
        h_source[i].z = source->points[i].z;
    }
    for(int i = 0; i < target_size; ++i)
    {
        h_target[i].x = target->points[i].x;
        h_target[i].y = target->points[i].y;
        h_target[i].z = target->points[i].z;
    }

    cudaMemcpy(d_source, h_source.data(), source_size * 3 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_target, h_target.data(), target_size * 3 * sizeof(float), cudaMemcpyHostToDevice);

    Eigen::Matrix4f cumulative_transform = Eigen::Matrix4f::Identity();
    float previous_fitness = std::numeric_limits<float>::max();

    const int threads = 256;
    const int blocks = (source_size + threads - 1) / threads;
    float max_dist_sq = max_correspondence_distance * max_correspondence_distance;

    for(int iter = 0; iter < max_iterations; ++iter)
    {
        // 1. 找最近邻
        if(iter == 0)
        {
            launchFindNearestNeighbors(
                d_source, source_size,
                d_target, target_size,
                d_correspondences, d_distances,
                max_dist_sq, threads, blocks
            );
        }
        else
        {
            launchFindNearestNeighbors(
                d_source_transformed, source_size,
                d_target, target_size,
                d_correspondences, d_distances,
                max_dist_sq, threads, blocks
            );
        }
        cudaDeviceSynchronize();

        // 2. 计算适配度
        std::vector<float> h_distances(source_size);
        std::vector<int> h_correspondences(source_size);
        cudaMemcpy(h_distances.data(), d_distances, source_size * sizeof(float), cudaMemcpyDeviceToHost);
        cudaMemcpy(h_correspondences.data(), d_correspondences, source_size * sizeof(int), cudaMemcpyDeviceToHost);

        float fitness = 0.0f;
        int valid_pairs = 0;

        for(int i = 0; i < source_size; ++i)
        {
            if(h_distances[i] >= 0)
            {
                fitness += h_distances[i] * h_distances[i];
                valid_pairs++;
            }
        }

        if(valid_pairs == 0)
            break;

        fitness /= valid_pairs;

        // 3. 检查收敛
        if(iter > 0)
        {
            float fitness_change = std::abs(previous_fitness - fitness);
            if(fitness_change < fitness_epsilon)
            {
                result.converged = true;
                result.fitness_score = fitness;
                result.iterations = iter + 1;
                break;
            }
        }
        previous_fitness = fitness;

        // 4. 获取对应点对
        std::vector<ICP_Point3D> h_src_current(source_size);
        if(iter == 0)
        {
            h_src_current = h_source;
        }
        else
        {
            cudaMemcpy(h_src_current.data(), d_source_transformed, source_size * 3 * sizeof(float), cudaMemcpyDeviceToHost);
        }

        std::vector<ICP_Point3D> src_matched;
        std::vector<ICP_Point3D> tgt_matched;

        for(int i = 0; i < source_size; ++i)
        {
            if(h_correspondences[i] >= 0)
            {
                src_matched.push_back(h_src_current[i]);
                tgt_matched.push_back(h_target[h_correspondences[i]]);
            }
        }

        // 5. 计算变换
        float h_delta_transform[16];
        computeTransformationSVD(src_matched, tgt_matched, h_delta_transform);

        Eigen::Matrix4f delta_transform;
        for(int i = 0; i < 4; ++i)
            for(int j = 0; j < 4; ++j)
                delta_transform(i, j) = h_delta_transform[i * 4 + j];

        // 6. 更新累积变换
        cumulative_transform = delta_transform * cumulative_transform;

        // 7. 应用变换
        float h_transform[16];
        for(int i = 0; i < 4; ++i)
            for(int j = 0; j < 4; ++j)
                h_transform[i * 4 + j] = cumulative_transform(i, j);

        cudaMemcpy(d_transform, h_transform, 16 * sizeof(float), cudaMemcpyHostToDevice);

        launchApplyTransform(
            d_source, d_source_transformed, source_size,
            d_transform, threads, blocks
        );
        cudaDeviceSynchronize();

        // 8. 检查变换收敛
        Eigen::Matrix3f R_delta = delta_transform.block<3, 3>(0, 0);
        Eigen::Vector3f t_delta = delta_transform.block<3, 1>(0, 3);

        float rotation_change = Eigen::AngleAxisf(R_delta).angle();
        float translation_change = t_delta.norm();

        if(rotation_change < transformation_epsilon && translation_change < transformation_epsilon)
        {
            result.converged = true;
            result.fitness_score = fitness;
            result.iterations = iter + 1;
            break;
        }

        result.iterations = iter + 1;
        result.fitness_score = fitness;
    }

    result.transformation = cumulative_transform;

    // 清理 GPU 内存
    cudaFree(d_source);
    cudaFree(d_target);
    cudaFree(d_source_transformed);
    cudaFree(d_correspondences);
    cudaFree(d_distances);
    cudaFree(d_transform);

    return result;
}

} // namespace ICP_CUDA
