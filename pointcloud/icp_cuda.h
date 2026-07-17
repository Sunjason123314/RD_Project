#ifndef ICP_CUDA_H
#define ICP_CUDA_H

#include <cuda_runtime.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>

namespace ICP_CUDA {

struct ICPResult {
    Eigen::Matrix4f transformation;
    float fitness_score;
    bool converged;
    int iterations;
};

// CUDA 加速的 ICP 配准
ICPResult performICP(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    int max_iterations = 50,
    float max_correspondence_distance = 1.0f,
    float transformation_epsilon = 1e-6f,
    float fitness_epsilon = 1e-6f
);

} // namespace ICP_CUDA

#endif // ICP_CUDA_H
