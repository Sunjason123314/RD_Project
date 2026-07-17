#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include "draw.h"
#include <cuda_runtime.h>
#include <QGLViewer/camera.h>
#include <QGLViewer/vec.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <qglviewer/camera.h>
#include <qglviewer/qglviewer.h>
#include <cstdio>
#include <vector>
#include <GL/glu.h>
#include <thrust/copy.h>
#include <vector>
#include <iostream>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/copy.h>
#include <vector>
#include <iostream>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/copy.h>
#include <vector>
#include <iostream>

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/copy.h>
#include <vector>
#include <iostream>

struct PointCloud {
    float* x;
    float* y;
    float* z;
};

__device__ void transformVertex(float x, float y, float z, const float* model_view_matrix, const float* projection_matrix, const GLint* viewport, float* vs) {
    float mv[4];
    mv[0] = model_view_matrix[0] * x + model_view_matrix[4] * y + model_view_matrix[8] * z + model_view_matrix[12];
    mv[1] = model_view_matrix[1] * x + model_view_matrix[5] * y + model_view_matrix[9] * z + model_view_matrix[13];
    mv[2] = model_view_matrix[2] * x + model_view_matrix[6] * y + model_view_matrix[10] * z + model_view_matrix[14];
    mv[3] = model_view_matrix[3] * x + model_view_matrix[7] * y + model_view_matrix[11] * z + model_view_matrix[15];

    float pv[4];
    pv[0] = projection_matrix[0] * mv[0] + projection_matrix[4] * mv[1] + projection_matrix[8] * mv[2] + projection_matrix[12] * mv[3];
    pv[1] = projection_matrix[1] * mv[0] + projection_matrix[5] * mv[1] + projection_matrix[9] * mv[2] + projection_matrix[13] * mv[3];
    pv[2] = projection_matrix[2] * mv[0] + projection_matrix[6] * mv[1] + projection_matrix[10] * mv[2] + projection_matrix[14] * mv[3];
    pv[3] = projection_matrix[3] * mv[0] + projection_matrix[7] * mv[1] + projection_matrix[11] * mv[2] + projection_matrix[15] * mv[3];

    vs[0] = (pv[0] / pv[3] * 0.5f + 0.5f) * viewport[2] + viewport[0];
    vs[1] = (pv[1] / pv[3] * 0.5f + 0.5f) * viewport[3] + viewport[1];
    vs[2] = pv[2] / pv[3];
}


__global__ void cudaKernel(const float* points, const float* model_view_matrix, const float* projection_matrix, const GLint* viewport, int min_x, int max_x, int min_y, int max_y, int* selected_indices, int num_points) {
    int idx = threadIdx.x + blockIdx.x * blockDim.x;

    if (idx >= num_points) return;

    // 提取点的坐标
    float x = points[idx * 3];
    float y = points[idx * 3 + 1];
    float z = points[idx * 3 + 2];

    // 使用模型视图和投影矩阵进行变换
    float vs[4];
    transformVertex(x, y, z, model_view_matrix, projection_matrix, viewport, vs);

    // 计算窗口坐标
    float winX = vs[0];
    float winY = viewport[3] - vs[1];
    float winZ = vs[2];

    // 打印窗口坐标
    //printf("winX: %f, winY: %f, winZ: %f\n", winX, winY, winZ);

    // 判断是否在选择区域内
    if (winX >= min_x && winX <= max_x && winY >= min_y && winY <= max_y) {
        // 将索引添加到选中点集
        int selected_idx = atomicAdd(&selected_indices[0], 1);
        selected_indices[selected_idx + 1] = idx;
    }
}

void performAreaSelectionCuda(const std::vector<float>& points, const float* model_view_matrix, const float* projection_matrix, const GLint* viewport, int min_x, int max_x, int min_y, int max_y, std::vector<int>& selected_indices) {
    // 分配和复制数据到设备内存
    float* d_points;
    float* d_model_view_matrix;
    float* d_projection_matrix;
    GLint* d_viewport;
    int* d_selected_indices;
    int num_points = points.size() / 3;

    cudaMalloc(&d_points, points.size() * sizeof(float));
    cudaMalloc(&d_model_view_matrix, 16 * sizeof(float));
    cudaMalloc(&d_projection_matrix, 16 * sizeof(float));
    cudaMalloc(&d_viewport, 4 * sizeof(GLint));
    cudaMalloc(&d_selected_indices, (num_points + 1) * sizeof(int));

    cudaMemcpy(d_points, points.data(), points.size() * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_model_view_matrix, model_view_matrix, 16 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_projection_matrix, projection_matrix, 16 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_viewport, viewport, 4 * sizeof(GLint), cudaMemcpyHostToDevice);
    cudaMemset(d_selected_indices, 0, (num_points + 1) * sizeof(int));

    // 调用CUDA内核
    int threadsPerBlock = 256;
    int blocksPerGrid = (num_points + threadsPerBlock - 1) / threadsPerBlock;
    cudaKernel<<<blocksPerGrid, threadsPerBlock>>>(d_points, d_model_view_matrix, d_projection_matrix, d_viewport, min_x, max_x, min_y, max_y, d_selected_indices, num_points);

    // 复制结果回主机
    int selected_count;
    cudaMemcpy(&selected_count, d_selected_indices, sizeof(int), cudaMemcpyDeviceToHost);
    selected_indices.resize(selected_count);
    cudaMemcpy(selected_indices.data(), d_selected_indices + 1, selected_count * sizeof(int), cudaMemcpyDeviceToHost);

    // 释放设备内存
    cudaFree(d_points);
    cudaFree(d_model_view_matrix);
    cudaFree(d_projection_matrix);
    cudaFree(d_viewport);
    cudaFree(d_selected_indices);
}

