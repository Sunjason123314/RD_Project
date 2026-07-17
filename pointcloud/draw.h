#pragma once
#ifndef DRAW_H
#define DRAW_H
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <device_launch_parameters.h>
#include <vector>
#include <QGLViewer/camera.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/StdVector>
#include <QGLViewer/camera.h>

__global__ void cudaKernel(const float* points, const float* model_view_matrix, const float* projection_matrix, const GLint* viewport, int min_x, int max_x, int min_y, int max_y, int* selected_indices, int num_points);

void performAreaSelectionCuda(const std::vector<float>& points, const float* model_view_matrix, const float* projection_matrix, const GLint* viewport, int min_x, int max_x, int min_y, int max_y, std::vector<int>& selected_indices);



#endif
