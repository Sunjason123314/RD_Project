#pragma once
/*相位匹配方法——cuda*/

//std
#include <iostream>
#include <math.h>

//opencv
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

//mine
#ifdef USECUDA
#include "cudaimage.h"
#include "cublas_v2.h"
#endif

using namespace std;

class Matcher
{
private:
    /* data */
    bool match_2D=false;//检查初始化

    double grid_size;  // 每个网格的相位跨度
    int grid_resolution; // 网格总数（单边）
    const int max_points_per_grid = 150; // 每个网格最大存储点数
    int *d_grid_count;//每格数量
    int2 *d_grid_data;//每格存储的坐标
public:
    Matcher(/* args */);
    Matcher(float ratio);
    ~Matcher();
    void cudafree();
    // int winsize_col=3;
    // int winsize_row=3;

    #ifdef USECUDA
    //主函数-开始匹配
    void match_cuda(CudaImage<double> L,CudaImage<double> R,CudaImage<double>& output,double minerr,dim3 blocksize,dim3 gridsize);
    void match_2D_cuda(CudaImage<double> L_v,CudaImage<double> L_h,CudaImage<double> R_v,CudaImage<double> R_h,CudaImage<double>& output_xy,double minerr,dim3 blocksize,dim3 gridsize);
    void reprojectTo3D_cuda(CudaImage<double>& disp,CudaImage<double>& Pointcloud,cv::Mat Q,dim3 blocksize,dim3 gridsize);
    void reprojectTo3D_2D_cuda(CudaImage<double>& disp,CudaImage<double>& Pointcloud,CudaImage<double>& PL,CudaImage<double>& PR,dim3 blocksize,dim3 gridsize);
    // void reprojectTo3D_refraction_2D_cuda(CudaImage<double>& disp,CudaImage<double>& Pointcloud,CudaImage<double>& PL,CudaImage<double>& PR,dim3 blocksize,dim3 gridsize);
    void reprojectTo3D_refraction_2D_cuda(CudaImage<double>& disp,CudaImage<double>& Pointcloud,
        CudaImage<double>& camMat_L,CudaImage<double>& camMat_R,
        CudaImage<double>& R,CudaImage<double>& T,
        CudaImage<double>& PL,CudaImage<double>& PR,
        const double *norm_L,const double *norm_R,const double n,const double d,
        dim3 blocksize,dim3 gridsize,CudaImage<double>& err_Lx,CudaImage<double>& err_Ly);
    #endif

    void stereoSGBM(cv::Mat lpng,cv::Mat rpng,string filename,cv::Mat &disp);
    void match(cv::Mat& img_L,cv::Mat& img_R,cv::Mat& disp,double minerr);
    void reprojectTo3D(cv::Mat &dif_map, cv::Mat &depth_map,cv::Mat Q,pcl::PointCloud<pcl::PointXYZ>& depth_cloud);
};
