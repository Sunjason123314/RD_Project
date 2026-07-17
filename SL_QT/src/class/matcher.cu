#include "matcher.h"

/// @brief 2D匹配版本的初始化
/// @param ratio 单个网格比率
Matcher::Matcher(float ratio)
{
    grid_size = ratio * 2 * M_PI;
    grid_resolution = static_cast<int>(2 * M_PI / grid_size);

    // 分配网格存储
    const int total_grids = grid_resolution * grid_resolution;
    cudaMalloc(&d_grid_count, sizeof(int) * total_grids);
    cudaMalloc(&d_grid_data, sizeof(int2) * total_grids * max_points_per_grid);
    cudaMemset(d_grid_count, 0, sizeof(int) * total_grids);

    // cout<<"[match_2D_cuda]:grid_size:"<<grid_size<<endl;
    // cout<<"[match_2D_cuda]:grid_resolution:"<<grid_resolution<<endl;

    // cout<<"[match_2D_cuda]:total_grids:"<<total_grids<<endl;
    // cout<<"[match_2D_cuda]:max_points_per_grid:"<<max_points_per_grid<<endl;
    match_2D=true;
}
void Matcher::cudafree()
{
    const int total_grids = grid_resolution * grid_resolution;
    cudaMemset(d_grid_count, 0, sizeof(int) * total_grids);
    cudaMemset(d_grid_data, 0, sizeof(int2) * total_grids * max_points_per_grid);
}

/*************************************************纵坐标匹配*********************************************************/
/// @brief 获取三次样条系数
/// @param GivenX 
/// @param GivenY 
/// @param PartialDerivative 
/// @param GivenNum 
/// @param LeftB 
/// @param RightB 
/// @return 
__inline__ __device__ void GetPartialDerivative(const double* GivenX,const double* GivenY, double* PartialDerivative,int GivenNum,const double& LeftB=0,const double& RightB=0)
{
    //追赶法解方程求二阶偏导数
    float a[5];                //  a:稀疏矩阵最下边一串数
    float b[5];                //  b:稀疏矩阵最中间一串数
    float c[5];                //  c:稀疏矩阵最上边一串数
    float d[5];

    float f[5];

    float bt[5];
    float gm[5];

    float h[5];
// /*
    //边界条件2
    for(int i=0;i<GivenNum;i++)  b[i]=2;
    for(int i=0;i<GivenNum-1;i++)  h[i]=GivenX[i+1]-GivenX[i];
    for(int i=1;i<GivenNum-1;i++)  a[i]=h[i-1]/(h[i-1]+h[i]);
    a[GivenNum-1]=1;

    c[0]=1;
    for(int i=1;i<GivenNum-1;i++)  c[i]=h[i]/(h[i-1]+h[i]);

    for(int i=0;i<GivenNum-1;i++)
        f[i]=(GivenY[i+1]-GivenY[i])/(GivenX[i+1]-GivenX[i]);

    for(int i=1;i<GivenNum-1;i++)  d[i]=6*(f[i]-f[i-1])/(h[i-1]+h[i]);

    d[1]=d[1]-a[1]*LeftB;
    d[GivenNum-2]=d[GivenNum-2]-c[GivenNum-2]*RightB;

    bt[1]=c[1]/b[1];
    for(int i=2;i<GivenNum-2;i++)  bt[i]=c[i]/(b[i]-a[i]*bt[i-1]);

    gm[1]=d[1]/b[1];
    for(int i=2;i<=GivenNum-2;i++)  gm[i]=(d[i]-a[i]*gm[i-1])/(b[i]-a[i]*bt[i-1]);

    PartialDerivative[GivenNum-2]=gm[GivenNum-2];//
    for(int i=GivenNum-3;i>=1;i--)  PartialDerivative[i]=gm[i]-bt[i]*PartialDerivative[i+1];

    PartialDerivative[0]=LeftB;
    PartialDerivative[GivenNum-1]=RightB;
// */
/*
    //边界条件1
    for(int i=0;i<GivenNum;i++)  b[i]=2;          //  中间一串数为2
    for(int i=0;i<GivenNum-1;i++)  h[i]=GivenX[i+1]-GivenX[i];                   // 各段步长
    for(int i=1;i<GivenNum-1;i++)  a[i]=h[i-1]/(h[i-1]+h[i]);
    a[GivenNum-1]=1;

    c[0]=1;
    for(int i=1;i<GivenNum-1;i++)  c[i]=h[i]/(h[i-1]+h[i]);

    for(int i=0;i<GivenNum-1;i++)
        f[i]=(GivenY[i+1]-GivenY[i])/(GivenX[i+1]-GivenX[i]);

    d[0]=6*(f[0]-LeftB)/h[0];
    d[GivenNum-1]=6*(RightB-f[GivenNum-2])/h[GivenNum-2];

    for(int i=1;i<GivenNum-1;i++)  d[i]=6*(f[i]-f[i-1])/(h[i-1]+h[i]);

    bt[0]=c[0]/b[0];                                             //  追赶法求解方程
    for(int i=1;i<GivenNum-1;i++)  bt[i]=c[i]/(b[i]-a[i]*bt[i-1]);

    gm[0]=d[0]/b[0];
    for(int i=1;i<=GivenNum-1;i++)  gm[i]=(d[i]-a[i]*gm[i-1])/(b[i]-a[i]*bt[i-1]);

    PartialDerivative[GivenNum-1]=gm[GivenNum-1];
    for(int i=GivenNum-2;i>=0;i--)  PartialDerivative[i]=gm[i]-bt[i]*PartialDerivative[i+1];
*/
}

//单个插值的实现
__inline__ __device__ void SinglePointInterp(const double* GivenX,const double* GivenY, double* PartialDerivative,int GivenNum, const double& x,double& y)
{
    int klo,khi,k;
    klo=0; khi=GivenNum-1;
    double hh,bb,aa;

    while(khi-klo>1)            //  二分法查找x所在区间段
    {
        k=(khi+klo)>>1;
        if(GivenX[k]>x)  khi=k;
        else klo=k;
    }
    hh=GivenX[khi]-GivenX[klo];

    aa=(GivenX[khi]-x)/hh;
    bb=(x-GivenX[klo])/hh;

    y=aa*GivenY[klo]+bb*GivenY[khi]+((aa*aa*aa-aa)*PartialDerivative[klo]+(bb*bb*bb-bb)*PartialDerivative[khi])*hh*hh/6.0;
}

// 线性插值函数
__device__ double linearInterpolation(double x, const double* xData, const double* yData, int dataSize) {
    // 找到左右最近的自变量数据点
    int leftIdx = -1;
    int rightIdx = -1;
    for (int i = 0; i < dataSize; ++i) {
        if (xData[i] <= x) {
            leftIdx = i;
        } else {
            rightIdx = i;
            break;
        }
    }

    // 如果x小于或等于最小的自变量数据点，则直接返回yData[0]
    if (leftIdx == -1) {
        return yData[0];
    }

    // 如果x大于或等于最大的自变量数据点，则直接返回yData[dataSize - 1]
    if (rightIdx == -1) {
        return yData[dataSize - 1];
    }

    // 计算插值
    double xLeft = xData[leftIdx];
    double xRight = xData[rightIdx];
    double yLeft = yData[leftIdx];
    double yRight = yData[rightIdx];
    double slope = (yRight - yLeft) / (xRight - xLeft);
    double yInterpolated = yLeft + slope * (x - xLeft);

    return yInterpolated;
}

/// @brief 亚像素搜索极线中匹配相位(暴力匹配)
/// @param L 左相机图
/// @param R 右相机图
/// @param output 输出视差图
/// @param colsize 图像x方向尺寸，用于遍历
/// @return 
__global__ void __match_cuda(CudaImage<double> L,CudaImage<double> R,CudaImage<double> output,double minerr)//,CudaImage<double> err
{
    int x=blockIdx.x*blockDim.x+threadIdx.x;
    int y=blockIdx.y*blockDim.y+threadIdx.y;
    
    double Lp=L(x,y);//待搜索的左图像相位
    if(Lp<0)//跳过剔除点
    {
        // output(x,y)=-100;
        return;
    }
    
    //整像素搜索
    double min=100;//存储搜索到的相位差最小值
    int mincol=-1;
    for (int i = 0; i < L.x_size; i++)
    {
        double L_R=abs(Lp-R(i,y));//相位差的绝对值
        if(L_R<=min)
        {
            min=L_R;//更新最小值
            mincol=i;//更新最小值对应的x坐标
        }
        if(min<=minerr)
        {
            break;
        }
    }
    if(min>minerr)//设置阈值，超出限制跳过搜索
    {
        // output(x,y)=-100;
        return;
    }
    // else
    // {
        // output(x,y)=x-mincol;//计算视差(整像素)
        // return;
    // }
    
    //获取邻域插值点
    double xs[5],ys[5];
    for (int i = 0; i < 5; i++)
    {
        ys[i]=2-i+mincol;//此处需要控制自变量递增
        // ys[i]=-2+i+mincol;//此处需要控制自变量递增
        // if((ys[i]<0)||(ys[i]>(L.x_size-1)))//不确定是否需要
        // {
        //     return;//超出边界，终止搜索亚像素相位
        // }
        xs[i]=R(ys[i],y);
    }

    //三次样条插值
    double parameter[5];
    GetPartialDerivative(xs,ys,parameter,5);

    double subpix_Rx=0;
    SinglePointInterp(xs,ys,parameter,5,Lp, subpix_Rx);
    
    output(x,y)=x-subpix_Rx;//计算视差（亚像素）
    

    //线性插值
    // double temp=x-linearInterpolation(Lp,xs,ys,5);
    // output(x,y)=temp;//计算视差（亚像素）
}

/// @brief 左右图像相位匹配
/// @param L 左相位图
/// @param R 右相位图
/// @param output 输出视差图
void Matcher::match_cuda(CudaImage<double> L,CudaImage<double> R,CudaImage<double>& output,double minerr,dim3 blocksize,dim3 gridsize)
{
    __match_cuda<<<gridsize,blocksize>>>(L,R,output,minerr);//,err
    checkCudaErrors(cudaDeviceSynchronize());
    // checkCudaErrors(cudaStreamSynchronize(cudaStreamDefault));//暂时修改

    // cudaError_t cudaError = cudaGetLastError();
    // if (cudaError != cudaSuccess) 
    // {
    //     fprintf(stderr, "CUDA error: %s\n", cudaGetErrorString(cudaError));
    // }

    // showcudaimage_match(err);
    // showcudaimage_match(output);
}

/*************************************************横纵坐标相位匹配*********************************************************/
/// @brief 预处理，给相位划分网格
/// @param right_phi_v 
/// @param right_phi_h 
/// @param width 
/// @param height 
/// @param grid_count 
/// @param grid_data 
/// @param max_points_per_grid 
/// @param grid_resolution 
/// @param grid_size 
/// @return 
__global__ void __buildPhaseGrid(CudaImage<double> right_phi_v, CudaImage<double> right_phi_h,
    int width, int height,
    int* grid_count, int2* grid_data, int max_points_per_grid,const int grid_resolution,const double grid_size)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= width || y >= height) return;

    double phi_v = right_phi_v(x,y);
    double phi_h = right_phi_h(x,y);
    if (phi_v < 0 || phi_h < 0) return;

    
    // 计算网格索引
    int grid_v = min(static_cast<int>(phi_v / grid_size), grid_resolution - 1);
    int grid_h = min(static_cast<int>(phi_h / grid_size), grid_resolution - 1);
    int grid_idx = grid_h * grid_resolution + grid_v;
    
    // 原子操作写入网格（确保线程安全）
    int pos = atomicAdd(&grid_count[grid_idx], 1);
    if (pos < max_points_per_grid) {
        grid_data[grid_idx * max_points_per_grid + pos] = make_int2(x, y);
    }
}

/// @brief 一阶局部多项式亚像素逆插值
/// @param hs 3*3的像素块，通道1
/// @param vs 3*3的像素块，通道2
/// @param h 通道1的搜索灰度
/// @param v 通道2的搜索灰度
/// @return 搜索结果
__device__ float2 findSubPixel(double hs[3][3], double vs[3][3], double h, double v) {
    // 计算hs块的梯度参数
    double sum_x_hs = (hs[0][2] + hs[1][2] + hs[2][2]) - (hs[0][0] + hs[1][0] + hs[2][0]);
    double sum_y_hs = (hs[2][0] + hs[2][1] + hs[2][2]) - (hs[0][0] + hs[0][1] + hs[0][2]);
    double sum_f_hs = hs[0][0] + hs[0][1] + hs[0][2] + hs[1][0] + hs[1][1] + hs[1][2] + hs[2][0] + hs[2][1] + hs[2][2];
    double a_h = sum_x_hs / 6.0;
    double b_h = sum_y_hs / 6.0;
    double c_h = sum_f_hs / 9.0;

    // 计算vs块的梯度参数
    double sum_x_vs = (vs[0][2] + vs[1][2] + vs[2][2]) - (vs[0][0] + vs[1][0] + vs[2][0]);
    double sum_y_vs = (vs[2][0] + vs[2][1] + vs[2][2]) - (vs[0][0] + vs[0][1] + vs[0][2]);
    double sum_f_vs = vs[0][0] + vs[0][1] + vs[0][2] + vs[1][0] + vs[1][1] + vs[1][2] + vs[2][0] + vs[2][1] + vs[2][2];
    double a_v = sum_x_vs / 6.0;
    double b_v = sum_y_vs / 6.0;
    double c_v = sum_f_vs / 9.0;

    // 计算目标与当前值的差异
    double h_delta = h - c_h;
    double v_delta = v - c_v;

    // 解线性方程组
    float2 result = {0.0f, 0.0f};
    double det = a_h * b_v - b_h * a_v;

    if (fabs(det) > 1e-6) { // 确保行列式不为零
        result.x = static_cast<float>((h_delta * b_v - v_delta * b_h) / det);
        result.y = static_cast<float>((v_delta * a_h - h_delta * a_v) / det);
    }

    return result;
}

/// @brief 二阶局部多项式拟合亚像素从插值
/// @param hs 3*3的像素块，通道1
/// @param vs 3*3的像素块，通道2
/// @param h 通道1的搜索灰度
/// @param v 通道2的搜索灰度
/// @param x 搜索结果x
/// @param y 搜索结果y
/// @return 
__device__ void findSubpixelPoint(
    const double hs[9], const double vs[9],
    double h, double v,
    double &x, double &y
) {
    // 拟合hs块的二次多项式系数
    double b0_hs = hs[0] + hs[1] + hs[2] + hs[3] + hs[4] + hs[5] + hs[6] + hs[7] + hs[8];
    double b1_hs = (-1)*hs[0] + (-1)*hs[1] + (-1)*hs[2] +
                   0*(hs[3] + hs[4] + hs[5]) +
                   1*hs[6] + 1*hs[7] + 1*hs[8];
    double b2_hs = (-1)*hs[0] + 0*hs[1] + 1*hs[2] +
                   (-1)*hs[3] + 0*hs[4] + 1*hs[5] +
                   (-1)*hs[6] + 0*hs[7] + 1*hs[8];
    double b3_hs = 1*(hs[0] + hs[1] + hs[2]) +
                   0*(hs[3] + hs[4] + hs[5]) +
                   1*(hs[6] + hs[7] + hs[8]);
    double b4_hs = 1*hs[0] + 0*hs[1] + 1*hs[2] +
                   1*hs[3] + 0*hs[4] + 1*hs[5] +
                   1*hs[6] + 0*hs[7] + 1*hs[8];
    double b5_hs = 1*hs[0] + 0*hs[1] -1*hs[2] +
                   0*hs[3] +0*hs[4] +0*hs[5] +
                   (-1)*hs[6] +0*hs[7] +1*hs[8];
    
    double a0 = (20*b0_hs - 12*b3_hs - 12*b4_hs)/36.0;
    double a3 = (-12*b0_hs + 18*b3_hs)/36.0;
    double a4 = (-12*b0_hs + 18*b4_hs)/36.0;
    double a1 = b1_hs/6.0;
    double a2 = b2_hs/6.0;
    double a5 = b5_hs/4.0;
    
    // 拟合vs块的二次多项式系数
    double b0_vs = vs[0] + vs[1] + vs[2] + vs[3] + vs[4] + vs[5] + vs[6] + vs[7] + vs[8];
    double b1_vs = (-1)*vs[0] + (-1)*vs[1] + (-1)*vs[2] +
                   0*(vs[3] + vs[4] + vs[5]) +
                   1*vs[6] + 1*vs[7] + 1*vs[8];
    double b2_vs = (-1)*vs[0] + 0*vs[1] + 1*vs[2] +
                   (-1)*vs[3] + 0*vs[4] + 1*vs[5] +
                   (-1)*vs[6] + 0*vs[7] + 1*vs[8];
    double b3_vs = 1*(vs[0] + vs[1] + vs[2]) +
                   0*(vs[3] + vs[4] + vs[5]) +
                   1*(vs[6] + vs[7] + vs[8]);
    double b4_vs = 1*vs[0] + 0*vs[1] + 1*vs[2] +
                   1*vs[3] + 0*vs[4] + 1*vs[5] +
                   1*vs[6] + 0*vs[7] + 1*vs[8];
    double b5_vs = 1*vs[0] + 0*vs[1] -1*vs[2] +
                   0*vs[3] +0*vs[4] +0*vs[5] +
                   (-1)*vs[6] +0*vs[7] +1*vs[8];
    
    double b0 = (20*b0_vs - 12*b3_vs - 12*b4_vs)/36.0;
    double b3 = (-12*b0_vs + 18*b3_vs)/36.0;
    double b4 = (-12*b0_vs + 18*b4_vs)/36.0;
    double b1 = b1_vs/6.0;
    double b2 = b2_vs/6.0;
    double b5 = b5_vs/4.0;
    
    // 牛顿迭代法求解方程组
    x = 0.0;
    y = 0.0;
    const int max_iter = 10;
    const double tolerance = 1e-6;
    
    for (int iter = 0; iter < max_iter; ++iter) {
        double f_hs = a0 + a1*x + a2*y + a3*x*x + a4*y*y + a5*x*y - h;
        double f_vs = b0 + b1*x + b2*y + b3*x*x + b4*y*y + b5*x*y - v;
        
        if (fabs(f_hs) < tolerance && fabs(f_vs) < tolerance)
            break;
        
        // 雅可比矩阵
        double J11 = a1 + 2*a3*x + a5*y;
        double J12 = a2 + 2*a4*y + a5*x;
        double J21 = b1 + 2*b3*x + b5*y;
        double J22 = b2 + 2*b4*y + b5*x;
        
        double det = J11 * J22 - J12 * J21;
        if (fabs(det) < 1e-12)
            break; // 避免除零
        
        // 计算增量
        double delta_x = (-f_hs * J22 + f_vs * J12) / det;
        double delta_y = (f_hs * J21 - f_vs * J11) / det;
        
        x += delta_x;
        y += delta_y;
    }
}

/// @brief 核函数：横纵条纹相位匹配
/// @param L_v 左相机纵条纹相位
/// @param L_h 左相机横条纹相位
/// @param R_v 右相机纵条纹相位
/// @param R_h 右相机横条纹相位
/// @param output_xy 匹配的亚像素点
/// @param minerr 匹配允许的最大的相位误差阈值
/// @return 
// __global__ void __match_2D_cuda(CudaImage<double> L_v,CudaImage<double> L_h,CudaImage<double> R_v,CudaImage<double> R_h,CudaImage<double> output_xy,int width,int height,double minerr)
// {
//     int x=blockIdx.x*blockDim.x+threadIdx.x;
//     int y=blockIdx.y*blockDim.y+threadIdx.y;
    
//     double Lp_v=L_v(x,y);//待搜索的左图像相位(x方向)
//     double Lp_h=L_h(x,y);//待搜索的左图像相位(x方向)
//     if((Lp_v<0)||(Lp_h<0))//跳过剔除点
//     {
//         return;
//     }
    
//     //整像素搜索
//     int int_x=-1,int_y=-1;//初始整像素位置
//     float min=100;//最小值
    
//     //设置搜索边界范围
//     int a=600,b=100;
//     int bound_v_min=x-a,bound_v_max=x+a;
//     int bound_h_min=y-b,bound_h_max=y+b;
//     if(bound_v_min<0)
//     {
//         bound_v_min=0;
//     }
//     if(bound_v_max>width)
//     {
//         bound_v_min=width;
//     }
//     if(bound_h_min<0)
//     {
//         bound_h_min=0;
//     }
//     if(bound_h_max>height)
//     {
//         bound_h_max=height;
//     }

    
//     for (int i = bound_v_min; i < bound_v_max; i++)
//     {
//         for (int j = bound_h_min; j < bound_h_max; j++)
//         {
//             if(R_h(i,j)>0)
//             {
//                 float h=R_h(i,j)-Lp_h;
//                 float v=R_v(i,j)-Lp_v;
//                 float min_temp=abs(h)+abs(v);
//                 if(min_temp<minerr)
//                 {
//                     min=min_temp;
//                     int_x=i;
//                     int_y=j;
//                     goto end;
//                 }
//             }
//         }
//     }
//     end:

//     if(min>minerr)//设置阈值，超出限制跳过搜索
//     {
//         return;
//     }

//     //==一阶局部多项式亚像素逆插值
//     double hs[3][3], vs[3][3];
//     for (int j = 0; j < 3; j++)
//     {
//         for (int i = 0; i < 3; i++)
//         {
//             hs[i][j] = R_h(i-1+int_x,j-1+int_y);
//             vs[i][j] = R_v(i-1+int_x,j-1+int_y);
//         }
//     }
//     float2 result;
//     result=findSubPixel(hs, vs, Lp_h, Lp_v);
//     output_xy(x,y,0)=int_x+result.y;
//     output_xy(x,y,1)=int_y+result.x;
// }

/// @brief 横纵条纹匹配（网格整像素搜索）
/// @param L_v 左相机横向相位
/// @param L_h 左相机纵向相位
/// @param R_v 右相机横向相位
/// @param R_h 右相机纵向相位
/// @param output_xy 输出匹配值
/// @param grid_count 网格存储数量
/// @param grid_data 网格存储数据
/// @param max_points_per_grid 每个网格存储最大数量
/// @param grid_resolution 网格相位分辨力
/// @param grid_size 网格尺寸
/// @param minerr 相位误差
/// @return 
__global__ void __match_2D_cuda(CudaImage<double> L_v,CudaImage<double> L_h,CudaImage<double> R_v,CudaImage<double> R_h,CudaImage<double> output_xy,int* grid_count, int2* grid_data, int max_points_per_grid,int grid_resolution,double grid_size,double minerr)
{
    int x=blockIdx.x*blockDim.x+threadIdx.x;
    int y=blockIdx.y*blockDim.y+threadIdx.y;
    
    double Lp_v=L_v(x,y);//待搜索的左图像相位(x方向)
    double Lp_h=L_h(x,y);//待搜索的左图像相位(x方向)
    if((Lp_v<0)||(Lp_h<0))//跳过剔除点
    {
        return;
    }
    
    //整像素搜索
    // 计算当前相位所属的网格
    int center_grid_x = static_cast<int>(Lp_v / grid_size);
    int center_grid_y = static_cast<int>(Lp_h / grid_size);
    double min_dist_sq = DBL_MAX;
    int2 nearest = make_int2(-1, -1);

    // 搜索3x3邻域网格
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            int grid_x = center_grid_x + dx;
            int grid_y = center_grid_y + dy;
            if (grid_x < 0 || grid_x >= grid_resolution || grid_y < 0 || grid_y >= grid_resolution) continue;

            int grid_idx = grid_y * grid_resolution + grid_x;
            int count = min(grid_count[grid_idx], max_points_per_grid); // 避免越界

            // 遍历网格中的候选点
            for (int i = 0; i < count; i++) {
                int2 candidate = grid_data[grid_idx * max_points_per_grid + i];
                double candidate_phi_x = R_v(candidate.x,candidate.y);
                double candidate_phi_y = R_h(candidate.x,candidate.y);

                // 计算相位差的平方（避免开根号）
                double dx_phi = candidate_phi_x - Lp_v;
                double dy_phi = candidate_phi_y - Lp_h;
                double dist_sq = dx_phi * dx_phi + dy_phi * dy_phi;

                if (dist_sq < min_dist_sq) {
                    min_dist_sq = dist_sq;
                    nearest = candidate;
                }
            }
        }
    }

    // // 计算相位差的平方（避免开根号）
    // double dx_phi = R_v(nearest.x,nearest.y) - Lp_v;
    // double dy_phi = R_h(nearest.x,nearest.y) - Lp_h;
    // double dist_sq = dx_phi * dx_phi + dy_phi * dy_phi;
    // if(dist_sq>0.00001)
    // {
    //     return;
    // }
    

    // if(880<x&&x<900&&880<y&&y<900)
    // {
    //     double Rv=R_v(nearest.x,nearest.y)-Lp_v,Rh=R_h(nearest.x,nearest.y)-Lp_h;
    //     // printf("/%f,%f,%f/",Rv,Rh,abs(Rv)-abs(Rh));
    //     printf("/%f/",1000*(abs(Rv)-abs(Rh)));
    //     return;
    // }
    


    //==一阶局部多项式亚像素逆插值
    // double hs[3][3], vs[3][3];
    // for (int j = 0; j < 3; j++)
    // {
    //     for (int i = 0; i < 3; i++)
    //     {
    //         hs[i][j] = R_h(i-1+nearest.x,j-1+nearest.y);
    //         vs[i][j] = R_v(i-1+nearest.x,j-1+nearest.y);
    //     }
    // }
    // float2 result;
    // result=findSubPixel(hs, vs, Lp_h, Lp_v);
    // output_xy(x,y,0)=nearest.x+result.y;
    // output_xy(x,y,1)=nearest.y+result.x;

    //二阶局部多项式拟合
    double hs[9], vs[9];
    for (int j = 0; j < 3; j++)
    {
        for (int i = 0; i < 3; i++)
        {
            hs[i*3+j] = R_h(i-1+nearest.x,j-1+nearest.y);
            vs[i*3+j] = R_v(i-1+nearest.x,j-1+nearest.y);
        }
    }
    double x_,y_;
    findSubpixelPoint(hs, vs, Lp_h, Lp_v,x_,y_);
    output_xy(x,y,0)=nearest.x+x_;
    output_xy(x,y,1)=nearest.y+y_;

    //直接给出整像素
    // output_xy(x,y,0)=nearest.x;
    // output_xy(x,y,1)=nearest.y;
}

/// @brief 横纵坐标相位匹配
/// @param L_h 左相位图（横向）
/// @param L_v 左相位图（纵向）
/// @param R_h 右相位图（横向）
/// @param R_v 右相位图（纵向）
/// @param output_xy 匹配坐标x
/// @param minerr 搜索最误差阈值
/// @param blocksize 
/// @param gridsize 
void Matcher::match_2D_cuda(CudaImage<double> L_v,CudaImage<double> L_h,
    CudaImage<double> R_v,CudaImage<double> R_h,
    CudaImage<double>& output_xy,
    double minerr,dim3 blocksize,dim3 gridsize)
{
    if(match_2D==false)
    {
        std::cerr << "[Matcher]:未初始化2D" << std::endl;
    }
    dim3 blocksize_,gridsize_;
    blocksize_.x=blocksize.x/2;
    blocksize_.y=blocksize.y/2;
    gridsize_.x=gridsize.x*2;
    gridsize_.y=gridsize.y*2;
    __buildPhaseGrid<<<gridsize_,blocksize_>>>(R_v,R_h,R_v.x_size,R_v.y_size,d_grid_count,d_grid_data,max_points_per_grid,grid_resolution,grid_size);//,err
    checkCudaErrors(cudaDeviceSynchronize());
    __match_2D_cuda<<<gridsize_,blocksize_>>>(L_v,L_h,R_v,R_h,output_xy,d_grid_count,d_grid_data,max_points_per_grid,grid_resolution,grid_size,minerr);//,err
    checkCudaErrors(cudaDeviceSynchronize());
}

/// @brief 横纵坐标相位匹配
/// @param L_h 左相位图（横向）
/// @param L_v 左相位图（纵向）
/// @param R_h 右相位图（横向）
/// @param R_v 右相位图（纵向）
/// @param output_xy 匹配坐标x
/// @param minerr 搜索最误差阈值
/// @param blocksize 
/// @param gridsize 
// void Matcher::match_2D_cuda(CudaImage<double> L_v,CudaImage<double> L_h,
//     CudaImage<double> R_v,CudaImage<double> R_h,
//     CudaImage<double>& output_xy,
//     double minerr,dim3 blocksize,dim3 gridsize)
// {
//     dim3 blocksize_;
//     dim3 gridsize_;
//     blocksize_.x=blocksize.x/2;
//     blocksize_.y=blocksize.y/2;
//     gridsize_.x=gridsize.x*2;
//     gridsize_.y=gridsize.y*2;
//     // __match_2D_cuda<<<gridsize_,blocksize_>>>(L_v,L_h,R_v,R_h,output_xy,output_xy.x_size,output_xy.y_size,minerr);
//     __match_2D_cuda<<<gridsize_,blocksize_>>>(L_v,L_h,R_v,R_h,output_xy,output_xy.x_size,output_xy.y_size,minerr);//,err
//     checkCudaErrors(cudaDeviceSynchronize());
// }

/*************************************************纵坐标相位匹配的三维重建*********************************************************/

/// @brief 点云计算-核函数
/// @param disp 视差图
/// @param Pointcloud 点云图
/// @param Q 重投影矩阵
/// @return 
__global__ void __reprojectTo3D_cuda(CudaImage<double> disp,CudaImage<double> Pointcloud,CudaImage<double> Q)
{
    int x=blockIdx.x*blockDim.x+threadIdx.x;
    int y=blockIdx.y*blockDim.y+threadIdx.y;

    //如果视差为0表示调试度滤波已经剔除，不进行计算，否则w为0计算失败
    if(disp(x,y)<=0)
    {
        Pointcloud(x,y,0)=0;
        Pointcloud(x,y,1)=0;
        Pointcloud(x,y,2)=0;
        // return;
    }
    else
    {
        // 根据公式计算三维坐标
        double w=Q(2,3)*disp(x,y)+Q(3,3);
        double x_p=(Q(0,0)*x+Q(3,0))/w;
        double y_p=(Q(1,1)*y+Q(3,1))/w;
        double z_p=Q(3,2)/w;

        Pointcloud(x,y,0)=x_p;
        Pointcloud(x,y,1)=y_p;
        Pointcloud(x,y,2)=z_p;
    }
    

    // printf("%d,%d disp(x,y)：%f\n",x,y,disp(x,y));
}

/// @brief 计算点云
/// @param disp 视差图
/// @param Pointcloud 存储点云信息
/// @param Q 重投影矩阵
void Matcher::reprojectTo3D_cuda(CudaImage<double>& disp,CudaImage<double>& Pointcloud,cv::Mat Q,dim3 blocksize,dim3 gridsize)
{
    CudaImage<double> Q_cuda(Q);
    __reprojectTo3D_cuda<<<gridsize,blocksize>>>(disp, Pointcloud,Q_cuda);
    checkCudaErrors(cudaDeviceSynchronize());
    // checkCudaErrors(cudaStreamSynchronize(cudaStreamDefault));//暂时修改
}

/*************************************************横纵坐标匹配的三维重建*********************************************************/
//矩阵转置
template <typename T>
__inline__ __device__ void transposeMatrix(const T* A, T* B, int rows, int cols) {
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            B[j * rows + i] = A[i * cols + j];
        }
    }
}

//矩阵乘法
template <typename T>
__inline__ __device__ void matrixMultiplication(const T* A, const T* B, T* C, int rowsA, int colsA, int colsB) {
    for (int i = 0; i < rowsA; ++i) {
        for (int j = 0; j < colsB; ++j) {
            C[i * colsB + j] = 0.0;
            for (int k = 0; k < colsA; ++k) {
                C[i * colsB + j] += A[i * colsA + k] * B[k * colsB + j];
            }
        }
    }
}

//函数用于进行矩阵的LU分解, L,U初始为单位矩阵
template <typename T>
__inline__  __device__ void performLUDecomposition(const T* matrix, T* L, T* U, int size) {
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            if (i > j) {
                L[i * size + j] = matrix[i * size + j];
                for (int k = 0; k < j; k++) {
                    L[i * size + j] -= L[i * size + k] * U[k * size + j];
                }
                L[i * size + j] /= U[j * size + j];
            }
            else {
                U[i * size + j] = matrix[i * size + j];
                for (int k = 0; k < i; k++) {
                    U[i * size + j] -= L[i * size + k] * U[k * size + j];
                }
            }
        }
    }
}

//3×3矩阵求逆（三维重建中使用）
template <typename T>
__inline__ __device__ void calculateInverseMatrix_size3(const T* matrix, T* matrix_inv)
{
    T L[3 * 3];
    T U[3 * 3];
    T Y[3 * 3];
    int size = 3;
    // 初始化L,U矩阵为单位矩阵
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            if (i == j) {
                L[i * size + j] = 1.0;
                U[i * size + j] = 1.0;
            }
            else {
                L[i * size + j] = 0.0;
                U[i * size + j] = 0.0;
            }
        }
    }
    //执行LU分解
    performLUDecomposition(matrix, L, U, size);

    //计算逆矩阵， 先 Ly = E 后 Ux = y
    for (int k = 0; k < size; k++) {
        //解Ly = E
        for (int i = 0; i < size; i++) {
            T sum = 0.0;
            for (int j = 0; j < i; j++) {
                sum += L[i * size + j] * Y[j * size + k];
            }
            Y[i * size + k] = (i == k) ? 1.0 : 0.0 - sum;
        }

        //解Ux = y  ，X = matrix_inv
        for (int i = size - 1; i >= 0; i--) {
            T sum = 0.0;
            for (int j = i + 1; j < size; j++) {
                sum += U[i * size + j] * matrix_inv[j * size + k];
            }
            matrix_inv[i * size + k] = (Y[i * size + k] - sum) / U[i * size + i];
        }
    }
}

/// @brief 横纵条纹匹配三维重建核函数
/// @param disp 输入匹配点
/// @param Pointcloud 输出点云
/// @param PL 左相机投影矩阵
/// @param PR 右相机投影矩阵
/// @return 
__global__ void __reprojectTo3D_2D_cuda(CudaImage<double> disp,CudaImage<double> Pointcloud,CudaImage<double> err_Lx,CudaImage<double> err_Ly,
                                const double* PL, const double* PR)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if(disp(x,y,0)==-100)//未匹配到的直接跳过
    {
        return;
    }

    double A[4 * 3];
    double B[4 * 1];
    double AT[3 * 4];
    double ATA[3 * 3];
    double ATA_invert[3 * 3];
    double ATA_invertAT[3 * 4];
    double res_temp[3 * 1];
    double res[3 * 1];
    A[0] = x * PL[8] - PL[0];
    A[1] = x * PL[9] - PL[1];
    A[2] = x * PL[10] - PL[2];

    A[3] = y * PL[8] - PL[4];
    A[4] = y * PL[9] - PL[5];
    A[5] = y * PL[10] - PL[6];

    A[6] = disp(x,y,0) * PR[8] - PR[0];
    A[7] = disp(x,y,0) * PR[9] - PR[1];
    A[8] = disp(x,y,0) * PR[10] - PR[2];

    A[9] = disp(x,y,1) * PR[8] - PR[4];
    A[10] = disp(x,y,1) * PR[9] - PR[5];
    A[11] = disp(x,y,1) * PR[10] - PR[6];
    B[0] = PL[3] - x * PL[11];
    B[1] = PL[7] - y * PL[11];
    B[2] = PR[3] - disp(x,y,0) * PR[11];
    B[3] = PR[7] - disp(x,y,1) * PR[11];

    transposeMatrix(A, AT, 4, 3);
    matrixMultiplication(AT, A, ATA, 3, 4, 3);
    calculateInverseMatrix_size3(ATA, ATA_invert);
    matrixMultiplication(ATA_invert, AT, ATA_invertAT, 3, 3, 4);
    matrixMultiplication(ATA_invertAT, B, res_temp, 3, 4, 1);

    Pointcloud(x,y,0)=res_temp[0];
    Pointcloud(x,y,1)=res_temp[1];
    Pointcloud(x,y,2)=res_temp[2];
/*
    if(y==1040)
    {
        double res_temp_[4]={res_temp[0],res_temp[1],res_temp[2],1.0};
        double PL_uv[3],PR_uv[3];
        matrixMultiplication(PL, res_temp_, PL_uv, 3, 4, 1);
        matrixMultiplication(PR, res_temp_, PR_uv, 3, 4, 1);
        // printf("[%d,%d][%f,%f]:[%f,%f],[%f,%f]\n",x,y,disp(x,y,0),disp(x,y,1),PL_uv[0]/PL_uv[2],PL_uv[1]/PL_uv[2],PR_uv[0]/PR_uv[2],PR_uv[1]/PR_uv[2]);
        // printf("[%d,%d][%f,%f][%f,%f]\n",x,y,static_cast<double>(x)-PL_uv[0]/PL_uv[2],static_cast<double>(y)-PL_uv[1]/PL_uv[2],disp(x,y,0)-PR_uv[0]/PR_uv[2],disp(x,y,1)-PR_uv[1]/PR_uv[2]);
    }


    double res_temp_[4]={res_temp[0],res_temp[1],res_temp[2],1.0};
    double PL_uv[3],PR_uv[3];
    matrixMultiplication(PL, res_temp_, PL_uv, 3, 4, 1);
    matrixMultiplication(PR, res_temp_, PR_uv, 3, 4, 1);
    // err_Lx(x,y)=static_cast<double>(x)-PL_uv[0]/PL_uv[2];
    // err_Ly(x,y)=static_cast<double>(y)-PL_uv[1]/PL_uv[2];

    err_Lx(x,y)=disp(x,y,0)-PR_uv[0]/PR_uv[2];
    err_Ly(x,y)=disp(x,y,1)-PR_uv[1]/PR_uv[2];
    if(abs(err_Lx(x,y))>10)
    {
        err_Lx(x,y)=10;
    }
    if(abs(err_Ly(x,y))>10)
    {
        err_Ly(x,y)=10;
    }
    
*/


    // matrixMultiplication(R,res_temp,res,3,3,1);
    // res_x[index] = res[0]+T[0];
    // res_y[index] = res[1]+T[1];
    // res_z[index] = res[2]+T[2];
}

void Matcher::reprojectTo3D_2D_cuda(CudaImage<double>& disp,CudaImage<double>& Pointcloud,CudaImage<double>& PL,CudaImage<double>& PR,dim3 blocksize,dim3 gridsize)
{
    CudaImage<double> err_Lx(disp.x_size,disp.y_size);
    CudaImage<double> err_Ly(disp.x_size,disp.y_size);
    __reprojectTo3D_2D_cuda<<<gridsize,blocksize>>>(disp, Pointcloud,err_Lx,err_Ly,PL.data,PR.data);
    checkCudaErrors(cudaDeviceSynchronize());
    
    // 检查是否有错误
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        printf("CUDA Error: %s\n", cudaGetErrorString(err));
    }
}


// 计算向量叉积（仅适用于3D向量）
__inline__ __device__ void crossProduct(const double* u, const double* v, double* result) {
    result[0] = u[1] * v[2] - u[2] * v[1];
    result[1] = u[2] * v[0] - u[0] * v[2];
    result[2] = u[0] * v[1] - u[1] * v[0];
}

// 计算向量模长
__inline__ __device__ double vectorNorm(const double* u, int size) {
    double result = 0.0;
    for (int i = 0; i < size; i++) {
        result += u[i] * u[i];
    }
    return sqrt(result);
}

// 计算两个向量的夹角正弦值
__inline__ __device__ double computeSineAngle(const double* u, const double* v, int size) {
    if (size != 3) {
        return;
    }

    // 计算叉积
    double cross[3];
    crossProduct(u, v, cross);

    // 计算叉积的模长
    double crossNorm = vectorNorm(cross, 3);

    // 计算向量的模长
    double norm_u = vectorNorm(u, size);
    double norm_v = vectorNorm(v, size);

    // 计算正弦值
    return crossNorm / (norm_u * norm_v);
}

//向量归一化
__device__ void normalizeVector(double* vec) {
    double x = vec[0], y = vec[1], z = vec[2];
    double length = rsqrt(x * x + y * y + z * z); // 使用快速倒数平方根
    vec[0] = x * length;
    vec[1] = y * length;
    vec[2] = z * length;
}

//计算dw点
// __inline__ __device__ void get_dw(const double* axis,const double d, const double n,const double* L,double* dw_p)
// {
//     double sin0 = computeSineAngle(axis, L, 3);
//     double dw=-d*sin0*(1-sqrt(1-sin0*sin0)/sqrt(n*n-sin0*sin0))/2;//这个公式存疑

//     double axisXL[3];
//     crossProduct(axis, L, axisXL);
//     crossProduct(axisXL, L, dw_p);

//     normalizeVector(dw_p);
//     dw_p[0]=dw_p[0]*dw;
//     dw_p[1]=dw_p[1]*dw;
//     dw_p[2]=dw_p[2]*dw;
// }
//计算dw点
__inline__ __device__ void get_dw(const double* axis,const double d, const double n,const double* L,double* dw_p)
{
    double sin0 = computeSineAngle(axis, L, 3);
    double dw=d*(1-sqrt(1-sin0*sin0)/sqrt(n*n-sin0*sin0));//这个公式存疑

    dw_p[0]=axis[0]*dw;
    dw_p[1]=axis[1]*dw;
    dw_p[2]=axis[2]*dw;
    // dw_p[0]=0;
    // dw_p[1]=0;
    // dw_p[2]=dw;
}


/// @brief 横纵条纹匹配三维重建核函数
/// @param disp 左图像每个像素对应右图像中的匹配点（亚像素）
/// @param Pointcloud 根据每个像素计算得到点的三维坐标
/// @param camMat_L 左相机内参矩阵
/// @param camMat_R 右相机内参矩阵
/// @param R 右相机坐标系相对于左相机坐标系下的旋转
/// @param T 右相机坐标系相对于左相机坐标系下的平移
/// @param PL 左相机投影矩阵
/// @param PR 右相机投影矩阵
/// @return 
__global__ void __reprojectTo3D_refraction_2D_cuda(CudaImage<double> disp,CudaImage<double> Pointcloud,
    double* camMat_L,double* camMat_R,double* R,double* T,
    const double* PL, const double* PR,
    const double* axis_L,const double* axis_R,const double n,const double d,CudaImage<double> dis)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x==1&&y==1)
    {
        // printf("//%f,%f,%f,%f,%f,%f//\n",axis_L[0],axis_L[1],axis_L[2],axis_R[0],axis_R[1],axis_R[2]);
    }
    // double axis_L_[3];
    // axis_L_[0]=axis_L[0];
    // axis_L_[1]=axis_L[1]-0.2;
    // axis_L_[2]=axis_L[2];
    // normalizeVector(axis_L_);


    if(disp(x,y,0)==-100)//未匹配到的直接跳过
    {
        return;
    }


    // if(500<x&&x<1000&&y==900)
    // if(x==980&&y==900)
    // if(y==1000)
    {
        // /*
        // 计算 K_T 的逆矩阵
        double KL_inv[9],KR_inv[9];//后续改为外部输入
        calculateInverseMatrix_size3(camMat_L, KL_inv);//求逆
        calculateInverseMatrix_size3(camMat_R, KR_inv);//求逆

        // 提取当前点的齐次坐标
        double point_L[3] = {static_cast<double>(x),static_cast<double>(y),1.0};
        double point_R[3] = {disp(x,y,0),disp(x,y,1),1.0};

        // 计算相机坐标系中的点
        double cameraPoint_L[3];
        double cameraPoint_R[3];
        matrixMultiplication(KL_inv, point_L, cameraPoint_L, 3, 3, 1);//矩阵乘法
        matrixMultiplication(KR_inv, point_R, cameraPoint_R, 3, 3, 1);//矩阵乘法
        normalizeVector(cameraPoint_L);
        normalizeVector(cameraPoint_R);

        double R_[9];
        double cameraPoint_R_[3];
        transposeMatrix(R,R_,3,3);
        matrixMultiplication(R_, cameraPoint_R, cameraPoint_R_, 3, 3, 1);//矩阵乘法


        // double point_[3] = {Pointcloud(x,y,0),Pointcloud(x,y,1),Pointcloud(x,y,2)};
        // normalizeVector(point_);

        // printf("[%f,%f,%f][%f,%f,%f]\n",point_[0],point_[1],point_[2],cameraPoint[0],cameraPoint[1],cameraPoint[2]);
        // return;
        // */

        // /*
        // 折射前后的光线方向（单位向量）
        // double L1[3]={Pointcloud(x,y,0),Pointcloud(x,y,1),Pointcloud(x,y,2)};
        // normalizeVector(L1);
        // double L2[3]={Pointcloud(x,y,0)-T[0],Pointcloud(x,y,1)-T[1],Pointcloud(x,y,2)-T[2]};
        // normalizeVector(L2);

        double L1[3]={cameraPoint_L[0],cameraPoint_L[1],cameraPoint_L[2]};
        normalizeVector(L1);
        double L2[3]={cameraPoint_R_[0],cameraPoint_R_[1],cameraPoint_R_[2]};
        normalizeVector(L2);


        //折射后直线经过的点
        double dw1[3],dw2[3];
        // dw1[0]=0;
        // dw1[1]=0;
        // dw1[2]=0;
        // dw2[0]=0;
        // dw2[1]=0;
        // dw2[2]=0;
        get_dw(axis_L,d,n,L1,dw1);
        get_dw(axis_L,d,n,L2,dw2);

        dw2[0]=dw2[0]+T[0];
        dw2[1]=dw2[1]+T[1];
        dw2[2]=dw2[2]+T[2];

        // printf("//[%f,%f]//", computeSineAngle(L1,dw1,3), computeSineAngle(L2,dw2,3));

        
        // 提取直线1的点和方向向量
        double x1 = dw1[0], y1 = dw1[1], z1 = dw1[2];
        double a1 = L1[0], b1 = L1[1], c1 = L1[2];

        // 提取直线2的点和方向向量
        double x2 = dw2[0], y2 = dw2[1], z2 = dw2[2];
        double a2 = L2[0], b2 = L2[1], c2 = L2[2];

        //最小二乘法求解
        double ATA_00 = a1*a1 + b1*b1 + c1*c1;
        double ATA_01 = -a1*a2 - b1*b2 - c1*c2;
        double ATA_11 = a2*a2 + b2*b2 + c2*c2;
        double ATB_0 = a1*(x2-x1) + b1*(y2-y1) + c1*(z2-z1);
        double ATB_1 = -a2*(x2-x1) - b2*(y2-y1) - c2*(z2-z1);
        double det_ATA = ATA_00 * ATA_11 - ATA_01 * ATA_01;

        if (fabs(det_ATA) < 1e-10) { 
            Pointcloud(x,y,0) = NAN;
            Pointcloud(x,y,1) = NAN;
            Pointcloud(x,y,2) = NAN;
            return;
        }

        double t, s;
        t = (ATA_11 * ATB_0 - ATA_01 * ATB_1) / det_ATA;
        s = (ATA_00 * ATB_1 - ATA_01 * ATB_0) / det_ATA;

        // 计算残差
        double P1[3] = {x1 + a1*t, y1 + b1*t, z1 + c1*t};
        double P2[3] = {x2 + a2*s, y2 + b2*s, z2 + c2*s};
        double dx = P1[0] - P2[0], dy = P1[1] - P2[1], dz = P1[2] - P2[2];
        double residual = sqrt(dx*dx + dy*dy + dz*dz);

        Pointcloud(x,y,0) = 0.5 * (P1[0] + P2[0]);
        Pointcloud(x,y,1) = 0.5 * (P1[1] + P2[1]);
        Pointcloud(x,y,2) = 0.5 * (P1[2] + P2[2]);

        // 计算两点之间的距离
        double distance = sqrt(dx * dx + dy * dy + dz * dz);

        dis(x,y)=distance;

        if(y==1024)
        {
            Pointcloud(x,y,0) = NAN;
            Pointcloud(x,y,1) = NAN;
            Pointcloud(x,y,2) = NAN;
            // printf("B:%f",distance);
            // printf("%d,%d::%f,%f,%f//%f,%f,%f\n",x,y,L1[0],L1[1],L1[2],L2[0],L2[1],L2[2]);
            // printf("dw1:%f",dw1[2]);computeSineAngle(axis, L, 3)
            // printf("%d,%d//%f,%f//\n",x,y,asin(computeSineAngle(axis_L, L1, 3))*180/M_PI,asin(computeSineAngle(axis_L, L2, 3))*180/M_PI);
        }

        // */
    }
}

void Matcher::reprojectTo3D_refraction_2D_cuda(CudaImage<double>& disp,CudaImage<double>& Pointcloud,
    CudaImage<double>& camMat_L,CudaImage<double>& camMat_R,
    CudaImage<double>& R,CudaImage<double>& T,
    CudaImage<double>& PL,CudaImage<double>& PR,
    const double *norm_L,const double *norm_R,const double n,const double d,
    dim3 blocksize,dim3 gridsize,CudaImage<double>& err_Lx,CudaImage<double>& err_Ly)
{
    dim3 blocksize_,gridsize_;
    blocksize_.x=blocksize.x/2;
    blocksize_.y=blocksize.y/2;
    gridsize_.x=gridsize.x*2;
    gridsize_.y=gridsize.y*2;

    __reprojectTo3D_2D_cuda<<<gridsize_,blocksize_>>>(disp, Pointcloud,err_Lx,err_Ly,PL.data,PR.data);
    // __reprojectTo3D_refraction_2D_cuda<<<gridsize_,blocksize_>>>(disp, Pointcloud,camMat_L.data,camMat_R.data,R.data,T.data,PL.data,PR.data,norm_L,norm_R,n,d,err_Ly);
    checkCudaErrors(cudaDeviceSynchronize());
    
    // 检查是否有错误
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        printf("CUDA Error: %s\n", cudaGetErrorString(err));
    }
}