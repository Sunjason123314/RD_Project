#include "cudaimage.h"

#include <iostream>
using namespace std;

#define TYPE_TO_VALUE(type) \
    ( \
        (typeid(type) == typeid(uchar)) ? 0 : \
        (typeid(type) == typeid(float)) ? 5 : \
        (typeid(type) == typeid(double)) ? 6 : \
        -1  /* Default value if type doesn't match any of the expected types */ \
    )

int CudaImage<uchar>::typenumber = TYPE_TO_VALUE(uchar);
int CudaImage<float>::typenumber = TYPE_TO_VALUE(float);
int CudaImage<double>::typenumber = TYPE_TO_VALUE(double);


template <typename T>
cudaStream_t CudaImage<T>::stream_ = nullptr;
template <typename T>
cudaStream_t CudaImage<T>::stream_core = nullptr;

template <typename T>
CudaImage<T>::CudaImage():
x_size(0),
y_size(0),
z_size(0),
xy_size(0),
xy_bytes(0),
bytes(0)
{
    
}

template <typename T>
CudaImage<T>::CudaImage(int x,int y,int z,T value):
x_size(x),
y_size(y),
z_size(z),
xy_size(x*y),
xy_bytes(sizeof(T)*x*y),
bytes(sizeof(T)*x*y*z)
{
    cudaMalloc((void**)&data, bytes);
    // cudaMallocAsync((void**)&data, bytes,stream_);
    // cudaMallocHost((void**)&dataH, bytes);
    if (value!=0)
    {
        setConstant(value);
    }
    
}

template <typename T>
CudaImage<T>::CudaImage(const cv::Mat& cvimage):
x_size(cvimage.cols),
y_size(cvimage.rows),
z_size(1),
xy_size(cvimage.cols*cvimage.rows),
xy_bytes(sizeof(T)*cvimage.cols*cvimage.rows),
bytes(sizeof(T)*cvimage.cols*cvimage.rows*1)
{
    #ifdef _DEBUG
    checkCudaErrors(cudaMalloc((void**)&data, bytes));
    #else
    cudaMalloc((void**)&data, bytes);
    // cudaMallocAsync((void**)&data, bytes,stream_);
    // cudaMallocHost((void**)&dataH,bytes);
    #endif
    CopyCVMat(cvimage);
}

template <typename T>
CudaImage<T>::CudaImage(const std::vector<cv::Mat>& cvimagevector):
x_size(cvimagevector[0].cols),
y_size(cvimagevector[0].rows),
z_size(cvimagevector.size()),
xy_size(cvimagevector[0].cols*cvimagevector[0].rows),
xy_bytes(sizeof(T)*cvimagevector[0].cols*cvimagevector[0].rows),
bytes(sizeof(T)*cvimagevector[0].cols*cvimagevector[0].rows*cvimagevector.size())
{
    #ifdef _DEBUG
    checkCudaErrors(cudaMalloc((void**)&data, bytes));
    #else
    cudaMalloc((void**)&data, bytes);
    // cudaMallocAsync((void**)&data, bytes,stream_);
    // cudaMallocHost((void**)&dataH,bytes);
    #endif
    CopyCVMat(cvimagevector);
}

template <typename T>
CudaImage<T>::~CudaImage()
{
    
}

template <typename T>
void CudaImage<T>::ini(int x,int y,int z=1,T value=0)
{
    x_size=x;
    y_size=y;
    z_size=z;
    xy_size=x*y;
    xy_bytes=sizeof(T)*x*y;
    bytes=sizeof(T)*x*y*z;

    if(data!=NULL)
    {
        ImageFree();
    }
    cudaMalloc((void**)&data, bytes);
    // cudaMallocAsync((void**)&data, bytes,stream_);
    // cudaMallocHost((void**)&dataH, bytes);
    if (value!=0)
    {
        setConstant(value);
    }
}

template <typename T>
__global__ void AddKernel(const T* src1, const T* src2, T* dst, int size)
{
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid < size)
    {
        dst[tid] = src1[tid] + src2[tid];
    }
}

template <typename T>
CudaImage<T> CudaImage<T>::operator+(const CudaImage<T>& other) const
{
    CudaImage<T> result(x_size, y_size, z_size);
    int numElements = x_size * y_size * z_size;
    int threadsPerBlock = 256;
    int blocksPerGrid = (numElements + threadsPerBlock - 1) / threadsPerBlock;

    AddKernel<<<blocksPerGrid, threadsPerBlock>>>(data, other.data, result.data, numElements);
    cudaDeviceSynchronize();

    return result;
}

template <typename T>
void CudaImage<T>::minmax(T& min,T&max)
{
    //还没实现cuda加速最值搜索
}

template <typename T>
__global__ void __setConstant(T value,T* data,int x_size,int xy_size,int size)
{
    //计算索引
	int x=blockIdx.x*blockDim.x+threadIdx.x;
    int y=blockIdx.y*blockDim.y+threadIdx.y;
	int z=blockIdx.z;
    int id=x+y*x_size+z*xy_size;

    if (id<size)
    {
        data[id]=value;
    }
}

/// @brief 初始化图像的值
/// @tparam T 
/// @param value 
template <typename T>
inline void CudaImage<T>::setConstant(T value)
{
    dim3 BlockSize(32,32);
	dim3 GridSize(x_size/32+1,y_size/32+1,z_size);
    __setConstant<<<GridSize,BlockSize>>>(value,data,x_size,xy_size,x_size*y_size*z_size);
    checkCudaErrors(cudaDeviceSynchronize());
}

/// @brief 初始化图像的值(异步版本，需要手动同步)
/// @tparam T 
/// @param value 
template <typename T>
inline void CudaImage<T>::setConstantAsync(T value)
{
    dim3 BlockSize(32,32);
	dim3 GridSize(x_size/32+1,y_size/32+1,z_size);
    __setConstant<<<GridSize,BlockSize,0,stream_core>>>(value,data,x_size,xy_size,x_size*y_size*z_size);
    // __setConstant<<<GridSize,BlockSize>>>(value,data,x_size,xy_size,x_size*y_size*z_size);
}

template <typename T>
__global__ void __division(T* data,double n,int x_size,int xy_size,int size)
{
    //计算索引
	int x=blockIdx.x*blockDim.x+threadIdx.x;
    int y=blockIdx.y*blockDim.y+threadIdx.y;
	int z=blockIdx.z;
    // int z=0;
    int id=x+y*x_size+z*xy_size;

    if (id<size)
    {
        data[id]=data[id]/n;
    }
}

template <typename T>
void CudaImage<T>::division(const double &n)
{
    dim3 BlockSize(32,32);
	dim3 GridSize(x_size/32+1,y_size/32+1,z_size);
    int x_size_=GridSize.x*BlockSize.x;
    int y_size_=GridSize.y*BlockSize.y;
    int xy_size_=x_size_*y_size_;
    __division<<<GridSize,BlockSize>>>(data,n,x_size_,xy_size_,xy_size*z_size);
    checkCudaErrors(cudaDeviceSynchronize());
}

//**滤波start**/

template <typename T>
__global__ void __convolution()
{
    //计算索引
	// int x=blockIdx.x*blockDim.x+threadIdx.x;
    // int y=blockIdx.y*blockDim.y+threadIdx.y;
	// int z=blockIdx.z;
    // int id=x+y*x_size+z*xy_size;
}

// CUDA核函数：高斯滤波
template <typename T>
__global__ void __convolution_inplace_kernel(T* image, // 既是输入也是输出
                                          const double* __restrict__ kernel,
                                          int x_size, int y_size, int xy_size,
                                          int kernel_radius) {
    // 1. 计算当前线程对应的输出像素位置 (x, y)
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int z = blockIdx.z;
    int id = x+y*x_size+z*xy_size;

    // 2. 检查是否在有效图像范围内
    if (x >= x_size || y >= y_size) return;

    // 3. 从全局内存（原图像）中读取当前线程计算所需的输入区域
    //    注意：读取的是原图像中(x, y)位置周围kernel_radius区域的值
    float sum = 0.0f;
    for (int ky = -kernel_radius; ky <= kernel_radius; ky++) {
        for (int kx = -kernel_radius; kx <= kernel_radius; kx++) {
            int ix = x + kx;
            int iy = y + ky;
            // 处理边界（例如使用填充）
            if (ix >= 0 && ix < x_size && iy >= 0 && iy < y_size) {
                float pixel = image[z * xy_size +iy * x_size + ix]; // 从原图像读取
                float coeff = kernel[(ky + kernel_radius) * (2*kernel_radius+1) + (kx + kernel_radius)];
                sum += pixel * coeff;
            } else {
                // 边界处理，例如填充0
                // sum += 0.0f;
            }
        }
    }

    // 等待块内所有线程都计算完成
    __syncthreads();

    // 4. 重要：确保所有用于计算的输入数据都已读取后，再将结果写回原图像内存
    //    由于每个线程只写自己对应的(x,y)位置，且位置唯一，不会发生写冲突
    image[id] = sum; // 将计算结果写回原图像内存
}

// CUDA核函数：高斯滤波（线程安全版本）
template <typename T>
__global__ void __convolution_inplace_safe_kernel(T* image,
                                               const double* __restrict__ kernel,
                                               int x_size, int y_size, int xy_size,
                                               int kernel_radius) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int z = blockIdx.z;
    int id = x + y * x_size + z * xy_size;
    
    if (x >= x_size || y >= y_size) return;
    
    // 使用共享内存存储中间结果
    extern __shared__ float s_output[];
    
    float sum = 0.0f;
    for (int ky = -kernel_radius; ky <= kernel_radius; ky++) {
        for (int kx = -kernel_radius; kx <= kernel_radius; kx++) {
            int ix = x + kx;
            int iy = y + ky;
            if (ix >= 0 && ix < x_size && iy >= 0 && iy < y_size) {
                float pixel = image[z * xy_size + iy * x_size + ix];
                float coeff = kernel[(ky + kernel_radius) * (2*kernel_radius+1) + (kx + kernel_radius)];
                sum += pixel * coeff;
            }
        }
    }
    
    // 将结果先存入共享内存
    int local_idx = threadIdx.y * blockDim.x + threadIdx.x;
    s_output[local_idx] = sum;
    
    // 等待块内所有线程都计算完成
    __syncthreads();
    
    // 现在可以安全写回
    image[id] = s_output[local_idx];
}

template <typename T>
void CudaImage<T>::convolution(double kernel[],int size)
{
    dim3 BlockSize(32,32);
	dim3 GridSize(x_size/32+1,y_size/32+1,z_size);

    __convolution_inplace_safe_kernel<<<GridSize,BlockSize,32*32*sizeof(float)>>>(data,kernel,x_size,y_size,xy_size,(int)(size/2));
    // __convolution_inplace_kernel<<<GridSize,BlockSize>>>(data,kernel,x_size,y_size,xy_size,(int)(size/2));
    checkCudaErrors(cudaDeviceSynchronize());
}

// CUDA核函数：相位均值滤波-核函数
template <typename T>
__global__ void __phaseFilter_kernel(const T* input, T* output, 
                                        int x_size, int y_size, int xy_size, 
                                        int kernel_radius) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int z = blockIdx.z;
    int id = x + y * x_size + z * xy_size;

    if (x >= x_size || y >= y_size) return;
    if (input[id]<=-99) return;//排除调制度滤除点

    float sum_sin = 0.0f;
    float sum_cos = 0.0f;
    int count = 0;

    // 遍历滤波窗口
    for (int ky = -kernel_radius; ky <= kernel_radius; ky++) {
        for (int kx = -kernel_radius; kx <= kernel_radius; kx++) {
            int ix = x + kx;
            int iy = y + ky;

            //边界检查
            ix = max(0, min(x_size - 1, ix));
            iy = max(0, min(y_size - 1, iy));

            int neighbor_id = z * xy_size + iy * x_size + ix;
            float phase_val = (float)input[neighbor_id];

            //在复数域做均值滤波
            if(phase_val>=-M_PI)//排除超出相位的点
            {
                sum_sin += sinf(phase_val);
                sum_cos += cosf(phase_val);
            }
            count++;
        }
    }

    //重构相位
    if (count > 0) {
        output[id] = (T)atan2f(sum_sin, sum_cos);
    } else {
        output[id] = input[id];
    }
}

/**
 * @brief 
 * @param kernel_size 
 */
template <typename T>
void CudaImage<T>::phaseFilter(int kernel_size) {
    if (kernel_size % 2 == 0) kernel_size++; // 确保为奇数
    int kernel_radius = kernel_size / 2;

    // 创建临时输出图像，避免原图在计算过程中被污染
    CudaImage<T> output(x_size, y_size, z_size, -100);

    dim3 BlockSize(32, 32);
    dim3 GridSize((x_size + BlockSize.x - 1) / BlockSize.x,
                  (y_size + BlockSize.y - 1) / BlockSize.y,
                  z_size);

    __phaseFilter_kernel<T><<<GridSize, BlockSize>>>(
        data, output.data, x_size, y_size, xy_size, kernel_radius);

    checkCudaErrors(cudaDeviceSynchronize());

    // 将平滑后的数据拷贝回原图像
    checkCudaErrors(cudaMemcpy(data, output.data, bytes, cudaMemcpyDeviceToDevice));
}

// 别忘了在 .cu 文件末尾添加显式实例化，否则编译器无法找到模板实现
template void CudaImage<float>::phaseFilter(int kernel_size);
template void CudaImage<double>::phaseFilter(int kernel_size);

//*****滤波end****//

template <typename T>
__global__ void __centralize(T* data,int x_size,int xy_size,int size)
{
    //计算索引
	int x=blockIdx.x*blockDim.x+threadIdx.x;
    int y=blockIdx.y*blockDim.y+threadIdx.y;
	int z=blockIdx.z;
    int id=x+y*x_size+z*xy_size;

    if (id<size)
    {
        data[id]=data[id]*pow(-1,x+y);
    }
}

template <typename T>
void CudaImage<T>::centralize()
{
    dim3 BlockSize(32,32);
	dim3 GridSize(x_size/32+1,y_size/32+1,z_size);
    int x_size_=GridSize.x*BlockSize.x;
    int y_size_=GridSize.y*BlockSize.y;
    int xy_size_=x_size_*y_size_;
    __centralize<<<GridSize,BlockSize>>>(data,x_size_,xy_size_,xy_size*z_size);
    checkCudaErrors(cudaDeviceSynchronize());
}

template <typename T>
__global__ void __uncentralize(T* data,int x_size,int xy_size,int size)
{
    //计算索引
	int x=blockIdx.x*blockDim.x+threadIdx.x;
    int y=blockIdx.y*blockDim.y+threadIdx.y;
	int z=blockIdx.z;
    int id=x+y*x_size+z*xy_size;

    if (id<size)
    {
        data[id]=data[id]*pow(-1,x+y+1);
    }
}

template <typename T>
void CudaImage<T>::uncentralize()
{
    dim3 BlockSize(32,32);
	dim3 GridSize(x_size/32+1,y_size/32+1,z_size);
    int x_size_=GridSize.x*BlockSize.x;
    int y_size_=GridSize.y*BlockSize.y;
    int xy_size_=x_size_*y_size_;
    __centralize<<<GridSize,BlockSize>>>(data,x_size_,xy_size_,xy_size*z_size);
    checkCudaErrors(cudaDeviceSynchronize());
}

template <typename T>
__global__ void __toComplex(T* data,cufftDoubleComplex* output,int x_size)
{
    //计算索引
	int x=blockIdx.x*blockDim.x+threadIdx.x;
    int y=blockIdx.y*blockDim.y+threadIdx.y;
	// int z=blockIdx.z;//多张
    int id=x+y*x_size;

    output[id].x=data[id];
    output[id].y=0;
}

/// @brief 转换为复数分布用于傅里叶变换（实数填充实部，虚部设置为0）(外部分配内存)（暂时只支持单张图片处理）
/// @tparam T 
template <typename T>
void CudaImage<T>::toComplex(cufftDoubleComplex* output,const dim3 &GridSize,const dim3 &BlockSize)
{
    __toComplex<<<GridSize,BlockSize>>>(data,output,x_size);
    checkCudaErrors(cudaDeviceSynchronize());
}

template <typename T>
__global__ void __fromComplex(T* data,cufftDoubleComplex* input,int x_size)
{
    //计算索引
	int x=blockIdx.x*blockDim.x+threadIdx.x;
    int y=blockIdx.y*blockDim.y+threadIdx.y;
	// int z=blockIdx.z;//多张
    int id=x+y*x_size;

    data[id]=input[id].x;//实部
}

/// @brief 转换为复数分布用于傅里叶变换（实数填充实部，虚部设置为0）(外部分配内存)（暂时只支持单张图片处理）
/// @tparam T 
template <typename T>
void CudaImage<T>::fromComplex(cufftDoubleComplex* input,const dim3 &GridSize,const dim3 &BlockSize)
{
    __fromComplex<<<GridSize,BlockSize>>>(data,input,x_size);
    checkCudaErrors(cudaDeviceSynchronize());
}

/// @brief 拷贝另一个实例
/// @tparam T 
/// @param other 
template <typename T>
void CudaImage<T>::copyImage(const CudaImage<T>& other)
{
    bytes=other.bytes;
    #ifdef _DEBUG
    checkCudaErrors(cudaMalloc((void**)&data,bytes));
    checkCudaErrors(cudaMemcpy(data,other.data,bytes,cudaMemcpyDeviceToDevice));
    #else
    cudaMalloc((void**)&data,bytes);
    cudaMemcpy(data,other.data,bytes,cudaMemcpyDeviceToDevice);
    #endif
    x_size=other.x_size,
    y_size=other.y_size;
    z_size=other.z_size;
    xy_size=other.xy_size;
    xy_bytes=other.xy_bytes;
}
template <typename T>
inline void CudaImage<T>::DownLoadHostAsync(T* data_host)
{
    cudaMemcpyAsync(data_host, data, bytes, cudaMemcpyDeviceToHost,stream_);
}

/// @brief 手动释放显存
/// @tparam T 
template <typename T>
void CudaImage<T>::ImageFree()
{
    #ifdef _DEBUG
    checkCudaErrors(cudaFree(data));
    #else
    cudaFree(data);
    // cudaFreeAsync(data,stream_);
    // cudaFreeHost(dataH);
    #endif
}

template <typename T>
inline void CudaImage<T>::CopyCVMat(const cv::Mat& cvimage)
{
    #ifdef _DEBUG
    if(z_size>1)//检查z方向大小，如果不为1则不可以使用这个函数
    {
        throw std::runtime_error("z size more than 1,please use [std::vector<cv::Mat>] to copy data");
    }
    if ((x_size!=cvimage.cols)||(y_size!=cvimage.rows))//检查传入图像大小是否符合
    {
        throw std::runtime_error("input image size incorrect");
    }
    //以后还要检查图像类型，通道数等！！！！！
    // if(cvimage.type()==float)
    // {
    //     cout<<"float"<<endl;
    // }
    // else if(cvimage.type()==uchar)
    // {
    //     cout<<"ucahr"<<endl;
    // }
    // else if(cvimage.type()==double)
    // {
    //     cout<<"double"<<endl;
    // }
    // else
    // {
    //     cout<<"unknown"<<endl;
    // }
    checkCudaErrors(cudaMemcpy(data,cvimage.data,bytes,cudaMemcpyHostToDevice));
    #else
    cudaMemcpy(data,cvimage.data,bytes,cudaMemcpyHostToDevice);
    #endif
    
}

template <typename T>
inline void CudaImage<T>::CopyCVMat(const std::vector<cv::Mat>& cvimagevector)
{
    #ifdef _DEBUG
    if(cvimagevector.size()!=z_size)
    {
        throw std::runtime_error("z size is incorrect,please use [cv::Mat] to copy data or [std::vector<cv::Mat>] with correct size");
    }
    if ((x_size!=cvimagevector[0].cols)||(y_size!=cvimagevector[0].rows))//检查传入图像大小(第一张)是否符合
    {
        throw std::runtime_error("input image size incorrect");
    }
    //以后还要检查图像类型，通道数等！！！！！
    #endif
    for (size_t i = 0; i < z_size; i++)
    {
        #ifdef _DEBUG
        checkCudaErrors(cudaMemcpy(data+i*xy_size,cvimagevector[i].ptr<T>(),xy_bytes,cudaMemcpyHostToDevice));
        #else
        // cudaMemcpyAsync(data+i*xy_size,cvimagevector[i].ptr<T>(),xy_bytes,cudaMemcpyHostToDevice,stream_);//异步拷贝
        cudaMemcpy(data+i*xy_size,cvimagevector[i].ptr<T>(),xy_bytes,cudaMemcpyHostToDevice);//同步拷贝
        #endif
    }
    // cudaDeviceSynchronize();
}

template <typename T>
void CudaImage<T>::OutPutCVMat(cv::Mat& cvimage)
{
    cvimage=cv::Mat(y_size,x_size,cv::DataType<T>::type);
    #ifdef _DEBUG
    checkCudaErrors(cudaMemcpy(cvimage.data, data, bytes, cudaMemcpyDeviceToHost));
    #else
    cudaMemcpy(cvimage.data, data, bytes, cudaMemcpyDeviceToHost);
    #endif
}

template <typename T>
void CudaImage<T>::OutPutCVMat(std::vector<cv::Mat>& cvimagevector)
{
    cvimagevector.clear();
    cvimagevector.resize(z_size);
    // cudaMemcpyAsync(dataH, data, bytes, cudaMemcpyDeviceToHost,stream);
    for (size_t i = 0; i < z_size; i++)
    {
        cvimagevector[i].create(y_size,x_size,cv::DataType<T>::type);
        #ifdef _DEBUG
        checkCudaErrors(cudaMemcpy(cvimagevector[i].data, data+i*xy_size, xy_bytes, cudaMemcpyDeviceToHost));
        #else
        cudaMemcpy(cvimagevector[i].data, data+i*xy_size, xy_bytes, cudaMemcpyDeviceToHost);
        // cudaMemcpyAsync(cvimagevector[i].data, dataH+i*xy_size, xy_bytes, cudaMemcpyHostToHost,stream);
        #endif
    }
    // cudaDeviceSynchronize();
}

/// @brief 输出点云数据（z方向三层分别存储x、y、z信息,深度点云专用）
/// @tparam T 数据类型
/// @param cloud 输出点云
/// @param data_host 用于拷贝的固定内存
template <typename T>
void CudaImage<T>::OutPutPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud,T* data_host)
{
    cudaMemcpy(data_host, data, bytes, cudaMemcpyDeviceToHost);
    for (int i = 0; i < xy_size; i++)
    {
        cloud.push_back(pcl::PointXYZ(data_host[i],data_host[i+xy_size],data_host[i+2*xy_size]));
    }
}

/// @brief 输出为点云数据（点云xy分别对应图像像素位置，z坐标代表图像像素大小，显示点云专用）
/// @tparam T 数据类型
/// @param cloud 输出点云
template <typename T>
void CudaImage<T>::OutPutPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    T* data_host;
    data_host=(T*)malloc(bytes);
    cudaMemcpy(data_host, data, bytes, cudaMemcpyDeviceToHost);
    for (int i = 0; i < x_size; i++)
    {
        for (int j = 0; j < y_size; j++)
        {
            // double value=data_host[i*y_size+j];
            double value=data_host[j*x_size+i];//(行列错误)
            // if (value!=-100)
            // {
                cloud.push_back(pcl::PointXYZ(i,j,value));
            // }
        }
    }
}

/// @brief 获得图像大小（总字节）
/// @tparam T 
/// @return 
template <typename T>
int CudaImage<T>::getbyte()
{
    return bytes;
}

/// @brief 获得图像大小（xy字节）
/// @tparam T 
/// @return 
template <typename T>
int CudaImage<T>::getxybyte()
{
    return xy_bytes;
}

// CUDA核函数：高斯滤波(后续实现一些图像预处理，边缘提取等)
__global__ void gaussianBlurKernel(const double* input, double* output, int width, int height,
                                   const double* kernel, int ksize)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x < width && y < height)
    {
        int radius = ksize / 2;
        double sum = 0.0;

        for (int i = -radius; i <= radius; i++)
        {
            for (int j = -radius; j <= radius; j++)
            {
                int px = x + i;
                int py = y + j;

                if (px >= 0 && px < width && py >= 0 && py < height)
                {
                    double weight = kernel[(i + radius) + (j + radius) * ksize];
                    sum += input[px + py * width] * weight;
                }
            }
        }

        output[x + y * width] = sum;
    }
}

// 调用CUDA核函数进行高斯滤波
void cudaGaussianBlur(const CudaImage<double>& inputImage, CudaImage<double>& outputImage,
                      const double* gaussianKernel, int kernelSize)
{
    dim3 threadsPerBlock(16, 16);
    dim3 numBlocks((inputImage.x_size + threadsPerBlock.x - 1) / threadsPerBlock.x,
                   (inputImage.y_size + threadsPerBlock.y - 1) / threadsPerBlock.y);

    gaussianBlurKernel<<<numBlocks, threadsPerBlock>>>(inputImage.data, outputImage.data,
                                                        inputImage.x_size, inputImage.y_size,
                                                        gaussianKernel, kernelSize);

    cudaDeviceSynchronize();
}