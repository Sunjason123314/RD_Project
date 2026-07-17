#include "patterncoder.h"

/// @brief cuda预热
/// @return 
__global__ void cuda_warm(uchar* warmdata_d)
{
	uchar test=warmdata_d[0]*2;
}

/// @brief 构造函数
/// @param rows_in 解码图像尺寸
/// @param cols_in 解码图像尺寸
PatternCoder::PatternCoder(int rows_in,int cols_in):
rows(rows_in),
cols(cols_in)
{
	//cudaimage初始化
	CudaImage<uchar>::InitializeStream();
	CudaImage<float>::InitializeStream();
	CudaImage<double>::InitializeStream();
	
	//预热
	int byte=10*10*sizeof(uchar);
	uchar* warmdata_d;
	cudaMalloc((void**)&warmdata_d,byte);
	uchar* warmdata_h;
	warmdata_h=(uchar*)malloc(byte);
	cudaMemcpy(warmdata_d,warmdata_h,byte,cudaMemcpyHostToDevice);
	cuda_warm<<<1,1>>>(warmdata_d);//预热
	cudaDeviceSynchronize();
	free(warmdata_h);
	cudaFree(warmdata_d);

	//提前规划线程分布
	__GetBlockSize(rows,cols,BlockSize,GridSize);
}

/// @brief 析构函数
PatternCoder::~PatternCoder()
{
	CudaImage<uchar>::DestroyStream();
	CudaImage<float>::DestroyStream();
	CudaImage<double>::DestroyStream();
}

/*************************************************格雷码*********************************************************/
/// @brief 获取相移法投影图案集-核函数
/// @param patterns 传入分配的图像集显存地址
/// @param T 一个周期对应的像素数
/// @param A 背景光强
/// @param B 调制度
/// @return 
__global__ void __GetShiftedPhaseSet_cuda(CudaImage<uchar> myimages,double T,int A,int B,double step)
{
	//计算索引
	int x=blockIdx.x*blockDim.x+threadIdx.x;
    int y=blockIdx.y*blockDim.y+threadIdx.y;

	//计算
	double phase = blockIdx.z*step-M_PI;
	
    // myimages(x,y,blockIdx.z)= round(B * cos(((x + 0.5) / T) * 2 * M_PI + phase) / 2 + A);//四舍五入-像素中心
	// myimages(x,y,blockIdx.z)= B * cos(((x + 0.5) / T) * 2 * M_PI + phase) / 2 + A;//直接截断-像素中心
	myimages(x,y,blockIdx.z)= round(B * cos(((double)(x) / T) * 2 * M_PI + phase) / 2 + A);//四舍五入
	// myimages(x,y,blockIdx.z)= B * cos(((double)(x) / T) * 2 * M_PI + phase) / 2 + A;//直接截断
}

/// @brief 获取相移法投影图案集
/// @param n 相移法步数
/// @param T 一个周期对应的像素数
/// @param A 背景光强
/// @param B 调制度
/// @param pattern_out 返回图案集
void PatternCoder::GetShiftedPhaseSet_cuda(int n, double T, int A, int B, CudaImage<uchar>& pattern_out,int rows_p,int cols_p)
{
	//计算线程布局
	dim3 BlockSize_p;
	dim3 GridSize_p;
	__GetBlockSize(rows_p,cols_p,BlockSize_p,GridSize_p);
	GridSize.z=n;

	//分配内存(之后挪到外边去)
	pattern_out=CudaImage<uchar>::CudaImage(cols_p,rows_p,n);

	//计算
	__GetShiftedPhaseSet_cuda<<<GridSize_p,BlockSize_p>>>(pattern_out,T,A,B,2*M_PI/n);
	checkCudaErrors(cudaDeviceSynchronize());
}

/// @brief N步相移法求解相位-核函数
/// @param phaseimage 条纹图案
/// @param phase 求解相位
/// @param B 调制度
/// @return 
__global__ void __DeShiftedPhaseSet_cuda(const CudaImage<uchar> phaseimage,CudaImage<double> phase,CudaImage<double> B)
{
	//计算索引
	int x=blockIdx.x*blockDim.x+threadIdx.x;
    int y=blockIdx.y*blockDim.y+threadIdx.y;
	int N=phaseimage.z_size;

	//计算相位
	float denominator = 0, molecule = 0;
	for (int i = 0; i < N; i++)
	{
		denominator+=phaseimage(x,y,i)*cos(2*M_PI*i/N);
		molecule+=phaseimage(x,y,i)*sin(2*M_PI*i/N);
	}
	phase(x,y)=atan2(-molecule, denominator) + M_PI;//抬高后用于生成图像
	B(x,y)=2/N*sqrt(pow(molecule,2)+pow(denominator,2));
}

/// @brief N步相移法求解相位
/// @param patterns 条纹图案
/// @param phase 求解相位
/// @param B 调制度
void PatternCoder::DeShiftedPhase_cuda(const CudaImage<uchar> &patterns, CudaImage<double>& phase,CudaImage<double>& B)
{
	//分配内存
	phase=CudaImage<double>::CudaImage(cols,rows);
	B=CudaImage<double>::CudaImage(cols,rows);
	
	//开始计算
	__DeShiftedPhaseSet_cuda<<<GridSize,BlockSize>>>(patterns,phase,B);
	checkCudaErrors(cudaDeviceSynchronize());
}

/// @brief 获取格雷码图像集-核函数
/// @param grayimages 输入图像
/// @param max 最大亮度
/// @return 
__global__ void __GetGrayCodePattern_cuda(CudaImage<uchar> grayimages,int max,int* code)
{
	//计算索引
	int x=blockIdx.x*blockDim.x+threadIdx.x;
    int y=blockIdx.y*blockDim.y+threadIdx.y;
	int z=blockIdx.z;//0,1,2,3,4(五)

	int patternWidth =  grayimages.x_size / pow(2,z+1);
	int patternIdx = x/patternWidth;

	grayimages(x,y,z)=code[patternIdx%4]*max;
}

/// @brief 获取投影格雷码条纹图像
/// @param n 格雷码阶数
/// @param max 最大亮度
/// @param pics 输出图案集
void PatternCoder::GetGrayCodePattern_cuda(int n, int max, CudaImage<uchar>& grayPatterns)
{
	// test(5);
	//设置线程布局
	dim3 BlockSize;
	dim3 GridSize;
	__GetBlockSize(rows,cols,BlockSize,GridSize);
	GridSize.z=n;

	//上传基础数组格雷码
	int* code_h;
	code_h=(int*)malloc(sizeof(int)*4);
	code_h[0]=0;
	code_h[1]=1;
	code_h[2]=1;
	code_h[3]=0;
	int* code_d;
	cudaMalloc((void**)&code_d,sizeof(int)*4);
	cudaMemcpy(code_d,code_h,sizeof(int)*4,cudaMemcpyHostToDevice);

	//开始计算
	grayPatterns=CudaImage<uchar>::CudaImage(cols,rows,n);
	__GetGrayCodePattern_cuda<<<GridSize,BlockSize>>>(grayPatterns,max,code_d);
	checkCudaErrors(cudaDeviceSynchronize());
}

/// @brief 格雷码图案二值化
/// @param grayimages 输入图像
/// @param n 二值化阈值
/// @return 
__global__ void __Binaryzation_cuda(CudaImage<uchar> grayimages,int n)
{
	//计算索引
	int x=blockIdx.x*blockDim.x+threadIdx.x;
    int y=blockIdx.y*blockDim.y+threadIdx.y;
	int z=blockIdx.z;//0,1,2,3,4(五)

	if(grayimages(x,y,z)>=n)
	{
		grayimages(x,y,z)=1;
	}
	else
	{
		grayimages(x,y,z)=0;
	}
}

/// @brief 格雷码相位求解-核函数
/// @param grayimages_b 二值化后的格雷码
/// @param binimages_b 用于存储的二进制码
/// @param phasepic 输出相位
/// @param n 阶数
/// @return 
__global__ void __DeGrayCode_cuda(CudaImage<uchar> grayimages_b,CudaImage<uchar> binimages_b,CudaImage<double> phasepic,int n)
{
	//计算索引
	int x=blockIdx.x*blockDim.x+threadIdx.x;
    int y=blockIdx.y*blockDim.y+threadIdx.y;

	//将格雷码转为10进制码得到相位
	double phase=0;
	for (size_t i = 1; i < n; i++)
	{
		binimages_b(x,y,i)=(grayimages_b(x,y,i) == binimages_b(x,y,i-1))? 0 : 1;
		phase+=binimages_b(x,y,i)*pow(2,n-i-1);
	}
	phase+=binimages_b(x,y,0)*pow(2,n-1);
	phasepic(x,y)=phase*2*M_PI;
}

/// @brief 格雷码相位求解
/// @param patterns_gray 输入格雷码图片
/// @param n 二值化阈值
/// @return 
void PatternCoder::DeGrayCode_cuda(CudaImage<uchar>& patterns_gray, int n,CudaImage<double>& gray)
{
	//设置线程布局
	int rows_=patterns_gray.y_size;
	int cols_=patterns_gray.x_size;
	dim3 BlockSize;
	dim3 GridSize;
	__GetBlockSize(rows_,cols_,BlockSize,GridSize);
	GridSize.z=patterns_gray.z_size;

	//图像二值化
	__Binaryzation_cuda<<<GridSize,BlockSize>>>(patterns_gray,n);
	checkCudaErrors(cudaDeviceSynchronize());

	//解码
	CudaImage<uchar> patterns_bin_cuda;
	patterns_bin_cuda.copyImage(patterns_gray);
	gray=CudaImage<double>::CudaImage(cols_,rows_);
	GridSize.z=1;
	__DeGrayCode_cuda<<<GridSize,BlockSize>>>(patterns_gray,patterns_bin_cuda,gray,patterns_gray.z_size);
	checkCudaErrors(cudaDeviceSynchronize());
}

/************************************多频外差******************************************/

/// @brief 提前计算多频外差核函数需要用到的参数并上传，防止重复计算
/// @param T1 多频周期
/// @param T2 多频周期
/// @param T3 多频周期
/// @param N 相移步数
void PatternCoder::ParametersPreCompute(double T1,double T2,double T3,double N,double& T123)
{
	//预计算cos和sin系数
	double* coslist_h;
	double* sinlist_h;
	int listsize=sizeof(double)*N;
	coslist_h=(double*)malloc(listsize);
	sinlist_h=(double*)malloc(listsize);
	for (int i = 0; i < N; i++)
	{
		coslist_h[i]=cos(2*M_PI*i/N);
		sinlist_h[i]=sin(2*M_PI*i/N);
	}
	// double* coslist_d;
	// double* sinlist_d;
	cudaMalloc((void**)&coslist_d,listsize);
	cudaMemcpy(coslist_d,coslist_h,listsize,cudaMemcpyHostToDevice);
	cudaMalloc((void**)&sinlist_d,listsize);
	cudaMemcpy(sinlist_d,sinlist_h,listsize,cudaMemcpyHostToDevice);

	//计算合成周期
	double T12=__getT_cuda(T1,T2);
	double T23=__getT_cuda(T2,T3);
	T123=__getT_cuda(T12,T23);
	cout<<"[PatternCoder::ParametersPreCompute]:T123="<<T123<<endl;

	double TTT_h[3];
	double TT_h[2];
	double T_h[1];
	TTT_h[0]=T1;
	TTT_h[1]=T2;
	TTT_h[2]=T3;
	TT_h[0]=T12;
	TT_h[1]=T23;
	T_h[0]=T123;
	cudaMalloc((void**)&TTT_d,sizeof(double)*3);
	cudaMemcpy(TTT_d,TTT_h,sizeof(double)*3,cudaMemcpyHostToDevice);
	cudaMalloc((void**)&TT_d,sizeof(double)*2);
	cudaMemcpy(TT_d,TT_h,sizeof(double)*2,cudaMemcpyHostToDevice);
	cudaMalloc((void**)&T_d,sizeof(double)*1);
	cudaMemcpy(T_d,T_h,sizeof(double)*1,cudaMemcpyHostToDevice);

	//计算归一化系数
	double max2_h[2];
	double max_h[1];
	max2_h[0]=TT_h[0]/TTT_h[0];
	max2_h[1]=TT_h[1]/TTT_h[1];
	max_h[0]=T_h[0]/TT_h[0];
	cudaMalloc((void**)&max2_d,sizeof(double)*2);
	cudaMemcpy(max2_d,max2_h,sizeof(double)*2,cudaMemcpyHostToDevice);
	cudaMalloc((void**)&max_d,sizeof(double)*1);
	cudaMemcpy(max_d,max_h,sizeof(double)*1,cudaMemcpyHostToDevice);
}

/// @brief 获取相移法投影图案集-多频外差版-核函数-纵向条纹
/// @param patterns 传入分配的图像集显存地址
/// @param TTT 多个周期对应的像素数
/// @param step 相移大小
/// @param A 背景光强
/// @param B 调制度
/// @return 
__global__ void __GetShiftedPhaseSetN_cuda_v(CudaImage<uchar> patterns,double* TTT,double step,double A,double B,int n)
{
	//计算索引
	int x=blockIdx.x*blockDim.x+threadIdx.x;
    int y=blockIdx.y*blockDim.y+threadIdx.y;

	//计算
	// double phase = (blockIdx.z%n)*step-M_PI;//原本
	double phase = (blockIdx.z%n)*step;//wyc对比
	double T=TTT[(int)(blockIdx.z/n)];

	
    // patterns(x,y,blockIdx.z)= round(B * cos(((x + 0.5) / T) * 2 * M_PI + phase) / 2 + A);//四舍五入-像素中心
	// patterns(x,y,blockIdx.z)= B * cos(((x + 0.5) / T) * 2 * M_PI + phase) / 2 + A;//直接截断-像素中心
	patterns(x,y,blockIdx.z)= round(B * cos(((double)(x) / T) * 2 * M_PI + phase) / 2 + A);//四舍五入 //wyc的版本会加一
	// if((100<x)&&(x<200))//卷积仿真测试
	// {
	// 	patterns(x,y,blockIdx.z)=0.1*patterns(x,y,blockIdx.z);
	// }
	// patterns(x,y,blockIdx.z)= B * cos(((double)(x) / T) * 2 * M_PI + phase) / 2 + A;//直接截断
}
// 生成一维高斯核
__device__ void generate_gaussian_kernel(double* kernel, int kernel_size, double sigma) {
    double sum = 0.0;
    int center = kernel_size / 2;

    // 计算高斯核的值
    for (int i = 0; i < kernel_size; ++i) {
        double x = i - center;
        kernel[i] = exp(-(x * x) / (2 * sigma * sigma));
        sum += kernel[i];
    }

    // 归一化
    for (int i = 0; i < kernel_size; ++i) {
        kernel[i] /= sum;
    }
}
__global__ void __GetShiftedPhaseSetN_cuda_juanji(CudaImage<uchar> patterns,CudaImage<uchar> patterns_)
{
	//计算索引
	int x=blockIdx.x*blockDim.x+threadIdx.x;
    int y=blockIdx.y*blockDim.y+threadIdx.y;
	if((x<3)||(x>297))
	{
		return;
	}

	double kernel[7];
	generate_gaussian_kernel(kernel,7,1);
	

	double value=0;
	for (int i = -3; i < 4; i++)
	{
		// printf("-[%d][%f]-",i,kernel[i+2]);

		double a=static_cast<double>(patterns_(x+i,y,blockIdx.z));

		// value+=a*kernel[i+3];
		value+=a/7;
	}
	patterns(x,y,blockIdx.z)=static_cast<uchar>(value);
}

/// @brief 获取相移法投影图案集-多频外差版-核函数-横向条纹
/// @param patterns 传入分配的图像集显存地址
/// @param TTT 多个周期对应的像素数
/// @param step 相移大小
/// @param A 背景光强
/// @param B 调制度
/// @return 
__global__ void __GetShiftedPhaseSetN_cuda_h(CudaImage<uchar> patterns,double* TTT,double step,double A,double B,int n)
{
	//计算索引
	int x=blockIdx.x*blockDim.x+threadIdx.x;
    int y=blockIdx.y*blockDim.y+threadIdx.y;

	//计算
	// double phase = (blockIdx.z%n)*step-M_PI;//原本
	double phase = (blockIdx.z%n)*step;//wyc对比
	double T=TTT[(int)(blockIdx.z/n)];

	
    // patterns(x,y,blockIdx.z)= round(B * cos(((x + 0.5) / T) * 2 * M_PI + phase) / 2 + A);//四舍五入-像素中心
	// patterns(x,y,blockIdx.z)= B * cos(((x + 0.5) / T) * 2 * M_PI + phase) / 2 + A;//直接截断-像素中心
	patterns(x,y,blockIdx.z)= round(B * cos(((double)(y) / T) * 2 * M_PI + phase) / 2 + A);//四舍五入 //wyc的版本会加一
	// patterns(x,y,blockIdx.z)= B * cos(((double)(x) / T) * 2 * M_PI + phase) / 2 + A;//直接截断
}

/// @brief 获取相移法投影图案集--多频外差
/// @param n 相移法步数
/// @param A 背景光强
/// @param B 调制度
/// @param TTT 多频外差（三频）周期
/// @param pattern_out 返回图案集
void PatternCoder::GetShiftedPhaseSet_cuda(int n, double A, double B,vector<double> TTT, CudaImage<uchar>& patterns,int rows_p,int cols_p,PatternDirection direction)
{
	//计算线程布局
	dim3 BlockSize_p;
	dim3 GridSize_p;
	__GetBlockSize(rows_p,cols_p,BlockSize_p,GridSize_p);
	GridSize_p.z=n*TTT.size();

	//分配内存
	patterns=CudaImage<uchar>::CudaImage(cols_p,rows_p,n*TTT.size());

	size_t TTT_size=sizeof(double)*TTT.size();
	double* TTT_h;
	TTT_h=(double*)malloc(TTT_size);
	for (size_t i = 0; i < TTT.size(); i++)
	{
		TTT_h[i]=TTT[i];
	}

	double* TTT_d;
	cudaMalloc((void**)&TTT_d,TTT_size);
	cudaMemcpy(TTT_d,TTT_h,TTT_size,cudaMemcpyHostToDevice);
	
	// CudaImage<uchar> patterns_=CudaImage<uchar>::CudaImage(cols_p,rows_p,n*TTT.size());
	//计算
	// if (rows_p==1)
	if (direction==V)
	{
		__GetShiftedPhaseSetN_cuda_v<<<GridSize_p,BlockSize_p>>>(patterns,TTT_d,2*M_PI/n,A,B,n);
		// __GetShiftedPhaseSetN_cuda_juanji<<<GridSize_p,BlockSize_p>>>(patterns,patterns_);//卷积仿真测试
	}
	// else if(cols_p==1)
	else if(direction==H)
	{
		__GetShiftedPhaseSetN_cuda_h<<<GridSize_p,BlockSize_p>>>(patterns,TTT_d,2*M_PI/n,A,B,n);
	}
	else
	{
		throw std::runtime_error("[GetShiftedPhaseSet_cuda]行和列至少有一个为1");
	}
	checkCudaErrors(cudaDeviceSynchronize());
}

__global__ void __GetB_cuda(int N,double* coslist,double* sinlist,const CudaImage<uchar> phaseimage,CudaImage<double> B)//,CudaImage<double> B
{
	//计算索引
	int x=blockIdx.x*blockDim.x+threadIdx.x;
    int y=blockIdx.y*blockDim.y+threadIdx.y;
	int z=blockIdx.z;

	//计算相位
	float denominator = 0, molecule = 0;
	for (int i = 0; i < N; i++)
	{
		denominator+=phaseimage(x,y,z*N+i)*coslist[i];
		molecule+=phaseimage(x,y,z*N+i)*sinlist[i];
	}

	B(x,y,z)=2*sqrt(molecule*molecule+denominator*denominator)/N;
}

void PatternCoder::GetB(int n,const CudaImage<uchar> &patterns_N,CudaImage<double>& B,int Tsize)
{
	//如果输入的是模拟相位则修改网格尺寸
	dim3 BlockSize_temp=BlockSize;
	dim3 GridSize_temp=GridSize;
	GridSize_temp.z=Tsize;
	if(patterns_N.x_size==1)
	{
		BlockSize_temp.x=1;
		GridSize_temp.x=1;
	}
	if(patterns_N.y_size==1)
	{
		BlockSize_temp.y=1;
		GridSize_temp.y=1;
	}
	//执行核函数
	__GetB_cuda<<<GridSize_temp,BlockSize_temp>>>(n,coslist_d,sinlist_d,patterns_N,B);//,B_N
	checkCudaErrors(cudaDeviceSynchronize());
	// checkCudaErrors(cudaStreamSynchronize(cudaStreamDefault));//暂时修改
}

/// @brief N步相移法求解相位-多频外差-核函数
/// @param n 相移步数
/// @param phaseimage 条纹图案
/// @param phase 求解相位
/// @param B 调制度
/// @return 
__global__ void __DeShiftedPhaseSetN_cuda(int N,double* coslist,double* sinlist,double B_min,const CudaImage<uchar> phaseimage,CudaImage<double> phase)//,CudaImage<double> B
{
	//计算索引
	int x=blockIdx.x*blockDim.x+threadIdx.x;
    int y=blockIdx.y*blockDim.y+threadIdx.y;
	int z=blockIdx.z;

	//计算相位
	float denominator = 0, molecule = 0;
	for (int i = 0; i < N; i++)
	{
		denominator+=phaseimage(x,y,z*N+i)*coslist[i];
		molecule+=phaseimage(x,y,z*N+i)*sinlist[i];
	}
	// phase(x,y,z)=atan2(-molecule, denominator) + M_PI;//抬高后用于生成图像

	// phase(x,y,z)=atan2(-molecule, denominator);//wyc的没有抬高
	// B(x,y,z)=2*sqrt(molecule*molecule+denominator*denominator)/N;

	// phase(x,y,z)=(molecule*molecule+denominator*denominator)/N;//用于测试观察调制度
	// if(phase(x,y,z)<=0.1)
	// {
	// 	phase(x,y,z)=phase(x,y,z)-1000;
	// }
	if(((molecule*molecule+denominator*denominator)/N)>B_min)//同步进行调制度滤波，减少后续需要处理的像素点
	{
		phase(x,y,z)=atan2(-molecule, denominator);//wyc的没有抬高
	}
}

/// @brief N步相移法求解相位-多频外差
/// @param n 相移步数
/// @param B_min  调制度滤波阈值
/// @param patterns_N 相移图案
/// @param phaseN 求解相位
/// @param Tsize 频率数
void PatternCoder::DeShiftedPhase_cuda(int n,double B_min,const CudaImage<uchar> &patterns_N, CudaImage<double>& phaseN,int Tsize)//,CudaImage<double>& B_N（目前不需要调制度图像）
{
	//如果输入的是模拟相位则修改网格尺寸
	dim3 BlockSize_temp=BlockSize;
	dim3 GridSize_temp=GridSize;
	GridSize_temp.z=Tsize;
	if(patterns_N.x_size==1)
	{
		BlockSize_temp.x=1;
		GridSize_temp.x=1;
	}
	if(patterns_N.y_size==1)
	{
		BlockSize_temp.y=1;
		GridSize_temp.y=1;
	}
	//执行核函数
	__DeShiftedPhaseSetN_cuda<<<GridSize_temp,BlockSize_temp>>>(n,coslist_d,sinlist_d,B_min,patterns_N,phaseN);//,B_N
	checkCudaErrors(cudaDeviceSynchronize());
	// checkCudaErrors(cudaStreamSynchronize(cudaStreamDefault));//暂时修改
}

/// @brief 解包裹函数
/// @param a 
/// @param m 
/// @return 
__inline__ __device__ double __mod_cuda(double a, double m)
{
	return a - m * floor(a / m);
}

/// @brief 分层解包裹
/// @param phN 底层包裹相位
/// @param TTT 底层周期
/// @param TT 上层周期
/// @param max 相位标准化系数
/// @param ph123 输出解包裹相位
/// @return 
__global__ void __UnwrappPhase_cuda3(const CudaImage<double> phN,double* TTT,double* TT,double* max,CudaImage<double> ph123)
{
	//计算索引
	int x=blockIdx.x*blockDim.x+threadIdx.x;
    int y=blockIdx.y*blockDim.y+threadIdx.y;
	int z=blockIdx.z;

	//合成
	double pha1_v=phN(x,y,z);
	if(pha1_v==-100)//跳过调制度滤除点
	{
		return;
	}
	double pha2_v=phN(x,y,z+1);
	double delta = __mod_cuda(pha1_v - pha2_v, 2. * M_PI);
	ph123(x,y,z)=(pha1_v + round((delta * max[z] - pha1_v) / (2. * M_PI)) * 2. * M_PI)/max[z];
}

/// @brief 分次解包裹
/// @param ph1 相位1
/// @param ph2 相位2
/// @param T1 周期1
/// @param T2 周期2
/// @param T12 合成周期
/// @param max 归一化参数
/// @param ph12 合成相位
/// @return 
__global__ void __UnwrappPhase_cuda(const CudaImage<double> ph1,const CudaImage<double> ph2,double T1,double T2,double T12,double max,CudaImage<double> ph12)
{
	//计算索引
	int x=blockIdx.x*blockDim.x+threadIdx.x;
    int y=blockIdx.y*blockDim.y+threadIdx.y;
	int z=blockIdx.z;

	//合成
	double pha1_v=ph1(x,y);
	double pha2_v=ph2(x,y);
	double delta = __mod_cuda(pha1_v - pha2_v, 2. * M_PI);
	ph12(x,y)=(pha1_v + round((delta * max - pha1_v) / (2. * M_PI)) * 2. * M_PI)/max;
}

/// @brief 三频外差合成主相位-三频同时求解
/// @param phaseN 三频相位
/// @param ph1223 中间相位
/// @param ph123 输出合成最终相位
void PatternCoder::UnwrappPhase3_cuda(const CudaImage<double>& phaseN, CudaImage<double>& ph1223,CudaImage<double>& ph123)
{
	//如果输入的是模拟相位则修改网格尺寸
	dim3 BlockSize_temp=BlockSize;
	dim3 GridSize_temp=GridSize;
	if(phaseN.x_size==1)
	{
		BlockSize_temp.x=1;
		GridSize_temp.x=1;
		cout<<"[PatternCoder::UnwrappPhase3_cuda]：模拟相位"<<endl;
	}
	if(phaseN.y_size==1)
	{
		BlockSize_temp.y=1;
		GridSize_temp.y=1;
		cout<<"[PatternCoder::UnwrappPhase3_cuda]：模拟相位"<<endl;
	}

	GridSize_temp.z=2;
	__UnwrappPhase_cuda3<<<GridSize_temp,BlockSize_temp>>>(phaseN,TTT_d,TT_d,max2_d,ph1223);
	checkCudaErrors(cudaDeviceSynchronize());
	// checkCudaErrors(cudaStreamSynchronize(cudaStreamDefault));//暂时修改
	GridSize_temp.z=1;
	__UnwrappPhase_cuda3<<<GridSize_temp,BlockSize_temp>>>(ph1223,TT_d,T_d,max_d,ph123);
	checkCudaErrors(cudaDeviceSynchronize());
	// checkCudaErrors(cudaStreamSynchronize(cudaStreamDefault));//暂时修改
	// ph1223.ImageFree();
}

/// @brief 调制度滤波核函数（单张）-添加sobel边缘检测-改为边缘腐蚀
/// @param B 计算的调制度
/// @param min 调制度阈值
/// @param e 边缘检测阈值
/// @param ph123 待滤波的相位
/// @return 
template <typename T>
__global__ void __filterB(double e,CudaImage<T> imgsrc,CudaImage<T> imgdst)//const CudaImage<T> B,double min,
{
	//计算索引
	int x=blockIdx.x*blockDim.x+threadIdx.x;
    int y=blockIdx.y*blockDim.y+threadIdx.y;
	// int z=blockId,x.z;
	// imgsrc(x,y)=-200;
/*
	double gx = 0, gy = 0;//sobel边缘剔除
	gx+=imgsrc(x-1,y-1)*-1;
	gx+=imgsrc(x,y-1)*-2;
	gx+=imgsrc(x+1,y-1)*-1;
	gx+=imgsrc(x-1,y+1)*1;
	gx+=imgsrc(x,y+1)*2;
	gx+=imgsrc(x+1,y+1)*1;

	gy+=imgsrc(x-1,y-1)*-1;
	gy+=imgsrc(x,y-1)*-2;
	gy+=imgsrc(x+1,y-1)*-1;
	gy+=imgsrc(x-1,y+1)*1;
	gy+=imgsrc(x,y+1)*2;
	gy+=imgsrc(x+1,y+1)*1;

	// if((B(x,y)<=min)||(sqrt(gx * gx + gy * gy)>e))//去除边缘
	// if(B(x,y)<=min)//不去除边缘
	if((sqrt(gx * gx + gy * gy)>e))//仅仅去除边缘
	{
		// imgdst(x,y)=-100;
		return;
	}
	else
	{
		imgdst(x,y)=imgsrc(x,y);
	}
*/
// /*
	// imgdst(x,y)=imgsrc(x,y);
	if(imgsrc(x,y)==-100)//跳过调制度剔除点
	{
		// imgdst(x,y)=-100;
		return;
	}

	if((imgsrc(x+1,y)<0)||(imgsrc(x-1,y)<0)||(imgsrc(x,y+1)<0)||(imgsrc(x,y-1)<0))//图像腐蚀
	{
		// imgdst(x,y)=-100;
		return;
	}
	else if((imgsrc(x+1,y+1)<0)||(imgsrc(x-1,y-1)<0)||(imgsrc(x-1,y+1)<0)||(imgsrc(x+1,y-1)<0))//图像腐蚀
	{
		// imgdst(x,y)=-100;
		return;
	}
	else
	{
		imgdst(x,y)=imgsrc(x,y);//不腐蚀的点进行赋值
	}
// */
}

template void PatternCoder::filterB<uchar>(double min,CudaImage<uchar>& imgsrc,CudaImage<uchar>& imgdst);//const CudaImage<uchar>& B,
template void PatternCoder::filterB<float>(double min,CudaImage<float>& imgsrc,CudaImage<float>& imgdst);//const CudaImage<float>& B,
template void PatternCoder::filterB<double>(double min,CudaImage<double>& imgsrc,CudaImage<double>& imgdst);//const CudaImage<double>& B,
/// @brief 调制度滤波(单张图像)左右图像可以独立处理，最后一起同步
/// @param ph123 待修改的相位
/// @param B 调制度
template <typename T>
void PatternCoder::filterB(double e,CudaImage<T>& imgsrc,CudaImage<T>& imgdst)//const CudaImage<T>& B,
{
	// GridSize.z=ph123.z_size;
	// cout<<GridSize.z<<endl;
	__filterB<<<GridSize,BlockSize>>>(e,imgsrc,imgdst);//B,min,
	// checkCudaErrors(cudaDeviceSynchronize());
}

/// @brief 获取计算的block大小
/// @return 
dim3 PatternCoder::getblockdim()
{
	return BlockSize;
}
/// @brief 获取计算的grid大小
/// @return 
dim3 PatternCoder::getgriddim()
{
	return GridSize;
}
//*********************************私有函数****************************************//

/// @brief 合成周期
/// @param T1 周期1
/// @param T2 周期2
/// @return 合成的周期T12
inline double PatternCoder::__getT_cuda(double T1,double T2)
{
	return T1 * T2 / (T2 - T1);
}

/// @brief 根据图像大小自动分配block和grid的大小
/// @param rows 
/// @param cols 
/// @param BlockSize 
/// @param Gridsize 
void PatternCoder::__GetBlockSize(int rows,int cols,dim3& BlockSize,dim3& Gridsize)
{
	//找出两个方向中所有可以被2的n次方整除的值
    vector<int> row_pows,col_pows;
    for (int i = 10; i >= 0; i--)
    {
        int result1=rows%(int)pow(2,i);
        int result2=cols%(int)pow(2,i);

        if(result1==0)
        {
            row_pows.push_back(i);
        }
        if(result2==0)
        {
            col_pows.push_back(i);
        }

    }

    //计算所有值的配对选项，满足线程块最大尺寸限制，并计算可以达到的最大线程数
    vector<pair<int,int>> ijlist;
    vector<int> valuelist;
    int value=0;
    int max_value=0;
    for (int i = 0; i < row_pows.size(); i++)
    {
        for (int j = 0; j < col_pows.size(); j++)
        {
            value=pow(2,row_pows[i])*pow(2,col_pows[j]);
            if(value<=1024)
            {
                // cout<<row_pows[i]<<" "<<col_pows[j]<<endl;
                ijlist.push_back(make_pair(row_pows[i],col_pows[j]));
                valuelist.push_back(value);
                if(value>max_value)
                {
                    max_value=value;
                }
            }
        }
    }

    //挑出所有满足最大线程数限制的配对，找出其中两个方向差值最小的值
    vector<pair<int,int>> ijlist_max;
    vector<int> offsetlist;
    int min_value=1024;
    int offset=0;
    for (size_t i = 0; i < ijlist.size(); i++)
    {
        if(valuelist[i]==max_value)
        {
            ijlist_max.push_back(ijlist[i]);
            offset=abs(ijlist[i].first-ijlist[i].second);
            offsetlist.push_back(offset);
            if(offset<min_value)
            {
                min_value=offset;
            }
        }
    }

    //计算最终大小
    for (size_t i = 0; i < ijlist_max.size(); i++)
    {
        if(offsetlist[i]==min_value)
        {
            // cout<<"ijlist_max:"<<ijlist_max[i].first<<" "<<ijlist_max[i].second<<" "<<offsetlist[i]<<endl;
			BlockSize.y=pow(2,ijlist_max[i].first);
			BlockSize.x=pow(2,ijlist_max[i].second);
			BlockSize.z=1;
			cout<<"[__GetBlockSize]:BlockSize="<<BlockSize.x<<" "<<BlockSize.y<<" "<<BlockSize.z<<endl;
			break;
        }
    }
	Gridsize.y=rows/BlockSize.y;
	Gridsize.x=cols/BlockSize.x;
	Gridsize.z=1;
	cout<<"[__GetBlockSize]:GirdSize="<<Gridsize.x<<" "<<Gridsize.y<<" "<<Gridsize.z<<endl;
}