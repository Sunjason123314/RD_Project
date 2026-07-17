#include "calibration.h"
/*************************************************图像重映射矫正*********************************************************/
__global__ void cudaRemap(double* pSrcImg, double* pDstImg, float* pMapx, float* pMapy, int inWidth, int inHeight, int outWidth, int outHeight, int channels)
{
	const int tidx = blockDim.x * blockIdx.x + threadIdx.x;
	const int tidy = blockDim.y * blockIdx.y + threadIdx.y;
	if (tidx < outWidth && tidy < outHeight)
	{
		int mapIdx = tidy * outWidth + tidx;
		float u = pMapx[mapIdx];
		float v = pMapy[mapIdx];

		int u1 = floor(u);
		int v1 = floor(v);
		int u2 = u1 + 1;
		int v2 = v1 + 1;
		if (u1 >= 0 && v1 >= 0 && u2 < inWidth && v2 < inHeight)
		{
			float dx = u - u1;
			float dy = v - v1;
			int resultIdx = mapIdx * 3;
			for (int chan = 0; chan < channels; chan++)
			{
				float pixel1 = pSrcImg[(v1 * inWidth + u1) * channels + chan];
				float pixel2 = pSrcImg[(v1 * inWidth + u2) * channels + chan];
				float pixel3 = pSrcImg[(v2 * inWidth + u1) * channels + chan];
				float pixel4 = pSrcImg[(v2 * inWidth + u2) * channels + chan];

				float weightedPixel1 = (1 - dx) * (1 - dy) * pixel1;
				float weightedPixel2 = dx * (1 - dy) * pixel2;
				float weightedPixel3 = (1 - dx) * dy * pixel3;
				float weightedPixel4 = dx * dy * pixel4;

				pDstImg[resultIdx + chan] = double(weightedPixel1 + weightedPixel2 + weightedPixel3 + weightedPixel4);
			}
		}
	}
}

__global__ void cudaRemapSingleChannel(double* pSrcImg, double* pDstImg, float* pMapx, float* pMapy, int inWidth, int inHeight, int outWidth, int outHeight)
{
	const int tidx = blockDim.x * blockIdx.x + threadIdx.x;
	const int tidy = blockDim.y * blockIdx.y + threadIdx.y;
	int mapIdx = tidy * outWidth + tidx;
	// if(pSrcImg[mapIdx]<=-100)//这块有的时候会剔除掉不应当剔除的点，以后还是不加的好
	// {
	// 	return;
	// }
	if (tidx < outWidth && tidy < outHeight)
	{
		// int mapIdx = tidy * outWidth + tidx;
		float u = pMapy[mapIdx];
		float v = pMapx[mapIdx];

		int u1 = floor(u);
		int v1 = floor(v);
		int u2 = u1 + 1;
		int v2 = v1 + 1;
		if (u1 >= 0 && v1 >= 0 && u2 < inWidth && v2 < inHeight)
		{
			float dx = u - u1;
			float dy = v - v1;
			int resultIdx = mapIdx;

			// float pixel1 = pSrcImg[v1 * inWidth + u1];
			// float pixel2 = pSrcImg[v1 * inWidth + u2];
			// float pixel3 = pSrcImg[v2 * inWidth + u1];
			// float pixel4 = pSrcImg[v2 * inWidth + u2];

            //
			float pixel1 = pSrcImg[v1 + u1*inHeight];
			float pixel2 = pSrcImg[v1 + u2*inHeight];
			float pixel3 = pSrcImg[v2 + u1*inHeight];
			float pixel4 = pSrcImg[v2 + u2*inHeight];

			float weightedPixel1 = (1 - dx) * (1 - dy) * pixel1;
			float weightedPixel2 = dx * (1 - dy) * pixel2;
			float weightedPixel3 = (1 - dx) * dy * pixel3;
			float weightedPixel4 = dx * dy * pixel4;

			pDstImg[resultIdx] = double(weightedPixel1 + weightedPixel2 + weightedPixel3 + weightedPixel4);
		}
	}
}

void Calibration::Rectify_cuda(CudaImage<double>& src_L, CudaImage<double>& src_R,CudaImage<double>& rectifyImageL, CudaImage<double>& rectifyImageR,dim3 blocksize,dim3 gridsize,int width,int height)
{
    cudaRemapSingleChannel<<<gridsize,blocksize>>>(src_L.data,rectifyImageL.data,mapLx_cuda.data,mapLy_cuda.data,width,height,width,height);
    cudaRemapSingleChannel<<<gridsize,blocksize>>>(src_R.data,rectifyImageR.data,mapRx_cuda.data,mapRy_cuda.data,width,height,width,height);
    checkCudaErrors(cudaDeviceSynchronize());
	// checkCudaErrors(cudaStreamSynchronize(cudaStreamDefault));//暂时修改
}

void Calibration::Rectify_distortion_cuda(CudaImage<double>& src_L, CudaImage<double>& src_R,CudaImage<double>& rectifyImageL, CudaImage<double>& rectifyImageR,dim3 blocksize,dim3 gridsize,int width,int height)
{
    cudaRemapSingleChannel<<<gridsize,blocksize>>>(src_L.data,rectifyImageL.data,mapLx_distortion_cuda.data,mapLy_distortion_cuda.data,width,height,width,height);
    cudaRemapSingleChannel<<<gridsize,blocksize>>>(src_R.data,rectifyImageR.data,mapRx_distortion_cuda.data,mapRy_distortion_cuda.data,width,height,width,height);
    checkCudaErrors(cudaDeviceSynchronize());
	// checkCudaErrors(cudaStreamSynchronize(cudaStreamDefault));//暂时修改
}