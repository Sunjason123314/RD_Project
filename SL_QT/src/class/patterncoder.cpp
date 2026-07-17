#pragma once
#include "patterncoder.h"

#ifndef USECUDA
PatternCoder::PatternCoder(int rows_in,int cols_in):
rows(rows_in),
cols(cols_in)
{

}
PatternCoder::~PatternCoder()
{

}
#endif


/// <summary>
/// 生成相移法投影图像
/// </summary>
/// <param name="T">一个周期对应的像素数</param>
/// <param name="phase">偏移的相位值</param>
/// <param name="A">背景光强</param>
/// <param name="B">调制度</param>
/// <returns></returns>
void PatternCoder::__GetShiftedPhase(double T, double phase, int A,int B,cv::Mat& pattern)
{
	pattern=cv::Mat::zeros(rows,cols,CV_8UC1);

	double temp = 0;
	for (int i = 0; i < pattern.cols; i++)
	{
		// temp = B * cos(((i + 0.5) / T) * 2 * M_PI + phase) / 2 + A;//像素中心点对应值
		temp = B * cos(((double)(i) / T) * 2 * M_PI + phase) / 2 + A;
		//cout << temp << endl;
		pattern.col(i).setTo(temp);
	}
}

/// <summary>
/// 获取相移法投影图案集
/// </summary>
/// <param name="n">相移法步数</param>
/// <param name="T">一个周期对应的像素数</param>
/// <param name="max">最大像素值</param>
/// <returns>返回图案集</returns>
void PatternCoder::GetShiftedPhaseSet(int n, double T, int A, int B, vector<cv::Mat>& pattern_out)
{
	pattern_out.clear();
	double phase = 2 * M_PI / n;
	for (int i = 0; i < n; i++)
	{
		cv::Mat pattern;
		__GetShiftedPhase(T, i * phase - M_PI, A, B, pattern);
		pattern_out.push_back(pattern);
	}
}

/// <summary>
/// N步相移法求解相位
/// </summary>
/// <param name="patterns"></param>
/// <returns></returns>
void PatternCoder::DeShiftedPhase(const vector<cv::Mat> &patterns, cv::Mat& phase,cv::Mat& B)//N步相移法
{
	int N = patterns.size();
	int rows = patterns[0].rows, cols = patterns[0].cols;

	//cv::Mat phase2(rows, cols, CV_64FC1);
	//phase2.setTo(0);
	//cv::Mat B(rows, cols, CV_64FC1);
	//B.setTo(0);
	phase = cv::Mat(rows, cols, CV_64FC1);
	B = cv::Mat(rows, cols, CV_64FC1);

	double denominator = 0, molecule = 0;
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			denominator = 0;//分母
			molecule = 0;//分子
			for (int n = 0; n < N; n++)
			{
				denominator += patterns[n].at<uchar>(i, j) * cos(2 * M_PI * n / N);
				molecule += patterns[n].at<uchar>(i, j) * sin(2 * M_PI * n / N);
			}
			//正常公式用这种算法，但是周期不对
			/*if (denominator != 0)
			{
				phase2.at<double>(i, j) = atan(-molecule / denominator);
			}
			else
			{
				if (molecule > 0)
				{
					phase2.at<double>(i, j) = 0.5 * M_PI;
				}
				else
				{
					phase2.at<double>(i, j) = -0.5 * M_PI;
				}

			}*/
			phase.at<double>(i, j) = atan2(-molecule, denominator) + M_PI;//抬高后用于生成图像
			//phase2.at<double>(i, j) = atan2(-molecule, denominator) + 0;//不抬高，用于观察理论公式
			B.at<double>(i, j) = 2 / N * sqrt(pow(molecule, 2)+ pow(denominator, 2));
		}
	}
}

/// <summary>
/// 计算给定大小的格雷码条纹图像
/// </summary>
/// <param name="rows"></param>
/// <param name="cols"></param>
/// <param name="n">n为格雷码阶数</param>
/// <param name="max">最大亮度</param>
/// <param name="pics">输出图案组</param>
void PatternCoder::GetGrayCodePattern(int n, int max, vector<cv::Mat>& pics)
{
	vector<int> code={0,1,1,0};
	int width = 0;
	for (int i = 0; i < n; i++)
	{
		cv::Mat pic(rows, cols, CV_8U);
		width = cols / pow(2,i+1);
		cout << "width" << (double)cols / (double)(i+2) << endl;
		for (int j = 0; j < pow(2,i+1); j++)
		{
			pic(cv::Rect(j * width, 0, width, rows)).setTo(max * code[j%4]);
		}
		pics.push_back(pic);
	}
}

/// <summary>
/// 格雷码二值化
/// </summary>
/// <param name="input">需要二值化的格雷码图案</param>
/// <param name="n">分割0和1的像素阈值</param>
/// <returns>二值化结果</returns>
cv::Mat PatternCoder::__Binaryzation(cv::Mat input, int n)
{
	cv::Mat output(input.rows, input.cols, CV_8U);
	output.setTo(0);
	for (int i = 0; i < input.rows; i++)
	{
		for (int j = 0; j < input.cols; j++)
		{
			if (input.at<uchar>(i, j) > n)
			{
				output.at<uchar>(i, j) = 1;
			}
		}
	}
	return output;
}

/// <summary>
/// 格雷码相位求解
/// </summary>
/// <param name="patterns_gray">输入格雷码图片</param>
/// <param name="n">二值化阈值</param>
/// <returns></returns>
void PatternCoder::DeGrayCode(const vector<cv::Mat>& patterns_gray, int n,cv::Mat& gray)
{
	vector<cv::Mat> grayPattern_B;
	for (size_t i = 0; i < patterns_gray.size(); i++)
	{
		grayPattern_B.push_back(__Binaryzation(patterns_gray[i], n));
	}
	vector<cv::Mat> binPattern_B=grayPattern_B;

	int GCT = grayPattern_B.size();
	int rows = grayPattern_B[0].rows, cols = grayPattern_B[0].cols;
	//求解相位
	gray=cv::Mat::zeros(rows, cols, CV_64FC1);
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			double phase=0;
			for (int k = 1; k < GCT; k++)
			{
				binPattern_B[k].at<uchar>(i,j)=(grayPattern_B[k].at<uchar>(i, j)==binPattern_B[k-1].at<uchar>(i,j))?0:1;
				phase+=binPattern_B[k].at<uchar>(i,j)*pow(2,GCT-k-1);
			}
			phase+=binPattern_B[0].at<uchar>(i,j)*pow(2,GCT-1);
			gray.at<double>(i, j) = phase*2*M_PI;
		}
	}
}

/// @brief 多频外差相位解包裹
/// @param ph1 相位1
/// @param ph2 相位2
/// @param T1 周期1
/// @param T2 周期2
/// @param ph12 解包裹相位
/// @param T12 解包裹相位周期
void PatternCoder::UnwrappPhase(const cv::Mat& ph1, const cv::Mat& ph2, double T1, double T2, cv::Mat& ph12, double& T12)
{
	ph12 = cv::Mat::zeros(ph1.size(), CV_64FC1);
	T12 = __getT(T1,T2);
	double max=T12/T1;//新版归一化
	for (int row = 0; row < ph1.rows; ++row)
	{
		for (int col = 0; col < ph1.cols; ++col)
		{
			double pha1_v = ph1.at<double>(row, col);
			double pha2_v = ph2.at<double>(row, col);
			double delta = __mod(pha1_v - pha2_v, 2. * M_PI);
			ph12.at<double>(row, col) = pha1_v + round((delta * T12 / T1 - pha1_v) / (2. * M_PI)) * 2. * M_PI;
			ph12.at<double>(row, col) = ph12.at<double>(row, col)/max;//新版归一化
		}
	}
	// __toStdPhase(ph12);//旧版本归一化
}

inline double PatternCoder::__mod(double a, double m)
{
	return a - m * floor(a / m);
}

inline double PatternCoder::__getT(double T1,double T2)
{
	return T1 * T2 / (T2 - T1);
}

inline void PatternCoder::__toStdPhase(cv::Mat& pha)//暂时废弃
{
	double pha_min = 0.0, pha_max = 0.0;
	double* min_p = &pha_min, * max_p = &pha_max;
	cv::minMaxIdx(pha, min_p, max_p);
	if ((pha_max - pha_min)==0)
	{
		cout << "[PatternCoder::toStdPhase]:最大值减最小值为0" << endl;
		return;
	}
	pha = (pha - pha_min) / (pha_max - pha_min) * 2. * M_PI;
}

//消除格雷码相位误差(仅限平板的情况)
cv::Mat MainphaseFilter(const cv::Mat&mainphase)
{
	int rows = mainphase.rows;
	int cols = mainphase.cols;
	cv::Mat result(rows,cols,CV_64FC1);
	result.setTo(0);
	result = mainphase;
	for (int i = 1; i < rows-1; i++)
	{
		for (int j = 1; j < cols-1; j++)
		{
			if (((mainphase.at<double>(i, j) - mainphase.at<double>(i, j - 1)) > 1)& ((mainphase.at<double>(i, j) - mainphase.at<double>(i, j + 1))>1))
			{
				//result(i, j) = 0;
				result.at<double>(i, j) = mainphase.at<double>(i, j) - 2*M_PI;
			}
			else if(((mainphase.at<double>(i, j) - mainphase.at<double>(i, j - 1)) < -1) & ((mainphase.at<double>(i, j) - mainphase.at<double>(i, j + 1)) < -1))
			{
				//result(i, j) = 0;
				result.at<double>(i, j) = mainphase.at<double>(i, j) + 2*M_PI;
			}
		}
	}
	return result;
	//return mainphase;
}

void printtime(const std::vector<timepoint>& timePoints,string name)//计时函数
{
    if((timePoints.size()%2)!=0)
    {
        cout<<"记录时刻不是偶数"<<endl;
        return;
    }
    for (size_t i = 0; i < timePoints.size(); i=i+2)
    {
        std::chrono::duration<double> duration = timePoints[i+1] - timePoints[i];
        std::cout<< name << "运行时间: " << duration.count() << " 秒" << std::endl;
    }
}