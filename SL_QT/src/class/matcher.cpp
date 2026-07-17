#include "matcher.h"
//pcl
#include <pcl/visualization/pcl_visualizer.h>
// #include "Spline.h"

Matcher::Matcher(/* args */)
{

}

Matcher::~Matcher()
{
    cudaFree(d_grid_count);
    cudaFree(d_grid_data);
}

/// @brief 将图像以点云形式显示
/// @tparam T 图像元素类型
/// @param pic 图片
/// @param cloud 点云
template<typename T>
void cvMat_PCL1(const cv::Mat& pic,pcl::PointCloud<pcl::PointXYZ>& cloud)
{
	cloud.clear();
	for (int i = 0; i < pic.rows; i++)
	{
		for (int j = 0; j < pic.cols; j++)
		{
			pcl::PointXYZ p;
			p.x=i;
			p.y=j;
			p.z=pic.at<T>(i,j);
			cloud.push_back(p);
		}
        // cout<<i<<endl;
	}
}


//sgbm放弃了
void Matcher::stereoSGBM(cv::Mat lpng,cv::Mat rpng,string filepath,cv::Mat &disp)
{
    disp.create(lpng.rows,lpng.cols,CV_16S);
    cv::Mat disp1 = cv::Mat(lpng.rows,lpng.cols,CV_8UC1);
    cv::Size imgSize = lpng.size();
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create();

    int nmDisparities = ((imgSize.width / 8) + 15) & -16;//视差搜索范围
    // nmDisparities=100;
    int pngChannels = lpng.channels();//获取左视图通道数
    int winSize = 3;

    // int p1=8*pngChannels*winSize*winSize;
    int p1=10;
    // cout<<"p1="<<p1<<endl;
    // int p2=32*pngChannels*winSize*winSize;
    int p2=100;
    // cout<<"p2="<<p2<<endl;

    sgbm->setPreFilterCap(31);//预处理滤波器截断值
    sgbm->setBlockSize(winSize);//SAD窗口大小
    sgbm->setP1(p1);//控制视差平滑度第一参数
    sgbm->setP2(p2);//控制视差平滑度第二参数
    sgbm->setMinDisparity(0);//最小视差
    sgbm->setNumDisparities(nmDisparities);//视差搜索范围
    sgbm->setUniquenessRatio(1);//视差唯一性百分比
    sgbm->setSpeckleWindowSize(100);//检查视差连通区域变化度的窗口大小
    sgbm->setSpeckleRange(32);//视差变化阈值
    sgbm->setDisp12MaxDiff(1);//左右视差图最大容许差异
    sgbm->setMode(cv::StereoSGBM::MODE_SGBM);//采用全尺寸双通道动态编程算法
    sgbm->compute(lpng,rpng,disp);

    disp.convertTo(disp1,CV_8U,255 / (nmDisparities*16.));//转8位

    cv::imwrite(filepath+"/disp"+to_string(p1)+".bmp",disp1);
    cout<<"Disp saved!,path="+filepath+"/disp"+to_string(p1)+".bmp"<<endl;

    pcl::PointCloud<pcl::PointXYZ> disp1_cloud;
    cvMat_PCL1<short>(disp,disp1_cloud);
}

void Matcher::match(cv::Mat& img_L,cv::Mat& img_R,cv::Mat& disp,double minerr)
{
    disp.create(img_L.size(),CV_64FC1);
    for (int i = 0; i < img_L.rows; i++)//遍历每一行
    {
        for (int j = 0; j < img_L.cols; j++)
        {
            double Lp=img_L.at<double>(i,j);//待搜索的左图像相位
            double min=100;//存储搜索到的相位差最小值
            int mincol=-1;
            if(Lp<=0)
            {
                break;
            }
            for (int k = 0; k < img_L.cols; k++)
            {
                double L_R=abs(Lp-img_R.at<double>(i,k));//相位差的绝对值
                if(L_R<min)
                {
                    min=L_R;//更新最小值
                    mincol=k;//更新最小值对应的x坐标
                }
                if(min>minerr)//设置阈值，满足即设置为0视差
                {
                    break;
                }
                else
                {
                    disp.at<double>(i,j)=j-mincol;
                }
            }
            
        }
    }
    
}

void Matcher::reprojectTo3D(cv::Mat &dif_map, cv::Mat &depth_map,cv::Mat Q,pcl::PointCloud<pcl::PointXYZ>& depth_cloud)
{
    cv::reprojectImageTo3D(dif_map, depth_map, Q, false, CV_32FC1);
    for (int i = 0; i < depth_map.rows; i++)
    {
        for (int j = 0; j < depth_map.cols; j++)
        {
            if((depth_map.at<cv::Vec3f>(i, j)[2]>10)&(depth_map.at<cv::Vec3f>(i, j)[2]<1000))
            {
                depth_cloud.push_back(pcl::PointXYZ(
                depth_map.at<cv::Vec3f>(i, j)[0],
                depth_map.at<cv::Vec3f>(i, j)[1],
                depth_map.at<cv::Vec3f>(i, j)[2])
                );
            }
        }
    }
}