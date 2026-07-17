//------------------------------------------------------------------------
/**
\file		GXBitmap.h
\brief		此类主要用于图像的显示和存储，图像显示和存储可以自适应黑白彩色相机，
图像存储可以存储为Bmp、Raw，对图像显示和存储进行了声明

*/
//------------------------------------------------------------------------
#pragma once

#include "CAM/inc/GalaxyIncludes.h"
#include "threadpool.h"
//qt
#include <QObject>
#include <QImage>
//#include <thread>//调试用


class CGXBitmap:public QObject,public ICaptureEventHandler
{
    Q_OBJECT
public:
	///构造函数
    CGXBitmap(CGXDevicePointer& objCGXDevicePointer);

	///析构函数
	~CGXBitmap(void);

    //返回图像尺寸
    QSize GetPicSize();

    //设置存图路径
    void SetSavePath(std::string SavePath);

    //获取存图路径
    std::string GetSavepath();

    //设置是否保存
    void SetSave(bool save);

    //设置是否单次保存
    void SaveOnce();

    //设置是否计数保存
    void SetCount(bool c);

    //重新计数
    void ReCount();

signals:
    void ShowCallback(BYTE* pBuffer);//采集触发信号并传送图像内存地址

private:
    //回调函数
    void DoOnImageCaptured(CImageDataPointer& objImageDataPointer, void* pUserParam);

    ///显示图像
    void Show(CImageDataPointer& objCImageDataPointer);

    //显示图像及帧率
    void ShowWithFPS(CImageDataPointer& objCImageDataPointer,char* strDeviceSNFPS);

    ///图像处理后并显示图像
    void ShowImageProcess(CImageProcessConfigPointer& objCfg,CImageDataPointer& objCImageDataPointer);

    /// 存储Bmp图像
    void SaveBmp(CImageDataPointer& objCImageDataPointer);

    /// 存储Raw图像
    void SaveRaw(CImageDataPointer& objCImageDataPointer,const std::string& strFilePath);

    ///通过GX_PIXEL_FORMAT_ENTRY获取最优Bit位
    GX_VALID_BIT_LIST GetBestValudBit(GX_PIXEL_FORMAT_ENTRY emPixelFormatEntry);

	///判断PixelFormat是否为8位
	bool __IsPixelFormat8(GX_PIXEL_FORMAT_ENTRY emPixelFormatEntry);

	///为彩色相机图像显示准备资源
	void __ColorPrepareForShowImg();

	///为黑白相机图像显示准备资源
	void __MonoPrepareForShowImg();

	///判断是否兼容
	bool __IsCompatible(BITMAPINFO *pBmpInfo, uint64_t nWidth, uint64_t nHeight);

	///更新Bitmap的信息
	void __UpdateBitmap(CImageDataPointer& objCImageDataPointer);

	///将m_pBufferRGB中图像显示到界面
	void __DrawImg(BYTE* pBuffer);

	///将m_pBufferRGB中图像和帧率显示到界面
	void __DrawImg(BYTE* pBuffer, char* strDeviceSNFPS);

	///计算宽度所占的字节数
	int64_t __GetStride(int64_t nWidth, bool bIsColor);

    /// 是否支持彩色
    void __IsSupportColor(CGXDevicePointer& objCGXDevicePointer, bool &bIsColorFilter);

private:
	CGXBitmap& operator=(const CGXBitmap&);
    CGXBitmap(const CGXBitmap&);
    bool               m_bIsColor ;                      ///<是否支持彩色相机
    int64_t            m_nImageHeight;                   ///<原始图像高
    int64_t            m_nImageWidth;                    ///<原始图像宽
    BITMAPINFO         *m_pBmpInfo;	                     ///<BITMAPINFO 结构指针，显示图像时使用
    char               m_chBmpBuf[2048];	             ///<BIMTAPINFO 存储缓冲区，m_pBmpInfo即指向此缓冲区
    HDC                m_hDC;                            ///<绘制图像DC句柄
    BYTE               *m_pImageBuffer;                  ///<保存翻转后的图像用于显示和保存

    BOOL m_bCheckSaveBmp;///< 用于检查是否保存图像
    bool saveonce;
    std::string m_strSavePath; ///< 图像保存路径
    bool count_save=false;
    int count=0;

    ThreadPool *pool;
    std::mutex mtx;

};

