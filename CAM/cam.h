#ifndef CAM_H
#define CAM_H

#include "CAM/inc/GalaxyIncludes.h"
#include "CAM/Common/GXBitmap.h"

//qt
#include <QString>
#include <QStringList>

class CAM
{
public:
    CAM();
    void Open_Cam(int index);
    void Close_Cam();

public:// 显示、保存图像相关函数与成员变量
    void StartSnap();
    void StopSnap();

    CGXBitmap*  m_pBitmap;///< 图像采集与保存控制

private:// 显示、保存图像相关函数与成员变量

public://设备控制相关成员
    std::string GetDeviceSN();

    //控制当前触发模式
    QStringList TriggerModeList;
    int GetCurrentTriggerMode();
    void SetCurrentTriggerMode(int index);

    //控制当前触发源
    QStringList TriggerSourceList;
    int GetCurrentTriggerSource();
    void SetCurrentTriggerSource(int index);

    //控制当前触发开关
    QStringList TriggerActiveList;
    int GetCurrentTriggerActive();
    void SetCurrentTriggerActive(int index);

    //软触发
    void SoftwareTrigger();

    //曝光时间控制
    bool SetShutterValue(double value);
    double GetShutterValue();
    void GetShutterValueMinMax(double& min,double& max);

    //增益控制
    bool SetGain(double value);
    double GetGain();
    void GetGainMinMax(double& min,double& max);

    //帧率模式控制
    QStringList FrameRateModeList;
    int GetCurrentFrameRateMode();
    void SetCurrentFrameRateMode(int index);

    //帧率控制
    bool SetFrameRate(double value);
    double GetFrameRate();
    void GetFrameRateMinMax(double& min,double& max);

    //降噪状态控制
    QStringList DeNoiseModeList;
    int GetCurrentDeNoiseMode();
    void SetCurrentDeNoiseMode(int index);

    //降噪控制
    bool SetDeNoiseValue(double value);
    double GetDeNoiseValue();
    void GetDeNoiseMinMax(double& min,double& max);

    //状态判断
    bool isColor();
    bool isOpen();
    bool isSnap();

    // 子线程中不能直接弹窗，错误信息记录到这里并由上层 worker 通过信号通知 GUI。
    QString lastError() const;
    void clearLastError();

private://设备控制相关成员
    void setLastError(const QString& errorMessage);
    void InitParam();
    void InitModelList(gxstring strName,QStringList& ModeList,int& parameter);

    gxstring SN;
    CGXDevicePointer m_objDevicePtr; ///< 设备句柄
    CGXStreamPointer m_objStreamPtr; ///< 设备流
    CGXFeatureControlPointer m_objFeatureControlPtr; ///< 属性控制器
    CGXFeatureControlPointer m_objStreamFeatureControlPtr; ///< 流层控制器对象

    bool m_bIsOpen; //< 设备打开标识
    bool m_bIsSnap; ///< 设备采集标识
    bool m_bColorFilter; ///< 是否支持彩色相机
    bool m_bTriggerMode; ///< 是否支持触发模式
    bool m_bTriggerSource; ///< 是否支持触发源
    bool m_bTriggerActive; ///< 是否支持触发极性
    bool m_bNoiseReductionMode;///< 是否支持降噪
    bool m_bBalanceWhiteAuto; ///< 是否支持自动白平衡
    bool m_bBalanceWhiteRatioSelect; ///< 是否支持白平衡通道选择
    double m_dShutterValueMax; ///< 曝光时间最大值
    double m_dShutterValueMin; ///< 曝光时间最小值
    double m_dShutterValue;///< 曝光时间当前值
    double m_dGainValueMax; ///< 增益最大值
    double m_dGainValueMin; ///< 增益最小值
    double m_dGainValue;///< 增益当前值
    double m_dFrameRateMax; ///< 帧率最大值
    double m_dFrameRateMin; ///< 帧率最小值
    double m_dFrameRateValue;///< 帧率当前值
    double m_dDeNoiseMax; ///< 降噪最大值
    double m_dDeNoiseMin; ///< 降噪最小值
    double m_dDeNoiseValue;///< 降噪当前值
    double m_dBalanceWhiteRatioMax; ///< 自动白平衡系数最大值
    double m_dBalanceWhiteRatioMin; ///< 自动白平衡系数最小值
    double m_dEditBalanceRatioValue;///< 当前白平衡系数值
    int m_nTriggerModeOld; ///< 记录触发模式
    int m_nTriggerSourceOld; ///< 记录触发源
    int m_nTriggerActiveOld; ///< 记录触发极性
    int m_nFrameRateModeOld; ///< 记录帧率调节模式
    int m_nDeNoiseModeOld; ///< 记录降噪模式
    int m_nBalanceWhiteAutoOld; ///< 记录自动白平衡
    int m_nBanlanceWhiteRatioOld; ///< 记录自动白平衡系数
    gxstring m_strBalanceWhiteAutoMode; ///< 记录自动白平衡方式
    QString m_lastError; ///< 最近一次相机 SDK 操作错误信息。
};
#endif // CAM_H
