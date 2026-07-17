#include "cam.h"
///
/// \brief CAM::CAM 初始化相机类以及相关参数
///
CAM::CAM()
{
    //初始化参数
    m_bIsOpen=false;
    m_bIsSnap=false;
    m_bColorFilter=false;
    m_bTriggerMode=false;
    m_bTriggerSource=false;
    m_bTriggerActive=false;
    m_bBalanceWhiteAuto=false;
    m_bBalanceWhiteRatioSelect=false;

    m_strBalanceWhiteAutoMode="Off";
    m_pBitmap=NULL;
    m_dEditBalanceRatioValue=0;
    m_nTriggerModeOld=0;
    m_nTriggerSourceOld=0;
    m_nTriggerActiveOld=0;
    m_nBalanceWhiteAutoOld=0;
    m_nBanlanceWhiteRatioOld=0;
    m_nFrameRateModeOld=0;
    m_nDeNoiseModeOld=0;

    m_dShutterValueMax=0;
    m_dShutterValueMin=0;
    m_dShutterValue=0;
    m_dGainValueMax=0;
    m_dGainValueMin=0;
    m_dGainValue=0;
    m_dFrameRateMax=0;
    m_dFrameRateMin=0;
    m_dFrameRateValue=0;
    m_dDeNoiseMax=0;
    m_dDeNoiseMin=0;
    m_dDeNoiseValue=0;
    m_dBalanceWhiteRatioMax=0;
    m_dBalanceWhiteRatioMin=0;

    TriggerModeList.push_back("none");
    TriggerSourceList.push_back("none");
    TriggerActiveList.push_back("none");
    FrameRateModeList.push_back("none");
    DeNoiseModeList.push_back("none");

    //初始化设备
    IGXFactory::GetInstance().Init();
    SN="null";
}

QString CAM::lastError() const
{
    return m_lastError;
}

void CAM::clearLastError()
{
    m_lastError.clear();
}

void CAM::setLastError(const QString& errorMessage)
{
    m_lastError = errorMessage;
}

///
/// \brief CAM::Open_Cam 打开设备
/// \param index 打开的设备编号
///
void CAM::Open_Cam(int index)
{
    clearLastError();
    if(m_bIsOpen)
    {
        setLastError(QString::fromLocal8Bit("设备已经打开!"));
        return;
    }

    bool bIsDeviceOpen = false;         ///< 设备是否打开标志
    bool bIsStreamOpen = false;         ///< 设备流是否打开标志

    try
    {
        //枚举设备
        GxIAPICPP::gxdeviceinfo_vector vectorDeviceInfo;
        IGXFactory::GetInstance().UpdateDeviceList(1000,vectorDeviceInfo);

        //判断枚举到的设备是否大于零，如果不是则弹框提示
        if (vectorDeviceInfo.size() <= 0)
        {
           setLastError(QString::fromLocal8Bit("未发现设备!"));
           return;
        }
        else if(index < 0 || index>vectorDeviceInfo.size()-1)
        {
            setLastError(QString::fromLocal8Bit("打开设备超出范围!"));
            return;
        }

        //打开设备
        m_objDevicePtr = IGXFactory::GetInstance().OpenDeviceBySN(vectorDeviceInfo[index].GetSN(),GX_ACCESS_EXCLUSIVE);
        SN=vectorDeviceInfo[index].GetSN();
        bIsDeviceOpen = true;
        m_objFeatureControlPtr = m_objDevicePtr->GetRemoteFeatureControl();

        //判断画图对象是否为空
        if (m_pBitmap != NULL)
        {
            delete m_pBitmap;
            m_pBitmap = NULL;
        }

        //为画图对象分配内存
        m_pBitmap = new CGXBitmap(m_objDevicePtr);

        //判断设备流是否大于零，如果大于零则打开流
        int nStreamCount = m_objDevicePtr->GetStreamCount();

        if (nStreamCount > 0)
        {
            m_objStreamPtr = m_objDevicePtr->OpenStream(0);
            m_objStreamFeatureControlPtr = m_objStreamPtr->GetFeatureControl();
            bIsStreamOpen = true;
        }
        else
        {
            setLastError(QString::fromLocal8Bit("未发现设备流!"));
            if (bIsDeviceOpen)
            {
                m_objDevicePtr->Close();
            }
            if (m_pBitmap != NULL)
            {
                delete m_pBitmap;
                m_pBitmap = NULL;
            }
            return;
        }

        // 建议用户在打开网络相机之后，根据当前网络环境设置相机的流通道包长值，
        // 以提高网络相机的采集性能,设置方法参考以下代码。
        GX_DEVICE_CLASS_LIST objDeviceClass = m_objDevicePtr->GetDeviceInfo().GetDeviceClass();
        if(GX_DEVICE_CLASS_GEV == objDeviceClass)
        {
            // 判断设备是否支持流通道数据包功能
            if(true == m_objFeatureControlPtr->IsImplemented("GevSCPSPacketSize"))
            {
                // 获取当前网络环境的最优包长值
                int nPacketSize = m_objStreamPtr->GetOptimalPacketSize();
                // 将最优包长值设置为当前设备的流通道包长值
                m_objFeatureControlPtr->GetIntFeature("GevSCPSPacketSize")->SetValue(nPacketSize);
            }
        }

        //初始化设备参数
        InitParam();
        m_bIsOpen = true;//设置设备打开标识

    }
    catch (CGalaxyException& e)
    {
        //判断设备流是否已打开
        if (bIsStreamOpen)
        {
            m_objStreamPtr->Close();
        }

        //判断设备是否已打开
        if (bIsDeviceOpen)
        {
            m_objDevicePtr->Close();
        }

        if (m_pBitmap != NULL)
        {
            delete m_pBitmap;
            m_pBitmap = NULL;
        }

        setLastError(QString::fromLocal8Bit(e.what()));
        return;
    }
    catch (std::exception& e)
    {
        //判断设备流是否已打开
        if (bIsStreamOpen)
        {
            m_objStreamPtr->Close();
        }

        //判断设备是否已打开
        if (bIsDeviceOpen)
        {
            m_objDevicePtr->Close();
        }

        if (m_pBitmap != NULL)
        {
            delete m_pBitmap;
            m_pBitmap = NULL;
        }

        setLastError(QString::fromLocal8Bit(e.what()));

        return;
    }
}

///
/// \brief CAM::Close_Cam——关闭设备
///
void CAM::Close_Cam()
{
    clearLastError();
    // TODO: Add your control notification handler code here
    try
    {
        //判断是否已停止采集
        if (m_bIsSnap)
        {
            //发送停采命令
            m_objFeatureControlPtr->GetCommandFeature("AcquisitionStop")->Execute();

            //关闭流层采集
            m_objStreamPtr->StopGrab();

            //注销采集回调
            m_objStreamPtr->UnregisterCaptureCallback();
        }
    }
    catch(CGalaxyException)
    {
        //do noting
    }

    try
    {
        //关闭流对象
        m_objStreamPtr->Close();
    }
    catch(CGalaxyException)
    {
        //do noting
    }

    try
    {
        //关闭设备
        m_objDevicePtr->Close();
    }
    catch(CGalaxyException)
    {
        //do noting
    }

    m_bIsOpen = false;
    m_bIsSnap = false;

    if (m_pBitmap != NULL)
    {
        delete m_pBitmap;
        m_pBitmap = NULL;
    }

}

///
/// \brief CAM::InitParam 初始化相机参数
///
void CAM::InitParam()
{
    bool bBalanceWhiteAutoRead = false;         ///< 白平衡是否可读

    //设置采集模式为连续采集模式
    m_objFeatureControlPtr->GetEnumFeature("AcquisitionMode")->SetValue("Continuous");

    //是否支持触发模式选择
    m_bTriggerMode = m_objFeatureControlPtr->IsImplemented("TriggerMode");
    if (m_bTriggerMode)
    {
        //设置触发模式关
        m_objFeatureControlPtr->GetEnumFeature("TriggerMode")->SetValue("Off");
        InitModelList("TriggerMode",TriggerModeList,m_nTriggerModeOld);
    }

    //是否支持Bayer格式
    m_bColorFilter = m_objFeatureControlPtr->IsImplemented("PixelColorFilter");

    //是否支持触发源选择
    m_bTriggerSource = m_objFeatureControlPtr->IsImplemented("TriggerSource");
    if(m_bTriggerSource)
    {
        InitModelList("TriggerSource",TriggerSourceList,m_nTriggerSourceOld);
    }

    //是否支持触发极性选择
    m_bTriggerActive = m_objFeatureControlPtr->IsImplemented("TriggerActivation");
    if(m_bTriggerActive)
    {
        InitModelList("TriggerActivation",TriggerActiveList,m_nTriggerActiveOld);
    }

    //是否支持帧率调节模式（这里还没找到如何读取是否支持）
    InitModelList("AcquisitionFrameRateMode",FrameRateModeList,m_nFrameRateModeOld);

    //是否支持降噪
    m_bNoiseReductionMode = m_objFeatureControlPtr->IsImplemented("NoiseReductionMode");
    if(m_bNoiseReductionMode)
    {
        InitModelList("NoiseReductionMode",DeNoiseModeList,m_nDeNoiseModeOld);
    }

    //是否支持自动白平衡
    m_bBalanceWhiteAuto = m_objFeatureControlPtr->IsImplemented("BalanceWhiteAuto");

    //白平衡是否可读
    bBalanceWhiteAutoRead   = m_objFeatureControlPtr->IsReadable("BalanceWhiteAuto");

    //如果支持且可读，则获取设备当前白平衡模式
    if (m_bBalanceWhiteAuto)
    {
        if (bBalanceWhiteAutoRead)
        {
            m_strBalanceWhiteAutoMode = m_objFeatureControlPtr->GetEnumFeature("BalanceWhiteAuto")->GetValue();
        }
    }

    //是否支持自动白平衡通道选择
    m_bBalanceWhiteRatioSelect = m_objFeatureControlPtr->IsImplemented("BalanceRatioSelector");

    //获取曝光时间、增益、帧率、降噪及自动白平衡系数的最大值和最小值
    m_dShutterValueMax      = m_objFeatureControlPtr->GetFloatFeature("ExposureTime")->GetMax();
    m_dShutterValueMin      = m_objFeatureControlPtr->GetFloatFeature("ExposureTime")->GetMin();
    m_dShutterValue            =m_objFeatureControlPtr->GetFloatFeature("ExposureTime")->GetValue();

    m_dGainValueMax         = m_objFeatureControlPtr->GetFloatFeature("Gain")->GetMax();
    m_dGainValueMin         = m_objFeatureControlPtr->GetFloatFeature("Gain")->GetMin();
    m_dGainValue                = m_objFeatureControlPtr->GetFloatFeature("Gain")->GetValue();

    m_dFrameRateMax         = m_objFeatureControlPtr->GetFloatFeature("AcquisitionFrameRate")->GetMax();
    m_dFrameRateMin         = m_objFeatureControlPtr->GetFloatFeature("AcquisitionFrameRate")->GetMin();
    m_dFrameRateValue       = m_objFeatureControlPtr->GetFloatFeature("AcquisitionFrameRate")->GetValue();

    if(m_bNoiseReductionMode)
    {
        m_dDeNoiseMax         = m_objFeatureControlPtr->GetFloatFeature("NoiseReduction")->GetMax();
        m_dDeNoiseMin         = m_objFeatureControlPtr->GetFloatFeature("NoiseReduction")->GetMin();
        m_dDeNoiseValue       = m_objFeatureControlPtr->GetFloatFeature("NoiseReduction")->GetValue();
    }

    m_dBalanceWhiteRatioMax = m_objFeatureControlPtr->GetFloatFeature("BalanceRatio")->GetMax();
    m_dBalanceWhiteRatioMin = m_objFeatureControlPtr->GetFloatFeature("BalanceRatio")->GetMin();
}

///
/// \brief CAM::InitModelList——初始化模式选项列表
/// \param strName——模式类型
/// \param ModeList——输出
///
void CAM::InitModelList(gxstring strName,QStringList& ModeList,int& parameter)
{
    ModeList.clear();
    CEnumFeaturePointer objEnumFeaturePtr = m_objFeatureControlPtr->GetEnumFeature(strName);

    //获取设备当前功能值
    std::string  strCurrentEnumList = "";                 // 设备当前功能
    strCurrentEnumList = objEnumFeaturePtr->GetValue().c_str();

    //获取设备所有枚举项
    GxIAPICPP::gxstring_vector vectorEnumEntryList;  // 设备枚举项
    vectorEnumEntryList = objEnumFeaturePtr->GetEnumEntryList();
    for (uint32_t i = 0; i<vectorEnumEntryList.size(); i++)
    {
        std::string strEnumList = vectorEnumEntryList[i].c_str();
        ModeList.push_back(QString::fromStdString(strEnumList));
        if (strCurrentEnumList == vectorEnumEntryList[i].c_str())
        {
            parameter = i;
        }
    }
}

///
/// \brief CAM::StartSnap 开始采集图像并显示
///
void CAM::StartSnap()
{
    clearLastError();
    // TODO: Add your control notification handler code here
    try
    {
        try
        {
            //设置Buffer处理模式
            m_objStreamFeatureControlPtr->GetEnumFeature("StreamBufferHandlingMode")->SetValue("OldestFirst");
        }
        catch (...)
        {
        }

        //注册回调函数
        m_objStreamPtr->RegisterCaptureCallback(m_pBitmap,this);

        //开启流层通道
        m_objStreamPtr->StartGrab();

        //发送开采命令
        m_objFeatureControlPtr->GetCommandFeature("AcquisitionStart")->Execute();
        m_bIsSnap = true;
    }
    catch (CGalaxyException& e)
    {
        setLastError(QString::fromLocal8Bit(e.what()));
        return;
    }
    catch (std::exception& e)
    {
        setLastError(QString::fromLocal8Bit(e.what()));
        return;
    }
}

///
/// \brief CAM::StopSnap 停止采集
///
void CAM::StopSnap()
{
    clearLastError();
    // TODO: Add your control notification handler code here
    try
    {
        //发送停采命令
        m_objFeatureControlPtr->GetCommandFeature("AcquisitionStop")->Execute();

        //关闭流层通道
        m_objStreamPtr->StopGrab();

        //注销采集回调
        m_objStreamPtr->UnregisterCaptureCallback();
        m_bIsSnap = false;
    }
    catch (CGalaxyException& e)
    {
        setLastError(QString::fromLocal8Bit(e.what()));
        return;
    }
    catch (std::exception& e)
    {
        setLastError(QString::fromLocal8Bit(e.what()));
        return;
    }
}

///
/// \brief CAM::GetDeviceSN——获得设备SN号
/// \return
///
std::string CAM::GetDeviceSN()
{
    return SN.c_str();
}

///
/// \brief CAM::GetCurrentTriggerMode——获得参数
/// \return
///
int CAM::GetCurrentTriggerMode()
{
    return m_nTriggerModeOld;
}

///
/// \brief CAM::SetCurrentTriggerMode——设置参数
/// \param index
///
void CAM::SetCurrentTriggerMode(int index)
{
    clearLastError();
    try
    {
        gxstring  strCurrentText    = "";
        strCurrentText = TriggerModeList[index].toStdString().c_str();

        //将当前功能设置到设备中
        m_objFeatureControlPtr->GetEnumFeature("TriggerMode")->SetValue(strCurrentText);
        m_nTriggerModeOld = index;
    }
    catch (CGalaxyException& e)
    {
        setLastError(QString::fromLocal8Bit(e.what()));
        return;
    }
    catch (std::exception& e)
    {
        setLastError(QString::fromLocal8Bit(e.what()));
        return;
    }
}

///
/// \brief CAM::GetCurrentTriggerSource——获得参数
/// \return
///
int CAM::GetCurrentTriggerSource()
{
    return m_nTriggerSourceOld;
}

///
/// \brief CAM::SetCurrentTriggerSource——设置参数
/// \param index
///
void CAM::SetCurrentTriggerSource(int index)
{
    clearLastError();
    try
    {
        gxstring  strCurrentText    = "";
        strCurrentText = TriggerSourceList[index].toStdString().c_str();

        //将当前功能设置到设备中
        m_objFeatureControlPtr->GetEnumFeature("TriggerSource")->SetValue(strCurrentText);
        m_nTriggerSourceOld = index;
    }
    catch (CGalaxyException& e)
    {
        setLastError(QString::fromLocal8Bit(e.what()));
        return;
    }
    catch (std::exception& e)
    {
        setLastError(QString::fromLocal8Bit(e.what()));
        return;
    }
}

///
/// \brief CAM::GetCurrentTriggerActive——获得参数
/// \return
///
int CAM::GetCurrentTriggerActive()
{
    return m_nTriggerActiveOld;
}

///
/// \brief CAM::SetCurrentTriggerActive——设置参数
/// \param index
///
void CAM::SetCurrentTriggerActive(int index)
{
    clearLastError();
    try
    {
        gxstring  strCurrentText    = "";
        strCurrentText = TriggerActiveList[index].toStdString().c_str();

        //将当前功能设置到设备中
        m_objFeatureControlPtr->GetEnumFeature("TriggerActivation")->SetValue(strCurrentText);
        m_nTriggerActiveOld = index;
    }
    catch (CGalaxyException& e)
    {
        setLastError(QString::fromLocal8Bit(e.what()));
        return;
    }
    catch (std::exception& e)
    {
        setLastError(QString::fromLocal8Bit(e.what()));
        return;
    }
}

///
/// \brief CAM::SoftwareTrigger——发送软触发指令
///
void CAM::SoftwareTrigger()
{
    clearLastError();
    try
    {
        //发送软触发命令(在触发模式开启时有效)
        m_objFeatureControlPtr->GetCommandFeature("TriggerSoftware")->Execute();
    }
    catch (CGalaxyException& e)
    {
        setLastError(QString::fromLocal8Bit(e.what()));
        return;
    }
    catch (std::exception& e)
    {
        setLastError(QString::fromLocal8Bit(e.what()));
        return;
    }
}

bool CAM::SetShutterValue(double value)
{
    clearLastError();
    try
    {
        if((value<=m_dShutterValueMax)&(value>=m_dShutterValueMin))
        {
            m_objFeatureControlPtr->GetFloatFeature("ExposureTime")->SetValue(value);
            m_dShutterValue=m_objFeatureControlPtr->GetFloatFeature("ExposureTime")->GetValue();
            return true;
        }

        setLastError(QString::fromLocal8Bit("曝光时间超出允许范围"));
        return false;
    }
    catch (CGalaxyException& e)
    {
        setLastError(QString::fromLocal8Bit(e.what()));
        return false;
    }
    catch (std::exception& e)
    {
        setLastError(QString::fromLocal8Bit(e.what()));
        return false;
    }
}

double CAM::GetShutterValue()
{
    m_dShutterValue=m_objFeatureControlPtr->GetFloatFeature("ExposureTime")->GetValue();
    return m_dShutterValue;
}

void CAM::GetShutterValueMinMax(double &min, double &max)
{
    min=m_dShutterValueMin;
    max=m_dShutterValueMax;
}

bool CAM::SetGain(double value)
{
    clearLastError();
    try
    {
        if((value<=m_dGainValueMax)&(value>=m_dGainValueMin))
        {
            m_objFeatureControlPtr->GetFloatFeature("Gain")->SetValue(value);
            m_dGainValue=m_objFeatureControlPtr->GetFloatFeature("Gain")->GetValue();
            return true;
        }

        setLastError(QString::fromLocal8Bit("增益超出允许范围"));
        return false;
    }
    catch (CGalaxyException& e)
    {
        setLastError(QString::fromLocal8Bit(e.what()));
        return false;
    }
    catch (std::exception& e)
    {
        setLastError(QString::fromLocal8Bit(e.what()));
        return false;
    }
}

double CAM::GetGain()
{
    return m_dGainValue;
}

void CAM::GetGainMinMax(double &min, double &max)
{
    min=m_dGainValueMin;
    max=m_dGainValueMax;
}

int CAM::GetCurrentFrameRateMode()
{
    return m_nFrameRateModeOld;
}

void CAM::SetCurrentFrameRateMode(int index)
{
    clearLastError();
    try
    {
        gxstring  strCurrentText    = "";
        strCurrentText = FrameRateModeList[index].toStdString().c_str();

        //将当前功能设置到设备中
        m_objFeatureControlPtr->GetEnumFeature("AcquisitionFrameRateMode")->SetValue(strCurrentText);
        m_nFrameRateModeOld = index;
    }
    catch (CGalaxyException& e)
    {
        setLastError(QString::fromLocal8Bit(e.what()));
        return;
    }
    catch (std::exception& e)
    {
        setLastError(QString::fromLocal8Bit(e.what()));
        return;
    }
}

bool CAM::SetFrameRate(double value)
{
    clearLastError();
    try
    {
        if((value<=m_dFrameRateMax)&(value>=m_dFrameRateMin))
        {
            m_objFeatureControlPtr->GetFloatFeature("AcquisitionFrameRate")->SetValue(value);
            m_dFrameRateValue=m_objFeatureControlPtr->GetFloatFeature("AcquisitionFrameRate")->GetValue();
            return true;
        }

        setLastError(QString::fromLocal8Bit("帧率超出允许范围"));
        return false;
    }
    catch (CGalaxyException& e)
    {
        setLastError(QString::fromLocal8Bit(e.what()));
        return false;
    }
    catch (std::exception& e)
    {
        setLastError(QString::fromLocal8Bit(e.what()));
        return false;
    }
}

double CAM::GetFrameRate()
{
    return m_dFrameRateValue;
}

void CAM::GetFrameRateMinMax(double &min, double &max)
{
    min=m_dFrameRateMin;
    max=m_dFrameRateMax;
}

int CAM::GetCurrentDeNoiseMode()
{
    return m_nDeNoiseModeOld;
}

void CAM::SetCurrentDeNoiseMode(int index)
{
    clearLastError();
    try
    {
        gxstring  strCurrentText    = "";
        strCurrentText = DeNoiseModeList[index].toStdString().c_str();

        //将当前功能设置到设备中
        m_objFeatureControlPtr->GetEnumFeature("NoiseReductionMode")->SetValue(strCurrentText);
        m_nDeNoiseModeOld = index;
    }
    catch (CGalaxyException& e)
    {
        setLastError(QString::fromLocal8Bit(e.what()));
        return;
    }
    catch (std::exception& e)
    {
        setLastError(QString::fromLocal8Bit(e.what()));
        return;
    }
}

bool CAM::SetDeNoiseValue(double value)
{
    clearLastError();
    try
    {
        if((value<=m_dDeNoiseMax)&(value>=m_dDeNoiseMin))
        {
            m_objFeatureControlPtr->GetFloatFeature("NoiseReduction")->SetValue(value);
            m_dDeNoiseValue=m_objFeatureControlPtr->GetFloatFeature("NoiseReduction")->GetValue();
            return true;
        }

        setLastError(QString::fromLocal8Bit("out of permission range"));
        return false;
    }
    catch (CGalaxyException& e)
    {
        setLastError(QString::fromLocal8Bit(e.what()));
        return false;
    }
    catch (std::exception& e)
    {
        setLastError(QString::fromLocal8Bit(e.what()));
        return false;
    }
}

double CAM::GetDeNoiseValue()
{
    return m_dDeNoiseValue;
}

void CAM::GetDeNoiseMinMax(double &min, double &max)
{
    min=m_dDeNoiseMin;
    max=m_dDeNoiseMax;
}

bool CAM::isColor()
{
    return m_bColorFilter;
}

bool CAM::isOpen()
{
    return m_bIsOpen;
}

bool CAM::isSnap()
{
    return m_bIsSnap;
}
