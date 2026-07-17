#include "confocalsensorworker.h"

#include <QByteArray>
#include <QTextCodec>
#include <algorithm>
#include <cstdint>
#include <cstring>

namespace
{
QString decodeCfSdkText(const char *text)
{
    if(text == nullptr || *text == '\0')
        return QString();

    const QByteArray bytes(text);

    // Hypersen/CF SDK 的回调文本在不同版本中可能是 UTF-8 或 GBK/GB18030。
    // 先严格按 UTF-8 解码；如果出现非法字节，再按中文 Windows 常见编码回退，避免中文乱码。
    QTextCodec *utf8Codec = QTextCodec::codecForName("UTF-8");
    if(utf8Codec)
    {
        QTextCodec::ConverterState state;
        const QString decodedText = utf8Codec->toUnicode(bytes.constData(), bytes.size(), &state);
        if(state.invalidChars == 0)
            return decodedText;
    }

    QTextCodec *chineseCodec = QTextCodec::codecForName("GB18030");
    if(!chineseCodec)
        chineseCodec = QTextCodec::codecForName("GBK");
    if(chineseCodec)
        return chineseCodec->toUnicode(bytes.constData(), bytes.size());

    return QString::fromLocal8Bit(bytes.constData(), bytes.size());
}
}

ConfocalSensorWorker::ConfocalSensorWorker(QObject *parent)
    : QObject(parent)
{
}

ConfocalSensorWorker::~ConfocalSensorWorker()
{
    // worker 销毁时兜底释放 SDK 资源，避免线程退出或窗口关闭时设备仍保持采样状态。
    if(m_hasHandle)
    {
        CF_RegisterEventCallback(nullptr, nullptr);
        CF_StartSample(m_handle, false);
        for(int channel = 0; channel < 4; ++channel)
            CF_SetIntParam(m_handle, PARAM_SIGNAL_DATA_OUTPUT, channel, 0);

        m_started.store(false, std::memory_order_release);
        CF_CloseDevice(m_handle);
        m_hasHandle = false;
        m_handle = 0;
    }
}

void ConfocalSensorWorker::setDeviceHandle(int handle, bool isCF3000Lite)
{
    // 登录对话框只会在 SDK 打开设备成功后调用这里；即使 SDK 返回的句柄值为 0，也按有效句柄接管。
    m_handle = handle;
    m_hasHandle = true;
    m_isCF3000Lite = isCF3000Lite;
}

bool ConfocalSensorWorker::hasHandle() const
{
    return m_hasHandle;
}

void ConfocalSensorWorker::verifyConnection()
{
    // 没有句柄说明还未登录或设备打开失败。
    if(!hasHandle())
    {
        emit connectionChecked(false, 0);
        return;
    }

    int connectStatus = 0;
    StatusTypeDef ret = CF_GetIntParam(m_handle, PARAM_CONNECT_STATE, 0, &connectStatus);
    const bool connected = (ret == Status_Succeed && connectStatus == 1);

    if(connected)
    {
        // 连接成功后注册 SDK 事件回调，用于接收实时测量值、断线和异常通知。
        CF_RegisterEventCallback(&ConfocalSensorWorker::sdkEventCallback, this);
    }
    else if(ret != Status_Succeed)
    {
        emit operationFailed(QStringLiteral("verifyConnection"), static_cast<int>(ret));
    }

    emit connectionChecked(connected, connectStatus);
}

void ConfocalSensorWorker::requestInitialConfig(int singleChannelIndex, int lightChannelIndex, int filterAndSignalChannelIndex)
{
    if(!hasHandle())
        return;

    // 读取界面初始化所需的全部 SDK 参数，集中打包后一次性通知 UI 线程刷新控件。
    ConfocalInitialConfig config;
    config.isCF3000Lite = m_isCF3000Lite;
    config.communicationEnabled = !m_isCF3000Lite;
    config.cf3000LiteFilePathEnabled = m_isCF3000Lite;

    // 单通道测量参数。
    CF_GetIntParam(m_handle, PARAM_CHANNEL_MEASURE_MODE, singleChannelIndex, &config.singleMeasureMode);
    CF_GetIntParam(m_handle, PARAM_CHANNEL_ABS_MODE, singleChannelIndex, &config.singleAbsMode);
    CF_GetIntParam(m_handle, PARAM_CHANNEL_REVERSE_VALUE, singleChannelIndex, &config.singleReverse);
    CF_GetIntParam(m_handle, PARAM_CHANNEL_MULT_DIS, singleChannelIndex, &config.singleMultDistance);

    // 曝光时间是两类设备都需要的基础参数。
    CF_GetIntParam(m_handle, PARAM_EXPOSURE_TIME, 0, &config.exposureTime);

    if(m_isCF3000Lite)
    {
        // CF3000Lite 使用自动曝光相关参数。
        CF_GetIntParam(m_handle, PARAM_AUTO_EXP, 0, &config.autoExposureEnable);
        CF_GetIntParam(m_handle, PARAM_AUTO_EXP_CHANNEL, 0, &config.autoExposureChannel);
        CF_GetIntParam(m_handle, PARAM_MAX_AUTO_EXP, 0, &config.autoExposureTime);
    }
    else
    {
        // 非 Lite 设备使用光源、背景光和串口通信参数。
        CF_GetIntParam(m_handle, PARAM_AUTO_LIGHT, lightChannelIndex, &config.lightMode);
        CF_GetIntParam(m_handle, PARAM_AUTO_LIGHT_MODE, lightChannelIndex, &config.autoLightReference);
        CF_GetFloatParam(m_handle, PARAM_LIGHT_VALUE, lightChannelIndex, &config.lightValue);
        CF_GetIntParam(m_handle, PARAM_SUB_BACKGROUND_LIGHT, 0, &config.subBackgroundLight);

        CF_GetIntParam(m_handle, PARAM_RSXXX_SELECT, 0, &config.communicationMode);
        CF_GetIntParam(m_handle, PARAM_RSXXX_PROTOCOL, 0, &config.serialPort);
        CF_GetIntParam(m_handle, PARAM_RSXXX_DATA_FORMAT, 0, &config.dataFormat);
        CF_GetIntParam(m_handle, PARAM_RSXXX_BAUDRATE, 0, &config.baudRate);
        CF_GetIntParam(m_handle, PARAM_RSXXX_PARITY, 0, &config.verify);
    }

    // 信号选择、阈值、比例和滤波参数。
    CF_GetIntParam(m_handle, PARAM_SELECT_SIGNAL, filterAndSignalChannelIndex, &config.exportSignalSelect);
    CF_GetIntParam(m_handle, PARAM_SIGNAL_SORT, filterAndSignalChannelIndex, &config.signalSort);
    CF_GetFloatParam(m_handle, PARAM_SIGNAL_DETECT_THRE, filterAndSignalChannelIndex, &config.signalThreshold);
    CF_GetFloatParam(m_handle, PARAM_SIGNAL_CAL_RATIO, filterAndSignalChannelIndex, &config.signalRatio);
    CF_GetIntParam(m_handle, PARAM_SIGNAL_DETECT_SENSITIVITY, filterAndSignalChannelIndex, &config.signalSensitivity);
    CF_GetIntParam(m_handle, PARAM_SIGNAL_DETECT_NUM, filterAndSignalChannelIndex, &config.signalCount);
    CF_GetIntParam(m_handle, PARAM_SIGNAL_MINI_POINTS, filterAndSignalChannelIndex, &config.signalMinNum);
    CF_GetIntParam(m_handle, PARAM_SIGNAL_SMOOTH, filterAndSignalChannelIndex, &config.signalSmoothLength);
    CF_GetIntParam(m_handle, PARAM_MOVING_MEDIAN_FILTER, filterAndSignalChannelIndex, &config.signalMedianFilter);
    CF_GetIntParam(m_handle, PARAM_MOVING_AVG_FILTER, filterAndSignalChannelIndex, &config.signalAverageFilter);

    // 触发模式决定界面上帧率设置区域是否可编辑。
    CF_GetIntParam(m_handle, PARAM_TRIGGER_MODE, 0, &config.triggerMode);
    config.frameSettingEnabled = (config.triggerMode == 3);

    if(!m_isCF3000Lite)
    {
        // 编码器参数仅对非 Lite 设备读取。
        CF_GetIntParam(m_handle, PARAM_ENCODER_WORKING_MODE, 0, &config.encodeWorkMode);
        CF_GetIntParam(m_handle, PARAM_ENCODER_INPUT_MODE, 0, &config.encodeInputMode);
        CF_GetIntParam(m_handle, PARAM_ENCODER_DIVISION, 0, &config.encodeDivision);
    }

    emit initialConfigReady(config);
}

void ConfocalSensorWorker::requestStatusSnapshot()
{
    if(!hasHandle())
        return;

    // SDK 字符串接口使用 char 数组输出，随后按 UTF-8/GBK 自动识别后转成 QString 交给界面显示。
    char sdkVersionResult[64] = {0};
    char controlerVersionResult[64] = {0};
    char sensorSNResult[64] = {0};
    char controlerSNResult[64] = {0};

    ConfocalStatusSnapshot snapshot;
    CF_GetStringParam(m_handle, PARAM_SDK_VERSION, 0, sdkVersionResult);
    CF_GetStringParam(m_handle, PARAM_CONTROLER_VERSION, 0, controlerVersionResult);
    CF_GetStringParam(m_handle, PARAM_SENSOR_SN, 0, sensorSNResult);
    CF_GetStringParam(m_handle, PARAM_CONTROLER_SN, 0, controlerSNResult);
    CF_GetIntParam(m_handle, PARAM_TRIGGER_MODE, 0, &snapshot.triggerMode);
    CF_GetIntParam(m_handle, PARAM_RT_FRAME_RATE, 0, &snapshot.curFrameRate);

    snapshot.sdkVersion = decodeCfSdkText(sdkVersionResult);
    snapshot.controlerVersion = decodeCfSdkText(controlerVersionResult);
    snapshot.sensorSN = decodeCfSdkText(sensorSNResult);
    snapshot.controlerSN = decodeCfSdkText(controlerSNResult);

    emit statusSnapshotReady(snapshot);
}

void ConfocalSensorWorker::startSample(int singleChannelIndex)
{
    if(!hasHandle())
    {
        emit operationFailed(QStringLiteral("startSample"), static_cast<int>(Status_Others));
        return;
    }

    // 记录通道索引，便于停止采样和析构清理时关闭同一通道的数据输出。
    m_lastSingleChannelIndex = singleChannelIndex;
    StatusTypeDef ret = CF_StartSample(m_handle, true);
    if(ret == Status_Succeed)
    {
        // 开启信号数据输出后，SDK 回调中才能拿到对应通道曲线数据。
        CF_SetIntParam(m_handle, PARAM_SIGNAL_DATA_OUTPUT, singleChannelIndex, 1);
        m_started.store(true, std::memory_order_release);
    }

    emit sampleStateChanged(ret == Status_Succeed, static_cast<int>(ret));
}

void ConfocalSensorWorker::stopSample(int singleChannelIndex)
{
    if(!hasHandle())
    {
        emit sampleStateChanged(false, static_cast<int>(Status_Others));
        return;
    }

    m_lastSingleChannelIndex = singleChannelIndex;
    StatusTypeDef ret = CF_StartSample(m_handle, false);
    if(ret == Status_Succeed)
    {
        // 停止后关闭信号数据输出，减少 SDK 回调压力。
        CF_SetIntParam(m_handle, PARAM_SIGNAL_DATA_OUTPUT, singleChannelIndex, 0);
        m_started.store(false, std::memory_order_release);
    }

    emit sampleStateChanged(false, static_cast<int>(ret));
}

void ConfocalSensorWorker::closeDevice()
{
    if(!hasHandle())
    {
        emit deviceClosed();
        return;
    }

    const bool wasStarted = m_started.load(std::memory_order_acquire);

    // 先注销 SDK 回调，再停止采样并关闭所有通道的数据输出，避免退出过程中 SDK 继续回调已释放对象。
    CF_RegisterEventCallback(nullptr, nullptr);
    CF_StartSample(m_handle, false);
    for(int channel = 0; channel < 4; ++channel)
        CF_SetIntParam(m_handle, PARAM_SIGNAL_DATA_OUTPUT, channel, 0);

    m_started.store(false, std::memory_order_release);

    if(wasStarted)
        emit sampleStateChanged(false, static_cast<int>(Status_Succeed));

    CF_CloseDevice(m_handle);
    m_hasHandle = false;
    m_handle = 0;
    m_isCF3000Lite = false;

    emit deviceClosed();
}

void ConfocalSensorWorker::setFactoryFilePath(const QString &path)
{
    // SDK 接口接收本地编码字符串，路径来自用户选择的出厂配置文件目录。
    QByteArray pathBytes = path.toLocal8Bit();
    CF_SetFactoryFilePath(pathBytes.constData());
}

void ConfocalSensorWorker::setIntParam(const QString &paramName, int channelIndex, int value, const QString &successMessage)
{
    if(!hasHandle())
        return;

    // 参数名来自 SDK 宏定义，转为 Latin1 后传入 C 接口。
    QByteArray paramNameBytes = paramName.toLatin1();
    StatusTypeDef ret = CF_SetIntParam(m_handle, paramNameBytes.constData(), channelIndex, value);
    if(ret == Status_Succeed)
    {
        if(!successMessage.isEmpty())
            emit logMessage(successMessage);
    }
    else
    {
        emit operationFailed(paramName, static_cast<int>(ret));
    }
}

void ConfocalSensorWorker::setFloatParam(const QString &paramName, int channelIndex, float value, const QString &successMessage)
{
    if(!hasHandle())
        return;

    QByteArray paramNameBytes = paramName.toLatin1();
    StatusTypeDef ret = CF_SetFloatParam(m_handle, paramNameBytes.constData(), channelIndex, value);
    if(ret == Status_Succeed)
    {
        if(!successMessage.isEmpty())
            emit logMessage(successMessage);
    }
    else
    {
        emit operationFailed(paramName, static_cast<int>(ret));
    }
}

void ConfocalSensorWorker::darkSignal(int channelIndex)
{
    if(!hasHandle())
        return;

    // 暗信号校正用于扣除环境/背景信号。
    StatusTypeDef ret = CF_DarkSignal(m_handle, channelIndex, true);
    if(ret == Status_Succeed)
        emit logMessage(tr("Single channel Dark successfully!"));
    else
        emit operationFailed(QStringLiteral("Dark"), static_cast<int>(ret));
}

void ConfocalSensorWorker::zero(int channelIndex)
{
    if(!hasHandle())
        return;

    // 清零当前通道测量值，通常用于建立相对测量基准。
    StatusTypeDef ret = CF_Zero(m_handle, channelIndex);
    if(ret == Status_Succeed)
        emit logMessage(tr("Zero successfully!"));
    else
        emit operationFailed(QStringLiteral("Zero"), static_cast<int>(ret));
}

void ConfocalSensorWorker::requestLatestResult(int channelIndex)
{
    if(!hasHandle())
        return;

    int dataLength = 0;
    QVector<SC_ResultDataTypeDef_t> singleChannelLastResult;
    singleChannelLastResult.resize(4);

    // 主动读取最近一次结果，适合未等待实时回调时刷新单个通道显示。
    StatusTypeDef ret = CF_GetLatestResult(m_handle, singleChannelLastResult.data(), &dataLength);
    if(ret != Status_Succeed)
    {
        emit operationFailed(QStringLiteral("GetLatestResult"), static_cast<int>(ret));
        return;
    }

    for(int i = 0; i < dataLength && i < singleChannelLastResult.size(); ++i)
    {
        if(singleChannelLastResult[i].channelIndex == channelIndex)
        {
            emit latestResultReady(channelIndex, singleChannelLastResult[i].result[0]);
            return;
        }
    }

    // 某些情况下 SDK 返回的数据顺序可能和通道号一致但 channelIndex 未匹配，做一次兜底读取。
    if(channelIndex >= 0 && channelIndex < singleChannelLastResult.size())
        emit latestResultReady(channelIndex, singleChannelLastResult[channelIndex].result[0]);
}

void ConfocalSensorWorker::sdkEventCallback(DeviceHandle_t handler, EventCallbackArgs_t arg, void *userPara)
{
    // SDK 的 C 风格回调无法直接调用成员函数，因此通过 userPara 传回 this 指针。
    ConfocalSensorWorker *worker = static_cast<ConfocalSensorWorker *>(userPara);
    if(worker)
        worker->handleSdkEvent(handler, arg);
}

void ConfocalSensorWorker::handleSdkEvent(DeviceHandle_t handler, const EventCallbackArgs_t &arg)
{
    Q_UNUSED(handler)

    // 这里只关心 SDK 上报的数据接收事件，其他事件直接忽略。
    if(arg.eventType != EventType_DataRecv)
        return;

    if(arg.rid == RID_RESULT)
    {
        if(!m_started.load(std::memory_order_acquire) || arg.data == nullptr)
            return;

        SC_ResultDataTypeDef_t *resultData = static_cast<SC_ResultDataTypeDef_t *>(arg.data);
        const int resultCount = std::min(arg.dataLen, 4);

        for(int i = 0; i < resultCount; ++i)
        {
            const int index = resultData[i].channelIndex;
            if(index < 0 || index >= 4)
                continue;

            QVector<int> signalData;
            // SDK 将信号数组地址放在整数字段中返回，这里按 SDK 约定转换回指针。
            int *signalDataPtr = reinterpret_cast<int *>(static_cast<uintptr_t>(resultData[i].signal));
            if(signalDataPtr)
            {
                // 信号曲线最大按 1440 点处理，防止异常长度导致拷贝越界或内存过大。
                const int signalLength = resultData[i].signalLength > 0
                                             ? std::min(resultData[i].signalLength, 1440)
                                             : 1440;
                signalData.resize(signalLength);
                std::memcpy(signalData.data(), signalDataPtr, signalLength * sizeof(int));
            }

            emit channelResultReady(index, resultData[i].result[0], resultData[i].saturation, signalData);
        }
    }
    else if(arg.rid == RID_DEVICE_DISCONNECT)
    {
        // 设备断线提示会传回 UI，由主窗口负责停止采样和提示用户。
        const char *errorText = static_cast<const char *>(arg.data);
        emit deviceDisconnected(decodeCfSdkText(errorText));
    }
    else if(arg.rid == RID_API_CALL_EXCEPTION)
    {
        // SDK 内部 API 异常同样转成 Qt 信号，避免在回调里直接操作界面。
        const char *errorText = static_cast<const char *>(arg.data);
        emit apiException(decodeCfSdkText(errorText));
    }
}
