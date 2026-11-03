#ifndef CONFOCALSENSORWORKER_H
#define CONFOCALSENSORWORKER_H

#include <QObject>
#include <QMetaType>
#include <QString>
#include <QVector>
#include <atomic>

#include "CF_UserInterface.h"
#include "CF_ParameterDefine.h"

/**
 * @brief 光谱共焦传感器运行状态快照。
 *
 * 该结构体用于把 SDK/控制器/传感器的关键状态一次性传回 UI 线程，
 * 避免界面线程直接访问耗时或阻塞的 SDK 接口。
 */
struct ConfocalStatusSnapshot
{
    QString sdkVersion;        ///< SDK 版本号。
    QString controlerVersion;  ///< 控制器固件版本号。
    QString controlerSN;       ///< 控制器序列号。
    QString sensorSN;          ///< 传感器序列号。
    int triggerMode = -1;      ///< 当前触发模式。
    int curFrameRate = 0;      ///< 当前实时帧率。
};

/**
 * @brief 光谱共焦传感器初始化参数集合。
 *
 * MainWindow 在连接设备后通过 worker 读取这些参数，并据此刷新界面控件状态。
 * 字段按界面功能区划分：单通道测量、曝光/光源、通信、信号处理、触发与编码器。
 */
struct ConfocalInitialConfig
{
    bool isCF3000Lite = false; ///< 是否为 CF3000Lite 设备，用于决定部分功能是否可用。

    // 单通道测量设置。
    int singleMeasureMode = -1;
    int singleAbsMode = 0;
    int singleReverse = 0;
    int singleMultDistance = 0;

    // 曝光与光源设置。
    int exposureTime = 0;
    int autoExposureEnable = 0;
    int autoExposureChannel = 0;
    int autoExposureTime = 0;
    int lightMode = 0;
    int autoLightReference = 0;
    float lightValue = 0.0f;
    int subBackgroundLight = 0;

    // 通信设置，CF3000Lite 不开放这些串口通信参数。
    bool communicationEnabled = true;
    int communicationMode = 0;
    int serialPort = 0;
    int dataFormat = 0;
    int baudRate = 0;
    int verify = 0;

    // 信号选择、排序和滤波设置。
    int exportSignalSelect = 0;
    int signalSort = 0;
    float signalThreshold = 0.0f;
    float signalRatio = 0.0f;
    int signalSensitivity = 0;
    int signalCount = 0;
    int signalMinNum = 0;
    int signalSmoothLength = 0;
    int signalMedianFilter = 0;
    int signalAverageFilter = 0;

    // 触发与编码器设置。
    int triggerMode = 0;
    bool frameSettingEnabled = false;
    bool cf3000LiteFilePathEnabled = false;
    int encodeWorkMode = 0;
    int encodeInputMode = 0;
    int encodeDivision = 0;
};

// 允许 ConfocalStatusSnapshot / ConfocalInitialConfig 通过 Qt queued connection 跨线程传递。
// MainWindow 初始化光谱共焦线程时会 qRegisterMetaType，之后这些快照才能安全跨线程发送。
Q_DECLARE_METATYPE(ConfocalStatusSnapshot)
Q_DECLARE_METATYPE(ConfocalInitialConfig)

/**
 * @brief 光谱共焦传感器后台工作对象。
 *
 * 该类只封装和 SDK 直接交互的操作，通常会被移动到 ThreadManager 创建的工作线程中。
 * UI 线程通过信号槽或 QMetaObject::invokeMethod 调用这些槽函数，从而避免界面卡顿。
 */
class ConfocalSensorWorker : public QObject
{
    Q_OBJECT

public:
    explicit ConfocalSensorWorker(QObject *parent = nullptr);
    ~ConfocalSensorWorker() override;

    /**
     * @brief SDK 事件回调入口。
     * @note 这是静态函数，SDK 调用后会通过 userPara 找回对应的 worker 实例。
     */
    static void sdkEventCallback(DeviceHandle_t handler, EventCallbackArgs_t arg, void *userPara);

public slots:
    // 以下槽函数是光谱共焦 worker 的线程入口。
    // MainWindow 通过 QMetaObject::invokeMethod/信号槽排队调用，避免 UI 线程直接访问 SDK。
    void setDeviceHandle(int handle, bool isCF3000Lite); ///< 保存登录窗口返回的设备句柄与设备型号。
    void autoConnectCF4000();                            ///< 扫描并连接第一台可用的 CF4000。
    void verifyConnection();                             ///< 检查设备连接状态，并在成功后注册 SDK 回调。
    void requestInitialConfig(int singleChannelIndex, int lightChannelIndex, int filterAndSignalChannelIndex); ///< 读取初始化配置。
    void requestStatusSnapshot();                        ///< 读取 SDK、控制器、传感器状态快照。
    void startSample(int singleChannelIndex);            ///< 开始采样并打开指定通道的信号数据输出。
    void stopSample(int singleChannelIndex);             ///< 停止采样并关闭指定通道的信号数据输出。
    void closeDevice();                                  ///< 停止采样并关闭设备句柄。
    void setFactoryFilePath(const QString &path);        ///< 设置 CF3000Lite 出厂配置文件路径。
    void setIntParam(const QString &paramName, int channelIndex, int value, const QString &successMessage); ///< 设置整型 SDK 参数。
    void setFloatParam(const QString &paramName, int channelIndex, float value, const QString &successMessage); ///< 设置浮点型 SDK 参数。
    void darkSignal(int channelIndex);                   ///< 对指定通道执行暗信号校正。
    void zero(int channelIndex);                         ///< 对指定通道执行清零操作。
    void requestLatestResult(int channelIndex);          ///< 主动读取指定通道最近一次测量结果。

signals:
    // worker 不直接操作界面，所有状态、日志、测量值都通过信号返回 GUI 线程处理。
    void connectionChecked(bool connected, int connectStatus); ///< 连接检查完成。
    void autoConnectionOpened(int handle); ///< 自动打开 CF4000 后返回设备句柄。
    void autoConnectionAttemptFinished(bool connected); ///< 一次自动连接尝试完成。
    void initialConfigReady(const ConfocalInitialConfig &config); ///< 初始化参数读取完成。
    void statusSnapshotReady(const ConfocalStatusSnapshot &snapshot); ///< 状态快照读取完成。
    void sampleStateChanged(bool started, int status); ///< 采样启停状态变化。
    void darkSignalFinished(bool succeeded, int channelIndex); ///< 暗信号校正完成。
    void deviceClosed(); ///< 设备已关闭。
    void logMessage(const QString &message); ///< 需要显示到 UI 的普通日志。
    void operationFailed(const QString &operation, int status); ///< SDK 操作失败。
    void latestResultReady(int channelIndex, float value); ///< 最近一次测量结果读取完成。
    void channelResultReady(int channelIndex, float result, float saturation, const QVector<int> &signalData); ///< SDK 回调返回通道数据。
    void deviceDisconnected(const QString &message); ///< SDK 上报设备断开。
    void apiException(const QString &message); ///< SDK 上报 API 调用异常。

private:
    bool hasHandle() const; ///< 当前 worker 是否持有有效设备句柄。
    void handleSdkEvent(DeviceHandle_t handler, const EventCallbackArgs_t &arg); ///< 处理 SDK 回调事件。

    // 以下成员只在光谱共焦 worker 内部维护；界面线程通过信号结果读取状态，不直接访问。
    DeviceHandle_t m_handle = 0;               ///< SDK 设备句柄。
    bool m_hasHandle = false;                  ///< 是否已保存有效句柄。
    bool m_isCF3000Lite = false;               ///< 当前设备是否为 CF3000Lite。
    bool m_autoConnectionAttempt = false;      ///< 当前连接校验是否属于自动连接。
    int m_lastSingleChannelIndex = 0;          ///< 最近一次启停采样使用的通道索引。
    std::atomic_bool m_started{false};         ///< 采样状态标志，回调线程和 worker 线程都会读取。
};

#endif // CONFOCALSENSORWORKER_H
