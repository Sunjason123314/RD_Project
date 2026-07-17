#ifndef CAMWORKER_H
#define CAMWORKER_H

#include "CAM/cam.h"

#include <QObject>
#include <QSize>
#include <QString>
#include <QStringList>

#include <memory>

/**
 * @brief 相机状态快照。
 *
 * CAM 真实对象运行在相机子线程，GUI 线程不能直接读取 CAM 成员。worker 每次打开相机
 * 或修改参数后，把界面需要显示的数据打包成该结构，通过 queued connection 返回主线程。
 */
struct CamDeviceState
{
    bool isOpen = false;                 ///< 相机是否已打开。
    bool isSnap = false;                 ///< 相机是否正在采集。
    bool isColor = false;                ///< 是否为彩色相机。
    QString deviceSN;                    ///< 当前设备序列号。

    QStringList triggerModeList;         ///< 触发模式列表。
    int currentTriggerMode = 0;          ///< 当前触发模式索引。
    QStringList triggerSourceList;       ///< 触发源列表。
    int currentTriggerSource = 0;        ///< 当前触发源索引。
    QStringList triggerActiveList;       ///< 触发极性列表。
    int currentTriggerActive = 0;        ///< 当前触发极性索引。
    QStringList frameRateModeList;       ///< 帧率模式列表。
    int currentFrameRateMode = 0;        ///< 当前帧率模式索引。
    QStringList deNoiseModeList;         ///< 降噪模式列表。
    int currentDeNoiseMode = 0;          ///< 当前降噪模式索引。

    double shutterValue = 0.0;           ///< 当前曝光时间。
    double shutterMin = 0.0;             ///< 曝光时间下限。
    double shutterMax = 0.0;             ///< 曝光时间上限。
    double gainValue = 0.0;              ///< 当前增益。
    double gainMin = 0.0;                ///< 增益下限。
    double gainMax = 0.0;                ///< 增益上限。
    double frameRateValue = 0.0;         ///< 当前帧率。
    double frameRateMin = 0.0;           ///< 帧率下限。
    double frameRateMax = 0.0;           ///< 帧率上限。
    double deNoiseValue = 0.0;           ///< 当前降噪值。
    double deNoiseMin = 0.0;             ///< 降噪下限。
    double deNoiseMax = 0.0;             ///< 降噪上限。

    QSize imageSize;                     ///< 采集图像尺寸。
    QString savePath;                    ///< 当前保存路径。
    CGXBitmap *bitmap = nullptr;         ///< 图像回调对象，只用于在 GUI 线程建立信号连接。
};
Q_DECLARE_METATYPE(CamDeviceState)
Q_DECLARE_METATYPE(CGXBitmap*)

/**
 * @brief 工业相机后台 worker。
 *
 * CAM 原始类不是 QObject，且包含相机 SDK 句柄。该 worker 在相机子线程中创建并持有 CAM，
 * 主界面/相机窗口只通过排队信号槽投递控制命令，避免 GUI 线程被 SDK 打开、采集、参数设置阻塞。
 */
class CamWorker : public QObject
{
    Q_OBJECT

public:
    explicit CamWorker(QObject *parent = nullptr);
    ~CamWorker() override;

public slots:
    // 在相机子线程中打开指定索引的工业相机。
    void openCamera(int index);

    // 停止采集并关闭相机，ThreadManager 停止线程前也会调用该槽。
    void closeCamera();
    void shutdown();

    // 以下槽函数均在相机子线程中执行真实 SDK 操作。
    void startSnap();
    void stopSnap();
    void softwareTrigger();
    void setCurrentTriggerMode(int index);
    void setCurrentTriggerSource(int index);
    void setCurrentTriggerActive(int index);
    void setShutterValue(double value);
    void setGain(double value);
    void setCurrentFrameRateMode(int index);
    void setFrameRate(double value);
    void setCurrentDeNoiseMode(int index);
    void setDeNoiseValue(double value);
    void setSaveEnabled(bool enabled);
    void setCountSaveEnabled(bool enabled);
    void saveOnce();
    void setSavePath(const QString& path);

signals:
    void cameraOpened(const CamDeviceState& state);                 ///< 相机打开成功并返回初始状态。
    void cameraClosed();                                            ///< 相机已关闭。
    void stateReady(const CamDeviceState& state);                   ///< 参数或状态刷新。
    void snapStateChanged(bool started);                            ///< 采集启停状态变化。
    void savePathChanged(const QString& path);                      ///< 保存路径变化。
    void logMessage(const QString& message);                        ///< 普通日志。
    void operationFailed(const QString& operation, const QString& reason); ///< SDK 操作失败。

private:
    bool ensureOpen(const QString& operation);
    bool ensureBitmap(const QString& operation);
    void emitLastError(const QString& operation, const QString& fallbackReason = QString());
    CamDeviceState collectState() const;

    std::unique_ptr<CAM> m_camera; ///< 仅在相机子线程中访问的真实相机对象。
};

#endif // CAMWORKER_H
