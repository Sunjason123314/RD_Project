#ifndef DLPWORKER_H
#define DLPWORKER_H

#include "dlp.h"

#include <QObject>
#include <QString>
#include <QStringList>

#include <memory>
#include <vector>

// DLP 条纹图像集合跨线程传递类型：外层为图案组，内层为该组中的多张 8bit 图像。
using DlpImageSet = std::vector<std::vector<cv::Mat>>;
Q_DECLARE_METATYPE(DlpImageSet)

/**
 * @brief DLP 投影仪后台 worker。
 *
 * DLP 原始类不是 QObject，且构造函数中会连接硬件。该 worker 负责在 DLP 子线程中
 * 创建和持有 DLP 实例，主线程只通过 queued connection 投递控制命令，避免界面卡顿。
 */
class DlpWorker : public QObject
{
    Q_OBJECT

public:
    explicit DlpWorker(QObject *parent = nullptr);
    ~DlpWorker() override;

public slots:
    // 在 DLP 子线程中连接投影仪并读取初始状态。
    void initialize();

    // 停止线程前释放 DLP 硬件连接，由 ThreadManager 在 DLP 子线程中调用。
    void shutdown();

    // 以下槽函数均在 DLP 子线程中执行真实硬件控制。
    void selectOperatingMode(const QString& operatingModeName);
    void internalPatternControl(const QString& patternControlName, int times);
    void displayGridLines();
    void setBrightness(int brightness);
    void input1DImageSet8bitV(const DlpImageSet& imageSets);

signals:
    // DLP 连接成功后把下拉框和亮度滑块所需状态返回 GUI 线程。
    void initialized(const QStringList& operatingModeNames,
                     const QString& currentOperatingModeName,
                     const QStringList& patternControlNames,
                     int maxBlueLedCurrent,
                     int blueLedCurrent);

    // 亮度设置完成后返回实际蓝光电流，GUI 线程据此刷新滑块和标签。
    void brightnessChanged(int blueLedCurrent);

    void logMessage(const QString& message);
    void operationFailed(const QString& operation, const QString& reason);

private:
    bool ensureInitialized(const QString& operation);

    std::unique_ptr<DLP> m_dlp; // 仅在 DLP 子线程中访问的真实硬件对象。
};

#endif // DLPWORKER_H
