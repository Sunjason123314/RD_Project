#include "dlpworker.h"

#include <QMetaType>
#include <exception>

DlpWorker::DlpWorker(QObject *parent)
    : QObject(parent)
{
    // 注册图像集合类型，保证 queued connection 可以把条纹图案任务投递到 DLP 子线程。
    qRegisterMetaType<DlpImageSet>("DlpImageSet");
}

DlpWorker::~DlpWorker()
{
    // worker 在线程结束时 deleteLater，此处兜底释放硬件连接。
    shutdown();
}

void DlpWorker::initialize()
{
    if(m_dlp)
    {
        emit logMessage(tr("DLP projector has already been initialized."));
        return;
    }

    try
    {
        // DLP 构造函数会初始化 I2C/命令层，必须在 DLP 子线程中创建。
        // 部分投影仪板卡只能稳定接收写命令，读状态命令会 NAK/timeout，因此连接阶段不再强制读状态。
        m_dlp = std::make_unique<DLP>();

        QStringList operatingModeNames;
        QString currentOperatingModeName;
        for(const auto& item : m_dlp->OperatingModeList)
        {
            const QString name = QString::fromStdString(item.first);
            operatingModeNames << name;
            if(m_dlp->Current_OperatingMode == item.second)
                currentOperatingModeName = name;
        }

        QStringList patternControlNames;
        for(const auto& item : m_dlp->PatternControlList)
            patternControlNames << QString::fromStdString(item.first);

        emit initialized(operatingModeNames,
                         currentOperatingModeName,
                         patternControlNames,
                         static_cast<int>(m_dlp->MaxBlueLedCurrent),
                         static_cast<int>(m_dlp->BlueLedCurrent));
        emit logMessage(tr("DLP projector initialized in background thread."));
    }
    catch(const std::exception& e)
    {
        m_dlp.reset();
        emit operationFailed(tr("Initialize DLP"), QString::fromLocal8Bit(e.what()));
    }
    catch(...)
    {
        m_dlp.reset();
        emit operationFailed(tr("Initialize DLP"), tr("Unknown exception."));
    }
}

void DlpWorker::shutdown()
{
    if(!m_dlp)
        return;

    // reset 会调用 DLP 析构函数并释放 I2C 总线访问权，必须在 DLP 子线程完成。
    m_dlp.reset();
    emit logMessage(tr("DLP projector connection released."));
}

void DlpWorker::selectOperatingMode(const QString& operatingModeName)
{
    if(!ensureInitialized(tr("Select DLP operating mode")))
        return;

    const std::string key = operatingModeName.toStdString();
    const auto iter = m_dlp->OperatingModeList.find(key);
    if(iter == m_dlp->OperatingModeList.end())
    {
        emit operationFailed(tr("Select DLP operating mode"), tr("Unknown operating mode: %1").arg(operatingModeName));
        return;
    }

    if(!m_dlp->SelectOperatingMode(iter->second))
    {
        emit operationFailed(tr("Select DLP operating mode"),
                             tr("DLP command write failed. Please check projector connection."));
        return;
    }

    emit logMessage(tr("DLP operating mode selected: %1").arg(operatingModeName));
}

void DlpWorker::internalPatternControl(const QString& patternControlName, int times)
{
    if(!ensureInitialized(tr("Control DLP internal pattern")))
        return;

    const std::string key = patternControlName.toStdString();
    const auto iter = m_dlp->PatternControlList.find(key);
    if(iter == m_dlp->PatternControlList.end())
    {
        emit operationFailed(tr("Control DLP internal pattern"), tr("Unknown pattern control: %1").arg(patternControlName));
        return;
    }

    if(!m_dlp->InternalPatternControl(iter->second, times))
    {
        emit operationFailed(tr("Control DLP internal pattern"),
                             tr("DLP command write failed. Please check projector connection."));
        return;
    }

    emit logMessage(tr("DLP internal pattern command sent: %1").arg(patternControlName));
}

void DlpWorker::startInternalPatternProjection(int times)
{
    if(!ensureInitialized(tr("Start DLP projection")))
        return;

    // PC_START 只在内部图案模式下有效，因此先强制切到 SENS_INTERNAL_PATTERN，
    // 再启动图案序列。这样上层点一个按钮即可可靠开始投影，无需关心操作模式的先后顺序。
    if(!m_dlp->SelectOperatingMode(DLPC347X_OM_SENS_INTERNAL_PATTERN))
    {
        emit operationFailed(tr("Start DLP projection"),
                             tr("Failed to switch to internal pattern mode. Please check projector connection."));
        return;
    }

    // times=1 只播放一遍就停；times<=0 底层按 0xFF(无限循环)处理，持续投影直到 STOP。
    if(!m_dlp->InternalPatternControl(DLPC347X_PC_START, times))
    {
        emit operationFailed(tr("Start DLP projection"),
                             tr("DLP command write failed. Please check projector connection."));
        return;
    }

    emit logMessage(times == 1 ? tr("DLP projection started (project once).")
                               : tr("DLP projection started (continuous)."));
}

void DlpWorker::stopInternalPatternProjection()
{
    if(!ensureInitialized(tr("Stop DLP projection")))
        return;

    if(!m_dlp->InternalPatternControl(DLPC347X_PC_STOP, 0))
    {
        emit operationFailed(tr("Stop DLP projection"),
                             tr("DLP command write failed. Please check projector connection."));
        return;
    }

    emit logMessage(tr("DLP projection stopped (PC_STOP)."));
}

void DlpWorker::displayGridLines()
{
    if(!ensureInitialized(tr("Display DLP grid lines")))
        return;

    if(!m_dlp->DisplayGridLines())
    {
        emit operationFailed(tr("Display DLP grid lines"),
                             tr("DLP command write failed. Please check projector connection."));
        return;
    }

    emit logMessage(tr("DLP grid line display command sent."));
}

void DlpWorker::setBrightness(int brightness)
{
    if(!ensureInitialized(tr("Set DLP brightness")))
        return;

    if(!m_dlp->SetBrightness(brightness))
    {
        emit operationFailed(tr("Set DLP brightness"),
                             tr("DLP command write failed. Please check projector connection."));
        return;
    }

    emit brightnessChanged(static_cast<int>(m_dlp->BlueLedCurrent));
}

void DlpWorker::input1DImageSet8bitV(const DlpImageSet& imageSets)
{
    if(!ensureInitialized(tr("Program DLP 8bit pattern set")))
        return;

    // cv::Mat 是隐式共享对象，queued connection 会拷贝 Mat 头；该调用在 DLP 子线程中完成烧录流程。
    if(!m_dlp->Input1DImageSet_8bit_V(imageSets))
    {
        emit operationFailed(tr("Program DLP 8bit pattern set"),
                             tr("DLP command write failed. Please check projector connection and image set."));
        return;
    }

    emit logMessage(tr("DLP 8bit pattern set programming finished."));
}

bool DlpWorker::ensureInitialized(const QString& operation)
{
    if(m_dlp)
        return true;

    emit operationFailed(operation, tr("DLP projector is not initialized. Please connect it first."));
    return false;
}
