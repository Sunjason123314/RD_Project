#include "camworker.h"

#include <QMetaType>

CamWorker::CamWorker(QObject *parent)
    : QObject(parent)
{
    // 状态快照和图像回调对象指针需要跨线程传递，必须注册到 Qt 元对象系统。
    qRegisterMetaType<CamDeviceState>("CamDeviceState");
    qRegisterMetaType<CGXBitmap*>("CGXBitmap*");
}

CamWorker::~CamWorker()
{
    // worker 在线程结束时 deleteLater，此处兜底关闭相机 SDK 句柄。
    shutdown();
}

void CamWorker::openCamera(int index)
{
    if(!m_camera)
        m_camera = std::make_unique<CAM>();

    if(m_camera->isOpen())
    {
        emit operationFailed(tr("Open camera"), tr("Camera is already open."));
        return;
    }

    // CAM::Open_Cam 内部只记录错误，不再弹窗；这里统一通过信号返回 GUI。
    m_camera->Open_Cam(index);
    if(!m_camera->isOpen())
    {
        emitLastError(tr("Open camera"), tr("Failed to open camera."));
        return;
    }

    CamDeviceState state = collectState();
    emit cameraOpened(state);
    emit stateReady(state);
    emit logMessage(tr("Camera opened in background thread."));
}

void CamWorker::closeCamera()
{
    if(!m_camera)
    {
        emit cameraClosed();
        return;
    }

    m_camera->Close_Cam();
    emit cameraClosed();
    emit logMessage(tr("Camera connection released."));
}

void CamWorker::shutdown()
{
    if(!m_camera)
        return;

    // reset 会调用 CAM 析构前先显式关闭设备，保证 SDK 句柄在相机子线程释放。
    closeCamera();
    m_camera.reset();
}

void CamWorker::startSnap()
{
    if(!ensureOpen(tr("Start camera acquisition")))
        return;

    m_camera->StartSnap();
    if(!m_camera->lastError().isEmpty())
    {
        emitLastError(tr("Start camera acquisition"));
        return;
    }

    emit snapStateChanged(true);
    emit stateReady(collectState());
}

void CamWorker::stopSnap()
{
    if(!ensureOpen(tr("Stop camera acquisition")))
        return;

    m_camera->StopSnap();
    if(!m_camera->lastError().isEmpty())
    {
        emitLastError(tr("Stop camera acquisition"));
        return;
    }

    emit snapStateChanged(false);
    emit stateReady(collectState());
}

void CamWorker::softwareTrigger()
{
    if(!ensureOpen(tr("Software trigger")))
        return;

    m_camera->SoftwareTrigger();
    if(!m_camera->lastError().isEmpty())
        emitLastError(tr("Software trigger"));
}

void CamWorker::setCurrentTriggerMode(int index)
{
    if(!ensureOpen(tr("Set trigger mode")))
        return;

    m_camera->SetCurrentTriggerMode(index);
    if(!m_camera->lastError().isEmpty())
    {
        emitLastError(tr("Set trigger mode"));
        return;
    }

    emit stateReady(collectState());
}

void CamWorker::setCurrentTriggerSource(int index)
{
    if(!ensureOpen(tr("Set trigger source")))
        return;

    m_camera->SetCurrentTriggerSource(index);
    if(!m_camera->lastError().isEmpty())
    {
        emitLastError(tr("Set trigger source"));
        return;
    }

    emit stateReady(collectState());
}

void CamWorker::setCurrentTriggerActive(int index)
{
    if(!ensureOpen(tr("Set trigger active")))
        return;

    m_camera->SetCurrentTriggerActive(index);
    if(!m_camera->lastError().isEmpty())
    {
        emitLastError(tr("Set trigger active"));
        return;
    }

    emit stateReady(collectState());
}

void CamWorker::setShutterValue(double value)
{
    if(!ensureOpen(tr("Set exposure time")))
        return;

    if(!m_camera->SetShutterValue(value))
    {
        emitLastError(tr("Set exposure time"), tr("Invalid exposure time."));
        emit stateReady(collectState());
        return;
    }

    emit stateReady(collectState());
}

void CamWorker::setGain(double value)
{
    if(!ensureOpen(tr("Set gain")))
        return;

    if(!m_camera->SetGain(value))
    {
        emitLastError(tr("Set gain"), tr("Invalid gain."));
        emit stateReady(collectState());
        return;
    }

    emit stateReady(collectState());
}

void CamWorker::setCurrentFrameRateMode(int index)
{
    if(!ensureOpen(tr("Set frame rate mode")))
        return;

    m_camera->SetCurrentFrameRateMode(index);
    if(!m_camera->lastError().isEmpty())
    {
        emitLastError(tr("Set frame rate mode"));
        return;
    }

    emit stateReady(collectState());
}

void CamWorker::setFrameRate(double value)
{
    if(!ensureOpen(tr("Set frame rate")))
        return;

    if(!m_camera->SetFrameRate(value))
    {
        emitLastError(tr("Set frame rate"), tr("Invalid frame rate."));
        emit stateReady(collectState());
        return;
    }

    emit stateReady(collectState());
}

void CamWorker::setCurrentDeNoiseMode(int index)
{
    if(!ensureOpen(tr("Set denoise mode")))
        return;

    m_camera->SetCurrentDeNoiseMode(index);
    if(!m_camera->lastError().isEmpty())
    {
        emitLastError(tr("Set denoise mode"));
        return;
    }

    emit stateReady(collectState());
}

void CamWorker::setDeNoiseValue(double value)
{
    if(!ensureOpen(tr("Set denoise value")))
        return;

    if(!m_camera->SetDeNoiseValue(value))
    {
        emitLastError(tr("Set denoise value"), tr("Invalid denoise value."));
        emit stateReady(collectState());
        return;
    }

    emit stateReady(collectState());
}

void CamWorker::setSaveEnabled(bool enabled)
{
    if(!ensureBitmap(tr("Set continuous save")))
        return;

    m_camera->m_pBitmap->SetSave(enabled);
}

void CamWorker::setCountSaveEnabled(bool enabled)
{
    if(!ensureBitmap(tr("Set count save")))
        return;

    m_camera->m_pBitmap->SetCount(enabled);
}

void CamWorker::saveOnce()
{
    if(!ensureBitmap(tr("Save one image")))
        return;

    m_camera->m_pBitmap->SaveOnce();
}

void CamWorker::setSavePath(const QString& path)
{
    if(!ensureBitmap(tr("Set save path")))
        return;

    m_camera->m_pBitmap->SetSavePath(path.toStdString());
    const QString savePath = QString::fromStdString(m_camera->m_pBitmap->GetSavepath());
    emit savePathChanged(savePath);
    emit stateReady(collectState());
}

bool CamWorker::ensureOpen(const QString& operation)
{
    if(m_camera && m_camera->isOpen())
        return true;

    emit operationFailed(operation, tr("Camera is not open."));
    return false;
}

bool CamWorker::ensureBitmap(const QString& operation)
{
    if(ensureOpen(operation) && m_camera->m_pBitmap)
        return true;

    emit operationFailed(operation, tr("Camera image callback object is not ready."));
    return false;
}

void CamWorker::emitLastError(const QString& operation, const QString& fallbackReason)
{
    QString reason = fallbackReason;
    if(m_camera && !m_camera->lastError().isEmpty())
        reason = m_camera->lastError();
    if(reason.isEmpty())
        reason = tr("Unknown camera SDK error.");

    emit operationFailed(operation, reason);
}

CamDeviceState CamWorker::collectState() const
{
    CamDeviceState state;
    if(!m_camera)
        return state;

    state.isOpen = m_camera->isOpen();
    state.isSnap = m_camera->isSnap();
    state.isColor = m_camera->isColor();
    state.deviceSN = QString::fromStdString(m_camera->GetDeviceSN());

    state.triggerModeList = m_camera->TriggerModeList;
    state.currentTriggerMode = m_camera->GetCurrentTriggerMode();
    state.triggerSourceList = m_camera->TriggerSourceList;
    state.currentTriggerSource = m_camera->GetCurrentTriggerSource();
    state.triggerActiveList = m_camera->TriggerActiveList;
    state.currentTriggerActive = m_camera->GetCurrentTriggerActive();
    state.frameRateModeList = m_camera->FrameRateModeList;
    state.currentFrameRateMode = m_camera->GetCurrentFrameRateMode();
    state.deNoiseModeList = m_camera->DeNoiseModeList;
    state.currentDeNoiseMode = m_camera->GetCurrentDeNoiseMode();

    if(state.isOpen)
    {
        state.shutterValue = m_camera->GetShutterValue();
        m_camera->GetShutterValueMinMax(state.shutterMin, state.shutterMax);
        state.gainValue = m_camera->GetGain();
        m_camera->GetGainMinMax(state.gainMin, state.gainMax);
        state.frameRateValue = m_camera->GetFrameRate();
        m_camera->GetFrameRateMinMax(state.frameRateMin, state.frameRateMax);
        state.deNoiseValue = m_camera->GetDeNoiseValue();
        m_camera->GetDeNoiseMinMax(state.deNoiseMin, state.deNoiseMax);
    }

    if(m_camera->m_pBitmap)
    {
        state.imageSize = m_camera->m_pBitmap->GetPicSize();
        state.savePath = QString::fromStdString(m_camera->m_pBitmap->GetSavepath());
        state.bitmap = m_camera->m_pBitmap;
    }

    return state;
}
