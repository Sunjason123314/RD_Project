#include "camdisplay.h"
#include "ui_camdisplay.h"

#include <QMessageBox>
#include <QMetaObject>
#include <QSignalBlocker>

using namespace std;

CamDisplay::CamDisplay(QWidget *parent, int index, ThreadManager *threadManager, int instanceId) :
    QMainWindow(parent),
    ui(new Ui::CamDisplay),
    m_threadManager(threadManager),
    camindex(index),
    m_instanceId(instanceId == 0 ? index + 1 : instanceId)
{
    ui->setupUi(this);

    // 只允许输入数字，防止明显非法参数进入相机 worker。
    ui->lineEdit_exposetime->setValidator(new QIntValidator(0, 9999, ui->lineEdit_exposetime));
    ui->lineEdit_framerate->setValidator(new QIntValidator(0, 9999, ui->lineEdit_framerate));
    ui->lineEdit_denoise->setValidator(new QIntValidator(0, 9999, ui->lineEdit_denoise));

    initCameraThread();
}

CamDisplay::~CamDisplay()
{
    shutdownCameraThread();
    delete ui;
}

void CamDisplay::initCameraThread()
{
    if(!m_threadManager)
    {
        QMessageBox::warning(this, QString::fromLocal8Bit("警告"), QString::fromLocal8Bit("相机线程管理未初始化"));
        return;
    }

    if(m_cameraWorker)
        return;

    qRegisterMetaType<CamDeviceState>("CamDeviceState");
    qRegisterMetaType<CGXBitmap*>("CGXBitmap*");

    // CamWorker 不能设置父对象，否则 QObject 有父对象时无法 moveToThread。
    m_cameraWorker = new CamWorker;

    connect(m_cameraWorker, &CamWorker::cameraOpened,
            this, [this](const CamDeviceState& state) {
                applyCameraState(state);
                const QString defaultPath = QStringLiteral("C:/Users/10782/Desktop/calibrate/data/test_multifrequency_cuda%1").arg(camindex);
                QMetaObject::invokeMethod(m_cameraWorker,
                                          "setSavePath",
                                          Qt::QueuedConnection,
                                          Q_ARG(QString, defaultPath));
            }, Qt::QueuedConnection);

    connect(m_cameraWorker, &CamWorker::cameraClosed,
            this, [this]() {
                disconnectBitmapSignal();
                ui->openGLWidget->Clear();
                m_cameraOpen = false;
                m_cameraSnap = false;
                m_state = CamDeviceState();
                refreshUiFromState(m_state);
            }, Qt::QueuedConnection);

    connect(m_cameraWorker, &CamWorker::stateReady,
            this, &CamDisplay::applyCameraState,
            Qt::QueuedConnection);

    connect(m_cameraWorker, &CamWorker::snapStateChanged,
            this, [this](bool started) {
                m_cameraSnap = started;
            }, Qt::QueuedConnection);

    connect(m_cameraWorker, &CamWorker::savePathChanged,
            this, [this](const QString& path) {
                m_state.savePath = path;
                ui->label_savepath_display->setText(path);
                ui->label_savepath_display->setWordWrap(true);
            }, Qt::QueuedConnection);

    connect(m_cameraWorker, &CamWorker::operationFailed,
            this, [this](const QString& operation, const QString& reason) {
                QMessageBox::warning(this,
                                     QString::fromLocal8Bit("警告"),
                                     operation + QStringLiteral(": ") + reason);
            }, Qt::QueuedConnection);

    // 相机模块由 ThreadManager 统一 moveToThread/停止/释放；实例编号保证多个相机窗口互不冲突。
    m_threadStarted = m_threadManager->startModule(ThreadManager::Module::Camera,
                                                   m_cameraWorker,
                                                   QStringLiteral("CameraThread_%1").arg(m_instanceId),
                                                   "shutdown",
                                                   QThread::InheritPriority,
                                                   m_instanceId);
    if(!m_threadStarted)
    {
        m_cameraWorker = nullptr;
    }
}

void CamDisplay::shutdownCameraThread()
{
    disconnectBitmapSignal();
    if(ui && ui->openGLWidget)
        ui->openGLWidget->Clear();

    if(m_threadManager && m_threadStarted)
        m_threadManager->stopModule(ThreadManager::Module::Camera, 3000, m_instanceId);

    m_cameraWorker = nullptr;
    m_bitmap = nullptr;
    m_threadStarted = false;
    m_cameraOpen = false;
    m_cameraSnap = false;
}

void CamDisplay::disconnectBitmapSignal()
{
    if(m_bitmap && ui && ui->openGLWidget)
        disconnect(m_bitmap, &CGXBitmap::ShowCallback, ui->openGLWidget, &GL_Image::refresh);

    if(m_bitmap && LineDisplay)
        disconnect(m_bitmap, &CGXBitmap::ShowCallback, LineDisplay, &Form_linepix_display::picture_refreshed);

    m_bitmap = nullptr;
}

void CamDisplay::applyCameraState(const CamDeviceState& state)
{
    m_state = state;
    m_cameraOpen = state.isOpen;
    m_cameraSnap = state.isSnap;

    if(state.bitmap && state.bitmap != m_bitmap)
    {
        disconnectBitmapSignal();
        m_bitmap = state.bitmap;
        connect(m_bitmap, &CGXBitmap::ShowCallback,
                ui->openGLWidget, &GL_Image::refresh,
                Qt::QueuedConnection);
    }

    if(state.imageSize.isValid())
        ui->openGLWidget->setImageSize(state.imageSize);

    refreshUiFromState(state);
}

void CamDisplay::refreshUiFromState(const CamDeviceState& state)
{
    refresh = true;

    QSignalBlocker blockerTriggerMode(ui->comboBox_trigger_model);
    QSignalBlocker blockerTriggerSource(ui->comboBox_trigger_source);
    QSignalBlocker blockerTriggerActive(ui->comboBox_trigger_active);
    QSignalBlocker blockerFrameRateMode(ui->comboBox_framrate_mode);
    QSignalBlocker blockerDeNoiseMode(ui->comboBox_denoise);

    ui->comboBox_trigger_model->clear();
    ui->comboBox_trigger_model->addItems(state.triggerModeList);
    if(state.currentTriggerMode >= 0 && state.currentTriggerMode < ui->comboBox_trigger_model->count())
        ui->comboBox_trigger_model->setCurrentIndex(state.currentTriggerMode);

    ui->comboBox_trigger_source->clear();
    ui->comboBox_trigger_source->addItems(state.triggerSourceList);
    if(state.currentTriggerSource >= 0 && state.currentTriggerSource < ui->comboBox_trigger_source->count())
        ui->comboBox_trigger_source->setCurrentIndex(state.currentTriggerSource);

    ui->comboBox_trigger_active->clear();
    ui->comboBox_trigger_active->addItems(state.triggerActiveList);
    if(state.currentTriggerActive >= 0 && state.currentTriggerActive < ui->comboBox_trigger_active->count())
        ui->comboBox_trigger_active->setCurrentIndex(state.currentTriggerActive);

    ui->comboBox_framrate_mode->clear();
    ui->comboBox_framrate_mode->addItems(state.frameRateModeList);
    if(state.currentFrameRateMode >= 0 && state.currentFrameRateMode < ui->comboBox_framrate_mode->count())
        ui->comboBox_framrate_mode->setCurrentIndex(state.currentFrameRateMode);

    ui->comboBox_denoise->clear();
    ui->comboBox_denoise->addItems(state.deNoiseModeList);
    if(state.currentDeNoiseMode >= 0 && state.currentDeNoiseMode < ui->comboBox_denoise->count())
        ui->comboBox_denoise->setCurrentIndex(state.currentDeNoiseMode);

    if(state.isOpen)
    {
        ui->lineEdit_exposetime->setText(QString::number(state.shutterValue));
        ui->lineEdit_gain->setText(QString::number(state.gainValue));
        ui->lineEdit_framerate->setText(QString::number(state.frameRateValue));
        ui->lineEdit_denoise->setText(QString::number(state.deNoiseValue));

        ui->label_display_ShutterValue->setToolTip(QString::number(state.shutterMin) + QStringLiteral("~") + QString::number(state.shutterMax));
        ui->label_display_Gain->setToolTip(QString::number(state.gainMin) + QStringLiteral("~") + QString::number(state.gainMax));
        ui->label_display_FrameRate->setToolTip(QString::number(state.frameRateMin) + QStringLiteral("~") + QString::number(state.frameRateMax));
        ui->label_display_DeNoise->setToolTip(QString::number(state.deNoiseMin) + QStringLiteral("~") + QString::number(state.deNoiseMax));

        ui->label_savepath_display->setText(state.savePath);
        ui->label_savepath_display->setWordWrap(true);
    }

    refresh = false;
}

void CamDisplay::warnCameraNotOpen()
{
    QMessageBox::warning(this, QString::fromLocal8Bit("警告"), QString::fromLocal8Bit("相机未打开"));
}

void CamDisplay::warnCameraNotSnap()
{
    QMessageBox::warning(this, QString::fromLocal8Bit("警告"), QString::fromLocal8Bit("Camera doesn't collect"));
}

///
/// \brief CamDisplay::soft_trigger 软触发采集
///
void CamDisplay::soft_trigger()
{
    if(!m_cameraOpen)
    {
        warnCameraNotOpen();
        return;
    }
    if(!m_cameraSnap)
    {
        warnCameraNotSnap();
        return;
    }

    QMetaObject::invokeMethod(m_cameraWorker, "softwareTrigger", Qt::QueuedConnection);
}

///
/// \brief CamDisplay::SaveOnce 单次保存
///
void CamDisplay::SaveOnce()
{
    if(!m_cameraOpen)
    {
        warnCameraNotOpen();
        return;
    }
    if(!m_cameraSnap)
    {
        warnCameraNotSnap();
        return;
    }

    QMetaObject::invokeMethod(m_cameraWorker, "saveOnce", Qt::QueuedConnection);
}

void CamDisplay::SaveContinuous(bool checkstate)
{
    ui->checkBox->setChecked(checkstate);
}

string CamDisplay::get_saveoncename()
{
    return m_state.savePath.toStdString();
}

void CamDisplay::closeEvent(QCloseEvent *event)
{
    shutdownCameraThread();
    if(event)
        event->accept();
    QMainWindow::closeEvent(event);
}

///
/// \brief CamDisplay::on_pushButton_open_cam_clicked 打开相机
///
void CamDisplay::on_pushButton_open_cam_clicked()
{
    if(m_cameraOpen)
    {
        QMessageBox::warning(this, QString::fromLocal8Bit("警告"), QString::fromLocal8Bit("相机已打开"));
        return;
    }

    if(!m_cameraWorker)
        initCameraThread();

    if(!m_cameraWorker)
        return;

    QMetaObject::invokeMethod(m_cameraWorker,
                              "openCamera",
                              Qt::QueuedConnection,
                              Q_ARG(int, camindex));
}

///
/// \brief CamDisplay::on_pushButton_close_cam_clicked 关闭相机并清空显示
///
void CamDisplay::on_pushButton_close_cam_clicked()
{
    if(!m_cameraWorker)
        return;

    QMetaObject::invokeMethod(m_cameraWorker, "closeCamera", Qt::QueuedConnection);
}

///
/// \brief CamDisplay::on_pushButton_startsnap_clicked 开始采集
///
void CamDisplay::on_pushButton_startsnap_clicked()
{
    if(!m_cameraOpen)
    {
        warnCameraNotOpen();
        return;
    }
    if(m_cameraSnap)
    {
        QMessageBox::warning(this, QString::fromLocal8Bit("警告"), QString::fromLocal8Bit("相机正在采集"));
        return;
    }

    QMetaObject::invokeMethod(m_cameraWorker, "startSnap", Qt::QueuedConnection);
}

///
/// \brief CamDisplay::on_pushButton_stopsnap_clicked 停止采集
///
void CamDisplay::on_pushButton_stopsnap_clicked()
{
    if(!m_cameraWorker)
        return;

    QMetaObject::invokeMethod(m_cameraWorker, "stopSnap", Qt::QueuedConnection);
}

void CamDisplay::on_comboBox_trigger_model_currentIndexChanged(int index)
{
    if(refresh || !m_cameraOpen || index < 0)
        return;

    QMetaObject::invokeMethod(m_cameraWorker,
                              "setCurrentTriggerMode",
                              Qt::QueuedConnection,
                              Q_ARG(int, index));
}

void CamDisplay::on_comboBox_trigger_source_currentIndexChanged(int index)
{
    if(refresh || !m_cameraOpen || index < 0)
        return;

    QMetaObject::invokeMethod(m_cameraWorker,
                              "setCurrentTriggerSource",
                              Qt::QueuedConnection,
                              Q_ARG(int, index));
}

void CamDisplay::on_pushButton_soft_trigger_clicked()
{
    soft_trigger();
}

void CamDisplay::on_comboBox_trigger_active_currentIndexChanged(int index)
{
    if(refresh || !m_cameraOpen || index < 0)
        return;

    QMetaObject::invokeMethod(m_cameraWorker,
                              "setCurrentTriggerActive",
                              Qt::QueuedConnection,
                              Q_ARG(int, index));
}

void CamDisplay::on_checkBox_stateChanged(int arg1)
{
    Q_UNUSED(arg1)
    if(!m_cameraOpen || !m_cameraWorker)
        return;

    QMetaObject::invokeMethod(m_cameraWorker,
                              "setSaveEnabled",
                              Qt::QueuedConnection,
                              Q_ARG(bool, ui->checkBox->isChecked()));
}

void CamDisplay::on_lineEdit_exposetime_editingFinished()
{
    if(!m_cameraOpen)
    {
        warnCameraNotOpen();
        return;
    }

    QMetaObject::invokeMethod(m_cameraWorker,
                              "setShutterValue",
                              Qt::QueuedConnection,
                              Q_ARG(double, ui->lineEdit_exposetime->text().toDouble()));
}

void CamDisplay::on_pushButton_saveonce_clicked()
{
    SaveOnce();
}

void CamDisplay::on_checkBox_countsave_stateChanged(int arg1)
{
    Q_UNUSED(arg1)
    if(!m_cameraOpen || !m_cameraWorker)
        return;

    QMetaObject::invokeMethod(m_cameraWorker,
                              "setCountSaveEnabled",
                              Qt::QueuedConnection,
                              Q_ARG(bool, ui->checkBox_countsave->isChecked()));
}

void CamDisplay::on_pushButton_setsavepath_clicked()
{
    if(!m_cameraOpen)
    {
        warnCameraNotOpen();
        return;
    }

    QString path = QFileDialog::getExistingDirectory(this,
                                                     "SaveFilePath",
                                                     "C:/Users/10782/Desktop/calibrate/data/test_multifrequency_cuda");
    if(path.isEmpty())
        return;

    QMetaObject::invokeMethod(m_cameraWorker,
                              "setSavePath",
                              Qt::QueuedConnection,
                              Q_ARG(QString, path));
}

void CamDisplay::on_action_switch_triggered()
{
    if(ui->tabWidget->isHidden())
        ui->tabWidget->show();
    else
        ui->tabWidget->hide();
}

void CamDisplay::on_actionshowLine_triggered()
{
    if(!m_cameraOpen)
    {
        warnCameraNotOpen();
        return;
    }
    if(!m_cameraSnap)
    {
        QMessageBox::warning(this, QString::fromLocal8Bit("警告"), QString::fromLocal8Bit("Camera doesn't collect"));
        return;
    }
    if(!m_bitmap)
        return;

    LineDisplay = new Form_linepix_display(nullptr);
    LineDisplay->show();
    for (int i = 0; i < m_state.imageSize.width(); ++i)
    {
        LineDisplay->x.push_back(i);
        LineDisplay->y.push_back(0);
    }
    LineDisplay->pic_width = m_state.imageSize.width();
    LineDisplay->pic_height = m_state.imageSize.height();
    connect(m_bitmap, &CGXBitmap::ShowCallback,
            LineDisplay, &Form_linepix_display::picture_refreshed,
            Qt::QueuedConnection);
}

void CamDisplay::on_lineEdit_gain_editingFinished()
{
    if(!m_cameraOpen)
    {
        warnCameraNotOpen();
        return;
    }

    QMetaObject::invokeMethod(m_cameraWorker,
                              "setGain",
                              Qt::QueuedConnection,
                              Q_ARG(double, ui->lineEdit_gain->text().toDouble()));
}

void CamDisplay::on_comboBox_framrate_mode_currentIndexChanged(int index)
{
    if(refresh || !m_cameraOpen || index < 0)
        return;

    QMetaObject::invokeMethod(m_cameraWorker,
                              "setCurrentFrameRateMode",
                              Qt::QueuedConnection,
                              Q_ARG(int, index));
}

void CamDisplay::on_lineEdit_framerate_editingFinished()
{
    if(!m_cameraOpen)
    {
        warnCameraNotOpen();
        return;
    }

    QMetaObject::invokeMethod(m_cameraWorker,
                              "setFrameRate",
                              Qt::QueuedConnection,
                              Q_ARG(double, ui->lineEdit_framerate->text().toDouble()));
}

void CamDisplay::on_comboBox_denoise_currentIndexChanged(int index)
{
    if(refresh || !m_cameraOpen || index < 0)
        return;

    QMetaObject::invokeMethod(m_cameraWorker,
                              "setCurrentDeNoiseMode",
                              Qt::QueuedConnection,
                              Q_ARG(int, index));
}

void CamDisplay::on_lineEdit_denoise_editingFinished()
{
    if(!m_cameraOpen)
    {
        warnCameraNotOpen();
        return;
    }

    QMetaObject::invokeMethod(m_cameraWorker,
                              "setDeNoiseValue",
                              Qt::QueuedConnection,
                              Q_ARG(double, ui->lineEdit_denoise->text().toDouble()));
}
