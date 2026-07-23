#include "camdisplay.h"
#include "ui_camdisplay.h"

#include <QMessageBox>
#include <QSignalBlocker>

using namespace std;

CamDisplay::CamDisplay(QWidget *parent, int index) :
    QMainWindow(parent),
    ui(new Ui::CamDisplay),
    camindex(index)
{
    ui->setupUi(this);

    // 只允许输入数字，防止明显非法参数进入相机。
    ui->lineEdit_exposetime->setValidator(new QIntValidator(0, 9999, ui->lineEdit_exposetime));
    ui->lineEdit_framerate->setValidator(new QIntValidator(0, 9999, ui->lineEdit_framerate));
    ui->lineEdit_denoise->setValidator(new QIntValidator(0, 9999, ui->lineEdit_denoise));

    // 直接在主线程创建相机对象（对齐 qt_camera，避免与投影仪 Cypress I2C 跨线程冲突）。
    camera = new CAM;
}

CamDisplay::~CamDisplay()
{
    disconnectBitmapSignal();
    if(ui && ui->openGLWidget)
        ui->openGLWidget->Clear();
    if(camera)
    {
        camera->Close_Cam();
        delete camera;
        camera = nullptr;
    }
    delete ui;
}

void CamDisplay::disconnectBitmapSignal()
{
    if(camera && camera->m_pBitmap && ui && ui->openGLWidget)
        disconnect(camera->m_pBitmap, &CGXBitmap::ShowCallback, ui->openGLWidget, &GL_Image::refresh);

    if(camera && camera->m_pBitmap && LineDisplay)
        disconnect(camera->m_pBitmap, &CGXBitmap::ShowCallback, LineDisplay, &Form_linepix_display::picture_refreshed);
}

// 直接从 camera 读取参数并刷新 UI 控件（对齐 qt_camera 的 __RefreshUi）。
void CamDisplay::refreshUiFromCamera()
{
    if(!camera)
        return;

    refresh = true;

    QSignalBlocker blockerTriggerMode(ui->comboBox_trigger_model);
    QSignalBlocker blockerTriggerSource(ui->comboBox_trigger_source);
    QSignalBlocker blockerTriggerActive(ui->comboBox_trigger_active);
    QSignalBlocker blockerFrameRateMode(ui->comboBox_framrate_mode);
    QSignalBlocker blockerDeNoiseMode(ui->comboBox_denoise);

    ui->comboBox_trigger_model->clear();
    ui->comboBox_trigger_model->addItems(camera->TriggerModeList);
    ui->comboBox_trigger_model->setCurrentIndex(camera->GetCurrentTriggerMode());

    ui->comboBox_trigger_source->clear();
    ui->comboBox_trigger_source->addItems(camera->TriggerSourceList);
    ui->comboBox_trigger_source->setCurrentIndex(camera->GetCurrentTriggerSource());

    ui->comboBox_trigger_active->clear();
    ui->comboBox_trigger_active->addItems(camera->TriggerActiveList);
    ui->comboBox_trigger_active->setCurrentIndex(camera->GetCurrentTriggerActive());

    ui->comboBox_framrate_mode->clear();
    ui->comboBox_framrate_mode->addItems(camera->FrameRateModeList);
    ui->comboBox_framrate_mode->setCurrentIndex(camera->GetCurrentFrameRateMode());

    ui->comboBox_denoise->clear();
    ui->comboBox_denoise->addItems(camera->DeNoiseModeList);
    ui->comboBox_denoise->setCurrentIndex(camera->GetCurrentDeNoiseMode());

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
    if(!camera->isOpen())
    {
        warnCameraNotOpen();
        return;
    }
    if(!camera->isSnap())
    {
        warnCameraNotSnap();
        return;
    }
    camera->SoftwareTrigger();
}

///
/// \brief CamDisplay::SaveOnce 单次保存
///
void CamDisplay::SaveOnce()
{
    if(!camera->isOpen())
    {
        warnCameraNotOpen();
        return;
    }
    if(!camera->isSnap())
    {
        warnCameraNotSnap();
        return;
    }
    if(camera->m_pBitmap)
        camera->m_pBitmap->SaveOnce();
}

void CamDisplay::SaveContinuous(bool checkstate)
{
    ui->checkBox->setChecked(checkstate);
}

void CamDisplay::setCountSaveChecked(bool checked)
{
    // setChecked 会触发 on_checkBox_countsave_stateChanged，自动下发到相机。
    ui->checkBox_countsave->setChecked(checked);
}

void CamDisplay::setSaveChecked(bool checked)
{
    // setChecked 会触发 on_checkBox_stateChanged，自动下发到相机。
    ui->checkBox->setChecked(checked);
}

bool CamDisplay::isCountSaveChecked() const
{
    return ui->checkBox_countsave->isChecked();
}

bool CamDisplay::isSaveChecked() const
{
    return ui->checkBox->isChecked();
}

string CamDisplay::get_saveoncename()
{
    if(camera && camera->m_pBitmap)
        return camera->m_pBitmap->GetSavepath();
    return string();
}

void CamDisplay::closeEvent(QCloseEvent *event)
{
    disconnectBitmapSignal();
    if(ui && ui->openGLWidget)
        ui->openGLWidget->Clear();
    if(camera)
        camera->Close_Cam();
    if(event)
        event->accept();
    QMainWindow::closeEvent(event);
}

///
/// \brief CamDisplay::on_pushButton_open_cam_clicked 打开相机
///
void CamDisplay::on_pushButton_open_cam_clicked()
{
    if(camera->isOpen())
    {
        QMessageBox::warning(this, QString::fromLocal8Bit("警告"), QString::fromLocal8Bit("相机已打开"));
        return;
    }

    camera->Open_Cam(camindex);

    // Open_Cam 失败时会记录 lastError 且 m_pBitmap 可能未创建。
    if(!camera->isOpen() || !camera->m_pBitmap)
    {
        const QString err = camera->lastError();
        camera->clearLastError();
        QMessageBox::warning(this, QString::fromLocal8Bit("警告"),
                             err.isEmpty() ? QString::fromLocal8Bit("打开相机失败") : err);
        return;
    }

    ui->openGLWidget->setImageSize(camera->m_pBitmap->GetPicSize());
    connect(camera->m_pBitmap, &CGXBitmap::ShowCallback,
            ui->openGLWidget, &GL_Image::refresh, Qt::QueuedConnection);

    const QString defaultPath = QStringLiteral("C:/Users/10782/Desktop/calibrate/data/test_multifrequency_cuda%1").arg(camindex);
    camera->m_pBitmap->SetSavePath(defaultPath.toStdString());
    ui->label_savepath_display->setText(QString::fromStdString(camera->m_pBitmap->GetSavepath()));
    ui->label_savepath_display->setWordWrap(true);

    ui->lineEdit_exposetime->setText(QString::number(camera->GetShutterValue()));
    ui->lineEdit_gain->setText(QString::number(camera->GetGain()));
    ui->lineEdit_framerate->setText(QString::number(camera->GetFrameRate()));
    ui->lineEdit_denoise->setText(QString::number(camera->GetDeNoiseValue()));

    double shutterMin, shutterMax;
    camera->GetShutterValueMinMax(shutterMin, shutterMax);
    ui->label_display_ShutterValue->setToolTip(QString::number(shutterMin) + QStringLiteral("~") + QString::number(shutterMax));

    double gainMin, gainMax;
    camera->GetGainMinMax(gainMin, gainMax);
    ui->label_display_Gain->setToolTip(QString::number(gainMin) + QStringLiteral("~") + QString::number(gainMax));

    double frameRateMin, frameRateMax;
    camera->GetFrameRateMinMax(frameRateMin, frameRateMax);
    ui->label_display_FrameRate->setToolTip(QString::number(frameRateMin) + QStringLiteral("~") + QString::number(frameRateMax));

    double deNoiseMin, deNoiseMax;
    camera->GetDeNoiseMinMax(deNoiseMin, deNoiseMax);
    ui->label_display_DeNoise->setToolTip(QString::number(deNoiseMin) + QStringLiteral("~") + QString::number(deNoiseMax));

    refreshUiFromCamera();
}

///
/// \brief CamDisplay::on_pushButton_close_cam_clicked 关闭相机并清空显示
///
void CamDisplay::on_pushButton_close_cam_clicked()
{
    disconnectBitmapSignal();
    if(ui && ui->openGLWidget)
        ui->openGLWidget->Clear();
    if(camera)
        camera->Close_Cam();
    refreshUiFromCamera();
}

///
/// \brief CamDisplay::on_pushButton_startsnap_clicked 开始采集
///
void CamDisplay::on_pushButton_startsnap_clicked()
{
    if(!camera->isOpen())
    {
        warnCameraNotOpen();
        return;
    }
    if(camera->isSnap())
    {
        QMessageBox::warning(this, QString::fromLocal8Bit("警告"), QString::fromLocal8Bit("相机正在采集"));
        return;
    }
    camera->StartSnap();
}

///
/// \brief CamDisplay::on_pushButton_stopsnap_clicked 停止采集
///
void CamDisplay::on_pushButton_stopsnap_clicked()
{
    if(camera)
        camera->StopSnap();
}

void CamDisplay::on_comboBox_trigger_model_currentIndexChanged(int index)
{
    if(refresh || !camera->isOpen() || index < 0)
        return;
    camera->SetCurrentTriggerMode(index);
}

void CamDisplay::on_comboBox_trigger_source_currentIndexChanged(int index)
{
    if(refresh || !camera->isOpen() || index < 0)
        return;
    camera->SetCurrentTriggerSource(index);
}

void CamDisplay::on_pushButton_soft_trigger_clicked()
{
    soft_trigger();
}

void CamDisplay::on_comboBox_trigger_active_currentIndexChanged(int index)
{
    if(refresh || !camera->isOpen() || index < 0)
        return;
    camera->SetCurrentTriggerActive(index);
}

void CamDisplay::on_checkBox_stateChanged(int arg1)
{
    Q_UNUSED(arg1)
    if(!camera->isOpen() || !camera->m_pBitmap)
        return;
    camera->m_pBitmap->SetSave(ui->checkBox->isChecked());
}

void CamDisplay::on_lineEdit_exposetime_editingFinished()
{
    if(!camera->isOpen())
    {
        warnCameraNotOpen();
        return;
    }
    if(!camera->SetShutterValue(ui->lineEdit_exposetime->text().toDouble()))
    {
        QMessageBox::warning(this, QString::fromLocal8Bit("警告"), QString::fromLocal8Bit("输入范围无效"));
        ui->lineEdit_exposetime->setText(QString::number(camera->GetShutterValue()));
    }
}

void CamDisplay::on_pushButton_saveonce_clicked()
{
    SaveOnce();
}

void CamDisplay::on_checkBox_countsave_stateChanged(int arg1)
{
    Q_UNUSED(arg1)
    if(!camera->isOpen() || !camera->m_pBitmap)
        return;
    camera->m_pBitmap->SetCount(ui->checkBox_countsave->isChecked());
}

void CamDisplay::on_pushButton_setsavepath_clicked()
{
    if(!camera->isOpen())
    {
        warnCameraNotOpen();
        return;
    }

    QString path = QFileDialog::getExistingDirectory(this,
                                                     "SaveFilePath",
                                                     "C:/Users/10782/Desktop/calibrate/data/test_multifrequency_cuda");
    if(path.isEmpty())
        return;

    if(camera->m_pBitmap)
    {
        camera->m_pBitmap->SetSavePath(path.toStdString());
        ui->label_savepath_display->setText(QString::fromStdString(camera->m_pBitmap->GetSavepath()));
        ui->label_savepath_display->setWordWrap(true);
    }
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
    if(!camera->isOpen())
    {
        warnCameraNotOpen();
        return;
    }
    if(!camera->isSnap())
    {
        QMessageBox::warning(this, QString::fromLocal8Bit("警告"), QString::fromLocal8Bit("Camera doesn't collect"));
        return;
    }
    if(!camera->m_pBitmap)
        return;

    const QSize picSize = camera->m_pBitmap->GetPicSize();
    LineDisplay = new Form_linepix_display(nullptr);
    LineDisplay->show();
    for (int i = 0; i < picSize.width(); ++i)
    {
        LineDisplay->x.push_back(i);
        LineDisplay->y.push_back(0);
    }
    LineDisplay->pic_width = picSize.width();
    LineDisplay->pic_height = picSize.height();
    connect(camera->m_pBitmap, &CGXBitmap::ShowCallback,
            LineDisplay, &Form_linepix_display::picture_refreshed,
            Qt::QueuedConnection);
}

void CamDisplay::on_lineEdit_gain_editingFinished()
{
    if(!camera->isOpen())
    {
        warnCameraNotOpen();
        return;
    }
    if(!camera->SetGain(ui->lineEdit_gain->text().toDouble()))
    {
        QMessageBox::warning(this, QString::fromLocal8Bit("警告"), QString::fromLocal8Bit("输入范围无效"));
        ui->lineEdit_gain->setText(QString::number(camera->GetGain()));
    }
}

void CamDisplay::on_comboBox_framrate_mode_currentIndexChanged(int index)
{
    if(refresh || !camera->isOpen() || index < 0)
        return;
    camera->SetCurrentFrameRateMode(index);
}

void CamDisplay::on_lineEdit_framerate_editingFinished()
{
    if(!camera->isOpen())
    {
        warnCameraNotOpen();
        return;
    }
    if(!camera->SetFrameRate(ui->lineEdit_framerate->text().toDouble()))
    {
        QMessageBox::warning(this, QString::fromLocal8Bit("警告"), QString::fromLocal8Bit("输入范围无效"));
        ui->lineEdit_framerate->setText(QString::number(camera->GetFrameRate()));
    }
}

void CamDisplay::on_comboBox_denoise_currentIndexChanged(int index)
{
    if(refresh || !camera->isOpen() || index < 0)
        return;
    camera->SetCurrentDeNoiseMode(index);
}

void CamDisplay::on_lineEdit_denoise_editingFinished()
{
    if(!camera->isOpen())
    {
        warnCameraNotOpen();
        return;
    }
    if(!camera->SetDeNoiseValue(ui->lineEdit_denoise->text().toDouble()))
    {
        QMessageBox::warning(this, QString::fromLocal8Bit("警告"), QString::fromLocal8Bit("输入范围无效"));
        ui->lineEdit_denoise->setText(QString::number(camera->GetDeNoiseValue()));
    }
}
