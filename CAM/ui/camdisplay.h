#ifndef CAMDISPLAY_H
#define CAMDISPLAY_H

#include <QCloseEvent>
#include <QFileDialog>
#include <QGraphicsItem>
#include <QMainWindow>
#include <QPointer>
#include <QThread>

#include "CAM/camworker.h"
#include "form_linepix_display.h"
#include "thread/threadmanager.h"

#include <iostream>

namespace Ui {
class CamDisplay;
}

class CamDisplay : public QMainWindow
{
    Q_OBJECT

public:
    // threadManager �� MainWindow ���룬CamDisplay ������ Camera ���߳�ʵ������ֱ�� new/delete QThread��
    explicit CamDisplay(QWidget *parent = nullptr,
                        int index = 0,
                        ThreadManager *threadManager = nullptr,
                        int instanceId = 0);
    ~CamDisplay();

public:
    void soft_trigger();
    void SaveOnce();
    void SaveContinuous(bool checkstate);
    std::string get_saveoncename();

    // 供主窗口“全部勾选”按钮调用：设置计数存图/存图复选框的勾选状态。
    // 通过 setChecked 触发已有的 stateChanged 槽，复用原有下发 worker 的逻辑。
    void setCountSaveChecked(bool checked);
    void setSaveChecked(bool checked);

    // 读取当前勾选状态，供主窗口判断是否“全部已勾选”，实现全选/取消全选的切换。
    bool isCountSaveChecked() const;
    bool isSaveChecked() const;

private slots:
    void on_pushButton_open_cam_clicked();

    void on_pushButton_close_cam_clicked();

    void on_pushButton_startsnap_clicked();

    void on_pushButton_stopsnap_clicked();

    void on_comboBox_trigger_model_currentIndexChanged(int index);

    void on_comboBox_trigger_source_currentIndexChanged(int index);

    void on_pushButton_soft_trigger_clicked();

    void on_comboBox_trigger_active_currentIndexChanged(int index);

    void on_checkBox_stateChanged(int arg1);

    void on_lineEdit_exposetime_editingFinished();

    void on_pushButton_saveonce_clicked();

    void on_checkBox_countsave_stateChanged(int arg1);

    void on_pushButton_setsavepath_clicked();

    void on_action_switch_triggered();

    void on_actionshowLine_triggered();

    void on_lineEdit_gain_editingFinished();

    void on_comboBox_framrate_mode_currentIndexChanged(int index);

    void on_lineEdit_framerate_editingFinished();

    void on_comboBox_denoise_currentIndexChanged(int index);

    void on_lineEdit_denoise_editingFinished();

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    void initCameraThread();
    void shutdownCameraThread();
    void disconnectBitmapSignal();
    void applyCameraState(const CamDeviceState& state);
    void refreshUiFromState(const CamDeviceState& state);
    void warnCameraNotOpen();
    void warnCameraNotSnap();

    Ui::CamDisplay *ui = nullptr;
    ThreadManager *m_threadManager = nullptr; ///< ?????????????? MainWindow ?????????????? QThread ??????
    QPointer<CamWorker> m_cameraWorker;       ///< ???????????????? worker????? QPointer ????????????????
    QPointer<CGXBitmap> m_bitmap;             ///< ???????????????????????????

    int camindex = 0;                         ///< ?��?��??��??????????
    int m_instanceId = 0;                     ///< ThreadManager ?? Camera ????��????????
    bool m_threadStarted = false;             ///< ?????? worker ?????????? ThreadManager ??????
    bool m_cameraOpen = false;                ///< GUI ????????????????
    bool m_cameraSnap = false;                ///< GUI ?????????????????
    bool refresh = false;                     ///< ?????????????? currentIndexChanged ?????????

    CamDeviceState m_state;                   ///< GUI ???????????????????????
    Form_linepix_display* LineDisplay = nullptr; ///< ????????????????????????
};

#endif // CAMDISPLAY_H
