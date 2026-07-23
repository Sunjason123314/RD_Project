#ifndef CAMDISPLAY_H
#define CAMDISPLAY_H

#include <QCloseEvent>
#include <QFileDialog>
#include <QGraphicsItem>
#include <QMainWindow>

#include "CAM/cam.h"
#include "form_linepix_display.h"

#include <iostream>

namespace Ui {
class CamDisplay;
}

class CamDisplay : public QMainWindow
{
    Q_OBJECT

public:
    // 直接在主线程创建并持有 CAM 对象（对齐 qt_camera），不再使用 CamWorker/ThreadManager。
    explicit CamDisplay(QWidget *parent = nullptr,
                        int index = 0);
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
    void disconnectBitmapSignal();
    void refreshUiFromCamera();               ///< 直接从 camera 读取参数并刷新 UI 控件
    void warnCameraNotOpen();
    void warnCameraNotSnap();

    Ui::CamDisplay *ui = nullptr;
    CAM *camera = nullptr;                     ///< 直接持有的相机对象，主线程内同步调用

    int camindex = 0;                          ///< 打开的设备编号
    bool refresh = false;                      ///< UI 刷新期间屏蔽 currentIndexChanged 联动

    Form_linepix_display* LineDisplay = nullptr; ///< 行像素显示窗口
};

#endif // CAMDISPLAY_H
