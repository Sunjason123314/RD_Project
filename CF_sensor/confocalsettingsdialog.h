#ifndef CONFOCALSETTINGSDIALOG_H
#define CONFOCALSETTINGSDIALOG_H

#include <QDialog>

class QTabWidget;

namespace Ui {
class ConfocalSettingsDialog;
}

/**
 * @brief 光谱共焦传感器设置对话框。
 *
 * 由主菜单 actionCF_Sensor 触发弹出。提供三个按钮：
 *  - 取消：不写入，直接关闭；
 *  - 确认：写入当前设置后关闭；
 *  - 应用：写入当前设置但保持打开，便于调试。
 *
 * 具体设置项后续从 tabWidget_tri 迁入 contentArea。
 */
class ConfocalSettingsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ConfocalSettingsDialog(QWidget *parent = nullptr);
    ~ConfocalSettingsDialog();

    /// 返回内部的 QTabWidget，供主窗口把 CF 设置分页 reparent 进来。
    QTabWidget *contentTabWidget() const;

signals:
    /// 请求写入当前设置（确认/应用时发出）。由主窗口连接并执行实际写入。
    void settingsApplyRequested();

private slots:
    void onCancelClicked(); ///< 取消：直接关闭，不写入。
    void onOkClicked();     ///< 确认：写入后关闭。
    void onApplyClicked();  ///< 应用：写入但不关闭。

private:
    Ui::ConfocalSettingsDialog *ui;
};

#endif // CONFOCALSETTINGSDIALOG_H
