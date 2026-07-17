#ifndef LOGINDIALOG_H
#define LOGINDIALOG_H

#include "shareResource.h"
#include "CF_UserInterface.h"

#include <QDialog>
#include <QMessageBox>
#include <QHostAddress>
#include <QNetworkInterface>
#include <QAbstractSocket>

namespace Ui {
class loginDialog;
}

class loginDialog : public QDialog
{
    Q_OBJECT   

public:
    explicit loginDialog(QWidget *parent = nullptr);
    ~loginDialog();


private:
    Ui::loginDialog *ui;

    ShareResource   *g_global;              //全局变量类指针

    int             deviceNumber = 0;       //扫描返回的CF4000设备数
    DeviceInfo_t*   deviceInfo;             //扫描返回的CF4000和CF3000Lite设备信息
    DeviceInfo_t    CF4000Info[10];             //拆分出来的CF4000的设备信息
    DeviceInfo_t    CF3000LiteInfo[10];         //拆分出来的CF3000Lite的设备信息

    QList<QString>  localIP;                //存储本地IP的队列

private slots:
    void on_radioButtonCF2000_clicked();    //选择CF2000设备

    void on_radioButtonCF3000_clicked();    //选择CF3000设备

    void on_radioButtonCF4000_clicked();    //选择CF4000设备

    void on_pushButtonRefresh_clicked();    //刷新设备列表

    void on_pushButtonLink_clicked();       //连接所选设备

    void on_radioButtonCF3000Lite_clicked(bool checked);

private:
    void scanLocalIP();         //CF2000/CF3000设备，扫描本地IP，写入设备列表

    void scanDeviceList();      //CF4000设备，扫描设备列表，写入设备列表

    void linkCF20003000Device();    //连接CF2000/CF3000设备

    void linkCF4000CF3000LiteDevice();    //连接CF4000/CF3000Lite设备



};

#endif // LOGINDIALOG_H
