#include "logindialog.h"
#include "ui_logindialog.h"


loginDialog::loginDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::loginDialog)
{
    ui->setupUi(this);
    this->g_global = ShareResource::get_instance();
}

loginDialog::~loginDialog()
{
    delete ui;
}


//选择CF2000设备
void loginDialog::on_radioButtonCF2000_clicked()
{
    if(ui->radioButtonCF2000->isChecked())
        scanLocalIP();
}

//选择CF3000设备
void loginDialog::on_radioButtonCF3000_clicked()
{
    if(ui->radioButtonCF3000->isChecked())
        scanLocalIP();
}

//选择CF4000设备
void loginDialog::on_radioButtonCF4000_clicked()
{
    if(ui->radioButtonCF4000->isChecked())
        scanDeviceList();
}

//选择CF3000Lite设备
void loginDialog::on_radioButtonCF3000Lite_clicked(bool checked)
{
    if(checked)
        scanDeviceList();
}

//刷新设备列表
void loginDialog::on_pushButtonRefresh_clicked()
{
    if(ui->radioButtonCF2000->isChecked())
        scanLocalIP();
    else if(ui->radioButtonCF3000->isChecked())
        scanLocalIP();
    else if(ui->radioButtonCF4000->isChecked())
        scanDeviceList();
    else if(ui->radioButtonCF3000Lite->isChecked())
        scanDeviceList();
    else
        QMessageBox::warning(this, "Waring", "请选择设备!" );
}

//连接选定的设备
void loginDialog::on_pushButtonLink_clicked()
{
    if(ui->radioButtonCF4000->isChecked() || ui->radioButtonCF3000Lite->isChecked())
    {
        if(deviceNumber > 0)
            linkCF4000CF3000LiteDevice();
    }
    else if(ui->radioButtonCF2000->isChecked() || ui->radioButtonCF3000->isChecked())
    {
        if(ui->comboBoxDeviceList->count() > 0)
            linkCF20003000Device();
    }
    else
    {
        QMessageBox::warning(this, tr("连接"), tr("请选择设备!"));
    }
}

//CF2000/CF3000设备，扫描本地IP，写入设备列表
void loginDialog::scanLocalIP()
{
    //清空队列列表
    localIP.clear();

    //清空下拉框列表
    ui->comboBoxDeviceList->clear();

    //扫描本地IP
    QList<QHostAddress> list = QNetworkInterface::allAddresses();
    foreach (QHostAddress address, list)
    {
       if(address.protocol() == QAbstractSocket::IPv4Protocol)
           //使用IPv4地址
           localIP.append(address.toString());
    }


    //写入设备列表
    if(localIP.count() > 0)
        ui->comboBoxDeviceList->addItems(localIP);
    else
    {
        ui->comboBoxDeviceList->setCurrentText("None");
        return;
    }

}

//CF4000和 CF3000Lite设备，扫描设备列表，写入设备列表
void loginDialog::scanDeviceList()
{
    int j = 0;

    ui->comboBoxDeviceList->clear();

    StatusTypeDef ret = CF_ScanDeviceList(&deviceInfo, &deviceNumber);
    if((ret != Status_Succeed) && deviceNumber < 0)
    {
        QMessageBox::warning(this, "warning", tr("设备列表获取失败!"));
        return;
    }

    if(ui->radioButtonCF3000Lite->isChecked())
    {
        for(int i = 0; i < deviceNumber; i++)
        {
            if(deviceInfo[i].descriptor[2] == '3')
            {
                CF3000LiteInfo[j] = deviceInfo[i];
                j++;
                ui->comboBoxDeviceList->addItem(deviceInfo[i].descriptor);
            }
        }
    }
    else if(ui->radioButtonCF4000->isChecked())
    {
        for(int i = 0; i < deviceNumber; i++)
        {
            if(deviceInfo[i].descriptor[2] == '4')
            {
                CF4000Info[j] = deviceInfo[i];
                j++;
                ui->comboBoxDeviceList->addItem(deviceInfo[i].descriptor);
            }
        }
    }
}

//连接CF2000/CF3000设备
void loginDialog::linkCF20003000Device()
{
    int listIndex = ui->comboBoxDeviceList->currentIndex();  //设备列表索引

    //设置控制器IP
    ControllerGEPara_t* ethernetDeviceInfo = new ControllerGEPara_t;
    ethernetDeviceInfo->controllerIp = NULL;

    QByteArray ipPtr = localIP[listIndex].toLatin1();   //QString转char*
    char* ip=ipPtr.data();

    //StatusTypeDef ret = CF_GE_OpenDevice(ethernetDeviceInfo, ip, &(g_global->g_handle));
    StatusTypeDef ret;
    if(ui->radioButtonCF2000->isChecked())
    {
        ret = CF_GE_OpenDevice(ethernetDeviceInfo, ip, &(g_global->g_handle), HPS_CF2000);
    }
    else if(ui->radioButtonCF3000->isChecked())
    {
        ret = CF_GE_OpenDevice(ethernetDeviceInfo, ip, &(g_global->g_handle), HPS_CF3000);
    }

    if(ret == Status_Succeed)
    {
        this->done(1);
    }
    else
    {
        this->done(0);
        QMessageBox::warning(this, "warning", tr("设备连接失败!") + ret);
    }
}

//连接CF4000/CF3000Lite设备
void loginDialog::linkCF4000CF3000LiteDevice()
{
    int listIndex = ui->comboBoxDeviceList->currentIndex();  //设备列表索引

    StatusTypeDef ret;
    if(ui->radioButtonCF4000->isChecked())
    {
        ret = CF_OpenDevice(&CF4000Info[listIndex], &(g_global->g_handle), HPS_CF4000);
    }
    else if(ui->radioButtonCF3000Lite->isChecked())
    {
        ret = CF_OpenDevice(&CF3000LiteInfo[listIndex], &(g_global->g_handle), HPS_CF3000Lite);
        g_global->g_isCF3000Lite = (ret == Status_Succeed ? true : false);
    }

    if(ret == Status_Succeed)
    {
        this->done(1);
    }
    else
    {
        this->done(0);
        QMessageBox::warning(this, "warning", tr("设备连接失败!") + ret);
    }
}


