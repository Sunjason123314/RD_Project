#include "plcconnect.h"
#include "ui_plcconnect.h"

#include <QPushButton>

PlcConnect::PlcConnect(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::PlcConnect)
    , m_isConnected(false)
{
    ui->setupUi(this);

    setWindowTitle(tr("PLC连接控制"));
    setWindowFlags(Qt::Window);

    initStyle();

    connect(ui->btnConnect, &QPushButton::clicked,
            this, [this]() {
                emit connectRequested();
            });

    connect(ui->btnDisconnect, &QPushButton::clicked,
            this, [this]() {
                emit disconnectRequested();
            });

    connect(ui->btnClose, &QPushButton::clicked,
            this, &QWidget::close);

    setPlcConnected(false);
}

PlcConnect::~PlcConnect()
{
    delete ui;
}

void PlcConnect::setPlcConnected(bool connected)
{
    m_isConnected = connected;

    if (m_isConnected) {
        ui->labelStatus->setText(tr("PLC has been connected"));

        ui->btnConnect->setEnabled(false);
        ui->btnDisconnect->setEnabled(true);
    }
    else {
        ui->labelStatus->setText(tr("PLC has not been connected"));

        ui->btnConnect->setEnabled(true);
        ui->btnDisconnect->setEnabled(false);
    }
}

void PlcConnect::setPlcConnecting()
{
    m_isConnected = false;

    ui->labelStatus->setText(tr("当前状态：PLC正在连接..."));

    ui->btnConnect->setEnabled(false);
    ui->btnDisconnect->setEnabled(false);
}

void PlcConnect::initStyle()
{
    ui->labelStatus->setAlignment(Qt::AlignCenter);

    ui->btnConnect->setMinimumSize(80, 32);
    ui->btnDisconnect->setMinimumSize(80, 32);
    ui->btnClose->setMinimumSize(80, 32);

    this->setStyleSheet(R"(
QWidget {
    background-color: #f5f7fa;
    font-family: "Microsoft YaHei";
    font-size: 11pt;
}

QLabel {
    color: #303133;
    font-size: 12pt;
    font-weight: 600;
}

QPushButton {
    background-color: #409eff;
    color: white;
    border: 1px solid #409eff;
    border-radius: 6px;
    padding: 6px 16px;
    min-width: 70px;
    min-height: 28px;
}

QPushButton:hover {
    background-color: #66b1ff;
}

QPushButton:pressed {
    background-color: #3a8ee6;
}

QPushButton:disabled {
    background-color: #c0c4cc;
    color: #ffffff;
    border: 1px solid #bfc4cd;
}
)");
}
