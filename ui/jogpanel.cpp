#include "jogpanel.h"
#include "ui_jogpanel.h"

#include <QPushButton>

JogPanel::JogPanel(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::JogPanel)
{
    ui->setupUi(this);

    //  在这里加
    this->setStyleSheet(R"(
    QWidget {
        background-color: #1e1f22;
        font-family: "Microsoft YaHei";
        font-size: 11pt;
        color: #e8eaed;
    }

    QFrame#panelFrame {
        background-color: #2b2d31;
        border: 1px solid #3a3d44;
        border-radius: 14px;
    }

    QLabel#titleLabel {
        color: #ffffff;
        font-size: 13pt;
        font-weight: 600;
        padding: 4px 0px 8px 0px;
    }

    QPushButton {
        background-color: #36393f;
        border: 1px solid #4b4f58;
        border-radius: 10px;
        padding: 10px 18px;
        color: #f2f3f5;
        min-height: 42px;
    }

    QPushButton:hover {
        background-color: #4f8cff;
        border: 1px solid #4f8cff;
    }

    QPushButton:pressed {
        background-color: #2f6fe4;
        border: 1px solid #2f6fe4;
    }

    QPushButton:disabled {
        background-color: #2a2c30;
        color: #7b8088;
        border: 1px solid #3a3d44;
    }
    )");

    // 浮动小面板：无边框、置顶
    setWindowFlags(Qt::Tool | Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
    setAttribute(Qt::WA_DeleteOnClose, false);

    // 固定大小
    setFixedSize(sizeHint());

    // 这里只保留鼠标按钮，不再处理键盘
    setFocusPolicy(Qt::NoFocus);

    // X
    connect(ui->btnXPositive, &QPushButton::pressed,
            this, &JogPanel::xPositivePressed);
    connect(ui->btnXPositive, &QPushButton::released,
            this, &JogPanel::xPositiveReleased);

    connect(ui->btnXNegative, &QPushButton::pressed,
            this, &JogPanel::xNegativePressed);
    connect(ui->btnXNegative, &QPushButton::released,
            this, &JogPanel::xNegativeReleased);

    // Y
    connect(ui->btnYPositive, &QPushButton::pressed,
            this, &JogPanel::yPositivePressed);
    connect(ui->btnYPositive, &QPushButton::released,
            this, &JogPanel::yPositiveReleased);

    connect(ui->btnYNegative, &QPushButton::pressed,
            this, &JogPanel::yNegativePressed);
    connect(ui->btnYNegative, &QPushButton::released,
            this, &JogPanel::yNegativeReleased);

    // Z
    connect(ui->btnZPositive, &QPushButton::pressed,
            this, &JogPanel::zPositivePressed);
    connect(ui->btnZPositive, &QPushButton::released,
            this, &JogPanel::zPositiveReleased);

    connect(ui->btnZNegative, &QPushButton::pressed,
            this, &JogPanel::zNegativePressed);
    connect(ui->btnZNegative, &QPushButton::released,
            this, &JogPanel::zNegativeReleased);

    // 调心 A
    connect(ui->btnAdjustAPositive, &QPushButton::pressed,
            this, &JogPanel::adjustAPositivePressed);
    connect(ui->btnAdjustAPositive, &QPushButton::released,
            this, &JogPanel::adjustAPositiveReleased);

    connect(ui->btnAdjustANegative, &QPushButton::pressed,
            this, &JogPanel::adjustANegativePressed);
    connect(ui->btnAdjustANegative, &QPushButton::released,
            this, &JogPanel::adjustANegativeReleased);

    // 调心 B
    connect(ui->btnAdjustBPositive, &QPushButton::pressed,
            this, &JogPanel::adjustBPositivePressed);
    connect(ui->btnAdjustBPositive, &QPushButton::released,
            this, &JogPanel::adjustBPositiveReleased);

    connect(ui->btnAdjustBNegative, &QPushButton::pressed,
            this, &JogPanel::adjustBNegativePressed);
    connect(ui->btnAdjustBNegative, &QPushButton::released,
            this, &JogPanel::adjustBNegativeReleased);
}

JogPanel::~JogPanel()
{
    delete ui;
}

void JogPanel::showEvent(QShowEvent *event)
{
    QWidget::showEvent(event);
    activateWindow();
}

void JogPanel::mousePressEvent(QMouseEvent *event)
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    if (event->button() == Qt::LeftButton) {
        dragOffset = event->globalPosition().toPoint() - frameGeometry().topLeft();
        event->accept();
    }
#else
    if (event->button() == Qt::LeftButton) {
        dragOffset = event->globalPos() - frameGeometry().topLeft();
        event->accept();
    }
#endif

    QWidget::mousePressEvent(event);
}

void JogPanel::mouseMoveEvent(QMouseEvent *event)
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    if (event->buttons() & Qt::LeftButton) {
        move(event->globalPosition().toPoint() - dragOffset);
        event->accept();
    }
#else
    if (event->buttons() & Qt::LeftButton) {
        move(event->globalPos() - dragOffset);
        event->accept();
    }
#endif

    QWidget::mouseMoveEvent(event);
}

// ================= 外部控制按钮外观 =================

void JogPanel::setXPositiveDown(bool down)
{
    ui->btnXPositive->setDown(down);
}

void JogPanel::setXNegativeDown(bool down)
{
    ui->btnXNegative->setDown(down);
}

void JogPanel::setYPositiveDown(bool down)
{
    ui->btnYPositive->setDown(down);
}

void JogPanel::setYNegativeDown(bool down)
{
    ui->btnYNegative->setDown(down);
}

void JogPanel::setZPositiveDown(bool down)
{
    ui->btnZPositive->setDown(down);
}

void JogPanel::setZNegativeDown(bool down)
{
    ui->btnZNegative->setDown(down);
}


#include <QKeyEvent>

void JogPanel::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Escape) {
        this->hide();
        event->accept();
        return;
    }

    QWidget::keyPressEvent(event);
}
