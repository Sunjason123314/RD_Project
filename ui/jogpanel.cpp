#include "jogpanel.h"
#include "ui_jogpanel.h"

#include <QPushButton>
#include <QLineEdit>
#include <QDoubleValidator>
#include <QToolButton>

JogPanel::JogPanel(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::JogPanel)
{
    ui->setupUi(this);

    //  在这里加
    this->setStyleSheet(R"(
    QWidget#JogPanel {
        background-color: #f3f6fa;
        font-family: "Microsoft YaHei";
        font-size: 10.5pt;
        color: #24364b;
    }

    QLabel#titleLabel {
        color: #315d89;
        font-size: 14pt;
        font-weight: 600;
        padding: 2px 0px;
    }

    QFrame#titleBar {
        background-color: #e8f0f8;
        border: 1px solid #cfdbe8;
        border-radius: 8px;
    }

    QPushButton {
        background-color: #6f9fd0;
        border: 1px solid #5c8dbc;
        border-radius: 8px;
        padding: 4px 12px;
        color: #ffffff;
        min-height: 28px;
    }

    QPushButton:hover {
        background-color: #5f92c6;
        border: 1px solid #4f83b6;
    }

    QPushButton:pressed {
        background-color: #4f83b6;
        border: 1px solid #3f739f;
    }

    QPushButton:disabled {
        background-color: #d9e2eb;
        color: #91a1b2;
        border: 1px solid #c7d2de;
    }

    QToolButton {
        background-color: #ffffff;
        border: 1px solid #cfdbe8;
        border-radius: 8px;
        padding: 6px 10px;
        min-height: 24px;
        color: #315d89;
        font-weight: 600;
        text-align: left;
    }

    QToolButton:hover {
        background-color: #eaf2fa;
        border-color: #9ebbd8;
    }

    QToolButton#btnClosePanel {
        min-width: 26px;
        max-width: 26px;
        min-height: 26px;
        max-height: 26px;
        padding: 0px;
        font-size: 15pt;
        color: #52677d;
    }

    QToolButton#btnClosePanel:hover {
        color: #ffffff;
        background-color: #d9534f;
        border-color: #c7433f;
    }

    QLineEdit {
        background-color: #ffffff;
        border: 1px solid #cfdbe8;
        border-radius: 7px;
        padding: 4px 8px;
        min-height: 22px;
        color: #24364b;
    }

    QLineEdit:focus {
        border: 1px solid #6f9fd0;
    }
    )");

    // 浮动小面板：无边框、置顶
    setWindowFlags(Qt::Tool | Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
    setAttribute(Qt::WA_DeleteOnClose, false);

    // 展开/收起时根据实体布局自动调整窗口高度。
    setMinimumWidth(460);
    setMaximumWidth(560);

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

    connect(ui->btnClosePanel, &QToolButton::clicked, this, &QWidget::hide);

    const auto configureExpander = [this](QToolButton* toggle, QWidget* content, const QString& title) {
        content->setVisible(false);
        toggle->setChecked(false);
        connect(toggle, &QToolButton::toggled, this, [this, toggle, content, title](bool expanded) {
            content->setVisible(expanded);
            toggle->setText((expanded ? QStringLiteral("▼  ") : QStringLiteral("▶  ")) + title);
            adjustSize();
        });
    };
    configureExpander(ui->btnToggleAbsolute, ui->absoluteContent, tr("绝对位置运行"));
    configureExpander(ui->btnToggleStep, ui->stepContent, tr("相对步进"));
    configureExpander(ui->btnToggleJog, ui->jogContent, tr("点动控制"));
    adjustSize();

    const auto numberValidator = new QDoubleValidator(-1000000000.0, 1000000000.0, 6, this);
    const QList<QLineEdit*> numericInputs = {
        ui->editTargetX, ui->editTargetY1, ui->editTargetY2, ui->editTargetZ1, ui->editTargetZ2,
        ui->editTargetLargeRotate, ui->editTargetSmallRotate,
        ui->editStepX, ui->editStepY1, ui->editStepZ1, ui->editStepY2, ui->editStepZ2,
        ui->editStepLargeRotate, ui->editStepSmallRotate
    };
    for(QLineEdit* input : numericInputs)
        input->setValidator(numberValidator);
    ui->editStepLargeRotate->setValidator(new QDoubleValidator(-180.0, 180.0, 6, this));
    ui->editStepSmallRotate->setValidator(new QDoubleValidator(-180.0, 180.0, 6, this));

    const auto readValue = [this](QLineEdit* input, double& value) {
        bool ok = false;
        value = input->text().trimmed().toDouble(&ok);
        if(!ok)
        {
            input->setFocus();
            return false;
        }
        return true;
    };

    connect(ui->btnMoveX, &QPushButton::clicked, this, [this, readValue]() { double v = 0.0; if(readValue(ui->editTargetX, v)) emit moveXRequested(v); });
    connect(ui->btnMoveY1, &QPushButton::clicked, this, [this, readValue]() { double v = 0.0; if(readValue(ui->editTargetY1, v)) emit moveY1Requested(v); });
    connect(ui->btnMoveY2, &QPushButton::clicked, this, [this, readValue]() { double v = 0.0; if(readValue(ui->editTargetY2, v)) emit moveY2Requested(v); });
    connect(ui->btnMoveZ1, &QPushButton::clicked, this, [this, readValue]() { double v = 0.0; if(readValue(ui->editTargetZ1, v)) emit moveZ1Requested(v); });
    connect(ui->btnMoveZ2, &QPushButton::clicked, this, [this, readValue]() { double v = 0.0; if(readValue(ui->editTargetZ2, v)) emit moveZ2Requested(v); });
    connect(ui->btnMoveLargeRotate, &QPushButton::clicked, this, [this, readValue]() { double v = 0.0; if(readValue(ui->editTargetLargeRotate, v)) emit moveLargeRotateRequested(v); });
    connect(ui->btnMoveSmallRotate, &QPushButton::clicked, this, [this, readValue]() { double v = 0.0; if(readValue(ui->editTargetSmallRotate, v)) emit moveSmallRotateRequested(v); });

    connect(ui->btnStepX, &QPushButton::clicked, this, [this, readValue]() { double v = 0.0; if(readValue(ui->editStepX, v)) emit stepXRequested(v); });
    connect(ui->btnStepY1, &QPushButton::clicked, this, [this, readValue]() { double v = 0.0; if(readValue(ui->editStepY1, v)) emit stepY1Requested(v); });
    connect(ui->btnStepZ1, &QPushButton::clicked, this, [this, readValue]() { double v = 0.0; if(readValue(ui->editStepZ1, v)) emit stepZ1Requested(v); });
    connect(ui->btnStepY2, &QPushButton::clicked, this, [this, readValue]() { double v = 0.0; if(readValue(ui->editStepY2, v)) emit stepY2Requested(v); });
    connect(ui->btnStepZ2, &QPushButton::clicked, this, [this, readValue]() { double v = 0.0; if(readValue(ui->editStepZ2, v)) emit stepZ2Requested(v); });
    connect(ui->btnStepLargeRotate, &QPushButton::clicked, this, [this, readValue]() { double v = 0.0; if(readValue(ui->editStepLargeRotate, v)) emit stepLargeRotateRequested(v); });
    connect(ui->btnStepSmallRotate, &QPushButton::clicked, this, [this, readValue]() { double v = 0.0; if(readValue(ui->editStepSmallRotate, v)) emit stepSmallRotateRequested(v); });
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
