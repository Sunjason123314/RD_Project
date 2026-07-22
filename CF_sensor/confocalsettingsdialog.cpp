#include "confocalsettingsdialog.h"
#include "ui_confocalsettingsdialog.h"

#include <QTabWidget>

ConfocalSettingsDialog::ConfocalSettingsDialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::ConfocalSettingsDialog)
{
    ui->setupUi(this);

    connect(ui->btnCancel, &QPushButton::clicked, this, &ConfocalSettingsDialog::onCancelClicked);
    connect(ui->btnOk,     &QPushButton::clicked, this, &ConfocalSettingsDialog::onOkClicked);
    connect(ui->btnApply,  &QPushButton::clicked, this, &ConfocalSettingsDialog::onApplyClicked);
}

ConfocalSettingsDialog::~ConfocalSettingsDialog()
{
    delete ui;
}

QTabWidget *ConfocalSettingsDialog::contentTabWidget() const
{
    return ui->tabWidgetCF;
}

void ConfocalSettingsDialog::onCancelClicked()
{
    // 取消：不写入，直接关闭。
    reject();
}

void ConfocalSettingsDialog::onOkClicked()
{
    // 确认：写入当前设置后关闭。
    emit settingsApplyRequested();
    accept();
}

void ConfocalSettingsDialog::onApplyClicked()
{
    // 应用：写入当前设置但保持打开，便于调试。
    emit settingsApplyRequested();
}
