#ifndef MORECOLORDIALOG_H
#define MORECOLORDIALOG_H

#include <QDialog>
#include <QColor>
#include <QPushButton>

namespace Ui {
class MoreColorDialog;
}

class MoreColorDialog : public QDialog
{
    Q_OBJECT

public:
    explicit MoreColorDialog(QWidget *parent = nullptr);
    ~MoreColorDialog();

    QColor selectedColor() const;
    void setCurrentColor(const QColor &color);

private slots:
    void onColorButtonClicked();
    void onRGBChanged();

private:
    void createColorButtons();
    void updatePreview();

    Ui::MoreColorDialog *ui;
    QColor m_selectedColor;
    QList<QPushButton *> m_colorButtons;
    QPushButton *m_lastCheckedButton = nullptr;
};

#endif // MORECOLORDIALOG_H
