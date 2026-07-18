#ifndef JOGPANEL_H
#define JOGPANEL_H

#include <QWidget>
#include <QPoint>
#include <QMouseEvent>
#include <QShowEvent>
#include <QKeyEvent>


QT_BEGIN_NAMESPACE
namespace Ui { class JogPanel; }
QT_END_NAMESPACE

class JogPanel : public QWidget
{
    Q_OBJECT

public:
    explicit JogPanel(QWidget *parent = nullptr);
    ~JogPanel();

    // 供 MainWindow 主动控制按钮外观
    void setXPositiveDown(bool down);
    void setXNegativeDown(bool down);
    void setYPositiveDown(bool down);
    void setYNegativeDown(bool down);
    void setZPositiveDown(bool down);
    void setZNegativeDown(bool down);

signals:
    // 鼠标按钮控制信号
    void xPositivePressed();
    void xPositiveReleased();

    void xNegativePressed();
    void xNegativeReleased();

    void yPositivePressed();
    void yPositiveReleased();

    void yNegativePressed();
    void yNegativeReleased();

    void zPositivePressed();
    void zPositiveReleased();

    void zNegativePressed();
    void zNegativeReleased();

    // ===== 新增：调心 A / B =====
    void adjustAPositivePressed();
    void adjustAPositiveReleased();

    void adjustANegativePressed();
    void adjustANegativeReleased();

    void adjustBPositivePressed();
    void adjustBPositiveReleased();

    void adjustBNegativePressed();
    void adjustBNegativeReleased();

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void showEvent(QShowEvent *event) override;

private:
    Ui::JogPanel *ui;
    QPoint dragOffset;

protected:
    void keyPressEvent(QKeyEvent *event) override;
};

#endif // JOGPANEL_H
