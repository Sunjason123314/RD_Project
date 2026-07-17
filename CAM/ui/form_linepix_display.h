#ifndef FORM_LINEPIX_DISPLAY_H
#define FORM_LINEPIX_DISPLAY_H

#include <QWidget>

namespace Ui {
class Form_linepix_display;
}
typedef unsigned char       BYTE;

class Form_linepix_display : public QWidget
{
    Q_OBJECT

public:
    explicit Form_linepix_display(QWidget *parent = nullptr);
    ~Form_linepix_display();

    QVector<double> x, y;//ÏÔÊ¾Êý¾Ý
    int pic_width;
    int pic_height;

public slots:
    void picture_refreshed(BYTE* pBuffer);

private slots:

private:
    Ui::Form_linepix_display *ui;

};

#endif // FORM_LINEPIX_DISPLAY_H
