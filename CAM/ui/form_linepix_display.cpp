#include "form_linepix_display.h"
#include "ui_form_linepix_display.h"
#include <iostream>
using namespace std;

Form_linepix_display::Form_linepix_display(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Form_linepix_display)
{
    ui->setupUi(this);

    //显示控件初始化
    ui->widget_showLine->addGraph();
    // give the axes some labels:
    ui->widget_showLine->xAxis->setLabel("x");
    ui->widget_showLine->yAxis->setLabel("y");
    ui->widget_showLine->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);//设置拖拽和缩放

    ui->spinBox_showLine->setMaximum(pic_height);
}

Form_linepix_display::~Form_linepix_display()
{
    delete ui;
}

void Form_linepix_display::picture_refreshed(BYTE* pBuffer)
{

//    y.clear();
//    cout<<pBuffer[0]<<endl;
    if (x.size() != y.size())
    {
        y.resize(x.size());
    }
    int adress=pic_width*ui->spinBox_showLine->value();
    for (int i = 0; i < x.size(); ++i)
    {
        y[i]=static_cast<double>(pBuffer[i+adress]);
    }
    ui->widget_showLine->graph(0)->setData(x, y);
    ui->widget_showLine->replot();
}

