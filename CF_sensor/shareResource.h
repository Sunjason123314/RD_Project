#ifndef SHARERESOURCE_H
#define SHARERESOURCE_H

#include "CF_UserInterface.h"

#include <QVector>
#include <QTranslator>

//C++单例模式，在一处写多处读取，或多处写多处读。
//将构造函数设为私有，通过方法调用构造函数

class ShareResource
{
public:
    ~ShareResource() {}

    //复制构造函数 和 赋值构造函数 禁用
    //C++11， 类成员函数后面用“=delete”修饰，表示这个函数被定义为deleted，不能再被调用
    ShareResource(const ShareResource&)=delete;
    ShareResource& operator=(const ShareResource&)=delete;
    //构造函数在私有里，通过方法调用
    static ShareResource* get_instance()
    {
        static ShareResource instance;
        return &instance;
    }

    QTranslator translator;         //中英文切换
    bool    g_isEnglish = false;    //英文状态

    bool    g_isCF3000Lite = false;      //设备是否是CF3000Lite

    int     g_handle;                    //设备句柄
    bool    g_isConnect = false;         //连接状态
    bool    g_isStart = false;           //启动状态
    bool    g_doubleModeEnable = false;  //双头模式使能状态


    float   g_measureResult[4] = {0.0};    //测量结果
    float   g_saturation[4] = {0.0};       //信号饱和度
    int     signalDate[4][1440] = {0};       //信号数据

    int     singleChannelIndex = 0;      //单通道索引
    int     doubleChannelIndex = 0;      //双通道索引

private:
    ShareResource(){}

};



















#endif // SHARERESOURCE_H
