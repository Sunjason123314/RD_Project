# *******************************************************************************
#
# package:     CFxxxx_Lib
# file:        CFxxxx_Lib.pri
# created:     2022-09-29
# author:      Perry
#
#
# Copyright 2022 Hypersen
#
#
# *******************************************************************************
CONFIG += resources_big   #解决编译器堆空间不足问题

LIBS += -lDbgHelp

INCLUDEPATH += $$PWD\src_inc
DEPENDPATH += $$PWD/

HEADERS += \
    $$PWD\src_inc/CF_ParameterDefine.h \
    $$PWD\src_inc/CF_UserInterface.h \
    $$PWD\src_inc/TypeDefine.h


INCLUDEPATH += "D:/Windows Kits/10/Include/10.0.17763.0/ucrt"


CONFIG += debug_and_release
CONFIG(debug, debug|release){
    contains(DEFINES, WIN64) {
    message("build x64 debug")
    message("current path:" + $$PWD);
     #以太网设备库路径
     #win32:LIBS += $$PWD/lib/x64/CF2000-CF3000/hps_cfxxxx_sdk.lib
     #USB3.0设备用下面的库路径
     win32:LIBS += $$PWD/lib/x64/CF4000-CF3000Lite/hps_cfxxxx_sdk.lib
    } else {
     #win32:LIBS += $$PWD/lib/x86/CF2000-CF3000/hps_cfxxxx_sdk.lib
     win32:LIBS += $$PWD/lib/x86/CF4000-CF3000Lite/hps_cfxxxx_sdk.lib
    }
} else {
    contains(DEFINES, WIN64) {
     #win32:LIBS += $$PWD/lib/x64/CF2000-CF3000/hps_cfxxxx_sdk.lib
     win32:LIBS += $$PWD/lib/x64/CF4000-CF3000Lite/hps_cfxxxx_sdk.lib
    } else {
     #win32:LIBS += $$PWD/lib/x86/CF2000-CF3000/hps_cfxxxx_sdk.lib
     win32:LIBS += $$PWD/lib/x86/CF4000-CF3000Lite/hps_cfxxxx_sdk.lib
    }
}




