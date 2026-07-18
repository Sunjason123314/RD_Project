cmake_minimum_required(VERSION 3.5)

# ******************************** CUDA 配置 ******************************** #
# 是否启用 CUDA
add_definitions(-DUSECUDA)
set(USECUDA ON)

if(USECUDA)
    message("Define USECUDA!")
endif()
# test
if(USECUDA)
    # 设置 CUDA 架构目标
    # 75 对应 NVIDIA GeForce GTX 1660 Ti / Super 等 Turing 架构显卡
    # 如果更换显卡，可根据实际 GPU 架构修改该数值
    set(CMAKE_CUDA_ARCHITECTURES 75)

    project(auto_measurement_cmake VERSION 0.1 LANGUAGES C CXX CUDA)

    # 查找并配置 CUDA
    find_package(CUDA REQUIRED)
    if(NOT CUDA_FOUND)
        message(FATAL_ERROR "CUDA not found. Please make sure CUDA Toolkit is installed and properly configured.")
    endif()

    message(STATUS "cuda version: " ${CUDA_VERSION_STRING})
else()
    project(auto_measurement_cmake VERSION 0.1 LANGUAGES C CXX CUDA)
endif()


# ******************************** CMake 基本设置 ******************************** #
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# MSVC: 仅对 C/C++ 源启用 UTF-8，避免把 /utf-8 传给 nvcc 导致 CUDA 编译失败
if(MSVC)
    add_compile_options(
        $<$<COMPILE_LANGUAGE:C>:/utf-8>
        $<$<COMPILE_LANGUAGE:CXX>:/utf-8>
    )
endif()

add_definitions(-D_HAS_STD_BYTE=0)

# 查找 Qt 包
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets Core Gui SerialBus SerialPort Network Charts Xml PrintSupport OpenGL Concurrent)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Core Gui SerialBus SerialPort Network Charts Xml PrintSupport OpenGL Concurrent)


# 主程序源文件
set(SOURCES
    app/main.cpp
    app/mainwindow.cpp
    SL_QT/sl_integration_runner.cpp
    CF_sensor/confocalsensorworker.cpp
    thread/threadmanager.cpp
    utils/calibration.cpp
    utils/file_read_write.cpp
    # Pushcontrollerfinal.cpp
    PLC/plc.cpp
    PLC/plcconnect.cpp
    CF_sensor/logindialog.cpp
    CF_sensor/qcustomplot.cpp
    ui/morecolordialog.cpp
)

# 头文件
set(HEADERS
    app/mainwindow.h
    CF_sensor/confocalsensorworker.h
    thread/threadmanager.h
    utils/calibration.h
    utils/file_read_write.h
    PLC/plc.h
    # Pushcontrollerfinal.h
    PLC/plcconnect.h
    CF_sensor/logindialog.h
    CF_sensor/shareResource.h
    CF_sensor/qcustomplot.h
    CF_sensor/CFxxxx_Lib/src_inc/CF_ParameterDefine.h
    CF_sensor/CFxxxx_Lib/src_inc/CF_UserInterface.h
    CF_sensor/CFxxxx_Lib/src_inc/TypeDefine.h
    SL_QT/sl_integration_runner.h
    ui/morecolordialog.h
)

# UI 文件
set(UIS
    app/mainwindow.ui
    PLC/plcconnect.ui
    CF_sensor/logindialog.ui
    ui/morecolordialog.ui
    Select_UI/stltopcd_dialog.ui
    Select_UI/reconst_surface_dialog.ui
    Select_UI/sor_dialog.ui
    Select_UI/voxel_grid_dialog.ui
    Select_UI/mls_dialog.ui
    Select_UI/normal_consistency_dialog.ui
    Select_UI/pca_dialog.ui
    Select_UI/curvature_analysis_dialog.ui
    Select_UI/coarse_registration_dialog.ui
    Select_UI/axis_rotate_dialog.ui
    Select_UI/icp_dialog.ui
)

# 资源文件
set(RESOURCES
    CF_sensor/language.qrc
)

# 翻译文件
set(TS_FILES
    CF_sensor/tr_zh.ts
    CF_sensor/tr_en.ts
)

# DLP 模块文件
set(DLP_SOURCE
    DLP/dlp.h
    DLP/dlp.cpp
    DLP/api/dlpc_common.h
    DLP/api/dlpc_common_private.h
    DLP/api/dlpc347x.h
    DLP/api/dlpc347x_dual.h
    DLP/api/dlpc347x_internal_patterns.h
    DLP/api/dlpc_common.c
    DLP/api/dlpc347x.c
    DLP/api/dlpc347x_dual.c
    DLP/api/dlpc347x_internal_patterns.c
    DLP/samples/cypress_i2c.h
    DLP/samples/dlpc347x_samples.c
    DLP/samples/cypress_i2c.c
    DLP/dlpworker.cpp
    DLP/dlpworker.h
)

# CAM 相机模块文件
set(CAM_SOURCE
    CAM/cam.h
    CAM/cam.cpp
    CAM/camworker.h
    CAM/camworker.cpp
    CAM/Common/GXBitmap.h
    CAM/Common/GXBitmap.cpp
    CAM/Common/threadpool.h
    CAM/ui/camdisplay.h
    CAM/ui/camdisplay.cpp
    CAM/ui/camdisplay.ui
    CAM/ui/gl_image.cpp
    CAM/ui/gl_image.h
    CAM/ui/form_linepix_display.h
    CAM/ui/form_linepix_display.cpp
    CAM/ui/form_linepix_display.ui
)

# 点云显示模块文件
set(QGL_SOURCE
    pointcloud/dicqglviewer.h
    pointcloud/dicqglviewer.cpp
    pointcloud/pointcloudlistview.h
    pointcloud/pointcloudlistview.cpp
    pointcloud/pointcloudworker.h
    pointcloud/pointcloudworker.cpp
    pointcloud/Point_Algorithm.h
    pointcloud/Point_Algorithm.cpp
    pointcloud/drawalgorithm.h
    pointcloud/drawalgorithm.cpp
    pointcloud/stepexporter.h
    pointcloud/stepexporter.cpp
    pointcloud/stepwriter_occt.h
    pointcloud/stepwriter_occt.cpp
    pointcloud/stepimporter.h
    pointcloud/stepimporter.cpp
    pointcloud/draw.h
    pointcloud/draw.cu
    pointcloud/icp_cuda.h
    pointcloud/icp_cuda.cpp
    pointcloud/icp_cuda_kernels.h
    pointcloud/icp_cuda_kernels.cu
    pointcloud/icp_svd.h
    pointcloud/icp_svd.cpp
)

# SL_QT 结构光核心模块文件
set(SL_QT_SOURCE
    SL_QT/src/class/cudaimage.h
    SL_QT/src/class/cudaimage.cu
    SL_QT/src/class/patterncoder.h
    SL_QT/src/class/patterncoder.cpp
    SL_QT/src/class/patterncoder.cu
    SL_QT/src/class/matcher.h
    SL_QT/src/class/matcher.cpp
    SL_QT/src/class/matcher.cu
    SL_QT/src/class/calibration.h
    SL_QT/src/class/calibration.cpp
    SL_QT/src/class/calibration.cu
)

# 添加可执行程序，统一汇总所有模块文件
add_executable(auto_measurement_cmake
    ${SOURCES}
    ${HEADERS}
    ${UIS}
    ${RESOURCES}
    ${TS_FILES}
    ${DLP_SOURCE}
    ${CAM_SOURCE}
    ${QGL_SOURCE}
    ${SL_QT_SOURCE}
    app/include.h


    ui/jogpanel.h
    ui/jogpanel.cpp
    ui/jogpanel.ui



)

# 解决主工程 Calibration 与 SL_QT Calibration 的同名冲突：
# 仅对 SL_QT 核心源和集成入口做编译期类名重写（Calibration -> SLCalibration）。
set_source_files_properties(
    SL_QT/sl_integration_runner.cpp
    SL_QT/src/class/calibration.cpp
    SL_QT/src/class/calibration.cu
    SL_QT/src/class/matcher.cpp
    SL_QT/src/class/matcher.cu
    SL_QT/src/class/patterncoder.cpp
    SL_QT/src/class/patterncoder.cu
    PROPERTIES COMPILE_DEFINITIONS "Calibration=SLCalibration;SL_ENABLE_MATLAB"
)

# Point_Algorithm.cpp 包含大量模板和内联函数，需要 /bigobj 选项
if(MSVC)
    set_source_files_properties(
        pointcloud/Point_Algorithm.cpp
        PROPERTIES COMPILE_FLAGS "/bigobj"
    )
endif()

# 包含路径
# 注意：utils/ 必须排在 SL_QT/src/class 之前，
# 因为主工程 utils/calibration.h 与 SL_QT/src/class/calibration.h 同名，
# app/include.h 里的 #include "calibration.h" 需优先解析到主工程版本。
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/app
    ${CMAKE_CURRENT_SOURCE_DIR}/ui
    ${CMAKE_CURRENT_SOURCE_DIR}/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/SL_QT
    ${CMAKE_CURRENT_SOURCE_DIR}/CF_sensor
    ${CMAKE_CURRENT_SOURCE_DIR}/CF_sensor/CFxxxx_Lib/src_inc
    ${CMAKE_CURRENT_SOURCE_DIR}/PLC
    ${CMAKE_CURRENT_SOURCE_DIR}/thread
    ${CMAKE_CURRENT_SOURCE_DIR}/pointcloud
    ${CMAKE_CURRENT_SOURCE_DIR}/centerlevel
    ${QXLSX_SRC_DIR}/heade
    ${CMAKE_CURRENT_SOURCE_DIR}/SL_QT/src
    ${CMAKE_CURRENT_SOURCE_DIR}/SL_QT/src/class
)


# *************************************************************************** #
# VCPKG 配置
# ===========

# 引入 vcpkg 构建系统
set(MY_COMPUTER_TYPE "DESKTOP")

# 根据变量判断机器类型并设置路径
if(MY_COMPUTER_TYPE STREQUAL "LAPTOP")
    message(STATUS "为笔记本电脑设置路径")
    include("E:/software/vcpkg-master/scripts/buildsystems/vcpkg.cmake")
    include_directories("E:/software/vcpkg-master/installed/x64-windows/include")

elseif(MY_COMPUTER_TYPE STREQUAL "DESKTOP")
    message(STATUS "为台式机设置路径")
    include("D:/application/vcpkg-master/scripts/buildsystems/vcpkg.cmake")
    include_directories("D:/application/vcpkg-master/installed/x64-windows/include")

else()
    message(FATAL_ERROR "未知的机器类型。请将 MY_COMPUTER_TYPE 设置为 LAPTOP 或 DESKTOP。")
endif()



# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# OpenGL
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})

# JsonCpp（SL_QT/commonfunc.hpp 里的 readJsonParameters 依赖）
find_package(jsoncpp CONFIG REQUIRED)

# OpenCascade 用于 STEP 导出
find_package(OpenCASCADE CONFIG REQUIRED)

# MATLAB 库配置
if(MY_COMPUTER_TYPE STREQUAL "LAPTOP")
    set(MATLABLIBS_PATH E:/software/matlab2022a/extern/lib/win64/microsoft)
    set(MATLAB_INCLUDE_DIR E:/software/matlab2022a/extern/include)
elseif(MY_COMPUTER_TYPE STREQUAL "DESKTOP")
    set(MATLABLIBS_PATH D:/application/matlab2022a/extern/lib/win64/microsoft)
    set(MATLAB_INCLUDE_DIR D:/application/matlab2022a/extern/include)
endif()
file(GLOB MATLAB_LIBS "${MATLABLIBS_PATH}/*.lib")
include_directories(${MATLAB_INCLUDE_DIR})

# CAM 相机 SDK
file(GLOB CAMERA_LIB "${CMAKE_CURRENT_SOURCE_DIR}/CAM/lib/x64/*.lib")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/CAM/inc")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/CAM/Common")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/CAM/ui")

# DLP
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/DLP")

# x64 版本 cyusbserial 库路径
set(CY_PATH ${CMAKE_CURRENT_SOURCE_DIR}/DLP/cyusbserial/x64)

# win32 版本 cyusbserial 库路径
# set(CY_PATH ${CMAKE_CURRENT_SOURCE_DIR}/DLP/third_party/cyusbserial)

file(GLOB CY_LIB "${CY_PATH}/cyusbserial.lib")

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/DLP/api")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/DLP/samples")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/DLP/cyusbserial")

# win32 版本头文件路径
# include_directories("${CMAKE_CURRENT_SOURCE_DIR}/DLP/third_party/cyusbserial")


# QGLViewer
set(QGLViewer_LIBS_PATH E:/project/610/libQGLViewer-2.8.0/libQGLViewer-2.8.0/QGLViewer)
file(GLOB QGLViewer_LIBS "${QGLViewer_LIBS_PATH}/QGLViewer2.lib")
file(GLOB QGLViewer_LIBS_D "${QGLViewer_LIBS_PATH}/QGLViewerd2.lib")
include_directories("E:/project/610/libQGLViewer-2.8.0/libQGLViewer-2.8.0")


# CUDA 库配置
if(USECUDA)
    # CUDA 头文件
    target_include_directories(auto_measurement_cmake PRIVATE ${CUDA_INCLUDE_DIRS})

    # CUDA 库文件
    target_link_libraries(auto_measurement_cmake PRIVATE ${CUDA_LIBRARIES})

    # 允许 CUDA 代码中使用扩展 lambda 表达式
    target_compile_options(auto_measurement_cmake PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda>)

    # 允许 CUDA 中使用放宽限制的 constexpr
    target_compile_options(auto_measurement_cmake PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>)
endif()

# CUDA 结构光模块，如需要可开启
# add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/StructuredLightCuda")


# *************************************************************************** #

# 链接 Qt 及第三方库
# 兼容单配置生成器（Ninja/Make）与多配置生成器（Visual Studio/Qt Creator）。
target_link_libraries(auto_measurement_cmake PRIVATE
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Gui
    Qt${QT_VERSION_MAJOR}::SerialBus
    Qt${QT_VERSION_MAJOR}::SerialPort
    Qt${QT_VERSION_MAJOR}::Network
    Qt${QT_VERSION_MAJOR}::Charts
    Qt${QT_VERSION_MAJOR}::Xml
    Qt${QT_VERSION_MAJOR}::PrintSupport
    Qt${QT_VERSION_MAJOR}::OpenGL
    ${CAMERA_LIB}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    # OpenCascade for STEP export
    TKernel
    TKMath
    TKG2d
    TKG3d
    TKGeomBase
    TKGeomAlgo
    TKBRep
    TKTopAlgo
    TKSTEP
    TKSTL
    TKMesh
    TKShHealing
    ${CY_LIB}
    Boost::thread
    ${CUDA_LIBRARIES}
    $<$<CONFIG:Debug>:${QGLViewer_LIBS_D}>
    $<$<NOT:$<CONFIG:Debug>>:${QGLViewer_LIBS}>
    ${OPENGL_LIBRARIES}
    ${MATLAB_LIBS}
)

# 兼容不同 vcpkg / JsonCpp 导出目标名
if(TARGET JsonCpp::JsonCpp)
    target_link_libraries(auto_measurement_cmake PRIVATE JsonCpp::JsonCpp)
elseif(TARGET jsoncpp_lib)
    target_link_libraries(auto_measurement_cmake PRIVATE jsoncpp_lib)
elseif(TARGET jsoncpp_static)
    target_link_libraries(auto_measurement_cmake PRIVATE jsoncpp_static)
else()
    message(FATAL_ERROR "jsoncpp target not found after find_package(jsoncpp CONFIG REQUIRED)")
endif()

# 链接外部 SDK 和 Windows API
target_link_libraries(auto_measurement_cmake PRIVATE DbgHelp)

if(WIN32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(SDK_PLATFORM_DIR x64)
    else()
        set(SDK_PLATFORM_DIR x86)
    endif()

    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        message(STATUS "build ${SDK_PLATFORM_DIR} debug")
    else()
        message(STATUS "build ${SDK_PLATFORM_DIR} release")
    endif()

    # 指定 SDK 库文件的绝对路径
    set(SDK_LIB_FILE "${CMAKE_CURRENT_SOURCE_DIR}/CF_sensor/CFxxxx_Lib/lib/${SDK_PLATFORM_DIR}/CF4000-CF3000Lite/hps_cfxxxx_sdk.lib")

    # 检查 SDK 库文件是否存在
    if(EXISTS ${SDK_LIB_FILE})
        message(STATUS "Found SDK lib: ${SDK_LIB_FILE}")
        target_link_libraries(auto_measurement_cmake PRIVATE ${SDK_LIB_FILE} DbgHelp)
    else()
        message(FATAL_ERROR "SDK library not found at: ${SDK_LIB_FILE}")
    endif()
endif()


# 设置输出路径
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(ARCH_DIR x64)
else()
    set(ARCH_DIR x86)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../${ARCH_DIR}/debug)
    message(STATUS "CF_SDK:debug")
else()
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../${ARCH_DIR}/release)
    message(STATUS "CF_SDK:release")
endif()


# 设置 AUTOUIC 搜索路径，让 MOC 能发现 Select_UI/ 下的 .ui 文件
set_target_properties(auto_measurement_cmake PROPERTIES
    AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/Select_UI"
)

# 设置目标属性
set_target_properties(auto_measurement_cmake PROPERTIES
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)


# 安装配置
include(GNUInstallDirs)

install(TARGETS auto_measurement_cmake
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)


# Qt6 特殊处理：最终化可执行程序
if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(auto_measurement_cmake)
endif()
