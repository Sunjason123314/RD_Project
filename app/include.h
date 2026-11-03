#ifndef INCLUDE_H
#define INCLUDE_H

// ============================
// Qt Modules
// ============================
#include <QMainWindow>
#include <QWidget>
#include <QSplitter>
#include <QLabel>
#include <QMessageBox>
#include <QTimer>
#include <QThread>
#include <QDateTime>
#include <QFileDialog>
#include <QDir>
#include <QDebug>
#include <QStringList>
#include <QImage>
#include <QProgressDialog>
#include <QDomElement>

// Qt Concurrent
#include <QtConcurrent/QtConcurrent>
#include <QFuture>
#include <QFutureWatcher>

// ============================
// Standard C++ Library
// ============================
#include <iostream>
#include <fstream>
#include <chrono>
#include <future>
#include <filesystem>

// ============================
// OpenCV
// ============================
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <corecrt_io.h>  // Windows CRT IO

// ============================
// PCL (Point Cloud Library)
// ============================
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d.h>

#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/correspondence_estimation.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/time.h>
#include <pcl/common/transforms.h>

// ============================
// Boost
// ============================
#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>

// ============================
// Project Modules
// ============================
#include "calibration.h"
#include "file_read_write.h"
#include "./CF_sensor/logindialog.h"
#include "CF_UserInterface.h"
#include "CF_ParameterDefine.h"
#include "./DLP/dlp.h"
#include "./CAM/ui/camdisplay.h"
// #include "pointcloud/dicqglviewer.h"  // 已通过 mainwindow.h 间接包含，避免 using namespace std 污染全局作用域导致 C2533 错误
// #include "HardwareDataProvider.h"

#include <QKeyEvent>
#include <QEvent>
#include <QLineEdit>
#include <QApplication>
#include <QFocusEvent>
// #include "cam/Cam.h"  // 如需启用，取消注释
// #include <cam/Cam.h>

#endif // INCLUDE_H
