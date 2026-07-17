#ifndef POINTCLOUDWORKER_H
#define POINTCLOUDWORKER_H

#include "dicqglviewer.h"

#include <QObject>
#include <QString>
#include <pcl/PolygonMesh.h>

/**
 * @brief 点云文件加载后台 worker。
 *
 * QGLViewer/QOpenGLWidget 必须保留在 GUI 线程中绘制；本 worker 负责把 STL/PCD/TXT
 * 文件读取和点云数据准备放到 PointCloudDisplayThread 中，读取完成后通过 queued signal
 * 把点云交回主线程刷新显示，避免文件导入时阻塞界面。
 */
class PointCloudWorker : public QObject
{
    Q_OBJECT

public:
    explicit PointCloudWorker(QObject *parent = nullptr);
    ~PointCloudWorker() override;

public slots:
    // 在点云子线程中读取 STL/PCD/TXT 文件。
    void loadFile(const QString& filePath);

    // ThreadManager 停止线程前调用；当前 worker 无硬件资源，仅用于统一生命周期接口。
    void shutdown();

signals:
    void fileLoadStarted(const QString& filePath);
    // 点云导入成功：用于 PCD/TXT/STL(点模式) 路径。
    void fileLoadSucceeded(const PCLCloud::Ptr& cloud,
                           const QString& filePath,
                           const QString& fileExtension,
                           double elapsedSeconds);
    // 网格导入成功：用于 STL 原始三角面显示路径（接近 CAD/SolidWorks 视觉效果）。
    void meshLoadSucceeded(const PCLMesh& mesh,
                           const QString& filePath,
                           const QString& fileExtension,
                           double elapsedSeconds);
    void fileLoadFailed(const QString& filePath, const QString& reason);
    void logMessage(const QString& message);
};

Q_DECLARE_METATYPE(PCLCloud::Ptr)
// 允许 PCLMesh 在 Qt queued signal/slot 中跨线程传递。
Q_DECLARE_METATYPE(PCLMesh)

#endif // POINTCLOUDWORKER_H
