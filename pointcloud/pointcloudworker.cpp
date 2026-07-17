#include "pointcloudworker.h"

#include <QFileInfo>

#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>

#include <chrono>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <array>
#include <set>

// 构造函数：注册跨线程传递的点云智能指针元类型。
PointCloudWorker::PointCloudWorker(QObject *parent)
    : QObject(parent)
{
    qRegisterMetaType<PCLCloud::Ptr>("PCLCloud::Ptr");
    // 注册网格类型，供 meshLoadSucceeded 跨线程回到 GUI 线程。
    qRegisterMetaType<PCLMesh>("PCLMesh");
}

PointCloudWorker::~PointCloudWorker()
{
    shutdown();
}

// 后台加载入口：在点云子线程中执行文件IO与点云构建，
// 完成后通过信号把结果回传到GUI线程。
void PointCloudWorker::loadFile(const QString& filePath)
{
    emit fileLoadStarted(filePath);

    // 1) 基础合法性校验
    QFileInfo fileInfo(filePath);
    if(filePath.isEmpty() || !fileInfo.exists() || !fileInfo.isFile())
    {
        emit fileLoadFailed(filePath, QString::fromLocal8Bit("文件不存在或无法打开"));
        return;
    }

    // 2) 扩展名分流：仅支持 STL / PCD / TXT
    const QString fileExtension = fileInfo.suffix().toLower();
    if(fileExtension != QStringLiteral("stl") &&
       fileExtension != QStringLiteral("pcd") &&
       fileExtension != QStringLiteral("txt") &&
       fileExtension != QStringLiteral("ply"))
    {
        emit fileLoadFailed(filePath, QString::fromLocal8Bit("不支持的文件格式，请选择 STL/PCD/TXT/PLY 文件"));
        return;
    }

    // 3) 开始计时并准备输出点云
    const auto start = std::chrono::high_resolution_clock::now();
    const std::string pathLocal = filePath.toLocal8Bit().constData();
    PCLCloud::Ptr cloud(new PCLCloud);

    // 4) 按类型读取文件
    if(fileExtension == QStringLiteral("stl"))
    {
        PCLMesh mesh;
        const int result = pcl::io::loadPolygonFileSTL(pathLocal, mesh);
        if(result == -1)
        {
            emit fileLoadFailed(filePath, QString::fromLocal8Bit("STL 文件加载失败"));
            return;
        }

        // STL 优先走 mesh.cloud；部分 STL（常见于 ASCII/某些导出器）cloud 为空，
        // 但 polygons 有效，此时从多边形索引反建点云做兼容。
        pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

        // 兼容兜底：若 PCL 没有给出有效点云，则尝试按 ASCII STL 的 vertex 行重建点集。
        if(cloud->points.empty())
        {
            std::ifstream stlIn(pathLocal, std::ios::in);
            if(stlIn.is_open())
            {
                std::set<std::array<float, 3>> uniquePts;
                std::string line;
                while(std::getline(stlIn, line))
                {
                    std::istringstream iss(line);
                    std::string token;
                    iss >> token;
                    if(token == "vertex")
                    {
                        float x = 0.0f;
                        float y = 0.0f;
                        float z = 0.0f;
                        if(iss >> x >> y >> z)
                            uniquePts.insert({x, y, z});
                    }
                }

                cloud->points.reserve(uniquePts.size());
                for(const auto& p : uniquePts)
                    cloud->points.emplace_back(p[0], p[1], p[2]);
            }
        }

        emit logMessage(QStringLiteral("STL file loaded successfully. The mesh contains %1 polygons.")
                        .arg(mesh.polygons.size()));

        // 若 STL 含有效三角面，则优先走 mesh 通道，让前端按实体网格显示。
        if(!mesh.polygons.empty())
        {
            const auto end = std::chrono::high_resolution_clock::now();
            const std::chrono::duration<double> duration = end - start;
            emit meshLoadSucceeded(mesh, filePath, fileExtension, duration.count());
            return;
        }
    }
    else if(fileExtension == QStringLiteral("pcd"))
    {
        const int result = pcl::io::loadPCDFile(pathLocal, *cloud);
        if(result == -1)
        {
            emit fileLoadFailed(filePath, QString::fromLocal8Bit("PCD 文件加载失败"));
            return;
        }

        emit logMessage(QStringLiteral("PCD file loaded successfully."));
    }
    else if(fileExtension == QStringLiteral("txt"))
    {
        std::ifstream infile(pathLocal);
        if(!infile.is_open())
        {
            emit fileLoadFailed(filePath, QString::fromLocal8Bit("无法打开 TXT 文件"));
            return;
        }

        // TXT 按空行分组: 每遇到一个空行, 当前组结束, 新组开始.
        // 每组作为一个独立的点云, 方便后续分别处理.
        std::vector<PCLCloud::Ptr> groups;
        PCLCloud::Ptr currentGroup(new PCLCloud);

        std::string line;
        while(std::getline(infile, line))
        {
            size_t pos = line.find_first_not_of(" \t\r");
            if(pos == std::string::npos)
            {
                if(!currentGroup->points.empty())
                {
                    currentGroup->width  = static_cast<std::uint32_t>(currentGroup->points.size());
                    currentGroup->height = 1;
                    currentGroup->is_dense = false;
                    groups.push_back(std::move(currentGroup));
                    currentGroup.reset(new PCLCloud);
                }
                continue;
            }
            size_t endpos = line.find_last_not_of(" \t\r");
            std::string trimmed = line.substr(pos, endpos - pos + 1);

            std::istringstream iss(trimmed);
            double x = 0.0, y = 0.0, z = 0.0;
            if(!(iss >> x >> y >> z))
                continue;

            currentGroup->points.emplace_back(pcl::PointXYZ(x, y, z));
        }
        if(!currentGroup->points.empty())
        {
            currentGroup->width  = static_cast<std::uint32_t>(currentGroup->points.size());
            currentGroup->height = 1;
            currentGroup->is_dense = false;
            groups.push_back(std::move(currentGroup));
        }

        if(groups.empty())
        {
            emit fileLoadFailed(filePath, QString::fromLocal8Bit("TXT 文件中没有有效点数据"));
            return;
        }

        if(groups.size() == 1)
        {
            cloud = groups[0];
            emit logMessage(QStringLiteral("TXT file loaded: %1 points.").arg(cloud->points.size()));
        }
        else
        {
            const int totalGroups = static_cast<int>(groups.size());
            emit logMessage(QStringLiteral("TXT file has %1 groups, loading as separate point clouds...").arg(totalGroups));

            const auto end = std::chrono::high_resolution_clock::now();
            const std::chrono::duration<double> duration = end - start;

            for(int i = 0; i < totalGroups; ++i)
            {
                QString groupPath = QString("%1 %2")
                    .arg(filePath)
                    .arg(i + 1, 2, 10, QLatin1Char('0'));

                emit fileLoadSucceeded(groups[i], groupPath, fileExtension, duration.count());
            }
            return;
        }
    }
    else if(fileExtension == QStringLiteral("ply"))
    {
        PCLMesh mesh;
        const int result = pcl::io::loadPLYFile(pathLocal, mesh);
        if(result == -1)
        {
            emit fileLoadFailed(filePath, QString::fromLocal8Bit("PLY 文件加载失败"));
            return;
        }

        pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

        emit logMessage(QStringLiteral("PLY file loaded successfully. The mesh contains %1 polygons.")
                        .arg(mesh.polygons.size()));

        // 若 PLY 含有效三角面，则优先走 mesh 通道，让前端按实体网格显示。
        if(!mesh.polygons.empty())
        {
            const auto end = std::chrono::high_resolution_clock::now();
            const std::chrono::duration<double> duration = end - start;
            emit meshLoadSucceeded(mesh, filePath, fileExtension, duration.count());
            return;
        }
    }

    // 5) 统一检查输出是否有效
    if(cloud->points.empty())
    {
        emit fileLoadFailed(filePath, QString::fromLocal8Bit("no effective points"));
        return;
    }

    // 按无组织点云格式回填尺寸信息
    cloud->width = static_cast<std::uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = false;

    // 6) 回传成功结果（含耗时）
    const auto end = std::chrono::high_resolution_clock::now();
    const std::chrono::duration<double> duration = end - start;

    emit fileLoadSucceeded(cloud, filePath, fileExtension, duration.count());
}

void PointCloudWorker::shutdown()
{
    // 当前 worker 没有需要显式释放的硬件/文件句柄，
    // 保留该槽函数用于 ThreadManager 的统一关闭流程。
}
