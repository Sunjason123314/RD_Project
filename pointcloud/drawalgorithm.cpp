#include "drawalgorithm.h"

#include <pcl/conversions.h>

#include <cmath>
#include <limits>

namespace DrawAlgorithm {

PCLMesh buildSphereMesh(const Eigen::Vector3f& center, float radius, int latSteps, int lonSteps)
{
    pcl::PointCloud<pcl::PointXYZ> pts;
    std::vector<pcl::Vertices> polys;
    pts.reserve((latSteps + 1) * (lonSteps + 1));

    for(int i = 0; i <= latSteps; ++i)
    {
        const float v = static_cast<float>(i) / static_cast<float>(latSteps);
        const float theta = static_cast<float>(M_PI) * v;
        const float st = std::sin(theta);
        const float ct = std::cos(theta);
        for(int j = 0; j <= lonSteps; ++j)
        {
            const float u = static_cast<float>(j) / static_cast<float>(lonSteps);
            const float phi = static_cast<float>(2.0 * M_PI) * u;
            const float sp = std::sin(phi);
            const float cp = std::cos(phi);
            pts.emplace_back(center.x() + radius * st * cp,
                             center.y() + radius * st * sp,
                             center.z() + radius * ct);
        }
    }

    const int row = lonSteps + 1;
    for(int i = 0; i < latSteps; ++i)
    {
        for(int j = 0; j < lonSteps; ++j)
        {
            const int a = i * row + j;
            const int b = a + 1;
            const int c = (i + 1) * row + j;
            const int d = c + 1;
            pcl::Vertices t1; t1.vertices = {static_cast<int>(a), static_cast<int>(c), static_cast<int>(b)};
            pcl::Vertices t2; t2.vertices = {static_cast<int>(b), static_cast<int>(c), static_cast<int>(d)};
            polys.push_back(t1);
            polys.push_back(t2);
        }
    }

    PCLMesh mesh;
    pcl::toPCLPointCloud2(pts, mesh.cloud);
    mesh.polygons = std::move(polys);
    return mesh;
}

PCLMesh buildCylinderMesh(const Eigen::Vector3f& axisPoint,
                                         Eigen::Vector3f axisDir,
                                         float radius,
                                         const PCLCloud::Ptr& refCloud,
                                         int hSteps,
                                         int radialSteps)
{
    axisDir.normalize();
    Eigen::Vector3f helper = std::fabs(axisDir.z()) < 0.9f ? Eigen::Vector3f(0, 0, 1) : Eigen::Vector3f(0, 1, 0);
    Eigen::Vector3f u = axisDir.cross(helper).normalized();
    Eigen::Vector3f v = axisDir.cross(u).normalized();

    float minT = -radius;
    float maxT = radius;
    if(refCloud && !refCloud->empty())
    {
        minT = std::numeric_limits<float>::max();
        maxT = -std::numeric_limits<float>::max();
        for(const auto& p : refCloud->points)
        {
            const Eigen::Vector3f q(p.x, p.y, p.z);
            const float t = (q - axisPoint).dot(axisDir);
            minT = std::min(minT, t);
            maxT = std::max(maxT, t);
        }
        if(!std::isfinite(minT) || !std::isfinite(maxT) || maxT - minT < 1e-4f)
        {
            minT = -radius;
            maxT = radius;
        }
    }

    pcl::PointCloud<pcl::PointXYZ> pts;
    std::vector<pcl::Vertices> polys;
    pts.reserve((hSteps + 1) * (radialSteps + 1));

    for(int i = 0; i <= hSteps; ++i)
    {
        const float t = minT + (maxT - minT) * (static_cast<float>(i) / static_cast<float>(hSteps));
        const Eigen::Vector3f c = axisPoint + axisDir * t;
        for(int j = 0; j <= radialSteps; ++j)
        {
            const float ang = static_cast<float>(2.0 * M_PI) * (static_cast<float>(j) / static_cast<float>(radialSteps));
            const Eigen::Vector3f p = c + radius * (std::cos(ang) * u + std::sin(ang) * v);
            pts.emplace_back(p.x(), p.y(), p.z());
        }
    }

    const int row = radialSteps + 1;
    for(int i = 0; i < hSteps; ++i)
    {
        for(int j = 0; j < radialSteps; ++j)
        {
            const int a = i * row + j;
            const int b = a + 1;
            const int c = (i + 1) * row + j;
            const int d = c + 1;
            pcl::Vertices t1; t1.vertices = {static_cast<int>(a), static_cast<int>(c), static_cast<int>(b)};
            pcl::Vertices t2; t2.vertices = {static_cast<int>(b), static_cast<int>(c), static_cast<int>(d)};
            polys.push_back(t1);
            polys.push_back(t2);
        }
    }

    PCLMesh mesh;
    pcl::toPCLPointCloud2(pts, mesh.cloud);
    mesh.polygons = std::move(polys);
    return mesh;
}

PCLMesh buildPlaneMesh(const Eigen::Vector4f& coeff,
                                      const PCLCloud::Ptr& refCloud,
                                      int gridN)
{
    Eigen::Vector3f n(coeff.x(), coeff.y(), coeff.z());
    n.normalize();
    Eigen::Vector3f helper = std::fabs(n.z()) < 0.9f ? Eigen::Vector3f(0, 0, 1) : Eigen::Vector3f(0, 1, 0);
    Eigen::Vector3f u = n.cross(helper).normalized();
    Eigen::Vector3f v = n.cross(u).normalized();
    const Eigen::Vector3f origin = -coeff.w() * n;

    float minU = -10.f, maxU = 10.f, minV = -10.f, maxV = 10.f;
    if(refCloud && !refCloud->empty())
    {
        minU = minV = std::numeric_limits<float>::max();
        maxU = maxV = -std::numeric_limits<float>::max();
        for(const auto& p : refCloud->points)
        {
            const Eigen::Vector3f q(p.x, p.y, p.z);
            const Eigen::Vector3f r = q - origin;
            const float pu = r.dot(u);
            const float pv = r.dot(v);
            minU = std::min(minU, pu); maxU = std::max(maxU, pu);
            minV = std::min(minV, pv); maxV = std::max(maxV, pv);
        }
        const float padU = std::max((maxU - minU) * 0.08f, 1.0f);
        const float padV = std::max((maxV - minV) * 0.08f, 1.0f);
        minU -= padU; maxU += padU;
        minV -= padV; maxV += padV;
    }

    pcl::PointCloud<pcl::PointXYZ> pts;
    std::vector<pcl::Vertices> polys;
    pts.reserve((gridN + 1) * (gridN + 1));
    for(int i = 0; i <= gridN; ++i)
    {
        const float su = static_cast<float>(i) / static_cast<float>(gridN);
        const float uu = minU + (maxU - minU) * su;
        for(int j = 0; j <= gridN; ++j)
        {
            const float sv = static_cast<float>(j) / static_cast<float>(gridN);
            const float vv = minV + (maxV - minV) * sv;
            const Eigen::Vector3f p = origin + uu * u + vv * v;
            pts.emplace_back(p.x(), p.y(), p.z());
        }
    }

    const int row = gridN + 1;
    for(int i = 0; i < gridN; ++i)
    {
        for(int j = 0; j < gridN; ++j)
        {
            const int a = i * row + j;
            const int b = a + 1;
            const int c = (i + 1) * row + j;
            const int d = c + 1;
            pcl::Vertices t1; t1.vertices = {static_cast<int>(a), static_cast<int>(c), static_cast<int>(b)};
            pcl::Vertices t2; t2.vertices = {static_cast<int>(b), static_cast<int>(c), static_cast<int>(d)};
            polys.push_back(t1);
            polys.push_back(t2);
        }
    }

    PCLMesh mesh;
    pcl::toPCLPointCloud2(pts, mesh.cloud);
    mesh.polygons = std::move(polys);
    return mesh;
}

PCLMesh buildCircleDiskMesh(const Eigen::Vector3f& center,
                                           Eigen::Vector3f normal,
                                           float radius,
                                           int radialSteps,
                                           int ringSteps)
{
    normal.normalize();
    Eigen::Vector3f helper = std::fabs(normal.z()) < 0.9f ? Eigen::Vector3f(0, 0, 1) : Eigen::Vector3f(0, 1, 0);
    Eigen::Vector3f u = normal.cross(helper).normalized();
    Eigen::Vector3f v = normal.cross(u).normalized();

    pcl::PointCloud<pcl::PointXYZ> pts;
    std::vector<pcl::Vertices> polys;
    pts.reserve(1 + ringSteps * (radialSteps + 1));
    pts.emplace_back(center.x(), center.y(), center.z());

    for(int r = 1; r <= ringSteps; ++r)
    {
        const float rr = radius * (static_cast<float>(r) / static_cast<float>(ringSteps));
        for(int j = 0; j <= radialSteps; ++j)
        {
            const float ang = static_cast<float>(2.0 * M_PI) * (static_cast<float>(j) / static_cast<float>(radialSteps));
            const Eigen::Vector3f p = center + rr * (std::cos(ang) * u + std::sin(ang) * v);
            pts.emplace_back(p.x(), p.y(), p.z());
        }
    }

    const int row = radialSteps + 1;
    for(int j = 0; j < radialSteps; ++j)
    {
        pcl::Vertices t;
        t.vertices = {0u, static_cast<int>(1 + j), static_cast<int>(1 + j + 1)};
        polys.push_back(t);
    }

    for(int r = 1; r < ringSteps; ++r)
    {
        const int base0 = 1 + (r - 1) * row;
        const int base1 = 1 + r * row;
        for(int j = 0; j < radialSteps; ++j)
        {
            const int a = base0 + j;
            const int b = a + 1;
            const int c = base1 + j;
            const int d = c + 1;
            pcl::Vertices t1; t1.vertices = {static_cast<int>(a), static_cast<int>(c), static_cast<int>(b)};
            pcl::Vertices t2; t2.vertices = {static_cast<int>(b), static_cast<int>(c), static_cast<int>(d)};
            polys.push_back(t1);
            polys.push_back(t2);
        }
    }

    PCLMesh mesh;
    pcl::toPCLPointCloud2(pts, mesh.cloud);
    mesh.polygons = std::move(polys);
    return mesh;
}

PCLCloud::Ptr buildAxisLine(const Eigen::Vector3f& center,
                                           Eigen::Vector3f direction,
                                           const PCLCloud::Ptr& refCloud,
                                           int samples)
{
    PCLCloud::Ptr axisLine(new PCLCloud);
    if(samples < 2)
        samples = 2;

    if(direction.norm() < 1e-6f)
        direction = Eigen::Vector3f::UnitZ();
    direction.normalize();

    float minT = -50.f;
    float maxT = 50.f;
    if(refCloud && !refCloud->empty())
    {
        minT = std::numeric_limits<float>::max();
        maxT = -std::numeric_limits<float>::max();
        for(const auto& p : refCloud->points)
        {
            const Eigen::Vector3f q(p.x, p.y, p.z);
            const float t = (q - center).dot(direction);
            minT = std::min(minT, t);
            maxT = std::max(maxT, t);
        }
        if(!std::isfinite(minT) || !std::isfinite(maxT) || maxT - minT < 1e-4f)
        {
            minT = -50.f;
            maxT = 50.f;
        }
        const float pad = std::max((maxT - minT) * 0.15f, 2.0f);
        minT -= pad;
        maxT += pad;
    }

    axisLine->points.reserve(samples);
    for(int i = 0; i < samples; ++i)
    {
        const float r = static_cast<float>(i) / static_cast<float>(samples - 1);
        const float t = minT + (maxT - minT) * r;
        const Eigen::Vector3f p = center + direction * t;
        axisLine->points.emplace_back(p.x(), p.y(), p.z());
    }
    axisLine->width = static_cast<uint32_t>(axisLine->points.size());
    axisLine->height = 1;
    axisLine->is_dense = true;
    return axisLine;
}

RunResult reconSphereInliers(const PCLCloud::Ptr& input,
                                                           const QString& baseName,
                                                           double distanceThreshold)
{
    RunResult out;
    out.fitKey = QStringLiteral("sphere");
    if(!input || input->size() < 10)
        return out;

    const auto r = Point_Algorithm::fitSphere3D(input, distanceThreshold);
    if(!r.success || !r.inliers || r.inliers->empty())
        return out;

    GeometryItem item;
    item.geometryType = 1;
    item.name = baseName + QStringLiteral("_ReconBall");
    item.displayTag = QStringLiteral("recon_sphere_inliers");
    item.axisLine = r.inliers;

    out.geometries.push_back(item);
    out.logText = QObject::tr("ReconBall %1: inliers=%2, center=(%3,%4,%5), r=%6")
                  .arg(baseName)
                  .arg(static_cast<int>(r.inliers->size()))
                  .arg(r.center.x(), 0, 'f', 3)
                  .arg(r.center.y(), 0, 'f', 3)
                  .arg(r.center.z(), 0, 'f', 3)
                  .arg(r.radius, 0, 'f', 3);
    out.popupText = QObject::tr("Sphere fit succeeded.\nCenter: (%1, %2, %3)\nRadius: %4")
                    .arg(r.center.x(), 0, 'f', 6)
                    .arg(r.center.y(), 0, 'f', 6)
                    .arg(r.center.z(), 0, 'f', 6)
                    .arg(r.radius, 0, 'f', 6);
    out.success = true;
    return out;
}

RunResult fitByKey(const QString& fitKey,
                                                 const PCLCloud::Ptr& input,
                                                 const QString& baseName)
{
    RunResult out;
    out.fitKey = fitKey;
    if(!input || input->empty())
        return out;

    if(fitKey == QStringLiteral("sphere"))
    {
        const auto r = Point_Algorithm::fitSphere3D(input, 0.5);
        if(!r.success)
            return out;

        GeometryItem item;
        item.geometryType = 0;
        item.mesh = buildSphereMesh(r.center, r.radius, 72, 144);
        item.name = baseName + QStringLiteral("_FitSphere");
        item.displayTag = QStringLiteral("sphere");
        out.geometries.push_back(item);

        out.logText = QObject::tr("Sphere fit: center=(%1,%2,%3), r=%4")
                      .arg(r.center.x(), 0, 'f', 3)
                      .arg(r.center.y(), 0, 'f', 3)
                      .arg(r.center.z(), 0, 'f', 3)
                      .arg(r.radius, 0, 'f', 3);
        out.popupText = QObject::tr("Sphere fit succeeded.\nCenter: (%1, %2, %3)\nRadius: %4")
                        .arg(r.center.x(), 0, 'f', 6)
                        .arg(r.center.y(), 0, 'f', 6)
                        .arg(r.center.z(), 0, 'f', 6)
                        .arg(r.radius, 0, 'f', 6);
        out.success = !item.mesh.polygons.empty();
        return out;
    }

    if(fitKey == QStringLiteral("cylinder"))
    {
        const auto r = Point_Algorithm::fitCylinder3D(input, 0.1, 2000, 0.5, 0.1f, 1e6f);
        if(!r.success)
            return out;

        GeometryItem item;
        item.geometryType = 0;
        item.mesh = buildCylinderMesh(r.axisPoint, r.axisDirection, r.radius, input, 96, 120);
        item.name = baseName + QStringLiteral("_FitCylinder");
        item.displayTag = QStringLiteral("cylinder");
        out.geometries.push_back(item);

        out.logText = QObject::tr("Cylinder fit: axisPoint=(%1,%2,%3), axisDir=(%4,%5,%6), r=%7")
                      .arg(r.axisPoint.x(), 0, 'f', 3)
                      .arg(r.axisPoint.y(), 0, 'f', 3)
                      .arg(r.axisPoint.z(), 0, 'f', 3)
                      .arg(r.axisDirection.x(), 0, 'f', 3)
                      .arg(r.axisDirection.y(), 0, 'f', 3)
                      .arg(r.axisDirection.z(), 0, 'f', 3)
                      .arg(r.radius, 0, 'f', 3);
        out.success = !item.mesh.polygons.empty();
        return out;
    }

    if(fitKey == QStringLiteral("plane"))
    {
        const auto r = Point_Algorithm::fitPlane3D(input, 0.5);
        if(!r.success)
            return out;

        GeometryItem item;
        item.geometryType = 0;
        item.mesh = buildPlaneMesh(r.coefficients, input, 80);
        item.name = baseName + QStringLiteral("_FitPlane");
        item.displayTag = QStringLiteral("plane");
        out.geometries.push_back(item);

        out.logText = QObject::tr("Plane fit: [%1x + %2y + %3z + %4 = 0]")
                      .arg(r.coefficients.x(), 0, 'f', 6)
                      .arg(r.coefficients.y(), 0, 'f', 6)
                      .arg(r.coefficients.z(), 0, 'f', 6)
                      .arg(r.coefficients.w(), 0, 'f', 6);
        out.success = !item.mesh.polygons.empty();
        return out;
    }

    if(fitKey == QStringLiteral("circle3d"))
    {
        const auto r = Point_Algorithm::fitCircle3D(input, 2000, 0.5, 0.1f, 1e6f);
        if(!r.success)
            return out;

        GeometryItem disk;
        disk.geometryType = 0;
        disk.mesh = buildCircleDiskMesh(r.center, r.normal, r.radius, 180, 32);
        disk.name = baseName + QStringLiteral("_FitCircle3D");
        disk.displayTag = QStringLiteral("circle3d");
        out.geometries.push_back(disk);

        GeometryItem axis;
        axis.geometryType = 1;
        axis.axisLine = buildAxisLine(r.center, r.normal, input, 100);
        axis.name = baseName + QStringLiteral("_RotationAxis");
        axis.displayTag = QStringLiteral("rotation_axis");
        out.geometries.push_back(axis);

        out.logText = QObject::tr("Circle3D fit: center=(%1,%2,%3), normal=(%4,%5,%6), r=%7")
                      .arg(r.center.x(), 0, 'f', 3)
                      .arg(r.center.y(), 0, 'f', 3)
                      .arg(r.center.z(), 0, 'f', 3)
                      .arg(r.normal.x(), 0, 'f', 3)
                      .arg(r.normal.y(), 0, 'f', 3)
                      .arg(r.normal.z(), 0, 'f', 3)
                      .arg(r.radius, 0, 'f', 3);
        out.success = !disk.mesh.polygons.empty() && axis.axisLine && !axis.axisLine->empty();
        return out;
    }

    return out;
}

} // namespace DrawAlgorithm
