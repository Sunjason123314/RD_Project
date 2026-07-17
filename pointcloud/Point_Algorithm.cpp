#include "Point_Algorithm.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/conversions.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/on_nurbs/fitting_surface_pdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>
#include <pcl/surface/on_nurbs/nurbs_data.h>
#include <pcl/search/kdtree.h>

#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkCleanPolyData.h>
#include <vtkDecimatePro.h>
#include <vtkQuadricDecimation.h>

#include <array>

#include <cmath>
#include <limits>
#include <random>
#include <chrono>
#include <Eigen/SVD>

namespace {

// 计算点云AABB包围盒，用于后续采样尺度估计。

static inline void computeCloudBounds(const PCLCloud::Ptr& cloud,
                                      Eigen::Vector3f& minPt,
                                      Eigen::Vector3f& maxPt)
{
    minPt = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
    maxPt = Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());

    if(!cloud || cloud->empty())
        return;

    for(const auto& p : cloud->points)
    {
        minPt.x() = std::min(minPt.x(), p.x);
        minPt.y() = std::min(minPt.y(), p.y);
        minPt.z() = std::min(minPt.z(), p.z);
        maxPt.x() = std::max(maxPt.x(), p.x);
        maxPt.y() = std::max(maxPt.y(), p.y);
        maxPt.z() = std::max(maxPt.z(), p.z);
    }
}

// 由法向构建一组正交基(u,v,n)，用于在平面/圆周上采样。
static inline void buildOrthonormalBasis(const Eigen::Vector3f& n,
                                         Eigen::Vector3f& u,
                                         Eigen::Vector3f& v)
{
    Eigen::Vector3f normal = n.normalized();
    const Eigen::Vector3f ref = (std::abs(normal.z()) < 0.9f)
                               ? Eigen::Vector3f::UnitZ()
                               : Eigen::Vector3f::UnitX();
    u = normal.cross(ref).normalized();
    v = normal.cross(u).normalized();
}

}

// 体素下采样：每个体素保留代表点，降低点数并平滑高频噪声。
PCLCloud::Ptr Point_Algorithm::voxelGridDownsample(const PCLCloud::Ptr& input,
                                                   float leafX,
                                                   float leafY,
                                                   float leafZ)
{
    PCLCloud::Ptr output(new PCLCloud);
    if(!input || input->empty())
        return output;

    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(input);
    voxel.setLeafSize(leafX, leafY, leafZ);
    voxel.filter(*output);
    return output;
}

// SOR滤波：依据邻域统计分布剔除离群点。
PCLCloud::Ptr Point_Algorithm::sorFilter(const PCLCloud::Ptr& input,
                                         int meanK,
                                         double stddevMulThresh)
{
    PCLCloud::Ptr output(new PCLCloud);
    if(!input || input->empty())
        return output;

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(stddevMulThresh);
    sor.filter(*output);
    return output;
}

// MLS 移动最小二乘平滑重采样。
PCLCloud::Ptr Point_Algorithm::mlsSmoothing(const PCLCloud::Ptr& input,
                                             double searchRadius,
                                             int polynomialOrder)
{
    PCLCloud::Ptr output(new PCLCloud);
    if(!input || input->empty())
        return output;

    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setInputCloud(input);
    mls.setSearchRadius(searchRadius);
    mls.setPolynomialOrder(polynomialOrder);
    mls.setComputeNormals(false);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    mls.setSearchMethod(tree);
    mls.process(*output);

    return output;
}

// NURBS 曲面重建。
PCLMesh Point_Algorithm::nurbsReconstruction(const PCLCloud::Ptr& input,
                                               int order,
                                               int refinement,
                                               int resolution)
{
    PCLMesh output;
    if(!input || input->size() < 10)
        return output;

    // 1. 体素下采样降点，加速拟合（叶大小自适应点云尺度）
    //    先计算 bounding box 以确定合适的 leaf size
    Eigen::Vector3f minPt = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
    Eigen::Vector3f maxPt = Eigen::Vector3f::Constant(-std::numeric_limits<float>::max());
    for(const auto& p : input->points)
    {
        minPt = minPt.cwiseMin(Eigen::Vector3f(p.x, p.y, p.z));
        maxPt = maxPt.cwiseMax(Eigen::Vector3f(p.x, p.y, p.z));
    }
    const float scale = (maxPt - minPt).norm();
    const float leaf = std::max(scale * 0.002f, 1e-4f);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(input);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter(*cloud);
    if(cloud->size() < 10)
        *cloud = *input;

    // 2. PCA → 点云变换到 XY 平面 (最小特征值方向为法向 Z)
    pcl::on_nurbs::NurbsDataSurface data;
    data.interior.reserve(cloud->size());
    for(const auto& p : cloud->points)
        data.interior.emplace_back(
            static_cast<double>(p.x),
            static_cast<double>(p.y),
            static_cast<double>(p.z));

    // 3. 计算 bounding box 并生成边界点
    Eigen::Vector3d bMin = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
    Eigen::Vector3d bMax = Eigen::Vector3d::Constant(-std::numeric_limits<double>::max());
    for(const auto& p : data.interior)
    {
        bMin = bMin.cwiseMin(p);
        bMax = bMax.cwiseMax(p);
    }

    const Eigen::Vector3d center = 0.5 * (bMin + bMax);
    const Eigen::Vector3d span = bMax - bMin;
    // 将点云中心化，让 PCA 在原点附近工作
    data.mean = center;
    for(auto& p : data.interior)
        p -= center;

    // 用 initNurbsPCA 构造初始 NURBS 曲面
    ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox(order, &data);

    // 4. 生成边界点（四个边的采样点，使用投影到 XY 平面后的 bounding box）
    //    先做 PCA 投影，获取投影后坐标的 min/max
    bMin = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
    bMax = Eigen::Vector3d::Constant(-std::numeric_limits<double>::max());
    for(const auto& p : data.interior)
    {
        bMin = bMin.cwiseMin(p);
        bMax = bMax.cwiseMax(p);
    }
    const double u0 = bMin.x(), u1 = bMax.x();
    const double v0 = bMin.y(), v1 = bMax.y();

    const int nBnd = 8;
    data.boundary.reserve(static_cast<std::size_t>(nBnd * 4));
    for(int i = 0; i < nBnd; ++i)
    {
        const double t = static_cast<double>(i) / static_cast<double>(nBnd - 1);
        data.boundary.emplace_back(u0, v0 + t * (v1 - v0), 0.0);
        data.boundary.emplace_back(u1, v0 + t * (v1 - v0), 0.0);
        data.boundary.emplace_back(u0 + t * (u1 - u0), v0, 0.0);
        data.boundary.emplace_back(u0 + t * (u1 - u0), v1, 0.0);
    }
    data.boundary_weight.assign(data.boundary.size(), 1.0);

    data.interior_weight.assign(data.interior.size(), 1.0);

    // 5. 使用 PDM 拟合器迭代拟合
    pcl::on_nurbs::FittingSurface fit(order, &data);
    fit.setQuiet(true);
    for(int iter = 0; iter < refinement; ++iter)
    {
        fit.assemble();
        fit.solve();
    }

    // 6. 三角化 NURBS 曲面
    pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(fit.m_nurbs, output,
        static_cast<unsigned>(resolution));

    // 7. 将结果顶点平移回原始坐标系
    if(!output.cloud.data.empty() && !output.polygons.empty())
    {
        // 使用字段偏移直接读写，避免 fromPCLPointCloud2 字段名不匹配问题
        int xIdx = pcl::getFieldIndex(output.cloud, "x");
        int yIdx = pcl::getFieldIndex(output.cloud, "y");
        int zIdx = pcl::getFieldIndex(output.cloud, "z");
        if(xIdx >= 0 && yIdx >= 0 && zIdx >= 0)
        {
            const std::size_t pointStep = output.cloud.point_step;
            for(std::size_t i = 0; i < output.cloud.width * output.cloud.height; ++i)
            {
                float* px = reinterpret_cast<float*>(&output.cloud.data[i * pointStep] +
                    output.cloud.fields[xIdx].offset);
                float* py = reinterpret_cast<float*>(&output.cloud.data[i * pointStep] +
                    output.cloud.fields[yIdx].offset);
                float* pz = reinterpret_cast<float*>(&output.cloud.data[i * pointStep] +
                    output.cloud.fields[zIdx].offset);
                *px = static_cast<float>(static_cast<double>(*px) + center.x());
                *py = static_cast<float>(static_cast<double>(*py) + center.y());
                *pz = static_cast<float>(static_cast<double>(*pz) + center.z());
            }
        }
    }

    return output;
}

// 平面拟合：RANSAC + 平面模型，输出系数与内点。
Point_Algorithm::PlaneFitResult Point_Algorithm::fitPlane3D(const PCLCloud::Ptr& input, double distanceThreshold)
{
    PlaneFitResult result;
    if(!input || input->size() < 3)
        return result;

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setMaxIterations(2000);
    seg.setInputCloud(input);
    seg.segment(*inliers, *coeff);

    if(inliers->indices.empty() || coeff->values.size() < 4)
        return result;

    result.coefficients = Eigen::Vector4f(coeff->values[0], coeff->values[1], coeff->values[2], coeff->values[3]);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input);
    extract.setIndices(inliers);
    extract.filter(*result.inliers);

    result.success = !result.inliers->empty();
    return result;
}

// 球拟合：RANSAC + 球模型，输出球心、半径与内点。
Point_Algorithm::SphereFitResult Point_Algorithm::fitSphere3D(const PCLCloud::Ptr& input, double distanceThreshold)
{
    SphereFitResult result;
    if(!input || input->size() < 4)
        return result;

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_SPHERE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setMaxIterations(3000);
    seg.setInputCloud(input);
    seg.segment(*inliers, *coeff);

    if(inliers->indices.empty() || coeff->values.size() < 4)
        return result;

    result.center = Eigen::Vector3f(coeff->values[0], coeff->values[1], coeff->values[2]);
    result.radius = coeff->values[3];

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input);
    extract.setIndices(inliers);
    extract.filter(*result.inliers);

    result.success = result.radius > 0.0f && !result.inliers->empty();
    return result;
}

// 最小二乘球拟合：通过线性方程组求解球心和半径。
// 输入：3×N 矩阵，每列是一个点 [x; y; z]
// 方法：将球方程 (x-cx)² + (y-cy)² + (z-cz)² = r² 展开为线性形式，
//      构造超定方程 A·[cx cy cz d]ᵀ = b，其中 d = cx²+cy²+cz²-r²
void Point_Algorithm::fitSphereLeastSquares(const Eigen::MatrixXd& points,
                                            Eigen::Vector3d& center,
                                            double& radius)
{
    const int N = static_cast<int>(points.cols());
    if(N < 4)
    {
        center = Eigen::Vector3d::Zero();
        radius = 0.0;
        return;
    }

    Eigen::MatrixXd A(N, 4);
    Eigen::VectorXd b(N);
    for(int i = 0; i < N; ++i)
    {
        const double x = points(0, i);
        const double y = points(1, i);
        const double z = points(2, i);
        A(i, 0) = 2.0 * x;
        A(i, 1) = 2.0 * y;
        A(i, 2) = 2.0 * z;
        A(i, 3) = 1.0;
        b(i) = x * x + y * y + z * z;
    }

    const Eigen::Vector4d params = A.colPivHouseholderQr().solve(b);
    center = params.head<3>();
    radius = std::sqrt(params(3) + center.squaredNorm());
}

// Kabsch-Umeyama SVD 刚性变换：最小化 sum || R*src_i + t - dst_i ||^2
Point_Algorithm::RigidTransformResult Point_Algorithm::estimateRigidTransformSVD(
    const std::vector<Eigen::Vector3f>& src,
    const std::vector<Eigen::Vector3f>& dst)
{
    RigidTransformResult result;
    const size_t N = src.size();
    if(N < 3 || N != dst.size())
        return result;

    // 1. 计算质心
    Eigen::Vector3f srcCtr = Eigen::Vector3f::Zero();
    Eigen::Vector3f dstCtr = Eigen::Vector3f::Zero();
    for(size_t i = 0; i < N; ++i) {
        srcCtr += src[i];
        dstCtr += dst[i];
    }
    srcCtr /= static_cast<float>(N);
    dstCtr /= static_cast<float>(N);

    // 2. 构建协方差矩阵 H = sum( (src_i - srcCtr) * (dst_i - dstCtr)^T )
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    for(size_t i = 0; i < N; ++i) {
        const Eigen::Vector3f s = src[i] - srcCtr;
        const Eigen::Vector3f d = dst[i] - dstCtr;
        H += s * d.transpose();
    }

    // 3. SVD
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();

    // 4. 反射检测
    if(R.determinant() < 0.0f) {
        Eigen::Matrix3f V = svd.matrixV();
        V.col(2) = -V.col(2);
        R = V * svd.matrixU().transpose();
    }

    // 5. 平移
    const Eigen::Vector3f t = dstCtr - R * srcCtr;

    // 6. 构建 4x4 变换矩阵
    result.transformation.setIdentity();
    result.transformation.block<3,3>(0,0) = R;
    result.transformation.block<3,1>(0,3) = t;

    // 7. RMS 误差
    float rms = 0.0f;
    for(size_t i = 0; i < N; ++i) {
        const Eigen::Vector3f transformed = R * src[i] + t;
        rms += (transformed - dst[i]).squaredNorm();
    }
    result.rmsError = std::sqrt(rms / static_cast<float>(N));
    result.success = true;
    return result;
}

// 圆柱拟合：先估计法向，再使用带法向约束的RANSAC圆柱模型。
Point_Algorithm::CylinderFitResult Point_Algorithm::fitCylinder3D(const PCLCloud::Ptr& input,
                                                                   double normalDistanceWeight,
                                                                   int maxIterations,
                                                                   double distanceThreshold,
                                                                   float minRadius,
                                                                   float maxRadius)
{
    CylinderFitResult result;
    if(!input || input->size() < 20)
        return result;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(input);
    ne.setKSearch(30);
    ne.compute(*normals);

    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(normalDistanceWeight);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setRadiusLimits(minRadius, maxRadius);
    seg.setInputCloud(input);
    seg.setInputNormals(normals);
    seg.segment(*inliers, *coeff);

    if(inliers->indices.empty() || coeff->values.size() < 7)
        return result;

    result.axisPoint = Eigen::Vector3f(coeff->values[0], coeff->values[1], coeff->values[2]);
    result.axisDirection = Eigen::Vector3f(coeff->values[3], coeff->values[4], coeff->values[5]).normalized();
    result.radius = coeff->values[6];

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input);
    extract.setIndices(inliers);
    extract.filter(*result.inliers);

    result.success = result.radius > 0.0f && result.axisDirection.norm() > 0.0f && !result.inliers->empty();
    return result;
}

// 三维圆拟合：RANSAC圆模型，输出圆心、法向、半径与内点。
Point_Algorithm::Circle3DFitResult Point_Algorithm::fitCircle3D(const PCLCloud::Ptr& input,
                                                                 int maxIterations,
                                                                 double distanceThreshold,
                                                                 float minRadius,
                                                                 float maxRadius)
{
    Circle3DFitResult result;
    if(!input || input->size() < 6)
        return result;

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setRadiusLimits(minRadius, maxRadius);
    seg.setInputCloud(input);
    seg.segment(*inliers, *coeff);

    if(inliers->indices.empty() || coeff->values.size() < 7)
        return result;

    result.center = Eigen::Vector3f(coeff->values[0], coeff->values[1], coeff->values[2]);
    result.radius = coeff->values[3];
    result.normal = Eigen::Vector3f(coeff->values[4], coeff->values[5], coeff->values[6]).normalized();

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input);
    extract.setIndices(inliers);
    extract.filter(*result.inliers);

    result.success = result.radius > 0.0f && result.normal.norm() > 0.0f && !result.inliers->empty();
    return result;
}

// 多球心回转轴拟合：先拟合点集平面，再在平面内拟合圆，法向即回转轴方向。
Point_Algorithm::RotationAxisFitResult Point_Algorithm::fitRotationAxisFromCenters(const std::vector<Eigen::Vector3f>& centers)
{
    RotationAxisFitResult result;

    const int N = static_cast<int>(centers.size());
    if(N < 3)
        return result;

    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for(const auto& p : centers)
        centroid += p;
    centroid /= static_cast<float>(N);

    Eigen::MatrixXf Q(N, 3);
    for(int i = 0; i < N; ++i)
        Q.row(i) = (centers[static_cast<std::size_t>(i)] - centroid).transpose();

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(Q, Eigen::ComputeThinU | Eigen::ComputeThinV);
    if(svd.matrixV().cols() < 3)
        return result;

    Eigen::Vector3f e1 = svd.matrixV().col(0);
    Eigen::Vector3f e2 = svd.matrixV().col(1);
    Eigen::Vector3f axisDir = svd.matrixV().col(2);
    if(axisDir.norm() < 1e-6f)
        return result;
    axisDir.normalize();

    Eigen::VectorXf planeDist = Q * axisDir;
    result.planeRMSE = std::sqrt(planeDist.array().square().mean());

    Eigen::VectorXf x = Q * e1;
    Eigen::VectorXf y = Q * e2;

    Eigen::Vector2f center2D(0.f, 0.f);
    float radius = 0.f;

    if(N == 3)
    {
        const float x1 = x(0), y1 = y(0);
        const float x2 = x(1), y2 = y(1);
        const float x3 = x(2), y3 = y(2);

        const float d = 2.0f * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
        if(std::fabs(d) < 1e-8f)
            return result;

        const float x1y1 = x1 * x1 + y1 * y1;
        const float x2y2 = x2 * x2 + y2 * y2;
        const float x3y3 = x3 * x3 + y3 * y3;

        center2D.x() = (x1y1 * (y2 - y3) + x2y2 * (y3 - y1) + x3y3 * (y1 - y2)) / d;
        center2D.y() = (x1y1 * (x3 - x2) + x2y2 * (x1 - x3) + x3y3 * (x2 - x1)) / d;

        radius = std::sqrt((x1 - center2D.x()) * (x1 - center2D.x()) +
                           (y1 - center2D.y()) * (y1 - center2D.y()));
    }
    else
    {
        Eigen::MatrixXf A(N, 3);
        Eigen::VectorXf b(N);
        for(int i = 0; i < N; ++i)
        {
            A(i, 0) = 2.0f * x(i);
            A(i, 1) = 2.0f * y(i);
            A(i, 2) = 1.0f;
            b(i) = x(i) * x(i) + y(i) * y(i);
        }

        const Eigen::Vector3f sol = A.colPivHouseholderQr().solve(b);
        center2D = sol.head<2>();
        const float c = sol(2);
        const float r2 = center2D.squaredNorm() + c;
        if(r2 <= 0.0f)
            return result;
        radius = std::sqrt(r2);
    }

    Eigen::VectorXf residuals(N);
    for(int i = 0; i < N; ++i)
    {
        const float dx = x(i) - center2D.x();
        const float dy = y(i) - center2D.y();
        residuals(i) = std::sqrt(dx * dx + dy * dy) - radius;
    }
    result.circleRMSE = std::sqrt(residuals.array().square().mean());

    result.axisPoint = centroid + center2D.x() * e1 + center2D.y() * e2;
    result.axisDirection = axisDir;
    result.radius = radius;

    result.circle3D->clear();
    const int sampleCount = 200;
    result.circle3D->points.reserve(static_cast<std::size_t>(sampleCount));
    for(int i = 0; i < sampleCount; ++i)
    {
        const float t = static_cast<float>(2.0 * M_PI) * static_cast<float>(i) / static_cast<float>(sampleCount - 1);
        const Eigen::Vector3f p3 = result.axisPoint + radius * std::cos(t) * e1 + radius * std::sin(t) * e2;
        result.circle3D->points.emplace_back(p3.x(), p3.y(), p3.z());
    }
    result.circle3D->width = static_cast<uint32_t>(result.circle3D->points.size());
    result.circle3D->height = 1;
    result.circle3D->is_dense = true;

    result.success = true;
    return result;
}

// 平面采样：根据参考点云范围，在拟合平面上生成规则网格点。
PCLCloud::Ptr Point_Algorithm::samplePlanePatch(const Eigen::Vector4f& plane,
                                                 const PCLCloud::Ptr& reference,
                                                 int resolution)
{
    PCLCloud::Ptr sampled(new PCLCloud);
    if(!reference || reference->empty() || resolution < 2)
        return sampled;

    Eigen::Vector3f n(plane.x(), plane.y(), plane.z());
    if(n.norm() < 1e-6f)
        return sampled;
    n.normalize();

    Eigen::Vector3f u, v;
    buildOrthonormalBasis(n, u, v);

    Eigen::Vector3f minPt, maxPt;
    computeCloudBounds(reference, minPt, maxPt);
    const Eigen::Vector3f center = 0.5f * (minPt + maxPt);
    const float span = (maxPt - minPt).norm() * 0.6f;

    const float a = plane.x();
    const float b = plane.y();
    const float c = plane.z();
    const float d = plane.w();
    const float z = std::abs(c) > 1e-6f ? (-(a * center.x() + b * center.y() + d) / c) : center.z();
    const Eigen::Vector3f p0(center.x(), center.y(), z);

    for(int i = 0; i < resolution; ++i)
    {
        const float t = -span + (2.0f * span * i) / static_cast<float>(resolution - 1);
        for(int j = 0; j < resolution; ++j)
        {
            const float s = -span + (2.0f * span * j) / static_cast<float>(resolution - 1);
            const Eigen::Vector3f p = p0 + u * s + v * t;
            sampled->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
        }
    }
    return sampled;
}

// 球面采样：经纬线离散生成球面点集。
PCLCloud::Ptr Point_Algorithm::sampleSphereSurface(const Eigen::Vector3f& center,
                                                    float radius,
                                                    int rings,
                                                    int segments)
{
    PCLCloud::Ptr sampled(new PCLCloud);
    if(radius <= 0.0f || rings < 3 || segments < 6)
        return sampled;

    for(int i = 0; i <= rings; ++i)
    {
        const float phi = static_cast<float>(M_PI) * static_cast<float>(i) / static_cast<float>(rings);
        const float z = radius * std::cos(phi);
        const float r = radius * std::sin(phi);
        for(int j = 0; j < segments; ++j)
        {
            const float theta = 2.0f * static_cast<float>(M_PI) * static_cast<float>(j) / static_cast<float>(segments);
            sampled->push_back(pcl::PointXYZ(center.x() + r * std::cos(theta),
                                             center.y() + r * std::sin(theta),
                                             center.z() + z));
        }
    }
    return sampled;
}

// 圆柱侧面采样：沿轴向+周向离散，生成可显示点云。
PCLCloud::Ptr Point_Algorithm::sampleCylinderSurface(const Eigen::Vector3f& axisPoint,
                                                      const Eigen::Vector3f& axisDirection,
                                                      float radius,
                                                      const PCLCloud::Ptr& reference,
                                                      int axisSteps,
                                                      int radialSteps)
{
    PCLCloud::Ptr sampled(new PCLCloud);
    if(!reference || reference->empty() || radius <= 0.0f || axisSteps < 2 || radialSteps < 6)
        return sampled;

    Eigen::Vector3f d = axisDirection.normalized();
    if(d.norm() < 1e-6f)
        return sampled;

    Eigen::Vector3f u, v;
    buildOrthonormalBasis(d, u, v);

    float tMin = std::numeric_limits<float>::max();
    float tMax = std::numeric_limits<float>::lowest();
    for(const auto& p : reference->points)
    {
        const Eigen::Vector3f q(p.x, p.y, p.z);
        const float t = d.dot(q - axisPoint);
        tMin = std::min(tMin, t);
        tMax = std::max(tMax, t);
    }
    if(tMax <= tMin)
        tMax = tMin + 1.0f;

    for(int i = 0; i < axisSteps; ++i)
    {
        const float t = tMin + (tMax - tMin) * static_cast<float>(i) / static_cast<float>(axisSteps - 1);
        const Eigen::Vector3f c = axisPoint + d * t;
        for(int j = 0; j < radialSteps; ++j)
        {
            const float ang = 2.0f * static_cast<float>(M_PI) * static_cast<float>(j) / static_cast<float>(radialSteps);
            const Eigen::Vector3f p = c + radius * (std::cos(ang) * u + std::sin(ang) * v);
            sampled->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
        }
    }
    return sampled;
}

// 圆周采样：在给定法向定义的平面内均匀取样。
PCLCloud::Ptr Point_Algorithm::sampleCircle3D(const Eigen::Vector3f& center,
                                               const Eigen::Vector3f& normal,
                                               float radius,
                                               int segments)
{
    PCLCloud::Ptr sampled(new PCLCloud);
    if(radius <= 0.0f || segments < 12)
        return sampled;

    Eigen::Vector3f u, v;
    buildOrthonormalBasis(normal, u, v);

    for(int i = 0; i < segments; ++i)
    {
        const float ang = 2.0f * static_cast<float>(M_PI) * static_cast<float>(i) / static_cast<float>(segments);
        const Eigen::Vector3f p = center + radius * (std::cos(ang) * u + std::sin(ang) * v);
        sampled->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
    }
    return sampled;
}

PCLCloud::Ptr Point_Algorithm::samplePointsFromMesh(const PCLMesh& mesh, int numPoints)
{
    PCLCloud::Ptr output(new PCLCloud);
    if(numPoints <= 0 || mesh.polygons.empty())
        return output;

    // 提取顶点坐标
    pcl::PointCloud<pcl::PointXYZ> vertices;
    pcl::fromPCLPointCloud2(mesh.cloud, vertices);
    if(vertices.empty())
        return output;

    const int nFaces = static_cast<int>(mesh.polygons.size());

    // Step 1: 计算每个三角面片的面积
    std::vector<float> faceAreas(nFaces);
    float totalArea = 0.0f;
    for(int i = 0; i < nFaces; ++i)
    {
        const auto& tri = mesh.polygons[i].vertices;
        if(tri.size() < 3)
        {
            faceAreas[i] = 0.0f;
            continue;
        }
        const int i0 = tri[0];
        const int i1 = tri[1];
        const int i2 = tri[2];
        if(i0 >= static_cast<int>(vertices.size()) ||
           i1 >= static_cast<int>(vertices.size()) ||
           i2 >= static_cast<int>(vertices.size()))
        {
            faceAreas[i] = 0.0f;
            continue;
        }
        const auto& p0 = vertices[i0];
        const auto& p1 = vertices[i1];
        const auto& p2 = vertices[i2];
        const Eigen::Vector3f v0(p0.x, p0.y, p0.z);
        const Eigen::Vector3f v1(p1.x, p1.y, p1.z);
        const Eigen::Vector3f v2(p2.x, p2.y, p2.z);
        const Eigen::Vector3f cross = (v1 - v0).cross(v2 - v0);
        faceAreas[i] = 0.5f * cross.norm();
        totalArea += faceAreas[i];
    }

    if(totalArea <= 0.0f)
        return output;

    // Step 2: 构建归一化累积面积分布
    std::vector<float> cumAreas(nFaces);
    float accum = 0.0f;
    for(int i = 0; i < nFaces; ++i)
    {
        accum += faceAreas[i] / totalArea;
        cumAreas[i] = accum;
    }
    // 确保最后一项因浮点误差抵达 1.0
    if(!cumAreas.empty())
        cumAreas.back() = 1.0f;

    // 随机种子
    std::mt19937 rng(static_cast<unsigned>(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    std::uniform_real_distribution<float> dist01(0.0f, 1.0f);

    output->points.reserve(static_cast<std::size_t>(numPoints));

    for(int k = 0; k < numPoints; ++k)
    {
        // 按面积加权随机选择面片 (二分查找)
        const float rv = dist01(rng);
        const auto it = std::lower_bound(cumAreas.begin(), cumAreas.end(), rv);
        int faceId = static_cast<int>(std::distance(cumAreas.begin(), it));
        if(faceId < 0) faceId = 0;
        if(faceId >= nFaces) faceId = nFaces - 1;

        // 跳过面积为 0 的面片
        if(faceAreas[faceId] <= 0.0f)
        {
            --k;
            continue;
        }

        const auto& tri = mesh.polygons[faceId].vertices;
        const auto& p0 = vertices[tri[0]];
        const auto& p1 = vertices[tri[1]];
        const auto& p2 = vertices[tri[2]];

        // Step 3: 生成三角形内均匀分布的随机重心坐标
        float r1 = dist01(rng);
        float r2 = dist01(rng);
        if(r1 + r2 > 1.0f)
        {
            r1 = 1.0f - r1;
            r2 = 1.0f - r2;
        }

        // Step 4: 计算采样点位置
        const float x = p0.x + r1 * (p1.x - p0.x) + r2 * (p2.x - p0.x);
        const float y = p0.y + r1 * (p1.y - p0.y) + r2 * (p2.y - p0.y);
        const float z = p0.z + r1 * (p1.z - p0.z) + r2 * (p2.z - p0.z);

        output->push_back(pcl::PointXYZ(x, y, z));
    }

    output->width = static_cast<uint32_t>(output->points.size());
    output->height = 1;
    output->is_dense = true;
    return output;
}

void Point_Algorithm::clipMeshToCloudBounds(PCLMesh& mesh, const PCLCloud::Ptr& cloud,
                                              float marginRatio)
{
    if(mesh.polygons.empty() || !cloud || cloud->empty())
        return;

    // 计算原始点云的包围盒
    Eigen::Vector3f bMin = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
    Eigen::Vector3f bMax = Eigen::Vector3f::Constant(-std::numeric_limits<float>::max());
    for(const auto& p : cloud->points)
    {
        bMin = bMin.cwiseMin(Eigen::Vector3f(p.x, p.y, p.z));
        bMax = bMax.cwiseMax(Eigen::Vector3f(p.x, p.y, p.z));
    }
    const Eigen::Vector3f margin = (bMax - bMin) * marginRatio;
    bMin -= margin;
    bMax += margin;

    // 提取网格顶点坐标
    pcl::PointCloud<pcl::PointXYZ> verts;
    pcl::fromPCLPointCloud2(mesh.cloud, verts);
    if(verts.empty())
        return;

    // 为每个顶点预计算是否在包围盒内
    std::vector<bool> inside(verts.size(), false);
    for(std::size_t i = 0; i < verts.size(); ++i)
    {
        const auto& v = verts[i];
        inside[i] = (v.x >= bMin.x() && v.x <= bMax.x() &&
                     v.y >= bMin.y() && v.y <= bMax.y() &&
                     v.z >= bMin.z() && v.z <= bMax.z());
    }

    // 筛选三角面片：三个顶点都在盒内才保留
    std::vector<pcl::Vertices> kept;
    kept.reserve(mesh.polygons.size());
    for(const auto& tri : mesh.polygons)
    {
        if(tri.vertices.size() >= 3 &&
           inside[tri.vertices[0]] &&
           inside[tri.vertices[1]] &&
           inside[tri.vertices[2]])
        {
            kept.push_back(tri);
        }
    }

    mesh.polygons.swap(kept);
}

pcl::PointCloud<pcl::Normal>::Ptr Point_Algorithm::estimateConsistentNormals(
    const PCLCloud::Ptr& cloud,
    int kSearch,
    float vpX, float vpY, float vpZ)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    if(!cloud || cloud->size() < 3)
        return normals;

    // 1. 估计法向
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(kSearch);
    ne.compute(*normals);

    // 2. 一致化法向朝向：翻转向着视点方向
    for(auto& n : normals->points)
    {
        // 视点方向向量 (从点指向视点)
        if(n.normal_x * vpX + n.normal_y * vpY + n.normal_z * vpZ > 0)
        {
            n.normal_x = -n.normal_x;
            n.normal_y = -n.normal_y;
            n.normal_z = -n.normal_z;
            n.curvature = n.curvature;
        }
    }

    return normals;
}

Point_Algorithm::PCAResult Point_Algorithm::pcaAnalysis(const PCLCloud::Ptr& cloud)
{
    PCAResult r;
    if(!cloud || cloud->size() < 3)
        return r;

    // 1. 计算质心
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for(const auto& p : cloud->points)
        centroid += Eigen::Vector3f(p.x, p.y, p.z);
    centroid /= static_cast<float>(cloud->size());

    // 2. 计算协方差矩阵
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for(const auto& p : cloud->points)
    {
        const Eigen::Vector3f d = Eigen::Vector3f(p.x, p.y, p.z) - centroid;
        cov += d * d.transpose();
    }
    cov /= static_cast<float>(cloud->size());

    // 3. 特征分解
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
    if(eig.info() != Eigen::Success)
        return r;

    // 特征值大到小排序（SelfAdjointEigenSolver 默认升序，翻转）
    r.eigenvalues = eig.eigenvalues().reverse();
    r.eigenvectors.col(0) = eig.eigenvectors().col(2);
    r.eigenvectors.col(1) = eig.eigenvectors().col(1);
    r.eigenvectors.col(2) = eig.eigenvectors().col(0);
    r.centroid = centroid;
    r.success = true;
    return r;
}

// 计算 NURBS surface triangulation 产生的三角面数估算值。
// NURBS resolution=N → (N-1)×(N-1) 个四边形 → 2*(N-1)*(N-1) 个三角形
int Point_Algorithm::numFacesFromNurbsResolution(int resolution)
{
    if (resolution < 2) return 0;
    return 2 * (resolution - 1) * (resolution - 1);
}

// 根据目标面数反推合适的 NURBS resolution，上限 300 面。
int Point_Algorithm::nurbsResolutionForTargetFaces(int targetFaces)
{
    if (targetFaces < 2) return 2;
    // resolution = floor(sqrt(targetFaces / 2)) + 1
    int res = static_cast<int>(std::sqrt(targetFaces / 2.0)) + 1;
    if (res < 2) res = 2;
    // 确保不超目标
    while (numFacesFromNurbsResolution(res) > targetFaces && res > 2)
        --res;
    return res;
}

// 计算每个点的表面曲率（PCA 最小特征值 / 特征值之和）
// 返回值：0 = 完全平坦，值越大曲率越高
std::vector<float> Point_Algorithm::computePointCurvature(const PCLCloud::Ptr& cloud,
                                                           int kNeighbors)
{
    std::vector<float> curvatures(cloud->size(), 0.0f);
    if (!cloud || cloud->size() < 4)
        return curvatures;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // 估算平均点间距 → 搜索半径 = 间距 × sqrt(k)
    double avgSpacing = 0.0;
    {
        std::vector<int> idx2(2);
        std::vector<float> sqD2(2);
        const std::size_t sampleStep = std::max<std::size_t>(1, cloud->size() / 500);
        int count = 0;
        for (std::size_t i = 0; i < cloud->size(); i += sampleStep) {
            if (tree->nearestKSearch(cloud->points[i], 2, idx2, sqD2) == 2 && sqD2[1] > 0.0) {
                avgSpacing += std::sqrt(sqD2[1]);
                ++count;
            }
        }
        avgSpacing = (count > 0) ? (avgSpacing / count) : 0.1;
    }
    const float searchRadius = static_cast<float>(avgSpacing * std::sqrt(static_cast<double>(kNeighbors)));

    std::vector<int> indices;
    std::vector<float> sqDistances;

    for (std::size_t i = 0; i < cloud->size(); ++i)
    {
        const int found = tree->radiusSearch(cloud->points[i], searchRadius,
                                              indices, sqDistances,
                                              static_cast<unsigned int>(kNeighbors));
        if (found < 4) {
            curvatures[i] = 0.0f;
            continue;
        }

        Eigen::Vector3f centroid(0, 0, 0);
        for (int j = 0; j < found; ++j)
            centroid += Eigen::Vector3f(cloud->points[indices[j]].x,
                                        cloud->points[indices[j]].y,
                                        cloud->points[indices[j]].z);
        centroid /= static_cast<float>(found);

        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for (int j = 0; j < found; ++j) {
            const Eigen::Vector3f d = Eigen::Vector3f(
                cloud->points[indices[j]].x,
                cloud->points[indices[j]].y,
                cloud->points[indices[j]].z) - centroid;
            cov += d * d.transpose();
        }
        cov /= static_cast<float>(found);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
        const float l0 = eig.eigenvalues()[0];
        const float sum = l0 + eig.eigenvalues()[1] + eig.eigenvalues()[2];
        curvatures[i] = (sum > 1e-12f) ? (l0 / sum) : 0.0f;
    }
    return curvatures;
}

// 检测曲率突变点（棱线/顶点）：
//   1. 计算所有点的曲率 ci
//   2. 对每个点 i，搜索半径 r 内邻域点，计算曲率梯度 gi = RMS(ci - cj)
//   3. 统计全局梯度分布，阈值 = mean(g) + stddevMultiplier * stddev(g)
//   4. gi >= 阈值 → 特征点（棱线/顶点），gi < 阈值 → 普通点（面内）
Point_Algorithm::FeaturePointsResult Point_Algorithm::detectFeaturePoints(
    const PCLCloud::Ptr& cloud, float searchRadius, float stddevMultiplier)
{
    FeaturePointsResult result;
    result.gradientThreshold = 0.0f;
    result.featureCount = 0;

    if (!cloud || cloud->size() < 10)
        return result;

    // Step 1: 曲率计算
    result.curvatures = computePointCurvature(cloud, 20);
    const std::size_t N = result.curvatures.size();
    result.isFeature.resize(N, false);
    result.curvatureGrad.resize(N, 0.0f);

    // Step 2: 构建 KD 树做半径搜索
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // Step 3: 对每个点计算曲率梯度（与邻域曲率差异的RMS）
    std::vector<float> grads(N, 0.0f);
    float gradSum = 0.0f;
    int gradCount = 0;

    for (std::size_t i = 0; i < N; ++i)
    {
        std::vector<int> neighIdx;
        std::vector<float> neighSqDist;
        const int nFound = tree->radiusSearch(cloud->points[i], searchRadius, neighIdx, neighSqDist);
        if (nFound < 3) {
            grads[i] = 0.0f;
            continue;
        }

        // 曲率梯度 = 与邻域各点曲率差的均方根
        float sumSqDiff = 0.0f;
        int validNeigh = 0;
        for (int j = 0; j < nFound; ++j) {
            const std::size_t nj = static_cast<std::size_t>(neighIdx[j]);
            if (nj == i) continue;
            const float diff = result.curvatures[i] - result.curvatures[nj];
            sumSqDiff += diff * diff;
            ++validNeigh;
        }
        if (validNeigh > 0) {
            grads[i] = std::sqrt(sumSqDiff / validNeigh);
            gradSum += grads[i];
            ++gradCount;
        }
    }

    if (gradCount == 0)
        return result;

    // Step 4: 统计梯度分布，计算阈值
    const float gradMean = gradSum / gradCount;
    float gradVar = 0.0f;
    for (float g : grads)
        gradVar += (g - gradMean) * (g - gradMean);
    gradVar /= gradCount;
    const float gradStd = std::sqrt(gradVar);
    result.gradientThreshold = gradMean + stddevMultiplier * gradStd;

    // Step 5: 标记特征点
    for (std::size_t i = 0; i < N; ++i) {
        result.isFeature[i] = (grads[i] >= result.gradientThreshold);
        if (result.isFeature[i])
            ++result.featureCount;
    }
    result.curvatureGrad = std::move(grads);

    std::cout << "[FeatureDetect] Total=" << N
              << ", features=" << result.featureCount
              << " (" << (100.0f * result.featureCount / N) << "%)"
              << ", gradMean=" << gradMean
              << ", gradStd=" << gradStd
              << ", threshold=" << result.gradientThreshold << std::endl;

    return result;
}

// DoN 边缘检测：两半径法向夹角 → 边缘概率
std::vector<bool> Point_Algorithm::detectEdgesDoN(const PCLCloud::Ptr& cloud,
                                                    float smallRadius,
                                                    float largeRadius,
                                                    float angleThresholdDeg)
{
    const std::size_t N = cloud->size();
    std::vector<bool> edges(N, false);
    if (!cloud || N < 10)
        return edges;

    // KD 树
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // 小半径法向
    pcl::PointCloud<pcl::Normal>::Ptr normalsSmall(new pcl::PointCloud<pcl::Normal>);
    {
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(smallRadius);
        ne.compute(*normalsSmall);
    }

    // 大半径法向
    pcl::PointCloud<pcl::Normal>::Ptr normalsLarge(new pcl::PointCloud<pcl::Normal>);
    {
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(largeRadius);
        ne.compute(*normalsLarge);
    }

    const float cosThreshold = std::cos(angleThresholdDeg * static_cast<float>(M_PI) / 180.0f);
    int edgeCount = 0;

    for (std::size_t i = 0; i < N; ++i)
    {
        const auto& ns = normalsSmall->points[i];
        const auto& nl = normalsLarge->points[i];

        // 跳过无效法向
        if (!std::isfinite(ns.normal_x) || !std::isfinite(nl.normal_x))
            continue;

        // |dot(n_small, n_large)| — 夹角余弦的绝对值
        const float dotAbs = std::abs(ns.normal_x * nl.normal_x +
                                      ns.normal_y * nl.normal_y +
                                      ns.normal_z * nl.normal_z);
        // 夹角 > 阈值 → 边缘
        if (dotAbs < cosThreshold) {
            edges[i] = true;
            ++edgeCount;
        }
    }

    std::cout << "[DoN] smallR=" << smallRadius << ", largeR=" << largeRadius
              << ", angleThresh=" << angleThresholdDeg << "°"
              << ", edges=" << edgeCount << "/" << N
              << " (" << (100.0 * edgeCount / N) << "%)" << std::endl;

    return edges;
}

// 联合特征提取：曲率梯度 ∪ DoN边缘
PCLCloud::Ptr Point_Algorithm::extractFeatureCloud(const PCLCloud::Ptr& cloud,
                                                     float searchRadius,
                                                     float donSmallRadius,
                                                     float donLargeRadius)
{
    PCLCloud::Ptr output(new PCLCloud);
    if (!cloud || cloud->empty())
        return output;

    const std::size_t N = cloud->size();
    std::vector<bool> isFeature(N, false);

    // 1. 曲率梯度特征
    {
        auto result = detectFeaturePoints(cloud, searchRadius, 2.0f);
        for (std::size_t i = 0; i < N; ++i)
            if (result.isFeature[i])
                isFeature[i] = true;
    }

    // 2. DoN 边缘特征
    {
        auto edges = detectEdgesDoN(cloud, donSmallRadius, donLargeRadius, 15.0f);
        for (std::size_t i = 0; i < N; ++i)
            if (edges[i])
                isFeature[i] = true;
    }

    // 3. 提取所有特征点
    std::size_t featureCount = 0;
    for (std::size_t i = 0; i < N; ++i) {
        if (isFeature[i]) {
            output->push_back(cloud->points[i]);
            ++featureCount;
        }
    }

    output->width = static_cast<uint32_t>(output->size());
    output->height = 1;
    output->is_dense = false;

    std::cout << "[ExtractFeature] " << featureCount << "/" << N
              << " points (" << (100.0 * featureCount / N) << "%)" << std::endl;

    return output;
}

// 曲率自适应降采样：
// 计算每个点的曲率 → 按百分位分特征区/平坦区 → 不同体素尺寸分别降采样 → 合并
PCLCloud::Ptr Point_Algorithm::adaptiveCurvatureDownsample(const PCLCloud::Ptr& input,
                                                            float featureLeafSize,
                                                            float flatLeafSize,
                                                            float curvaturePercentile)
{
    PCLCloud::Ptr output(new PCLCloud);
    if (!input || input->empty())
        return output;

    // 1. 计算曲率
    const std::vector<float> curvatures = computePointCurvature(input, 20);
    if (curvatures.empty())
        return input;

    // 2. 找到曲率分位阈值
    std::vector<float> sortedCurv = curvatures;
    std::sort(sortedCurv.begin(), sortedCurv.end());
    const std::size_t thresholdIdx = static_cast<std::size_t>(
        sortedCurv.size() * (1.0f - curvaturePercentile / 100.0f));
    const float curvatureThreshold = sortedCurv[std::min(thresholdIdx, sortedCurv.size() - 1)];

    // 3. 拆分为特征点（高曲率）和平坦点（低曲率）
    PCLCloud::Ptr featurePts(new PCLCloud);
    PCLCloud::Ptr flatPts(new PCLCloud);
    featurePts->reserve(input->size());
    flatPts->reserve(input->size());

    for (std::size_t i = 0; i < input->size(); ++i) {
        if (curvatures[i] >= curvatureThreshold)
            featurePts->push_back(input->points[i]);
        else
            flatPts->push_back(input->points[i]);
    }

    std::cout << "[AdaptiveDS] Curvature threshold=" << curvatureThreshold
              << ", feature pts=" << featurePts->size()
              << ", flat pts=" << flatPts->size() << std::endl;

    // 4. 分别体素降采样
    auto downsample = [](const PCLCloud::Ptr& pts, float leaf) -> PCLCloud::Ptr {
        if (!pts || pts->empty()) return pts;
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(pts);
        vg.setLeafSize(leaf, leaf, leaf);
        PCLCloud::Ptr out(new PCLCloud);
        vg.filter(*out);
        return out;
    };

    PCLCloud::Ptr featureDown = downsample(featurePts, featureLeafSize);
    PCLCloud::Ptr flatDown = downsample(flatPts, flatLeafSize);

    // 5. 合并
    *output = *featureDown;
    if (flatDown && !flatDown->empty())
        *output += *flatDown;

    std::cout << "[AdaptiveDS] " << input->size() << " → "
              << output->size() << " pts (feature=" << featureDown->size()
              << ", flat=" << (flatDown ? flatDown->size() : 0) << ")" << std::endl;
    return output;
}

// 网格修复：去重顶点 + 删退化面片 + 去孤立点
// 返回移除的退化面片数
int Point_Algorithm::repairMesh(PCLMesh& mesh)
{
    if (mesh.polygons.empty())
        return 0;

    const int nBefore = static_cast<int>(mesh.polygons.size());

    // ---------- PCLMesh -> vtkPolyData ----------
    pcl::PointCloud<pcl::PointXYZ> vertices;
    pcl::fromPCLPointCloud2(mesh.cloud, vertices);

    vtkNew<vtkPoints> pts;
    for (const auto& p : vertices)
        pts->InsertNextPoint(p.x, p.y, p.z);

    vtkNew<vtkCellArray> polys;
    for (const auto& tri : mesh.polygons) {
        if (tri.vertices.size() == 3) {
            polys->InsertNextCell(3, std::array<vtkIdType,3>{{
                static_cast<vtkIdType>(tri.vertices[0]),
                static_cast<vtkIdType>(tri.vertices[1]),
                static_cast<vtkIdType>(tri.vertices[2])
            }}.data());
        }
    }

    vtkNew<vtkPolyData> pd;
    pd->SetPoints(pts);
    pd->SetPolys(polys);

    // --- 1) 合并重合顶点（容差1e-6），去除未引用点 ---
    {
        vtkNew<vtkCleanPolyData> cleaner;
        cleaner->SetInputData(pd);
        cleaner->SetTolerance(1e-6);
        cleaner->PointMergingOn();
        cleaner->Update();
        pd->DeepCopy(cleaner->GetOutput());
    }

    // --- 2) 剔除退化三角形（零面积 / 重复顶点） ---
    {
        vtkIdType npts;
        const vtkIdType* cellPts;
        vtkCellArray* inPolys = pd->GetPolys();
        vtkPoints* inPts = pd->GetPoints();

        vtkNew<vtkCellArray> goodPolys;
        int removed = 0;
        inPolys->InitTraversal();
        while (inPolys->GetNextCell(npts, cellPts)) {
            if (npts != 3) continue;
            double a[3], b[3], c[3];
            inPts->GetPoint(cellPts[0], a);
            inPts->GetPoint(cellPts[1], b);
            inPts->GetPoint(cellPts[2], c);

            // 边向量
            const double ux = b[0]-a[0], uy = b[1]-a[1], uz = b[2]-a[2];
            const double vx = c[0]-a[0], vy = c[1]-a[1], vz = c[2]-a[2];
            // 叉积的模平方 = (2×面积)²
            const double cx = uy*vz - uz*vy;
            const double cy = uz*vx - ux*vz;
            const double cz = ux*vy - uy*vx;
            const double area2 = cx*cx + cy*cy + cz*cz;

            // 边长平方
            const double l12 = ux*ux + uy*uy + uz*uz;
            const double l23 = (c[0]-b[0])*(c[0]-b[0]) + (c[1]-b[1])*(c[1]-b[1]) + (c[2]-b[2])*(c[2]-b[2]);
            const double l31 = (a[0]-c[0])*(a[0]-c[0]) + (a[1]-c[1])*(a[1]-c[1]) + (a[2]-c[2])*(a[2]-c[2]);

            // 零面积 或 存在极短边（< 点云尺度×1e-8）→ 退化
            if (area2 < 1e-24 || l12 < 1e-16 || l23 < 1e-16 || l31 < 1e-16) {
                ++removed;
                continue;
            }
            goodPolys->InsertNextCell(3, cellPts);
        }

        if (removed > 0) {
            pd->SetPolys(goodPolys);
        }
    }

    if (pd->GetNumberOfPolys() < 1)
        return nBefore;

    // ---------- vtkPolyData -> PCLMesh ----------
    vtkPoints* outPts = pd->GetPoints();
    vtkCellArray* outPolys = pd->GetPolys();

    pcl::PointCloud<pcl::PointXYZ> outVertices;
    outVertices.resize(outPts->GetNumberOfPoints());
    for (vtkIdType i = 0; i < outPts->GetNumberOfPoints(); ++i) {
        double xyz[3];
        outPts->GetPoint(i, xyz);
        outVertices[i].x = static_cast<float>(xyz[0]);
        outVertices[i].y = static_cast<float>(xyz[1]);
        outVertices[i].z = static_cast<float>(xyz[2]);
    }

    mesh.polygons.clear();
    mesh.polygons.reserve(pd->GetNumberOfPolys());
    vtkIdType npts2;
    const vtkIdType* cellPts2;
    outPolys->InitTraversal();
    while (outPolys->GetNextCell(npts2, cellPts2)) {
        if (npts2 == 3) {
            pcl::Vertices v;
            v.vertices.resize(3);
            v.vertices[0] = static_cast<uint32_t>(cellPts2[0]);
            v.vertices[1] = static_cast<uint32_t>(cellPts2[1]);
            v.vertices[2] = static_cast<uint32_t>(cellPts2[2]);
            mesh.polygons.push_back(v);
        }
    }

    pcl::toPCLPointCloud2(outVertices, mesh.cloud);

    const int nAfter = static_cast<int>(mesh.polygons.size());
    const int nRemoved = nBefore - nAfter;
    if (nRemoved > 0) {
        std::cout << "[RepairMesh] Removed " << nRemoved
                  << " degenerate faces (" << nBefore << " → " << nAfter << ")" << std::endl;
    }
    return nRemoved;
}

// 网格精简：渐进式 VTK Quadric Decimation
void Point_Algorithm::decimateMesh(PCLMesh& mesh, int targetFaces)
{
    const int nFaces = static_cast<int>(mesh.polygons.size());
    if (nFaces <= targetFaces)
        return;

    // ---------- PCLMesh -> vtkPolyData ----------
    pcl::PointCloud<pcl::PointXYZ> vertices;
    pcl::fromPCLPointCloud2(mesh.cloud, vertices);

    vtkNew<vtkPoints> pts;
    for (const auto& p : vertices)
        pts->InsertNextPoint(p.x, p.y, p.z);

    vtkNew<vtkCellArray> polys;
    for (const auto& tri : mesh.polygons) {
        if (tri.vertices.size() == 3) {
            vtkIdType ids[3] = {
                static_cast<vtkIdType>(tri.vertices[0]),
                static_cast<vtkIdType>(tri.vertices[1]),
                static_cast<vtkIdType>(tri.vertices[2])
            };
            polys->InsertNextCell(3, ids);
        }
    }

    vtkNew<vtkPolyData> pd;
    pd->SetPoints(pts);
    pd->SetPolys(polys);

    // === 阶段 1：特征边保护减面（保留曲面，优先合并平面区域） ===
    // DecimatePro 按 FeatureAngle 保留棱线：平面区域二面角≈0° → 优先合并
    // 曲面区域二面角大 → 自动保留。榫头（平面）大量减面，叶身（曲面）保留细节。
    {
        const int stage1Target = std::max(targetFaces * 2, 100);
        const int cur = static_cast<int>(pd->GetNumberOfPolys());
        if (cur > stage1Target) {
            const double ratio = 1.0 - static_cast<double>(stage1Target) / cur;
            vtkNew<vtkDecimatePro> decPro;
            decPro->SetInputData(pd);
            decPro->SetTargetReduction(ratio);
            decPro->SetPreserveTopology(true);
            decPro->SetFeatureAngle(15.0);    // 二面角>15°的棱线受保护
            decPro->Update();
            vtkPolyData* out = decPro->GetOutput();
            if (out && out->GetNumberOfPolys() >= 10)
                pd->DeepCopy(out);
        }
    }

    // === 阶段 2：渐进 Quadric 补刀到目标 ===
    int currentFaces = static_cast<int>(pd->GetNumberOfPolys());
    while (currentFaces > targetFaces) {
        const int stepTarget = std::max(targetFaces, currentFaces / 2);
        const double ratio = 1.0 - static_cast<double>(stepTarget) / currentFaces;

        vtkNew<vtkQuadricDecimation> dec;
        dec->SetInputData(pd);
        dec->SetTargetReduction(ratio);
        dec->Update();

        vtkPolyData* out = dec->GetOutput();
        if (!out || out->GetNumberOfPolys() < 10)
            break;
        pd->DeepCopy(out);
        const int newFaces = static_cast<int>(pd->GetNumberOfPolys());
        if (newFaces >= currentFaces)
            break;
        currentFaces = newFaces;
    }

    // ---------- 清理退化三角形（去除零面积/重复顶点的破损面片） ----------
    {
        vtkIdType npts;
        const vtkIdType* cellPts;
        vtkCellArray* inPolys = pd->GetPolys();
        vtkPoints* inPts = pd->GetPoints();

        vtkNew<vtkCellArray> cleanPolys;
        int removed = 0;
        inPolys->InitTraversal();
        while (inPolys->GetNextCell(npts, cellPts)) {
            if (npts != 3) continue;
            double a[3], b[3], c[3];
            inPts->GetPoint(cellPts[0], a);
            inPts->GetPoint(cellPts[1], b);
            inPts->GetPoint(cellPts[2], c);
            // 计算两条边向量的叉积（面积×2）
            const double ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2];
            const double vx = c[0] - a[0], vy = c[1] - a[1], vz = c[2] - a[2];
            const double area2 = (uy * vz - uz * vy) * (uy * vz - uz * vy)
                               + (uz * vx - ux * vz) * (uz * vx - ux * vz)
                               + (ux * vy - uy * vx) * (ux * vy - uy * vx);
            // 面积太小（边长 < 1e-12）视为退化面片，丢弃
            if (area2 < 1e-24) {
                ++removed;
                continue;
            }
            // 检查是否有重复顶点（两点距离 < 1e-9）
            const double d12 = (b[0]-a[0])*(b[0]-a[0]) + (b[1]-a[1])*(b[1]-a[1]) + (b[2]-a[2])*(b[2]-a[2]);
            const double d23 = (c[0]-b[0])*(c[0]-b[0]) + (c[1]-b[1])*(c[1]-b[1]) + (c[2]-b[2])*(c[2]-b[2]);
            const double d31 = (a[0]-c[0])*(a[0]-c[0]) + (a[1]-c[1])*(a[1]-c[1]) + (a[2]-c[2])*(a[2]-c[2]);
            if (d12 < 1e-18 || d23 < 1e-18 || d31 < 1e-18) {
                ++removed;
                continue;
            }
            cleanPolys->InsertNextCell(3, cellPts);
        }
        if (removed > 0) {
            pd->SetPolys(cleanPolys);
            std::cout << "[Decimate] Removed " << removed << " degenerate triangles" << std::endl;
        }
    }

    if (pd->GetNumberOfPolys() < 10)
        return;

    // ---------- vtkPolyData -> PCLMesh ----------
    vtkPoints* outPts = pd->GetPoints();
    vtkCellArray* outPolys = pd->GetPolys();

    pcl::PointCloud<pcl::PointXYZ> outVertices;
    outVertices.resize(outPts->GetNumberOfPoints());
    for (vtkIdType i = 0; i < outPts->GetNumberOfPoints(); ++i) {
        double xyz[3];
        outPts->GetPoint(i, xyz);
        outVertices[i].x = static_cast<float>(xyz[0]);
        outVertices[i].y = static_cast<float>(xyz[1]);
        outVertices[i].z = static_cast<float>(xyz[2]);
    }

    mesh.polygons.clear();
    mesh.polygons.reserve(pd->GetNumberOfPolys());
    vtkIdType npts2;
    const vtkIdType* cellPts2;
    outPolys->InitTraversal();
    while (outPolys->GetNextCell(npts2, cellPts2)) {
        if (npts2 == 3) {
            pcl::Vertices v;
            v.vertices.resize(3);
            v.vertices[0] = static_cast<uint32_t>(cellPts2[0]);
            v.vertices[1] = static_cast<uint32_t>(cellPts2[1]);
            v.vertices[2] = static_cast<uint32_t>(cellPts2[2]);
            mesh.polygons.push_back(v);
        }
    }

    pcl::toPCLPointCloud2(outVertices, mesh.cloud);
    std::cout << "[Decimate] Mesh " << nFaces
              << " -> " << mesh.polygons.size() << " faces (target: " << targetFaces << ")" << std::endl;
}

// 表面重建：Greedy / Poisson / Nurbs 统一入口
PCLMesh Point_Algorithm::reconstructMesh(const PCLCloud::Ptr& input, ReconMethod method,
                                         float voxelLeafSize, float maxSurfaceAngleDeg,
                                         int nurbsOrder, int nurbsRefinement, int nurbsResolution,
                                         int poissonDepth, int poissonSolverDivide, int poissonIsoDivide)
{
    PCLMesh output;
    if(!input || input->size() < 3)
        return output;

    if(method == ReconMethod::Nurbs)
        return nurbsReconstruction(input, nurbsOrder, nurbsRefinement, nurbsResolution);

    // --- Greedy 与 Poisson 共享的预处理 ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(input);
    voxel_filter.setLeafSize(voxelLeafSize, voxelLeafSize, voxelLeafSize);
    voxel_filter.filter(*cloud_filtered);

    if(cloud_filtered->empty())
    {
        std::cerr << "Downsampled cloud is empty." << std::endl;
        return output;
    }

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    tree->setInputCloud(cloud_filtered);
    n.setInputCloud(cloud_filtered);
    n.setSearchMethod(tree);

    if(method == ReconMethod::Poisson)
    {
        n.setKSearch(30);
        n.compute(*normals);

        // 法向一致化：封闭曲面 → 所有法向朝外（远离质心）
        {
            Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
            for(const auto& p : cloud_filtered->points)
                centroid += Eigen::Vector3f(p.x, p.y, p.z);
            centroid /= static_cast<float>(cloud_filtered->size());

            for(std::size_t i = 0; i < cloud_filtered->size(); ++i)
            {
                const auto& p = cloud_filtered->points[i];
                auto& nm = normals->points[i];
                // 从质心到点的方向 ≈ 外法向
                const float dx = p.x - centroid.x();
                const float dy = p.y - centroid.y();
                const float dz = p.z - centroid.z();
                const float dot = nm.normal_x * dx + nm.normal_y * dy + nm.normal_z * dz;
                if(dot < 0.0f)
                {
                    nm.normal_x = -nm.normal_x;
                    nm.normal_y = -nm.normal_y;
                    nm.normal_z = -nm.normal_z;
                }
            }
        }

        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields(*cloud_filtered, *normals, *cloud_with_normals);

        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud(cloud_with_normals);

        pcl::Poisson<pcl::PointNormal> pn;
        pn.setConfidence(false);
        pn.setDegree(2);
        pn.setDepth(poissonDepth);
        pn.setIsoDivide(poissonIsoDivide);
        pn.setManifold(true);          // 封闭曲面要求流形输出，去除非流形边
        pn.setOutputPolygons(false);
        pn.setSamplesPerNode(2.0);     // 更多采样 → 更光滑
        pn.setScale(1.25);             // 略微放大重构尺度，减少边界截断
        pn.setSolverDivide(poissonSolverDivide);
        pn.setSearchMethod(tree2);
        pn.setInputCloud(cloud_with_normals);

        pn.performReconstruction(output);
        std::cout << "Poisson reconstruction completed" << std::endl;
    }
    else // ReconMethod::Greedy
    {
        n.setKSearch(20);
        n.compute(*normals);

        // 估算平均点间距，用于确定三角化搜索半径
        double spacingSum = 0.0;
        int spacingCount = 0;
        std::vector<int> nearestIndices(2);
        std::vector<float> nearestSquaredDistances(2);
        const std::size_t sampleStep = std::max<std::size_t>(1, cloud_filtered->size() / 2000);
        for(std::size_t i = 0; i < cloud_filtered->size(); i += sampleStep)
        {
            if(tree->nearestKSearch(cloud_filtered->points[i], 2, nearestIndices, nearestSquaredDistances) == 2 &&
                nearestSquaredDistances[1] > 0.0f)
            {
                spacingSum += std::sqrt(nearestSquaredDistances[1]);
                ++spacingCount;
            }
        }
        const double averageSpacing = spacingCount > 0 ? spacingSum / spacingCount : 1.0;

        // 搜索半径：3.5 倍平均点间距，最小 2.5（原始 PCL 推荐值）
        const double triangulationRadius = std::max(averageSpacing * 3.5, 2.5);

        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields(*cloud_filtered, *normals, *cloud_with_normals);

        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud(cloud_with_normals);

        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
        gp3.setSearchRadius(triangulationRadius);
        gp3.setMu(2.0);                       // 标准距离乘数
        gp3.setMaximumNearestNeighbors(80);   // 标准邻域上限
        gp3.setMaximumSurfaceAngle(maxSurfaceAngleDeg * M_PI / 180.0);
        gp3.setMinimumAngle(M_PI / 18);
        gp3.setMaximumAngle(2 * M_PI / 3);
        gp3.setNormalConsistency(true);
        gp3.setInputCloud(cloud_with_normals);
        gp3.setSearchMethod(tree2);
        gp3.reconstruct(output);

        std::cout << "Greedy reconstruction done, adaptive radius = " << triangulationRadius << std::endl;
    }

    return output;
}
