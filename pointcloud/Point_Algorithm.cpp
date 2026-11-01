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

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <chrono>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>

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

// 多条与回转轴近似平行的圆柱轴线联合拟合（虚拟圆柱）。
// 每条线 L_i = p_i + t*d_i 中 p_i 可沿 d_i 任意平移。把 p_i 投影到
// 垂直共同方向的平面后，这种任意平移会被消除，投影点正好构成圆轨迹。
Point_Algorithm::RotationAxisFitResult Point_Algorithm::fitRotationAxisFromLines(
    const std::vector<Eigen::Vector3f>& linePoints,
    const std::vector<Eigen::Vector3f>& lineDirections)
{
    RotationAxisFitResult result;
    const int count = static_cast<int>(linePoints.size());
    if(count < 3 || lineDirections.size() != linePoints.size())
        return result;

    std::vector<Eigen::Vector3f> directions;
    directions.reserve(static_cast<std::size_t>(count));
    Eigen::Vector3f reference;
    for(int i = 0; i < count; ++i)
    {
        Eigen::Vector3f direction = lineDirections[static_cast<std::size_t>(i)];
        if(direction.norm() < 1e-6f)
            return result;
        direction.normalize();
        if(directions.empty())
            reference = direction;
        // 圆柱轴线没有正反方向；统一朝向以消除拟合器的随机翻转。
        if(direction.dot(reference) < 0.0f)
            direction = -direction;
        directions.push_back(direction);
    }

    Eigen::Vector3f commonDirection = Eigen::Vector3f::Zero();
    for(const Eigen::Vector3f& direction : directions)
        commonDirection += direction;
    if(commonDirection.norm() < 1e-6f)
        return result;

    commonDirection.normalize();
    std::vector<Eigen::Vector3f> projectedLinePoints;
    projectedLinePoints.reserve(static_cast<std::size_t>(count));
    float directionSquareError = 0.0f;
    for(int i = 0; i < count; ++i)
    {
        const Eigen::Vector3f& point = linePoints[static_cast<std::size_t>(i)];
        const Eigen::Vector3f& direction = directions[static_cast<std::size_t>(i)];
        projectedLinePoints.push_back(point - commonDirection * commonDirection.dot(point));
        const float parallelResidual = commonDirection.cross(direction).norm();
        directionSquareError += parallelResidual * parallelResidual;
    }

    result = fitRotationAxisFromCenters(projectedLinePoints);
    if(!result.success)
        return result;

    // 锁定为各圆柱轴线的平均方向，避免圆轨迹平面的微小数值扰动改变方向。
    result.axisDirection = commonDirection;
    result.planeRMSE = std::sqrt(directionSquareError / static_cast<float>(count));
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

std::vector<Eigen::Vector3f> Point_Algorithm::orderAndResampleSection(
    const PCLCloud::Ptr& section, int nSamples,
    const Eigen::Vector3f& refChord, const Eigen::Vector3f& refThick)
{
    std::vector<Eigen::Vector3f> result;
    if(!section || section->size() < 3 || nSamples < 2)
        return result;

    const int n = static_cast<int>(section->size());

    std::vector<Eigen::Vector3f> pts;
    pts.reserve(n);
    for(const auto& p : section->points)
        pts.emplace_back(p.x, p.y, p.z);

    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for(const auto& p : pts) centroid += p;
    centroid /= static_cast<float>(n);

    // --- 1. 截面平面 2D 基底 (u=弦向, v=厚度向) ---
    // 优先用外部传入的共用基底（保证各截面绕行方向一致，避免拧麻花）；否则本截面 PCA。
    Eigen::Vector3f u, v;
    if(refChord.norm() > 1e-6f && refThick.norm() > 1e-6f)
    {
        u = refChord.normalized();
        v = (refThick - refThick.dot(u) * u).normalized(); // 对 u 正交化
    }
    else
    {
        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for(const auto& p : pts) { Eigen::Vector3f d = p - centroid; cov += d * d.transpose(); }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
        u = es.eigenvectors().col(2).normalized(); // 最大方差 = 弦向
        v = es.eigenvectors().col(1).normalized(); // 次方向 = 厚度向
    }

    // --- 2. 上下表面分离排序(翼型专用,避免极角排序在薄翼时上下点交替混杂) ---
    // 投影到截面 (u,v) 平面:(chord, thickness) = (d·u, d·v)
    struct Pt2D { int idx; float chord, thick; };
    std::vector<Pt2D> pts2d;
    pts2d.reserve(n);
    for(int i = 0; i < n; ++i)
    {
        const Eigen::Vector3f d = pts[i] - centroid;
        pts2d.push_back({ i, d.dot(u), d.dot(v) });
    }

    // 找前/后缘(弦向极值)
    auto cmpChord = [](const Pt2D& a, const Pt2D& b){ return a.chord < b.chord; };
    const float minChord = std::min_element(pts2d.begin(), pts2d.end(), cmpChord)->chord;
    const float maxChord = std::max_element(pts2d.begin(), pts2d.end(), cmpChord)->chord;
    const float midChord = (minChord + maxChord) * 0.5f;

    // 上下表面分离 + 弦向排序
    std::vector<Pt2D> upper, lower;
    for(const auto& p : pts2d)
    {
        if(p.thick >= 0.0f)
            upper.push_back(p);
        else
            lower.push_back(p);
    }
    // 上表面:弦向从大到小(后缘→前缘),下表面:弦向从小到大(前缘→后缘)
    std::sort(upper.begin(), upper.end(), [](const Pt2D& a, const Pt2D& b){ return a.chord > b.chord; });
    std::sort(lower.begin(), lower.end(), [](const Pt2D& a, const Pt2D& b){ return a.chord < b.chord; });

    // 拼接闭合轮廓:后缘(upper首) → 上表面 → 前缘 → 下表面 → 回后缘
    // (前后缘附近可能上下都有点,按 chord 距离分配给最近端)
    std::vector<int> contour;
    contour.reserve(n);
    for(const auto& p : upper) contour.push_back(p.idx);
    for(const auto& p : lower) contour.push_back(p.idx);

    // --- 3. 闭合轮廓(首点补到末尾),累积弧长归一化,线性插值重采样 ---
    std::vector<Eigen::Vector3f> loop;
    loop.reserve(contour.size() + 1);
    for(int idx : contour) loop.push_back(pts[idx]);
    loop.push_back(loop.front()); // 闭合

    const int m = static_cast<int>(loop.size());
    std::vector<float> s(m, 0.0f);
    for(int i = 1; i < m; ++i)
        s[i] = s[i - 1] + (loop[i] - loop[i - 1]).norm();
    if(s[m - 1] <= 1e-9f)
        return result; // 退化：所有点重合
    for(int i = 0; i < m; ++i) s[i] /= s[m - 1];

    result.resize(nSamples);
    for(int k = 0; k < nSamples; ++k)
    {
        const float sq = static_cast<float>(k) / static_cast<float>(nSamples - 1);
        int seg = 0;
        while(seg < m - 2 && s[seg + 1] < sq) ++seg;
        const float denom = std::max(s[seg + 1] - s[seg], 1e-9f);
        const float t = (sq - s[seg]) / denom;
        result[k] = loop[seg] * (1.0f - t) + loop[seg + 1] * t;
    }
    return result;
}

// ========== 方案二：用中弧线把截面分成上/下两个面，各自插值（开放曲线，不闭合）==========
bool Point_Algorithm::separateBladeSurfaces(
    const PCLCloud::Ptr& section, int nInterp,
    PCLCloud::Ptr& upperOut, PCLCloud::Ptr& lowerOut,
    Eigen::Vector3f* refU, Eigen::Vector3f* refV)
{
    upperOut.reset(new PCLCloud);
    lowerOut.reset(new PCLCloud);
    if(!section || section->size() < 10 || nInterp < 2)
        return false;

    const int n = static_cast<int>(section->size());
    std::vector<Eigen::Vector3f> pts;
    pts.reserve(n);
    for(const auto& p : section->points)
        pts.emplace_back(p.x, p.y, p.z);

    // --- 1. PCA 求截面平面基底：u=弦向(最大方差), v=厚度向(次方差) ---
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for(const auto& p : pts) centroid += p;
    centroid /= static_cast<float>(n);
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for(const auto& p : pts)
    { Eigen::Vector3f d = p - centroid; cov += d * d.transpose(); }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
    Eigen::Vector3f u = es.eigenvectors().col(2).normalized();
    Eigen::Vector3f v = es.eigenvectors().col(1).normalized();

    // 多截面对齐：若给了参考方向且已初始化，则翻转本截面 u/v 使符号一致，
    // 避免相邻截面因 PCA 特征向量方向任意而互换 LE/TE 或上下面，导致 mesh 螺旋扭曲。
    if(refU && refU->squaredNorm() > 1e-6f && u.dot(*refU) < 0.0f)
        u = -u;
    if(refV && refV->squaredNorm() > 1e-6f && v.dot(*refV) < 0.0f)
        v = -v;
    // 写回本截面实际方向，供下一截面参考
    if(refU) *refU = u;
    if(refV) *refV = v;

    // 投影到 (s=弦向, t=厚度向)
    std::vector<float> S(n), T(n);
    float sMin = std::numeric_limits<float>::max(), sMax = -sMin;
    for(int i = 0; i < n; ++i)
    {
        const Eigen::Vector3f d = pts[i] - centroid;
        S[i] = d.dot(u);
        T[i] = d.dot(v);
        sMin = std::min(sMin, S[i]);
        sMax = std::max(sMax, S[i]);
    }
    const float sSpan = std::max(sMax - sMin, 1e-6f);

    // --- 2. 沿弦向分箱，估中弧线：每箱取厚度的几何中点 (min+max)/2 ---
    // 用中点而非均值：均值会被点多的一面带偏，中点是上下表面的几何中线，分面更准。
    const int nBins = std::max(10, std::min(60, n / 8));
    std::vector<float> binMin(nBins,  std::numeric_limits<float>::max());
    std::vector<float> binMax(nBins, -std::numeric_limits<float>::max());
    std::vector<int> binCnt(nBins, 0);
    auto binOf = [&](float s){
        int b = static_cast<int>((s - sMin) / sSpan * nBins);
        return std::max(0, std::min(nBins - 1, b));
    };
    for(int i = 0; i < n; ++i)
    {
        const int b = binOf(S[i]);
        binMin[b] = std::min(binMin[b], T[i]);
        binMax[b] = std::max(binMax[b], T[i]);
        binCnt[b] += 1;
    }
    // 填补空箱 + 得到中弧线采样
    std::vector<float> camber(nBins, 0.0f);
    for(int b = 0; b < nBins; ++b)
        camber[b] = binCnt[b] > 0 ? 0.5f * (binMin[b] + binMax[b]) : 0.0f;
    for(int b = 0; b < nBins; ++b)
        if(binCnt[b] == 0)
        {
            int l = b, r = b;
            while(l >= 0 && binCnt[l] == 0) --l;
            while(r < nBins && binCnt[r] == 0) ++r;
            if(l < 0 && r < nBins) camber[b] = camber[r];
            else if(r >= nBins && l >= 0) camber[b] = camber[l];
            else if(l >= 0 && r < nBins) camber[b] = 0.5f * (camber[l] + camber[r]);
        }

    auto camberAt = [&](float s){
        const float f = (s - sMin) / sSpan * (nBins - 1);
        int b0 = static_cast<int>(f);
        b0 = std::max(0, std::min(nBins - 2, b0));
        const float lt = f - b0;
        return camber[b0] * (1.0f - lt) + camber[b0 + 1] * lt;
    };

    // --- 3. 按中弧线精分上下表面：t > camber(s) → 上表面, 否则下表面 ---
    std::vector<std::pair<float,int>> up, lo; // (s, idx)
    up.reserve(n); lo.reserve(n);
    for(int i = 0; i < n; ++i)
    {
        if(T[i] >= camberAt(S[i])) up.emplace_back(S[i], i);
        else                       lo.emplace_back(S[i], i);
    }
    if(up.size() < 2 || lo.size() < 2)
        return false;
    std::sort(up.begin(), up.end(),
        [](const std::pair<float,int>& a, const std::pair<float,int>& b){ return a.first < b.first; });
    std::sort(lo.begin(), lo.end(),
        [](const std::pair<float,int>& a, const std::pair<float,int>& b){ return a.first < b.first; });

    // --- 4. 清洗每个面的折线（去离群 + 平滑），返回有序 3D 点列 ---
    auto cleanPoly = [&](const std::vector<std::pair<float,int>>& seq){
        std::vector<Eigen::Vector3f> poly;
        poly.reserve(seq.size());
        for(const auto& e : seq) poly.push_back(pts[e.second]);

        // 离群剔除：沿排序折线，段长远超中位数的点判为噪点丢弃(避免插值画长飞线)
        if(poly.size() >= 5)
        {
            std::vector<float> seg;
            seg.reserve(poly.size() - 1);
            for(size_t i = 1; i < poly.size(); ++i)
                seg.push_back((poly[i] - poly[i - 1]).norm());
            std::vector<float> sorted = seg;
            std::sort(sorted.begin(), sorted.end());
            const float medSeg = sorted[sorted.size() / 2];
            const float thr = std::max(medSeg * 4.0f, 1e-6f); // 阈值=4倍中位段长

            std::vector<Eigen::Vector3f> kept;
            kept.reserve(poly.size());
            // 端点修剪：首/末点若与相邻点距离异常大，视为离群丢弃
            const size_t iBeg = ((poly[1] - poly[0]).norm() > thr) ? 1 : 0;
            const size_t iEnd = ((poly[poly.size()-1] - poly[poly.size()-2]).norm() > thr)
                                ? poly.size() - 1 : poly.size();
            kept.push_back(poly[iBeg]);
            for(size_t i = iBeg + 1; i + 1 < iEnd; ++i)
            {
                // 该点前后两段都异常长 → 孤立离群点，丢弃
                const float dPrev = (poly[i] - poly[i - 1]).norm();
                const float dNext = (poly[i + 1] - poly[i]).norm();
                if(dPrev > thr && dNext > thr) continue;
                kept.push_back(poly[i]);
            }
            if(iEnd - 1 > iBeg) kept.push_back(poly[iEnd - 1]);
            poly.swap(kept);
        }

        // 移动平均平滑：抑制原始点噪声(半窗 w)，端点保持不动
        if(poly.size() >= 5)
        {
            const int w = 2;
            std::vector<Eigen::Vector3f> sm = poly;
            for(int i = w; i < static_cast<int>(poly.size()) - w; ++i)
            {
                Eigen::Vector3f acc = Eigen::Vector3f::Zero();
                for(int j = -w; j <= w; ++j) acc += poly[i + j];
                sm[i] = acc / static_cast<float>(2 * w + 1);
            }
            poly.swap(sm);
        }
        return poly;
    };
    std::vector<Eigen::Vector3f> polyUp = cleanPoly(up); // s: 小(LE)→大(TE)
    std::vector<Eigen::Vector3f> polyLo = cleanPoly(lo);
    if(polyUp.size() < 3 || polyLo.size() < 3) return false;

    // --- 5. 前后缘平滑汇聚：用圆弧拟合边缘区域，实现平滑过渡 ---
    auto to2D = [&](const Eigen::Vector3f& p){
        const Eigen::Vector3f d = p - centroid;
        return Eigen::Vector2f(d.dot(u), d.dot(v));
    };
    auto to3D = [&](const Eigen::Vector2f& q){ return centroid + q.x() * u + q.y() * v; };
    (void)to2D; (void)to3D;

    // 对一端边缘做汇聚：尖端 = 两面真实端点的中点。
    // 纯插值、不做圆拟合/外推：中点必在两个真实数据点连线上，
    // 数学上保证落在原始点云凸包内 → 绝不会向外鼓出。
    // (边缘点本就稀疏，圆拟合不可靠，外推是之前"鼓包"的根因，已弃用)
    auto smoothEdge = [&](bool atLE){
        if(polyUp.size() < 2 || polyLo.size() < 2) return;
        const Eigen::Vector3f eUp = atLE ? polyUp.front() : polyUp.back();
        const Eigen::Vector3f eLo = atLE ? polyLo.front() : polyLo.back();
        const Eigen::Vector3f tip = 0.5f * (eUp + eLo);
        polyUp[atLE ? 0 : polyUp.size()-1] = tip;
        polyLo[atLE ? 0 : polyLo.size()-1] = tip;
    };

    smoothEdge(true);   // 前缘 LE 汇聚
    smoothEdge(false);  // 后缘 TE 汇聚

    // --- 6. 每个面沿弧长等间隔重采样成开放曲线 ---
    auto resampleOut = [&](const std::vector<Eigen::Vector3f>& poly, PCLCloud::Ptr& out){
        std::vector<float> cum(poly.size(), 0.0f);
        for(size_t i = 1; i < poly.size(); ++i)
            cum[i] = cum[i - 1] + (poly[i] - poly[i - 1]).norm();
        const float total = cum.back();
        if(total < 1e-6f) return;
        for(int k = 0; k < nInterp; ++k)
        {
            const float target = static_cast<float>(k) / static_cast<float>(nInterp - 1) * total;
            auto it = std::lower_bound(cum.begin(), cum.end(), target);
            int idx = static_cast<int>(it - cum.begin());
            if(idx <= 0) idx = 1;
            if(idx >= static_cast<int>(poly.size())) idx = static_cast<int>(poly.size()) - 1;
            const float segLen = cum[idx] - cum[idx - 1];
            const float localT = segLen > 1e-9f ? (target - cum[idx - 1]) / segLen : 0.0f;
            const Eigen::Vector3f p = poly[idx - 1] * (1.0f - localT) + poly[idx] * localT;
            out->points.emplace_back(p.x(), p.y(), p.z());
        }
        out->width = static_cast<uint32_t>(out->points.size());
        out->height = 1;
    };
    resampleOut(polyUp, upperOut);
    resampleOut(polyLo, lowerOut);

    std::cout << "separateBladeSurfaces: " << n << " → upper " << upperOut->size()
              << " / lower " << lowerOut->size() << " (smooth edges)" << std::endl;
    return !upperOut->empty() && !lowerOut->empty();
}

// ========== 任务1：弦向切片重采样（基于几何特征，统一点数和分布）==========
PCLCloud::Ptr Point_Algorithm::resampleSectionByChordSlicing(
    const PCLCloud::Ptr& section, int nSlices)
{
    PCLCloud::Ptr result(new PCLCloud);
    if(!section || section->size() < 10 || nSlices < 5)
        return result;

    const int n = static_cast<int>(section->size());

    // --- 1. 粗排序：按绕质心极角排序，形成闭合轮廓 ---
    std::vector<Eigen::Vector3f> pts;
    pts.reserve(n);
    for(const auto& p : section->points)
        pts.emplace_back(p.x, p.y, p.z);

    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for(const auto& p : pts) centroid += p;
    centroid /= static_cast<float>(n);

    // PCA 求截面平面基底(u=弦向,v=厚度向)
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for(const auto& p : pts)
    { Eigen::Vector3f d = p - centroid; cov += d * d.transpose(); }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
    Eigen::Vector3f u = es.eigenvectors().col(2).normalized();
    Eigen::Vector3f v = es.eigenvectors().col(1).normalized();

    // 弦向上下表面拆分排序（消除最近邻/极角排序在薄叶型上的跨表面跳跃）：
    //   1) 每个点投影到 (弦向s, 厚度t)
    //   2) 按 t 的符号分上/下表面
    //   3) 上/下各自按 s 单调排序
    //   4) 拼成闭合轮廓：下表面(s增) → 上表面(s减)，首尾即两个端部，不自交
    std::vector<std::pair<float,int>> upper, lower; // (s, idx)
    upper.reserve(n); lower.reserve(n);
    for(int i = 0; i < n; ++i)
    {
        const Eigen::Vector3f d = pts[i] - centroid;
        const float s = d.dot(u);
        const float t = d.dot(v);
        if(t >= 0.0f) upper.emplace_back(s, i);
        else          lower.emplace_back(s, i);
    }
    std::sort(lower.begin(), lower.end(),
        [](const std::pair<float,int>& a, const std::pair<float,int>& b){ return a.first < b.first; });
    std::sort(upper.begin(), upper.end(),
        [](const std::pair<float,int>& a, const std::pair<float,int>& b){ return a.first > b.first; }); // s 递减

    std::vector<Eigen::Vector3f> loop;
    loop.reserve(n);
    for(const auto& e : lower) loop.push_back(pts[e.second]); // 下表面 s: 小→大
    for(const auto& e : upper) loop.push_back(pts[e.second]); // 上表面 s: 大→小
    if(loop.size() < 3) return result;

    // --- 2. 沿排序后的闭合轮廓等弧长插值，得到均匀分布的光滑曲线 ---
    // nSlices 在调试阶段直接作为插值点数(可定制，如 10000)；先按段长累积，再等间隔取点。
    const int nDense = std::max(50, nSlices);
    std::vector<float> cum(loop.size() + 1, 0.0f); // 闭合：末点接回首点
    for(size_t i = 1; i <= loop.size(); ++i)
        cum[i] = cum[i - 1] + (loop[i % loop.size()] - loop[i - 1]).norm();
    const float total = cum.back();
    if(total < 1e-6f) return result;

    for(int k = 0; k < nDense; ++k)
    {
        const float target = static_cast<float>(k) / static_cast<float>(nDense) * total;
        auto it = std::lower_bound(cum.begin(), cum.end(), target);
        int idx = static_cast<int>(it - cum.begin());
        if(idx <= 0) idx = 1;
        if(idx > static_cast<int>(loop.size())) idx = static_cast<int>(loop.size());
        const float segLen = cum[idx] - cum[idx - 1];
        const float localT = segLen > 1e-9f ? (target - cum[idx - 1]) / segLen : 0.0f;
        const Eigen::Vector3f p =
            loop[idx - 1] * (1.0f - localT) + loop[idx % loop.size()] * localT;
        result->points.emplace_back(p.x(), p.y(), p.z());
    }
    result->width = static_cast<uint32_t>(result->points.size());
    result->height = 1;

    std::cout << "resampleSectionByChordSlicing(interp only): " << n << " → "
              << result->size() << " points" << std::endl;
    return result;

    // ↓↓↓ 以下为后续步骤(圆拟合/拆弧/重采样)，当前调试阶段暂不执行 ↓↓↓
#if 0
    int idxLE = 0, idxTE = 0; // leading edge, trailing edge
    float maxDist = 0.0f;
    for(int i = 0; i < nDense; ++i)
        for(int j = i + 1; j < nDense; ++j)
        {
            const float d = (dense[i] - dense[j]).norm();
            if(d > maxDist) { maxDist = d; idxLE = i; idxTE = j; }
        }
    // 确保 TE(后缘) 在弦向 u 的正方向，LE(前缘) 在负方向
    const Eigen::Vector3f pTE = dense[idxTE], pLE = dense[idxLE];
    const Eigen::Vector3f chord0 = pTE - pLE;
    if(chord0.dot(u) < 0.0f) { std::swap(idxLE, idxTE); } // 调整方向

    // --- 2.5 曲率加强：最远两点只定大方向，在其附近小窗口找曲率最大点作更准尖端 ---
    // Menger 曲率 = 4*Area / (|AB||BC||CA|)，用间隔 w 的三点算(避免相邻点数值噪声)。
    auto mengerCurv = [&](int i, int w)->float{
        const int ia = (i - w + nDense) % nDense;
        const int ic = (i + w) % nDense;
        const Eigen::Vector3f& A = dense[ia];
        const Eigen::Vector3f& B = dense[i];
        const Eigen::Vector3f& C = dense[ic];
        const float a = (B - C).norm(), b = (A - C).norm(), c = (A - B).norm();
        if(a * b * c < 1e-9f) return 0.0f;
        const float area2 = ((B - A).cross(C - A)).norm(); // = 2*Area
        return 2.0f * area2 / (a * b * c);
    };
    auto refineIdxByCurvature = [&](int idxRough)->int{
        const int searchWin = std::max(5, nDense / 40);  // ±2.5% 曲线范围
        const int w = std::max(2, nDense / 200);
        int best = idxRough; float bestC = -1.0f;
        for(int off = -searchWin; off <= searchWin; ++off)
        {
            const int i = (idxRough + off + nDense) % nDense;
            const float cc = mengerCurv(i, w);
            if(cc > bestC) { bestC = cc; best = i; }
        }
        return best;
    };
    idxTE = refineIdxByCurvature(idxTE);
    idxLE = refineIdxByCurvature(idxLE);

    Eigen::Vector3f vTE = dense[idxTE], vLE = dense[idxLE];       // 顶点(下面再用圆拟合修正)
    const Eigen::Vector3f chordDir = (vTE - vLE).normalized();
    const float chordLen = (vTE - vLE).norm();

    std::cout << "  Chord: TE=" << vTE.transpose() << ", LE=" << vLE.transpose()
              << ", length=" << chordLen << std::endl;

    // --- 3.5 圆拟合补齐前后缘：高曲率区易丢点，取到的"最远点"不是真尖端 ---
    // 在截面平面(u,v)内做 2D 圆拟合(Kåsa法)，沿弦向外推出真实尖端顶点。
    auto to2D = [&](const Eigen::Vector3f& p){
        const Eigen::Vector3f d = p - centroid;
        return Eigen::Vector2f(d.dot(u), d.dot(v)); // (弦向, 厚度向)
    };
    auto to3D = [&](const Eigen::Vector2f& q){
        return centroid + q.x() * u + q.y() * v;
    };
    // Kåsa 圆拟合：返回是否成功，输出圆心(2D)与半径
    auto fitCircle2D = [](const std::vector<Eigen::Vector2f>& q,
                          Eigen::Vector2f& center, float& radius)->bool{
        const int m = static_cast<int>(q.size());
        if(m < 3) return false;
        // 解 [2x 2y 1][A B C]^T = x^2+y^2 的最小二乘
        Eigen::MatrixXf M(m, 3);
        Eigen::VectorXf b(m);
        for(int i = 0; i < m; ++i)
        {
            M(i,0) = 2.0f * q[i].x();
            M(i,1) = 2.0f * q[i].y();
            M(i,2) = 1.0f;
            b(i) = q[i].x()*q[i].x() + q[i].y()*q[i].y();
        }
        Eigen::Vector3f sol = M.colPivHouseholderQr().solve(b);
        center = Eigen::Vector2f(sol.x(), sol.y());
        const float r2 = sol.z() + center.squaredNorm();
        if(r2 <= 1e-9f) return false;
        radius = std::sqrt(r2);
        return true;
    };

    const Eigen::Vector2f cd2 = Eigen::Vector2f((vTE - vLE).dot(u), (vTE - vLE).dot(v)).normalized();
    const int kEdge = 7; // 只取尖端最近的 7 个高曲率点做圆拟合(多则不准)

    // 取距 tipRough 最近的 kEdge 个原始点做圆拟合，沿 outDir 外推出真实尖端(2D)
    auto refineTip = [&](const Eigen::Vector3f& tipRough, const Eigen::Vector2f& outDir,
                         Eigen::Vector3f& tipOut)->bool{
        // 按到粗尖端距离排序，取最近 kEdge 个
        std::vector<std::pair<float,int>> ds;
        ds.reserve(pts.size());
        for(int i = 0; i < static_cast<int>(pts.size()); ++i)
            ds.emplace_back((pts[i] - tipRough).squaredNorm(), i);
        const int k = std::min(kEdge, static_cast<int>(ds.size()));
        std::partial_sort(ds.begin(), ds.begin() + k, ds.end());
        std::vector<Eigen::Vector2f> nb;
        nb.reserve(k);
        for(int i = 0; i < k; ++i) nb.push_back(to2D(pts[ds[i].second]));

        Eigen::Vector2f c; float r;
        if(!fitCircle2D(nb, c, r)) return false;
        if(r > chordLen) return false;                 // 拟合发散，弃用
        tipOut = to3D(c + outDir * r);                 // 圆心沿外法向推 r → 尖端
        return true;
    };

    Eigen::Vector3f tipTE, tipLE;
    const bool okTE = refineTip(vTE,  cd2, tipTE);     // 后缘：弦向正方向外推
    const bool okLE = refineTip(vLE, -cd2, tipLE);     // 前缘：弦向负方向外推
    if(okTE) { vTE = tipTE; dense[idxTE] = tipTE; std::cout << "  TE refined by circle fit\n"; }
    if(okLE) { vLE = tipLE; dense[idxLE] = tipLE; std::cout << "  LE refined by circle fit\n"; }

    // --- 4. 以 TE/LE 为界，把闭合曲线拆成上下两条弧，再对每条弧等弧长重采样 ---
    // 关键：在连续插值曲线上重采样，每个站位必出一个点，点数恒为 nSlices，绝不会空。
    auto buildArc = [&](int from, int to) {
        std::vector<Eigen::Vector3f> arc;
        int i = from;
        arc.push_back(dense[i]);
        while(i != to) { i = (i + 1) % nDense; arc.push_back(dense[i]); }
        return arc;
    };
    std::vector<Eigen::Vector3f> arcA = buildArc(idxTE, idxLE); // TE→LE
    std::vector<Eigen::Vector3f> arcB = buildArc(idxLE, idxTE); // LE→TE

    // 等弧长重采样：在弧内部取 m 个点（不含两端顶点）
    auto resampleArcInterior = [](const std::vector<Eigen::Vector3f>& arc, int m) {
        std::vector<Eigen::Vector3f> out;
        if(arc.size() < 2 || m <= 0) return out;
        std::vector<float> cum(arc.size(), 0.0f);
        for(size_t i = 1; i < arc.size(); ++i)
            cum[i] = cum[i - 1] + (arc[i] - arc[i - 1]).norm();
        const float total = cum.back();
        if(total < 1e-9f) return out;
        out.reserve(m);
        for(int s = 0; s < m; ++s)
        {
            const float t = static_cast<float>(s + 1) / static_cast<float>(m + 1); // 内部站位
            const float target = t * total;
            auto it = std::lower_bound(cum.begin(), cum.end(), target);
            int idx = static_cast<int>(it - cum.begin());
            if(idx <= 0) idx = 1;
            if(idx >= static_cast<int>(arc.size())) idx = static_cast<int>(arc.size()) - 1;
            const float segLen = cum[idx] - cum[idx - 1];
            const float localT = segLen > 1e-9f ? (target - cum[idx - 1]) / segLen : 0.0f;
            out.push_back(arc[idx - 1] * (1.0f - localT) + arc[idx] * localT);
        }
        return out;
    };

    // 判定哪条弧是上表面（厚度方向正侧）
    Eigen::Vector3f thickDir = v - v.dot(chordDir) * chordDir;
    thickDir.normalize();
    const float signA = (arcA[arcA.size() / 2] - vLE).dot(thickDir);

    std::vector<Eigen::Vector3f> upperPts, lowerPts; // upper: TE→LE, lower: LE→TE
    if(signA >= 0.0f)
    {
        upperPts = resampleArcInterior(arcA, nSlices);          // arcA(TE→LE) 是上表面
        lowerPts = resampleArcInterior(arcB, nSlices);          // arcB(LE→TE) 是下表面
    }
    else
    {
        upperPts = resampleArcInterior(arcB, nSlices);          // arcB 是上表面(LE→TE)
        std::reverse(upperPts.begin(), upperPts.end());         // 翻成 TE→LE
        lowerPts = resampleArcInterior(arcA, nSlices);          // arcA 是下表面(TE→LE)
        std::reverse(lowerPts.begin(), lowerPts.end());         // 翻成 LE→TE
    }
    if(static_cast<int>(upperPts.size()) != nSlices ||
       static_cast<int>(lowerPts.size()) != nSlices)
        return result; // 重采样失败，返回空

    // --- 5. 组装最终输出：后缘顶点 + 上表面(TE→LE) + 前缘顶点 + 下表面(LE→TE) ---
    result->points.reserve(static_cast<size_t>(nSlices) * 2 + 2);
    result->points.emplace_back(vTE.x(), vTE.y(), vTE.z()); // 后缘顶点
    for(const auto& p : upperPts)
        result->points.emplace_back(p.x(), p.y(), p.z());
    result->points.emplace_back(vLE.x(), vLE.y(), vLE.z()); // 前缘顶点
    for(const auto& p : lowerPts)
        result->points.emplace_back(p.x(), p.y(), p.z());

    result->width = static_cast<uint32_t>(result->points.size());
    result->height = 1;

    std::cout << "resampleSectionByChordSlicing: " << n << " → " << result->size()
              << " points (" << nSlices << " slices)" << std::endl;
    return result;
#endif
}

std::vector<PCLCloud::Ptr> Point_Algorithm::splitCloudIntoSections(const PCLCloud::Ptr& cloud)
{
    std::vector<PCLCloud::Ptr> sections;
    if(!cloud || cloud->size() < 4)
        return sections;

    const int n = static_cast<int>(cloud->size());

    // --- 1. 主轴方向：PCA 主成分（沿叶片展向，各截面沿此轴依次排开）---
    const PCAResult pr = pcaAnalysis(cloud);
    Eigen::Vector3f axis, centroid;
    if(pr.success)
    {
        axis = pr.eigenvectors.col(0).normalized();
        centroid = pr.centroid;
    }
    else
    {
        centroid = Eigen::Vector3f::Zero();
        for(const auto& p : cloud->points) centroid += Eigen::Vector3f(p.x, p.y, p.z);
        centroid /= static_cast<float>(n);
        axis = Eigen::Vector3f::UnitX();
    }

    // --- 2. 全部点沿主轴投影，按投影值排序 ---
    std::vector<std::pair<float,int>> proj(n); // (t, 原始索引)
    for(int i = 0; i < n; ++i)
    {
        const auto& p = cloud->points[i];
        proj[i] = { (Eigen::Vector3f(p.x, p.y, p.z) - centroid).dot(axis), i };
    }
    std::sort(proj.begin(), proj.end(),
        [](const std::pair<float,int>& a, const std::pair<float,int>& b){ return a.first < b.first; });

    const float span = std::max(proj.back().first - proj.front().first, 1e-6f);

    // --- 3. 自适应间隙阈值：相邻投影差的中位数 × 系数，或 span 的 2%（取大者）---
    std::vector<float> gaps;
    gaps.reserve(n - 1);
    for(int i = 1; i < n; ++i) gaps.push_back(proj[i].first - proj[i-1].first);
    std::vector<float> sortedGaps = gaps;
    std::sort(sortedGaps.begin(), sortedGaps.end());
    const float medGap = sortedGaps.empty() ? 0.0f : sortedGaps[sortedGaps.size()/2];
    const float gapThresh = std::max(medGap * 8.0f, span * 0.02f);

    // --- 4. 在大于阈值的间隙处切段 ---
    std::vector<PCLCloud::Ptr> raw;
    PCLCloud::Ptr cur(new PCLCloud);
    cur->points.push_back(cloud->points[proj[0].second]);
    for(int i = 1; i < n; ++i)
    {
        if(gaps[i-1] > gapThresh)
        {
            raw.push_back(cur);
            cur.reset(new PCLCloud);
        }
        cur->points.push_back(cloud->points[proj[i].second]);
    }
    raw.push_back(cur);

    // --- 5. 过滤过小的段（噪声），保留点数 >=3 的截面 ---
    for(auto& sec : raw)
    {
        if(sec->size() >= 3)
        {
            sec->width = static_cast<uint32_t>(sec->points.size());
            sec->height = 1;
            sections.push_back(sec);
        }
    }

    std::cout << "splitCloudIntoSections: " << sections.size()
              << " sections (gapThresh=" << gapThresh << ", span=" << span << ")" << std::endl;
    return sections;
}

PCLMesh Point_Algorithm::loftReconstruction(
    const std::vector<PCLCloud::Ptr>& sections, int nSamples)
{
    PCLMesh output;
    if(sections.size() < 2)
        return output;

    // --- 使用新的 separateBladeSurfaces 方案：每截面分成上下面，合并成闭合环 ---
    const int nInterpPerSurface = (nSamples > 0) ? nSamples : 1000;  // 每个面的插值点数

    std::vector<std::vector<Eigen::Vector3f>> grid;
    grid.reserve(sections.size());

    // 参考方向：第一个成功的截面确定基准，后续截面对齐符号，避免螺旋扭曲
    Eigen::Vector3f refU = Eigen::Vector3f::Zero();
    Eigen::Vector3f refV = Eigen::Vector3f::Zero();

    for(const auto& sec : sections)
    {
        if(!sec || sec->size() < 10) continue;

        PCLCloud::Ptr upper, lower;
        if(!separateBladeSurfaces(sec, nInterpPerSurface, upper, lower, &refU, &refV))
            continue;
        if(upper->size() != lower->size())
            continue;

        // 合并成闭合环：upper[0..n-1] + lower[n-2..1] (跳过 lower 两端，与 upper 共享)
        std::vector<Eigen::Vector3f> ring;
        ring.reserve(upper->size() + lower->size() - 2);

        for(const auto& p : upper->points)
            ring.emplace_back(p.x, p.y, p.z);

        for(int i = static_cast<int>(lower->size()) - 2; i >= 1; --i)
        {
            const auto& p = lower->points[i];
            ring.emplace_back(p.x, p.y, p.z);
        }

        grid.push_back(std::move(ring));
    }

    if(grid.size() < 2)
    {
        std::cout << "Loft: too few valid sections (< 2)." << std::endl;
        return output;
    }

    // 检查所有环点数是否一致
    const int nPerRow = static_cast<int>(grid[0].size());
    for(size_t k = 0; k < grid.size(); ++k)
    {
        if(static_cast<int>(grid[k].size()) != nPerRow)
        {
            std::cout << "Loft: section " << k << " has " << grid[k].size()
                      << " points, expected " << nPerRow << std::endl;
            return output;
        }
    }

    // --- 生成顶点：nSect 行 × nPerRow 列 ---
    const int nSect = static_cast<int>(grid.size());
    pcl::PointCloud<pcl::PointXYZ> verts;
    verts.reserve(nSect * nPerRow);
    for(int k = 0; k < nSect; ++k)
        for(int j = 0; j < nPerRow; ++j)
            verts.emplace_back(grid[k][j].x(), grid[k][j].y(), grid[k][j].z());
    pcl::toPCLPointCloud2(verts, output.cloud);

    // --- 蒙皮三角化：ring-to-ring ---
    auto vidx = [nPerRow](int k, int j) { return k * nPerRow + (j % nPerRow); };
    output.polygons.reserve(2 * (nSect - 1) * nPerRow);
    for(int k = 0; k < nSect - 1; ++k)
    {
        for(int j = 0; j < nPerRow; ++j)
        {
            pcl::Vertices tri1;
            tri1.vertices = { vidx(k, j), vidx(k + 1, j), vidx(k + 1, j + 1) };
            output.polygons.push_back(tri1);

            pcl::Vertices tri2;
            tri2.vertices = { vidx(k, j), vidx(k + 1, j + 1), vidx(k, j + 1) };
            output.polygons.push_back(tri2);
        }
    }

    std::cout << "Loft reconstruction: " << nSect << " sections x "
              << nPerRow << " points/section → " << output.polygons.size()
              << " triangles" << std::endl;
    return output;
}
