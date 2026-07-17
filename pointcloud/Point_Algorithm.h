#ifndef POINT_ALGORITHM_H
#define POINT_ALGORITHM_H

#include "dicqglviewer.h"

// 点云算法集合：
// 1) 预处理（体素下采样、SOR离群点滤波）
// 2) 基础几何拟合（平面/球/圆柱/3D圆）
// 3) 拟合结果采样（将几何体转成点云用于显示）
class Point_Algorithm
{
public:
    // 平面拟合结果：ax + by + cz + d = 0
    struct PlaneFitResult
    {
        bool success = false;
        Eigen::Vector4f coefficients = Eigen::Vector4f::Zero(); // ax + by + cz + d = 0
        PCLCloud::Ptr inliers{new PCLCloud};
    };

    // 球拟合结果：center + radius
    struct SphereFitResult
    {
        bool success = false;
        Eigen::Vector3f center = Eigen::Vector3f::Zero();
        float radius = 0.0f;
        PCLCloud::Ptr inliers{new PCLCloud};
    };

    // SVD 刚性变换结果
    struct RigidTransformResult
    {
        bool success = false;
        Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
        float rmsError = 0.0f;
    };

    // 圆柱拟合结果：轴上一点 + 轴方向 + 半径
    struct CylinderFitResult
    {
        bool success = false;
        Eigen::Vector3f axisPoint = Eigen::Vector3f::Zero();
        Eigen::Vector3f axisDirection = Eigen::Vector3f::UnitZ();
        float radius = 0.0f;
        PCLCloud::Ptr inliers{new PCLCloud};
    };

    // 三维圆拟合结果：圆心 + 法向 + 半径
    struct Circle3DFitResult
    {
        bool success = false;
        Eigen::Vector3f center = Eigen::Vector3f::Zero();
        Eigen::Vector3f normal = Eigen::Vector3f::UnitZ();
        float radius = 0.0f;
        PCLCloud::Ptr inliers{new PCLCloud};
    };

    // 回转轴拟合结果：轴上一点 + 轴方向 + 轨迹半径 + 拟合误差。
    struct RotationAxisFitResult
    {
        bool success = false;
        Eigen::Vector3f axisPoint = Eigen::Vector3f::Zero();
        Eigen::Vector3f axisDirection = Eigen::Vector3f::UnitZ();
        float radius = 0.0f;
        float planeRMSE = 0.0f;
        float circleRMSE = 0.0f;
        PCLCloud::Ptr circle3D{new PCLCloud};
    };

    // 体素下采样：按 leaf 尺寸合并邻近点，降低点数与噪声。
    static PCLCloud::Ptr voxelGridDownsample(const PCLCloud::Ptr& input,
                                             float leafX,
                                             float leafY,
                                             float leafZ);

    // SOR 统计滤波：移除局部邻域统计上偏离较大的离群点。
    static PCLCloud::Ptr sorFilter(const PCLCloud::Ptr& input,
                                   int meanK,
                                   double stddevMulThresh);

    // MLS 移动最小二乘平滑重采样。
    static PCLCloud::Ptr mlsSmoothing(const PCLCloud::Ptr& input,
                                      double searchRadius,
                                      int polynomialOrder = 2);

    // 表面重建方法
    enum class ReconMethod { Greedy, Poisson, Nurbs };

    // 表面重建：根据指定方法从点云重建三角网格。
    // voxelLeafSize / maxSurfaceAngleDeg 仅对 Greedy 方法生效。
    static PCLMesh reconstructMesh(const PCLCloud::Ptr& input, ReconMethod method,
                                   float voxelLeafSize = 1.0f,
                                   float maxSurfaceAngleDeg = 45.0f,
                                   int nurbsOrder = 3,
                                   int nurbsRefinement = 5,
                                   int nurbsResolution = 64,
                                   int poissonDepth = 8,
                                   int poissonSolverDivide = 5,
                                   int poissonIsoDivide = 5);

    // NURBS 曲面重建，返回三角形网格。
    static PCLMesh nurbsReconstruction(const PCLCloud::Ptr& input,
                                        int order = 3,
                                        int refinement = 5,
                                        int resolution = 64);

    // 网格精简到目标面数（渐进式 Quadric Decimation）。
    static void decimateMesh(PCLMesh& mesh, int targetFaces);

    // 曲率突变点检测结果
    struct FeaturePointsResult
    {
        std::vector<bool> isFeature;       // 每个点是否为特征点
        std::vector<float> curvatures;     // 每个点的曲率值
        std::vector<float> curvatureGrad;  // 每个点的曲率梯度（与邻域曲率差的RMS）
        float gradientThreshold;           // 判定阈值
        std::size_t featureCount;
    };

    // 计算点云每个点的表面曲率（通过 PCA 特征值比，0=平坦，越大约曲）
    static std::vector<float> computePointCurvature(const PCLCloud::Ptr& cloud,
                                                     int kNeighbors = 20);

    // 检测曲率突变点（特征棱线/顶点）：
    // 对每个点计算其曲率与邻域点曲率的差异（曲率梯度），
    // 梯度超过阈值（邻域梯度均值+N倍标准差）的标记为特征点
    static FeaturePointsResult detectFeaturePoints(const PCLCloud::Ptr& cloud,
                                                    float searchRadius,
                                                    float stddevMultiplier = 2.0f);

    // DoN（Difference of Normals）边缘检测：
    // 对每个点用两个半径分别估算法向，法向夹角越大越可能位于边缘/棱线
    // angleThresholdDeg: 法向夹角超过此值视为边缘点
    static std::vector<bool> detectEdgesDoN(const PCLCloud::Ptr& cloud,
                                             float smallRadius,
                                             float largeRadius,
                                             float angleThresholdDeg = 15.0f);

    // 联合特征提取：曲率突变点 ∪ DoN边缘点 → 合并为特征点云
    // 大曲率区+棱线保留原始密度，平坦区可大幅降采样后用于重建
    static PCLCloud::Ptr extractFeatureCloud(const PCLCloud::Ptr& cloud,
                                              float searchRadius,
                                              float donSmallRadius,
                                              float donLargeRadius);

    // 曲率自适应降采样：高曲率区保留细节（小体素），平坦区大幅压缩（大体素）
    // curvaturePercentile: 前 N% 的高曲率点视为特征区
    // featureLeafSize: 特征区体素尺寸（较小）
    // flatLeafSize: 平坦区体素尺寸（较大）
    static PCLCloud::Ptr adaptiveCurvatureDownsample(const PCLCloud::Ptr& input,
                                                      float featureLeafSize,
                                                      float flatLeafSize,
                                                      float curvaturePercentile = 20.0f);

    // 网格修复：去除退化面片、合并重复顶点、修复非流形边
    static int repairMesh(PCLMesh& mesh);

    // 根据 NURBS resolution 参数估算三角面数：≈ 2*(res-1)^2
    static int numFacesFromNurbsResolution(int resolution);

    // 根据目标面数反推 NURBS resolution（不超过 300 面时的推荐分辨率）
    static int nurbsResolutionForTargetFaces(int targetFaces);

    // 3D平面拟合（RANSAC）。distanceThreshold 越小越严格。
    static PlaneFitResult fitPlane3D(const PCLCloud::Ptr& input, double distanceThreshold = 0.5);
    // 3D球拟合（RANSAC）。
    static SphereFitResult fitSphere3D(const PCLCloud::Ptr& input, double distanceThreshold = 0.5);

    // 最小二乘球拟合（非RANSAC，适用于无离群点的干净数据）
    // 输入：3×N Eigen矩阵，每列是一个点 (x,y,z)
    // 输出：球心和半径，通过引用参数返回
    static void fitSphereLeastSquares(const Eigen::MatrixXd& points,
                                      Eigen::Vector3d& center,
                                      double& radius);
    // SVD 刚性变换：由对应点对计算最优旋转平移 (Kabsch-Umeyama)
    static RigidTransformResult estimateRigidTransformSVD(
        const std::vector<Eigen::Vector3f>& src,
        const std::vector<Eigen::Vector3f>& dst);
    // 3D圆柱拟合：基于点法向约束 + RANSAC。
    static CylinderFitResult fitCylinder3D(const PCLCloud::Ptr& input,
                                           double normalDistanceWeight = 0.1,
                                           int maxIterations = 2000,
                                           double distanceThreshold = 0.5,
                                           float minRadius = 0.1f,
                                           float maxRadius = 1e6f);
    // 3D圆拟合（SACMODEL_CIRCLE3D）。
    static Circle3DFitResult fitCircle3D(const PCLCloud::Ptr& input,
                                         int maxIterations = 2000,
                                         double distanceThreshold = 0.5,
                                         float minRadius = 0.1f,
                                         float maxRadius = 1e6f);

    // 多球心回转轴拟合（非RANSAC）：3点几何解，3点以上最小二乘平面+2D圆拟合。
    static RotationAxisFitResult fitRotationAxisFromCenters(const std::vector<Eigen::Vector3f>& centers);

    // 采样平面补丁：根据参考点云尺度生成一块可视化平面点云。
    static PCLCloud::Ptr samplePlanePatch(const Eigen::Vector4f& plane,
                                          const PCLCloud::Ptr& reference,
                                          int resolution = 50);
    // 采样球面：按经纬网格生成可视化点云。
    static PCLCloud::Ptr sampleSphereSurface(const Eigen::Vector3f& center,
                                             float radius,
                                             int rings = 36,
                                             int segments = 72);
    // 采样圆柱侧面：轴向步进 + 周向步进生成点云。
    static PCLCloud::Ptr sampleCylinderSurface(const Eigen::Vector3f& axisPoint,
                                               const Eigen::Vector3f& axisDirection,
                                               float radius,
                                               const PCLCloud::Ptr& reference,
                                               int axisSteps = 80,
                                               int radialSteps = 72);
    // 采样3D圆周：在圆平面内按角度均匀采样。
    static PCLCloud::Ptr sampleCircle3D(const Eigen::Vector3f& center,
                                        const Eigen::Vector3f& normal,
                                        float radius,
                                        int segments = 180);

    // 从网格面片按面积加权随机采样点云（类比 MATLAB STL→PCD 采样）。
    static PCLCloud::Ptr samplePointsFromMesh(const PCLMesh& mesh, int numPoints);

    // 按输入点云边界裁剪网格：移除所有顶点在扩展包围盒外的三角面片。
    static void clipMeshToCloudBounds(PCLMesh& mesh, const PCLCloud::Ptr& cloud,
                                       float marginRatio = 0.03f);

    // 估计点云法向量并一致化朝向（所有法向指向视点方向）。
    // 返回一致化后的法向量云，与原输入点云一一对应。
    static pcl::PointCloud<pcl::Normal>::Ptr estimateConsistentNormals(
        const PCLCloud::Ptr& cloud,
        int kSearch = 20,
        float vpX = 0.0f, float vpY = 0.0f, float vpZ = -1.0f);

    // PCA 主成分分析结果。
    struct PCAResult
    {
        bool success = false;
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        Eigen::Vector3f eigenvalues = Eigen::Vector3f::Zero();   // 降序排列
        Eigen::Matrix3f eigenvectors = Eigen::Matrix3f::Identity(); // 列向量为主方向
    };

    // 对点云计算 PCA 主成分分析。
    static PCAResult pcaAnalysis(const PCLCloud::Ptr& cloud);
};

#endif // POINT_ALGORITHM_H
