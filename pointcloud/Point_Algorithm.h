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
    enum class ReconMethod { Greedy, Poisson, Nurbs, Loft };

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

    // 将单个截面（乱序点）排成开放链并按弧长重采样为 nSamples 个点。
    // 针对翼型/细长开放弧：沿主弦方向投影排序，从一端排到另一端，不闭合。
    // refChord 为全局参考弦向（各截面共用，保证方向一致，无需事后翻转）；
    //   传 Zero 时函数内部用本截面 PCA 主方向。
    // 失败（点数不足）返回空 vector。
    // refChord/refThick 为截面平面内的弦向/厚度向基底（各截面共用同一符号→保证绕行方向一致）；
    //   传 Zero 时函数内部用本截面 PCA 估计。按绕质心极角排成闭合轮廓再弧长重采样。
    static std::vector<Eigen::Vector3f> orderAndResampleSection(
        const PCLCloud::Ptr& section,
        int nSamples,
        const Eigen::Vector3f& refChord = Eigen::Vector3f::Zero(),
        const Eigen::Vector3f& refThick = Eigen::Vector3f::Zero());

    // 蒙皮（Loft/放样）重建：多个有序开放截面 → 三角网格。
    // sections 需按空间顺序（如沿主轴）排好；每个截面内部乱序，函数内部排序+重采样。
    // 相邻两截面的对应重采样点连成四边形（每四边形拆 2 个三角形）。
    // nSamples <= 0 时自动取各截面点数的中位数（截断到 [20,400]）。
    static PCLMesh loftReconstruction(
        const std::vector<PCLCloud::Ptr>& sections,
        int nSamples = 0);

    // 将”已含多个离散截面”的单个点云沿主轴按间隙自动切分成多个截面。
    // 用于蒙皮：五个截面扫在同一片点云里时，按主轴投影上的空隙聚类分段。
    // 截面数由数据自动决定（无需外部指定）。切不出 >=2 段时返回空。
    static std::vector<PCLCloud::Ptr> splitCloudIntoSections(const PCLCloud::Ptr& cloud);

    // 【任务1】弦向切片重采样：沿弦线做垂面，基于几何特征统一各截面的点数和分布。
    // nSlices: 垂面数量（用户指定）。
    // 返回：有序点云，点数 = nSlices×2+2（每垂面上下各1点 + 前后缘顶点）。
    // 流程：粗排序 → 插值 → 找弦线端点 → 沿弦线放垂面 → 每垂面收集上下点 → 组装输出。
    static PCLCloud::Ptr resampleSectionByChordSlicing(
        const PCLCloud::Ptr& section, int nSlices);

    // 【方案二】用中弧线把截面分成上/下两个面(叶盆/叶背)，各自沿弦向等弧长插值。
    // camber-aware：先按弦线粗分 → 拟合中弧线 → 按中弧线精分，对弯度鲁棒。
    // nInterp: 每个面插值点数。两个面各自是一条开放曲线，不闭合。
    // 输出经 upperOut / lowerOut 返回（分别是叶背/叶盆），成功返回 true。
    // refU/refV: 可选的参考方向(用于多截面对齐)。非空时：
    //   - 输入若已初始化(非零)，则强制本截面 u/v 与其符号一致，避免截面间翻转；
    //   - 输出写回本截面实际使用的 u/v，供下一截面作参考。
    static bool separateBladeSurfaces(
        const PCLCloud::Ptr& section, int nInterp,
        PCLCloud::Ptr& upperOut, PCLCloud::Ptr& lowerOut,
        Eigen::Vector3f* refU = nullptr, Eigen::Vector3f* refV = nullptr);

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

    // 多条与回转轴近似平行的圆柱轴线联合标定回转轴（虚拟圆柱拟合）。
    // 先以最小二乘估计共同方向，再把各直线投影到其垂直平面并拟合圆轨迹；
    // linePoints 可为各轴线上的任意点，因此不受点云截取长度或轴线中点漂移影响。
    static RotationAxisFitResult fitRotationAxisFromLines(const std::vector<Eigen::Vector3f>& linePoints,
                                                          const std::vector<Eigen::Vector3f>& lineDirections);

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
