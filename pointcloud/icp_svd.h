#include <vector>

// 简单的 3D 点结构（与 .cu 文件中定义一致）
struct ICP_Point3D {
    float x, y, z;
};

// SVD 求解变换矩阵（使用 Eigen，在 CPU 上）
void computeTransformationSVD(
    const std::vector<ICP_Point3D>& src_points,
    const std::vector<ICP_Point3D>& tgt_points,
    float* transform_out);
