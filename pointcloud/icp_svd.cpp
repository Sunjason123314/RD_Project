#include "icp_svd.h"
#include "Point_Algorithm.h"
#include <Eigen/Dense>

// SVD 求解变换矩阵（使用 Eigen，在 CPU 上）
// 说明：本函数与 Point_Algorithm::estimateRigidTransformSVD 算法完全一致
// （Kabsch-Umeyama），此处作为薄封装：转换点结构 -> 调用共用实现 -> 拆成行优先数组，
// 避免同一 SVD 刚体变换算法存在多份重复实现。
void computeTransformationSVD(
    const std::vector<ICP_Point3D>& src_points,
    const std::vector<ICP_Point3D>& tgt_points,
    float* transform_out)
{
    auto writeMatrix = [transform_out](const Eigen::Matrix4f& m) {
        // 输出为行优先数组
        for(int i = 0; i < 4; ++i)
            for(int j = 0; j < 4; ++j)
                transform_out[i * 4 + j] = m(i, j);
    };

    if(src_points.empty() || src_points.size() != tgt_points.size())
    {
        // 返回单位矩阵
        writeMatrix(Eigen::Matrix4f::Identity());
        return;
    }

    // ICP_Point3D -> Eigen::Vector3f
    std::vector<Eigen::Vector3f> src(src_points.size());
    std::vector<Eigen::Vector3f> dst(tgt_points.size());
    for(size_t i = 0; i < src_points.size(); ++i)
    {
        src[i] = Eigen::Vector3f(src_points[i].x, src_points[i].y, src_points[i].z);
        dst[i] = Eigen::Vector3f(tgt_points[i].x, tgt_points[i].y, tgt_points[i].z);
    }

    // 复用统一的刚体变换实现；退化时结构体默认即单位矩阵
    const auto result = Point_Algorithm::estimateRigidTransformSVD(src, dst);
    writeMatrix(result.transformation);
}
