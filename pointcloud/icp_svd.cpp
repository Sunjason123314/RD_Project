#include "icp_svd.h"
#include <Eigen/Dense>

// SVD 求解变换矩阵（使用 Eigen，在 CPU 上）
void computeTransformationSVD(
    const std::vector<ICP_Point3D>& src_points,
    const std::vector<ICP_Point3D>& tgt_points,
    float* transform_out)
{
    if(src_points.empty() || src_points.size() != tgt_points.size())
    {
        // 返回单位矩阵
        for(int i = 0; i < 16; ++i)
            transform_out[i] = (i % 5 == 0) ? 1.0f : 0.0f;
        return;
    }

    // 计算质心
    Eigen::Vector3f src_centroid = Eigen::Vector3f::Zero();
    Eigen::Vector3f tgt_centroid = Eigen::Vector3f::Zero();

    for(size_t i = 0; i < src_points.size(); ++i)
    {
        src_centroid += Eigen::Vector3f(src_points[i].x, src_points[i].y, src_points[i].z);
        tgt_centroid += Eigen::Vector3f(tgt_points[i].x, tgt_points[i].y, tgt_points[i].z);
    }
    src_centroid /= static_cast<float>(src_points.size());
    tgt_centroid /= static_cast<float>(tgt_points.size());

    // 计算协方差矩阵
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    for(size_t i = 0; i < src_points.size(); ++i)
    {
        Eigen::Vector3f src_demean = Eigen::Vector3f(src_points[i].x, src_points[i].y, src_points[i].z) - src_centroid;
        Eigen::Vector3f tgt_demean = Eigen::Vector3f(tgt_points[i].x, tgt_points[i].y, tgt_points[i].z) - tgt_centroid;
        H += src_demean * tgt_demean.transpose();
    }

    // SVD 分解
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();
    Eigen::Matrix3f R = V * U.transpose();

    // 处理反射
    if(R.determinant() < 0)
    {
        V.col(2) *= -1;
        R = V * U.transpose();
    }

    // 计算平移
    Eigen::Vector3f t = tgt_centroid - R * src_centroid;

    // 构建 4x4 变换矩阵
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = R;
    transform.block<3, 1>(0, 3) = t;

    // 输出为行优先数组
    for(int i = 0; i < 4; ++i)
        for(int j = 0; j < 4; ++j)
            transform_out[i * 4 + j] = transform(i, j);
}
