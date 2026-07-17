// ball_fit.cpp
#include "calibration.h"
#include "pointcloud/Point_Algorithm.h"
#include <fstream>
#include <stdexcept>
#include <numeric>
#include <Eigen/SVD>
#include <unsupported/Eigen/NonLinearOptimization>
#include <QFile>
#include <QDebug>
#define M_PI 3.14159265358979323846
Calibration::Calibration(QObject* parent)
    : QObject(parent)
{
}
Matrix3d Calibration::customRotationMatrix(double theta1, double theta2, double theta3)
{
    // θ1, θ2, θ3 均为弧度制
    double a = -sin(theta1);
    double b =  cos(theta1);
    double c = -sin(theta2);
    double d = -(sin(theta3) + sin(theta2) * sin(theta1)) / cos(theta2);
    double e = sqrt(1 - c * c - d * d);

    Matrix3d M;
    M << 1, a, c,
        0, b, d,
        0, 0, e;
    return M;
}


// 获取初值
void Calibration::gainInitialParameter(
    const QVector<QVector<MatrixXd>>& Points_test_all,
    Matrix3d& M_g,
    QVector<QVector<Vector3d>>& initial_center)
{
    int num = Points_test_all[0].size();
    Matrix3d M_sum = Matrix3d::Zero();

    initial_center.resize(2);
    initial_center[0].resize(num);
    initial_center[1].resize(num);

    for (int k = 0; k < num; ++k) {
        // 拟合 A球
        auto [M1, center1, coeffs1] = fitEllipsoid(Points_test_all[0][k]);

        // 拟合 B球
        auto [M2, center2, coeffs2] = fitEllipsoid(Points_test_all[1][k]);

        // 累加旋转矩阵
        M_sum += M1 + M2;

        // 存储中心
        initial_center[0][k] = center1;
        initial_center[1][k] = center2;
    }

    M_g = M_sum / (2.0 * num); // 求平均旋转矩阵
}

std::tuple<Matrix3d, Vector3d, VectorXd> Calibration::fitEllipsoid(const MatrixXd& points)
{
    int N = points.cols();
    VectorXd x = points.row(0).transpose();
    VectorXd y = points.row(1).transpose();
    VectorXd z = points.row(2).transpose();

    MatrixXd D(N, 10);
    D.col(0) = x.array().square();
    D.col(1) = y.array().square();
    D.col(2) = z.array().square();
    D.col(3) = x.array() * y.array();
    D.col(4) = x.array() * z.array();
    D.col(5) = y.array() * z.array();
    D.col(6) = x;
    D.col(7) = y;
    D.col(8) = z;
    D.col(9) = VectorXd::Ones(N);

    JacobiSVD<MatrixXd> svd(D, ComputeThinV);
    VectorXd coeffs = svd.matrixV().col(9); // 最小奇异值对应的椭球参数

    auto [U, center] = ellipsoidCenter(coeffs);
    return {U, center, coeffs};
}

std::pair<Matrix3d, Vector3d> Calibration::ellipsoidCenter(const VectorXd& coeffs)
{
    // Extract parameters
    double A = coeffs(0), B = coeffs(1), C = coeffs(2);
    double D = coeffs(3), E = coeffs(4), F = coeffs(5);
    double G = coeffs(6), H = coeffs(7), I = coeffs(8);

    // 构造 Q 和 l 向量
    Matrix3d Q;
    Q << A, D / 2.0, E / 2.0,
        D / 2.0, B, F / 2.0,
        E / 2.0, F / 2.0, C;

    Vector3d l(G, H, I);

    // K = Q/Q(1,1)，类似 MATLAB 的 K = Q / Q(1)
    Matrix3d K = Q / Q(0, 0);

    // Cholesky 分解 K = L * L^T，取 U = L^T
    Eigen::LLT<Matrix3d> llt(K);
    Matrix3d U = llt.matrixU();

    // 椭球中心解：center = U * (-0.5 * Q⁻¹ * l)
    Vector3d center = -0.5 * Q.inverse() * l;
    center = U * center;

    return {U, center};
}


// === 数值雅可比矩阵 ===
int Calibration::FittingObjective::df(const VectorXd& x, MatrixXd& J) const
{
    double eps = 1e-8;
    VectorXd fx;
    (*this)(x, fx);
    J.resize(fx.size(), x.size());

    for (int i = 0; i < x.size(); ++i) {
        VectorXd x1 = x;
        x1(i) += eps;
        VectorXd fx1;
        (*this)(x1, fx1);
        J.col(i) = (fx1 - fx) / eps;
    }

    return 0;
}


// === 主接口 ===
void Calibration::nonLinearOptimalMulti(
    const Matrix3d& M,
    const QVector<QVector<MatrixXd>>& Points_tri,
    double real_r_1,
    double real_r_2,
    double ball_distance,
    const QVector<QVector<MatrixXd>>& sensor_t_all,
    QVector<QVector<Vector3d>>& center_all,
    VectorXd& theta,
    Matrix3d& M_opt,
    double& radius_1,
    double& radius_2)
{
    int num = Points_tri[0].size();
    VectorXd param(5);

    // 初始值
    param(0) = -asin(M(0, 1));
    param(1) = -asin(M(0, 2));
    param(2) = -asin(M(1, 2));
    param(3) = 0.0;
    param(4) = 0.0;

    OptimizationParameters p;
    p.Points_tri = &Points_tri;
    p.sensor_t_all = &sensor_t_all;
    p.real_r_1 = real_r_1;
    p.real_r_2 = real_r_2;
    p.ball_distance = ball_distance;
    p.num = num;

    int total_residuals = 3 * num; // 每组包括两个半径误差和一个球心间距误差

    FittingObjective functor(param.size(), total_residuals, &p);
    LevenbergMarquardt<FittingObjective> lm(functor);
    lm.parameters.maxfev = 1000;
    lm.parameters.xtol = 1e-8;

    // 优化
    lm.minimize(param);
    theta = param.head(5);
    M_opt = Calibration::customRotationMatrix(theta(0), theta(1), theta(2));

    double alpha = theta(3), beta = theta(4);
    center_all.resize(2);
    center_all[0].resize(num);
    center_all[1].resize(num);
    std::vector<double> d1, d2;

    for (int k = 0; k < num; ++k) {
        MatrixXd P1 = M_opt * Points_tri[0][k];
        MatrixXd P2 = M_opt * Points_tri[1][k];

        MatrixXd offset1 = MatrixXd::Zero(3, P1.cols());
        MatrixXd offset2 = MatrixXd::Zero(3, P2.cols());

        for (int i = 0; i < P1.cols(); ++i) {
            double t1 = sensor_t_all[0][k](0, i);
            double t2 = sensor_t_all[1][k](0, i);

            offset1.col(i) << t1 * sin(alpha) * sin(beta),
                t1 * cos(alpha),
                t1 * sin(alpha) * cos(beta);

            offset2.col(i) << t2 * sin(alpha) * sin(beta),
                t2 * cos(alpha),
                t2 * sin(alpha) * cos(beta);
        }

        MatrixXd P1_shifted = P1 + offset1;
        MatrixXd P2_shifted = P2 + offset2;

        Vector3d c1, c2;
        double r1, r2;
        Calibration::fitSphere(P1_shifted, c1, r1);
        Calibration::fitSphere(P2_shifted, c2, r2);

        center_all[0][k] = c1;
        center_all[1][k] = c2;

        d1.push_back(r1);
        d2.push_back(r2);
    }

    radius_1 = std::accumulate(d1.begin(), d1.end(), 0.0) / d1.size();
    radius_2 = std::accumulate(d2.begin(), d2.end(), 0.0) / d2.size();
}


// === 拟合球函数 ===
void Calibration::fitSphere(const MatrixXd& X, Vector3d& center, double& radius)
{
    Point_Algorithm::fitSphereLeastSquares(X, center, radius);
}


// === 残差函数 ===
int Calibration::FittingObjective::operator()(const VectorXd& param, VectorXd& residual) const {
    const auto& Points_tri = *params->Points_tri;
    const auto& sensor_t_all = *params->sensor_t_all;
    int num = params->num;

    double theta1 = param(0), theta2 = param(1), theta3 = param(2);
    double alpha = param(3), beta = param(4);
    Matrix3d M = Calibration::customRotationMatrix(theta1, theta2, theta3);

    residual.resize(3 * num);

    for (int k = 0; k < num; ++k) {
        MatrixXd P1 = M * Points_tri[0][k];
        MatrixXd P2 = M * Points_tri[1][k];

        MatrixXd offset1(3, P1.cols()), offset2(3, P2.cols());
        for (int i = 0; i < P1.cols(); ++i) {
            double t1 = sensor_t_all[0][k](0, i);
            double t2 = sensor_t_all[1][k](0, i);
            offset1.col(i) << t1 * sin(alpha) * sin(beta),
                t1 * cos(alpha),
                t1 * sin(alpha) * cos(beta);
            offset2.col(i) << t2 * sin(alpha) * sin(beta),
                t2 * cos(alpha),
                t2 * sin(alpha) * cos(beta);
        }

        MatrixXd P1_shifted = P1 + offset1;
        MatrixXd P2_shifted = P2 + offset2;

        Vector3d c1, c2;
        double r1, r2;
        Calibration::fitSphere(P1_shifted, c1, r1);
        Calibration::fitSphere(P2_shifted, c2, r2);

        residual(3 * k + 0) = r1 - params->real_r_1;
        residual(3 * k + 1) = r2 - params->real_r_2;
        residual(3 * k + 2) = (c1 - c2).norm() - params->ball_distance;
    }

    return 0;
}
