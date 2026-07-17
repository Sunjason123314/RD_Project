// ball_fit.h
#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <QObject>
#include <QVector>
#include <QString>
#include <Eigen/Dense>
#include <vector>
#include <QDebug>

using namespace Eigen;
using namespace std;



class Calibration : public QObject
{
    Q_OBJECT
public:
    explicit Calibration(QObject* parent = nullptr);

    void gainInitialParameter(
        const QVector<QVector<MatrixXd>>& Points_test_all,
        Matrix3d& M_g,
        QVector<QVector<Vector3d>>& initial_center) ;

    void nonLinearOptimalMulti(
        const Eigen::Matrix3d& M,
        const QVector<QVector<Eigen::MatrixXd>>& Points_tri,
        double real_r_1,
        double real_r_2,
        double ball_distance,
        const QVector<QVector<Eigen::MatrixXd>>& sensor_t_all,
        QVector<QVector<Eigen::Vector3d>>& center_all,
        Eigen::VectorXd& theta,
        Eigen::Matrix3d& M_opt,
        double& radius_1,
        double& radius_2
        );


private:

    std::tuple<Matrix3d, Vector3d, VectorXd> fitEllipsoid(const MatrixXd& points);
    std::pair<Matrix3d, Vector3d> ellipsoidCenter(const VectorXd& coeffs);
    static Matrix3d customRotationMatrix(double theta1, double theta2, double theta3);
    static void fitSphere(const Eigen::MatrixXd& X, Eigen::Vector3d& center, double& radius);
    struct OptimizationParameters {
        const QVector<QVector<MatrixXd>>* Points_tri;
        const QVector<QVector<MatrixXd>>* sensor_t_all;
        double real_r_1;
        double real_r_2;
        double ball_distance;
        int num;
    };

    struct FittingObjective {
        int m_inputs;
        int m_values;
        const OptimizationParameters* params;

        FittingObjective(int inputs, int values, const OptimizationParameters* p)
            : m_inputs(inputs), m_values(values), params(p) {}

        int inputs() const { return m_inputs; }
        int values() const { return m_values; }

        int operator()(const Eigen::VectorXd& param, Eigen::VectorXd& residual) const;
        int df(const VectorXd& x, MatrixXd& J) const;
    };

signals:

};

#endif // CALIBRATION_H
