#ifndef FILE_READ_WRITE_H
#define FILE_READ_WRITE_H
#include <QObject>
#include <QVector>
#include <QString>
#include <Eigen/Dense>
#include <QMessageBox>
#include <QDebug>
using namespace Eigen;
class File_Read_Write :public QObject
{
    Q_OBJECT
public:
    File_Read_Write();
    explicit File_Read_Write(QObject *parent = nullptr);

    void loadTxtABtoTriAndSensor(const QString& txtA, const QString& txtB,
                                 QVector<QVector<MatrixXd>>& Points_tri,
                                 QVector<QVector<MatrixXd>>& Points_sensor,
                                 int groupCount) ;
    void combineTriAndSensor(const QVector<QVector<MatrixXd>>& Points_tri,
                             const QVector<QVector<MatrixXd>>& Points_sensor,
                             QVector<QVector<MatrixXd>>& Points_test_all) ;
    void saveParameters(const Matrix3d& M_opt, const VectorXd& theta, const QString& filename) ;
    void loadRotationParameters(const QString& filePath, Matrix3d& M_opt, double& alpha, double& beta);

};

#endif // FILE_READ_WRITE_H
