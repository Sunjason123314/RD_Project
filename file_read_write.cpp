#include "file_read_write.h"
#include <QFile>
#include <fstream>
#include <QDebug>
#include <QString>
#include <Eigen/Dense>
#include <QDataStream>
using namespace Eigen;

File_Read_Write::File_Read_Write(QObject* parent)
    : QObject(parent)
{
}

// 从 A/B.txt 文件分别读取前三列为 Points_tri，最后一列为 Points_sensor，按组组织到内存
void File_Read_Write::loadTxtABtoTriAndSensor(const QString& txtA, const QString& txtB,
                                          QVector<QVector<MatrixXd>>& Points_tri,
                                          QVector<QVector<MatrixXd>>& Points_sensor,
                                          int groupCount)
{
    // 检查 A.txt 是否存在
    if (!QFile::exists(txtA)) {
        QMessageBox::warning(nullptr, tr("file doesn't exist"), tr("can not find A.txt:") + txtA);
        return;
    }
    // 检查 B.txt 是否存在
    if (!QFile::exists(txtB)) {
        QMessageBox::warning(nullptr, tr("file doesn't exist"), tr("can not find B.txt:") + txtB);
        return;
    }
    auto parseTxt = [&](const QString& path) -> std::pair<QVector<MatrixXd>, QVector<MatrixXd>> {
        QFile file(path);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            throw std::runtime_error("无法打开TXT文件: " + path.toStdString());
        }

        QTextStream in(&file);
        QVector<Vector4d> currentGroup;
        QVector<QVector<Vector4d>> allGroups;

        while (!in.atEnd()) {
            QString line = in.readLine().trimmed();
            if (line.isEmpty()) {
                if (!currentGroup.isEmpty()) {
                    allGroups.append(currentGroup);
                    currentGroup.clear();
                }
                continue;
            }

            QStringList tokens = line.split(QRegExp("\\s+"), Qt::SkipEmptyParts);
            if (tokens.size() != 4) {
                throw std::runtime_error("行格式错误: " + line.toStdString());
            }

            Vector4d vec(tokens[0].toDouble(), tokens[1].toDouble(), tokens[2].toDouble(), tokens[3].toDouble());
            currentGroup.append(vec);
        }
        if (!currentGroup.isEmpty()) {
            allGroups.append(currentGroup);
        }
        file.close();

        if (allGroups.size() != groupCount) {
            QMessageBox::warning(nullptr, tr("Data Size Mismatch"),
                                 tr("The number of groups in file %1 is %2, but %3 were expected.")
                                     .arg(path)
                                     .arg(allGroups.size())
                                     .arg(groupCount));
            // 根据需求决定是否继续或中止
            throw std::runtime_error("Group count mismatch: " + path.toStdString());
            // 或者 return 空数据结构
        }



        QVector<MatrixXd> triList, sensorList;
        for (const auto& group : allGroups) {
            int N = group.size();
            MatrixXd tri(3, N);
            MatrixXd sensor(1, N);
            for (int i = 0; i < N; ++i) {
                tri(0, i) = group[i][0];
                tri(1, i) = group[i][1];
                tri(2, i) = group[i][2];
                sensor(0, i) = group[i][3];
            }
            triList.append(tri);
            sensorList.append(sensor);
        }

        return {triList, sensorList};
    };

    // Resize 最终结构
    Points_tri.resize(2);
    Points_sensor.resize(2);

    // A 球
    auto [triA, sensorA] = parseTxt(txtA);
    Points_tri[0] = triA;
    Points_sensor[0] = sensorA;

    // B 球
    auto [triB, sensorB] = parseTxt(txtB);
    Points_tri[1] = triB;
    Points_sensor[1] = sensorB;
}
// 对传感器数据和三坐标测量数据进行整合
void File_Read_Write::combineTriAndSensor(const QVector<QVector<MatrixXd>>& Points_tri,
                                      const QVector<QVector<MatrixXd>>& Points_sensor,
                                      QVector<QVector<MatrixXd>>& Points_test_all)
{
    int dim1 = Points_tri.size();
    int dim2 = Points_tri[0].size();

    Points_test_all.resize(dim1);
    for (int i = 0; i < dim1; ++i) {
        Points_test_all[i].resize(dim2);
        for (int j = 0; j < dim2; ++j) {
            const MatrixXd& tri = Points_tri[i][j];         // 3×N
            const MatrixXd& sensor = Points_sensor[i][j];   // 1×N
            if (tri.cols() != sensor.cols()) {
                throw std::runtime_error("no pair") ;
            }

            MatrixXd result = tri;
            for (int k = 0; k < tri.cols(); ++k) {
                result(1, k) += sensor(0, k);  // 在 y 方向加上传感器值
            }
            Points_test_all[i][j] = result;
        }
    }
}
// 保存 M_opt 和 alpha, beta 到 .bin 文件
void File_Read_Write::saveParameters(const Matrix3d& M_opt, const VectorXd& theta, const QString& filename)
{
    if (theta.size() < 5) {
        qWarning() << "need five paremeters";
        return;
    }

    std::ofstream ofs(filename.toStdString(), std::ios::binary);
    if (!ofs.is_open()) {
        qWarning() << "can not open file:" << filename;
        return;
    }

    // 写入 M_opt（3×3 = 9 个 double）
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            ofs.write(reinterpret_cast<const char*>(&M_opt(i, j)), sizeof(double));

    // 写入 alpha = theta(3), beta = theta(4)
    double alpha = theta(3);
    double beta = theta(4);
    ofs.write(reinterpret_cast<const char*>(&alpha), sizeof(double));
    ofs.write(reinterpret_cast<const char*>(&beta), sizeof(double));

    ofs.close();
}

void File_Read_Write::loadRotationParameters(const QString& filePath, Matrix3d& M_opt, double& alpha, double& beta)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "无法打开文件:" << filePath;
        return;
    }

    QDataStream in(&file);
    in.setByteOrder(QDataStream::LittleEndian);

    // 读取 M_opt（3x3）
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            in >> M_opt(i, j);

    // 读取 alpha、beta
    in >> alpha;
    in >> beta;

    file.close();
    // QMessageBox::information(nullptr, tr("Loaded"), tr("rotation_result.bin loaded. Path: ") + filePath);


}
