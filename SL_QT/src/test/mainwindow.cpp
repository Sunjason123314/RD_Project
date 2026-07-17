#include "mainwindow.h"

#include "ui_mainwindow.h"

#include <QMessageBox>
#include <QDir>

#include "commonfunc.hpp"
#include "calibration.h"
#include "matcher.h"
#include "patterncoder.h"

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->runButton->setAutoDefault(false);
    ui->runButton->setDefault(false);
    connect(ui->runButton, &QPushButton::clicked, this, &MainWindow::runPipeline);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::appendLog(const QString& msg)
{
    ui->logEdit->append(msg);
}

void MainWindow::runPipeline()
{
    if (sender() != ui->runButton)
    {
        appendLog("Ignore non-button trigger.");
        return;
    }

    try
    {
        appendLog("===== Start =====");
        const std::string saveRootPath = ui->saveRootPathEdit->text().toStdString();
        const std::string calibFilename = ui->calibFilenameEdit->text().toStdString();
        const std::string saveName = ui->saveNameEdit->text().toStdString();
        const std::string patternsPath = saveRootPath + "/" + ui->patternsSubPathEdit->text().toStdString();
        const std::string calibratePath = saveRootPath + "/calibrate";
        const std::string outPath = saveRootPath + "/" + ui->outSubPathEdit->text().toStdString();

        appendLog(QString::fromStdString("save_root_path: " + saveRootPath));
        appendLog(QString::fromStdString("patterns_path: " + patternsPath));
        appendLog(QString::fromStdString("calibrate_path: " + calibratePath));
        appendLog(QString::fromStdString("out_path: " + outPath));

        const int rows = ui->rowsSpin->value();
        const int cols = ui->colsSpin->value();
        const int cutWidth = ui->cutWidthSpin->value();
        const int N = ui->nSpin->value();
        const double T1 = ui->t1Spin->value();
        const double T2 = ui->t2Spin->value();
        const double T3 = ui->t3Spin->value();
        const int Tsize = ui->tSizeSpin->value();
        const double B_min = ui->bMinSpin->value();
        const double e_min = ui->eMinSpin->value();
        const double minerr = ui->minErrSpin->value();
        const bool gauss = ui->gaussCheck->isChecked();
        const double sigma = ui->sigmaSpin->value();

        Calibration cb1(cv::Size(8, 11), 20, 20);
        cb1.LoadCalibrateResult(calibratePath + "/" + calibFilename);
        appendLog("Calibration loaded.");

        double T123 = 0.0;
        PatternCoder coder(rows, cols);
        coder.ParametersPreCompute(T1, T2, T3, N, T123);
        Matcher matcher;

        CudaImage<double> phaseLN_cuda(cols, rows, Tsize, -100), phaseRN_cuda(cols, rows, Tsize, -100);
        CudaImage<double> phL_cuda(cols, rows, 1, -100), phR_cuda(cols, rows, 1, -100);
        CudaImage<double> ph1223L_cuda(cols, rows, 2, -100), ph1223R_cuda(cols, rows, 2, -100);
        CudaImage<double> phL_rectify_cuda(phL_cuda.x_size, phL_cuda.y_size, 1, -100), phR_rectify_cuda(phL_cuda.x_size, phL_cuda.y_size, 1, -100);
        CudaImage<double> phL_B_cuda(phL_cuda.x_size, phL_cuda.y_size, 1, -100), phR_B_cuda(phL_cuda.x_size, phL_cuda.y_size, 1, -100);
        CudaImage<double> disp_cuda(phL_rectify_cuda.x_size, phL_rectify_cuda.y_size, 1, -100);
        CudaImage<double> pointcloud_cuda(disp_cuda.x_size, disp_cuda.y_size, 3);

        double* data_PLhost = nullptr;
        cudaHostAlloc((void**)&data_PLhost, pointcloud_cuda.getbyte(), cudaHostAllocDefault);

        vector<cv::Mat> patternsL, patternsR;
        read_imgs(patternsPath, patternsL, patternsR, rows, cols, cutWidth, 0, gauss, sigma);
        CudaImage<uchar> patternsL_cuda(patternsL);
        CudaImage<uchar> patternsR_cuda(patternsR);
        appendLog("Images loaded.");

        coder.DeShiftedPhase_cuda(N, B_min, patternsL_cuda, phaseLN_cuda, Tsize);
        coder.DeShiftedPhase_cuda(N, B_min, patternsR_cuda, phaseRN_cuda, Tsize);
        for (size_t i = 0; i < 10; i++)
        {
            phaseLN_cuda.phaseFilter(2);
            phaseRN_cuda.phaseFilter(2);
        }

        coder.UnwrappPhase3_cuda(phaseLN_cuda, ph1223L_cuda, phL_cuda);
        coder.UnwrappPhase3_cuda(phaseRN_cuda, ph1223R_cuda, phR_cuda);

        cb1.Rectify_cuda(phL_cuda, phR_cuda, phL_rectify_cuda, phR_rectify_cuda,
                         coder.getblockdim(), coder.getgriddim(), rows, cols);

        coder.filterB(e_min, phL_rectify_cuda, phL_B_cuda);
        coder.filterB(e_min, phR_rectify_cuda, phR_B_cuda);
        checkCudaErrors(cudaDeviceSynchronize());

        matcher.match_cuda(phL_B_cuda, phR_B_cuda, disp_cuda, minerr,
                           coder.getblockdim(), coder.getgriddim());

        matcher.reprojectTo3D_cuda(disp_cuda, pointcloud_cuda, cb1.Q,
                                   coder.getblockdim(), coder.getgriddim());

        pcl::PointCloud<pcl::PointXYZ> depth_cloud;
        pointcloud_cuda.OutPutPointCloud(depth_cloud, data_PLhost);
        depth_cloud = PCL_depth_filter(depth_cloud, 0, 1500);

        if (!QDir().mkpath(QString::fromStdString(outPath)))
        {
            cudaFreeHost(data_PLhost);
            throw std::runtime_error("Failed to create output directory: " + outPath);
        }

        pcl::io::savePCDFile(outPath + "/" + saveName, depth_cloud);
        cudaFreeHost(data_PLhost);

        appendLog("PCD saved successfully.");
        appendLog("===== Done =====");
        QMessageBox::information(this, "完成", "Points Clouds has been saved");
    }
    catch (const std::exception& e)
    {
        appendLog(QString("Error: ") + e.what());
        QMessageBox::critical(this, "错误", QString("运行失败: ") + e.what());
    }
    catch (...)
    {
        appendLog("Unknown error.");
        QMessageBox::critical(this, "错误", "运行失败: 未知异常");
    }
}
