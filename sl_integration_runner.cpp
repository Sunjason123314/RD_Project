#include "sl_integration_runner.h"

#include <QDir>
#include <QFileInfo>

#include "SL_QT/src/commonfunc.hpp"
#include "SL_QT/src/class/cudaimage.h"
#include "SL_QT/src/class/matcher.h"
#include "SL_QT/src/class/patterncoder.h"
#include "SL_QT/src/class/calibration.h"
#include <cuda_runtime.h>

SlReconstructResult runSlReconstruction(const SlReconstructParams& params)
{
    // 每次重建结束后都重置 CUDA 上下文，避免批量任务中显存/上下文缓存持续膨胀。
    struct CudaCleanupGuard
    {
        ~CudaCleanupGuard()
        {
            cudaDeviceSynchronize();
            cudaGetLastError();
            cudaDeviceReset();
        }
    } cleanupGuard;

    SlReconstructResult result;

    try
    {
        const std::string saveRootPath = QDir::fromNativeSeparators(params.saveRootPath).toStdString();
        const std::string calibFilename = params.calibFilename.toStdString();
        const std::string patternsPath = saveRootPath + "/" + QDir::fromNativeSeparators(params.patternsSubPath).toStdString();
        const std::string calibratePath = saveRootPath + "/calibrate";
        const std::string outPath = saveRootPath + "/SL_out";

        Calibration cb1(cv::Size(8, 11), 20, 20);
        cb1.LoadCalibrateResult(calibratePath + "/" + calibFilename);
        if(!cb1.isCalibrated())
        {
            vector<string> leftCalibFiles;
            vector<string> rightCalibFiles;
            getFiles(calibratePath + "/L", leftCalibFiles);
            getFiles(calibratePath + "/R", rightCalibFiles);

            if(leftCalibFiles.empty() || rightCalibFiles.empty())
            {
                result.message = QStringLiteral("Calibration load failed, and fallback calibration images were not found (calibrate/L or calibrate/R is empty)");
                return result;
            }

            QDir().mkpath(QString::fromStdString(calibratePath + "/Lcb"));
            QDir().mkpath(QString::fromStdString(calibratePath + "/Rcb"));

            cb1.corner_pointslists_L = cb1.read_img(leftCalibFiles, calibratePath + "/Lcb");
            cb1.corner_pointslists_R = cb1.read_img(rightCalibFiles, calibratePath + "/Rcb");
            cb1.calibrate();

            if(!cb1.isCalibrated())
            {
                result.message = QStringLiteral("Calibration load failed, and fallback recalibration also failed");
                return result;
            }
        }

        double T123 = 0.0;
        PatternCoder coder(params.rows, params.cols);
        coder.ParametersPreCompute(params.T1, params.T2, params.T3, params.N, T123);
        Matcher matcher;

        CudaImage<double> phaseLN_cuda(params.cols, params.rows, params.Tsize, -100), phaseRN_cuda(params.cols, params.rows, params.Tsize, -100);
        CudaImage<double> phL_cuda(params.cols, params.rows, 1, -100), phR_cuda(params.cols, params.rows, 1, -100);
        CudaImage<double> ph1223L_cuda(params.cols, params.rows, 2, -100), ph1223R_cuda(params.cols, params.rows, 2, -100);
        CudaImage<double> phL_rectify_cuda(phL_cuda.x_size, phL_cuda.y_size, 1, -100), phR_rectify_cuda(phL_cuda.x_size, phL_cuda.y_size, 1, -100);
        CudaImage<double> phL_B_cuda(phL_cuda.x_size, phL_cuda.y_size, 1, -100), phR_B_cuda(phL_cuda.x_size, phL_cuda.y_size, 1, -100);
        CudaImage<double> disp_cuda(phL_rectify_cuda.x_size, phL_rectify_cuda.y_size, 1, -100);
        CudaImage<double> pointcloud_cuda(disp_cuda.x_size, disp_cuda.y_size, 3);

        double* dataPLHost = nullptr;
        cudaHostAlloc((void**)&dataPLHost, pointcloud_cuda.getbyte(), cudaHostAllocDefault);

        vector<cv::Mat> patternsL, patternsR;
        read_imgs(patternsPath, patternsL, patternsR,
                  params.rows, params.cols, params.cutWidth,
                  0, params.gauss, params.sigma);

        CudaImage<uchar> patternsL_cuda(patternsL);
        CudaImage<uchar> patternsR_cuda(patternsR);

        coder.DeShiftedPhase_cuda(params.N, params.Bmin, patternsL_cuda, phaseLN_cuda, params.Tsize);
        coder.DeShiftedPhase_cuda(params.N, params.Bmin, patternsR_cuda, phaseRN_cuda, params.Tsize);

        for (size_t i = 0; i < params.phaseCycle; i++)
        {
            phaseLN_cuda.phaseFilter(params.phaseSize);
            phaseRN_cuda.phaseFilter(params.phaseSize);
        }

        coder.UnwrappPhase3_cuda(phaseLN_cuda, ph1223L_cuda, phL_cuda);
        coder.UnwrappPhase3_cuda(phaseRN_cuda, ph1223R_cuda, phR_cuda);

        cb1.Rectify_cuda(phL_cuda, phR_cuda, phL_rectify_cuda, phR_rectify_cuda,
                         coder.getblockdim(), coder.getgriddim(), params.rows, params.cols);

        coder.filterB(params.Emin, phL_rectify_cuda, phL_B_cuda);
        coder.filterB(params.Emin, phR_rectify_cuda, phR_B_cuda);
        checkCudaErrors(cudaDeviceSynchronize());

        matcher.match_cuda(phL_B_cuda, phR_B_cuda, disp_cuda, params.minErr,
                           coder.getblockdim(), coder.getgriddim());

        matcher.reprojectTo3D_cuda(disp_cuda, pointcloud_cuda, cb1.Q,
                                   coder.getblockdim(), coder.getgriddim());

        pcl::PointCloud<pcl::PointXYZ> depthCloud;
        pointcloud_cuda.OutPutPointCloud(depthCloud, dataPLHost);
        const std::size_t rawPointCount = depthCloud.size();
        depthCloud = PCL_depth_filter(depthCloud, 0, 1500);
        const std::size_t filteredPointCount = depthCloud.size();

        if (!QDir().mkpath(QString::fromStdString(outPath)))
        {
            cudaFreeHost(dataPLHost);
            result.message = QStringLiteral("Failed to create output directory: ") + QString::fromStdString(outPath);
            return result;
        }

        QString saveName = params.saveName.trimmed();
        if (saveName.isEmpty())
            saveName = QStringLiteral("sl_cloud.pcd");
        if (!saveName.endsWith(QStringLiteral(".pcd"), Qt::CaseInsensitive))
            saveName += QStringLiteral(".pcd");

        const QString pcdPath = QDir(QString::fromStdString(outPath)).filePath(saveName);
        if (pcl::io::savePCDFileBinary(pcdPath.toStdString(), depthCloud) < 0)
        {
            cudaFreeHost(dataPLHost);
            result.message = QStringLiteral("Failed to save PCD: ") + pcdPath;
            return result;
        }

        cudaFreeHost(dataPLHost);

        result.ok = true;
        result.savedPcdPath = QDir::toNativeSeparators(pcdPath);
        result.cloud = depthCloud;
        result.message = QStringLiteral("SL reconstruction completed, points: raw=%1, filtered=%2")
                             .arg(static_cast<qulonglong>(rawPointCount))
                             .arg(static_cast<qulonglong>(filteredPointCount));
        return result;
    }
    catch (const std::exception& e)
    {
        result.message = QStringLiteral("SL reconstruction exception: ") + QString::fromLocal8Bit(e.what());
        return result;
    }
    catch (...)
    {
        result.message = QStringLiteral("SL reconstruction exception: unknown exception");
        return result;
    }
}
