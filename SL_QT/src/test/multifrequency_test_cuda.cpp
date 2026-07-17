#include "commonfunc.hpp"
#include "calibration.h"
#include "patterncoder.h"
#include "matcher.h"

// Multifrequency CUDA reconstruction pipeline test entry.
int main()
{
    Json::Value configall, config;
    readJsonParameters("E:\\Code\\CPP\\SL\\data\\parameters.jsonc", configall);
    config = configall["multifrequency_test"];

    // Load runtime parameters.
    string save_root_path = config["saverootpath"].asString();
    string calib_filename = config["calib_filename"].asString();
    string savename = config["savename"].asString();
    string patterns_path = save_root_path + "/" + config["patterns_subpath"].asString();
    string calibrate_path = save_root_path + "/calibrate";
    string out_path = save_root_path + "/out";

    std::cout << "save_root_path: " << save_root_path << std::endl;
    std::cout << "calib_filename: " << calib_filename << std::endl;
    std::cout << "savename: " << savename << std::endl;
    std::cout << "patterns_path: " << patterns_path << std::endl;
    std::cout << "calibrate_path: " << calibrate_path << std::endl;
    std::cout << "out_path: " << out_path << std::endl;

    int rows = config["rows"].asInt();
    int cols = config["cols"].asInt();
    int cut_width = config["cut_width"].asInt();
    int N = config["N"].asInt();
    double T1 = config["T1"].asDouble();
    double T2 = config["T2"].asDouble();
    double T3 = config["T3"].asDouble();
    int Tsize = config["Tsize"].asInt();
    double B_min = config["B_min"].asDouble();
    double e_min = config["e_min"].asDouble();
    double minerr = config["minerr"].asDouble();
    bool Gauss = config["Gauss"].asBool();
    double sigma = config["sigma"].asDouble();

    // Load stereo calibration.
    Calibration cb1(cv::Size(8, 11), 20, 20);
    cb1.LoadCalibrateResult(calibrate_path + "/" + calib_filename);

    // Initialize core modules and CUDA buffers.
    double T123;
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

    double* data_PLhost;
    cudaHostAlloc((void**)&data_PLhost, pointcloud_cuda.getbyte(), cudaHostAllocDefault);

    // 0) Read and upload pattern images.
    vector<cv::Mat> patternsL, patternsR;
    read_imgs(patterns_path, patternsL, patternsR, rows, cols, cut_width, 0, Gauss, sigma);
    CudaImage<uchar> patternsL_cuda(patternsL);
    CudaImage<uchar> patternsR_cuda(patternsR);

    // 1) Demodulate wrapped phase and apply smoothing.
    coder.DeShiftedPhase_cuda(N, B_min, patternsL_cuda, phaseLN_cuda, Tsize);
    coder.DeShiftedPhase_cuda(N, B_min, patternsR_cuda, phaseRN_cuda, Tsize);
    for (size_t i = 0; i < 10; i++)
    {
        phaseLN_cuda.phaseFilter(2);
        phaseRN_cuda.phaseFilter(2);
    }


    // 2) Multi-frequency unwrapping.
    coder.UnwrappPhase3_cuda(phaseLN_cuda, ph1223L_cuda, phL_cuda);
    coder.UnwrappPhase3_cuda(phaseRN_cuda, ph1223R_cuda, phR_cuda);

    // 3) Stereo rectification.
    cb1.Rectify_cuda(phL_cuda, phR_cuda, phL_rectify_cuda, phR_rectify_cuda, coder.getblockdim(), coder.getgriddim(), rows, cols);

    // 4) Edge/quality masking.
    coder.filterB(e_min, phL_rectify_cuda, phL_B_cuda);
    coder.filterB(e_min, phR_rectify_cuda, phR_B_cuda);
    checkCudaErrors(cudaDeviceSynchronize());


    // 5) Phase matching to disparity.
    matcher.match_cuda(phL_B_cuda, phR_B_cuda, disp_cuda, minerr, coder.getblockdim(), coder.getgriddim());

    // 6) Reproject to 3D point cloud.
    matcher.reprojectTo3D_cuda(disp_cuda, pointcloud_cuda, cb1.Q, coder.getblockdim(), coder.getgriddim());

    pcl::PointCloud<pcl::PointXYZ> depth_cloud;
    pointcloud_cuda.OutPutPointCloud(depth_cloud, data_PLhost);
    depth_cloud = PCL_depth_filter(depth_cloud, 0, 1500);

    showcloud(depth_cloud, "depth_cloud");
    pcl::io::savePCDFile(out_path + "/" + savename, depth_cloud);

    cudaFreeHost(data_PLhost);
    return 0;
}
