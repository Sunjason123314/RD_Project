#include "calibration.h"

Calibration::Calibration(cv::Size pattern_size_in, double cb_length_in, double cb_high_in)
{
    //?????????????????
    pattern_size = pattern_size_in;
    cb_length = cb_length_in;
    cb_high = cb_high_in;
}
Calibration::~Calibration()
{

}

//0????????
//img_path???????????????????
//img_path_cb????????????????????
int FDcount = 0;
void FD(cv::Mat* img, vector<cv::Point2f>* corner_points_buf, cv::Size pattern_size)//????????????
{
    FDcount++;
    int index=FDcount;
    cout << "???" + to_string(index) << endl;
    if (!cv::findChessboardCorners(*img, pattern_size, *corner_points_buf))
    {
        cout << "??????????????" << index << "??" << endl;
    }
    else
    {
        cv::cornerSubPix(*img, *corner_points_buf, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.1));
        cv::drawChessboardCorners(*img, pattern_size, *corner_points_buf, true);
    }
}
vector<vector<cv::Point2f>> Calibration::read_img(vector<string> FileNames, string img_path_cb)
{
    //??????
    img_number = FileNames.size();
    if (img_number==0)
    {
        cout << "[calibration]:???????0" << endl;
    }

    vector<cv::Mat> Imgs;
    for (size_t i = 0; i < img_number; i++)
    {
        Imgs.push_back(cv::imread(FileNames[i], cv::IMREAD_GRAYSCALE));//??????????????????????????????
    }
    img_size.width = Imgs[0].cols;
    img_size.height = Imgs[0].rows;//////////????????????

    //?????
    vector<vector<cv::Point2f>> corner_pointslists;//??????????????????????
    vector<cv::Point2f> corner_points_buf;//?????????v???????????????Point2f???,?????????????????????????????????????
    for (size_t i = 0; i < img_number; i++)
    {
        corner_pointslists.push_back(corner_points_buf);
    }

    boost::thread_group group;
    for (size_t i = 0; i < img_number; i++)
    {
        group.create_thread(boost::bind(&FD, &Imgs[i], &corner_pointslists[i], std::ref(pattern_size)));
    }
    group.join_all();

    //????????????
    for (size_t i = 0; i < img_number; i++)
    {
        cv::imwrite(img_path_cb + "\\cb" + to_string(i)+".png", Imgs[i]);
    }

    return corner_pointslists;
}

//??
void Calibration::calibrate()
{
    if (calib)
    {
        cout << "[calibration]:?????????????????" << endl;
        return;
    }

    //????????????????????????
    vector<cv::Point3f> tempCornerPoints;//?????????????????
    for (int i = 0; i < pattern_size.height; i++)
    {
        for (int j = 0; j < pattern_size.width; j++)
        {
            cv::Point3f singleRealPoint;//???????????????????????
            singleRealPoint.x = i * cb_length;//??/????????????????????????????????????x,y,z)
            singleRealPoint.y = j * cb_high;
            singleRealPoint.z = 0;//????z=0
            tempCornerPoints.push_back(singleRealPoint);
        }
    }
    for (int k = 0; k < img_number; k++)//??????????
    {
        objectPoints.push_back(tempCornerPoints);
    }

    cv::calibrateCamera(objectPoints, corner_pointslists_L, img_size, cameraMatrix_L, distCoefficients_L, rvecsMat_L, tvecsMat_L, 0);
    cout << "[calibration]:?????????:" << endl;
    cout << "[calibration]:??????????:\n" << cameraMatrix_L << endl;
    cout << "[calibration]:??????????:\n" << distCoefficients_L << endl;

    cv::calibrateCamera(objectPoints, corner_pointslists_R, img_size, cameraMatrix_R, distCoefficients_R, rvecsMat_R, tvecsMat_R, 0);
    cout << "[calibration]:?????????:" << endl;
    cout << "[calibration]:??????????:\n" << cameraMatrix_R << endl;
    cout << "[calibration]:??????????:\n" << distCoefficients_R << endl;

    rms = cv::stereoCalibrate(objectPoints,
        corner_pointslists_L, corner_pointslists_R,
        cameraMatrix_L, distCoefficients_L,
        cameraMatrix_R, distCoefficients_R,
        img_size, R, T, E, F, cv::CALIB_USE_INTRINSIC_GUESS, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-6));

    cout << "[calibration]:?????????:" << endl;
    cout << "[calibration]:R=\n" << R << endl;
    cout << "[calibration]:T=\n" << T << endl;
    cout << "[calibration]:E=\n" << E << endl;
    cout << "[calibration]:F=\n" << F << endl;
    cout << "[calibration]:????rms=\n" << rms << endl;
    
    cv::Rect validROIL, validROIR;//?????????????????????????????validROI????????????
    cv::stereoRectify(cameraMatrix_L, distCoefficients_L,
        cameraMatrix_R, distCoefficients_R, img_size,
        R, T, Rl, Rr, Pl, Pr, Q,
        cv::CALIB_ZERO_DISPARITY, 1, img_size, &validROIL, &validROIR);

    cout << "[calibration]:Rl=\n" << Rl << endl;
    cout << "[calibration]:Rr=\n" << Rr << endl;
    cout << "[calibration]:Pl=\n" << Pl << endl;
    cout << "[calibration]:Pr=\n" << Pr << endl;
    cout << "[calibration]:img_size=\n" << img_size << endl;

    //?????????????????
    initUndistortRectifyMap(cameraMatrix_L, distCoefficients_L, Rl, Pl, img_size, CV_16SC2, mapLx, mapLy);
    //cout << mapLx << mapLy << endl;
    initUndistortRectifyMap(cameraMatrix_R, distCoefficients_R, Rr, Pr, img_size, CV_16SC2, mapRx, mapRy);
    //cout << mapRx << mapRy << endl;

    calib = true;
}


void Calibration::LoadCalibrateResult(const string &calib_file,bool refraction)
{
#ifdef SL_ENABLE_MATLAB
    cout << "[calibration]:????????" << endl;
    MATFile *pmatFile = matOpen(calib_file.c_str(),"r");
    if (pmatFile == NULL)
    {
        std::cerr << "[calibration]:??????????" << calib_file << std::endl;
        return;
    }
    cout << "[calibration]:???????????" << endl;

    mxArray* CameraParameters1;
    mxArray* CameraParameters2;
    CameraParameters1 = matGetVariable(pmatFile, "CameraParameters1");
    CameraParameters2 = matGetVariable(pmatFile, "CameraParameters2");

    mxArray* ImageSize=mxGetField(CameraParameters1, 0,"ImageSize");
    double* ImageSizedata=(double*)mxGetData(ImageSize);
    img_size.width=ImageSizedata[1];
    img_size.height=ImageSizedata[0];
    cout<<"[calibration]:img_size:\n"<<img_size<<endl;

    mxArray* IntrinsicMatrix_1 = mxGetField(CameraParameters1, 0, "IntrinsicMatrix");
    ToCVMat(IntrinsicMatrix_1,cameraMatrix_L);
    cv::transpose(cameraMatrix_L, cameraMatrix_L);
    cout<<"[calibration]:IntrinsicMatrix_1:\n"<<cameraMatrix_L<<endl;
    cameraMatrix_L_cuda=CudaImage<double>::CudaImage(cameraMatrix_L);

    mxArray* IntrinsicMatrix_2 = mxGetField(CameraParameters2, 0, "IntrinsicMatrix");
    ToCVMat(IntrinsicMatrix_2,cameraMatrix_R);
    cv::transpose(cameraMatrix_R, cameraMatrix_R);
    cout<<"[calibration]:IntrinsicMatrix_2:\n"<<cameraMatrix_R<<endl;
    cameraMatrix_R_cuda=CudaImage<double>::CudaImage(cameraMatrix_R);

    mxArray* RadialDistortion_1 = mxGetField(CameraParameters1, 0, "RadialDistortion");
    cv::Mat RadialDistortion_1_cv;
    ToCVMat(RadialDistortion_1,RadialDistortion_1_cv);
    cout<<"[calibration]:RadialDistortion_1:\n"<<RadialDistortion_1_cv<<endl;

    mxArray* RadialDistortion_2 = mxGetField(CameraParameters2, 0, "RadialDistortion");
    cv::Mat RadialDistortion_2_cv;
    ToCVMat(RadialDistortion_2,RadialDistortion_2_cv);
    cout<<"[calibration]:RadialDistortion_2:\n"<<RadialDistortion_2_cv<<endl;

    mxArray* TangentialDistortion_1 = mxGetField(CameraParameters1, 0, "TangentialDistortion");
    cv::Mat TangentialDistortion_1_cv;
    ToCVMat(TangentialDistortion_1,TangentialDistortion_1_cv);
    cout<<"[calibration]:TangentialDistortion_1:\n"<<TangentialDistortion_1_cv<<endl;

    mxArray* TangentialDistortion_2 = mxGetField(CameraParameters2, 0, "TangentialDistortion");
    cv::Mat TangentialDistortion_2_cv;
    ToCVMat(TangentialDistortion_2,TangentialDistortion_2_cv);
    cout<<"[calibration]:TangentialDistortion_2:\n"<<TangentialDistortion_2_cv<<endl;

    distCoefficients_L=(cv::Mat_<double>(5, 1)<<
        RadialDistortion_1_cv.at<double>(0),
        RadialDistortion_1_cv.at<double>(1),
        TangentialDistortion_1_cv.at<double>(0),
        TangentialDistortion_1_cv.at<double>(1),
        0.
    );
    distCoefficients_R=(cv::Mat_<double>(5, 1)<<
        RadialDistortion_2_cv.at<double>(0),
        RadialDistortion_2_cv.at<double>(1),
        TangentialDistortion_2_cv.at<double>(0),
        TangentialDistortion_2_cv.at<double>(1),
        0.
    );

    mxArray* RotationOfCamera2=matGetVariable(pmatFile,"RotationOfCamera2");
    ToCVMat(RotationOfCamera2,R);
    cv::Mat R__=R.clone();
    cv::transpose(R, R);
    cout<<"[calibration]:RotationOfCamera2:\n"<<R<<endl;
    R_cuda=CudaImage<double>::CudaImage(R);

    mxArray* TranslationOfCamera2=matGetVariable(pmatFile,"TranslationOfCamera2");
    ToCVMat(TranslationOfCamera2,T);
    cv::transpose(T, T);
    cout<<"[calibration]:TranslationOfCamera2:\n"<<T<<endl;
    T_cuda=CudaImage<double>::CudaImage(-R__*T);

    mxArray* EssentialMatrix=matGetVariable(pmatFile,"EssentialMatrix");
    ToCVMat(EssentialMatrix,E);
    cv::transpose(E, E);
    cout<<"[calibration]:EssentialMatrix:\n"<<E<<endl;

    mxArray* FundamentalMatrix=matGetVariable(pmatFile,"FundamentalMatrix");
    ToCVMat(FundamentalMatrix,F);
    cv::transpose(F, F);
    cout<<"[calibration]:FundamentalMatrix:\n"<<F<<endl;

    cv::Rect validROIL, validROIR;
    cv::stereoRectify(cameraMatrix_L, distCoefficients_L,
        cameraMatrix_R, distCoefficients_R, img_size,
        R, T, Rl, Rr, Pl, Pr, Q,
        cv::CALIB_ZERO_DISPARITY, 1, img_size, &validROIL, &validROIR);

    cout << "[calibration]:Rl=\n" << Rl << endl;
    cout << "[calibration]:Rr=\n" << Rr << endl;
    cout << "[calibration]:Pl=\n" << Pl << endl;
    cout << "[calibration]:Pr=\n" << Pr << endl;
    cout << "[calibration]:Q=\n" << Q << endl;

    initUndistortRectifyMap(cameraMatrix_L, distCoefficients_L, Rl, Pl, img_size, CV_32FC1, mapLx, mapLy);
    initUndistortRectifyMap(cameraMatrix_R, distCoefficients_R, Rr, Pr, img_size, CV_32FC1, mapRx, mapRy);
    mapLx_cuda=CudaImage<float>::CudaImage(mapLx);
    mapLy_cuda=CudaImage<float>::CudaImage(mapLy);
    mapRx_cuda=CudaImage<float>::CudaImage(mapRx);
    mapRy_cuda=CudaImage<float>::CudaImage(mapRy);

    cv::initUndistortRectifyMap(cameraMatrix_L, distCoefficients_L, cv::Mat(), cameraMatrix_L, img_size, CV_32FC1, mapLx_distortion, mapLy_distortion);
    cv::initUndistortRectifyMap(cameraMatrix_R, distCoefficients_R, cv::Mat(), cameraMatrix_R, img_size, CV_32FC1, mapRx_distortion, mapRy_distortion);
    mapLx_distortion_cuda=CudaImage<float>::CudaImage(mapLx_distortion);
    mapLy_distortion_cuda=CudaImage<float>::CudaImage(mapLy_distortion);
    mapRx_distortion_cuda=CudaImage<float>::CudaImage(mapRx_distortion);
    mapRy_distortion_cuda=CudaImage<float>::CudaImage(mapRy_distortion);

    Eigen::MatrixXd IL(3, 3), IR(3, 3);
    Eigen::MatrixXd r(3, 3), t(3, 1);
    cv::cv2eigen(cameraMatrix_L, IL);
    cv::cv2eigen(cameraMatrix_R, IR);
    cv::cv2eigen(R, r);
    cv::cv2eigen(T, t);

    Eigen::MatrixXd R_T(3, 4), ER_T(3, 4);
    Eigen::MatrixXd paral(3, 4), parar(3, 4);
    ER_T << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0;
    R_T << r, t;
    paral = IL * ER_T;
    parar = IR * R_T;
    Pl_2.create(3,4,CV_64FC1);
    Pr_2.create(3,4,CV_64FC1);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            Pl_2.at<double>(i,j) = paral(i, j);
            Pr_2.at<double>(i,j) = parar(i, j);
        }
    }

    cout << "[calibration]:Pl_2=\n" << Pl_2 << endl;
    cout << "[calibration]:Pr_2=\n" << Pr_2 << endl;
    PL2_cuda=CudaImage<double>::CudaImage(Pl_2);
    PR2_cuda=CudaImage<double>::CudaImage(Pr_2);

    calib = true;

    if(refraction)
    {
        mxArray* refractionParams;
        refractionParams = matGetVariable(pmatFile, "refractionParams");
        if(refractionParams==NULL)
        {
            std::cerr<<"[calibration]:????????????????????! "<<endl;
            return;
        }

        mxArray* axis_L_mx = mxGetField(refractionParams, 0, "axis_L");
        double* axis_L_data=(double*)mxGetData(axis_L_mx);
        mxArray* axis_R_mx = mxGetField(refractionParams, 0, "axis_R");
        double* axis_R_data=(double*)mxGetData(axis_R_mx);
        norm_L[0]=axis_L_data[0];
        norm_L[1]=axis_L_data[1];
        norm_L[2]=axis_L_data[2];
        norm_R[0]=axis_R_data[0];
        norm_R[1]=axis_R_data[1];
        norm_R[2]=axis_R_data[2];
        cout << "[calibration]:norm=["<<norm_L[0]<<","<<norm_L[1]<<","<<norm_L[2]<<"],["<<norm_R[0]<<","<<norm_R[1]<<","<<norm_R[2]<<"]"<<endl;

        int size = sizeof(norm_L);
        cudaMalloc((void**)&norm_L_cuda, size);
        cudaMemcpy(norm_L_cuda, norm_L, size, cudaMemcpyHostToDevice);
        cudaMalloc((void**)&norm_R_cuda, size);
        cudaMemcpy(norm_R_cuda, norm_R, size, cudaMemcpyHostToDevice);

        mxArray* thickness_mx = mxGetField(refractionParams, 0, "thickness");
        double* thicknessdata=(double*)mxGetData(thickness_mx);
        thickness=thicknessdata[0];
        cout << "[calibration]:thickness="<<thickness<<endl;

        mxArray* n_mx = mxGetField(refractionParams, 0, "n");
        double* ndata=(double*)mxGetData(n_mx);
        refractive_index=ndata[0];
        cout << "[calibration]:refractive_index="<<refractive_index<<endl;

        refraction_calib=true;
    }
    else
    {
        norm_L[0]=0;
        norm_L[1]=0;
        norm_L[2]=1;
        norm_R[0]=0;
        norm_R[1]=0;
        norm_R[2]=1;

        int size = sizeof(norm_L);
        cudaMalloc((void**)&norm_L_cuda, size);
        cudaMemcpy(norm_L_cuda, norm_L, size, cudaMemcpyHostToDevice);
        cudaMalloc((void**)&norm_R_cuda, size);
        cudaMemcpy(norm_R_cuda, norm_R, size, cudaMemcpyHostToDevice);

        thickness=0;
        refractive_index=1;

        refraction_calib=false;
    }
#else
    (void)calib_file;
    (void)refraction;
    std::cerr << "[calibration]: 当前构建未启用 MATLAB(.mat) 读取，无法加载标定文件" << std::endl;
    calib = false;
    return;
#endif
}

//3??????????
void Calibration::evaluate(const vector<vector<cv::Point2f>>& corner_pointslists, const vector<cv::Mat>& rvecsMat, const vector<cv::Mat>& tvecsMat, const cv::Mat& cameraMatrix, const cv::Mat& distCoefficients)
{
    if (!calib)
    {
        cout << "[calibration]:?????????????????????????" << endl;
        return;
    }
    //??????????????
    cout << "[calibration]:???????????......" << endl;

    //?????????????????????????????????????
    int corner_points_counts;
    corner_points_counts = pattern_size.width * pattern_size.height;

    cout << "[calibration]:???????????" << endl;
    double err = 0;//???????????
    double total_err = 0;//??????????????
    for (int i = 0; i < img_number; i++)
    {
        vector<cv::Point2f> image_points_calculated;//????????????????????
        cv::projectPoints(objectPoints[i], rvecsMat[i], tvecsMat[i], cameraMatrix, distCoefficients, image_points_calculated);  //??????????????????????????????????????image_points_calculated
        //??????????????????????????
        vector<cv::Point2f> image_points_old = corner_pointslists[i];    //????
        //?????????????Mat???
        cv::Mat image_points_calculated_mat = cv::Mat(1, image_points_calculated.size(), CV_32FC2);   //??mat???????1???????
        cv::Mat image_points_old_mat = cv::Mat(1, image_points_old.size(), CV_32FC2);
        for (int j = 0; j < objectPoints[i].size(); j++)
        {
            image_points_calculated_mat.at<cv::Vec2f>(0, j) = cv::Vec2f(image_points_calculated[j].x, image_points_calculated[j].y);  //vec2f->????????????
            image_points_old_mat.at<cv::Vec2f>(0, j) = cv::Vec2f(image_points_old[j].x, image_points_old[j].y);   //??????????????????????
        }
        err = cv::norm(image_points_calculated_mat, image_points_old_mat, cv::NORM_L2); //??????????
        err /= corner_points_counts;      //??????????
        total_err += err;
        cout << "[calibration]:??" << i + 1 << "????????????" << err << "????" << endl;
    }
    cout << "[calibration]:??????????" << total_err / img_number << "????" << endl;
    cout << "[calibration]:???????" << endl;
}

void Calibration::evaluateLR()
{
    evaluate(corner_pointslists_L, rvecsMat_L, tvecsMat_L, cameraMatrix_L, distCoefficients_L);
    evaluate(corner_pointslists_R, rvecsMat_R, tvecsMat_R, cameraMatrix_R, distCoefficients_R);
}

/// @brief matlab??????opencv????
/// @param labmat 
/// @param cvmat 
#ifdef SL_ENABLE_MATLAB
void Calibration::ToCVMat(mxArray* labmat,cv::Mat& cvmat)
{
    int N=mxGetN(labmat);
    int M=mxGetM(labmat);
    double* labmatdata=(double*)mxGetData(labmat);

    cvmat.create(M,N,CV_64FC1);
    for (int i = 0; i < M; i++)
    {
        for (int j = 0; j < N; j++)
        {
            cvmat.at<double>(i,j)=labmatdata[i+j*M];
        }
    }
}
#endif

/// @brief ????-???????
/// @param src_L 
/// @param src_R 
/// @param rectifyImageL 
/// @param rectifyImageR 
void Calibration::Rectify(cv::Mat& src_L, cv::Mat& src_R,cv::Mat& rectifyImageL, cv::Mat& rectifyImageR)
{
    if (!calib)
    {
        cout << "[calibration]:?????????????????????????" << endl;
        return;
    }
    // cv::undistort(src_L, rectifyImageL, cameraMatrix_L, distCoefficients_L);
    // cv::undistort(src_R, rectifyImageR, cameraMatrix_R, distCoefficients_R);

    // cv::remap(rectifyImageL, rectifyImageL, mapLx, mapLy, cv::INTER_LINEAR);
    // cv::remap(rectifyImageR, rectifyImageR, mapRx, mapRy, cv::INTER_LINEAR);
    
    //??????remap???????????????????????????????????
    cv::remap(src_L, rectifyImageL, mapLx, mapLy, cv::INTER_LINEAR);
    cv::remap(src_R, rectifyImageR, mapRx, mapRy, cv::INTER_LINEAR);
}