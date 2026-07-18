#pragma once

#include <QString>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct SlReconstructParams
{
    QString saveRootPath;
    QString calibFilename;
    QString patternsSubPath;
    QString saveName;

    int rows = 2048;
    int cols = 2448;
    int cutWidth = 5;
    int N = 12;
    double T1 = 24.0;
    double T2 = 26.0;
    double T3 = 28.0;
    int Tsize = 3;
    double Bmin = 10.0;
    double Emin = 1.0;
    double minErr = 0.003;
    bool gauss = false;
    double sigma = 1.0;
    int phaseCycle = 10;
    int phaseSize = 2;
};

struct SlReconstructResult
{
    bool ok = false;
    QString message;
    QString savedPcdPath;
    pcl::PointCloud<pcl::PointXYZ> cloud;
};

SlReconstructResult runSlReconstruction(const SlReconstructParams& params);
