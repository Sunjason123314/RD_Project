#ifndef DRAWALGORITHM_H
#define DRAWALGORITHM_H

#include "dicqglviewer.h"
#include "Point_Algorithm.h"

#include <QString>
#include <vector>

namespace DrawAlgorithm {

struct GeometryItem {
    int geometryType; // 0 mesh, 1 axis line
    QString name;
    QString displayTag;
    PCLMesh mesh;
    PCLCloud::Ptr axisLine;

    GeometryItem() : geometryType(0), axisLine(new PCLCloud) {}
};

struct RunResult {
    bool success;
    QString fitKey;
    QString logText;
    QString popupText;
    std::vector<GeometryItem> geometries;

    RunResult() : success(false) {}
};

// Mesh generation functions
PCLMesh buildSphereMesh(const Eigen::Vector3f& center, float radius, int latSteps, int lonSteps);

PCLMesh buildCylinderMesh(const Eigen::Vector3f& axisPoint,
                          Eigen::Vector3f axisDir,
                          float radius,
                          const PCLCloud::Ptr& refCloud,
                          int hSteps,
                          int radialSteps);

PCLMesh buildPlaneMesh(const Eigen::Vector4f& coeff,
                       const PCLCloud::Ptr& refCloud,
                       int gridN);

PCLMesh buildCircleDiskMesh(const Eigen::Vector3f& center,
                            Eigen::Vector3f normal,
                            float radius,
                            int radialSteps,
                            int ringSteps);

PCLCloud::Ptr buildAxisLine(const Eigen::Vector3f& center,
                            Eigen::Vector3f direction,
                            const PCLCloud::Ptr& refCloud,
                            int samples);

// Fitting and reconstruction functions
RunResult reconSphereInliers(const PCLCloud::Ptr& input,
                             const QString& baseName,
                             double distanceThreshold);

RunResult fitByKey(const QString& fitKey,
                   const PCLCloud::Ptr& input,
                   const QString& baseName);

}

#endif
