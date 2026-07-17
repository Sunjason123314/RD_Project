// ====== STEP 导出 ======
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "stepexporter.h"
#include "stepwriter_occt.h"
#include "Point_Algorithm.h"

#include <pcl/io/vtk_lib_io.h>

#include <QFileInfo>
#include <cstdio>
#include <iostream>
#include <string>

// ---------------------------------------------------------------------------
// PCLMesh -> STEP (via STL temp + OCCT shape fix / face unify)
// ---------------------------------------------------------------------------
static bool writeMeshToStep(const PCLMesh& meshIn, const QString& stepPath,
                            StepExporter::ProgressFn progress = nullptr)
{
    if (meshIn.polygons.empty())
        return false;

    if (progress) progress(10, "Decimating mesh...");

    // 精简到 ≤3000 面，减小 STEP 文件体积并避免 OCCT OOM
    PCLMesh mesh = meshIn;
    Point_Algorithm::decimateMesh(mesh, 3000);

    if (progress) progress(40, "Writing temporary STL...");

    const QByteArray stepBytes = stepPath.toLocal8Bit();
    const std::string stepPathStd(stepBytes.constData(), stepBytes.size());
    const std::string tmpStl = stepPathStd + ".tmp.stl";

    if (pcl::io::savePolygonFileSTL(tmpStl, mesh) < 0) {
        std::cerr << "Failed to write temp STL: " << tmpStl << std::endl;
        return false;
    }

    if (progress) progress(60, "Converting STL to STEP...");

    int ret = convertStlToStep(tmpStl.c_str(), stepPathStd.c_str());

    // Always clean up temp STL
    if (std::remove(tmpStl.c_str()) != 0)
        std::cerr << "Warning: failed to remove temp file: " << tmpStl << std::endl;

    if (progress) progress(100, "STEP export complete.");
    return ret != 0;
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------
bool StepExporter::exportPointCloudToStep(const PCLCloud::Ptr& cloud,
                                          const QString& filePath,
                                          ProgressFn progress)
{
    if (!cloud || cloud->size() < 10)
        return false;
    if (progress) progress(0, "Reconstructing NURBS surface...");

    // NURBS 重建：floor(sqrt(3000/2))+1=39 → ~2888面，顶点在光滑曲面上
    PCLMesh mesh = Point_Algorithm::nurbsReconstruction(cloud, 3, 5, 39);

    if (mesh.polygons.empty()) {
        // NURBS 失败时回退 Poisson
        if (progress) progress(0, "NURBS failed, fallback to Poisson...");
        mesh = Point_Algorithm::reconstructMesh(
            cloud, Point_Algorithm::ReconMethod::Poisson,
            0.003f, 70.0f, 3, 5, 64, 8, 5, 5);
    }

    if (!mesh.polygons.empty()) {
        // 修复退化面片后再精简
        Point_Algorithm::repairMesh(mesh);
        Point_Algorithm::decimateMesh(mesh, 3000);
    }

    if (mesh.polygons.empty())
        return false;
    return writeMeshToStep(mesh, filePath, progress);
}

bool StepExporter::exportMeshToStep(const PCLMesh& mesh,
                                    const QString& filePath,
                                    ProgressFn progress)
{
    if (mesh.polygons.empty())
        return false;
    return writeMeshToStep(mesh, filePath, progress);
}
