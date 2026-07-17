#ifndef STEPEXPORTER_H
#define STEPEXPORTER_H

#include "dicqglviewer.h"
#include <QString>
#include <functional>

class StepExporter
{
public:
    using ProgressFn = std::function<void(int pct, const char* msg)>;

    static bool exportPointCloudToStep(const PCLCloud::Ptr& cloud,
                                       const QString& filePath,
                                       ProgressFn progress = nullptr);

    static bool exportMeshToStep(const PCLMesh& mesh,
                                 const QString& filePath,
                                 ProgressFn progress = nullptr);
};

#endif // STEPEXPORTER_H
