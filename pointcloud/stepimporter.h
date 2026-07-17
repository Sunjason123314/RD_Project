#ifndef STEPIMPORTER_H
#define STEPIMPORTER_H

#include "dicqglviewer.h"
#include <QString>
#include <functional>

class StepImporter
{
public:
    using ProgressFn = std::function<void(int pct, const char* msg)>;

    static bool importStep(const QString& filePath, PCLMesh& mesh,
                           ProgressFn progress = nullptr);
    static QString lastError() { return s_lastError; }

private:
    static QString s_lastError;
};

#endif // STEPIMPORTER_H
