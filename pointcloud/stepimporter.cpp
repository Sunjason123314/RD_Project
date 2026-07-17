// ====== STEP 导入：纯 OCCT 实现 ======
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "stepimporter.h"

#include <pcl/conversions.h>

#include <opencascade/BRepMesh_IncrementalMesh.hxx>
#include <opencascade/BRep_Builder.hxx>
#include <opencascade/BRep_Tool.hxx>
#include <opencascade/Poly_Triangulation.hxx>
#include <opencascade/Poly_Triangle.hxx>
#include <opencascade/STEPControl_Reader.hxx>
#include <opencascade/TopExp_Explorer.hxx>
#include <opencascade/TopLoc_Location.hxx>
#include <opencascade/TopoDS.hxx>
#include <opencascade/TopoDS_Compound.hxx>
#include <opencascade/TopoDS_Face.hxx>
#include <opencascade/gp_Pnt.hxx>
#include <opencascade/gp_Trsf.hxx>

#include <QFileInfo>

QString StepImporter::s_lastError;

bool StepImporter::importStep(const QString& filePath, PCLMesh& mesh,
                              ProgressFn progress)
{
    s_lastError.clear();

    QFileInfo fi(filePath);
    if(!fi.exists() || !fi.isFile())
    {
        s_lastError = QStringLiteral("STEP file not found: ") + filePath;
        return false;
    }

    if (progress) progress(10, "Reading STEP file...");

    STEPControl_Reader reader;
    const std::string pathStd = filePath.toLocal8Bit().constData();
    IFSelect_ReturnStatus status = reader.ReadFile(pathStd.c_str());
    if(status != IFSelect_RetDone)
    {
        s_lastError = QStringLiteral("Failed to read STEP file.");
        return false;
    }

    if (progress) progress(30, "Transferring shapes...");

    if(reader.TransferRoots() < 1)
    {
        s_lastError = QStringLiteral("STEP file contains no transferable shape.");
        return false;
    }

    TopoDS_Shape shape;
    if(reader.NbShapes() == 1)
    {
        shape = reader.OneShape();
    }
    else
    {
        BRep_Builder cb;
        TopoDS_Compound compound;
        cb.MakeCompound(compound);
        for(int i = 1; i <= reader.NbShapes(); ++i)
            cb.Add(compound, reader.Shape(i));
        shape = compound;
    }

    if (progress) progress(50, "Meshing faces...");

    BRepMesh_IncrementalMesh mesher(shape, 1.0, Standard_False, 0.5);
    mesher.Perform();

    if (progress) progress(70, "Extracting triangles...");

    pcl::PointCloud<pcl::PointXYZ> vertices;
    std::vector<pcl::Vertices> triangles;
    int globalIdx = 0;

    for(TopExp_Explorer fe(shape, TopAbs_FACE); fe.More(); fe.Next())
    {
        TopoDS_Face face = TopoDS::Face(fe.Current());
        TopLoc_Location loc;
        const Handle(Poly_Triangulation)& tri = BRep_Tool::Triangulation(face, loc);
        if(tri.IsNull())
            continue;

        const gp_Trsf& trsf = loc.Transformation();
        const int nNodes = tri->NbNodes();
        const int nTris = tri->NbTriangles();
        const int base = globalIdx;

        for(int j = 1; j <= nNodes; ++j)
        {
            gp_Pnt p = tri->Node(j);
            p.Transform(trsf);
            vertices.emplace_back(static_cast<double>(p.X()),
                                  static_cast<double>(p.Y()),
                                  static_cast<double>(p.Z()));
            ++globalIdx;
        }

        for(int j = 1; j <= nTris; ++j)
        {
            const Poly_Triangle& t = tri->Triangle(j);
            int n1, n2, n3;
            t.Get(n1, n2, n3);
            pcl::Vertices v;
            v.vertices = {base + n1 - 1, base + n2 - 1, base + n3 - 1};
            triangles.push_back(v);
        }
    }

    if(triangles.empty())
    {
        s_lastError = QStringLiteral("STEP file contains no triangulatable faces.");
        return false;
    }

    mesh.polygons = triangles;
    pcl::toPCLPointCloud2(vertices, mesh.cloud);

    if (progress) progress(100, "STEP import complete.");
    return true;
}
