// ====== 纯 OCCT：无 PCL/Qt 依赖 ======
// STL → STEP 完整流程：读取 → 形状修复 → 共面合并 → 写出
// 合并失败时自动回退到基础转换，确保总能输出 STEP 文件
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "stepwriter_occt.h"

#include <opencascade/ShapeFix_Shape.hxx>
#include <opencascade/ShapeUpgrade_UnifySameDomain.hxx>
#include <opencascade/STEPControl_Writer.hxx>
#include <opencascade/StlAPI_Reader.hxx>
#include <opencascade/Standard_Failure.hxx>
#include <opencascade/TopExp_Explorer.hxx>
#include <opencascade/TopoDS.hxx>

#include <iostream>

// 面片数阈值：超过此值跳过 UnifySameDomain 避免 OOM
static constexpr int kMaxFacesForUnify = 50000;

static int countFaces(const TopoDS_Shape& shape)
{
    int n = 0;
    for(TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next())
        ++n;
    return n;
}

int convertStlToStep(const char* stlPath, const char* stepPath)
{
    if(!stlPath || !stepPath) return 0;

    try {

        // ============================================================
        // ① 读取 STL：自动缝合相邻三角形，生成有拓扑的 TopoDS_Shape
        // ============================================================
        StlAPI_Reader stlReader;
        TopoDS_Shape shape;
        if(!stlReader.Read(shape, stlPath)) {
            std::cerr << "[STL→STEP] StlAPI_Reader failed for " << stlPath << std::endl;
            return 0;
        }
        const int faceCountBefore = countFaces(shape);
        std::cout << "[STL→STEP] STL read OK, faces: " << faceCountBefore << std::endl;

        // ============================================================
        // ② 形状修复：修正容差超差、面方向不一致、缺边等几何缺陷
        // ============================================================
        ShapeFix_Shape fixer(shape);
        fixer.SetPrecision(1.0e-4);
        fixer.Perform();
        shape = fixer.Shape();
        std::cout << "[STL→STEP] ShapeFix done" << std::endl;

        // ============================================================
        // ③ 合并共面面片（面片数超过阈值则跳过，避免 OOM/超时）
        // ============================================================
        if(faceCountBefore <= kMaxFacesForUnify)
        {
            try {
                ShapeUpgrade_UnifySameDomain unifier(shape, true, true, true);
                unifier.SetAngularTolerance(1.0e-6);
                unifier.SetLinearTolerance(1.0e-4);
                unifier.SetSafeInputMode(true);
                unifier.AllowInternalEdges(false);
                unifier.Build();
                shape = unifier.Shape();
                const int faceCountAfter = countFaces(shape);
                std::cout << "[STL→STEP] Face unify done, faces: "
                          << faceCountBefore << " → " << faceCountAfter << std::endl;
            } catch(Standard_Failure const& e) {
                std::cerr << "[STL→STEP] Face unify skipped (OCCT error): "
                          << e.GetMessageString() << std::endl;
            } catch(std::exception const& e) {
                std::cerr << "[STL→STEP] Face unify skipped (std exception): "
                          << e.what() << std::endl;
            } catch(...) {
                std::cerr << "[STL→STEP] Face unify skipped (unknown exception)" << std::endl;
            }
        }
        else
        {
            std::cout << "[STL→STEP] Face unify skipped (too many faces: "
                      << faceCountBefore << " > " << kMaxFacesForUnify << ")" << std::endl;
        }

        // ============================================================
        // ④ 写出 STEP 文件
        // ============================================================
        STEPControl_Writer writer;
        IFSelect_ReturnStatus transferStatus = writer.Transfer(shape, STEPControl_AsIs);
        if(transferStatus != IFSelect_RetDone) {
            std::cerr << "[STL→STEP] STEP Transfer failed, status: " << transferStatus << std::endl;
            return 0;
        }
        if(writer.Write(stepPath) != IFSelect_RetDone) {
            std::cerr << "[STL→STEP] STEP Write failed" << std::endl;
            return 0;
        }
        std::cout << "[STL→STEP] STEP saved: " << stepPath << std::endl;
        return 1;

    } catch(Standard_Failure const& e) {
        std::cerr << "[STL→STEP] OCCT error: " << e.GetMessageString() << std::endl;
        return 0;
    } catch(std::exception const& e) {
        std::cerr << "[STL→STEP] std exception: " << e.what() << std::endl;
        return 0;
    } catch(...) {
        std::cerr << "[STL→STEP] unknown exception" << std::endl;
        return 0;
    }
}
