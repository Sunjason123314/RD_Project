# 叶片测量系统 — STEP 导入/导出模块交接说明

## 一、依赖库总览

| 库 | 用途 | 版本/来源 |
|---|---|---|
| **Qt** | UI框架、多线程、序列化 | 5.15.2 (MSVC 2019 x64) |
| **PCL** (Point Cloud Library) | 点云滤波、法向估计、曲面重建(Poisson/Greedy/NURBS)、文件I/O(PCD/PLY/STL) | vcpkg |
| **VTK** (via PCL) | 网格精简 (vtkQuadricDecimation) | 随 PCL 附带 |
| **OpenCASCADE (OCCT)** | STEP 文件读写、形状修复、面片合并 | vcpkg |
| **Boost** | 跨线程同步 (boost::thread) | vcpkg |
| **QGLViewer** | 3D 点云/网格可视化控件 | libQGLViewer-2.8.0 |
| **OpenCV** | 图像 I/O、标定图像处理 | vcpkg |
| **CUDA** | GPU 加速 (结构光解码、点云计算) | CUDA Toolkit (arch 75) |
| **Eigen** | 矩阵/向量运算 (PCA、SVD、几何拟合) | 随 PCL 附带 |
| **JsonCpp** | 结构光参数 JSON 解析 | vcpkg |
| **MATLAB Runtime** | 读取 stereoParams.mat | MATLAB 2022a |
| **CF4000 SDK** | 光谱共焦传感器驱动 | 厂商 SDK |
| **DLP API** | DLP 投影仪控制 | TI DLP SDK |
| **CAM SDK** | 工业相机采集 | Daheng Galaxy SDK |

### OCCT 链接的模块

```
TKernel  TKMath    TKG2d      TKG3d      TKGeomBase
TKGeomAlgo  TKBRep   TKTopAlgo  TKSTEP    TKSTL
TKMesh   TKShHealing
```

---

## 二、STEP 导入/导出相关文件

### 核心文件（6 个）

| 文件 | 职责 |
|---|---|
| [stepexporter.h](../pointcloud/stepexporter.h) | STEP **导出**类声明。定义 `ProgressFn` 回调类型，声明 `exportPointCloudToStep` / `exportMeshToStep` |
| [stepexporter.cpp](../pointcloud/stepexporter.cpp) | STEP **导出**实现。NURBS/面 → STL 临时文件 → OCCT STL→STEP |
| [stepimporter.h](../pointcloud/stepimporter.h) | STEP **导入**类声明。定义 `ProgressFn`，声明 `importStep` / `lastError` |
| [stepimporter.cpp](../pointcloud/stepimporter.cpp) | STEP **导入**实现。纯 OCCT 读取 STEP → 三角化 → PCLMesh |
| [stepwriter_occt.h](../pointcloud/stepwriter_occt.h) | 底层 STL→STEP 转换声明（无 PCL/Qt 依赖） |
| [stepwriter_occt.cpp](../pointcloud/stepwriter_occt.cpp) | 底层转换实现：STL 读入 → ShapeFix 修复 → UnifySameDomain 合并共面 → STEP 写出 |

### 支撑文件（3 个）

| 文件 | 职责 |
|---|---|
| [Point_Algorithm.h](../pointcloud/Point_Algorithm.h) | 网格处理算法声明：`reconstructMesh`、`decimateMesh`、`nurbsResolutionForTargetFaces`、`numFacesFromNurbsResolution` |
| [Point_Algorithm.cpp](../pointcloud/Point_Algorithm.cpp) | 网格处理算法实现：Greedy/Poisson/NURBS 曲面重建、VTK Quadric 减面、NURBS 分辨率计算 |
| [dicqglviewer.h](../pointcloud/dicqglviewer.h) | 定义全局类型别名 `PCLMesh` = `pcl::PolygonMesh`、`PCLCloud` = `pcl::PointCloud<pcl::PointXYZ>` |

### 调用入口（1 个）

| 文件 | 相关功能 |
|---|---|
| [mainwindow.cpp](../mainwindow.cpp) | `on_reconstructSurface_clicked()` — 曲面重建；`saveSinglePointCloudToFile()` — STEP 导出触发；`on_cadImport_clicked()` — STEP 导入触发（文件选择 → `StepImporter::importStep`） |

---

## 三、数据流

### STEP 导出流程

```
点云 (PCLCloud)
  │
  ├─ 优先路径：NURBS 重建
  │    Point_Algorithm::nurbsReconstruction(cloud, order=3, refinement=5, resolution=39)
  │    → 直接生成 ~2888 三角形，顶点在光滑 B-spline 曲面上，保持曲面特征
  │
  └─ 回退路径：Poisson 重建 + 精简
       Point_Algorithm::reconstructMesh(cloud, ReconMethod::Poisson, ...)
       → Point_Algorithm::decimateMesh(mesh, 3000)  ← VTK Quadric 减面
  │
  ▼
PCLMesh (~2888 面)
  │
  ▼
writeMeshToStep()
  ├─ decimateMesh(mesh, 3000)   ← 保底精简
  ├─ pcl::io::savePolygonFileSTL(tmp.stl, mesh)   ← 写临时 STL
  ├─ convertStlToStep()         ← OCCT 转换
  │    ├─ StlAPI_Reader: STL → TopoDS_Shape
  │    ├─ ShapeFix_Shape: 修复几何缺陷
  │    ├─ ShapeUpgrade_UnifySameDomain: 合并共面面片 (≤50000面时)
  │    └─ STEPControl_Writer: 写出 .stp 文件
  └─ 删除临时 STL
```

### STEP 导入流程

```
.stp 文件
  │
  ▼
StepImporter::importStep(path, mesh, progress)
  ├─ STEPControl_Reader: 读取 STEP → TopoDS_Shape
  ├─ BRepMesh_IncrementalMesh: 三角化所有面
  ├─ TopExp_Explorer: 遍历面提取三角形 & 顶点
  └─ 组装 PCLMesh (polygons + cloud)
  │
  ▼
PCLMesh → DicGLWidget 显示
```

---

## 四、关键配置参数

| 参数 | 值 | 说明 |
|---|---|---|
| 面片预算 | **3000 面** | 最终模型 ≤3000 三角面片 |
| NURBS resolution | **39** | `floor(√(3000/2))+1`，生成 2×(39−1)² = 2888 面 |
| NURBS order | 3 | 三次 B-spline，保证 C² 连续性 |
| NURBS refinement | 5 | 迭代拟合次数 |
| Poisson depth | 8 | 八叉树深度（回退路径用） |
| Poisson solver divide | 5 | 求解器细分（回退路径用） |
| Poisson iso divide | 5 | 等值面细分（回退路径用） |
| 面合并阈值 | 50000 | OCCT UnifySameDomain 的最大面数，超过跳过防 OOM |

---

## 五、注意事项

1. **曲面保持策略**：NURBS 重建是核心手段。顶点落在光滑 B-spline 曲面上，即使只有 ~2888 个三角形，法向仍连续变化，不会像 Poisson/Greedy 硬砍 3000 面那样"塌平"。

2. **OCCT 异常处理**：`writeMeshToStep` 和 `convertStlToStep` 都有 try-catch 保护，捕获 `Standard_Failure`、`std::exception` 和 unknown exception，失败时返回 false 而非崩溃。

3. **线程安全**：STEP 导出/导入均在 `QtConcurrent::run` 子线程执行，通过 `QFutureWatcher` + `QEventLoop` 同步等待，不阻塞 UI。进度通过 `ProgressFn` 回调投递到 GUI 线程。

4. **编码限制**：STEP 文件名和路径使用 `toLocal8Bit()` 转换，确保 OCCT（仅接受 `const char*`）在中文 Windows 下正常工作。

5. **减面顺序**：`decimateMesh` 先将网格转 vtkPolyData → vtkQuadricDecimation → 回写 PCLMesh。如果输入面数已 ≤ 目标值则直接返回。
