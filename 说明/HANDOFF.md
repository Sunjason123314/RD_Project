# HANDOFF.md — 叶片测量系统：表面重建与会话交接

## 一、项目背景

这是一个**叶片测量 QT/C++ 桌面应用**（`blade_measurement_v3.3.1`），使用结构光扫描 + 光谱共焦传感器获取叶片（叶身 + 榫头）的三维点云数据，需要对其进行表面重建并导出 STEP 格式用于 CAM 加工。

**核心约束**：最终网格面片数有上限（当前设为 ≤3000 面），但叶片是复杂封闭曲面（叶身大曲率 + 榫头近平面但有棱线），不能简单地均匀减面导致曲面"塌平"。

## 二、架构总览

### 依赖库

| 库 | 用途 |
|---|---|
| Qt 5.15.2 (MSVC 2019 x64) | UI、多线程 |
| PCL (vcpkg) | 点云处理、曲面重建 (Poisson/Greedy/NURBS)、文件 I/O |
| VTK (随 PCL) | 网格减面 (vtkQuadricDecimation) |
| OpenCASCADE (vcpkg) | STEP 文件读写 |
| QGLViewer 2.8.0 | 3D 可视化 |
| Boost, OpenCV, CUDA, Eigen | 辅助 |

### 关键源文件

```
pointcloud/
├── Point_Algorithm.h/cpp   ← 核心：重建、减面、修复、曲率计算
├── stepexporter.h/cpp       ← STEP 导出（NURBS→STL→OCCT→STEP）
├── stepimporter.h/cpp       ← STEP 导入（OCCT 读取+三角化）
├── stepwriter_occt.h/cpp    ← 底层 STL→STEP 转换（纯 OCCT）
├── dicqglviewer.h/cpp       ← 3D 显示控件 + PCLMesh/PCLCloud 类型定义
├── pointcloudworker.h/cpp   ← 后台点云加载线程
└── draw.cu / drawalgorithm  ← CUDA 相关
mainwindow.h/cpp             ← 主窗口，所有 UI 逻辑入口
Select_UI/                   ← 各种对话框 .ui 文件
说明/
├── STEP导入导出_交接说明.md ← 之前写的 STEP 模块文档
└── HANDOFF.md               ← 本文件
```

### 数据流（STEP 导出）

```
点云 → NURBS(resolution=39, ~2888面) → repairMesh → decimateMesh(3000) → STL → OCCT → STEP
      失败时回退 Poisson + decimateMesh(3000)
```

## 三、本轮已完成的工作

### 1. 面片预算控制
- `decimateMesh` 从局部静态函数提升为 `Point_Algorithm` 公共方法
- 所有路径（STEP 导出 + 交互重建）统一 ≤3000 面
- 渐进式减面（每步 ≤50%），避免一步到位产生破损
- 减面后清理退化三角形（零面积/重复顶点）

### 2. 网格修复 `repairMesh`
- 新增 `Point_Algorithm::repairMesh(PCLMesh&)` 
- 流程：`vtkCleanPolyData` 合并重合顶点 → 遍历剔除退化面片 → 回写 PCLMesh
- 主窗口重建流程和 STEP 导出都已接入

### 3. Poisson 重建优化
- **法向朝向修正**：从 PCA 视点法改为质心朝外法（封闭曲面正确方案）
- `setManifold(true)`、`setSamplesPerNode(2.0)`、`setScale(1.25)`

### 4. Greedy 重建
- 参数恢复到 PCL 推荐值（搜索半径 3.5×平均间距、mu=2.0、maxNN=80）
- **关键修复**：自适应降采样仅对 Poisson 启用（`usePoisson`），Greedy 跳过——因为 Greedy 需要均匀点距

### 5. 曲率分析与特征提取
- `computePointCurvature(cloud, k)`：半径搜索（自动计算半径=平均间距×√k）+ PCA 特征值比 `λ₀/Σλ`
- `detectFeaturePoints`：曲率梯度检测（邻域曲率差 RMS > mean+2σ）
- `detectEdgesDoN`：双半径法向差异边缘检测
- `extractFeatureCloud`：联合特征提取（曲率梯度 ∪ DoN）
- `adaptiveCurvatureDownsample`：按曲率分特征区/平坦区，不同体素尺寸降采样
- UI 按钮 "顶点分析"（`on_analysis_clicked`）：弹对话框选 k 和阈值 → 计算曲率 → 显示统计（min/max/mean/median） → 高曲率点红色显示

### 6. STEP 导入/导出
- `stepexporter.h/cpp`：添加 `ProgressFn` 回调，导出过程报告进度
- `stepimporter.h/cpp`：添加 `ProgressFn` 回调，导入过程报告进度
- `writeMeshToStep`：精简→写 STL→OCCT 转换，全流程异常保护

### 7. 文档
- `说明/STEP导入导出_交接说明.md`：库依赖、文件清单、数据流、配置参数

## 四、当前卡住的位置

### 核心问题：单一重建方法无法同时满足叶身和榫头

- **叶身**（复杂曲面）：Poisson 重建 + 充分面片预算 → 效果好
- **榫头**（近平面 + 棱线/榫槽）：Poisson 会把棱线抹平滑 → 损失几何特征；Greedy 会出离散面片

**目前已做到的**：曲率分析可以分离高曲率区（叶身）和低曲率区（榫头平面），两区点云已能分开显示。

**尚未做到的**：对两个区域分别用不同方法重建再合并。

## 五、下一步计划（推荐方案）

### 基于文献调研的混合重建方案

```
点云 → computePointCurvature → 按阈值分割
  │
  ├─ 高曲率区（叶身）
  │     └─ Poisson 重建 → 保留曲面细节
  │
  └─ 低曲率区（榫头）
        ├─ RANSAC 平面拟合（已有 fitPlane3D）
        ├─ 提取平面边界点 → 约束三角化
        └─ 形成平面基元网格
  │
  └─ 合并两个 mesh → repairMesh → decimateMesh(3000) → 最终网格
```

### 具体实施步骤

1. **分割保存**：在 `analysis` 按钮中增加"导出分割结果"——高曲率点云和低曲率点云分别保存为列表条目（不同颜色），用户可确认分割是否合理
2. **平面重建**：对低曲率区点云，用 `fitPlane3D`（RANSAC 平面拟合）提取主要平面 → 用 `BoundaryEstimation` 或凸包算法找平面边界 → 边界三角化
3. **合并**：`PCLMesh` 拼接（顶点合并 + 去重）
4. **调参**：曲率阈值需要针对实际数据校准（当前默认 0.00002，实际数据显示 max≈0.3, median≈5e-6）

### 参考论文

| 论文 | 关键思路 |
|---|---|
| Hu et al. ISPRS 2018 — *Hybrid Representation Reconstruction* | 超体素区域生长分离平面/曲面，平面用参数基元、曲面用三角网格 |
| Zhang et al. CVPR 2025 — *High-Fidelity Lightweight Mesh* | 曲率自适应顶点生成，高曲率区密平坦区疏，Delaunay 保水密 |
| Fu et al. IEEE TMM 2024 — *LFS-Aware Surface Reconstruction* | 局部特征尺寸统一度量平面和曲面，自适应网格密度 |
| Dong et al. SIGGRAPH 2024 — *NeurCADRecon* | 强制零高斯曲率重建可展 CAD 曲面，平面/圆柱自动分离 |

## 六、踩过的坑（不要再踩）

### 1. AUTOUIC 新增 .ui 文件不生效
- **现象**：在 CMakeLists.txt 加 `.ui` 文件后，`Ui::ClassName` 找不到
- **原因**：CMake AUTOUIC 需要 re-configure 才能识别新增 .ui。重新运行 CMake configure 后自动生成 `ui_xxx.h` 即可
- **解决方案**：新增 .ui 文件后需在 Qt Creator 中 re-run cmake（或删除 build 目录重新 configure）。项目中已有多个 .ui 正常工作（`Select_UI/*.ui`），新增对话框应优先用 .ui 文件以保持一致性
- **注意**：`mainwindow.cpp` 文件常被 IDE 锁定，Edit 工具可能报 `EPERM`，需要用 `python` 写脚本改，或关闭 IDE 中的该文件再改

### 2. `vtkDecimatePro.h` 可能不存在
- **现象**：编译报错找不到头文件
- **原因**：用户的 VTK 版本（随 PCL 安装）可能不包含此 filter
- **解决方案**：仅使用 `vtkQuadricDecimation`（已验证可用），不要依赖 `vtkDecimatePro`

### 3. `Point_Algorithm` 新增方法在其他翻译单元中找不到
- **现象**：`stepexporter.cpp` 调用 `Point_Algorithm::nurbsResolutionForTargetFaces(300)` 报未声明
- **原因**：MSVC 的某种头文件包含顺序问题（可能与 `using namespace std;` 在 `dicqglviewer.h` 中有关）
- **解决方案**：直接硬编码数值（如 resolution=39），在 `Point_Algorithm.cpp` 内部保留函数实现供本文件调用

### 4. Greedy 三角化 + 自适应降采样 = 灾难
- **现象**：Greedy 重建后出现大量离散面片和断裂
- **原因**：自适应降采样故意制造不均匀点距（曲面密、平坦疏），Greedy 的局部投影算法假设均匀点距
- **解决方案**：自适应降采样**仅对 Poisson 启用**（`if (usePoisson)`），Greedy 跳过

### 5. 圆柱体曲率检测为 0
- **现象**：`λ₀/Σλ` 对圆柱体表面给出极小的曲率值
- **原因**：用 k 最近邻搜索时，局部邻域点在圆柱面上近似平面，λ₀≈0
- **解决方案**：改用**半径搜索**（radius = 平均点间距 × √k），物理尺度固定，能捕获圆柱等大曲面

### 6. 文件锁
- `mainwindow.cpp` 和 `mainwindow.ui` 经常被 IDE 锁定，Edit 工具失败时尝试用 `python` 脚本或 `sed` 替代

### 7. `using namespace std;` 污染
- `dicqglviewer.h` 第 51 行有 `using namespace std;`
- 这在全局作用域引入了整个 std 命名空间，可能导致其他文件的符号冲突
- 新增代码时注意不要依赖这个 using，始终使用 `std::` 前缀

## 七、快速上手检查清单

1. 编译环境：Qt Creator + MSVC 2019 x64 + Ninja
2. 构建目录：`build/Desktop_Qt_5_15_2_MSVC2019_64bit-Release/`
3. 运行 `on_analysis_clicked` 测试曲率分析功能（选点云 → 点"顶点分析"按钮 → 输 k=30, threshold=0.00002）
4. 查看日志面板的曲率统计（min/max/mean/median）来校准后续阈值
5. 所有算法类方法在 `Point_Algorithm` 中（`pointcloud/Point_Algorithm.h`）
6. STEP 导出入口：`StepExporter::exportMeshToStep` / `exportPointCloudToStep`
7. STEP 导入入口：`StepImporter::importStep`
- 依赖 `Point_Algorithm::estimateRigidTransformSVD()` 计算矩阵
- 依赖 `Point_Algorithm::fitSphere3D()` 提供球心数据
