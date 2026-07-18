# 代码精简任务清单（TASK）

> 记录时间：2026-07-17
> 依据：两个只读盘点代理的结果（未引用文件 + 重复函数）
> 前提：本地 git 已建好，首次提交 8eb13f2 做兜底，删错可 `git checkout` 恢复。
> 执行原则：**风险从低到高分阶段做，每阶段做完编译验证 + 单独 git commit**。

---

## 阶段一 — 需求4：删除未使用的文件/文件夹（风险低，先做）

安全策略：**先移动到 `_trash/` 目录，编译通过后再物理删除**（不直接 rm）。

### 1.1 确定可删（盘点确认：不参与编译、无 #include、无调用）

- [ ] `third_party/QXlsx-/`（整个文件夹，含 QXlsx-1.4.5 / "QXlsx-1.4.5 - 副本" / QXlsx 及 TestExcel/WebServer/HelloWorld 等示例工程）
- [ ] `third_party/QXlsx/`（整个文件夹）
- [ ] `third_party/QXlsx-1.4.5.zip`
- [ ] `第三方库/qcustomplot/`（整个文件夹）+ `第三方库/QCustomPlot.tar.gz`
      —— 实际编译用的是 `CF_sensor/qcustomplot`
- [ ] `SL_QT/src/test/`（mainwindow.cpp/.h、multifrequency_qt_main.cpp、multifrequency_test_cuda.cpp）
- [ ] `SL_QT/src/voidcuda.cu`
- [ ] `pointcloud/icp_cuda_impl.cu`
- [ ] `pointcloud/icp_cuda_impl.h`
- [ ] `pointcloud/icp_cuda_impl_kernels.h`
- [ ] `pointcloud/icp_cuda_impl_svd.h`
- [ ] `pointcloud/icp_cuda_svd.cpp`
      —— 以上 5 个是 `ICP_CUDA_Impl` 死代码套，无任何调用者
- [ ] `pointcloud/fitalgorithm.h`（无 #include）
- [ ] `CAM/ui/qcustomplot.cpp` + `CAM/ui/qcustomplot.h`（重复副本，**删前先 diff 与 CF_sensor 版本确认等价**）
- [ ] `DLP/samples/dlpc347x_dual_samples.c`
- [ ] `DLP/third_party/cyusbserial/`（win32 版，已在 CMake 注释；活跃的是 DLP/cyusbserial/x64）
- [ ] `DLP/_CMakeLists.txt`（遗留独立构建脚本）

### 1.2 疑似可删（执行前需我先核实）

- [ ] `CAM/Common/FileVersion.cpp/.h`、`CAM/Common/Fps.cpp/.h`
      —— 不在 CAM_SOURCE，需确认相机 SDK 头（CAM/inc、CAM/Common）没间接引用
- [ ] `DLP/cyusbserial/x86/`（只要不编译 32 位版本就可删）

### 1.3 验证
- [ ] 重新 CMake configure（因为动了文件）
- [ ] 编译通过、程序能运行
- [ ] 通过后：物理删除 `_trash/`，`git add -A && git commit -m "chore: 删除未使用的文件与重复副本"`

---

## 阶段二 — 需求2+3：删冗余、合并同功能函数（中风险）

### 2.1 拟合球（用户点名）
- [ ] 删除 `calibration.cpp:231` 的 `Calibration::fitSphere`（一行转发壳）
- [ ] 其调用点（calibration.cpp:215-216, 270-271）改为直接调 `Point_Algorithm::fitSphereLeastSquares`
- [ ] 保留 `fitSphere3D`(RANSAC) 与 `fitSphereLeastSquares`(代数LS)——两者算法互补，保留并补注释说明适用场景
- [ ] 保留 `fitEllipsoid`（椭球，非球）

### 2.2 SVD 刚体变换（三份几乎逐字相同）
- [ ] 保留 `Point_Algorithm.cpp:355 estimateRigidTransformSVD` 为唯一实现
- [ ] `icp_svd.cpp:5 computeTransformationSVD` 改为薄封装（内部转 Vector3f 调上者，再拆 float[16]）
- [ ] 第三份 `icp_cuda_svd.cpp` 已随阶段一的 ICP_CUDA_Impl 删除

### 2.3 拟合平面
- [ ] 删除 `dicqglviewer.cpp:1151 fitPlaneToSelection`（无调用者的 RANSAC 重复）及其在 dicqglviewer.h:239 的声明
- [ ] 保留 `fitPlane3D`(主实现) 与 `fitPlaneNormal`(传感器极坐标域，语义不同)

### 2.4 验证
- [ ] 编译通过、程序能运行
- [ ] `git commit -m "refactor: 合并重复的拟合/SVD实现，删除冗余入口"`

---

## 阶段三 — 需求1：mainwindow 分区（风险最高，最后做）

- [ ] `mainwindow.cpp`（11027 行）按功能加分区注释，**对齐 mainwindow.h 已有的分区分组**
      （极坐标图表 / 导轨PLC运动 / 光谱共焦传感器 / 对外运动控制槽 / 文件选择连接测量 /
       标定与导轨 / 相机DLP投影 / 菜单点云三维处理 / 电机旋转位置 / 状态栏参数 /
       光源信号通讯编码 / 测量流程按钮 / 急停辅助导轨图表 / 手动点动面板 / PLC连接语言 / 点云标签栏交互）
- [ ] 原则：只加分区注释 + 按分区聚拢函数顺序，**不改任何逻辑**
- [ ] 注意：mainwindow.cpp/.ui 可能被 IDE 锁定（Edit 报 EPERM），必要时先关 IDE
- [ ] 验证：编译通过、程序能运行
- [ ] `git commit -m "refactor: mainwindow.cpp 按功能分区"`

---

## 待用户确认的问题

1. **删除方式**：同意"先移 `_trash/` → 编译通过 → 再物理删"的安全策略吗？还是直接删（有 git 兜底）？
2. **疑似可删项**（1.2 的 FileVersion/Fps、x86）：要不要一并处理，还是保守起见先留着？
3. **阶段三分区**是否要真的移动函数位置（顺序对齐分区），还是只在现有位置加分区注释标记（改动更小、更安全）？
