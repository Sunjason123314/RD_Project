# 交接文档 — 点云列表分组功能 & ICP 多片

写给完全没有上下文的新会话。项目是一个 Qt5 (5.15.2 / MSVC2019 x64) + CUDA + PCL 的叶片测量桌面程序，
主工程 CMake 目标名 `auto_measurement_cmake`。**用户自己在 Qt Creator 里 build，不要在命令行尝试
build**（命令行缺少 MSVC 开发环境变量，会报 `type_traits`/`memory` 找不到的假错误，那是环境问题不是代码问题）。

## 版本控制约定

- 本地 `data/` 是采集图片、点云、标定和中间输出数据（当前约 64.77 GB），**不提交、不推送**；已由 `.gitignore` 忽略。
- 如需共享测试数据，请使用项目约定的独立数据存储，不要将其加入 Git 提交。

### 分支与双机协作（2026-11-01 起）

- `main` 是已在笔记本完成实机验证的稳定主程序；只接收已验证的合并。两台电脑均以它为稳定基线。
- `beta` 是集成与测试分支；新功能在笔记本完成实机验证前，先合入这里。
- `feature/<功能名>` 用于较独立或较大的功能开发。例如 `feature/blade-reconstruction`、`feature/jog-control`。
  **分支按代码成熟度划分，不按台式机或笔记本划分。**
- `origin`（HIT-Visual-Measurement-Research-Group/Auto-Measurement）是唯一主仓库；`blade`
  （Sunjason123314/blade_measurement）仅作为镜像备份。日常拉取和协作一律使用 `origin`。

推荐流程：台式机从 `beta` 创建 `feature/<功能名>` 开发并推送；笔记本检出同一功能分支进行设备实测；
通过后合入 `beta`；`beta` 整体验证稳定后再合入 `main`，并创建版本标签（如 `v3.3.8`）。不要直接在 `main` 上开发。

台式机开发新功能：

```powershell
git switch beta
git pull --ff-only origin beta
git switch -c feature/<功能名>
```

笔记本测试已推送的功能：

```powershell
git fetch origin
git switch feature/<功能名>
git pull --ff-only origin feature/<功能名>
```

更新稳定主程序：

```powershell
git switch main
git pull --ff-only origin main
```

---

## 一、任务总览

围绕主界面左侧的点云文件列表 `listWidgetPointCloudFiles`（在 `mainwindow.ui` 中），做三类改动：

1. **列表交互增强**：Word 式框选（可见橡皮筋矩形、从任意未选中项/空白起框、框选时滚轮滚动不丢已选）、
   拖拽排序后不弹回顶部、拖动项时能靠边缘自动滚动。
2. **分组功能**：右键新建分组、命名、把选中项归组、点击分组标题折叠/展开、拖拽入组、右键"移动到分组"、
   删除分组保留数据。
3. **ICP 多片**：基准点云可多选（合并为一个目标）、待变换点云可多选并支持"整体变换/各自独立"。

---

## 二、已完成（全部已实现，等用户 build 验证）

### 新增自定义控件 `PointCloudListView`（`pointcloud/pointcloudlistview.{h,cpp}`）
继承 **QListView**（不是 QTreeView，血泪教训见下），已在 `CMakeLists.txt`（QGL_SOURCE 段，167-168 行）、
`mainwindow.ui`（widget class + customwidgets 段 4685 行）注册。提供：
- 左键框选：起点用**内容坐标**（viewport 坐标 + 滚动偏移），显示 `QRubberBand`；重写 `scrollContentsBy`
  在框选中滚动时用最近鼠标位置立即重算选择，解决"框选时滚轮滚动导致滚出视野的项丢失"。
- 单击分组标题行 → `emit groupHeaderClicked(index)`。
- 拖拽落在分组标题/成员行 → `emit filesDroppedIntoGroup(组名)`，drop 设 `IgnoreAction`（不走默认删行）；
  落在无分组区 → 先基类排序再 `emit filesDroppedOutOfGroup()`。
- `dropEvent` 后 `restoreScroll()`（QTimer::singleShot(0)）恢复滚动位置，避免弹回顶部。
- 拖动项时边缘自动滚动：构造里 `setAutoScroll(true)` + `setAutoScrollMargin(28)`，并在 `dragMoveEvent`
  里手动兜底（鼠标 y 贴近上/下边缘时滚动 scrollbar）。因为 QDrag 拖放期间 Qt 不派发滚轮事件。

### 分组数据模型（扁平，不用父子树！）
- `mainwindow.h`：`PointCloudEntryType` 枚举加了 `Group = 4`；新增角色
  `RoleGroupName = Qt::UserRole+6`（标题行存组名；成员行存所属组名，空=未分组）、
  `RoleGroupCollapsed = Qt::UserRole+7`（仅标题行用）。
- 所有点云项仍在**模型顶层**（扁平）。分组 = 一个不可选/不可拖的"标题行" + 成员项打 `RoleGroupName` 标记 +
  用 `setRowHidden` 实现折叠。**数据项从不移动到子层级，从不销毁。**

### 分组相关函数（`mainwindow.cpp`）
- `createPointCloudGroup()` / `onPointCloudGroupCreated(name)`：新建分组标题行，把选中项标记归组并重排到标题行下方。
- `togglePointCloudGroupCollapsed(index)` + `refreshPointCloudGroupView()`：折叠/展开，标题显示 `▼组名`/`▶组名`，
  用 `setRowHidden` 隐藏成员行。
- `deletePointCloudGroupKeepItems(headerRow)`：删标题行 + 清成员 GroupName，**保留数据**。
- `assignSelectedItemsToGroup(name)` / `removeSelectedItemsFromGroup()`：拖拽入组/出组，也被右键菜单复用。
- 右键菜单 `showPointCloudListContextMenu`：
  - 右键分组标题 → 折叠/展开、删除分组（保留数据）。
  - 右键空白且无选中 → 只给"新建分组"（建空组）。
  - 右键选中项 → 显示/隐藏/保存/删除/新建分组 + **"移动到分组▸"子菜单**（列出已有组）+ **"移出分组"**（仅当选中项有已归组的）。
- 信号连接在 `initPointCloudFileListUi()` 里，用 `qobject_cast<PointCloudListView*>` 连
  groupHeaderClicked / filesDroppedIntoGroup / filesDroppedOutOfGroup。
- `deleteSelectedPointCloud()` 已适配：Group 行只删行本身；删除后选第一个非 Group 的可见数据行。

### ICP 多片（`Select_UI/icp_dialog.ui` + `mainwindow.cpp::on_ICP_Trans_clicked`）
- UI：基准从 `comboBoxReference`(单选) 改成 `listWidgetReference`(QListWidget 多选)；源列表 `listWidgetSource` 下方
  加 `checkBoxWholeTransform`("整体变换")。**comboBoxReference 已删除，别再引用。**
- 逻辑：多片基准 `*referenceCloud += *c` 合并成一个目标；勾选整体变换时多片源合并求**一个**变换矩阵，
  再用同一矩阵分别变换每片（各片仍独立成项）；不勾选则每片各自 `performICP`（原行为）。源点云自动排除已作基准的。
- `ICP_CUDA::performICP` 在 `pointcloud/icp_cuda.h`，`PCLCloud = pcl::PointCloud<pcl::PointXYZ>`。

---

## 三、当前状态 / 卡在哪里

**没有卡住。** 所有代码改动已落盘，逻辑自洽，等用户在 Qt Creator 里 build + 实机验证。
上一次编译错误（`QListView::isRowHidden` 只接受 1 个参数）已修复为 `isRowHidden(row)`。

改动文件清单：
- `pointcloud/pointcloudlistview.h` / `.cpp`（新增）
- `mainwindow.h`（枚举 + 角色 + 函数声明）
- `mainwindow.cpp`（分组/拖拽/右键/ICP/初始化连接/include）
- `mainwindow.ui`（widget 提升为 PointCloudListView + customwidgets）
- `Select_UI/icp_dialog.ui`（基准多选 + 整体变换勾选框）
- `CMakeLists.txt`（QGL_SOURCE 加两文件）

---

## 四、下一步计划（等用户反馈后）
- 用户 build 后实测：框选滚轮、拖拽入组、边缘自动滚动、折叠展开、右键移动到分组、ICP 多片各路径。
- 潜在待观察点（尚未验证）：
  - 拖拽入组后成员重排顺序是否符合预期（当前逐个 `takeRow`+`insertRow` 到标题行下方）。
  - 折叠状态在增删点云、拖拽后是否始终一致（`refreshPointCloudGroupView` 已在多处调用，但没穷举所有入口）。
  - 分组信息目前**不持久化**（保存/加载工程时不会记住分组）；用户没要求，先没做。
  - ICP 整体变换模式下 `convergedFlags` 与 `sourceCloudNames` 数量可能不完全对齐，主线程回调里已用
    `i < sourceCloudNames.size()` 守卫，但值得再核对。

---

## 五、踩过的坑（务必不要重蹈）

1. **绝对不要把列表改成 QTreeView / 父子树结构做分组。**
   曾经用 QTreeView + 把项 `appendRow` 到分组子节点，结果：**点云功能全失效 + 拖入分组数据丢失**。
   根因：`mainwindow.cpp` 里有 **34+ 处** `pointCloudListModel->item(row)`，这个只查顶层行；项一旦变成子项就找不到，
   索引断裂。要改成递归查询风险极高。**现在的扁平模型 + GroupName 标记 + setRowHidden 是最终正确方案，别推翻。**

2. **不要在命令行 build。** 用户明确要自己在 Qt Creator build（"我自己build就可以"）。命令行 `cmake --build`
   会因缺 MSVC 环境报 `type_traits`/`memory` 找不到——是环境问题，不是代码错误，别被误导去"修"。

3. **`QListView::isRowHidden(int)` 只接受 1 个参数**（QTreeView 才有带 parent 的重载）。别写成两参。

4. **拖入分组时 drop 必须设 `Qt::IgnoreAction`** 并自己重排，否则默认 InternalMove 会删源行留下空白可选行
   （用户明确抱怨过"原位置变成空白的可选择还能拖动"）。

5. **分组标题行必须不可选、不可拖、不可作为放置目标**（`~ItemIsSelectable & ~ItemIsDragEnabled & ~ItemIsDropEnabled`），
   否则会混进 selectedRows 参与点云操作导致异常。

6. **框选起点必须用内容坐标（含滚动偏移），不能用固定 viewport 坐标**，否则滚轮一滚，已选项因
   `ClearAndSelect` 重算 + 滚出视野而被清掉（用户明确抱怨过两次）。

7. **`.ui` 改了控件后**（如 icp_dialog.ui 删 comboBoxReference、加 listWidgetReference/checkBoxWholeTransform），
   AUTOUIC 一般自动重生成 `ui_*.h`；若报找不到成员，让用户重跑一次 CMake。

8. **Ctrl+C / Ctrl+V 用户最后决定不做**，别自作主张加。

9. 编辑 `mainwindow.cpp` 时注意：文件很大，函数有重复的相似代码块（删除逻辑等），改之前先 grep 定位准确函数，
   曾经因 Edit 产生过重复/错位代码块。
