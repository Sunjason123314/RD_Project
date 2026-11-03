# RD_Project 交接文档

## 项目状态

- 项目名称、CMake 目标和 Release 可执行文件均为 `RD_Project`。
- 新仓库：`https://github.com/Sunjason123314/RD_Project.git`
- 主分支：`main`。
- 本地设备采集数据、构建目录和运行时生成的 `CF4000JL*.bt` 均已忽略，禁止提交。

## 已移除的功能

为交付轻量版，已彻底删除 PLC / Modbus 相关内容：

- `PLC/` 目录及 PLC 连接、寄存器读写、运动控制代码。
- JogPanel、手动点动、导轨、转台、电机、急停、偏心/倾斜 PLC 页面。
- `mainwindow.ui` 内相关页面、按钮、状态显示和地址配置。
- Qt SerialBus 依赖。

同时已删除 `Polar_Figure` 页面、极坐标图表代码和 Qt Charts 依赖。

## 光谱共焦（CF4000）

- 单通道页保留“实时测量值”面板，显示当前选中通道的测量结果（mm）。
- 数值显示刷新限制为最高 10 Hz；传感器采样和数据缓存频率不受影响。
- 程序启动后自动在后台扫描并连接第一台 CF4000，不连接 CF3000Lite。
- 未找到设备时最多尝试 5 次，间隔 1 秒。扫描、连接、Dark 和采样均在 `ConfocalSensorThread` 中执行，不阻塞界面。
- 自动连接成功后执行当前通道 Dark；仅 Dark 成功时自动启动采样。Dark 或连接失败时不启动采样，并在日志中记录原因。

## Release 构建与运行

Release 构建目录：

```text
build\Desktop_Qt_5_15_2_MSVC2019_64bit-Release
```

在项目根目录的 PowerShell 中构建：

```powershell
cmd /c 'call "E:\software\VS\vs2019\VC\Auxiliary\Build\vcvars64.bat" >nul && "E:\software\QT15.2\Tools\CMake_64\bin\cmake.exe" --build "build\Desktop_Qt_5_15_2_MSVC2019_64bit-Release" --parallel 2'
```

运行：

```powershell
.\build\Desktop_Qt_5_15_2_MSVC2019_64bit-Release\RD_Project.exe
```

Release 目录已部署 Qt、PCL、VTK、Boost、OpenCV、OpenCascade、MATLAB、QGLViewer、CyUSB 和 CF4000 SDK 所需 DLL。重新创建新的构建目录后，需要再次部署运行依赖；优先运行 `windeployqt --release --compiler-runtime RD_Project.exe`，并将项目 SDK 与第三方运行 DLL 一并复制到可执行文件目录。

## 验证重点

在连接一台 CF4000 机箱的设备上验证：

1. 程序启动后 5 秒内自动开始扫描。
2. 找到设备后依次出现“连接成功 → Dark → 启动采样”日志。
3. 未连接设备时日志显示 5 次尝试后停止，界面仍可正常操作。
4. 单通道实时数值随当前通道变化，界面刷新平稳。

## 版本控制

- 使用 `origin` 指向新的 `RD_Project` 仓库。
- 推送时可通过本机代理：`git -c http.proxy=http://127.0.0.1:7897 push`。
- 提交前执行 `git diff --check`，不要提交 `data/`、`build/` 或设备运行生成文件。
