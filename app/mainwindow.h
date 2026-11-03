#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "include.h"
#include "CF_sensor/confocalsensorworker.h"
#include "DLP/dlp.h"
#include "thread/threadmanager.h"
#include "pointcloud/pointcloudworker.h"
#include "pointcloud/Point_Algorithm.h"

#include <QDockWidget>
#include <QElapsedTimer>
#include <QFutureWatcher>
#include <QPoint>
#include <QProgressBar>
#include <QColor>
#include <QModelIndex>
#include <QStringList>

class QStandardItemModel;
class QBoxLayout;

// Qt Designer 生成界面的前置声明，以及图表相关类型声明。
QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

/**
 * @brief 主窗口类
 *
 * MainWindow 负责协调 UI、标定计算、光谱共焦传感器、相机/DLP、
 * 点云处理、状态栏以及手动点动面板等主流程。
 *
 * 说明：本次整理仅对声明进行功能分区和注释补充，不改变原有成员声明。
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    // ==================== 构造、析构与外部访问接口 ====================
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    // 工业相机设备列表缓存。
    GxIAPICPP::gxdeviceinfo_vector vectorDeviceInfo;

    // DLP 已改为后台 worker，主线程不再直接创建或访问 DLP 硬件对象。


public:
    // ==================== 光谱共焦传感器：初始化、采集与数据处理 ====================
    // 获取系统时间字符串。
    QString systemTime();

    // 单通道模式初始化。
    void SingleChannelModeInit();

    // 光源模式初始化。
    void lightModeInit();

    // 通讯模式初始化；CF3000Lite 无通讯模式时可跳过部分配置。
    void communicationModeInit();

    // 滤波与信号参数初始化。
    void filterAndSignalModeInit();

    // 输入/触发模式初始化；CF3000Lite 会减少部分触发模式配置。
    void importModeInit();

    // 状态栏初始化。
    void statusBarInit();

    void StatuesBar_flash();

    // 测量数据保存。
    void saveJingtaiToFile(QVector<double>& jingtai, const QString& filePath);
    void saveTwoArraysToFile(const QVector<double>& jingtai, const QVector<double>& jiaodu, const QString& filePath);
    void someFunction();

    // 几何拟合计算。
    // 传感器和图表数据缓存。
    int jishu;
    QVector<double> jingtai;
    QVector<double> jiaodu;
    QVector<double> jiaodu2;
    QVector<double> cgq2;
    QVector<double> jiaodu3;
    QVector<double> cgq3;

    // 界面刷新与显示状态。
    int xunhuan; // 界面刷新频率。
    // 路径和绘图初始化流程。
    void Connect_executeStep();

private slots:
    // ==================== 文件选择、传感器连接与测量启动 ====================
    void on_actionLink_clicked();
    void on_pushButton_clicked();
    void on_actionStart_clicked();

    // 单通道测量参数。
    void on_comboBoxSingleChannelMeasureModeSelect_currentIndexChanged(int index);
    void on_checkBoxSingleChannelABS_stateChanged(int arg1);
    void on_checkBoxSingleChannelReverse_stateChanged(int arg1);
    void on_checkBoxSingleChannelMultiDistance_stateChanged(int arg1);
    void on_pushButtonSingleChannelMOdeDark_clicked();
    void on_pushButtonSingleChannelModeZero_clicked();
    void on_pushButtonSingleChannelModeGetLastResult_clicked();

    // ==================== 相机、DLP 与投影控制 ====================
    void on_pushButton_display_clicked();
    void on_pushButton_OperatingModeSelect_clicked();
    void on_pushButton_ProjectOnce_clicked();
    void on_pushButton_ProjectContinuous_clicked();
    void on_pushButton_ProjectStop_clicked();
    void on_pushButton_testui_clicked();
    void on_pushButton_refresh_device_clicked();
    void on_pushButton_snapall_clicked();
    void on_pushButton_saveonce_all_clicked();
    void on_pushButton_checkall_countsave_clicked();
    void on_pushButton_checkall_save_clicked();
    void on_pushButton_testpartteb_clicked();
    void on_SelectPatternroot_clicked();
    void on_groupnumber_confirm_clicked();
    void on_picturenumber_confirm_clicked();
    void on_horizontalSlider_sliderReleased();
    void on_saverootpath_confirm_clicked();
    void on_select_subpath_clicked();
    void on_Calculate_SL_clicked();

    // ==================== 菜单动作、点云导入与三维处理 ====================
    void on_action_v_clicked();
    void on_action_jilian_clicked();
    void on_action_hide_clicked();
    void on_cadImport_clicked();
    void on_reconstructSurface_clicked();
    void on_analysis_clicked();
    // ==================== 状态栏参数读写 ====================
    void getStatusBarResult();
    void setStatusBarValue();

    // ==================== 光源、信号滤波、通讯与编码输入参数 ====================
    // 光源参数。
    void on_comboBoxExposureTime_currentIndexChanged(int index);
    void on_comboBoxSingleChannelSelect_currentIndexChanged(int index);
    void on_comboBoxLightMode_currentIndexChanged(int index);
    void on_comboBoxLightReference_currentIndexChanged(int index);

    // 信号和滤波参数。
    void on_comboBoxExportSignalSelect_currentIndexChanged(int index);
    void on_comboBoxSignalSortSelect_currentIndexChanged(int index);
    void on_doubleSpinBoxSignalThresholdValue_valueChanged(double arg1);
    void on_doubleSpinBoxSignalRatio_valueChanged(double arg1);
    void on_spinBoxSignalSensitivity_valueChanged(int arg1);
    void on_spinBoxSignalCount_valueChanged(int arg1);
    void on_spinBoxSignalMinNum_valueChanged(int arg1);
    void on_spinBoxSignalSmoothlength_valueChanged(int arg1);
    void on_spinBoxSignalMedianFilter_valueChanged(int arg1);
    void on_spinBoxSignalAverageFilter_valueChanged(int arg1);

    // 通讯参数。
    void on_comboBoxCommunicationMode_currentIndexChanged(int index);
    void on_comboBoxSerialPort_currentIndexChanged(int index);
    void on_comboBoxDataFormat_currentIndexChanged(int index);
    void on_comboBoxBaudRate_currentIndexChanged(int index);
    void on_comboBoxVerify_currentIndexChanged(int index);

    // 编码输入和光强参数。
    void on_comboBoxTriggerMode_currentIndexChanged(int index);
    void on_comboBoxEncodeWorkMode_currentIndexChanged(int index);
    void on_comboBoxEncodeInputMode_currentIndexChanged(int index);
    void on_spinBoxEncodeDivision_valueChanged(int arg1);
    void on_horizontalSliderLightValue_valueChanged(int value);

    void on_actionCF_Sensor_triggered();
    void on_ChineseEnglishSwitch_clicked();

    // ==================== 点云标签栏交互 ====================
    void onPointCloudListCurrentChanged(const QModelIndex& current, const QModelIndex& previous);
    void onPointCloudListSelectionChanged(const QItemSelection& selected, const QItemSelection& deselected);
    void onDicLineSelected(int lineIndex);
    void showPointCloudListContextMenu(const QPoint& pos);
    void toggleSelectedPointCloudVisibility();
    void setSelectedPointCloudVisibility(bool visible);
    void deleteSelectedPointCloud();
    void createPointCloudGroup();
    void onPointCloudGroupCreated(const QString& groupName);
    // 点击分组标题行时切换折叠/展开
    void togglePointCloudGroupCollapsed(const QModelIndex& index);
    // 依据各分组的折叠状态刷新行的显示/隐藏与标题箭头
    void refreshPointCloudGroupView();
    // 删除分组标题行并清除成员的分组标记（数据项保留）
    void deletePointCloudGroupKeepItems(int headerRow);
    // 把当前选中的数据项归入指定分组（拖入分组时调用）
    void assignSelectedItemsToGroup(const QString& groupName);
    // 按唯一编号把指定数据项归入分组（自动生成拟合结果时调用）。
    void assignPointCloudItemToGroup(quint64 itemId, const QString& groupName);
    // 把当前选中的数据项移出分组（拖到无分组区域时调用）
    void removeSelectedItemsFromGroup();
    void on_comboBox_colorselect_currentIndexChanged(int index);
    void on_MoreColor_clicked();
    void on_Select_Multicorn_clicked(bool checked);
    void saveSelectedPointCloudEntries();
    void on_SOR_Method_clicked();
    void on_MLS_Method_clicked();
    void on_NormalConsistency_clicked();
    void on_PCA_Method_clicked();
    void on_Centroid_Method_clicked();
    void on_recon_ball_clicked();
    void on_recon_cylinder_clicked();
    void on_Fit_clicked();
    void on_coarseregistration_clicked(); // coarse registration
    void on_RT_Matrix_clicked();          // apply stored RT matrix to selected data
    void on_AxisCalibrate_clicked();
    void on_axisrotate_clicked();
    void on_ICP_Trans_clicked();
    void on_ToPcd_clicked();




private:
    // ==================== 标定计算参数与流程状态 ====================
    // 光谱共焦传感器配置状态。
    QString factoryFilePath; // CF3000 出厂配置文件搜索路径。

    // 图像文件读取工具。
    bool read_imgs(string path, vector<cv::Mat>& imgs);
    void getFiles(string path, vector<string>& files);

    // 结构光重建工具。
    QString normalizedPathForSl(const QString& path) const;
    void appendSlLog(const QString& msg);
    QStringList selectedSlSubpaths;


    // 状态栏参数。
    QLabel* lableStatus = new QLabel;
    int triggerMode = -1;
    QString sdkVersionResult;
    QString controlerVersionResult;
    QString controlerSNResult;
    QString sensorSNResult;
    int curFrameRate = 0;

private:
    void initial_UI();
    // 将主 Tab 与信息输出框放入可拖动的上下 QSplitter，分隔条可调整二者高度。
    void setupCentralSplitter();
    // 相机窗口关闭销毁时从 display_widget_list 移除，避免野指针；幂等，重复调用安全。
    void registerCameraWindowCleanup(CamDisplay *window);
    void setupPointCloudWorkspaceUi();
    void stylePointCloudWorkspace();
    void refreshPointCloudWorkspaceState();
    QWidget *ensurePointCloudSidebar();
    QWidget *ensurePointCloudViewerPanel();
    QWidget *ensurePointCloudControlPanel();
    QWidget *ensurePointCloudLogPanel();
    void setupPointCloudListToolbar(QBoxLayout *sidebarLayout);
    void regroupPointCloudActionButtons(QWidget *optionsPage);
private:
    // ==================== 核心对象、UI 与控制器 ====================
    Ui::MainWindow* ui;
    QProgressBar* m_stepProgressBar = nullptr;

    void showStepProgress(const QString& label);
    void updateStepProgress(int pct, const QString& msg);
    void hideStepProgress();

    Calibration* calibration;
    File_Read_Write* fileReaderWriter;
    ShareResource* g_Global;
    // 统一线程管理器负责各后台模块的停止和释放。
    ThreadManager* threadManager = nullptr;

    // 光谱共焦传感器后台 worker；GUI 线程只保存指针用于 invokeMethod 投递 SDK 操作。
    ConfocalSensorWorker* confocalWorker = nullptr;

    // 光谱共焦传感器设置对话框（持久成员，非 WA_DeleteOnClose）。
    // CF 设置分页在首次打开时从 tabWidget_tri 移入其内部 QTabWidget，之后常驻。
 
    // DLP 投影仪对象；直接在主线程创建并同步调用（对齐 qt_camera，避免与相机 USB 操作跨线程冲突导致 I2C NAK）。
    DLP *dlp1 = nullptr;

    // 点云文件加载后台 worker；OpenGL 显示控件留在 GUI 线程，文件读取和数据准备放入 PointCloudDisplayThread。
    PointCloudWorker *pointCloudWorker = nullptr;

    // GUI 线程中的 DLP 初始化状态缓存，避免按钮槽函数直接访问 DLP 子线程对象成员。
    bool dlpInitialized = false;

    // GUI 线程中的点云加载状态缓存，避免重复提交文件导入任务。
    bool pointCloudLoadInProgress = false;
    bool slReconstructBusy = false;
    // 批量导入时剩余的后台加载任务数量；归零后再恢复“文件导入”按钮。
    int pendingPointCloudLoadCount = 0;
    // 左侧点云标签栏模型；当前 ui 控件是 QListView，因此用 model 管理文件名列表。
    QStandardItemModel *pointCloudListModel = nullptr;
    QStringList pointCloudFilePaths;

    struct SphereFitData {
        Eigen::Vector3f center = Eigen::Vector3f::Zero();
        float radius = 0.0f;
    };
    QHash<int, SphereFitData> sphereFitResults; // key = meshIndex

    // 圆柱拟合的参数与其可视化轴线。当前仅在运行期保存，工程持久化将在下一步实现。
    struct CylinderFitData {
        Eigen::Vector3f axisPoint = Eigen::Vector3f::Zero();
        Eigen::Vector3f axisDirection = Eigen::Vector3f::UnitZ();
        float radius = 0.0f;
        float minAxisT = 0.0f;
        float maxAxisT = 0.0f;
        int axisLineIndex = -1;
    };
    QHash<int, CylinderFitData> cylinderFitResults; // key = meshIndex
    Eigen::Matrix4f m_coarseRegTransform = Eigen::Matrix4f::Identity();
    bool m_hasCoarseRegResult = false;

    // 回转轴标定结果
    bool m_hasRotationAxis = false;
    Eigen::Vector3f m_axisPoint;
    Eigen::Vector3f m_axisDirection;
    float m_axisRadius = 0.0f;

    enum class PointCloudEntryType : int {
        PointCloud = 0,
        Reconstruction = 1,
        FittedGeometry = 2,
        FittedAxisLine = 3,
        Group = 4
    };
    static constexpr int RoleEntryType = Qt::UserRole + 1;
    static constexpr int RoleMeshIndex = Qt::UserRole + 2;
    static constexpr int RolePointCloudIndex = Qt::UserRole + 3;
    static constexpr int RoleLineIndex = Qt::UserRole + 4;
    static constexpr int RoleUniqueId = Qt::UserRole + 5;
    // 分组相关角色：
    //  RoleGroupName  —— 分组标题行保存分组名；成员行保存其所属分组名（空=未分组）
    //  RoleGroupCollapsed —— 仅分组标题行使用，标记该分组当前是否折叠
    static constexpr int RoleGroupName = Qt::UserRole + 6;
    static constexpr int RoleGroupCollapsed = Qt::UserRole + 7;
    // 拖拽重排相关：为每个列表项分配唯一 id，用于拖拽移动后恢复选中项
    quint64 nextListItemId = 1;
    QList<quint64> pendingReorderIds;
    void restoreListSelectionByIds(const QList<quint64>& ids);
    void relinkReconstructionMeshIndicesAfterRemoval(int removedMeshIndex);
    void relinkFittedGeometryMeshIndicesAfterRemoval(int removedMeshIndex);
    void relinkPointCloudIndicesAfterRemoval(int removedPointCloudIndex);
    void relinkLineIndicesAfterRemoval(int removedLineIndex);

    void __InitUI();
    CamDisplay *display_widget = nullptr;
    std::vector<CamDisplay *> display_widget_list;

    struct FloatingTabPage
    {
        QWidget *page = nullptr;       ///< 从 Tab_Main 移出的原始页面对象。
        QDockWidget *dock = nullptr;   ///< 承载页面的浮动/停靠窗口。
        QString title;                 ///< 原 tab 标题。
        QIcon icon;                    ///< 原 tab 图标。
        int originalIndex = 0;         ///< 原 tab 顺序，用于停靠回 Tab_Main 时恢复位置。
    };
    QMap<QWidget*, FloatingTabPage> floatingMainTabs; ///< 已被拉出的 Tab_Main 页面表。

    // PCA 切片模式状态
    bool pcaSlicingActive = false;
    Point_Algorithm::PCAResult pcaSlicingResult;
    int pcaCurrentSliceCloudIndex = -1;
    int pcaOriginalCloudIndex = -1;
    float pcaSliceMinT = 0.0f, pcaSliceMaxT = 0.0f;
    int pcaSliceNum = 20;
    float pcaSliceTolerance = 0.5f;
    int pcaActiveSliceIndex = 0;
    std::vector<float> pcaSlicePositions;
    QWidget *pcaSlicePanel = nullptr;
    bool pcaPanelDragging = false;
    QPoint pcaPanelDragOffset;
    void enterPCASlicingMode(const Point_Algorithm::PCAResult& r, PCLCloud::Ptr cloud, int cloudIndex, int numSlices, float tolerance);
    void exitPCASlicingMode();
    void updatePCASlice();

private:
    // ==================== 定时器与辅助面板 ====================
    // Qt 界面刷新与传感器采集定时器。
    QTimer* confTimer = nullptr;
    QTimer* StatusBarTimer = nullptr;
    QElapsedTimer confocalDisplayRefreshTimer;
    bool autoStartSamplingAfterDark = false;
    int autoConnectCF4000Attempts = 0;
    void startAutoConnectCF4000Attempt();

    // ==================== 点云重建与窗口连接 ====================

    // ==================== 模块线程管理 ====================
    void initThreadManager();
    void shutdownModuleThreads();

    // ==================== Tab_Main 浮动/停靠管理 ====================
    void initFloatingMainTabs();
    void showMainTabContextMenu(const QPoint& pos);
    void floatMainTab(int index);
    void restoreMainTab(QWidget *page);
    int tabInsertIndexForOriginalIndex(int originalIndex) const;
 
    // ==================== 点云线程管理 ====================
    // 初始化点云 worker，并通过 ThreadManager 移入独立线程执行文件读取和数据准备。
    void initPointCloudThread();

    // 停止点云线程；OpenGL 显示控件自身仍由 GUI 线程释放。
    void shutdownPointCloudThread();

    // 初始化左侧点云标签栏和颜色下拉框。
    void initPointCloudFileListUi();
    void addPointCloudFileListItem(const QString& filePath,
                                   const QString& customDisplayName = QString(),
                                   int entryType = 0,
                                   int meshIndex = -1,
                                   int pointCloudIndex = -1,
                                   int lineIndex = -1);
    void updatePointCloudColorComboToCurrentCloud();
    QColor colorFromPointCloudColorCombo(int index) const;
    bool saveSinglePointCloudToFile(int row);
    bool savePointCloudPackage(const QModelIndexList& selectedRows);
    bool loadPointCloudPackage(const QString& packagePath);
    bool loadPointCloudFileOrPackage(const QString& path);
  
    // ==================== 光谱共焦传感器线程管理 ====================
    // 初始化光谱共焦传感器 worker，并通过 ThreadManager 移入独立线程运行。
    void initConfocalSensorThread();

    // 停止光谱共焦传感器线程；停止前会关闭 SDK 采样和设备句柄。
    void shutdownConfocalSensorThread();

    // 接收子线程读取到的初始化参数快照，并在 GUI 线程刷新控件。
    void applyConfocalInitialConfig(const ConfocalInitialConfig& config);

    // 构造光谱共焦设置对话框并把 5 个 CF 分页从 tabWidget_tri 移入（启动时调用）。

    // 接收子线程读取到的 SDK/控制器/传感器状态快照，并刷新状态栏缓存。
    void updateConfocalStatusSnapshot(const ConfocalStatusSnapshot& snapshot);

    // 接收 SDK 回调转换后的测量数据，并更新 GUI 线程侧共享缓存。
    void updateConfocalMeasurementResult(int channelIndex, float result, float saturation, const QVector<int>& signalData);

    // 以下 postConfocalXXX 是跨线程投递入口，保证 SDK 参数设置都在光谱共焦 worker 线程执行。
    void postConfocalSetIntParam(const QString& paramName, int channelIndex, int value, const QString& successMessage = QString());
    void postConfocalSetFloatParam(const QString& paramName, int channelIndex, float value, const QString& successMessage = QString());
 
protected:
    // ==================== Qt 事件与运行期初始化钩子 ====================
    void closeEvent(QCloseEvent* event) override;
    void dragEnterEvent(QDragEnterEvent *event) override;
    void dropEvent(QDropEvent *event) override;
    bool eventFilter(QObject* obj, QEvent* event) override;

    void initRuntimeBuffers();
    void initTimers();
};

#endif // MAINWINDOW_H
