#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "include.h"
#include "CF_sensor/confocalsensorworker.h"
#include "DLP/dlpworker.h"
#include "thread/threadmanager.h"
#include "pointcloud/pointcloudworker.h"
#include "pointcloud/Point_Algorithm.h"

#include <QDockWidget>
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
namespace QtCharts {
class QPolarChart;
class QChartView;
}
}
QT_END_NAMESPACE

/**
 * @brief 主窗口类
 *
 * MainWindow 负责协调 UI、标定计算、光谱共焦传感器、PLC/运动控制、相机/DLP、
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

    // PLC 控制器只读访问入口。PLC 已放入子线程，外部模块不要直接跨线程调用其成员函数。
    Plc* plc() const { return plcController; }

private:
    // ==================== 极坐标图表显示 ====================
    // 极坐标图表和视图指针，用于传感器数据的动态刷新显示。
    QtCharts::QPolarChart *m_polarChart = nullptr;
    QtCharts::QChartView *m_chartView = nullptr;

    // 初始化绘图容器，并绑定到界面中的 chartWidget。
    void initChartContainer();

    // 根据传感器值与角度刷新极坐标图。
    void updatePolarChart(const QVector<double> &cgq2, const QVector<double> &jiaodu2);

public:
    // ==================== 导轨/PLC 基础运动接口 ====================
    // 向 PLC 寄存器和线圈写入目标值，并按指定时间等待。
    void sendRegistersAndVar(const double Target, double address, double address2, double sleep_time);

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

    // 传感器数据读取入口。
    void mahrSensor();
    void confSensor();
    void StatuesBar_flash();
    void mahrSensor77();
    void mahrSensorpx();
    void mahrSensor772();
    void mahrSensor773();
    void mahrSensor774();

    // 测量数据保存。
    void saveJingtaiToFile(QVector<double>& jingtai, const QString& filePath);
    void saveTwoArraysToFile(const QVector<double>& jingtai, const QVector<double>& jiaodu, const QString& filePath);
    void someFunction();

    // 几何拟合计算。
    QVector<double> fitCircleCenter(const QVector<double>& l, const QVector<double>& a);
    QVector<double> fitPlaneNormal(const QVector<double>& l, const QVector<double>& a, double r);

    // 传感器和图表数据缓存。
    int jishu;
    QVector<double> jingtai;
    QVector<double> jiaodu;
    QVector<double> jiaodu2;
    QVector<double> cgq2;
    QVector<double> jiaodu3;
    QVector<double> cgqpxht;
    QVector<double> jiaodupxht;
    QVector<double> cgq3;

    // 界面刷新与显示状态。
    int xunhuan; // 界面刷新频率。
    double banjing;
    bool isYellow = true;
    bool isYellow1 = true;
    QVector<double> dataVector;
    QVector<double> dataVector2;

    // 路径、绘图和 PLC 初始化流程。
    void initPaths();
    void Draw();
    void initPlcPaths();
    void Connect_executeStep();

public slots:
    // ==================== 对外运动控制槽函数 ====================
    // 平移、Z 向和调倾/偏心运动控制。
    void move_motor(double center, double center2, double reg_adress, double xian_adress, double reg_adress2, double xian_adress2);
    void move_motor_z(double bottom_position_z, double reg_adress_z, double xian_adress_z);
    void move_motor_t(double tiltX, double tiltY, double eccX, double eccY,
                      double reg_adress_t_x, double xian_adress_t_x,
                      double reg_adress_t_y, double xian_adress_t_y,
                      double reg_adress_c_x, double xian_adress_c_x,
                      double reg_adress_c_y, double xian_adress_c_y);

    // 定时采集控制。
    void stop_timer();
    void start_timer();

private slots:
    // ==================== 文件选择、传感器连接与测量启动 ====================
    void on_btnChooseFile_clicked();
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

    // ==================== 标定与导轨移动 ====================
    void on_calculate_clicked();
    void on_load_calibration_data_clicked();
    void on_Daogui_X_clicked();
    void on_Daogui_Y_clicked();
    void on_Daogui_Z_clicked();
    void on_write_ball_A_points_clicked();
    void on_write_ball_A_done_clicked();
    void on_write_ball_B_points_clicked();
    void on_write_ball_B_done_clicked();

    // ==================== 相机、DLP 与投影控制 ====================
    void on_pushButton_display_clicked();
    void on_pushButton_OperatingModeSelect_clicked();
    void on_pushButton_InternalPatternControl_clicked();
    void on_pushButton_testui_clicked();
    void on_pushButton_refresh_device_clicked();
    void on_pushButton_snapall_clicked();
    void on_pushButton_saveonce_all_clicked();
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
    void on_confirm_clicked();
    void on_record_done_clicked();
    void on_recover_clicked();
    void on_rotation_clicked();

    // ==================== 电机使能、旋转与位置运动 ====================
    void on_btnMotor_Enable_clicked(bool checked);
    void setLED(QLabel* label, int color, int size);
    void on_btnRotate_Velocity_clicked();
    void on_btnRotate_clicked();
    void on_btnRotateStop_clicked();
    void on_btnPosition_Velocity_clicked();

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

    // ==================== 测量流程按钮 ====================
    void initial_velocity();
    void on_RecordY_clicked();
    void on_RecordSensor_clicked();
    void on_lcdNumberSingleChannelResult_overflow();
    void on_Motionless_clicked();
    void on_Motion_clicked();
    void on_SingleCenterMeasure_clicked();
    void on_ReadDistance_clicked();
    void on_Readangle_clicked();
    void on_sensorE_move_clicked();
    void on_sensorF_move_clicked();
    void on_sensorA_move_clicked();
    void on_sensorB_move_clicked();
    void on_sensorC_move_clicked();
    void on_sensorD_move_clicked();
    void on_smallRotate_angle_move_clicked();
    void on_RecordCenter_clicked();
    void on_RecordMeasure_clicked();
    void on_ReadHeight_clicked();
    void on_ReadAngle_clicked();
    void on_RecordTilt_clicked();

    // ==================== 急停、辅助导轨和图表窗口 ====================
    void on_emergency_stop_clicked();
    void on_Daogui_Y_2_clicked();
    void on_Daogui_Z_2_clicked();
    void on_actionFigure_triggered();

    // ==================== 手动点动面板与按钮按压释放事件 ====================
    void on_btnShowJogPanel_clicked();

    void X_positive_pressed();
    void X_positive_released();
    void X_negative_pressed();
    void X_negative_released();

    void Y_positive_pressed();
    void Y_positive_released();
    void Y_negative_pressed();
    void Y_negative_released();

    void Z_positive_pressed();
    void Z_positive_released();
    void Z_negative_pressed();
    void Z_negative_released();

    void adjustA_positive_pressed();
    void adjustA_positive_released();
    void adjustA_negative_pressed();
    void adjustA_negative_released();

    void adjustB_positive_pressed();
    void adjustB_positive_released();
    void adjustB_negative_pressed();
    void adjustB_negative_released();

    // ==================== PLC 连接与界面语言切换 ====================
    void on_actionPlcCConect_triggered();
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
    void on_Fit_clicked();
    void on_coarseregistration_clicked(); // coarse registration
    void on_RT_Matrix_clicked();          // apply stored RT matrix to selected data
    void on_AxisCalibrate_clicked();
    void on_axisrotate_clicked();
    void on_ICP_Trans_clicked();
    void on_ToPcd_clicked();




private:
    // ==================== 标定计算参数与流程状态 ====================
    double G_R1;
    double G_R2;
    double G_BALL_DISTANCE;
    Matrix3d M_opt;
    VectorXd theta;
    double alpha;
    double beta;
    bool enable_flag;
    double rotation_theta;
    double delta_rotation_theta;
    int i_rote;
    bool safeguard = true; // 传感器防撞保护。

    // 标定路径与结果路径。
    QString basePath;       // 标定存储根路径。
    QString ParameterPath;  // 标定结果路径。
    QString ball_A_path;    // 标定球 A 数据路径。
    QString ball_B_path;    // 标定球 B 数据路径。
    QString rotation_angel;

    // 光谱共焦传感器配置状态。
    QString factoryFilePath; // CF3000 出厂配置文件搜索路径。

    // 标定球与 PLC 地址配置。
    QVector<QVector<Vector3d>> center_all_tri; // 三坐标机下的修正球心坐标。
    int groupCount;                            // 标准球组数。
    double address_x1, address_y1, address_z1, address_x2, address_y2, address_z2; // PLC 寄存器地址。
    double address_y12, address_y22, address_z12, address_z22;
    double addressqing_reg_1, addressqing_reg_2, addressqing_reg_3, addressqing_reg_4; // 调倾寄存器地址。
    double addressqing_cyl_1, addressqing_cyl_2, addressqing_cyl_3, addressqing_cyl_4; // 调倾线圈地址。
    double addressxin_reg_1, addressxin_reg_2; // 偏心寄存器地址。
    double addressxin_cyl_1, addressxin_cyl_2; // 偏心线圈地址。

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

    // 目标位置与运动状态。
    double target;
    const double tolerance_z_target = 0.5;
    bool zMoveActive = false; // 当前是否仍允许 Z 正向运动。
    bool xMoveActive = false;
    bool yMoveActive = false;

    // ==================== 手动点动状态机 ====================
    enum class JogAxis {
        None,
        X,
        Y,
        Z,
        AdjustA,
        AdjustB
    };

    enum class JogDir {
        Positive,
        Negative
    };

    struct PendingJogCommand {
        bool valid = false;
        JogAxis axis = JogAxis::None;
        JogDir dir = JogDir::Positive;
    };

    bool jogActive = false;   // 当前是否在运动。
    bool jogBusy = false;     // 是否正在执行启动/停止过程。
    bool jogStopping = false; // 是否处于停止阶段。

    JogAxis currentJogAxis = JogAxis::None;
    JogDir currentJogDir = JogDir::Positive;

    PendingJogCommand pendingJog;

    int jogCommandId = 0;   // 用于取消旧命令。
    int speedRestoreId = 0; // 用于取消旧速度恢复。

private:
    // ==================== 手动点动时序常量 ====================
    static constexpr int STOP_SETTLE_MS = 180;         // 停止稳定时间。
    static constexpr int START_DELAY_MS = 120;         // 启动延迟。
    static constexpr int ESTOP_PULSE_MS = 80;          // 急停脉冲宽度。
    static constexpr int RESTORE_SPEED_DELAY_MS = 800; // 恢复正常速度延迟。

private:
    // ==================== 手动点动内部控制函数与键盘状态 ====================
    void setJogButtonDown(JogAxis axis, JogDir dir, bool down);
    void initial_UI();
    void setupPointCloudWorkspaceUi();
    void stylePointCloudWorkspace();
    void refreshPointCloudWorkspaceState();
    QWidget *ensurePointCloudSidebar();
    QWidget *ensurePointCloudViewerPanel();
    QWidget *ensurePointCloudControlPanel();
    QWidget *ensurePointCloudLogPanel();
    void setupPointCloudListToolbar(QBoxLayout *sidebarLayout);
    void regroupPointCloudActionButtons(QWidget *optionsPage);
    void requestStartJog(JogAxis axis, JogDir dir);
    void doStartJog(JogAxis axis, JogDir dir);
    void doStopJog();
    void finishStopAndStartPending();
    void emergencyStopJog();
    void clearAllJogKeyStates();

    bool aKeyPressed = false;
    bool dKeyPressed = false;
    bool leftKeyPressed = false;
    bool rightKeyPressed = false;
    bool upKeyPressed = false;
    bool downKeyPressed = false;

signals:
    // ==================== 跨模块通知信号 ====================
    void emit_move_motor(double center, double center2, double reg_adress, double xian_adress, double reg_adress2, double xian_adress2);
    void motorTiltFinished(); // 电机调倾动作完成信号。

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
    // 统一线程管理器：PLC 和光谱共焦传感器都通过它 moveToThread、停止和释放。
    ThreadManager* threadManager = nullptr;

    // 光谱共焦传感器后台 worker；GUI 线程只保存指针用于 invokeMethod 投递 SDK 操作。
    ConfocalSensorWorker* confocalWorker = nullptr;
 
    // PLC 控制对象已交给 ThreadManager 移入 PLC 子线程；主线程只保存指针用于投递 queued 调用。
    Plc *plcController = nullptr;

    // DLP 投影仪后台 worker；GUI 线程只保存指针用于投递 queued 调用，真实 DLP 对象在子线程创建。
    DlpWorker *dlpWorker = nullptr;

    // 点云文件加载后台 worker；OpenGL 显示控件留在 GUI 线程，文件读取和数据准备放入 PointCloudDisplayThread。
    PointCloudWorker *pointCloudWorker = nullptr;

    // GUI 线程使用的 PLC 数据缓存。由 Plc::dataUpdated 跨线程发送快照后更新，界面刷新只读这里。
    PlcDataSnapshot plcSnapshot;

    // GUI 线程中的 PLC 连接状态缓存，避免直接跨线程调用 Plc::isPlcConnected()。
    bool plcConnectedFlag = false;

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
    int nextCameraInstanceId = 1; ///< 相机线程实例递增编号，保证多个 CamDisplay 的 ThreadManager key 不冲突。

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
    QTimer* mahrTimer = nullptr;
    QTimer* mahrTimer77 = nullptr;
    QTimer* confTimer = nullptr;
    QTimer* StatusBarTimer = nullptr;
    JogPanel *jogPanel = nullptr;

    // ==================== 点云重建与窗口连接 ====================
    bool isArmInPosition();
    void intial_window_connect();

    // ==================== 模块线程管理 ====================
    void initThreadManager();
    void shutdownModuleThreads();

    // ==================== Tab_Main 浮动/停靠管理 ====================
    void initFloatingMainTabs();
    void showMainTabContextMenu(const QPoint& pos);
    void floatMainTab(int index);
    void restoreMainTab(QWidget *page);
    int tabInsertIndexForOriginalIndex(int originalIndex) const;
 
    // ==================== PLC 线程管理 ====================
    // 初始化 PLC worker，并通过 ThreadManager 移入独立线程运行。
    void initPlcThread();

    // 停止 PLC 线程；关闭窗口或统一停止模块线程时调用。
    void shutdownPlcThread();

    // 接收 PLC 子线程发来的数据快照，并更新 GUI 线程缓存 plcSnapshot。
    void updatePlcSnapshot(const PlcDataSnapshot& snapshot);

    // 以下 postPlcXXX 是跨线程投递入口：只负责把 PLC 操作排队到 PLC 子线程执行。
    // 新增或修改界面逻辑时不要直接调用 plcController->xxx()，应通过这些封装函数。
    void postPlcConnect();
    void postPlcDisconnect();
    void postPlcStartCollectingData();
    void postPlcStartContinuousMovement(double cylinder, double address);
    void postPlcMoveToAbsolutePosition(double cylinder, double address);
    void postPlcWriteHoldingRegisters(double registers, double address);
    void postPlcWriteCoilData(double value, double address);

    // ==================== DLP 线程管理 ====================
    // 初始化 DLP worker，并通过 ThreadManager 移入独立线程；真实硬件连接由按钮触发 initialize()。
    void initDlpThread();

    // 停止 DLP 线程；停止前会在 DLP 子线程调用 shutdown() 释放 I2C/硬件连接。
    void shutdownDlpThread();

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
    void initPlcRuntimeConnection();
};

#endif // MAINWINDOW_H
