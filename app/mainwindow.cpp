#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qglviewer/manipulatedCameraFrame.h>
#include "ui_stltopcd_dialog.h"
#include "ui_reconst_surface_dialog.h"
#include "ui_sor_dialog.h"
#include "ui_voxel_grid_dialog.h"
#include "ui_mls_dialog.h"
#include "ui_normal_consistency_dialog.h"
#include "ui_pca_dialog.h"
#include "ui_coarse_registration_dialog.h"
#include "ui_axis_rotate_dialog.h"
#include "ui_icp_dialog.h"
#include "morecolordialog.h"
#include "pointcloud/icp_cuda.h"
#include "pointcloud/pointcloudlistview.h"


#include <QtCharts/QLineSeries>
#include <QtCharts/QPolarChart>
#include <QtCharts/QValueAxis>
#include <QtCharts/QChartView>
#include <QGridLayout>
#include <QSplitter>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSlider>
#include <QFrame>
#include <QLabel>
#include <QSizePolicy>
#include <QPainter>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>
#include <QTimer>
#include <QDebug>
#include <QKeyEvent>
#include <QShortcut>
#include <QCloseEvent>
#include <QMetaObject>
#include <QSignalBlocker>
#include <QByteArray>
#include <QTabBar>
#include <QMenu>
#include <QAction>
#include <QDir>
#include <QFileInfo>
#include <QCollator>
#include <QInputDialog>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QScrollBar>
#include <QLineEdit>
#include <QRegularExpression>
#include <QEventLoop>
#include <QFutureWatcher>
#include <QtConcurrent/QtConcurrent>
#include <QStandardItemModel>
#include <QStandardItem>
#include <QItemSelectionModel>
#include <QItemSelection>
#include <QAbstractItemView>
#include <QListView>
#include <QComboBox>
#include <QColor>
#include <QHash>
#include <QVariant>
#include <QFileDialog>
#include <QFile>
#include <QDataStream>
#include <QMessageBox>
#include <QTextStream>
#include <QFileInfo>

#include "sl_integration_runner.h"
#include "pointcloud/stepexporter.h"
#include "pointcloud/stepimporter.h"

using namespace QtCharts;




#include <pcl/point_types.h>
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , calibration(new Calibration(this))
    , fileReaderWriter(new File_Read_Write(this))
    , plcController(nullptr)
    , jogPanel(nullptr)
    // , m_hardwaredataprovider(new  plcController,HardwareDataProvider(this))
{
    ui->setupUi(this);

    // 启动时即把光谱共焦的 5 个设置分页移入独立对话框，使主界面 tabWidget_tri
    // 一开始就只保留其余分页（对话框此时不显示，点菜单 actionCF_Sensor 才弹出）。
    initConfocalSettingsDialog();

    // 把主 Tab 与信息输出框放进上下 QSplitter，使二者之间的分隔条可拖动调整高度。
    setupCentralSplitter();

    if(ui->RT_Matrix)
        ui->RT_Matrix->setEnabled(false);  // disabled until coarse registration succeeds
    if(ui->axisrotate)
        ui->axisrotate->setEnabled(false);  // disabled until axis calibration is done or axis is imported

    // 嵌入 points_show 的进度条
    m_stepProgressBar = new QProgressBar(ui->points_show);
    m_stepProgressBar->setRange(0, 100);
    m_stepProgressBar->setValue(0);
    m_stepProgressBar->setTextVisible(true);
    m_stepProgressBar->setFormat(QStringLiteral("%p%"));
    m_stepProgressBar->setFixedHeight(22);
    m_stepProgressBar->hide();

    qApp->installEventFilter(this);
    this->setWindowTitle(tr("auto measurement"));
    this->setAcceptDrops(true);

    IGXFactory::GetInstance().Init();
    IGXFactory::GetInstance().UpdateDeviceList(1000,vectorDeviceInfo);

    for (int i = 0; i < vectorDeviceInfo.size(); ++i)
    {
        ui->comboBox_device_list->addItem(vectorDeviceInfo[i].GetDisplayName().c_str());
    }

    g_Global = ShareResource::get_instance();
    initThreadManager();
    initPlcThread();
    initConfocalSensorThread();
    // DLP 不再预创建线程；改为点击“显示”按钮时在主线程直接 new DLP（对齐 qt_camera）。
    initPointCloudThread();
    initFloatingMainTabs();
  
    connect(this ,&MainWindow::emit_move_motor,this,&MainWindow::move_motor);
    initial_UI();
    setupPointCloudWorkspaceUi();
    stylePointCloudWorkspace();
    initPointCloudFileListUi();
    refreshPointCloudWorkspaceState();
    ui->groupnumber->setMinimum(1);
    ui->picturenumber->setMinimum(1);

    initPaths();
    initPlcPaths();

    initRuntimeBuffers();
    initTimers();

    // 绑定信号槽
    intial_window_connect();

    initPlcRuntimeConnection();
    initChartContainer();
    updatePolarChart(cgqpxht, jiaodupxht);



}

void MainWindow::showStepProgress(const QString& label)
{
    if(!m_stepProgressBar || !ui)
        return;

    QWidget *viewerCard = ui->points_show
            ? ui->points_show->findChild<QWidget*>(QStringLiteral("pointCloudViewerCard"))
            : nullptr;
    QWidget *parent = viewerCard ? viewerCard : ui->points_show;
    if(!parent)
        return;

    if(m_stepProgressBar->parentWidget() != parent)
        m_stepProgressBar->setParent(parent);

    m_stepProgressBar->setGeometry(18, qMax(0, parent->height() - 40), qMax(120, parent->width() - 36), 22);
    m_stepProgressBar->setValue(0);
    m_stepProgressBar->setFormat(QStringLiteral("%v% - ") + label);
    m_stepProgressBar->show();
    m_stepProgressBar->raise();
}

void MainWindow::updateStepProgress(int pct, const QString& msg)
{
    if (!m_stepProgressBar) return;
    m_stepProgressBar->setValue(pct);
    if (!msg.isEmpty())
        m_stepProgressBar->setFormat(QStringLiteral("%v% - ") + msg);
}

void MainWindow::hideStepProgress()
{
    if (m_stepProgressBar)
        m_stepProgressBar->hide();
}

MainWindow::~MainWindow()
{
    if(confTimer)
        confTimer->stop();
    if(StatusBarTimer)
        StatusBarTimer->stop();

    shutdownModuleThreads();

    if(g_Global)
    {
        g_Global->g_handle = 0;
        g_Global->g_isConnect = false;
        g_Global->g_isStart = false;
    }

    delete ui;
}

void MainWindow::initThreadManager()
{
    if(threadManager)
        return;

    qRegisterMetaType<ThreadManager::Module>("ThreadManager::Module");

    threadManager = new ThreadManager(this);

    connect(threadManager, &ThreadManager::moduleStartFailed,
            this, [this](ThreadManager::Module module, const QString& reason) {
                ui->plainTextEdit->appendPlainText(
                    systemTime() + " " + tr("Module thread start failed: ") +
                    threadManager->moduleName(module) + QStringLiteral(" - ") + reason);
            }, Qt::QueuedConnection);

    connect(threadManager, &ThreadManager::moduleStopTimedOut,
            this, [this](ThreadManager::Module module, const QString& threadName) {
                Q_UNUSED(module)
                ui->plainTextEdit->appendPlainText(
                    systemTime() + " " + tr("Module thread stop timed out: ") + threadName);
            }, Qt::QueuedConnection);
}


void MainWindow::initPointCloudThread()
{
    qRegisterMetaType<PCLCloud::Ptr>("PCLCloud::Ptr");
    // 注册跨线程信号槽所需的元类型
    qRegisterMetaType<PCLMesh>("PCLMesh");

    if(!threadManager)
        initThreadManager();

    if(pointCloudWorker)
        return;

    // 创建工作线程对象
    pointCloudWorker = new PointCloudWorker;

    connect(pointCloudWorker, &PointCloudWorker::fileLoadStarted,
            this, [this](const QString& filePath) {
                pointCloudLoadInProgress = true;
                if(ui && ui->cadImport)
                    ui->cadImport->setEnabled(false);
                if(ui && ui->plainTextEdit)
                    ui->plainTextEdit->appendPlainText(
                        systemTime() + " " + tr("Start loading point cloud: ") + filePath);
            }, Qt::QueuedConnection);

    // 绑定信号槽
    connect(pointCloudWorker, &PointCloudWorker::meshLoadSucceeded,
            this, [this](const PCLMesh& mesh,
                         const QString& filePath,
                         const QString& fileExtension,
                         double elapsedSeconds) {
                if(pendingPointCloudLoadCount > 0)
                    --pendingPointCloudLoadCount;
                const bool allLoadsFinished = pendingPointCloudLoadCount <= 0;
                pointCloudLoadInProgress = !allLoadsFinished;
                if(ui && ui->cadImport)
                    ui->cadImport->setEnabled(allLoadsFinished);

                if(mesh.polygons.empty())
                {
                    if(ui && ui->plainTextEdit)
                        ui->plainTextEdit->appendPlainText(
                            systemTime() + " " + tr("Loaded mesh is empty. Cannot display."));
                    if(allLoadsFinished && ui && ui->plainTextEdit)
                        ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Point cloud import completed."));
                    return;
                }

                const int meshIndex = ui->DicGLWidget->addMesh(mesh);
                if(meshIndex >= 0)
                {
                    const QString tag = (fileExtension == QStringLiteral("ply"))
                        ? QStringLiteral("imported_ply")
                        : QStringLiteral("imported_stl");
                    ui->DicGLWidget->setMeshDisplayTag(meshIndex, tag);
                }

                const QFileInfo fi(filePath);
                const QString displayName = fi.completeBaseName().isEmpty() ? fi.fileName() : fi.completeBaseName();
                addPointCloudFileListItem(filePath,
                                          displayName,
                                          static_cast<int>(PointCloudEntryType::Reconstruction),
                                          meshIndex,
                                          -1);

                if(ui && ui->plainTextEdit)
                {
                    ui->plainTextEdit->appendPlainText(
                        systemTime() + " " + fileExtension.toUpper() +
                        tr(" mesh load time: ") + QString::number(elapsedSeconds, 'f', 3) + tr(" seconds."));
                    if(allLoadsFinished)
                        ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Point cloud import completed."));
                }
            }, Qt::QueuedConnection);

    // 绑定信号槽
    connect(pointCloudWorker, &PointCloudWorker::fileLoadSucceeded,
            this, [this](const PCLCloud::Ptr& cloud,
                         const QString& filePath,
                         const QString& fileExtension,
                         double elapsedSeconds) {
                if(pendingPointCloudLoadCount > 0)
                    --pendingPointCloudLoadCount;
                const bool allLoadsFinished = pendingPointCloudLoadCount <= 0;
                pointCloudLoadInProgress = !allLoadsFinished;
                if(ui && ui->cadImport)
                    ui->cadImport->setEnabled(allLoadsFinished);

                if(!cloud || cloud->empty())
                {
                    if(ui && ui->plainTextEdit)
                        ui->plainTextEdit->appendPlainText(
                            systemTime() + " " + tr("Loaded point cloud is empty. Cannot display."));
                    if(allLoadsFinished && ui && ui->plainTextEdit)
                        ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Point cloud import completed."));
                    return;
                }

                if(ui->DicGLWidget->pointCloudCount() == 0)
                {
                    ui->DicGLWidget->add_pointcloud(*cloud);
                    if(ui && ui->plainTextEdit)
                        ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Source point cloud loaded."));
                }
                else if(ui->DicGLWidget->pointCloudCount() == 1)
                {
                    ui->DicGLWidget->add_pointcloud(*cloud);
                    if(ui && ui->plainTextEdit)
                        ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Target point cloud loaded."));
                }
                else
                {
                    ui->DicGLWidget->add_pointcloud(*cloud);
                    if(ui && ui->plainTextEdit)
                        ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Additional point cloud loaded for display."));
                }

                addPointCloudFileListItem(filePath);

                if(ui && ui->plainTextEdit)
                {
                    ui->plainTextEdit->appendPlainText(
                        systemTime() + " " + fileExtension.toUpper() +
                        tr(" file load time: ") + QString::number(elapsedSeconds, 'f', 3) + tr(" seconds."));
                    if(allLoadsFinished)
                        ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Point cloud import completed."));
                }
            }, Qt::QueuedConnection);


    connect(pointCloudWorker, &PointCloudWorker::fileLoadFailed,
            this, [this](const QString& filePath, const QString& reason) {
                if(pendingPointCloudLoadCount > 0)
                    --pendingPointCloudLoadCount;
                const bool allLoadsFinished = pendingPointCloudLoadCount <= 0;
                pointCloudLoadInProgress = !allLoadsFinished;
                if(ui && ui->cadImport)
                    ui->cadImport->setEnabled(allLoadsFinished);
                if(ui && ui->plainTextEdit)
                {
                    ui->plainTextEdit->appendPlainText(
                        systemTime() + " " + tr("Point cloud load failed: ") + filePath +
                        QStringLiteral(" - ") + reason);
                    if(allLoadsFinished)
                        ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Point cloud import completed."));
                }
            }, Qt::QueuedConnection);

    connect(pointCloudWorker, &PointCloudWorker::logMessage,
            this, [this](const QString& message) {
                if(ui && ui->plainTextEdit)
                    ui->plainTextEdit->appendPlainText(systemTime() + " " + message);
            }, Qt::QueuedConnection);

    if(!threadManager->startModule(ThreadManager::Module::PointCloudDisplay,
                                   pointCloudWorker,
                                   QStringLiteral("PointCloudDisplayThread"),
                                   "shutdown"))
    {
        pointCloudWorker = nullptr;
        pointCloudLoadInProgress = false;
        pendingPointCloudLoadCount = 0;
        if(ui && ui->cadImport)
            ui->cadImport->setEnabled(true);
    }
}

void MainWindow::shutdownPointCloudThread()
{
    if(threadManager)
        threadManager->stopModule(ThreadManager::Module::PointCloudDisplay, 3000);

    pointCloudWorker = nullptr;
    pointCloudLoadInProgress = false;
    pendingPointCloudLoadCount = 0;
}

void MainWindow::initPointCloudFileListUi()
{
    if(!ui || !ui->listWidgetPointCloudFiles)
        return;

    if(!pointCloudListModel)
        pointCloudListModel = new QStandardItemModel(this);

    ui->listWidgetPointCloudFiles->setModel(pointCloudListModel);
    ui->listWidgetPointCloudFiles->setSelectionMode(QAbstractItemView::ExtendedSelection);
    ui->listWidgetPointCloudFiles->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->listWidgetPointCloudFiles->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->listWidgetPointCloudFiles->setContextMenuPolicy(Qt::CustomContextMenu);
    ui->listWidgetPointCloudFiles->setUniformItemSizes(true);

    // Word 式交互 + 树形分组：
    // - 从空白处按住拖拽 / Ctrl+左键 / Shift+左键 => 标准多选（ExtendedSelection 已提供）
    // - 在已选中项上按住左键拖拽 => 移动位置或拖入分组（InternalMove 提供）
    ui->listWidgetPointCloudFiles->setDragEnabled(true);
    ui->listWidgetPointCloudFiles->setAcceptDrops(true);
    ui->listWidgetPointCloudFiles->setDropIndicatorShown(true);
    ui->listWidgetPointCloudFiles->setDragDropMode(QAbstractItemView::InternalMove);
    ui->listWidgetPointCloudFiles->setDefaultDropAction(Qt::MoveAction);
    ui->listWidgetPointCloudFiles->setDragDropOverwriteMode(false);

    // 拖拽移动后同步 DicGLWidget 高亮与工作区状态
    connect(pointCloudListModel, &QAbstractItemModel::rowsMoved, this,
            [this](const QModelIndex&, int, int, const QModelIndex&, int) {
                refreshPointCloudWorkspaceState();
            }, Qt::UniqueConnection);
    connect(pointCloudListModel, &QAbstractItemModel::rowsInserted, this,
            [this](const QModelIndex&, int, int) {
                refreshPointCloudWorkspaceState();
            }, Qt::UniqueConnection);

    // Wire up reorder buttons (defined in mainwindow.ui)
    auto moveCurrentRow = [this](int delta) {
        if(!pointCloudListModel || !ui->listWidgetPointCloudFiles) return;
        const QModelIndex cur = ui->listWidgetPointCloudFiles->currentIndex();
        if(!cur.isValid()) return;
        const int row = cur.row();
        const int target = row + delta;
        if(target < 0 || target >= pointCloudListModel->rowCount()) return;
        QList<QStandardItem*> items = pointCloudListModel->takeRow(row);
        pointCloudListModel->insertRow(target, items);
        ui->listWidgetPointCloudFiles->setCurrentIndex(pointCloudListModel->index(target, 0));
    };
    if(ui->btnListUp)
        connect(ui->btnListUp, &QPushButton::clicked, this, [this, moveCurrentRow]() {
            moveCurrentRow(-1);
            refreshPointCloudWorkspaceState();
        });
    if(ui->btnListDown)
        connect(ui->btnListDown, &QPushButton::clicked, this, [this, moveCurrentRow]() {
            moveCurrentRow(+1);
            refreshPointCloudWorkspaceState();
        });

    if(ui->listWidgetPointCloudFiles->selectionModel())
    {
        connect(ui->listWidgetPointCloudFiles->selectionModel(),
                &QItemSelectionModel::currentChanged,
                this,
                &MainWindow::onPointCloudListCurrentChanged,
                Qt::UniqueConnection);
        connect(ui->listWidgetPointCloudFiles->selectionModel(),
                &QItemSelectionModel::selectionChanged,
                this,
                &MainWindow::onPointCloudListSelectionChanged,
                Qt::UniqueConnection);
    }

    connect(ui->listWidgetPointCloudFiles,
            &QListView::customContextMenuRequested,
            this,
            &MainWindow::showPointCloudListContextMenu,
            Qt::UniqueConnection);

    // 单击分组标题行 => 折叠/展开该分组；拖入/拖出分组 => 归组/移出
    if(auto* pcView = qobject_cast<PointCloudListView*>(ui->listWidgetPointCloudFiles))
    {
        connect(pcView, &PointCloudListView::groupHeaderClicked,
                this, &MainWindow::togglePointCloudGroupCollapsed,
                Qt::UniqueConnection);
        connect(pcView, &PointCloudListView::filesDroppedIntoGroup,
                this, &MainWindow::assignSelectedItemsToGroup,
                Qt::UniqueConnection);
        connect(pcView, &PointCloudListView::filesDroppedOutOfGroup,
                this, &MainWindow::removeSelectedItemsFromGroup,
                Qt::UniqueConnection);
    }

    if(ui->DicGLWidget)
    {
        connect(ui->DicGLWidget,
                &DICQGLViewer::lineSelected,
                this,
                &MainWindow::onDicLineSelected,
                Qt::UniqueConnection);
    }

    QShortcut *deleteShortcut = new QShortcut(QKeySequence::Delete, ui->listWidgetPointCloudFiles);
    deleteShortcut->setContext(Qt::WidgetWithChildrenShortcut);
    connect(deleteShortcut, &QShortcut::activated,
            this, &MainWindow::deleteSelectedPointCloud,
            Qt::UniqueConnection);

    QShortcut *saveShortcut = new QShortcut(QKeySequence::Save, ui->listWidgetPointCloudFiles);
    saveShortcut->setContext(Qt::WidgetWithChildrenShortcut);
    connect(saveShortcut, &QShortcut::activated,
            this, &MainWindow::saveSelectedPointCloudEntries,
            Qt::UniqueConnection);

    if(ui->comboBox_colorselect)
    {
        if(ui->comboBox_colorselect->count() == 0)
        {
            ui->comboBox_colorselect->addItem(QStringLiteral("Red"), QColor(255, 80, 80));
            ui->comboBox_colorselect->addItem(QStringLiteral("Green"), QColor(56, 209, 145));
            ui->comboBox_colorselect->addItem(QStringLiteral("Blue"), QColor(80, 150, 255));
            ui->comboBox_colorselect->addItem(QStringLiteral("Yellow"), QColor(242, 230, 92));
            ui->comboBox_colorselect->addItem(QStringLiteral("Purple"), QColor(199, 143, 255));
            ui->comboBox_colorselect->addItem(QStringLiteral("White"), QColor(255, 255, 255));
        }
        else
        {
            // Do not rely on non-ASCII item text. Existing UI items are mapped by index.
            for(int i = 0; i < ui->comboBox_colorselect->count(); ++i)
                ui->comboBox_colorselect->setItemData(i, colorFromPointCloudColorCombo(i), Qt::UserRole);
        }

        ui->comboBox_colorselect->setEnabled(false);
        connect(ui->comboBox_colorselect,
                QOverload<int>::of(&QComboBox::currentIndexChanged),
                this,
                &MainWindow::on_comboBox_colorselect_currentIndexChanged,
                Qt::UniqueConnection);

    }
}

void MainWindow::addPointCloudFileListItem(const QString& filePath,
                                           const QString& customDisplayName,
                                           int entryType,
                                           int meshIndex,
                                           int pointCloudIndex,
                                           int lineIndex)
{
    if(!pointCloudListModel)
        initPointCloudFileListUi();

    if(!ui || !ui->listWidgetPointCloudFiles || !pointCloudListModel)
        return;

    QFileInfo fileInfo(filePath);
    const QString displayName = customDisplayName.trimmed().isEmpty()
                                ? (fileInfo.fileName().isEmpty() ? filePath : fileInfo.fileName())
                                : customDisplayName.trimmed();

    if(pointCloudIndex < 0 && entryType == static_cast<int>(PointCloudEntryType::PointCloud) && ui->DicGLWidget)
        pointCloudIndex = ui->DicGLWidget->pointCloudCount() - 1;

    QStandardItem *item = new QStandardItem(displayName);
    item->setEditable(false);
    // 可拖拽、可选中，但不作为放置目标（禁止 ItemIsDropEnabled），
    // 使拖拽只在行与行之间移动，保持列表扁平不嵌套
    item->setFlags((item->flags() | Qt::ItemIsDragEnabled) & ~Qt::ItemIsDropEnabled);
    item->setToolTip(filePath);
    item->setData(filePath, Qt::UserRole);
    item->setData(entryType, RoleEntryType);
    item->setData(meshIndex, RoleMeshIndex);
    item->setData(pointCloudIndex, RolePointCloudIndex);
    item->setData(lineIndex, RoleLineIndex);
    item->setData(static_cast<qulonglong>(nextListItemId++), RoleUniqueId);
    pointCloudListModel->appendRow(item);
    pointCloudFilePaths.append(filePath);

    const QModelIndex newIndex = pointCloudListModel->index(pointCloudListModel->rowCount() - 1, 0);
    ui->listWidgetPointCloudFiles->setCurrentIndex(newIndex);
    if(ui->DicGLWidget)
    {
        if(entryType == static_cast<int>(PointCloudEntryType::PointCloud))
            ui->DicGLWidget->setSelectedPointCloudIndex(pointCloudIndex);
        else if(entryType == static_cast<int>(PointCloudEntryType::FittedAxisLine))
            ui->DicGLWidget->setSelectedLineIndex(lineIndex);
        else
            ui->DicGLWidget->setSelectedPointCloudIndex(-1);
    }
    updatePointCloudColorComboToCurrentCloud();
}

void MainWindow::onPointCloudListCurrentChanged(const QModelIndex& current, const QModelIndex& previous)
{
    Q_UNUSED(previous)

    if(!ui || !ui->DicGLWidget || !pointCloudListModel)
        return;

    int pointCloudIndex = -1;
    int lineIndex = -1;
    if(current.isValid())
    {
        if(const QStandardItem* item = pointCloudListModel->item(current.row()))
        {
            const int entryType = item->data(RoleEntryType).toInt();
            if(entryType == static_cast<int>(PointCloudEntryType::PointCloud))
                pointCloudIndex = item->data(RolePointCloudIndex).toInt();
            else if(entryType == static_cast<int>(PointCloudEntryType::FittedAxisLine))
                lineIndex = item->data(RoleLineIndex).toInt();
            else if(entryType == static_cast<int>(PointCloudEntryType::FittedGeometry) ||
                    entryType == static_cast<int>(PointCloudEntryType::Reconstruction))
            {
                const int ci = item->data(RolePointCloudIndex).toInt();
                if(ci >= 0)
                    pointCloudIndex = ci;
            }
        }
    }

    ui->DicGLWidget->setSelectedPointCloudIndex(pointCloudIndex);
    ui->DicGLWidget->setSelectedLineIndex(lineIndex);
    updatePointCloudColorComboToCurrentCloud();
    refreshPointCloudWorkspaceState();
}

void MainWindow::onPointCloudListSelectionChanged(const QItemSelection& selected, const QItemSelection& deselected)
{
    Q_UNUSED(selected)
    Q_UNUSED(deselected)

    if(!ui || !ui->DicGLWidget || !ui->listWidgetPointCloudFiles || !ui->listWidgetPointCloudFiles->selectionModel())
        return;

    const QModelIndexList rows = ui->listWidgetPointCloudFiles->selectionModel()->selectedRows();
    std::vector<int> cloudIndices;
    int firstLineIndex = -1;
    for(const QModelIndex& idx : rows)
    {
        if(!pointCloudListModel)
            continue;
        if(const QStandardItem* item = pointCloudListModel->item(idx.row()))
        {
            const int entryType = item->data(RoleEntryType).toInt();
            const int ci = item->data(RolePointCloudIndex).toInt();
            if(ci >= 0 && entryType == static_cast<int>(PointCloudEntryType::PointCloud))
                cloudIndices.push_back(ci);
            if(ci >= 0 && (entryType == static_cast<int>(PointCloudEntryType::FittedGeometry) ||
                           entryType == static_cast<int>(PointCloudEntryType::Reconstruction)))
                cloudIndices.push_back(ci);
            if(firstLineIndex < 0 && entryType == static_cast<int>(PointCloudEntryType::FittedAxisLine))
                firstLineIndex = item->data(RoleLineIndex).toInt();
        }
    }

    ui->DicGLWidget->setSelectedPointCloudIndices(cloudIndices);
    ui->DicGLWidget->setSelectedLineIndex(firstLineIndex);
    refreshPointCloudWorkspaceState();
}

void MainWindow::onDicLineSelected(int lineIndex)
{
    if(!ui || !ui->listWidgetPointCloudFiles || !pointCloudListModel || lineIndex < 0)
        return;

    for(int row = 0; row < pointCloudListModel->rowCount(); ++row)
    {
        const QStandardItem* item = pointCloudListModel->item(row);
        if(!item)
            continue;

        const int entryType = item->data(RoleEntryType).toInt();
        if(entryType != static_cast<int>(PointCloudEntryType::FittedAxisLine))
            continue;

        if(item->data(RoleLineIndex).toInt() == lineIndex)
        {
            const QModelIndex target = pointCloudListModel->index(row, 0);
            ui->listWidgetPointCloudFiles->setCurrentIndex(target);
            if(ui->listWidgetPointCloudFiles->selectionModel())
            {
                ui->listWidgetPointCloudFiles->selectionModel()->setCurrentIndex(
                    target, QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);
            }
            break;
        }
    }
}

void MainWindow::showPointCloudListContextMenu(const QPoint& pos)
{
    if(!ui || !ui->listWidgetPointCloudFiles || !pointCloudListModel)
        return;

    const QModelIndex index = ui->listWidgetPointCloudFiles->indexAt(pos);

    QItemSelectionModel* selModel = ui->listWidgetPointCloudFiles->selectionModel();
    const QModelIndexList selectedRows = selModel ? selModel->selectedRows() : QModelIndexList();

    // 右键分组标题行：提供折叠/展开、删除分组（保留数据）。
    if(index.isValid() &&
       pointCloudListModel->item(index.row()) &&
       pointCloudListModel->item(index.row())->data(RoleEntryType).toInt() == static_cast<int>(PointCloudEntryType::Group))
    {
        QStandardItem* headerItem = pointCloudListModel->item(index.row());
        const bool collapsed = headerItem->data(RoleGroupCollapsed).toBool();

        QMenu groupMenu(this);
        QAction* toggleAction = groupMenu.addAction(collapsed ? tr("Expand group") : tr("Collapse group"));
        groupMenu.addSeparator();
        QAction* ungroupAction = groupMenu.addAction(tr("Delete group (keep items)"));
        QAction* chosen = groupMenu.exec(ui->listWidgetPointCloudFiles->viewport()->mapToGlobal(pos));
        if(chosen == toggleAction)
            togglePointCloudGroupCollapsed(index);
        else if(chosen == ungroupAction)
            deletePointCloudGroupKeepItems(index.row());
        return;
    }

    // 右键空白处、且没有选中项：只提供「新建分组」（创建空分组）。
    if(!index.isValid() && selectedRows.isEmpty())
    {
        QMenu emptyMenu(this);
        QAction* groupOnly = emptyMenu.addAction(tr("Create group..."));
        QAction* chosen = emptyMenu.exec(ui->listWidgetPointCloudFiles->viewport()->mapToGlobal(pos));
        if(chosen == groupOnly)
            createPointCloudGroup();
        return;
    }

    if(!index.isValid())
        return;
    if(selModel && selectedRows.isEmpty())
        return;

    QMenu menu(this);

    bool hasVisible = false;
    bool hasHidden = false;
    for(const QModelIndex& rowIndex : selectedRows)
    {
        if(!ui->DicGLWidget || !pointCloudListModel)
            break;

        const QStandardItem* item = pointCloudListModel->item(rowIndex.row());
        const int entryType = item ? item->data(RoleEntryType).toInt() : static_cast<int>(PointCloudEntryType::PointCloud);
        const int cloudIndex = item ? item->data(RolePointCloudIndex).toInt() : -1;
        const int meshIndex = item ? item->data(RoleMeshIndex).toInt() : -1;
        const int lineIndex = item ? item->data(RoleLineIndex).toInt() : -1;

        bool isVisible = false;
        if(entryType == static_cast<int>(PointCloudEntryType::PointCloud))
            isVisible = (cloudIndex >= 0) ? ui->DicGLWidget->pointCloudVisible(cloudIndex) : false;
        else if(entryType == static_cast<int>(PointCloudEntryType::FittedAxisLine))
            isVisible = (lineIndex >= 0) ? ui->DicGLWidget->lineVisible(lineIndex) : false;
        else
            isVisible = (meshIndex >= 0) ? ui->DicGLWidget->meshVisible(meshIndex) : false;

        if(isVisible)
            hasVisible = true;
        else
            hasHidden = true;
    }

    QAction *visibilityAction = nullptr;
    if(hasVisible)
        visibilityAction = menu.addAction(tr("Hide selected"));
    else if(hasHidden)
        visibilityAction = menu.addAction(tr("Show selected"));

    if(visibilityAction)
        menu.addSeparator();

    QAction *saveAction = menu.addAction(tr("Save selected..."));
    QAction *deleteAction = menu.addAction(tr("Delete selected point cloud(s)"));
    menu.addSeparator();
    QAction *groupAction = menu.addAction(tr("Create group..."));

    // 收集已存在的分组名，构建「移动到分组」子菜单
    QStringList existingGroups;
    for(int r = 0; r < pointCloudListModel->rowCount(); ++r)
    {
        QStandardItem* it = pointCloudListModel->item(r);
        if(it && it->data(RoleEntryType).toInt() == static_cast<int>(PointCloudEntryType::Group))
        {
            const QString gname = it->data(RoleGroupName).toString();
            if(!gname.isEmpty() && !existingGroups.contains(gname))
                existingGroups.append(gname);
        }
    }

    QHash<QAction*, QString> moveToGroupActions;
    if(!existingGroups.isEmpty())
    {
        QMenu* moveMenu = menu.addMenu(tr("Move to group"));
        for(const QString& gname : existingGroups)
        {
            QAction* a = moveMenu->addAction(gname);
            moveToGroupActions.insert(a, gname);
        }
    }

    // 若选中项中存在已归组的项，提供「移出分组」
    bool anyGrouped = false;
    for(const QModelIndex& rowIndex : selectedRows)
    {
        QStandardItem* it = pointCloudListModel->item(rowIndex.row());
        if(it && !it->data(RoleGroupName).toString().isEmpty())
        {
            anyGrouped = true;
            break;
        }
    }
    QAction* removeFromGroupAction = anyGrouped ? menu.addAction(tr("Remove from group")) : nullptr;

    QAction *selectedAction = menu.exec(ui->listWidgetPointCloudFiles->viewport()->mapToGlobal(pos));
    if(selectedAction == visibilityAction)
        toggleSelectedPointCloudVisibility();
    else if(selectedAction == saveAction)
        saveSelectedPointCloudEntries();
    else if(selectedAction == deleteAction)
        deleteSelectedPointCloud();
    else if(selectedAction == groupAction)
        createPointCloudGroup();
    else if(selectedAction == removeFromGroupAction)
        removeSelectedItemsFromGroup();
    else if(selectedAction && moveToGroupActions.contains(selectedAction))
        assignSelectedItemsToGroup(moveToGroupActions.value(selectedAction));
}

void MainWindow::toggleSelectedPointCloudVisibility()
{
    if(!ui || !ui->DicGLWidget || !ui->listWidgetPointCloudFiles)
        return;

    QItemSelectionModel* selModel = ui->listWidgetPointCloudFiles->selectionModel();
    if(!selModel)
        return;

    const QModelIndexList selectedRows = selModel->selectedRows();
    if(selectedRows.isEmpty())
        return;

    bool hasVisible = false;
    bool hasHidden = false;
    for(const QModelIndex& rowIndex : selectedRows)
    {
        if(!pointCloudListModel)
            continue;

        const QStandardItem* item = pointCloudListModel->item(rowIndex.row());
        const int entryType = item ? item->data(RoleEntryType).toInt() : static_cast<int>(PointCloudEntryType::PointCloud);
        const int cloudIndex = item ? item->data(RolePointCloudIndex).toInt() : -1;
        const int meshIndex = item ? item->data(RoleMeshIndex).toInt() : -1;
        const int lineIndex = item ? item->data(RoleLineIndex).toInt() : -1;

        bool isVisible = false;
        if(entryType == static_cast<int>(PointCloudEntryType::PointCloud))
            isVisible = (cloudIndex >= 0) ? ui->DicGLWidget->pointCloudVisible(cloudIndex) : false;
        else if(entryType == static_cast<int>(PointCloudEntryType::FittedAxisLine))
            isVisible = (lineIndex >= 0) ? ui->DicGLWidget->lineVisible(lineIndex) : false;
        else
            isVisible = (meshIndex >= 0) ? ui->DicGLWidget->meshVisible(meshIndex) : false;

        if(isVisible)
            hasVisible = true;
        else
            hasHidden = true;
    }

    if(hasVisible)
        setSelectedPointCloudVisibility(false);
    else if(hasHidden)
        setSelectedPointCloudVisibility(true);
}

void MainWindow::setSelectedPointCloudVisibility(bool visible)
{
    if(!ui || !ui->DicGLWidget || !ui->listWidgetPointCloudFiles || !pointCloudListModel)
        return;

    QItemSelectionModel* selModel = ui->listWidgetPointCloudFiles->selectionModel();
    if(!selModel)
        return;

    const QModelIndexList selectedRows = selModel->selectedRows();
    if(selectedRows.isEmpty())
        return;

    int changedCount = 0;
    for(const QModelIndex& rowIndex : selectedRows)
    {
        const int row = rowIndex.row();
        if(row < 0 || row >= pointCloudListModel->rowCount())
            continue;

        QStandardItem *item = pointCloudListModel->item(row);
        if(!item)
            continue;

        const int entryType = item->data(RoleEntryType).toInt();
        const int cloudIndex = item->data(RolePointCloudIndex).toInt();
        const int meshIndex = item->data(RoleMeshIndex).toInt();
        const int lineIndex = item->data(RoleLineIndex).toInt();

        bool currentVisible = false;
        bool validTarget = false;
        if(entryType == static_cast<int>(PointCloudEntryType::PointCloud))
        {
            validTarget = (cloudIndex >= 0);
            if(validTarget)
                currentVisible = ui->DicGLWidget->pointCloudVisible(cloudIndex);
        }
        else if(entryType == static_cast<int>(PointCloudEntryType::FittedAxisLine))
        {
            validTarget = (lineIndex >= 0);
            if(validTarget)
                currentVisible = ui->DicGLWidget->lineVisible(lineIndex);
        }
        else
        {
            validTarget = (meshIndex >= 0);
            if(validTarget)
                currentVisible = ui->DicGLWidget->meshVisible(meshIndex);
        }

        if(!validTarget || currentVisible == visible)
            continue;

        if(entryType == static_cast<int>(PointCloudEntryType::PointCloud))
            ui->DicGLWidget->setPointCloudVisible(cloudIndex, visible);
        else if(entryType == static_cast<int>(PointCloudEntryType::FittedAxisLine))
            ui->DicGLWidget->setLineVisible(lineIndex, visible);
        else
            ui->DicGLWidget->setMeshVisible(meshIndex, visible);

        QFont f = item->font();
        f.setStrikeOut(!visible);
        item->setFont(f);
        item->setForeground(visible ? QColor(15, 25, 45) : QColor(140, 140, 140));
        ++changedCount;
    }

    if(ui->plainTextEdit)
        ui->plainTextEdit->appendPlainText(systemTime() + " " +
                                           (visible ? tr("Show point cloud count: ") : tr("Hide point cloud count: ")) +
                                           QString::number(changedCount));
}

void MainWindow::deleteSelectedPointCloud()
{
    if(!ui || !ui->DicGLWidget || !ui->listWidgetPointCloudFiles || !pointCloudListModel)
        return;

    QItemSelectionModel* selModel = ui->listWidgetPointCloudFiles->selectionModel();
    if(!selModel)
        return;

    QModelIndexList selectedRows = selModel->selectedRows();
    if(selectedRows.isEmpty())
    {
        const QModelIndex currentIndex = ui->listWidgetPointCloudFiles->currentIndex();
        if(currentIndex.isValid())
            selectedRows << currentIndex;
    }

    if(selectedRows.isEmpty())
        return;

    std::sort(selectedRows.begin(), selectedRows.end(), [](const QModelIndex& a, const QModelIndex& b){
        return a.row() > b.row();
    });

    int deletedCount = 0;
    for(const QModelIndex& index : selectedRows)
    {
        const int row = index.row();
        if(row < 0 || row >= pointCloudListModel->rowCount())
            continue;

        const QStandardItem* item = pointCloudListModel->item(row);
        const int entryType = item ? item->data(RoleEntryType).toInt() : static_cast<int>(PointCloudEntryType::PointCloud);

        // 分组项只删除项本身，不处理数据
        if(entryType == static_cast<int>(PointCloudEntryType::Group))
        {
            pointCloudListModel->removeRow(row);
            ++deletedCount;
            continue;
        }

        const int meshIndex = item ? item->data(RoleMeshIndex).toInt() : -1;
        const int pointCloudIndex = item ? item->data(RolePointCloudIndex).toInt() : -1;
        const int lineIndex = item ? item->data(RoleLineIndex).toInt() : -1;

        if(entryType == static_cast<int>(PointCloudEntryType::PointCloud))
        {
            if(pointCloudIndex >= 0)
            {
                ui->DicGLWidget->removePointCloud(pointCloudIndex);
                relinkPointCloudIndicesAfterRemoval(pointCloudIndex);
            }
        }
        else if(entryType == static_cast<int>(PointCloudEntryType::FittedAxisLine))
        {
            if(lineIndex >= 0 && ui->DicGLWidget->removeLine(lineIndex))
                relinkLineIndicesAfterRemoval(lineIndex);
        }
        else
        {
            if(meshIndex >= 0 && ui->DicGLWidget->removeMesh(meshIndex))
            {
                relinkReconstructionMeshIndicesAfterRemoval(meshIndex);
                relinkFittedGeometryMeshIndicesAfterRemoval(meshIndex);
            }
        }

        // 移除该行对应的文件路径记录
        if(item)
        {
            const QString rowPath = item->data(Qt::UserRole).toString();
            const int pathPos = pointCloudFilePaths.indexOf(rowPath);
            if(pathPos >= 0)
                pointCloudFilePaths.removeAt(pathPos);
        }

        pointCloudListModel->removeRow(row);
        ++deletedCount;
    }

    // 折叠状态与行隐藏可能因删除而变化，刷新分组视图
    refreshPointCloudGroupView();

    // 选中第一行可见的数据项（跳过分组标题行）
    int firstDataRow = -1;
    for(int r = 0; r < pointCloudListModel->rowCount(); ++r)
    {
        QStandardItem* it = pointCloudListModel->item(r);
        if(it && it->data(RoleEntryType).toInt() != static_cast<int>(PointCloudEntryType::Group))
        {
            firstDataRow = r;
            break;
        }
    }

    if(firstDataRow >= 0)
        ui->listWidgetPointCloudFiles->setCurrentIndex(pointCloudListModel->index(firstDataRow, 0));
    else
    {
        ui->DicGLWidget->setSelectedPointCloudIndex(-1);
        ui->DicGLWidget->setSelectedLineIndex(-1);
    }

    updatePointCloudColorComboToCurrentCloud();

    if(ui->plainTextEdit)
        ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Deleted point cloud count: ") + QString::number(deletedCount));
}

void MainWindow::createPointCloudGroup()
{
    if(!ui || !ui->listWidgetPointCloudFiles || !pointCloudListModel)
        return;

    // 弹出对话框获取分组名称
    bool ok = false;
    QString groupName = QInputDialog::getText(this,
                                              tr("Create Group"),
                                              tr("Enter group name:"),
                                              QLineEdit::Normal,
                                              QString(),
                                              &ok);
    if(!ok || groupName.trimmed().isEmpty())
        return;

    groupName = groupName.trimmed();
    onPointCloudGroupCreated(groupName);
}

void MainWindow::onPointCloudGroupCreated(const QString& groupName)
{
    if(!pointCloudListModel || groupName.isEmpty() || !ui || !ui->listWidgetPointCloudFiles)
        return;

    // 收集选中项的唯一 id（用于按 id 稳定处理，避免行号在重排中失效）
    QList<quint64> memberIds;
    if(ui->listWidgetPointCloudFiles->selectionModel())
    {
        const QModelIndexList selectedRows = ui->listWidgetPointCloudFiles->selectionModel()->selectedRows();
        for(const QModelIndex& idx : selectedRows)
        {
            QStandardItem* item = pointCloudListModel->item(idx.row());
            if(!item)
                continue;
            // 不把分组标题行归入分组
            if(item->data(RoleEntryType).toInt() == static_cast<int>(PointCloudEntryType::Group))
                continue;
            memberIds.append(item->data(RoleUniqueId).toULongLong());
        }
    }

    // 创建分组标题行：不可选、不可拖、不可作为放置目标。
    QStandardItem* groupItem = new QStandardItem();
    groupItem->setData(static_cast<int>(PointCloudEntryType::Group), RoleEntryType);
    groupItem->setData(groupName, RoleGroupName);
    groupItem->setData(false, RoleGroupCollapsed);
    groupItem->setData(static_cast<qulonglong>(nextListItemId++), RoleUniqueId);
    groupItem->setEditable(false);
    groupItem->setFlags((groupItem->flags() | Qt::ItemIsEnabled)
                        & ~Qt::ItemIsSelectable
                        & ~Qt::ItemIsDragEnabled
                        & ~Qt::ItemIsDropEnabled);
    groupItem->setBackground(QColor(226, 232, 240));
    groupItem->setForeground(QColor(30, 41, 59));
    QFont hf = groupItem->font();
    hf.setBold(true);
    groupItem->setFont(hf);
    pointCloudListModel->appendRow(groupItem);

    // 为选中项标记所属分组，并把它们重排到分组标题行之后，保持视觉连续。
    if(!memberIds.isEmpty())
    {
        int insertPos = pointCloudListModel->rowCount();  // 标题行的下一行位置（当前在末尾）
        for(quint64 id : memberIds)
        {
            // 定位该 id 当前所在行
            int srcRow = -1;
            for(int r = 0; r < pointCloudListModel->rowCount(); ++r)
            {
                QStandardItem* it = pointCloudListModel->item(r);
                if(it && it->data(RoleUniqueId).toULongLong() == id)
                {
                    srcRow = r;
                    break;
                }
            }
            if(srcRow < 0)
                continue;

            QList<QStandardItem*> rowItems = pointCloudListModel->takeRow(srcRow);
            if(rowItems.isEmpty())
                continue;
            rowItems.first()->setData(groupName, RoleGroupName);

            // takeRow 后行数减一，重新定位标题行以在其后插入
            int headerRow = -1;
            for(int r = 0; r < pointCloudListModel->rowCount(); ++r)
            {
                QStandardItem* it = pointCloudListModel->item(r);
                if(it && it->data(RoleEntryType).toInt() == static_cast<int>(PointCloudEntryType::Group) &&
                   it->data(RoleGroupName).toString() == groupName)
                {
                    headerRow = r;
                    break;
                }
            }
            insertPos = (headerRow >= 0) ? headerRow + 1 : pointCloudListModel->rowCount();
            pointCloudListModel->insertRow(insertPos, rowItems);
        }
    }

    refreshPointCloudGroupView();
    refreshPointCloudWorkspaceState();

    if(ui->plainTextEdit)
    {
        QString logMsg = systemTime() + " " + tr("Created group: ") + groupName;
        if(!memberIds.isEmpty())
            logMsg += " (" + QString::number(memberIds.count()) + tr(" item(s)") + ")";
        ui->plainTextEdit->appendPlainText(logMsg);
    }
}

void MainWindow::togglePointCloudGroupCollapsed(const QModelIndex& index)
{
    if(!pointCloudListModel || !index.isValid())
        return;

    QStandardItem* headerItem = pointCloudListModel->item(index.row());
    if(!headerItem || headerItem->data(RoleEntryType).toInt() != static_cast<int>(PointCloudEntryType::Group))
        return;

    const bool collapsed = headerItem->data(RoleGroupCollapsed).toBool();
    headerItem->setData(!collapsed, RoleGroupCollapsed);
    refreshPointCloudGroupView();
}

void MainWindow::refreshPointCloudGroupView()
{
    if(!pointCloudListModel || !ui || !ui->listWidgetPointCloudFiles)
        return;

    // 记录每个分组名当前的折叠状态
    QHash<QString, bool> collapsedByGroup;
    for(int r = 0; r < pointCloudListModel->rowCount(); ++r)
    {
        QStandardItem* it = pointCloudListModel->item(r);
        if(it && it->data(RoleEntryType).toInt() == static_cast<int>(PointCloudEntryType::Group))
        {
            const QString gname = it->data(RoleGroupName).toString();
            const bool collapsed = it->data(RoleGroupCollapsed).toBool();
            collapsedByGroup.insert(gname, collapsed);

            // 更新标题显示：▼ 展开 / ▶ 折叠
            it->setText((collapsed ? QStringLiteral("▶  ") : QStringLiteral("▼  ")) + gname);
        }
    }

    // 依据成员所属分组的折叠状态设置行隐藏
    for(int r = 0; r < pointCloudListModel->rowCount(); ++r)
    {
        QStandardItem* it = pointCloudListModel->item(r);
        if(!it)
            continue;
        if(it->data(RoleEntryType).toInt() == static_cast<int>(PointCloudEntryType::Group))
        {
            ui->listWidgetPointCloudFiles->setRowHidden(r, false);  // 标题行始终显示
            continue;
        }
        const QString gname = it->data(RoleGroupName).toString();
        const bool hide = !gname.isEmpty() && collapsedByGroup.value(gname, false);
        ui->listWidgetPointCloudFiles->setRowHidden(r, hide);
    }
}

void MainWindow::deletePointCloudGroupKeepItems(int headerRow)
{
    if(!pointCloudListModel || headerRow < 0 || headerRow >= pointCloudListModel->rowCount())
        return;

    QStandardItem* headerItem = pointCloudListModel->item(headerRow);
    if(!headerItem || headerItem->data(RoleEntryType).toInt() != static_cast<int>(PointCloudEntryType::Group))
        return;

    const QString groupName = headerItem->data(RoleGroupName).toString();

    // 清除成员的分组标记（数据项保留在列表中）
    for(int r = 0; r < pointCloudListModel->rowCount(); ++r)
    {
        QStandardItem* it = pointCloudListModel->item(r);
        if(it && it->data(RoleEntryType).toInt() != static_cast<int>(PointCloudEntryType::Group) &&
           it->data(RoleGroupName).toString() == groupName)
        {
            it->setData(QString(), RoleGroupName);
            ui->listWidgetPointCloudFiles->setRowHidden(r, false);
        }
    }

    // 删除标题行本身
    pointCloudListModel->removeRow(headerRow);

    refreshPointCloudGroupView();
    refreshPointCloudWorkspaceState();

    if(ui->plainTextEdit)
        ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Deleted group: ") + groupName);
}

void MainWindow::assignSelectedItemsToGroup(const QString& groupName)
{
    if(!pointCloudListModel || groupName.isEmpty() || !ui || !ui->listWidgetPointCloudFiles)
        return;

    QItemSelectionModel* selModel = ui->listWidgetPointCloudFiles->selectionModel();
    if(!selModel)
        return;

    // 收集选中数据项的唯一 id（分组标题行不参与）
    QList<quint64> memberIds;
    for(const QModelIndex& idx : selModel->selectedRows())
    {
        QStandardItem* item = pointCloudListModel->item(idx.row());
        if(!item)
            continue;
        if(item->data(RoleEntryType).toInt() == static_cast<int>(PointCloudEntryType::Group))
            continue;
        memberIds.append(item->data(RoleUniqueId).toULongLong());
    }
    if(memberIds.isEmpty())
        return;

    // 逐个按 id 定位、打标记、重排到目标分组标题行之后
    for(quint64 id : memberIds)
    {
        int srcRow = -1;
        for(int r = 0; r < pointCloudListModel->rowCount(); ++r)
        {
            QStandardItem* it = pointCloudListModel->item(r);
            if(it && it->data(RoleUniqueId).toULongLong() == id)
            {
                srcRow = r;
                break;
            }
        }
        if(srcRow < 0)
            continue;

        QList<QStandardItem*> rowItems = pointCloudListModel->takeRow(srcRow);
        if(rowItems.isEmpty())
            continue;
        rowItems.first()->setData(groupName, RoleGroupName);

        int headerRow = -1;
        for(int r = 0; r < pointCloudListModel->rowCount(); ++r)
        {
            QStandardItem* it = pointCloudListModel->item(r);
            if(it && it->data(RoleEntryType).toInt() == static_cast<int>(PointCloudEntryType::Group) &&
               it->data(RoleGroupName).toString() == groupName)
            {
                headerRow = r;
                break;
            }
        }
        const int insertPos = (headerRow >= 0) ? headerRow + 1 : pointCloudListModel->rowCount();
        pointCloudListModel->insertRow(insertPos, rowItems);
    }

    refreshPointCloudGroupView();
    refreshPointCloudWorkspaceState();

    if(ui->plainTextEdit)
        ui->plainTextEdit->appendPlainText(systemTime() + " " +
                                           tr("Moved into group: ") + groupName +
                                           " (" + QString::number(memberIds.count()) + tr(" item(s)") + ")");
}

void MainWindow::removeSelectedItemsFromGroup()
{
    if(!pointCloudListModel || !ui || !ui->listWidgetPointCloudFiles)
        return;

    QItemSelectionModel* selModel = ui->listWidgetPointCloudFiles->selectionModel();
    if(!selModel)
        return;

    int cleared = 0;
    for(const QModelIndex& idx : selModel->selectedRows())
    {
        QStandardItem* item = pointCloudListModel->item(idx.row());
        if(!item)
            continue;
        if(item->data(RoleEntryType).toInt() == static_cast<int>(PointCloudEntryType::Group))
            continue;
        if(!item->data(RoleGroupName).toString().isEmpty())
        {
            item->setData(QString(), RoleGroupName);
            ++cleared;
        }
    }

    if(cleared > 0)
    {
        refreshPointCloudGroupView();
        refreshPointCloudWorkspaceState();
    }
}


void MainWindow::relinkReconstructionMeshIndicesAfterRemoval(int removedMeshIndex)
{
    if(!pointCloudListModel)
        return;

    for(int row = 0; row < pointCloudListModel->rowCount(); ++row)
    {
        QStandardItem* item = pointCloudListModel->item(row);
        if(!item)
            continue;

        const int entryType = item->data(RoleEntryType).toInt();
        if(entryType != static_cast<int>(PointCloudEntryType::Reconstruction))
            continue;

        int meshIndex = item->data(RoleMeshIndex).toInt();
        if(meshIndex > removedMeshIndex)
            item->setData(meshIndex - 1, RoleMeshIndex);
        else if(meshIndex == removedMeshIndex)
            item->setData(-1, RoleMeshIndex);
    }
}

void MainWindow::relinkFittedGeometryMeshIndicesAfterRemoval(int removedMeshIndex)
{
    if(!pointCloudListModel)
        return;

    for(int row = 0; row < pointCloudListModel->rowCount(); ++row)
    {
        QStandardItem* item = pointCloudListModel->item(row);
        if(!item)
            continue;

        const int entryType = item->data(RoleEntryType).toInt();
        if(entryType != static_cast<int>(PointCloudEntryType::FittedGeometry))
            continue;

        int meshIndex = item->data(RoleMeshIndex).toInt();
        if(meshIndex > removedMeshIndex)
            item->setData(meshIndex - 1, RoleMeshIndex);
        else if(meshIndex == removedMeshIndex)
            item->setData(-1, RoleMeshIndex);
    }

    // Clean up sphere fit data for removed/shifted meshes
    sphereFitResults.remove(removedMeshIndex);
    QHash<int, SphereFitData> shifted;
    for(auto it = sphereFitResults.begin(); it != sphereFitResults.end(); ++it)
    {
        if(it.key() > removedMeshIndex)
            shifted.insert(it.key() - 1, it.value());
        else
            shifted.insert(it.key(), it.value());
    }
    sphereFitResults = shifted;
}

void MainWindow::relinkPointCloudIndicesAfterRemoval(int removedPointCloudIndex)
{
    if(!pointCloudListModel)
        return;

    for(int row = 0; row < pointCloudListModel->rowCount(); ++row)
    {
        QStandardItem* item = pointCloudListModel->item(row);
        if(!item)
            continue;

        const int entryType = item->data(RoleEntryType).toInt();
        if(entryType != static_cast<int>(PointCloudEntryType::PointCloud))
            continue;

        int cloudIndex = item->data(RolePointCloudIndex).toInt();
        if(cloudIndex > removedPointCloudIndex)
            item->setData(cloudIndex - 1, RolePointCloudIndex);
        else if(cloudIndex == removedPointCloudIndex)
            item->setData(-1, RolePointCloudIndex);
    }
}


void MainWindow::relinkLineIndicesAfterRemoval(int removedLineIndex)
{
    if(!pointCloudListModel)
        return;

    for(int row = 0; row < pointCloudListModel->rowCount(); ++row)
    {
        QStandardItem* item = pointCloudListModel->item(row);
        if(!item)
            continue;

        const int entryType = item->data(RoleEntryType).toInt();
        if(entryType != static_cast<int>(PointCloudEntryType::FittedAxisLine))
            continue;

        int lineIndex = item->data(RoleLineIndex).toInt();
        if(lineIndex > removedLineIndex)
            item->setData(lineIndex - 1, RoleLineIndex);
        else if(lineIndex == removedLineIndex)
            item->setData(-1, RoleLineIndex);
    }
}

QColor MainWindow::colorFromPointCloudColorCombo(int index) const
{
    if(!ui || !ui->comboBox_colorselect || index < 0 || index >= ui->comboBox_colorselect->count())
        return QColor(92, 214, 255);

    const QVariant colorData = ui->comboBox_colorselect->itemData(index, Qt::UserRole);
    if(colorData.canConvert<QColor>())
        return colorData.value<QColor>();

    const QString text = ui->comboBox_colorselect->itemText(index).trimmed().toLower();
    if(text.contains(QStringLiteral("red")))
        return QColor(255, 80, 80);
    if(text.contains(QStringLiteral("green")))
        return QColor(56, 209, 145);
    if(text.contains(QStringLiteral("blue")))
        return QColor(80, 150, 255);
    if(text.contains(QStringLiteral("yellow")))
        return QColor(242, 230, 92);
    if(text.contains(QStringLiteral("purple")))
        return QColor(199, 143, 255);
    if(text.contains(QStringLiteral("white")))
        return QColor(255, 255, 255);
    if(text.contains(QStringLiteral("black")))
        return QColor(20, 20, 20);

    // Existing Designer items may still use localized text. In that case map colors by order.
    switch(index)
    {
    case 0:
        return QColor(255, 80, 80);
    case 1:
        return QColor(56, 209, 145);
    case 2:
        return QColor(80, 150, 255);
    case 3:
        return QColor(242, 230, 92);
    case 4:
        return QColor(199, 143, 255);
    case 5:
        return QColor(255, 255, 255);
    case 6:
        return QColor(20, 20, 20);
    default:
        return QColor(92, 214, 255);
    }
}

void MainWindow::updatePointCloudColorComboToCurrentCloud()
{
    if(!ui || !ui->comboBox_colorselect || !ui->listWidgetPointCloudFiles || !ui->DicGLWidget)
        return;

    const QModelIndex currentIndex = ui->listWidgetPointCloudFiles->currentIndex();
    const bool hasSelection = currentIndex.isValid();
    ui->comboBox_colorselect->setEnabled(hasSelection);
    if(!hasSelection)
        return;

    if(!pointCloudListModel)
        return;

    const QStandardItem* item = pointCloudListModel->item(currentIndex.row());
    const int entryType = item ? item->data(RoleEntryType).toInt() : static_cast<int>(PointCloudEntryType::PointCloud);
    const int pointCloudIndex = item ? item->data(RolePointCloudIndex).toInt() : -1;
    if(entryType != static_cast<int>(PointCloudEntryType::PointCloud) || pointCloudIndex < 0)
        return;

    const QColor currentColor = ui->DicGLWidget->pointCloudColor(pointCloudIndex);
    int matchedIndex = -1;
    for(int i = 0; i < ui->comboBox_colorselect->count(); ++i)
    {
        const QColor comboColor = colorFromPointCloudColorCombo(i);
        if(comboColor == currentColor)
        {
            matchedIndex = i;
            break;
        }
    }

    if(matchedIndex >= 0 && matchedIndex != ui->comboBox_colorselect->currentIndex())
    {
        QSignalBlocker blocker(ui->comboBox_colorselect);
        ui->comboBox_colorselect->setCurrentIndex(matchedIndex);
    }
}

void MainWindow::on_comboBox_colorselect_currentIndexChanged(int index)
{
    if(!ui || !ui->DicGLWidget || !ui->listWidgetPointCloudFiles)
        return;

    const QModelIndex currentIndex = ui->listWidgetPointCloudFiles->currentIndex();
    if(!currentIndex.isValid())
        return;

    if(!pointCloudListModel)
        return;

    const QStandardItem* item = pointCloudListModel->item(currentIndex.row());
    const int entryType = item ? item->data(RoleEntryType).toInt() : static_cast<int>(PointCloudEntryType::PointCloud);
    const int pointCloudIndex = item ? item->data(RolePointCloudIndex).toInt() : -1;
    if(entryType != static_cast<int>(PointCloudEntryType::PointCloud) || pointCloudIndex < 0)
        return;

    ui->DicGLWidget->setPointCloudColor(pointCloudIndex, colorFromPointCloudColorCombo(index));
}

void MainWindow::on_MoreColor_clicked()
{
    if(!ui || !ui->DicGLWidget || !ui->listWidgetPointCloudFiles)
        return;

    const QModelIndex currentIndex = ui->listWidgetPointCloudFiles->currentIndex();
    if(!currentIndex.isValid() || !pointCloudListModel)
        return;

    const QStandardItem* item = pointCloudListModel->item(currentIndex.row());
    const int entryType = item ? item->data(RoleEntryType).toInt() : static_cast<int>(PointCloudEntryType::PointCloud);
    const int pointCloudIndex = item ? item->data(RolePointCloudIndex).toInt() : -1;
    if(entryType != static_cast<int>(PointCloudEntryType::PointCloud) || pointCloudIndex < 0)
        return;

    QColor currentColor = ui->DicGLWidget->pointCloudColor(pointCloudIndex);

    MoreColorDialog dlg(this);
    dlg.setCurrentColor(currentColor);

    if(dlg.exec() == QDialog::Accepted) {
        QColor newColor = dlg.selectedColor();
        ui->DicGLWidget->setPointCloudColor(pointCloudIndex, newColor);

        // Save this color back to the combo for future reference
        if(ui->comboBox_colorselect) {
            int idx = ui->comboBox_colorselect->currentIndex();
            if(idx >= 0) {
                ui->comboBox_colorselect->setItemData(idx, newColor, Qt::UserRole);
            }
        }
    }
}

void MainWindow::initConfocalSensorThread()
{
    // 注册跨线程信号槽所需的元类型
    qRegisterMetaType<ConfocalStatusSnapshot>("ConfocalStatusSnapshot");
    qRegisterMetaType<ConfocalInitialConfig>("ConfocalInitialConfig");
    qRegisterMetaType<QVector<int>>("QVector<int>");

    if(!threadManager)
        initThreadManager();

    // 创建工作线程对象
    confocalWorker = new ConfocalSensorWorker;

    // 绑定信号槽
    connect(confocalWorker, &ConfocalSensorWorker::connectionChecked,
            this, [this](bool connected, int connectStatus) {
                if(!connected)
                {
                    ui->plainTextEdit->appendPlainText(
                        systemTime() + " " + tr("Device connection verification failed!") +
                        QStringLiteral(" (%1)").arg(connectStatus));
                    return;
                }

                ui->actionLink->setText(tr("鏂紑"));
                ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Connection successful!"));
                g_Global->g_isConnect = true;

                SingleChannelModeInit();
                getStatusBarResult();
            }, Qt::QueuedConnection);

    // 绑定信号槽
    connect(confocalWorker, &ConfocalSensorWorker::initialConfigReady,
            this, &MainWindow::applyConfocalInitialConfig,
            Qt::QueuedConnection);

    connect(confocalWorker, &ConfocalSensorWorker::statusSnapshotReady,
            this, &MainWindow::updateConfocalStatusSnapshot,
            Qt::QueuedConnection);

    // 绑定信号槽
    connect(confocalWorker, &ConfocalSensorWorker::sampleStateChanged,
            this, [this](bool started, int status) {
                if(status != Status_Succeed)
                {
                    ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Sensor operation failed!"));
                    return;
                }

                g_Global->g_isStart = started;
                if(started)
                {
                    confTimer->start();
                    StatusBarTimer->start();
                    ui->actionStart->setText(tr("鍋滄"));
                    ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Sensor sampling activated!"));
                }
                else
                {
                    confTimer->stop();
                    StatusBarTimer->stop();
                    ui->actionStart->setText(tr("鍚姩"));
                    ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Sensor sampling stops!"));
                }
            }, Qt::QueuedConnection);

    // 绑定信号槽
    connect(confocalWorker, &ConfocalSensorWorker::deviceClosed,
            this, [this]() {
                if(confTimer)
                    confTimer->stop();
                if(StatusBarTimer)
                    StatusBarTimer->stop();

                ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("The device is closed!"));
                ui->actionLink->setText(tr("杩炴帴"));
                g_Global->g_handle = 0;
                g_Global->g_isConnect = false;
                g_Global->g_isStart = false;

                statusBarInit();

                g_Global->g_isCF3000Lite = false;
                ui->groupBoxLightSource->setEnabled(true);
                ui->tabCommunicationMode->setEnabled(true);
                ui->comboBoxEncodeInputMode->setEnabled(true);
                ui->comboBoxEncodeWorkMode->setEnabled(true);
                ui->spinBoxEncodeDivision->setEnabled(true);
                ui->groupBox_autoExporsure->setEnabled(true);
            }, Qt::QueuedConnection);

    // 绑定信号槽
    connect(confocalWorker, &ConfocalSensorWorker::logMessage,
            this, [this](const QString& message) {
                ui->plainTextEdit->appendPlainText(systemTime() + " " + message);
            }, Qt::QueuedConnection);

    connect(confocalWorker, &ConfocalSensorWorker::operationFailed,
            this, [this](const QString& operation, int status) {
                ui->plainTextEdit->appendPlainText(
                    systemTime() + " " + tr("Sensor operation failed: ") + operation +
                    QStringLiteral(" (%1)").arg(status));
            }, Qt::QueuedConnection);

    connect(confocalWorker, &ConfocalSensorWorker::latestResultReady,
            this, [this](int channelIndex, float value) {
                Q_UNUSED(channelIndex)
                ui->plainTextEdit->appendPlainText(
                    systemTime() + " Current channel last result is: " +
                    QString::number(value, 'f', 5));
            }, Qt::QueuedConnection);

    // 绑定信号槽
    connect(confocalWorker, &ConfocalSensorWorker::channelResultReady,
            this, &MainWindow::updateConfocalMeasurementResult,
            Qt::QueuedConnection);

    connect(confocalWorker, &ConfocalSensorWorker::deviceDisconnected,
            this, [this](const QString& message) {
                QMessageBox::information(this, tr("data return"), tr("Device disconnection: ") + message);
            }, Qt::QueuedConnection);

    connect(confocalWorker, &ConfocalSensorWorker::apiException,
            this, [this](const QString& message) {
                ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("API call exception: ") + message);
            }, Qt::QueuedConnection);

    // 参数校验，无效时提前返回。
    if(!threadManager->startModule(ThreadManager::Module::ConfocalSensor,
                                   confocalWorker,
                                   QStringLiteral("ConfocalSensorThread"),
                                   "closeDevice"))
    {
        confocalWorker = nullptr;
    }
}

void MainWindow::initPlcThread()
{
    // 注册跨线程信号槽所需的元类型
    qRegisterMetaType<PlcDataSnapshot>("PlcDataSnapshot");

    if(!threadManager)
        initThreadManager();

    if(plcController)
        return;

    // 创建工作线程对象
    plcController = new Plc;

    // 绑定信号槽
    connect(plcController, &Plc::dataUpdated,
            this, &MainWindow::updatePlcSnapshot,
            Qt::QueuedConnection);

    // 绑定信号槽
    connect(plcController, &Plc::plcConnectionStateReady,
            this, [this](bool connected) {
                plcConnectedFlag = connected;
            }, Qt::QueuedConnection);

    // 绑定信号槽
    connect(plcController, &Plc::plcConnected,
            this, [this]() {
                plcConnectedFlag = true;

                if(mahrTimer && !mahrTimer->isActive())
                    mahrTimer->start(100);

                ui->plainTextEdit->appendPlainText(systemTime() + " PLC连接成功");
            }, Qt::QueuedConnection);

    connect(plcController, &Plc::plcDisconnected,
            this, [this]() {
                plcConnectedFlag = false;

                if(mahrTimer && mahrTimer->isActive())
                    mahrTimer->stop();

                ui->plainTextEdit->appendPlainText(systemTime() + " PLC已断开");
            }, Qt::QueuedConnection);

    connect(plcController, &Plc::plcError,
            this, [this](const QString &err) {
                plcConnectedFlag = false;

                if(mahrTimer && mahrTimer->isActive())
                    mahrTimer->stop();

                ui->plainTextEdit->appendPlainText(systemTime() + " PLC杩炴帴閿欒: " + err);
            }, Qt::QueuedConnection);

    // 参数校验，无效时提前返回。
    if(!threadManager->startModule(ThreadManager::Module::Plc,
                                   plcController,
                                   QStringLiteral("PlcThread"),
                                   "disconnectPlc"))
    {
        plcController = nullptr;
        plcConnectedFlag = false;
    }
}

void MainWindow::shutdownPlcThread()
{
    // 参数校验，无效时提前返回。
    if(mahrTimer)
        mahrTimer->stop();

    if(threadManager)
        threadManager->stopModule(ThreadManager::Module::Plc, 3000);

    plcController = nullptr;
    plcConnectedFlag = false;
}

void MainWindow::updatePlcSnapshot(const PlcDataSnapshot& snapshot)
{
    plcSnapshot = snapshot;
}

void MainWindow::postPlcConnect()
{
    if(!plcController)
        return;

    // 异步投递任务到工作线程
    QMetaObject::invokeMethod(plcController,
                              "connectPlc",
                              Qt::QueuedConnection);
}

void MainWindow::postPlcDisconnect()
{
    if(!plcController)
        return;

    // 异步投递任务到工作线程
    QMetaObject::invokeMethod(plcController,
                              "disconnectPlc",
                              Qt::QueuedConnection);
}

void MainWindow::postPlcStartCollectingData()
{
    if(!plcController)
        return;

    // 异步投递任务到工作线程
    QMetaObject::invokeMethod(plcController,
                              "startCollectingData",
                              Qt::QueuedConnection);
}

void MainWindow::postPlcStartContinuousMovement(double cylinder, double address)
{
    if(!plcController)
        return;

    // 异步投递任务到工作线程
    QMetaObject::invokeMethod(plcController,
                              "startContinuousMovement",
                              Qt::QueuedConnection,
                              Q_ARG(double, cylinder),
                              Q_ARG(double, address));
}

void MainWindow::postPlcMoveToAbsolutePosition(double cylinder, double address)
{
    if(!plcController)
        return;

    // 异步投递任务到工作线程
    QMetaObject::invokeMethod(plcController,
                              "moveToAbsolutePosition",
                              Qt::QueuedConnection,
                              Q_ARG(double, cylinder),
                              Q_ARG(double, address));
}

void MainWindow::postPlcWriteHoldingRegisters(double registers, double address)
{
    if(!plcController)
        return;

    // 异步投递任务到工作线程
    QMetaObject::invokeMethod(plcController,
                              "writeHoldingRegisters",
                              Qt::QueuedConnection,
                              Q_ARG(double, registers),
                              Q_ARG(double, address));
}

void MainWindow::postPlcWriteCoilData(double value, double address)
{
    if(!plcController)
        return;

    // 异步投递任务到工作线程
    QMetaObject::invokeMethod(plcController,
                              "writeCoilData",
                              Qt::QueuedConnection,
                              Q_ARG(double, value),
                              Q_ARG(double, address));
}

void MainWindow::shutdownModuleThreads()
{
    if(mahrTimer)
        mahrTimer->stop();
    if(confTimer)
        confTimer->stop();
    if(StatusBarTimer)
        StatusBarTimer->stop();

    const auto cameraWindows = display_widget_list;
    for(CamDisplay *cameraWindow : cameraWindows)
    {
        if(cameraWindow)
            cameraWindow->close();
    }
    display_widget_list.clear();
    display_widget = nullptr;

    if(threadManager)
        threadManager->stopAll(3000);

    confocalWorker = nullptr;
    plcController = nullptr;
    // DLP 直接在主线程创建，释放它（析构会断开 I2C/硬件连接）。
    if(dlp1)
    {
        delete dlp1;
        dlp1 = nullptr;
    }
    pointCloudWorker = nullptr;
    plcConnectedFlag = false;
    dlpInitialized = false;
    pointCloudLoadInProgress = false;

    if(g_Global)
    {
        g_Global->g_handle = 0;
        g_Global->g_isConnect = false;
        g_Global->g_isStart = false;
    }
}

void MainWindow::initFloatingMainTabs()
{
    if(!ui || !ui->Tab_Main || !ui->Tab_Main->tabBar())
        return;

    for(int i = 0; i < ui->Tab_Main->count(); ++i)
    {
        QWidget *page = ui->Tab_Main->widget(i);
        if(page && !page->property("MainTabOriginalIndex").isValid())
            page->setProperty("MainTabOriginalIndex", i);
    }

    ui->Tab_Main->tabBar()->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(ui->Tab_Main->tabBar(), &QTabBar::customContextMenuRequested,
            this, &MainWindow::showMainTabContextMenu);
}

void MainWindow::showMainTabContextMenu(const QPoint& pos)
{
    if(!ui || !ui->Tab_Main || !ui->Tab_Main->tabBar())
        return;

    QTabBar *tabBar = ui->Tab_Main->tabBar();
    const int index = tabBar->tabAt(pos);
    if(index < 0)
        return;

    QMenu menu(this);
    QAction *floatAction = menu.addAction(QString::fromLocal8Bit("push out"));
    QAction *selectedAction = menu.exec(tabBar->mapToGlobal(pos));
    if(selectedAction == floatAction)
        floatMainTab(index);
}

void MainWindow::floatMainTab(int index)
{
    if(!ui || !ui->Tab_Main || index < 0 || index >= ui->Tab_Main->count())
        return;

    QWidget *page = ui->Tab_Main->widget(index);
    if(!page || floatingMainTabs.contains(page))
        return;

    FloatingTabPage info;
    info.page = page;
    info.title = ui->Tab_Main->tabText(index);
    info.icon = ui->Tab_Main->tabIcon(index);
    info.originalIndex = page->property("MainTabOriginalIndex").isValid()
                             ? page->property("MainTabOriginalIndex").toInt()
                             : index;

    QDockWidget *dock = new QDockWidget(info.title, this);
    dock->setObjectName(QStringLiteral("FloatingMainTab_%1").arg(info.originalIndex));
    dock->setAllowedAreas(Qt::AllDockWidgetAreas);
    dock->setFeatures(QDockWidget::DockWidgetClosable |
                      QDockWidget::DockWidgetMovable |
                      QDockWidget::DockWidgetFloatable);
    dock->setContextMenuPolicy(Qt::CustomContextMenu);

    connect(dock, &QDockWidget::customContextMenuRequested,
            this, [this, page, dock](const QPoint& pos) {
                if(!floatingMainTabs.contains(page))
                    return;

                QMenu menu(dock);
                QAction *restoreAction = menu.addAction(QString::fromLocal8Bit("鍋滈潬鍥炰富鐣岄潰"));
                QAction *selectedAction = menu.exec(dock->mapToGlobal(pos));
                if(selectedAction == restoreAction)
                    restoreMainTab(page);
            });

    info.dock = dock;
    floatingMainTabs.insert(page, info);

    ui->Tab_Main->removeTab(index);
    dock->setWidget(page);
    addDockWidget(Qt::RightDockWidgetArea, dock);
    dock->setFloating(true);
    dock->resize(qMax(640, width() / 2), qMax(480, height() / 2));
    dock->show();
    dock->raise();

    // 绑定信号槽
    connect(dock, &QDockWidget::visibilityChanged,
            this, [this, page](bool visible) {
                if(!visible && floatingMainTabs.contains(page))
                    restoreMainTab(page);
            });
}

void MainWindow::restoreMainTab(QWidget *page)
{
    if(!page || !floatingMainTabs.contains(page) || !ui || !ui->Tab_Main)
        return;

    FloatingTabPage info = floatingMainTabs.take(page);
    QDockWidget *dock = info.dock;

    if(dock)
    {
        dock->blockSignals(true);
        if(dock->widget() == page)
            dock->setWidget(nullptr);
        removeDockWidget(dock);
    }

    const int insertIndex = tabInsertIndexForOriginalIndex(info.originalIndex);
    ui->Tab_Main->insertTab(insertIndex, page, info.icon, info.title);
    ui->Tab_Main->setCurrentWidget(page);

    if(dock)
        dock->deleteLater();
}

int MainWindow::tabInsertIndexForOriginalIndex(int originalIndex) const
{
    if(!ui || !ui->Tab_Main)
        return 0;

    for(int i = 0; i < ui->Tab_Main->count(); ++i)
    {
        QWidget *currentPage = ui->Tab_Main->widget(i);
        const int currentOriginalIndex = currentPage && currentPage->property("MainTabOriginalIndex").isValid()
                                             ? currentPage->property("MainTabOriginalIndex").toInt()
                                             : i;
        if(currentOriginalIndex > originalIndex)
            return i;
    }

    return ui->Tab_Main->count();
}

void MainWindow::shutdownConfocalSensorThread()
{
    // 参数校验，无效时提前返回。
    if(confTimer)
        confTimer->stop();
    if(StatusBarTimer)
        StatusBarTimer->stop();

    if(threadManager)
        threadManager->stopModule(ThreadManager::Module::ConfocalSensor, 3000);

    confocalWorker = nullptr;

    if(g_Global)
    {
        g_Global->g_handle = 0;
        g_Global->g_isConnect = false;
        g_Global->g_isStart = false;
    }
}

void MainWindow::postConfocalSetIntParam(const QString& paramName, int channelIndex, int value, const QString& successMessage)
{
    if(!confocalWorker)
        return;

    // 异步投递任务到工作线程
    QMetaObject::invokeMethod(confocalWorker,
                              "setIntParam",
                              Qt::QueuedConnection,
                              Q_ARG(QString, paramName),
                              Q_ARG(int, channelIndex),
                              Q_ARG(int, value),
                              Q_ARG(QString, successMessage));
}

void MainWindow::postConfocalSetFloatParam(const QString& paramName, int channelIndex, float value, const QString& successMessage)
{
    if(!confocalWorker)
        return;

    // 异步投递任务到工作线程
    QMetaObject::invokeMethod(confocalWorker,
                              "setFloatParam",
                              Qt::QueuedConnection,
                              Q_ARG(QString, paramName),
                              Q_ARG(int, channelIndex),
                              Q_ARG(float, value),
                              Q_ARG(QString, successMessage));
}

void MainWindow::applyConfocalInitialConfig(const ConfocalInitialConfig& config)
{
    g_Global->g_isCF3000Lite = config.isCF3000Lite;

    QSignalBlocker blockerSingleMode(ui->comboBoxSingleChannelMeasureModeSelect);
    QSignalBlocker blockerSingleAbs(ui->checkBoxSingleChannelABS);
    QSignalBlocker blockerSingleReverse(ui->checkBoxSingleChannelReverse);
    QSignalBlocker blockerSingleMultiDistance(ui->checkBoxSingleChannelMultiDistance);
    QSignalBlocker blockerExposureTime(ui->comboBoxExposureTime);
    QSignalBlocker blockerAutoExposureMode(ui->comboBox_3LAutoExporsureMode);
    QSignalBlocker blockerAutoExposureChannel(ui->comboBox_3LAutoExporsureChannel);
    QSignalBlocker blockerAutoExposureValue(ui->spinBox_3LAutoExporsureValue);
    QSignalBlocker blockerLightMode(ui->comboBoxLightMode);
    QSignalBlocker blockerLightReference(ui->comboBoxLightReference);
    QSignalBlocker blockerLightValue(ui->horizontalSliderLightValue);
    QSignalBlocker blockerSubBackground(ui->rb_subBackgroundLight);
    QSignalBlocker blockerCommunicationMode(ui->comboBoxCommunicationMode);
    QSignalBlocker blockerSerialPort(ui->comboBoxSerialPort);
    QSignalBlocker blockerDataFormat(ui->comboBoxDataFormat);
    QSignalBlocker blockerBaudRate(ui->comboBoxBaudRate);
    QSignalBlocker blockerVerify(ui->comboBoxVerify);
    QSignalBlocker blockerExportSignal(ui->comboBoxExportSignalSelect);
    QSignalBlocker blockerSignalSort(ui->comboBoxSignalSortSelect);
    QSignalBlocker blockerSignalThreshold(ui->doubleSpinBoxSignalThresholdValue);
    QSignalBlocker blockerSignalRatio(ui->doubleSpinBoxSignalRatio);
    QSignalBlocker blockerSignalSensitivity(ui->spinBoxSignalSensitivity);
    QSignalBlocker blockerSignalCount(ui->spinBoxSignalCount);
    QSignalBlocker blockerSignalMinNum(ui->spinBoxSignalMinNum);
    QSignalBlocker blockerSignalSmooth(ui->spinBoxSignalSmoothlength);
    QSignalBlocker blockerSignalMedian(ui->spinBoxSignalMedianFilter);
    QSignalBlocker blockerSignalAverage(ui->spinBoxSignalAverageFilter);
    QSignalBlocker blockerTriggerMode(ui->comboBoxTriggerMode);
    QSignalBlocker blockerEncodeWorkMode(ui->comboBoxEncodeWorkMode);
    QSignalBlocker blockerEncodeInputMode(ui->comboBoxEncodeInputMode);
    QSignalBlocker blockerEncodeDivision(ui->spinBoxEncodeDivision);

    if(config.singleMeasureMode >= 0)
        ui->comboBoxSingleChannelMeasureModeSelect->setCurrentIndex(config.singleMeasureMode);
    ui->checkBoxSingleChannelABS->setChecked(config.singleAbsMode != 0);
    ui->checkBoxSingleChannelReverse->setChecked(config.singleReverse != 0);
    ui->checkBoxSingleChannelMultiDistance->setChecked(config.singleMultDistance != 0);

    switch(config.exposureTime)
    {
    case 20: ui->comboBoxExposureTime->setCurrentIndex(0); break;
    case 50: ui->comboBoxExposureTime->setCurrentIndex(1); break;
    case 100: ui->comboBoxExposureTime->setCurrentIndex(2); break;
    case 200: ui->comboBoxExposureTime->setCurrentIndex(3); break;
    case 400: ui->comboBoxExposureTime->setCurrentIndex(4); break;
    case 700: ui->comboBoxExposureTime->setCurrentIndex(5); break;
    case 1000: ui->comboBoxExposureTime->setCurrentIndex(6); break;
    case 1500: ui->comboBoxExposureTime->setCurrentIndex(7); break;
    default: break;
    }

    if(config.isCF3000Lite)
    {
        ui->comboBoxLightSourceChannelSelect->setEnabled(false);
        ui->comboBoxLightMode->setEnabled(false);
        ui->comboBoxLightReference->setEnabled(false);
        ui->horizontalSliderLightValue->setEnabled(false);
        ui->comboBoxExposureTime->setEnabled(false);
        ui->groupBox_autoExporsure->setEnabled(true);

        ui->comboBox_3LAutoExporsureMode->setCurrentIndex(config.autoExposureEnable);
        ui->comboBox_3LAutoExporsureChannel->setCurrentIndex(config.autoExposureChannel);
        ui->spinBox_3LAutoExporsureValue->setValue(config.autoExposureTime);
    }
    else
    {
        ui->comboBoxLightSourceChannelSelect->setEnabled(true);
        ui->comboBoxLightMode->setEnabled(true);
        ui->comboBoxLightReference->setEnabled(config.lightMode != 0);
        ui->horizontalSliderLightValue->setEnabled(config.lightMode == 0);
        ui->comboBoxExposureTime->setEnabled(true);
        ui->groupBox_autoExporsure->setEnabled(false);

        ui->comboBoxLightMode->setCurrentIndex(config.lightMode);
        ui->comboBoxLightReference->setCurrentIndex(config.autoLightReference);
        ui->horizontalSliderLightValue->setValue(static_cast<int>(config.lightValue));
        ui->rb_subBackgroundLight->setChecked(config.subBackgroundLight != 0);
    }

    ui->tabCommunicationMode->setEnabled(config.communicationEnabled);
    if(config.communicationEnabled)
    {
        ui->comboBoxCommunicationMode->setCurrentIndex(config.communicationMode);
        if(config.serialPort > 1 && config.serialPort < 4)
            ui->comboBoxSerialPort->setCurrentIndex(2);
        else if(config.serialPort <= 1)
            ui->comboBoxSerialPort->setCurrentIndex(config.serialPort);
        ui->comboBoxDataFormat->setCurrentIndex(config.dataFormat);
        ui->comboBoxBaudRate->setCurrentIndex(config.baudRate);
        if(config.verify <= 3)
            ui->comboBoxVerify->setCurrentIndex(config.verify);
        else if(config.verify >= 4 && config.verify <= 7)
            ui->comboBoxVerify->setCurrentIndex(4);
    }

    ui->comboBoxExportSignalSelect->setCurrentIndex(config.exportSignalSelect);
    ui->comboBoxSignalSortSelect->setCurrentIndex(config.signalSort);
    ui->doubleSpinBoxSignalThresholdValue->setValue(config.signalThreshold);
    ui->doubleSpinBoxSignalRatio->setValue(config.signalRatio);
    ui->spinBoxSignalSensitivity->setValue(config.signalSensitivity);
    ui->spinBoxSignalCount->setValue(config.signalCount);
    ui->spinBoxSignalMinNum->setValue(config.signalMinNum);
    ui->spinBoxSignalSmoothlength->setValue(config.signalSmoothLength);
    ui->spinBoxSignalMedianFilter->setValue(config.signalMedianFilter);
    ui->spinBoxSignalAverageFilter->setValue(config.signalAverageFilter);

    ui->comboBoxTriggerMode->setCurrentIndex(config.triggerMode);
    ui->gb_frameSetting->setEnabled(config.frameSettingEnabled);

    ui->groupBox_CF3000LiteFilePath->setEnabled(config.cf3000LiteFilePathEnabled);
    ui->comboBoxEncodeInputMode->setEnabled(!config.isCF3000Lite);
    ui->comboBoxEncodeWorkMode->setEnabled(!config.isCF3000Lite);
    ui->spinBoxEncodeDivision->setEnabled(!config.isCF3000Lite);

    if(config.isCF3000Lite)
    {
        ui->lineEdit_CF3000LiteFilePath->setText(QCoreApplication::applicationDirPath());
    }
    else
    {
        ui->comboBoxEncodeWorkMode->setCurrentIndex(config.encodeWorkMode);
        ui->comboBoxEncodeInputMode->setCurrentIndex(config.encodeInputMode);
        ui->spinBoxEncodeDivision->setValue(config.encodeDivision);
    }
}

void MainWindow::updateConfocalStatusSnapshot(const ConfocalStatusSnapshot& snapshot)
{
    sdkVersionResult = snapshot.sdkVersion;
    controlerVersionResult = snapshot.controlerVersion;
    controlerSNResult = snapshot.controlerSN;
    sensorSNResult = snapshot.sensorSN;
    triggerMode = snapshot.triggerMode;
    curFrameRate = snapshot.curFrameRate;

    setStatusBarValue();
}

void MainWindow::updateConfocalMeasurementResult(int channelIndex, float result, float saturation, const QVector<int>& signalData)
{
    // 参数校验，无效时提前返回。
    if(channelIndex < 0 || channelIndex >= 4)
        return;

    g_Global->g_measureResult[channelIndex] = result;
    g_Global->g_saturation[channelIndex] = saturation;

    std::fill(&g_Global->signalDate[channelIndex][0],
              &g_Global->signalDate[channelIndex][0] + 1440,
              0);

    const int copyLength = std::min(signalData.size(), 1440);
    if(copyLength > 0)
    {
        std::memcpy(&g_Global->signalDate[channelIndex][0],
                    signalData.constData(),
                    static_cast<size_t>(copyLength) * sizeof(int));
    }
}


void MainWindow::initRuntimeBuffers()
{
    xunhuan = 1024;

    jingtai.resize(350);

    jiaodu2.resize(xunhuan);
    cgq2.resize(xunhuan);

    jiaodu3.resize(xunhuan);
    cgq3.resize(xunhuan);

    // cgqpxht.resize(xunhuan);
    // jiaodupxht.resize(xunhuan);
}

void MainWindow::initTimers()
{
    mahrTimer = new QTimer(this);
    mahrTimer->setInterval(100);

    mahrTimer77 = new QTimer(this);
    mahrTimer77->setInterval(100);

    confTimer = new QTimer(this);
    confTimer->setInterval(100);

    StatusBarTimer = new QTimer(this);
    StatusBarTimer->setInterval(1000);



}

void MainWindow::setupCentralSplitter()
{
    // 目标：让「信息输出框」上方的分隔条可以拖动，从而调整它与主 Tab 的高度比例。
    // 原布局中二者直接放在 gridLayoutCentral 里，QGridLayout 无法拖动，故改为塞进 QSplitter。
    if(!ui->Tab_Main || !ui->groupBoxMessageExport)
        return;

    QWidget* central = ui->centralwidget;
    auto* centralLayout = qobject_cast<QGridLayout*>(central->layout());
    if(!centralLayout)
        return;

    // 信息框原有 maximumSize 高度 160 的硬限制会导致拖大被卡住，这里放开高度上限。
    ui->groupBoxMessageExport->setMaximumHeight(QWIDGETSIZE_MAX);

    // 从原布局中移除两个控件，交给 splitter 接管（setWidget 会自动 reparent）。
    centralLayout->removeWidget(ui->Tab_Main);
    centralLayout->removeWidget(ui->groupBoxMessageExport);

    auto* splitter = new QSplitter(Qt::Vertical, central);
    splitter->setObjectName(QStringLiteral("centralSplitter"));
    splitter->setChildrenCollapsible(false);       // 禁止把某一块拖到完全消失
    splitter->setHandleWidth(6);
    splitter->addWidget(ui->Tab_Main);              // 上：主 Tab
    splitter->addWidget(ui->groupBoxMessageExport); // 下：信息输出框

    // 拖动时优先拉伸上方 Tab，信息框保持相对稳定。
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 0);
    splitter->setSizes(QList<int>() << 760 << 160); // 初始高度，贴近原先 160 的信息框高度

    centralLayout->addWidget(splitter, 0, 0, 2, 1); // 占据原来两行的位置
}

void MainWindow::initial_UI()
{
    this->setStyleSheet("");

    const auto widgets = this->findChildren<QWidget *>();
    for (QWidget *w : widgets) {
        w->setStyleSheet("");
    }

    this->setStyleSheet(R"(
QMainWindow, QWidget {
    background-color: #f3f5f8;
    font-family: "Microsoft YaHei";
    font-size: 10.5pt;
    color: #2f3b4a;
}

QGroupBox {
    background-color: #f8fafc;
    border: 1px solid #d7dee8;
    border-radius: 12px;
    margin-top: 14px;
    padding: 14px;
    font-weight: 600;
}

QGroupBox::title {
    subcontrol-origin: margin;
    left: 14px;
    padding: 0 6px 0 6px;
    color: #41556d;
    background-color: #f8fafc;
}

QLabel {
    color: #334155;
    background: transparent;
}

QLineEdit,
QComboBox,
QDoubleSpinBox,
QSpinBox {
    background-color: #ffffff;
    border: 1px solid #d5dde8;
    border-radius: 10px;
    padding: 4px 34px 4px 8px;
    min-height: 28px;
    color: #2f3b4a;
}

QComboBox::drop-down {
    subcontrol-origin: padding;
    subcontrol-position: center right;
    width: 24px;
    border: none;
    background: transparent;
}

QComboBox::down-arrow {
    image: none;
    width: 0;
    height: 0;
    border-left: 5px solid transparent;
    border-right: 5px solid transparent;
    border-top: 6px solid #5b6f86;
    margin-right: 6px;
}

/* 下拉弹出列表：单独设置背景、圆角和 item 高度，避免下拉项文字显示不全 */
QComboBox QAbstractItemView {
    background-color: #ffffff;
    border: 1px solid #d5dde8;
    border-radius: 8px;
    padding: 4px;
    outline: none;
    selection-background-color: #4f86c6;
    selection-color: #ffffff;
}

QComboBox QAbstractItemView::item {
    min-height: 28px;
    padding: 2px 8px;
}

/* SpinBox 上下按钮：分别定位到右上/右下角，保留可点击区域 */
QSpinBox::up-button,
QDoubleSpinBox::up-button {
    subcontrol-origin: border;
    subcontrol-position: top right;
    width: 22px;
    border: none;
    border-top-right-radius: 10px;
    background: transparent;
}

QSpinBox::down-button,
QDoubleSpinBox::down-button {
    subcontrol-origin: border;
    subcontrol-position: bottom right;
    width: 22px;
    border: none;
    border-bottom-right-radius: 10px;
    background: transparent;
}

QSpinBox::up-button:hover,
QSpinBox::down-button:hover,
QDoubleSpinBox::up-button:hover,
QDoubleSpinBox::down-button:hover {
    background: #e8eef6;
}

/* 用 CSS 三角形重新绘制上下箭头，替代被 QSS 关掉的原生箭头 */
QSpinBox::up-arrow,
QDoubleSpinBox::up-arrow {
    image: none;
    width: 0;
    height: 0;
    border-left: 4px solid transparent;
    border-right: 4px solid transparent;
    border-bottom: 5px solid #5b6f86;
}

QSpinBox::down-arrow,
QDoubleSpinBox::down-arrow {
    image: none;
    width: 0;
    height: 0;
    border-left: 4px solid transparent;
    border-right: 4px solid transparent;
    border-top: 5px solid #5b6f86;
}

QSpinBox::up-arrow:disabled, QSpinBox::up-arrow:off,
QDoubleSpinBox::up-arrow:disabled, QDoubleSpinBox::up-arrow:off {
    border-bottom-color: #c6ced8;
}

QSpinBox::down-arrow:disabled, QSpinBox::down-arrow:off,
QDoubleSpinBox::down-arrow:disabled, QDoubleSpinBox::down-arrow:off {
    border-top-color: #c6ced8;
}

QLineEdit:focus,
QComboBox:focus,
QDoubleSpinBox:focus,
QSpinBox:focus {
    border: 1px solid #4f86c6;
    background-color: #ffffff;
}

QTextEdit, QPlainTextEdit {
    background-color: #ffffff;
    border: 1px solid #d7dee8;
    border-radius: 10px;
    padding: 10px 12px;
}

QPushButton {
    background-color: #6f9fd1;
    color: #ffffff;
    border: 1px solid #5f8ec2;
    border-radius: 10px;
    padding: 6px 10px;
    min-height: 24px;
}

QPushButton:hover {
    background-color: #7eacd9;
}

QPushButton:pressed {
    background-color: #5b88bd;
}

QPushButton:disabled {
    background-color: #d5dbe3;
    color: #f9fafb;
    border: 1px solid #c6ced8;
}

QPushButton#emergency_stop {
    background-color: #d96c6c;
    border: 1px solid #c85e5e;
}

QPushButton#emergency_stop:hover {
    background-color: #e28282;
}

QPushButton#emergency_stop:pressed {
    background-color: #bf5757;
}

QPushButton#btnMotor_Enable {
    background-color: #6ca86c;
    border: 1px solid #5c975c;
}

QPushButton#btnMotor_Enable:hover {
    background-color: #80b680;
}

QPushButton#btnMotor_Enable:pressed {
    background-color: #548954;
}

QWidget#points_show {
    background-color: #eef2f6;
}

QFrame#pointCloudSidebar,
QFrame#pointCloudControlShell,
QGroupBox#groupBoxMessageExport {
    background-color: #f9fbfd;
    border: 1px solid #d8e0ea;
    border-radius: 16px;
}

QFrame#pointCloudControlShell {
    padding: 0px;
}

QFrame#pointCloudViewerCard {
    background-color: #f7f9fc;
    border: 1px solid #d8e1ec;
    border-radius: 18px;
}

QFrame#pointCloudHeader,
QFrame#pointCloudListToolbar {
    background-color: transparent;
    border: none;
}

QFrame#pointCloudControlGroup {
    background-color: #f7f9fc;
    border: 1px solid #dde5ef;
    border-radius: 12px;
}

QLabel#pointCloudTitle {
    color: #223247;
    font-size: 15pt;
    font-weight: 700;
}

QLabel#pointCloudHint {
    color: #64748b;
    font-size: 9pt;
}

QLabel#pointCloudSectionTitle,
QLabel#pointControlGroupTitle,
QGroupBox#groupBoxMessageExport::title {
    color: #41556d;
    font-size: 10pt;
    font-weight: 700;
    padding-bottom: 2px;
}

DICQGLViewer#DicGLWidget, QWidget#DicGLWidget {
    background-color: #ffffff;
    border: 1px solid #d8e1ec;
    border-radius: 14px;
}

QListView#listWidgetPointCloudFiles {
   background-color: #ffffff;
   border: 1px solid #d8e1ec;
   border-radius: 12px;
   padding: 8px;
   color: #304156;
   outline: 0;
}

QListView#listWidgetPointCloudFiles::item {
   min-height: 36px;
   border-radius: 9px;
   padding: 5px 10px;
}

QListView#listWidgetPointCloudFiles::item:hover {
   background-color: #edf3f9;
}

QListView#listWidgetPointCloudFiles::item:selected {
   background-color: #5f87b2;
   color: #ffffff;
}

QPushButton#cadImport,
QPushButton#ToPcd,
QPushButton#reconstructSurface,
QPushButton#AxisCalibrate,
QPushButton#analysis,
QPushButton#Fit {
    background-color: #4c79a8;
    border: 1px solid #3f6790;
    border-radius: 10px;
    color: #ffffff;
    font-weight: 600;
    min-height: 22px;
    max-height: 26px;
    padding: 3px 8px;
}

QPushButton#cadImport:hover,
QPushButton#ToPcd:hover,
QPushButton#reconstructSurface:hover,
QPushButton#AxisCalibrate:hover,
QPushButton#analysis:hover,
QPushButton#Fit:hover {
    background-color: #5a88b7;
}

QPushButton#cadImport:pressed,
QPushButton#ToPcd:pressed,
QPushButton#reconstructSurface:pressed,
QPushButton#AxisCalibrate:pressed,
QPushButton#analysis:pressed,
QPushButton#Fit:pressed {
    background-color: #3e678f;
}

QPushButton#MoreColor,
QPushButton#RT_Matrix,
QPushButton#btnListUp,
QPushButton#btnListDown,
QPushButton#SOR_Method,
QPushButton#Centroid_Method,
QPushButton#MLS_Method,
QPushButton#NormalConsistency,
QPushButton#recon_ball,
QPushButton#PCA_Method,
QPushButton#coarseregistration {
    background-color: #edf2f7;
    color: #405266;
    border: 1px solid #cfd9e4;
    border-radius: 10px;
    font-weight: 600;
    min-height: 22px;
    max-height: 26px;
    padding: 3px 8px;
}

QPushButton#MoreColor:hover,
QPushButton#RT_Matrix:hover,
QPushButton#btnListUp:hover,
QPushButton#btnListDown:hover,
QPushButton#SOR_Method:hover,
QPushButton#Centroid_Method:hover,
QPushButton#MLS_Method:hover,
QPushButton#NormalConsistency:hover,
QPushButton#recon_ball:hover,
QPushButton#PCA_Method:hover,
QPushButton#coarseregistration:hover {
    background-color: #e4ebf3;
}

QPushButton#MoreColor:pressed,
QPushButton#RT_Matrix:pressed,
QPushButton#btnListUp:pressed,
QPushButton#btnListDown:pressed,
QPushButton#SOR_Method:pressed,
QPushButton#Centroid_Method:pressed,
QPushButton#MLS_Method:pressed,
QPushButton#NormalConsistency:pressed,
QPushButton#recon_ball:pressed,
QPushButton#PCA_Method:pressed,
QPushButton#coarseregistration:pressed {
    background-color: #d9e3ed;
}

QPushButton#RT_Matrix:disabled,
QPushButton#btnListUp:disabled,
QPushButton#btnListDown:disabled {
    background-color: #e4e8ee;
    color: #95a2b1;
    border: 1px solid #d3dae3;
}

QTabWidget#tabWidget_Points {
    background-color: transparent;
}

QTabWidget#tabWidget_Points::pane {
    background-color: #f9fbfd;
    border: 1px solid #d8e0ea;
    border-radius: 16px;
    padding: 12px;
    top: -1px;
}

QTabWidget#tabWidget_Points QTabBar::tab {
    background-color: #e7edf3;
    color: #50657c;
    border: 1px solid #d5dde8;
    border-bottom: none;
    border-top-left-radius: 10px;
    border-top-right-radius: 10px;
    padding: 9px 18px;
    margin-right: 4px;
    font-weight: 600;
}

QTabWidget#tabWidget_Points QTabBar::tab:selected {
    background-color: #f9fbfd;
    color: #2e4a67;
}

QWidget#tab_2,
QWidget#tab_sl {
    background-color: #f9fbfd;
}

QCheckBox#Select_Multicorn {
    color: #405266;
    spacing: 8px;
    padding-left: 2px;
}

QGroupBox#groupBoxMessageExport {
    margin-top: 14px;
    padding: 14px;
}

QPlainTextEdit#plainTextEdit {
    background-color: #ffffff;
    border: 1px solid #d9e2ec;
    border-radius: 10px;
    padding: 10px 12px;
    color: #2f3d4c;
}
)");

    ui->points_show->setAttribute(Qt::WA_StyledBackground, true);
    ui->tab_2->setAttribute(Qt::WA_StyledBackground, true);
    ui->tab_sl->setAttribute(Qt::WA_StyledBackground, true);
    ui->DicGLWidget->setMinimumSize(760, 560);
    ui->DicGLWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    ui->listWidgetPointCloudFiles->setMinimumWidth(220);
    ui->listWidgetPointCloudFiles->setMaximumWidth(320);
    ui->listWidgetPointCloudFiles->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    ui->tabWidget_Points->setMinimumWidth(360);
    ui->tabWidget_Points->setMaximumWidth(430);
    ui->tabWidget_Points->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    ui->plainTextEdit->setPlaceholderText(tr("系统运行日志会显示在这里"));

    ui->progressBar->setRange(0, 100);
    ui->progressBar->setFormat("%p%");
    ui->progressBar->setValue(0);

    ui->pianxinX->display(QString::number(0, 'f', 4));
    ui->pianxinY->display(QString::number(0, 'f', 4));

    ui->label_56->setStyleSheet("background-color: yellow;");

    ui->progressBar_2->setRange(0, 100);
    ui->progressBar_2->setFormat("%p%");
    ui->progressBar_2->setValue(0);

    ui->qingxieX->display(QString::number(0, 'f', 5));
    ui->qingxieY->display(QString::number(0, 'f', 5));

    ui->lcdNumber->display(QString::number(0, 'f', 4));
    ui->lcdNumber_2->display(QString::number(0, 'f', 4));

    ui->label_71->setStyleSheet("background-color: yellow;");
}

void MainWindow::setupPointCloudWorkspaceUi()
{
    // 不重新排列布局，保持 .ui 文件中定义的按钮位置
    // 只应用样式，不修改布局结构
}

void MainWindow::stylePointCloudWorkspace()
{
    if(!ui)
        return;

    if(ui->groupBoxMessageExport)
        ui->groupBoxMessageExport->setFlat(false);
}

void MainWindow::refreshPointCloudWorkspaceState()
{
    if(!ui || !ui->listWidgetPointCloudFiles || !pointCloudListModel)
    {
        if(ui && ui->btnListUp)
            ui->btnListUp->setEnabled(false);
        if(ui && ui->btnListDown)
            ui->btnListDown->setEnabled(false);
        return;
    }

    const QModelIndex current = ui->listWidgetPointCloudFiles->currentIndex();
    const int rowCount = pointCloudListModel->rowCount();
    const int row = current.isValid() ? current.row() : -1;

    if(ui->btnListUp)
        ui->btnListUp->setEnabled(row > 0);
    if(ui->btnListDown)
        ui->btnListDown->setEnabled(row >= 0 && row < rowCount - 1);
}

QWidget *MainWindow::ensurePointCloudSidebar()
{
    if(!ui || !ui->points_show || !ui->listWidgetPointCloudFiles)
        return nullptr;

    auto *sidebar = ui->points_show->findChild<QFrame*>(QStringLiteral("pointCloudSidebar"));
    if(sidebar)
        return sidebar;

    sidebar = new QFrame(ui->points_show);
    sidebar->setObjectName(QStringLiteral("pointCloudSidebar"));
    sidebar->setFrameShape(QFrame::NoFrame);
    sidebar->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

    auto *layout = new QVBoxLayout(sidebar);
    layout->setContentsMargins(14, 14, 14, 14);
    layout->setSpacing(12);

    auto *titleLabel = new QLabel(tr("点云列表"), sidebar);
    titleLabel->setObjectName(QStringLiteral("pointCloudSectionTitle"));
    layout->addWidget(titleLabel);

    if(ui->listWidgetPointCloudFiles->parentWidget() != sidebar)
        ui->listWidgetPointCloudFiles->setParent(sidebar);
    layout->addWidget(ui->listWidgetPointCloudFiles, 1);
    setupPointCloudListToolbar(layout);
    return sidebar;
}

QWidget *MainWindow::ensurePointCloudViewerPanel()
{
    if(!ui || !ui->points_show || !ui->DicGLWidget)
        return nullptr;

    auto *viewerCard = ui->points_show->findChild<QFrame*>(QStringLiteral("pointCloudViewerCard"));
    if(viewerCard)
        return viewerCard;

    viewerCard = new QFrame(ui->points_show);
    viewerCard->setObjectName(QStringLiteral("pointCloudViewerCard"));
    viewerCard->setFrameShape(QFrame::NoFrame);
    viewerCard->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    auto *viewerLayout = new QVBoxLayout(viewerCard);
    viewerLayout->setContentsMargins(18, 16, 18, 18);
    viewerLayout->setSpacing(14);

    auto *header = new QFrame(viewerCard);
    header->setObjectName(QStringLiteral("pointCloudHeader"));
    auto *headerLayout = new QHBoxLayout(header);
    headerLayout->setContentsMargins(0, 0, 0, 0);
    headerLayout->setSpacing(12);

    auto *titleLabel = new QLabel(tr("点云预览"), header);
    titleLabel->setObjectName(QStringLiteral("pointCloudTitle"));
    auto *hintLabel = new QLabel(tr("左键旋转视角 · 右键框选 · Ctrl/Shift 选点"), header);
    hintLabel->setObjectName(QStringLiteral("pointCloudHint"));
    hintLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    headerLayout->addWidget(titleLabel);
    headerLayout->addStretch(1);
    headerLayout->addWidget(hintLabel);

    if(ui->DicGLWidget->parentWidget() != viewerCard)
        ui->DicGLWidget->setParent(viewerCard);

    viewerLayout->addWidget(header);
    viewerLayout->addWidget(ui->DicGLWidget, 1);
    return viewerCard;
}

QWidget *MainWindow::ensurePointCloudControlPanel()
{
    if(!ui || !ui->points_show || !ui->tabWidget_Points)
        return nullptr;

    auto *panel = ui->points_show->findChild<QFrame*>(QStringLiteral("pointCloudControlShell"));
    if(panel)
        return panel;

    panel = new QFrame(ui->points_show);
    panel->setObjectName(QStringLiteral("pointCloudControlShell"));
    panel->setFrameShape(QFrame::NoFrame);
    panel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(14, 14, 14, 14);
    layout->setSpacing(10);

    if(ui->tabWidget_Points->parentWidget() != panel)
        ui->tabWidget_Points->setParent(panel);

    regroupPointCloudActionButtons(ui->tab_2);
    layout->addWidget(ui->tabWidget_Points, 1);
    return panel;
}

QWidget *MainWindow::ensurePointCloudLogPanel()
{
    return ui ? ui->groupBoxMessageExport : nullptr;
}

void MainWindow::setupPointCloudListToolbar(QBoxLayout *sidebarLayout)
{
    if(!ui || !ui->btnListUp || !ui->btnListDown || !sidebarLayout)
        return;

    auto *toolbar = new QFrame;
    toolbar->setObjectName(QStringLiteral("pointCloudListToolbar"));
    auto *toolbarLayout = new QHBoxLayout(toolbar);
    toolbarLayout->setContentsMargins(0, 0, 0, 0);
    toolbarLayout->setSpacing(10);

    ui->btnListUp->setText(tr("上移"));
    ui->btnListDown->setText(tr("下移"));
    ui->btnListUp->setMinimumHeight(34);
    ui->btnListDown->setMinimumHeight(34);
    ui->btnListUp->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    ui->btnListDown->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    if(ui->btnListUp->parentWidget() != toolbar)
        ui->btnListUp->setParent(toolbar);
    if(ui->btnListDown->parentWidget() != toolbar)
        ui->btnListDown->setParent(toolbar);

    toolbarLayout->addWidget(ui->btnListUp);
    toolbarLayout->addWidget(ui->btnListDown);
    sidebarLayout->addWidget(toolbar);
}

void MainWindow::regroupPointCloudActionButtons(QWidget *optionsPage)
{
    // 不重新排列按钮布局，保持 .ui 文件中定义的原始位置
    // 只通过 CSS 样式控制按钮外观，不修改父子关系和布局结构
}

void MainWindow::intial_window_connect()
{
    setLED(ui->lblLED, 1, 128);
    jogPanel = new JogPanel(nullptr);
    jogPanel->move(200, 200);

    // X
    connect(jogPanel, &JogPanel::xPositivePressed,
            this, &MainWindow::X_positive_pressed);
    connect(jogPanel, &JogPanel::xPositiveReleased,
            this, &MainWindow::X_positive_released);

    connect(jogPanel, &JogPanel::xNegativePressed,
            this, &MainWindow::X_negative_pressed);
    connect(jogPanel, &JogPanel::xNegativeReleased,
            this, &MainWindow::X_negative_released);

    // Y
    connect(jogPanel, &JogPanel::yPositivePressed,
            this, &MainWindow::Y_positive_pressed);
    connect(jogPanel, &JogPanel::yPositiveReleased,
            this, &MainWindow::Y_positive_released);

    connect(jogPanel, &JogPanel::yNegativePressed,
            this, &MainWindow::Y_negative_pressed);
    connect(jogPanel, &JogPanel::yNegativeReleased,
            this, &MainWindow::Y_negative_released);

    // Z
    connect(jogPanel, &JogPanel::zPositivePressed,
            this, &MainWindow::Z_positive_pressed);
    connect(jogPanel, &JogPanel::zPositiveReleased,
            this, &MainWindow::Z_positive_released);

    connect(jogPanel, &JogPanel::zNegativePressed,
            this, &MainWindow::Z_negative_pressed);
    connect(jogPanel, &JogPanel::zNegativeReleased,
            this, &MainWindow::Z_negative_released);


    // 绑定信号槽
    connect(jogPanel, &JogPanel::adjustAPositivePressed,
            this, &MainWindow::adjustA_positive_pressed);
    connect(jogPanel, &JogPanel::adjustAPositiveReleased,
            this, &MainWindow::adjustA_positive_released);

    connect(jogPanel, &JogPanel::adjustANegativePressed,
            this, &MainWindow::adjustA_negative_pressed);
    connect(jogPanel, &JogPanel::adjustANegativeReleased,
            this, &MainWindow::adjustA_negative_released);

    // 绑定信号槽
    connect(jogPanel, &JogPanel::adjustBPositivePressed,
            this, &MainWindow::adjustB_positive_pressed);
    connect(jogPanel, &JogPanel::adjustBPositiveReleased,
            this, &MainWindow::adjustB_positive_released);

    connect(jogPanel, &JogPanel::adjustBNegativePressed,
            this, &MainWindow::adjustB_negative_pressed);
    connect(jogPanel, &JogPanel::adjustBNegativeReleased,
            this, &MainWindow::adjustB_negative_released);

}
///======================= PLC =======================///
void MainWindow::initPlcRuntimeConnection()
{
    // 绑定信号槽
    connect(mahrTimer, &QTimer::timeout,
            this, &MainWindow::mahrSensor,
            Qt::UniqueConnection);

    connect(confTimer, &QTimer::timeout,
            this, &MainWindow::confSensor,
            Qt::UniqueConnection);

    connect(StatusBarTimer, &QTimer::timeout,
            this, &MainWindow::StatuesBar_flash,
            Qt::UniqueConnection);


    i_rote = 0;
}

void MainWindow::initial_velocity()
{
    on_btnPosition_Velocity_clicked() ; // 写入当前定位轴速度
    on_btnRotate_Velocity_clicked() ; // 写入当前旋转轴速度
}

void MainWindow::initPlcPaths()
{
    address_x1 = ui->addressx_reg_1->text().toDouble();
    address_y1 = ui->addressy_reg_1->text().toDouble();
    address_z1 = ui->addressz_reg_1->text().toDouble();
    address_x2 = ui->addressx_cyl_1->text().toDouble();
    address_y2 = ui->addressy_cyl_1->text().toDouble();
    address_z2 = ui->addressz_cyl_1->text().toDouble();
    address_y12 = ui->addressy_reg_2->text().toDouble();
    address_y22 = ui->addressy_cyl_2->text().toDouble();
    address_z12 = ui->addressz_reg_2->text().toDouble();
    address_z22 = ui->addressz_cyl_2->text().toDouble();

    addressqing_reg_1 = ui->addressqing_reg_1->text().toDouble();
    addressqing_reg_2 = ui->addressqing_reg_2->text().toDouble();
    addressqing_reg_3 = ui->addressqing_reg_3->text().toDouble();
    addressqing_reg_4 = ui->addressqing_reg_4->text().toDouble();
    addressqing_cyl_1 = ui->addressqing_cyl_1->text().toDouble();
    addressqing_cyl_2 = ui->addressqing_cyl_2->text().toDouble();
    addressqing_cyl_3 = ui->addressqing_cyl_3->text().toDouble();
    addressqing_cyl_4 = ui->addressqing_cyl_4->text().toDouble();

    addressxin_reg_1 = ui->addressxin_reg_1->text().toDouble();
    addressxin_reg_2 = ui->addressxin_reg_2->text().toDouble();
    addressxin_cyl_1 = ui->addressxin_cyl_1->text().toDouble();
    addressxin_cyl_2 = ui->addressxin_cyl_2->text().toDouble();


}
///======================= PLC =======================///
void MainWindow::mahrSensor()
{
    postPlcStartCollectingData();


    QDateTime dateTime = QDateTime::currentDateTime();
    ui->lblDataTime->setText(dateTime.toString("yyyy-MM-dd hh:mm:ss")); // 格式化时间

    // PLC

    // 2024.8.28
    ui->guangpu_X->display(QString::number(plcSnapshot.m_new_lizhu1_X, 'f', 4));

    ui->guangpu_Y->display(QString::number(plcSnapshot.m_new_sensor1_Y, 'f', 4));

    ui->guangpu_Y_2->display(QString::number(plcSnapshot.m_new_sensor2_Y, 'f', 4));

    ui->guangpu_Z->display(QString::number(plcSnapshot.m_new_sensor1_Z, 'f', 4));

    ui->guangpu_Z_2->display(QString::number(plcSnapshot.m_new_sensor2_Z, 'f', 4));

    ui->rotation_angel->display(QString::number(plcSnapshot.m_new_angle, 'f', 2));

    ui->LCD_1_position->display(QString::number(plcSnapshot.m_tiaoqing1, 'f', 4));

    ui->LCD_2_position->display(QString::number(plcSnapshot.m_tiaoqing2, 'f', 4));

    ui->LCD_3_position->display(QString::number(plcSnapshot.m_tiaoqing3, 'f', 4));

    ui->LCD_4_position->display(QString::number(plcSnapshot.m_tiaoqing4, 'f', 4));

    ui->LCD_5_position->display(QString::number(plcSnapshot.m_tiaoxin1, 'f', 4));

    ui->LCD_6_position->display(QString::number(plcSnapshot.m_tiaoxin2, 'f', 4));

    ui->LCDsmallRotate_angle->display(QString::number(plcSnapshot.m_small_angle, 'f', 4));

}

void MainWindow::confSensor()
{
    ui->lcdNumberSingleChannelResult->display(g_Global->g_measureResult[g_Global->singleChannelIndex]);

}

void MainWindow::on_actionPlcCConect_triggered()
{
    PlcConnect *dlg = new PlcConnect(this);
    dlg->setAttribute(Qt::WA_DeleteOnClose);
    dlg->setWindowModality(Qt::ApplicationModal);

    dlg->setPlcConnected(plcConnectedFlag);

    connect(dlg, &PlcConnect::connectRequested,
            this, [this, dlg]() {
                if (!plcController) {
                    ui->plainTextEdit->appendPlainText(
                        systemTime() + " PLC doesn't exist"
                        );
                    return;
                }

                dlg->setPlcConnecting();

                ui->plainTextEdit->appendPlainText(
                    systemTime() + " 姝ｅ湪杩炴帴PLC..."
                    );

                postPlcConnect();
            });

    connect(dlg, &PlcConnect::disconnectRequested,
            this, [this]() {
                if (!plcController) {
                    ui->plainTextEdit->appendPlainText(
                        systemTime() + " PLC doesn't exist"
                        );
                    return;
                }

                ui->plainTextEdit->appendPlainText(
                    systemTime() + " 姝ｅ湪鏂紑PLC..."
                    );

                // 绑定信号槽
                            postPlcDisconnect();
            });

    if (plcController) {
        // 绑定信号槽
            connect(plcController, &Plc::plcConnecting,
                dlg, [dlg]() {
                    dlg->setPlcConnecting();
                }, Qt::QueuedConnection);

        connect(plcController, &Plc::plcConnected,
                dlg, [dlg]() {
                    dlg->setPlcConnected(true);
                }, Qt::QueuedConnection);

        connect(plcController, &Plc::plcDisconnected,
                dlg, [dlg]() {
                    dlg->setPlcConnected(false);
                }, Qt::QueuedConnection);

        connect(plcController, &Plc::plcError,
                dlg, [dlg](const QString &err) {
                    Q_UNUSED(err)
                    dlg->setPlcConnected(false);
                }, Qt::QueuedConnection);
    }

    dlg->show();
    dlg->raise();
    dlg->activateWindow();
}

// 构造对话框并把 CF 的 5 个设置分页从主界面 tabWidget_tri 移入对话框。
// 在启动时（构造函数中）调用，使主界面 tabWidget_tri 一开始就只保留其余分页。
// 采用 reparent 方案：这 5 页仍定义在 mainwindow.ui，QTabWidget::addTab 只改变父子
// 关系，控件对象仍是 MainWindow 的成员，ui->xxx 引用与 on_xxx_ 自动槽全部不受影响。
void MainWindow::initConfocalSettingsDialog()
{
    if (confocalSettingsDialog)
        return;

    confocalSettingsDialog = new ConfocalSettingsDialog(this);
    // 非模态：不抢占焦点，可与主窗口同时操作，便于"应用"边调试边看主界面。
    confocalSettingsDialog->setWindowModality(Qt::NonModal);

    // 确认/应用时写入设置（当前为占位，后续接入实际写入逻辑）。
    connect(confocalSettingsDialog, &ConfocalSettingsDialog::settingsApplyRequested,
            this, [this]() {
                // TODO: 在此写入光谱共焦传感器设置（下发 confocalWorker 或存配置）。
            });

    QTabWidget *dst = confocalSettingsDialog->contentTabWidget();
    struct { QWidget *page; const char *title; } cfPages[] = {
        { ui->tabSingleChannelMode,   u8"单通道" },
        { ui->tabCommunicationMode,   u8"通讯" },
        { ui->tabLightSourceMode,     u8"光源" },
        { ui->tabImportMode,          u8"输入" },
        { ui->tabSignalAndFilterMode, u8"信号饱和度" },
    };
    for (const auto &p : cfPages) {
        dst->addTab(p.page, QString::fromUtf8(p.title));
    }
}

// 光谱共焦传感器设置：点击菜单 actionCF_Sensor 弹出已建好的设置对话框。
void MainWindow::on_actionCF_Sensor_triggered()
{
    initConfocalSettingsDialog();  // 兜底：若启动时未初始化则此处补建。
    confocalSettingsDialog->show();
    confocalSettingsDialog->raise();
    confocalSettingsDialog->activateWindow();
}




void MainWindow::StatuesBar_flash()
{

    getStatusBarResult();
    setStatusBarValue();
}

void MainWindow::on_btnMotor_Enable_clicked(bool checked)
{
    // double address;

    if (checked)
    {
        ui->btnMotor_Enable->setText("鐢垫満涓婄數");
        ui->btnMotor_Enable->setCheckable(false);

        enable_flag = !enable_flag;
        setLED(ui->lblLED, 1, 128);

    }
    else
    {
        ui->btnMotor_Enable->setCheckable(true);
        ui->btnMotor_Enable->setText("鐢垫満鏂數");

        enable_flag = !enable_flag;
        setLED(ui->lblLED, 2, 128);
    }

    std::cout << enable_flag << std::endl;
    postPlcStartContinuousMovement(enable_flag, 6);
}


// color 0:grey 1:red 2:green 3:yellow
void MainWindow::setLED(QLabel* label, int color, int size)
{
    label->setText("");
    QString min_width = QString("min-width: %1px;").arg(size);              // 最小宽度：size
    QString min_height = QString("min-height: %1px;").arg(size);            // 最小高度：size
    QString max_width = QString("max-width: %1px;").arg(size);              // 最小宽度：size
    QString max_height = QString("max-height: %1px;").arg(size);            // 最小高度：size
    QString border_radius = QString("border-radius: %1px;").arg(size/2);    // 圆角边框，半径为size/2
    QString border = QString("border:1px solid black;");                    // 1px黑色边框
    QString background = "background-color:";
    switch (color) {
    case 0:
        background += "rgb(190,190,190)";
        break;
    case 1:
        background += "rgb(255,0,0)";
        break;
    case 2:
        background += "rgb(0,255,0)";
        break;
    case 3:
        background += "rgb(255,255,0)";
        break;
    default:
        break;
    }

    const QString SheetStyle = min_width + min_height + max_width + max_height + border_radius + border + background;
    label->setStyleSheet(SheetStyle);
}
void MainWindow::on_btnPosition_Velocity_clicked()
{
    QString Velocity = ui->txtPosition_Velocity->text();
    double registers = Velocity.toDouble();
    postPlcWriteHoldingRegisters(registers, 28);
}


void MainWindow::on_rotation_clicked()
{
    ui->rotation->setEnabled(false);

    QString target = ui->rotaiton_table->text();
    double registers = target.toDouble();

    postPlcWriteHoldingRegisters(registers, 0); // 向转台角度寄存器写入 LREAL

    QTimer::singleShot(500, this, [=]() {
        postPlcMoveToAbsolutePosition(1, 7); // 线圈触发启动
        ui->rotation->setEnabled(true);
    });
}


void MainWindow::on_btnRotate_Velocity_clicked()
{
    QString Velocity = ui->txtRotate_Velocity->text();
    double registers = Velocity.toDouble();

    postPlcWriteHoldingRegisters(registers, 24);
}


void MainWindow::on_btnRotate_clicked()
{


    double var = 1;
    postPlcStartContinuousMovement(var, 0);

}


void MainWindow::on_btnRotateStop_clicked()
{
    double var = 0;
    // double address = 6;
    postPlcStartContinuousMovement(var, 0);
}


void MainWindow::on_Daogui_X_clicked()
{
    QString target = ui->daoguix_x->text();
    double Target = target.toDouble();
    sendRegistersAndVar(Target, address_x1, address_x2, 500);
}


void MainWindow::on_Daogui_Y_clicked()
{
    QString target = ui->daoguix_y->text();
    double Target = target.toDouble();
    sendRegistersAndVar(Target, address_y1, address_y2, 500);
}


void MainWindow::on_Daogui_Y_2_clicked()
{
    QString target = ui->daoguix_y_2->text();
    double Target = target.toDouble();
    sendRegistersAndVar(Target, address_y12, address_y22, 500);
}


void MainWindow::on_Daogui_Z_clicked()
{
    QString target = ui->daoguix_z->text();
    double Target = target.toDouble();
    sendRegistersAndVar(Target, address_z1, address_z2, 500);
}


void MainWindow::on_Daogui_Z_2_clicked()
{
    QString target = ui->daoguix_z_2->text();
    double Target = target.toDouble();
    sendRegistersAndVar(Target, address_z12, address_z22, 500);
}


void MainWindow::sendRegistersAndVar(const double Target, double address, double address2, double sleep_time)
{
    postPlcWriteHoldingRegisters(Target, address);

    QTimer::singleShot(sleep_time, this, [=]() {
        postPlcMoveToAbsolutePosition(1, address2); // 线圈触发启动
    });
}


void MainWindow::on_confirm_clicked()
{
    address_x1 = ui->addressx_reg_1->text().toDouble();
    address_y1 = ui->addressy_reg_1->text().toDouble();
    address_z1 = ui->addressz_reg_1->text().toDouble();

    address_x2 = ui->addressx_cyl_1->text().toDouble();
    address_y2 = ui->addressy_cyl_1->text().toDouble();
    address_z2 = ui->addressz_cyl_1->text().toDouble();

    address_y12 = ui->addressy_reg_2->text().toDouble();
    address_y22 = ui->addressy_cyl_2->text().toDouble();
    address_z12 = ui->addressz_reg_2->text().toDouble();
    address_z22 = ui->addressz_cyl_2->text().toDouble();

    addressqing_reg_1 = ui->addressqing_reg_1->text().toDouble();
    addressqing_reg_2 = ui->addressqing_reg_2->text().toDouble();
    addressqing_reg_3 = ui->addressqing_reg_3->text().toDouble();
    addressqing_reg_4 = ui->addressqing_reg_4->text().toDouble();

    addressqing_cyl_1 = ui->addressqing_cyl_1->text().toDouble();
    addressqing_cyl_2 = ui->addressqing_cyl_2->text().toDouble();
    addressqing_cyl_3 = ui->addressqing_cyl_3->text().toDouble();
    addressqing_cyl_4 = ui->addressqing_cyl_4->text().toDouble();

    addressxin_reg_1 = ui->addressxin_reg_1->text().toDouble();
    addressxin_reg_2 = ui->addressxin_reg_2->text().toDouble();

    addressxin_cyl_1 = ui->addressxin_cyl_1->text().toDouble();
    addressxin_cyl_2 = ui->addressxin_cyl_2->text().toDouble();
}



void MainWindow::initPaths()
{
    // basePath = ui->cali_data_root->text().trimmed();
    // ball_A_path = basePath + "/A.txt";
    // ball_B_path = basePath + "/B.txt";
    // rotation_angel = basePath + "/rotation_angel.txt";

    ui->calculate->setEnabled(false);
    ui->load_calibration_data->setEnabled(false);
    ui->write_ball_A_points->setEnabled(false);
    ui->write_ball_B_points->setEnabled(false);
    ui->write_ball_A_done->setEnabled(false);
    ui->write_ball_B_done->setEnabled(false);
    ui->record_done->setEnabled(false);
    ui->recover->setEnabled(false);

    enable_flag = 0; // 上电初始化
}
void MainWindow::on_calculate_clicked()
{
    QString strR1 = ui->ball_A_R->text().trimmed();
    QString strR2 = ui->ball_B_R->text().trimmed();
    QString strD  = ui->distance->text().trimmed();
    ParameterPath = basePath + "/rotation_result.bin";

    if (strR1.isEmpty()) {
        QMessageBox::warning(this, "鎻愮ず", "Please give R1");
        return;
    }

    if (strR2.isEmpty()) {
        QMessageBox::warning(this, "鎻愮ず", "Please give R2");
        return;
    }

    if (strD.isEmpty()) {
        QMessageBox::warning(this, "鎻愮ず", "Please give distance");
        return;
    }

    if (basePath.isEmpty()) {
        QMessageBox::warning(this, "鎻愮ず", "Please give root path");
        return;
    }

    G_R1 = strR1.toDouble();
    G_R2 = strR2.toDouble();
    G_BALL_DISTANCE = strD.toDouble();
    groupCount = ui->comboBox_ball_number->currentText().toInt();

    QVector<QVector<MatrixXd>> Points_tri, Points_sensor, Points_test_all;

    try {
        fileReaderWriter->loadTxtABtoTriAndSensor(
            ball_A_path,
            ball_B_path,
            Points_tri,
            Points_sensor,
            groupCount
            );

        fileReaderWriter->combineTriAndSensor(
            Points_tri,
            Points_sensor,
            Points_test_all
            );

    } catch (const std::exception& e) {
        QMessageBox::critical(
            this,
            "閿欒",
            "数据加载失败: " + QString::fromStdString(e.what())
            );
        return;
    }

    Matrix3d M_g;
    QVector<QVector<Vector3d>> initial_center;
    calibration->gainInitialParameter(Points_test_all, M_g, initial_center);
    qDebug() << "Initial Rotation Matrix M_g:";
    for (int i = 0; i < 3; ++i) {
        qDebug() << M_g(i, 0) << M_g(i, 1) << M_g(i, 2);
    }
    double radius_1 = 0, radius_2 = 0;
    calibration->nonLinearOptimalMulti(
        M_g, Points_tri,
        G_R1, G_R2, G_BALL_DISTANCE, Points_sensor,
        center_all_tri, theta, M_opt, radius_1, radius_2);

    ui->plainTextEdit->appendPlainText(systemTime());
    ui->plainTextEdit->appendPlainText(" M_opt and (alpha, beta) have been saved successfully to: " + ParameterPath);

    ui->plainTextEdit->appendPlainText(tr("=== Calibration Matrix M_opt ==="));
    for (int i = 0; i < 3; ++i) {
        QString row;
        for (int j = 0; j < 3; ++j) {
            row += QString::number(M_opt(i, j), 'f', 6) + " ";
        }
        ui->plainTextEdit->appendPlainText(row.trimmed());
    }

    fileReaderWriter->saveParameters(M_opt, theta, ParameterPath);
}

void MainWindow::on_load_calibration_data_clicked()
{
    basePath = ui->cali_data_root->text().trimmed();
    ParameterPath = basePath + "\\rotation_result.bin";
    // 参数校验，无效时提前返回。
    if (!QFile::exists(ParameterPath)) {
        QMessageBox::warning(this, tr("File Not Found"), tr("rotation_result.bin not found. Path: ") + ParameterPath);
        return;
    }
    fileReaderWriter->loadRotationParameters(ParameterPath, M_opt, alpha, beta);
    ui->plainTextEdit->appendPlainText(systemTime());
    ui->plainTextEdit->appendPlainText(" M_opt and (alpha, beta) have been loaded successfully");

}

void MainWindow::on_btnChooseFile_clicked()
{
    qDebug() << ui->cali_data_root->text().trimmed();

    basePath = QFileDialog::getExistingDirectory(
        this,
        tr("Select Folder"),
        ui->cali_data_root->text().trimmed(),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks
        );

    if (!basePath.isEmpty()) {

        ui->calculate->setEnabled(true);
        ui->load_calibration_data->setEnabled(true);
        ui->record_done->setEnabled(true);
        ui->write_ball_A_points->setEnabled(true);
        ui->write_ball_B_points->setEnabled(true);
        ui->write_ball_A_done->setEnabled(true);
        ui->write_ball_B_done->setEnabled(true);
        ui->recover->setEnabled(true);

        ui->cali_data_root->setText(basePath); // 鏄剧ず鍒扮晫闈?

        ball_A_path = basePath + "\\A.txt";
        ball_B_path = basePath + "\\B.txt";
        rotation_angel = basePath + "\\rotation_angel.txt";

        QStringList createdFiles;

        // 参数校验，无效时提前返回。
        if (!QFile::exists(ball_A_path)) {
            QFile fileA(ball_A_path);
            if (fileA.open(QIODevice::WriteOnly | QIODevice::Text)) {
                fileA.close();
                createdFiles << "A.txt";
            }
        }

        // 参数校验，无效时提前返回。
        if (!QFile::exists(ball_B_path)) {
            QFile fileB(ball_B_path);
            if (fileB.open(QIODevice::WriteOnly | QIODevice::Text)) {
                fileB.close();
                createdFiles << "B.txt";
            }
        }

        // 参数校验，无效时提前返回。
        if (!QFile::exists(rotation_angel)) {
            QFile filerotation_angel(rotation_angel);
            if (filerotation_angel.open(QIODevice::WriteOnly | QIODevice::Text)) {
                filerotation_angel.close();
                createdFiles << "rotation_angel.txt";
            }
        }

        // 参数校验，无效时提前返回。
        if (!createdFiles.isEmpty()) {
            ui->plainTextEdit->appendPlainText(
                systemTime() + " 宸插垱寤烘爣瀹氭枃浠讹細" + createdFiles.join(", ")
                );
        } else {
            ui->plainTextEdit->appendPlainText(
                systemTime() + " 鎵€鏈夋爣瀹氭枃浠跺凡瀛樺湪"
                );
        }
    } else {
        QMessageBox::information(this, tr("Notice"), tr("No folder selected."));
    }
}


void MainWindow::on_write_ball_A_points_clicked()
{
    double x = plcSnapshot.m_new_lizhu1_X;
    double y = plcSnapshot.m_new_sensor1_Y;
    double z = plcSnapshot.m_new_sensor1_Z;
    double sensor = g_Global->g_measureResult[g_Global->singleChannelIndex];
    // 参数校验，无效时提前返回。
    if (sensor == 888888) {
        QMessageBox::warning(this, tr("Invalid Data"), tr("The current confocal data is invalid. Please remeasure."));
        return;
    }

    QFile file(ball_A_path);
    if (!file.open(QIODevice::Append | QIODevice::Text)) {
        QMessageBox::warning(this, tr("Write Failed"), tr("Failed to open file: ") + ball_A_path);
        return;
    }

    QTextStream out(&file);
    out << QString::number(x, 'f', 6) << " "
        << QString::number(y, 'f', 6) << " "
        << QString::number(z, 'f', 6) << " "
        << QString::number(sensor, 'f', 6) << "\n";
    file.close();
    // QTextStream out(&file);
    // out << QString::number(sensor, 'f', 6) << "\n";
    // file.close();

    ui->plainTextEdit->appendPlainText(QString::number(x, 'f', 3)+" "+QString::number(y, 'f', 3)+" "+QString::number(z, 'f', 3)+" "+QString::number(sensor, 'f', 3));
    // ui->plainTextEdit->appendPlainText(QString::number(sensor, 'f', 3));

}


void MainWindow::on_write_ball_A_done_clicked()
{
    QFile file(ball_A_path);
    if (!file.open(QIODevice::Append | QIODevice::Text)) {
        QMessageBox::warning(this, tr("閿欒"), tr("无法打开文件: ") + ball_A_path);
        return;
    }

    QTextStream out(&file);
    out << "\n";  // 添加空行
    file.close();

    ui->plainTextEdit->appendPlainText(
        systemTime() + " " + tr("宸叉坊鍔犲垎缁勯棿闅旇嚦 A.txt")
        );
}

void MainWindow::on_write_ball_B_points_clicked()
{
    double x = plcSnapshot.m_new_lizhu1_X;
    double y = plcSnapshot.m_new_sensor1_Y;
    double z = plcSnapshot.m_new_sensor1_Z;
    double sensor = g_Global->g_measureResult[g_Global->singleChannelIndex];

    // 参数校验，无效时提前返回。
    if (sensor == 888888) {
        QMessageBox::warning(this, tr("Invalid Data"), tr("The current confocal data is invalid. Please measure again."));
        return;
    }

    QFile file(ball_B_path);
    if (!file.open(QIODevice::Append | QIODevice::Text)) {
        QMessageBox::warning(this, tr("Write Failed"), tr("Failed to open file: ") + ball_B_path);
        return;
    }

    QTextStream out(&file);
    out << QString::number(x, 'f', 6) << " "
        << QString::number(y, 'f', 6) << " "
        << QString::number(z, 'f', 6) << " "
        << QString::number(sensor, 'f', 6) << "\n";
    file.close();

    ui->plainTextEdit->appendPlainText(
        QString::number(x, 'f', 3) + " " +
        QString::number(y, 'f', 3) + " " +
        QString::number(z, 'f', 3)
        );
}


void MainWindow::on_write_ball_B_done_clicked()
{
    QFile file(ball_B_path);
    if (!file.open(QIODevice::Append | QIODevice::Text)) {
        QMessageBox::warning(this, tr("閿欒"), tr("无法打开文件: ") + ball_B_path);
        return;
    }

    QTextStream out(&file);
    out << "\n";  // 添加空行
    file.close();

    ui->plainTextEdit->appendPlainText(
        systemTime() + " " + tr("宸叉坊鍔犲垎缁勯棿闅旇嚦 B.txt")
        );
}


void MainWindow::on_record_done_clicked()
{
    QString z_target = ui->Z_target->text().trimmed();
    target = z_target.toDouble();

    rotation_theta = plcSnapshot.m_new_angle;

    QFile file(rotation_angel);
    if (file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        QTextStream out(&file);
        out << QString::number(rotation_theta, 'f', 6) << "\n";  // 写入角度并换行
        file.close();

        ui->plainTextEdit->appendPlainText(systemTime());
        ui->plainTextEdit->appendPlainText(
            QString("Angle %1 written to %2")
                .arg(rotation_theta, 0, 'f', 6)
                .arg(rotation_angel)
            );

    } else {
        QMessageBox::warning(this, tr("wrong file"), tr("can not write into rotation_angel: ") + rotation_angel);
    }

    delta_rotation_theta = ui->comboBox_ball_number->currentText().toDouble();
    rotation_theta = rotation_theta + 360 / delta_rotation_theta;
    i_rote = i_rote + 1;

    qDebug() << rotation_theta;

    sendRegistersAndVar(target, address_z1, address_z2, 500);

    ui->plainTextEdit->appendPlainText(systemTime());
    ui->plainTextEdit->appendPlainText("Arm moving");

    QTimer *checkTimer = new QTimer(this);
    connect(checkTimer, &QTimer::timeout, this, [=]() {
        if (isArmInPosition()) {
            checkTimer->stop();
            checkTimer->deleteLater();

            ui->plainTextEdit->appendPlainText(systemTime());
            ui->plainTextEdit->appendPlainText("Rote Table moving");

            postPlcWriteHoldingRegisters(rotation_theta, 0); // 向转台角度寄存器写入目标角度

            QTimer::singleShot(500, this, [=]() {
                postPlcMoveToAbsolutePosition(1, 7); // 线圈触发启动
            });

            // 参数校验，无效时提前返回。
            if (i_rote == ui->comboBox_ball_number->currentText().toInt()) {
                ui->record_done->setEnabled(false);
                ui->write_ball_A_points->setEnabled(false);
                ui->write_ball_B_points->setEnabled(false);
                ui->write_ball_A_done->setEnabled(false);
                ui->write_ball_B_done->setEnabled(false);

                QMessageBox::warning(this, tr("Notice"), tr("calibration has been done"));
                return;
            }
        }
    });

    checkTimer->start(200);  // 姣?200 ms 妫€鏌ヤ竴娆?
}


bool MainWindow::isArmInPosition()
{
    qDebug() << qAbs(plcSnapshot.m_new_sensor1_Z - target);
    return qAbs(plcSnapshot.m_new_sensor1_Z - target) < tolerance_z_target;
}


void MainWindow::on_recover_clicked()
{
    QString z_target = ui->Z_target->text().trimmed();
    target = z_target.toDouble();

    ui->calculate->setEnabled(false);
    ui->load_calibration_data->setEnabled(false);
    ui->write_ball_A_points->setEnabled(false);
    ui->write_ball_B_points->setEnabled(false);
    ui->write_ball_A_done->setEnabled(false);
    ui->write_ball_B_done->setEnabled(false);
    ui->record_done->setEnabled(false);
    ui->recover->setEnabled(false);

    i_rote = 0;

    sendRegistersAndVar(target, address_z1, address_z2, 500);

    // QThread::msleep(5000);

    QTimer *checkTimer = new QTimer(this);
    connect(checkTimer, &QTimer::timeout, this, [=]() {
        if (isArmInPosition()) {
            checkTimer->stop();
            checkTimer->deleteLater();

            postPlcWriteHoldingRegisters(0, 0); // 向转台角度寄存器写入 0 度

            double var = 1;
            QTimer::singleShot(500, this, [=]() {
                postPlcMoveToAbsolutePosition(1, 7); // 线圈触发启动
            });

            QMessageBox::StandardButton reply;
            reply = QMessageBox::question(this, "Confirm",
                                          "Do you want to clear A.txt, B.txt and rotation_angel.txt?",
                                          QMessageBox::Yes | QMessageBox::No);
            if (reply != QMessageBox::Yes)
            {
                return; // 用户选择不清空
            }

            QFile fileA(ball_A_path);
            if (fileA.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
                fileA.close();
                ui->plainTextEdit->appendPlainText(systemTime());
                ui->plainTextEdit->appendPlainText("A.txt cleared successfully.");
            } else {
                QMessageBox::critical(this, "File Error", "Failed to clear A.txt.");
            }

            QFile fileB(ball_B_path);
            if (fileB.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
                fileB.close();
                ui->plainTextEdit->appendPlainText(systemTime());
                ui->plainTextEdit->appendPlainText("B.txt cleared successfully.");
            } else {
                QMessageBox::critical(this, "File Error", "Failed to clear B.txt.");
            }

            QFile fileRotation(rotation_angel);
            if (fileRotation.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
                fileRotation.close();
                ui->plainTextEdit->appendPlainText(systemTime());
                ui->plainTextEdit->appendPlainText("rotation_angel.txt cleared successfully.");
            } else {
                QMessageBox::critical(this, "File Error", "Failed to clear rotation_angel.txt.");
            }
        }
    });

    checkTimer->start(200);  // 姣?200 ms 妫€鏌ヤ竴娆?
}



void MainWindow::on_actionLink_clicked()
{
    if(!g_Global->g_isConnect)
    {
        loginDialog connectWin(this);
        int loginDialogReturn = connectWin.exec();

        if(loginDialogReturn == 0)
        {
            return;
        }
        else if(loginDialogReturn == 1 && confocalWorker)
        {
            // 异步投递任务到工作线程
            // 异步投递任务到工作线程
            QMetaObject::invokeMethod(confocalWorker,
                                      "setDeviceHandle",
                                      Qt::QueuedConnection,
                                      Q_ARG(int, g_Global->g_handle),
                                      Q_ARG(bool, g_Global->g_isCF3000Lite));
            QMetaObject::invokeMethod(confocalWorker,
                                      "verifyConnection",
                                      Qt::QueuedConnection);
        }
    }
    else
    {
        confTimer->stop();
        StatusBarTimer->stop();

        if(confocalWorker)
        {
            // 异步投递任务到工作线程
            QMetaObject::invokeMethod(confocalWorker,
                                      "closeDevice",
                                      Qt::QueuedConnection);
        }
    }
}


QString MainWindow::systemTime()
{
    return QDateTime::currentDateTime().toString("yyyy/MM/dd hh:mm:ss");
}

void MainWindow::on_ChineseEnglishSwitch_clicked()
{
    if(!g_Global->g_isEnglish)
    {
        g_Global->translator.load(":/qm/tr_en.qm");     //English
        qApp->installTranslator(&g_Global->translator);
        ui->retranslateUi(this);
        g_Global->g_isEnglish = true;

        //Refresh the Status Bar and menu Bar
        if(g_Global->g_isConnect)
        {
            setStatusBarValue();

            ui->actionLink->setText("Disconnect");
            if(g_Global->g_isStart)
                ui->actionStart->setText("Stop");
            else
                ui->actionStart->setText("Start");
        }
        else
        {
            statusBarInit();

            ui->actionLink->setText("Link");
        }
    }
    else
    {
        g_Global->translator.load(":/qm/tr_zh.qm");     // 中文
        qApp->installTranslator(&g_Global->translator);
        ui->retranslateUi(this);
        g_Global->g_isEnglish = false;

        // 参数校验，无效时提前返回。
        if(g_Global->g_isConnect)
        {
            setStatusBarValue();

            ui->actionLink->setText("鏂紑");
            if(g_Global->g_isStart)
                ui->actionStart->setText("鍋滄");
            else
                ui->actionStart->setText("鍚姩");
        }
        else
        {
            statusBarInit();

            ui->actionLink->setText("杩炴帴");
        }
    }
}


void MainWindow::getStatusBarResult()
{
    if(!confocalWorker || !g_Global->g_isConnect)
        return;

    // 异步投递任务到工作线程
    QMetaObject::invokeMethod(confocalWorker,
                              "requestStatusSnapshot",
                              Qt::QueuedConnection);
}

void MainWindow::setStatusBarValue()
{
    QString sdkVersionStr = sdkVersionResult;
    QString controlerVersionStr = controlerVersionResult;
    QString controlerSNStr = controlerSNResult;
    QString sensorSNStr = sensorSNResult;
    QString triggerModeStr = "xxxxx";
    switch(triggerMode)
    {
    case 0:triggerModeStr = tr(" 鍐呴儴杩炵画瑙﹀彂 ");break;
    case 1:triggerModeStr = tr(" 淇濈暀 ");break;
    case 2:triggerModeStr = tr(" 缂栫爜鍣ㄨЕ鍙?");break;
    case 3:triggerModeStr = tr(" 鍐呴儴瀹氭椂瑙﹀彂 ");break;
    case 4:triggerModeStr = tr(" 鍐呴儴鍗曟瑙﹀彂 ");break;
    default:triggerModeStr = "";
    }

    QString statusText = " ";
    statusText += tr(" 璁惧鐘舵€?") + triggerModeStr + "  |  ";
    statusText += tr(" SDK鐗堟湰:") + sdkVersionStr + "  |  ";
    statusText += tr(" 鎺у埗鍣ㄧ増鏈?") + controlerVersionStr + "  |  ";
    statusText += tr(" 鎺у埗鍣ㄥ簭鍒楀彿:") + controlerSNStr + "  |  ";
    statusText += tr(" 浼犳劅澶村簭鍒楀彿:") + sensorSNStr + "  |  ";
    statusText += tr(" 褰撳墠甯х巼:") + QString::number(curFrameRate) + " " + "Hz";

    lableStatus->setText(statusText);
    ui->statusbar->addWidget(lableStatus);
}


void MainWindow::SingleChannelModeInit()
{
    if(!confocalWorker || !g_Global->g_isConnect)
        return;

    // 异步投递任务到工作线程
    QMetaObject::invokeMethod(confocalWorker,
                              "requestInitialConfig",
                              Qt::QueuedConnection,
                              Q_ARG(int, g_Global->singleChannelIndex),
                              Q_ARG(int, ui->comboBoxLightSourceChannelSelect->currentIndex()),
                              Q_ARG(int, ui->comboBoxFilterAndSignalChannelSelect->currentIndex()));
}


void MainWindow::on_comboBoxSingleChannelMeasureModeSelect_currentIndexChanged(int index)
{
    Q_UNUSED(index)
    if(g_Global->g_isConnect)
    {
        int singleChannelMeasurModeValue = ui->comboBoxSingleChannelMeasureModeSelect->currentIndex();
        postConfocalSetIntParam(QString::fromLatin1(PARAM_CHANNEL_MEASURE_MODE),
                                g_Global->singleChannelIndex,
                                singleChannelMeasurModeValue,
                                tr("Measurement mode selection is successful!"));
    }
}


void MainWindow::on_checkBoxSingleChannelABS_stateChanged(int arg1)
{
    if(g_Global->g_isStart)
    {
        bool singleABSState = ui->checkBoxSingleChannelABS->isChecked();

        if(singleABSState)
        {
            ui->pushButtonSingleChannelModeZero->setEnabled(false); // Zero 澶辫兘
        }
        else
        {
            ui->pushButtonSingleChannelModeZero->setEnabled(true);
        }

        postConfocalSetIntParam(QString::fromLatin1(PARAM_CHANNEL_ABS_MODE),
                                g_Global->singleChannelIndex,
                                static_cast<int>(singleABSState),
                                singleABSState ? tr("ABS is on!") : tr("ABS is off!"));
    }
    else
    {
        ui->checkBoxSingleChannelABS->setChecked(false);
    }
}


void MainWindow::on_comboBoxSingleChannelSelect_currentIndexChanged(int index)
{
    if(g_Global->g_isConnect)
    {
        g_Global->singleChannelIndex = ui->comboBoxSingleChannelSelect->currentIndex();
    }
}


void MainWindow::on_checkBoxSingleChannelReverse_stateChanged(int arg1)
{
    if(g_Global->g_isStart)
    {
        bool singleChannelReverseState = ui->checkBoxSingleChannelReverse->isChecked();
        postConfocalSetIntParam(QString::fromLatin1(PARAM_CHANNEL_REVERSE_VALUE),
                                g_Global->singleChannelIndex,
                                static_cast<int>(singleChannelReverseState),
                                singleChannelReverseState ? tr("Reversal is on!") : tr("Reversal is off!"));
    }
    else
    {
        ui->checkBoxSingleChannelReverse->setChecked(false);
    }
}


void MainWindow::on_checkBoxSingleChannelMultiDistance_stateChanged(int arg1)
{
    if(g_Global->g_isConnect)
    {
        bool singleChannelMultDistance = ui->checkBoxSingleChannelMultiDistance->isChecked();
        postConfocalSetIntParam(QString::fromLatin1(PARAM_CHANNEL_MULT_DIS),
                                g_Global->singleChannelIndex,
                                static_cast<int>(singleChannelMultDistance),
                                tr("Multiple distance mode set successfully!"));
    }
    else
    {
        ui->checkBoxSingleChannelReverse->setChecked(false);
    }
}


void MainWindow::on_pushButtonSingleChannelMOdeDark_clicked()
{
    if(!g_Global->g_isConnect)
        return;

    if(g_Global->g_isStart)
    {
        ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Please close sample!"));
        return;
    }

    if(confocalWorker)
    {
        ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("姝ｅ湪鍚敤 dark 鍔熻兘"));
        // 异步投递任务到工作线程
        QMetaObject::invokeMethod(confocalWorker,
                                  "darkSignal",
                                  Qt::QueuedConnection,
                                  Q_ARG(int, g_Global->singleChannelIndex));
    }
}



void MainWindow::on_pushButtonSingleChannelModeZero_clicked()
{
    if(g_Global->g_isConnect && confocalWorker)
    {
        // 异步投递任务到工作线程
        QMetaObject::invokeMethod(confocalWorker,
                                  "zero",
                                  Qt::QueuedConnection,
                                  Q_ARG(int, g_Global->singleChannelIndex));
    }
}


void MainWindow::on_pushButtonSingleChannelModeGetLastResult_clicked()
{
    if(!g_Global->g_isConnect)
    {
        ui->plainTextEdit->appendPlainText(systemTime() + tr("Device not connected!"));
        return;
    }

    if(!g_Global->g_isStart)
    {
        ui->plainTextEdit->appendPlainText(systemTime() + tr("Please stop sample!"));
        return;
    }

    if(confocalWorker)
    {
        // 异步投递任务到工作线程
        QMetaObject::invokeMethod(confocalWorker,
                                  "requestLatestResult",
                                  Qt::QueuedConnection,
                                  Q_ARG(int, g_Global->singleChannelIndex));
    }
}


void MainWindow::lightModeInit()
{
    SingleChannelModeInit();
}


void MainWindow::communicationModeInit()
{
    SingleChannelModeInit();
}


void MainWindow::filterAndSignalModeInit()
{
    SingleChannelModeInit();
}


void MainWindow::importModeInit()
{
    SingleChannelModeInit();
}


void MainWindow::statusBarInit()
{
    QString statusText = " ";
    statusText += tr(" 璁惧鐘舵€? xxxxx") + "  |  ";
    statusText += tr(" SDK 鐗堟湰: xxxxx") + "  |  ";
    statusText += tr(" 鎺у埗鍣ㄧ増鏈? xxxxx") + "  |  ";
    statusText += tr(" 鎺у埗鍣ㄥ簭鍒楀彿: xxxxx") + "  |  ";
    statusText += tr(" 浼犳劅澶村簭鍒楀彿: xxxxx") + "  |  ";
    statusText += tr(" 褰撳墠甯х巼: xxxxx Hz");

    lableStatus->setText(statusText);
    ui->statusbar->addWidget(lableStatus);
}


void MainWindow::on_actionStart_clicked()
{
    if(g_Global->g_isConnect && confocalWorker)
    {
        if(!g_Global->g_isStart)
        {
            // 异步投递任务到工作线程
            QMetaObject::invokeMethod(confocalWorker,
                                      "startSample",
                                      Qt::QueuedConnection,
                                      Q_ARG(int, g_Global->singleChannelIndex));
        }
        else
        {
            // 异步投递任务到工作线程
            QMetaObject::invokeMethod(confocalWorker,
                                      "stopSample",
                                      Qt::QueuedConnection,
                                      Q_ARG(int, g_Global->singleChannelIndex));
        }
    }
    else
    {
        ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Please connect the device!"));
    }
}


void MainWindow::on_pushButton_clicked()
{
    factoryFilePath = QFileDialog::getExistingDirectory(this, tr("Select Directory"), "C:", QFileDialog::ShowDirsOnly);
    if(factoryFilePath.isEmpty())
        return;

    ui->lineEdit_CF3000LiteFilePath->setText(factoryFilePath);

    if(confocalWorker)
    {
        // 异步投递任务到工作线程
        QMetaObject::invokeMethod(confocalWorker,
                                  "setFactoryFilePath",
                                  Qt::QueuedConnection,
                                  Q_ARG(QString, factoryFilePath));
    }
}


// void MainWindow::timerEvent(QTimerEvent *event)
// {
//     if(event->timerId() == timerData)
//     {
//         ui->lcdNumberSingleChannelResult->display(g_Global->g_measureResult[g_Global->singleChannelIndex]);
//     }
// }


void MainWindow::on_comboBoxExposureTime_currentIndexChanged(int index)
{
    if(g_Global->g_isConnect)
    {
        int exposureTime = ui->comboBoxExposureTime->currentIndex();
        int exposureValue = 20;
        switch(exposureTime)
        {
        case 0: exposureValue = 20; break;
        case 1: exposureValue = 50; break;
        case 2: exposureValue = 100; break;
        case 3: exposureValue = 200; break;
        case 4: exposureValue = 400; break;
        case 5: exposureValue = 700; break;
        case 6: exposureValue = 1000; break;
        case 7: exposureValue = 1500; break;
        default: break;
        }
        postConfocalSetIntParam(QString::fromLatin1(PARAM_EXPOSURE_TIME), 0, exposureValue);
    }
}


void MainWindow::on_comboBoxLightMode_currentIndexChanged(int index)
{
    Q_UNUSED(index)
    int LightMode = ui->comboBoxLightMode->currentIndex();
    if(LightMode == 0)
    {
        ui->comboBoxLightReference->setEnabled(false);
        ui->horizontalSliderLightValue->setEnabled(true);
    }
    else
    {
        ui->comboBoxLightReference->setEnabled(true);
        ui->horizontalSliderLightValue->setEnabled(false);
    }

    if(g_Global->g_isConnect)
    {
        int lightSourceChannelIndex = ui->comboBoxLightSourceChannelSelect->currentIndex();
        postConfocalSetIntParam(QString::fromLatin1(PARAM_AUTO_LIGHT),
                                lightSourceChannelIndex,
                                LightMode,
                                tr("Dimming mode is set successfully!"));
    }
}


void MainWindow::on_comboBoxLightReference_currentIndexChanged(int index)
{
    Q_UNUSED(index)
    if(g_Global->g_isConnect)
    {
        int lightSourceChannelIndex = ui->comboBoxLightSourceChannelSelect->currentIndex();
        int LightReference = ui->comboBoxLightReference->currentIndex();
        postConfocalSetIntParam(QString::fromLatin1(PARAM_AUTO_LIGHT_MODE),
                                lightSourceChannelIndex,
                                LightReference,
                                tr("Automatic dimming reference setting successful!"));
    }
}


void MainWindow::on_horizontalSliderLightValue_valueChanged(int value)
{
    if(g_Global->g_isConnect)
    {
        int lightSourceChannelIndex = ui->comboBoxLightSourceChannelSelect->currentIndex();
        postConfocalSetFloatParam(QString::fromLatin1(PARAM_LIGHT_VALUE),
                                  lightSourceChannelIndex,
                                  static_cast<float>(value));
    }
}

void MainWindow::on_comboBoxCommunicationMode_currentIndexChanged(int index)
{
    Q_UNUSED(index)
    if(g_Global->g_isConnect)
    {
        postConfocalSetIntParam(QString::fromLatin1(PARAM_RSXXX_SELECT),
                                0,
                                ui->comboBoxCommunicationMode->currentIndex(),
                                tr("The communication mode is set successfully!"));
    }
}

void MainWindow::on_comboBoxSerialPort_currentIndexChanged(int index)
{
    Q_UNUSED(index)
    if(g_Global->g_isConnect)
    {
        postConfocalSetIntParam(QString::fromLatin1(PARAM_RSXXX_PROTOCOL),
                                0,
                                ui->comboBoxSerialPort->currentIndex(),
                                tr("The serial port protocol is set successfully!"));
    }
}

void MainWindow::on_comboBoxDataFormat_currentIndexChanged(int index)
{
    Q_UNUSED(index)
    if(g_Global->g_isConnect)
    {
        postConfocalSetIntParam(QString::fromLatin1(PARAM_RSXXX_DATA_FORMAT),
                                0,
                                ui->comboBoxDataFormat->currentIndex(),
                                tr("The data format is set successfully!"));
    }
}

void MainWindow::on_comboBoxBaudRate_currentIndexChanged(int index)
{
    Q_UNUSED(index)
    if(g_Global->g_isConnect)
    {
        postConfocalSetIntParam(QString::fromLatin1(PARAM_RSXXX_BAUDRATE),
                                0,
                                ui->comboBoxBaudRate->currentIndex(),
                                tr("The baud rate is set successfully!"));
    }
}

void MainWindow::on_comboBoxVerify_currentIndexChanged(int index)
{
    Q_UNUSED(index)
    if(g_Global->g_isConnect)
    {
        postConfocalSetIntParam(QString::fromLatin1(PARAM_RSXXX_PARITY),
                                0,
                                ui->comboBoxVerify->currentIndex(),
                                tr("Odd-even is set successfully!"));
    }
}

void MainWindow::on_comboBoxExportSignalSelect_currentIndexChanged(int index)
{
    Q_UNUSED(index)
    if(g_Global->g_isConnect)
    {
        int filterAndSignalChannelIndex = ui->comboBoxFilterAndSignalChannelSelect->currentIndex();
        int signalSelectIndex = ui->comboBoxExportSignalSelect->currentIndex();

        postConfocalSetIntParam(QString::fromLatin1(PARAM_SELECT_SIGNAL),
                                filterAndSignalChannelIndex,
                                signalSelectIndex,
                                tr("Signal selection is set successfully!"));
    }
}


void MainWindow::on_comboBoxSignalSortSelect_currentIndexChanged(int index)
{
    Q_UNUSED(index)
    if(g_Global->g_isConnect)
    {
        int filterAndSignalChannelIndex = ui->comboBoxFilterAndSignalChannelSelect->currentIndex();
        int signalSortSelectIndex = ui->comboBoxSignalSortSelect->currentIndex();

        postConfocalSetIntParam(QString::fromLatin1(PARAM_SIGNAL_SORT),
                                filterAndSignalChannelIndex,
                                signalSortSelectIndex,
                                tr("Sort selection setting is successful!"));
    }
}


void MainWindow::on_doubleSpinBoxSignalThresholdValue_valueChanged(double arg1)
{
    Q_UNUSED(arg1)
    if(g_Global->g_isConnect)
    {
        int filterAndSignalChannelIndex = ui->comboBoxFilterAndSignalChannelSelect->currentIndex();
        float signalThresholdValue = static_cast<float>(ui->doubleSpinBoxSignalThresholdValue->value());

        postConfocalSetFloatParam(QString::fromLatin1(PARAM_SIGNAL_DETECT_THRE),
                                  filterAndSignalChannelIndex,
                                  signalThresholdValue,
                                  tr("The signal detection threshold is set successfully!"));
    }
}


void MainWindow::on_doubleSpinBoxSignalRatio_valueChanged(double arg1)
{
    Q_UNUSED(arg1)
    if(g_Global->g_isConnect)
    {
        int filterAndSignalChannelIndex = ui->comboBoxFilterAndSignalChannelSelect->currentIndex();
        float signalRatioValue = static_cast<float>(ui->doubleSpinBoxSignalRatio->value());

        postConfocalSetFloatParam(QString::fromLatin1(PARAM_SIGNAL_CAL_RATIO),
                                  filterAndSignalChannelIndex,
                                  signalRatioValue,
                                  tr("The signal calculation ratio is set successfully!"));
    }
}


void MainWindow::on_spinBoxSignalSensitivity_valueChanged(int arg1)
{
    Q_UNUSED(arg1)
    if(g_Global->g_isConnect)
    {
        int filterAndSignalChannelIndex = ui->comboBoxFilterAndSignalChannelSelect->currentIndex();
        int signalSensitivityValue = ui->spinBoxSignalSensitivity->value();

        postConfocalSetIntParam(QString::fromLatin1(PARAM_SIGNAL_DETECT_SENSITIVITY),
                                filterAndSignalChannelIndex,
                                signalSensitivityValue,
                                tr("The signal sensitivity is set successfully!"));
    }
}


void MainWindow::on_spinBoxSignalCount_valueChanged(int arg1)
{
    Q_UNUSED(arg1)
    if(g_Global->g_isConnect)
    {
        int filterAndSignalChannelIndex = ui->comboBoxFilterAndSignalChannelSelect->currentIndex();
        int signalCount = ui->spinBoxSignalCount->value();

        postConfocalSetIntParam(QString::fromLatin1(PARAM_SIGNAL_DETECT_NUM),
                                filterAndSignalChannelIndex,
                                signalCount,
                                tr("The number of signals is set successfully!"));
    }
}


void MainWindow::on_spinBoxSignalMinNum_valueChanged(int arg1)
{
    Q_UNUSED(arg1)
    if(g_Global->g_isConnect)
    {
        int filterAndSignalChannelIndex = ui->comboBoxFilterAndSignalChannelSelect->currentIndex();
        int signalMinNum = ui->spinBoxSignalMinNum->value();

        postConfocalSetIntParam(QString::fromLatin1(PARAM_SIGNAL_MINI_POINTS),
                                filterAndSignalChannelIndex,
                                signalMinNum,
                                tr("The minimum number of points is set successfully!"));
    }
}


void MainWindow::on_spinBoxSignalSmoothlength_valueChanged(int arg1)
{
    Q_UNUSED(arg1)
    if(g_Global->g_isConnect)
    {
        int filterAndSignalChannelIndex = ui->comboBoxFilterAndSignalChannelSelect->currentIndex();
        int signalSmoothlength = ui->spinBoxSignalSmoothlength->value();

        postConfocalSetIntParam(QString::fromLatin1(PARAM_SIGNAL_SMOOTH),
                                filterAndSignalChannelIndex,
                                signalSmoothlength,
                                tr("Signal smoothing length set successfully!"));
    }
}


void MainWindow::on_spinBoxSignalMedianFilter_valueChanged(int arg1)
{
    Q_UNUSED(arg1)
    if(g_Global->g_isConnect)
    {
        int filterAndSignalChannelIndex = ui->comboBoxFilterAndSignalChannelSelect->currentIndex();
        int signalMedianFilter = ui->spinBoxSignalMedianFilter->value();

        postConfocalSetIntParam(QString::fromLatin1(PARAM_MOVING_MEDIAN_FILTER),
                                filterAndSignalChannelIndex,
                                signalMedianFilter,
                                tr("The median filter depth is set successfully!"));
    }
}


void MainWindow::on_spinBoxSignalAverageFilter_valueChanged(int arg1)
{
    Q_UNUSED(arg1)
    if(g_Global->g_isConnect)
    {
        int filterAndSignalChannelIndex = ui->comboBoxFilterAndSignalChannelSelect->currentIndex();
        int signalAverageFilter = ui->spinBoxSignalAverageFilter->value();

        postConfocalSetIntParam(QString::fromLatin1(PARAM_MOVING_AVG_FILTER),
                                filterAndSignalChannelIndex,
                                signalAverageFilter,
                                tr("Sliding average filter depth is set successfully!!"));
    }
}

void MainWindow::on_comboBoxTriggerMode_currentIndexChanged(int index)
{
    if(!g_Global->g_isConnect)
        return;

    ui->gb_frameSetting->setEnabled(index == 3);  // 内部触发时可以设置帧率
    postConfocalSetIntParam(QString::fromLatin1(PARAM_TRIGGER_MODE),
                            0,
                            index,
                            tr("Trigger mode set successfully!"));
}


void MainWindow::on_comboBoxEncodeWorkMode_currentIndexChanged(int index)
{
    Q_UNUSED(index)
    if(g_Global->g_isConnect)
    {
        postConfocalSetIntParam(QString::fromLatin1(PARAM_ENCODER_WORKING_MODE),
                                0,
                                ui->comboBoxEncodeWorkMode->currentIndex(),
                                tr("Encoder operating mode set successfully!"));
    }
}

void MainWindow::on_comboBoxEncodeInputMode_currentIndexChanged(int index)
{
    Q_UNUSED(index)
    if(g_Global->g_isConnect)
    {
        postConfocalSetIntParam(QString::fromLatin1(PARAM_ENCODER_INPUT_MODE),
                                0,
                                ui->comboBoxEncodeInputMode->currentIndex(),
                                tr("Encoder input mode set successfully!"));
    }
}

void MainWindow::on_spinBoxEncodeDivision_valueChanged(int arg1)
{
    Q_UNUSED(arg1)
    if(g_Global->g_isConnect)
    {
        int encodeDivisionValue = ui->spinBoxEncodeDivision->value();
        postConfocalSetIntParam(QString::fromLatin1(PARAM_ENCODER_DIVISION),
                                0,
                                encodeDivisionValue,
                                tr("Encode dividion value set successfully!"));
    }
}






void MainWindow::on_pushButton_display_clicked()
{
    // 直接在主线程创建 DLP（对齐 qt_camera），避免与相机 USB 操作跨线程冲突导致 I2C NAK。
    if(dlp1)
    {
        ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("DLP projector has already been initialized."));
        return;
    }

    try
    {
        dlp1 = new DLP;
    }
    catch(const std::exception& e)
    {
        dlp1 = nullptr;
        dlpInitialized = false;
        QMessageBox::warning(this, QStringLiteral("Warning"),
                             tr("Initialize DLP") + QStringLiteral(": ") + QString::fromLocal8Bit(e.what()));
        return;
    }
    catch(...)
    {
        dlp1 = nullptr;
        dlpInitialized = false;
        QMessageBox::warning(this, QStringLiteral("Warning"),
                             tr("Initialize DLP") + QStringLiteral(": ") + tr("Unknown exception."));
        return;
    }

    dlpInitialized = true;

    QSignalBlocker blockerOperatingMode(ui->comboBox_DLP_OperatingModeSelect);
    QSignalBlocker blockerSlider(ui->horizontalSlider);

    ui->comboBox_DLP_OperatingModeSelect->clear();
    QString currentOperatingModeName;
    for(const auto& item : dlp1->OperatingModeList)
    {
        const QString name = QString::fromStdString(item.first);
        ui->comboBox_DLP_OperatingModeSelect->addItem(name);
        if(dlp1->Current_OperatingMode == item.second)
            currentOperatingModeName = name;
    }
    if(!currentOperatingModeName.isEmpty())
        ui->comboBox_DLP_OperatingModeSelect->setCurrentText(currentOperatingModeName);

    ui->horizontalSlider->setMaximum(static_cast<int>(dlp1->MaxBlueLedCurrent));
    ui->horizontalSlider->setValue(static_cast<int>(dlp1->BlueLedCurrent));
    ui->label_current->setText(QStringLiteral("Current: ") + QString::number(static_cast<int>(dlp1->BlueLedCurrent)));
    ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("DLP projector initialized."));
}


void MainWindow::on_pushButton_OperatingModeSelect_clicked()
{
    if(!dlp1)
    {
        QMessageBox::warning(this,
                             QStringLiteral("Warning"),
                             QStringLiteral("Please connect DLP first"));
        return;
    }

    const std::string key = ui->comboBox_DLP_OperatingModeSelect->currentText().toStdString();
    const auto iter = dlp1->OperatingModeList.find(key);
    if(iter == dlp1->OperatingModeList.end())
    {
        QMessageBox::warning(this, QStringLiteral("Warning"),
                             tr("Select DLP operating mode") + QStringLiteral(": ") +
                             tr("Unknown operating mode: %1").arg(ui->comboBox_DLP_OperatingModeSelect->currentText()));
        return;
    }

    if(!dlp1->SelectOperatingMode(iter->second))
    {
        QMessageBox::warning(this, QStringLiteral("Warning"),
                             tr("Select DLP operating mode") + QStringLiteral(": ") +
                             tr("DLP command write failed. Please check projector connection."));
    }
}


void MainWindow::on_pushButton_ProjectOnce_clicked()
{
    if(!dlp1)
    {
        QMessageBox::warning(this,
                             QStringLiteral("Warning"),
                             QStringLiteral("Please connect DLP first"));
        return;
    }

    // 投影一次：先切到内部图案模式，再以 times=1 发 PC_START，播放一遍即停。
    if(!dlp1->SelectOperatingMode(DLPC347X_OM_SENS_INTERNAL_PATTERN))
    {
        QMessageBox::warning(this, QStringLiteral("Warning"),
                             tr("Start DLP projection") + QStringLiteral(": ") +
                             tr("Failed to switch to internal pattern mode. Please check projector connection."));
        return;
    }
    if(!dlp1->InternalPatternControl(DLPC347X_PC_START, 1))
    {
        QMessageBox::warning(this, QStringLiteral("Warning"),
                             tr("Start DLP projection") + QStringLiteral(": ") +
                             tr("DLP command write failed. Please check projector connection."));
    }
}


void MainWindow::on_pushButton_ProjectContinuous_clicked()
{
    if(!dlp1)
    {
        QMessageBox::warning(this,
                             QStringLiteral("Warning"),
                             QStringLiteral("Please connect DLP first"));
        return;
    }

    // 持续投影：times=0，底层按 0xFF(无限循环)处理，直到点停止。
    if(!dlp1->SelectOperatingMode(DLPC347X_OM_SENS_INTERNAL_PATTERN))
    {
        QMessageBox::warning(this, QStringLiteral("Warning"),
                             tr("Start DLP projection") + QStringLiteral(": ") +
                             tr("Failed to switch to internal pattern mode. Please check projector connection."));
        return;
    }
    if(!dlp1->InternalPatternControl(DLPC347X_PC_START, 0))
    {
        QMessageBox::warning(this, QStringLiteral("Warning"),
                             tr("Start DLP projection") + QStringLiteral(": ") +
                             tr("DLP command write failed. Please check projector connection."));
    }
}


void MainWindow::on_pushButton_ProjectStop_clicked()
{
    if(!dlp1)
    {
        QMessageBox::warning(this,
                             QStringLiteral("Warning"),
                             QStringLiteral("Please connect DLP first"));
        return;
    }

    // 停止投影：发 PC_STOP 结束当前图案序列。
    if(!dlp1->InternalPatternControl(DLPC347X_PC_STOP, 0))
    {
        QMessageBox::warning(this, QStringLiteral("Warning"),
                             tr("Stop DLP projection") + QStringLiteral(": ") +
                             tr("DLP command write failed. Please check projector connection."));
    }
}


void MainWindow::registerCameraWindowCleanup(CamDisplay *window)
{
    if(!window)
        return;

    // 用属性标记避免对同一窗口重复连接（每次打开相机都会遍历整张列表）。
    if(window->property("cleanupRegistered").toBool())
        return;
    window->setProperty("cleanupRegistered", true);

    // destroyed 在对象析构时发出；此时不能再调用 CamDisplay 成员，只按裸指针从列表移除。
    connect(window, &QObject::destroyed, this, [this](QObject *obj) {
        auto *cam = static_cast<CamDisplay *>(obj);
        auto &list = display_widget_list;
        list.erase(std::remove(list.begin(), list.end(), cam), list.end());
        if(display_widget == cam)
            display_widget = nullptr;
    });
}

void MainWindow::on_pushButton_testui_clicked()
{
    if (ui->checkBox_open_all->isChecked())
    {
        display_widget_list.clear();

        for (int i = 0; i < ui->comboBox_device_list->count(); ++i)
        {
            display_widget = new CamDisplay(this, i);
            display_widget->setAttribute(Qt::WA_DeleteOnClose);
            ui->mdiArea->addSubWindow(display_widget);

            display_widget->setWindowTitle(
                ui->comboBox_device_list->itemText(i) +
                QString::fromLocal8Bit("-璁惧") +
                QString::number(i)
                );

            display_widget->show();
            display_widget_list.push_back(display_widget);
        }
    }
    else
    {
        display_widget = new CamDisplay(this,
                                        ui->comboBox_device_list->currentIndex());
        display_widget->setAttribute(Qt::WA_DeleteOnClose);
        ui->mdiArea->addSubWindow(display_widget);

        display_widget->setWindowTitle(
            ui->comboBox_device_list->currentText() +
            QString::fromLocal8Bit("-璁惧") +
            QString::number(ui->comboBox_device_list->currentIndex())
            );

        display_widget->show();
        // 单个打开的相机窗口同样纳入列表，否则“全部勾选/同时采集/同时存图”等批量操作会遗漏它。
        display_widget_list.push_back(display_widget);
    }

    // 窗口设置了 WA_DeleteOnClose，关闭即销毁；销毁时把它从列表移除，避免留下野指针。
    for (CamDisplay *w : display_widget_list)
        registerCameraWindowCleanup(w);

    ui->mdiArea->tileSubWindows();
}


void MainWindow::closeEvent(QCloseEvent *event)
{
    shutdownModuleThreads();

    if(event)
        event->accept();

    QMainWindow::closeEvent(event);
}


void MainWindow::on_pushButton_refresh_device_clicked()
{
    ui->comboBox_device_list->clear();

    GxIAPICPP::gxdeviceinfo_vector vectorDeviceInfo1;
    IGXFactory::GetInstance().UpdateDeviceList(1000, vectorDeviceInfo1);

    vectorDeviceInfo = vectorDeviceInfo1;

    for (int i = 0; i < vectorDeviceInfo.size(); ++i)
    {
        ui->comboBox_device_list->addItem(vectorDeviceInfo[i].GetDisplayName().c_str());
    }
}


void MainWindow::on_pushButton_snapall_clicked()
{
    for (int i = 0; i < display_widget_list.size(); ++i)
    {
        display_widget_list[i]->soft_trigger();
    }
}


void MainWindow::on_pushButton_saveonce_all_clicked()
{
    for (int i = 0; i < display_widget_list.size(); ++i)
    {
        display_widget_list[i]->SaveOnce();
    }
}


void MainWindow::on_pushButton_checkall_countsave_clicked()
{
    if(display_widget_list.empty())
        return;

    // 切换逻辑：只要有一个窗口未勾选，就全部勾上；若当前已全部勾选，则全部取消。
    // 这样无论之前是手动勾的还是按钮勾的，都能正确判断并反向操作。
    bool allChecked = true;
    for (int i = 0; i < display_widget_list.size(); ++i)
    {
        if(!display_widget_list[i]->isCountSaveChecked())
        {
            allChecked = false;
            break;
        }
    }

    const bool target = !allChecked;
    for (int i = 0; i < display_widget_list.size(); ++i)
    {
        display_widget_list[i]->setCountSaveChecked(target);
    }

    ui->pushButton_checkall_countsave->setText(target ? QStringLiteral("取消全部勾选")
                                                      : QStringLiteral("全部勾选计数"));
}


void MainWindow::on_pushButton_checkall_save_clicked()
{
    if(display_widget_list.empty())
        return;

    // 切换逻辑：同上，全部已勾则取消，否则全部勾选。
    bool allChecked = true;
    for (int i = 0; i < display_widget_list.size(); ++i)
    {
        if(!display_widget_list[i]->isSaveChecked())
        {
            allChecked = false;
            break;
        }
    }

    const bool target = !allChecked;
    for (int i = 0; i < display_widget_list.size(); ++i)
    {
        display_widget_list[i]->setSaveChecked(target);
    }

    ui->pushButton_checkall_save->setText(target ? QStringLiteral("取消全部勾选")
                                                 : QStringLiteral("全部勾选存图"));
}


void MainWindow::getFiles(string path, vector<string>& files)
{
    QDir dir(QString::fromLocal8Bit(path.c_str()));
    if(!dir.exists())
        return;

    const QStringList filters = QStringList()
                                << QStringLiteral("*.bmp")
                                << QStringLiteral("*.png")
                                << QStringLiteral("*.jpg")
                                << QStringLiteral("*.jpeg")
                                << QStringLiteral("*.tif")
                                << QStringLiteral("*.tiff");

    QFileInfoList fileInfos = dir.entryInfoList(filters,
                                                QDir::Files | QDir::Readable,
                                                QDir::Name | QDir::IgnoreCase);

    QCollator collator;
    collator.setNumericMode(true);
    std::sort(fileInfos.begin(), fileInfos.end(), [&collator](const QFileInfo& left, const QFileInfo& right) {
        return collator.compare(left.fileName(), right.fileName()) < 0;
    });

    for(const QFileInfo& fileInfo : fileInfos)
        files.push_back(fileInfo.absoluteFilePath().toLocal8Bit().constData());
}


bool MainWindow::read_imgs(string path, vector<cv::Mat>& imgs)
{
    imgs.clear();

    vector<string> files;
    getFiles(path, files);
    if(files.empty())
        return false;

    for (size_t i = 0; i < files.size(); i++)
    {
        std::cout << files[i] << std::endl;
        cv::Mat image = cv::imread(files[i], CV_8U); // 鎸?8 浣嶇伆搴﹀浘璇诲彇鍥惧儚
        if(image.empty())
        {
            imgs.clear();
            return false;
        }

        imgs.push_back(image);
    }

    return !imgs.empty();
}


void MainWindow::on_pushButton_testpartteb_clicked()
{
    const QString baseRoot = ui->base_root_dlp->text().trimmed();
    if(baseRoot.isEmpty())
    {
        QMessageBox::warning(this,
                             QStringLiteral("Warning"),
                             QStringLiteral("Please select the DLP pattern root first"));
        return;
    }

    QDir patternDir(baseRoot);
    if(!patternDir.exists())
    {
        QMessageBox::warning(this,
                             QStringLiteral("Warning"),
                             QStringLiteral("Invalid DLP pattern root"));
        return;
    }

    const int groupCount = ui->groupnumber->value();
    const int pictureCount = ui->picturenumber->value();
    const int expectedImageCount = groupCount * pictureCount;
    if(expectedImageCount <= 0)
    {
        QMessageBox::warning(this,
                             QStringLiteral("Warning"),
                             QStringLiteral("Group count and picture count must be greater than 0"));
        return;
    }

    // 参数校验，无效时提前返回。
    if(pictureCount > 6)
    {
        QMessageBox::warning(this,
                             QStringLiteral("Warning"),
                             QStringLiteral("Picture count per group cannot exceed 6"));
        return;
    }

    vector<cv::Mat> imgs;
    if(!read_imgs(baseRoot.toLocal8Bit().constData(), imgs) || imgs.empty())
    {
        QMessageBox::warning(this,
                             QStringLiteral("Warning"),
                             QStringLiteral("No readable images found in this root"));
        return;
    }

    if(static_cast<int>(imgs.size()) != expectedImageCount)
    {
        QMessageBox::warning(this,
                             QStringLiteral("Warning"),
                             QStringLiteral("Image count does not match group count * picture count"));
        return;
    }

    for(size_t i = 0; i < imgs.size(); ++i)
    {
        if(imgs[i].cols < DLP4710_WIDTH || imgs[i].rows < 1)
        {
            QMessageBox::warning(this,
                                 QStringLiteral("Warning"),
                                 QStringLiteral("Invalid image size. Width must be at least ") + QString::number(DLP4710_WIDTH));
            return;
        }
    }

    vector<vector<cv::Mat>> sets;
    sets.reserve(groupCount);
    int imageIndex = 0;
    for(int groupIndex = 0; groupIndex < groupCount; ++groupIndex)
    {
        vector<cv::Mat> set;
        set.reserve(pictureCount);
        for(int pictureIndex = 0; pictureIndex < pictureCount; ++pictureIndex)
            set.push_back(imgs[imageIndex++]);
        sets.push_back(set);
    }

    if(!dlp1)
    {
        QMessageBox::warning(this,
                             QStringLiteral("Warning"),
                             QStringLiteral("Please connect DLP first"));
        return;
    }

    // 从界面读取每张图案的明亮/前黑暗/后黑暗时间(微秒)，随烧录任务一起下发。
    const int illuminationTimeUs = ui->spinBox_illum_time->value();
    const int preDarkTimeUs      = ui->spinBox_predark_time->value();
    const int postDarkTimeUs     = ui->spinBox_postdark_time->value();

    // 烧录进度回调：在主线程内同步触发，直接刷新进度条 + processEvents 强制重绘。
    dlp1->SetProgressCallback([this](int percent, const std::string& stage) {
        if(percent < 0)
        {
            ui->progressBar_burn->setVisible(false);
        }
        else
        {
            ui->progressBar_burn->setVisible(true);
            ui->progressBar_burn->setValue(percent);
            ui->progressBar_burn->setFormat(QString::fromStdString(stage) + QStringLiteral(" %p%"));
        }
        QApplication::processEvents();
    });

    // 主线程直接同步烧录（会阻塞界面，属已知可接受代价）。
    const bool ok = dlp1->Input1DImageSet_8bit_V(sets, illuminationTimeUs, preDarkTimeUs, postDarkTimeUs);

    dlp1->SetProgressCallback(nullptr);

    if(!ok)
    {
        ui->progressBar_burn->setVisible(false);
        QMessageBox::warning(this, QStringLiteral("Warning"),
                             tr("Program DLP 8bit pattern set") + QStringLiteral(": ") +
                             tr("DLP command write failed. Please check projector connection and image set."));
        return;
    }

    // 完成后延时隐藏，让用户看到 100%。
    QTimer::singleShot(1500, this, [this]() {
        if(ui && ui->progressBar_burn)
            ui->progressBar_burn->setVisible(false);
    });
    ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("DLP 8bit pattern set programming finished."));
}


QString MainWindow::normalizedPathForSl(const QString& path) const
{
    return QDir::fromNativeSeparators(path.trimmed());
}

void MainWindow::appendSlLog(const QString& msg)
{
    if(ui && ui->plainTextEdit)
        ui->plainTextEdit->appendPlainText(systemTime() + " [SL] " + msg);
}

/// 点云显示部分程序 ///
void MainWindow::on_saverootpath_confirm_clicked()
{
    const QString currentPath = ui->saverootpath->text().trimmed();
    const QString selectedPath = QFileDialog::getExistingDirectory(
        this,
        QStringLiteral("Select SL data root"),
        currentPath.isEmpty() ? QDir::currentPath() : currentPath,
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

    if(!selectedPath.isEmpty())
        ui->saverootpath->setText(QDir::toNativeSeparators(selectedPath));
}

void MainWindow::on_select_subpath_clicked()
{
    if(!ui)
        return;

    const QString saveRoot = normalizedPathForSl(ui->saverootpath->text());
    if(saveRoot.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("SL"), QStringLiteral("Please set saverootpath first."));
        return;
    }

    QDir rootDir(saveRoot);
    if(!rootDir.exists())
    {
        QMessageBox::warning(this, QStringLiteral("SL"), QStringLiteral("saverootpath does not exist."));
        return;
    }

    const QFileInfoList dirInfos = rootDir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
    if(dirInfos.isEmpty())
    {
        QMessageBox::information(this, QStringLiteral("SL"), QStringLiteral("No subfolders found under saverootpath."));
        return;
    }

    // 筛选并多选可用子目录（满足 L/R 图像与数量约束）。
    // 筛选并多选可用子目录（满足 L/R 图像与数量约束）。
    const int expectCount = ui ? (ui->N->text().toInt() * ui->Tsize->text().toInt()) : 0;

    QDialog dlg(this);
    dlg.setWindowTitle(QStringLiteral("Select subfolders"));
    dlg.resize(520, 420);

    QVBoxLayout* layout = new QVBoxLayout(&dlg);
    QLabel* tip = new QLabel(QStringLiteral("Select one or more subfolders under saverootpath:"), &dlg);
    QListWidget* list = new QListWidget(&dlg);
    list->setSelectionMode(QAbstractItemView::MultiSelection);

    const QStringList filters = QStringList()
            << QStringLiteral("*.bmp") << QStringLiteral("*.png")
            << QStringLiteral("*.jpg") << QStringLiteral("*.jpeg")
            << QStringLiteral("*.tif") << QStringLiteral("*.tiff");

    for(const QFileInfo& info : dirInfos)
    {
        const QDir subDir(info.absoluteFilePath());
        const QDir dirL(subDir.filePath(QStringLiteral("L")));
        const QDir dirR(subDir.filePath(QStringLiteral("R")));

        const QFileInfoList filesL = dirL.entryInfoList(filters, QDir::Files, QDir::Name);
        const QFileInfoList filesR = dirR.entryInfoList(filters, QDir::Files, QDir::Name);

        bool valid = true;
        if(!dirL.exists() || !dirR.exists())
            valid = false;
        else if(filesL.isEmpty() || filesR.isEmpty())
            valid = false;
        else if(filesL.size() != filesR.size())
            valid = false;
        else if(expectCount > 0 && filesL.size() != expectCount)
            valid = false;

        if(!valid)
            continue;

        QListWidgetItem* item = new QListWidgetItem(info.fileName(), list);
        item->setData(Qt::UserRole, info.absoluteFilePath());
    }

    QDialogButtonBox* box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
    layout->addWidget(tip);
    layout->addWidget(list);
    layout->addWidget(box);

    connect(box, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(box, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    if(dlg.exec() != QDialog::Accepted)
        return;

    selectedSlSubpaths.clear();
    const QList<QListWidgetItem*> selected = list->selectedItems();
    for(QListWidgetItem* item : selected)
        selectedSlSubpaths.push_back(normalizedPathForSl(item->data(Qt::UserRole).toString()));

    if(selectedSlSubpaths.isEmpty())
    {
        QMessageBox::information(this,
                                 QStringLiteral("SL"),
                                 QStringLiteral("No valid subfolders selected."));
    }

    if(ui->patterns_subpath)
    {
        if(selectedSlSubpaths.isEmpty())
            ui->patterns_subpath->setText(QString());
        else
            ui->patterns_subpath->setText(QStringLiteral("%1 folders selected").arg(selectedSlSubpaths.size()));
    }

    appendSlLog(QStringLiteral("selected subfolders: %1").arg(selectedSlSubpaths.size()));
}


// 筛选并多选可用子目录（满足 L/R 图像与数量约束）。
void MainWindow::on_Calculate_SL_clicked()
{
    if(slReconstructBusy)
    {
        appendSlLog(QStringLiteral("SL is still running, please wait..."));
        return;
    }

    // 批量执行 SL 重建并逐条回写结果。
    SlReconstructParams baseParams;
    baseParams.saveRootPath = normalizedPathForSl(ui->saverootpath->text());
    baseParams.calibFilename = ui->calib_filename->text().trimmed();
    baseParams.saveName = ui->savename->text().trimmed();

    baseParams.rows = ui->rows_SL->text().toInt();
    baseParams.cols = ui->cols_SL->text().toInt();
    baseParams.cutWidth = ui->kernel_size->text().toInt();
    baseParams.N = ui->N->text().toInt();
    baseParams.T1 = ui->T1->text().toDouble();
    baseParams.T2 = ui->T2->text().toDouble();
    baseParams.T3 = ui->T3->text().toDouble();
    baseParams.Tsize = ui->Tsize->text().toInt();
    baseParams.Bmin = ui->B_min->text().toDouble();
    baseParams.Emin = ui->e_min->text().toDouble();
    baseParams.minErr = ui->minerr->text().toDouble();
    baseParams.gauss = ui->Gauss->isChecked();
    baseParams.sigma = ui->sigma->text().toDouble();

    if(baseParams.saveRootPath.isEmpty())
    {
        appendSlLog(QStringLiteral("save_root_path is empty"));
        return;
    }

    if(baseParams.calibFilename.isEmpty())
    {
        appendSlLog(QStringLiteral("calib_filename is empty"));
        return;
    }

    QStringList subpaths = selectedSlSubpaths;
    if(subpaths.isEmpty())
    {
        const QString single = normalizedPathForSl(ui->patterns_subpath->text());
        if(!single.isEmpty())
        {
            QDir root(baseParams.saveRootPath);
            const QString abs = normalizedPathForSl(root.filePath(single));
            if(QDir(abs).exists())
                subpaths << abs;
        }
    }

    if(subpaths.isEmpty())
    {
        appendSlLog(QStringLiteral("no selected subfolders"));
        return;
    }

    slReconstructBusy = true;
    if(ui && ui->Calculate_SL)
        ui->Calculate_SL->setEnabled(false);

    appendSlLog(QStringLiteral("SL batch reconstruction started, folders=%1").arg(subpaths.size()));

    QString baseName = baseParams.saveName.trimmed();
    if(baseName.isEmpty())
        baseName = QStringLiteral("sl_cloud.pcd");
    if(!baseName.endsWith(QStringLiteral(".pcd"), Qt::CaseInsensitive))
        baseName += QStringLiteral(".pcd");
    QFileInfo baseInfo(baseName);
    const QString stem = baseInfo.completeBaseName().isEmpty() ? QStringLiteral("sl_cloud") : baseInfo.completeBaseName();

    struct BatchItem { QString subpath; SlReconstructParams params; int index = 0; };
    struct BatchOutput { QString subpath; SlReconstructParams params; SlReconstructResult result; int index = 0; };

    QVector<BatchItem> items;
    items.reserve(subpaths.size());

    QDir rootDir(baseParams.saveRootPath);
    int seq = 1;
    for(const QString& absSub : subpaths)
    {
        BatchItem item;
        item.subpath = absSub;
        item.params = baseParams;
        item.params.patternsSubPath = normalizedPathForSl(rootDir.relativeFilePath(absSub));
        item.params.saveName = QStringLiteral("%1_%2.pcd").arg(stem).arg(seq++);
        item.index = seq - 1;
        items.push_back(item);
    }

    QtConcurrent::run([this, items]() {
        int okCount = 0;
        int failCount = 0;

        struct RunningTask
        {
            BatchItem item;
            QFuture<SlReconstructResult> future;
        };

        QList<RunningTask> running;
        int nextIndex = 0;

        auto launchOne = [&](const BatchItem& item) {
            RunningTask t;
            t.item = item;
            t.future = QtConcurrent::run([item]() {
                return runSlReconstruction(item.params);
            });
            running.push_back(std::move(t));
        };

        const int maxConcurrentTasks = 1; // 当前 SL CUDA 链路非线程安全，必须严格串行。

        // 批量执行 SL 重建并逐条回写结果。
        while(nextIndex < items.size() && running.size() < maxConcurrentTasks)
            launchOne(items[nextIndex++]);

        while(!running.isEmpty())
        {
            running.front().future.waitForFinished();
            RunningTask done = running.front();
            running.pop_front();

            BatchOutput out;
            out.subpath = done.item.subpath;
            out.params = done.item.params;
            out.index = done.item.index;
            out.result = done.future.result();

            if(out.result.ok)
                ++okCount;
            else
                ++failCount;

            QMetaObject::invokeMethod(this, [this, out]() {
                if(out.result.ok)
                {
                    if(ui && ui->DicGLWidget)
                    {
                        ui->DicGLWidget->add_pointcloud(out.result.cloud);
                        const int cloudIndex = ui->DicGLWidget->pointCloudCount() - 1;
                        const QString displayName = QFileInfo(out.result.savedPcdPath).completeBaseName();
                        addPointCloudFileListItem(out.result.savedPcdPath,
                                                  displayName,
                                                  static_cast<int>(PointCloudEntryType::PointCloud),
                                                  -1,
                                                  cloudIndex);
                    }
                    appendSlLog(QStringLiteral("[%1] ok: %2").arg(out.params.patternsSubPath, out.result.savedPcdPath));
                }
                else
                {
                    appendSlLog(QStringLiteral("[%1] failed: %2").arg(out.params.patternsSubPath, out.result.message));
                }

                // 批量执行 SL 重建并逐条回写结果。
            }, Qt::BlockingQueuedConnection);

            while(nextIndex < items.size() && running.size() < maxConcurrentTasks)
                launchOne(items[nextIndex++]);
        }

        QMetaObject::invokeMethod(this, [this, okCount, failCount]() {
            appendSlLog(QStringLiteral("SL batch done. success=%1, failed=%2").arg(okCount).arg(failCount));
            slReconstructBusy = false;
            if(ui && ui->Calculate_SL)
                ui->Calculate_SL->setEnabled(true);
        }, Qt::QueuedConnection);
    });
}

void MainWindow::on_SelectPatternroot_clicked()
{
    const QString currentPath = ui->base_root_dlp->text().trimmed();
    const QString selectedPath = QFileDialog::getExistingDirectory(this,
                                                                   QStringLiteral("Select DLP Pattern Root"),
                                                                   currentPath.isEmpty() ? QDir::currentPath() : currentPath,
                                                                   QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if(!selectedPath.isEmpty())
        ui->base_root_dlp->setText(QDir::toNativeSeparators(selectedPath));
}


void MainWindow::on_groupnumber_confirm_clicked()
{
    QMessageBox::information(this,
                             QStringLiteral("Info"),
                             QStringLiteral("DLP group count confirmed: ") + QString::number(ui->groupnumber->value()));
}


void MainWindow::on_picturenumber_confirm_clicked()
{
    QMessageBox::information(this,
                             QStringLiteral("Info"),
                             QStringLiteral("Picture count per group confirmed: ") + QString::number(ui->picturenumber->value()));
}


void MainWindow::on_horizontalSlider_sliderReleased()
{
    if(!dlp1)
    {
        QMessageBox::warning(this,
                             QStringLiteral("Warning"),
                             QStringLiteral("Please connect DLP first"));
        return;
    }

    const int brightness = ui->horizontalSlider->value();

    if(!dlp1->SetBrightness(brightness))
    {
        QMessageBox::warning(this, QStringLiteral("Warning"),
                             tr("Set DLP brightness") + QStringLiteral(": ") +
                             tr("DLP command write failed. Please check projector connection."));
        return;
    }

    // 用硬件实际生效的电流值回填滑块和标签。
    QSignalBlocker blockerSlider(ui->horizontalSlider);
    ui->horizontalSlider->setValue(static_cast<int>(dlp1->BlueLedCurrent));
    ui->label_current->setText(QStringLiteral("Current: ") + QString::number(static_cast<int>(dlp1->BlueLedCurrent)));
}


void MainWindow::on_action_v_clicked()
{
    ui->mdiArea->tileSubWindows();
}


void MainWindow::on_action_jilian_clicked()
{
    ui->mdiArea->cascadeSubWindows();
}


void MainWindow::on_action_hide_clicked()
{
    if (ui->tabWidget->isHidden())
    {
        ui->tabWidget->show();
    }
    else
    {
        ui->tabWidget->hide();
    }
}


void MainWindow::Connect_executeStep()
{
    connect(ui->DicGLWidget, &DICQGLViewer::saveSelectedPointsRequested, this, [this]() {
        if(!ui || !ui->DicGLWidget)
            return;

        PCLCloud::Ptr selectedCloud = ui->DicGLWidget->selectedPointsCloud();
        if(!selectedCloud || selectedCloud->empty())
            return;

        ui->DicGLWidget->add_pointcloud(*selectedCloud);
        const int newRow = ui->DicGLWidget->pointCloudCount() - 1;
        const QString displayName = QStringLiteral("selected_%1").arg(newRow + 1);
        addPointCloudFileListItem(displayName,
                                  displayName,
                                  static_cast<int>(PointCloudEntryType::PointCloud),
                                  -1);

        if(ui->plainTextEdit)
            ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Saved selected points to left list: ") + displayName);
    }, Qt::UniqueConnection);
}

void MainWindow::on_Select_Multicorn_clicked(bool checked)
{
    if(!ui || !ui->DicGLWidget)
        return;

    ui->DicGLWidget->setMultiCornerSelectionEnabled(checked);
    if(ui->plainTextEdit)
    {
        if(checked)
            ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Multi-corner selection enabled: left click to add vertices, right click to confirm polygon."));
        else
            ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Multi-corner selection disabled."));
    }
}

void MainWindow::on_cadImport_clicked()
{
    if(pointCloudLoadInProgress)
    {
        if(ui && ui->plainTextEdit)
            ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Point cloud is loading, please wait."));
        return;
    }

    if(!pointCloudWorker)
        initPointCloudThread();

    if(!pointCloudWorker)
    {
        QMessageBox::warning(this, tr("Point Cloud"), tr("Point cloud thread is not available."));
        return;
    }

    QStringList paths = QFileDialog::getOpenFileNames(
        this,
        tr("Open Points cloud"),
        QString(),
        //支持文件类型
        tr("Point Cloud Files (*.stl *.pcd *.txt *.ply *.pcpkg *.stp *.step);;STL Files (*.stl);;PCD Files (*.pcd);;TXT Files (*.txt);;PLY Files (*.ply);;STEP Files (*.stp *.step);;Point Cloud Package (*.pcpkg);;All Files (*.*)")
        );
    if(paths.isEmpty())
    {
        std::cout << "Model load cancelled. No file selected." << std::endl;
        return;
    }

    QStringList validPaths;
    for(const QString& path : paths)
    {
        QFileInfo fileInfo(path);
        const QString fileExtension = fileInfo.suffix().toLower();

        if(fileExtension == QStringLiteral("pcpkg"))
        {
            loadPointCloudFileOrPackage(path);
            continue;
        }

        // STEP 文件放子线程加载
        if(fileExtension == QStringLiteral("stp") || fileExtension == QStringLiteral("step"))
        {
            showStepProgress(tr("Importing STEP..."));

            auto* watcher = new QFutureWatcher<PCLMesh>(this);
            connect(watcher, &QFutureWatcher<PCLMesh>::finished, this, &MainWindow::hideStepProgress);
            connect(watcher, &QFutureWatcher<PCLMesh>::finished, this,
                    [this, path, watcher]() {
                PCLMesh mesh = watcher->result();
                if(!mesh.polygons.empty())
                {
                    const int meshIndex = ui->DicGLWidget->addMesh(mesh);
                    if(meshIndex >= 0)
                        ui->DicGLWidget->setMeshDisplayTag(meshIndex, QStringLiteral("imported_step"));
                    const QFileInfo fi(path);
                    addPointCloudFileListItem(path,
                                              fi.completeBaseName().isEmpty() ? fi.fileName() : fi.completeBaseName(),
                                              static_cast<int>(PointCloudEntryType::Reconstruction),
                                              meshIndex, -1);
                    if(ui && ui->plainTextEdit)
                        ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("STEP imported: ") + path);
                }
                else
                {
                    if(ui && ui->plainTextEdit)
                        ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("STEP import failed: ") + path
                                                           + QStringLiteral(" - ") + StepImporter::lastError());
                }
                watcher->deleteLater();
            });
            watcher->setFuture(QtConcurrent::run([this, path]() -> PCLMesh {
                PCLMesh m;
                auto progressFn = [this](int pct, const char* msg) {
                    QMetaObject::invokeMethod(this, [this, pct, msg = QString::fromLatin1(msg)]() {
                        updateStepProgress(pct, msg);
                    }, Qt::QueuedConnection);
                };
                StepImporter::importStep(path, m, progressFn);
                return m;
            }));
            continue;
        }

        if(fileExtension != QStringLiteral("stl") &&
           fileExtension != QStringLiteral("pcd") &&
           fileExtension != QStringLiteral("txt") &&
           fileExtension != QStringLiteral("ply"))
        {
            if(ui && ui->plainTextEdit)
                ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Unsupported point cloud file skipped: ") + path);
            continue;
        }

        validPaths.append(path);
    }

    if(validPaths.isEmpty())
    {
        if(ui && ui->plainTextEdit)
            ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("No supported point cloud files selected."));
        return;
    }

    pendingPointCloudLoadCount = validPaths.size();
    pointCloudLoadInProgress = true;
    if(ui && ui->cadImport)
        ui->cadImport->setEnabled(false);

    if(ui && ui->plainTextEdit)
        ui->plainTextEdit->appendPlainText(
            systemTime() + " " + tr("Point cloud import task submitted. Files: ") +
            QString::number(validPaths.size()));

    for(const QString& path : validPaths)
    {
        QMetaObject::invokeMethod(pointCloudWorker,
                                  "loadFile",
                                  Qt::QueuedConnection,
                                  Q_ARG(QString, path));
    }
}

void MainWindow::dragEnterEvent(QDragEnterEvent *event)
{
    if(!event || !event->mimeData() || !event->mimeData()->hasUrls())
    {
        if(event)
            event->ignore();
        return;
    }

    const QList<QUrl> urls = event->mimeData()->urls();
    for(const QUrl& url : urls)
    {
        if(!url.isLocalFile())
            continue;

        const QString ext = QFileInfo(url.toLocalFile()).suffix().toLower();
        if(ext == QStringLiteral("stl") ||
           ext == QStringLiteral("pcd") ||
           ext == QStringLiteral("txt") ||
           ext == QStringLiteral("ply") ||
           ext == QStringLiteral("stp") ||
           ext == QStringLiteral("step") ||
           ext == QStringLiteral("pcpkg"))
        {
            event->acceptProposedAction();
            return;
        }
    }

    event->ignore();
}

void MainWindow::dropEvent(QDropEvent *event)
{
    if(!event || !event->mimeData() || !event->mimeData()->hasUrls())
    {
        if(event)
            event->ignore();
        return;
    }

    if(pointCloudLoadInProgress)
    {
        if(ui && ui->plainTextEdit)
            ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Point cloud is loading, please wait."));
        event->ignore();
        return;
    }

    if(!pointCloudWorker)
        initPointCloudThread();

    if(!pointCloudWorker)
    {
        QMessageBox::warning(this, tr("Point Cloud"), tr("Point cloud thread is not available."));
        event->ignore();
        return;
    }

    QStringList validPaths;
    bool anyLoaded = false;
    const QList<QUrl> urls = event->mimeData()->urls();
    for(const QUrl& url : urls)
    {
        if(!url.isLocalFile())
            continue;

        const QString localPath = url.toLocalFile();
        const QString fileExtension = QFileInfo(localPath).suffix().toLower();
        if(fileExtension == QStringLiteral("pcpkg"))
        {
            loadPointCloudFileOrPackage(localPath);
            anyLoaded = true;
        }
        else if(fileExtension == QStringLiteral("stp") || fileExtension == QStringLiteral("step"))
        {
            anyLoaded = true;
            auto* watcher = new QFutureWatcher<PCLMesh>(this);
            connect(watcher, &QFutureWatcher<PCLMesh>::finished, this,
                    [this, localPath, watcher]() {
                PCLMesh mesh = watcher->result();
                if(!mesh.polygons.empty())
                {
                    const int meshIndex = ui->DicGLWidget->addMesh(mesh);
                    if(meshIndex >= 0)
                        ui->DicGLWidget->setMeshDisplayTag(meshIndex, QStringLiteral("imported_step"));
                    const QFileInfo fi(localPath);
                    addPointCloudFileListItem(localPath,
                                              fi.completeBaseName().isEmpty() ? fi.fileName() : fi.completeBaseName(),
                                              static_cast<int>(PointCloudEntryType::Reconstruction),
                                              meshIndex, -1);
                    if(ui && ui->plainTextEdit)
                        ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("STEP imported: ") + localPath);
                }
                else
                {
                    if(ui && ui->plainTextEdit)
                        ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("STEP import failed: ") + localPath
                                                           + QStringLiteral(" - ") + StepImporter::lastError());
                }
                watcher->deleteLater();
            });
            watcher->setFuture(QtConcurrent::run([localPath]() -> PCLMesh {
                PCLMesh m;
                StepImporter::importStep(localPath, m);
                return m;
            }));
        }
        else if(fileExtension == QStringLiteral("stl") ||
                fileExtension == QStringLiteral("pcd") ||
                fileExtension == QStringLiteral("txt") ||
                fileExtension == QStringLiteral("ply"))
        {
            validPaths.append(localPath);
        }
        else if(ui && ui->plainTextEdit)
        {
            ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Unsupported point cloud file skipped: ") + localPath);
        }
    }

    if(validPaths.isEmpty())
    {
        if(!anyLoaded && ui && ui->plainTextEdit)
            ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("No supported point cloud files selected."));
        if(!anyLoaded)
        {
            event->ignore();
            return;
        }
        event->acceptProposedAction();
        return;
    }

    pendingPointCloudLoadCount = validPaths.size();
    pointCloudLoadInProgress = true;
    if(ui && ui->cadImport)
        ui->cadImport->setEnabled(false);

    if(ui && ui->plainTextEdit)
        ui->plainTextEdit->appendPlainText(
            systemTime() + " " + tr("Point cloud import task submitted. Files: ") +
            QString::number(validPaths.size()));

    for(const QString& path : validPaths)
    {
        QMetaObject::invokeMethod(pointCloudWorker,
                                  "loadFile",
                                  Qt::QueuedConnection,
                                  Q_ARG(QString, path));
    }

    event->acceptProposedAction();
}

bool MainWindow::eventFilter(QObject* obj, QEvent* event)
{
    if(obj == pcaSlicePanel && pcaSlicePanel)
    {
        if(event->type() == QEvent::MouseButtonPress)
        {
            QMouseEvent* me = static_cast<QMouseEvent*>(event);
            if(me->button() == Qt::LeftButton)
            {
                // Only start dragging from non-interactive areas (labels / empty space)
                QWidget* child = pcaSlicePanel->childAt(me->pos());
                if(!child || qobject_cast<QLabel*>(child))
                {
                    pcaPanelDragging = true;
                    pcaPanelDragOffset = me->pos();
                    pcaSlicePanel->grabMouse();
                    return true;
                }
            }
        }
        else if(event->type() == QEvent::MouseMove)
        {
            if(pcaPanelDragging)
            {
                QMouseEvent* me = static_cast<QMouseEvent*>(event);
                // Use global position for reliable tracking
                QPoint panelPos = pcaSlicePanel->parentWidget()
                    ? pcaSlicePanel->parentWidget()->mapFromGlobal(me->globalPos())
                    : pcaSlicePanel->mapToParent(me->pos());
                QPoint newPos = panelPos - pcaPanelDragOffset;
                newPos.setX(std::max(0, std::min(width() - pcaSlicePanel->width(), newPos.x())));
                newPos.setY(std::max(0, std::min(height() - pcaSlicePanel->height(), newPos.y())));
                pcaSlicePanel->move(newPos);
                return true;
            }
        }
        else if(event->type() == QEvent::MouseButtonRelease)
        {
            if(pcaPanelDragging)
            {
                pcaPanelDragging = false;
                pcaSlicePanel->releaseMouse();
                return true;
            }
        }
    }
    return QMainWindow::eventFilter(obj, event);
}

void MainWindow::saveSelectedPointCloudEntries()
{
    if(!ui || !ui->DicGLWidget || !ui->listWidgetPointCloudFiles)
        return;

    QItemSelectionModel* selModel = ui->listWidgetPointCloudFiles->selectionModel();
    if(!selModel)
        return;

    QModelIndexList selectedRows = selModel->selectedRows();
    if(selectedRows.isEmpty())
    {
        const QModelIndex currentIndex = ui->listWidgetPointCloudFiles->currentIndex();
        if(currentIndex.isValid())
            selectedRows << currentIndex;
    }
    if(selectedRows.isEmpty())
        return;

    std::sort(selectedRows.begin(), selectedRows.end(), [](const QModelIndex& a, const QModelIndex& b){
        return a.row() < b.row();
    });

    if(selectedRows.size() == 1)
    {
        saveSinglePointCloudToFile(selectedRows.first().row());
        return;
    }

    savePointCloudPackage(selectedRows);
}

bool MainWindow::saveSinglePointCloudToFile(int row)
{
    if(!ui || !ui->DicGLWidget || !pointCloudListModel || row < 0 || row >= pointCloudListModel->rowCount())
        return false;

    const QStandardItem* item = pointCloudListModel->item(row);
    const int entryType = item ? item->data(RoleEntryType).toInt() : static_cast<int>(PointCloudEntryType::PointCloud);
    const int cloudIndex = item ? item->data(RolePointCloudIndex).toInt() : -1;
    const int meshIndex = item ? item->data(RoleMeshIndex).toInt() : -1;
    const int lineIndex = item ? item->data(RoleLineIndex).toInt() : -1;

    const QString rawName = item ? item->text() : QString("entry_%1").arg(row + 1);
    QString suggestedBaseName = rawName;
    suggestedBaseName.replace(QRegularExpression(QStringLiteral("[\\\\/:*?\"<>|]")), QStringLiteral("_"));
    if(suggestedBaseName.trimmed().isEmpty())
        suggestedBaseName = QString("entry_%1").arg(row + 1);

    // 保存拟合轴线（FittedAxisLine）：将线段点云写入 PCD/TXT
    if(entryType == static_cast<int>(PointCloudEntryType::FittedAxisLine))
    {
        if(lineIndex < 0)
            return false;

        QString selectedFilter = tr("PCD Files (*.pcd)");
        QString filePath = QFileDialog::getSaveFileName(
            this,
            tr("Save Axis Line"),
            suggestedBaseName + QStringLiteral(".pcd"),
            tr("PCD Files (*.pcd);;TXT Files (*.txt);;PLY Files (*.ply)"),
            &selectedFilter);
        if(filePath.isEmpty())
            return false;

        const bool usePcd = selectedFilter.contains(QStringLiteral("*.pcd"), Qt::CaseInsensitive);
        const bool useTxt = selectedFilter.contains(QStringLiteral("*.txt"), Qt::CaseInsensitive);
        const bool usePly = selectedFilter.contains(QStringLiteral("*.ply"), Qt::CaseInsensitive);
        if(usePcd && !filePath.endsWith(QStringLiteral(".pcd"), Qt::CaseInsensitive))
            filePath += QStringLiteral(".pcd");
        else if(useTxt && !filePath.endsWith(QStringLiteral(".txt"), Qt::CaseInsensitive))
            filePath += QStringLiteral(".txt");
        else if(usePly && !filePath.endsWith(QStringLiteral(".ply"), Qt::CaseInsensitive))
            filePath += QStringLiteral(".ply");

        const QString ext = QFileInfo(filePath).suffix().toLower();
        if(ext != QStringLiteral("pcd") && ext != QStringLiteral("txt") && ext != QStringLiteral("ply"))
        {
            QMessageBox::warning(this, tr("Save Axis Line"), tr("Please select txt, pcd or ply format."));
            return false;
        }

        PCLCloud::Ptr lineCloud = ui->DicGLWidget->lineByIndex(lineIndex);
        if(!lineCloud || lineCloud->empty())
            return false;

        if(ext == QStringLiteral("pcd"))
        {
            if(pcl::io::savePCDFileBinary(filePath.toStdString(), *lineCloud) < 0)
                return false;
        }
        else if(ext == QStringLiteral("ply"))
        {
            if(pcl::io::savePLYFileBinary(filePath.toStdString(), *lineCloud) < 0)
                return false;
        }
        else
        {
            QFile out(filePath);
            if(!out.open(QIODevice::WriteOnly | QIODevice::Text))
                return false;
            QTextStream ts(&out);
            ts.setRealNumberNotation(QTextStream::FixedNotation);
            ts.setRealNumberPrecision(9);
            for(const auto& p : lineCloud->points)
                ts << p.x << ' ' << p.y << ' ' << p.z << '\n';
        }

        if(ui && ui->plainTextEdit)
            ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Saved axis line: ") + filePath);
        return true;
    }

    if(entryType == static_cast<int>(PointCloudEntryType::PointCloud))
    {
        if(cloudIndex < 0)
            return false;

        QString selectedFilter = tr("PCD Files (*.pcd)");
        QString filePath = QFileDialog::getSaveFileName(
            this,
            tr("Save Point Cloud"),
            suggestedBaseName + QStringLiteral(".pcd"),
            tr("PCD Files (*.pcd);;TXT Files (*.txt);;PLY Files (*.ply);;STEP Files (*.stp *.step)"),
            &selectedFilter);
        if(filePath.isEmpty())
            return false;

        const bool usePcd = selectedFilter.contains(QStringLiteral("*.pcd"), Qt::CaseInsensitive);
        const bool useTxt = selectedFilter.contains(QStringLiteral("*.txt"), Qt::CaseInsensitive);
        const bool usePly = selectedFilter.contains(QStringLiteral("*.ply"), Qt::CaseInsensitive);
        const bool useStp = selectedFilter.contains(QStringLiteral("*.stp"), Qt::CaseInsensitive)
                         || selectedFilter.contains(QStringLiteral("*.step"), Qt::CaseInsensitive);
        if(usePcd && !filePath.endsWith(QStringLiteral(".pcd"), Qt::CaseInsensitive))
            filePath += QStringLiteral(".pcd");
        else if(useTxt && !filePath.endsWith(QStringLiteral(".txt"), Qt::CaseInsensitive))
            filePath += QStringLiteral(".txt");
        else if(usePly && !filePath.endsWith(QStringLiteral(".ply"), Qt::CaseInsensitive))
            filePath += QStringLiteral(".ply");
        else if(useStp && !filePath.endsWith(QStringLiteral(".stp"), Qt::CaseInsensitive)
                     && !filePath.endsWith(QStringLiteral(".step"), Qt::CaseInsensitive))
            filePath += QStringLiteral(".stp");

        const QString ext = QFileInfo(filePath).suffix().toLower();
        if(ext != QStringLiteral("pcd") && ext != QStringLiteral("txt")
           && ext != QStringLiteral("ply") && ext != QStringLiteral("stp")
           && ext != QStringLiteral("step"))
        {
            QMessageBox::warning(this, tr("Save Point Cloud"),
                                 tr("Please select txt, pcd, ply or step format."));
            return false;
        }

        PCLCloud::Ptr cloud = ui->DicGLWidget->pointCloudByIndex(cloudIndex);
        if(!cloud || cloud->empty())
            return false;

        if(ext == QStringLiteral("pcd"))
        {
            if(pcl::io::savePCDFileBinary(filePath.toStdString(), *cloud) < 0)
                return false;
        }
        else if(ext == QStringLiteral("ply"))
        {
            if(pcl::io::savePLYFileBinary(filePath.toStdString(), *cloud) < 0)
                return false;
        }
        else if(ext == QStringLiteral("stp") || ext == QStringLiteral("step"))
        {
            showStepProgress(tr("Exporting STEP..."));

            QEventLoop el;
            QFutureWatcher<bool> fw;
            connect(&fw, &QFutureWatcher<bool>::finished, this, &MainWindow::hideStepProgress);
            connect(&fw, &QFutureWatcher<bool>::finished, &el, &QEventLoop::quit);
            fw.setFuture(QtConcurrent::run([this, cloud, filePath]() {
                auto progressFn = [this](int pct, const char* msg) {
                    QMetaObject::invokeMethod(this, [this, pct, msg = QString::fromLatin1(msg)]() {
                        updateStepProgress(pct, msg);
                    }, Qt::QueuedConnection);
                };
                return StepExporter::exportPointCloudToStep(cloud, filePath, progressFn);
            }));
            el.exec();
            if(!fw.result())
            {
                QMessageBox::warning(this, tr("Save Point Cloud"),
                                     tr("STEP export failed."));
                return false;
            }
            if(ui && ui->plainTextEdit)
                ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Saved STEP: ") + filePath);
            return true;
        }
        else
        {
            QFile out(filePath);
            if(!out.open(QIODevice::WriteOnly | QIODevice::Text))
                return false;

            QTextStream ts(&out);
            ts.setRealNumberNotation(QTextStream::FixedNotation);
            ts.setRealNumberPrecision(9);
            for(const auto& p : cloud->points)
                ts << p.x << ' ' << p.y << ' ' << p.z << '\n';
        }

        if(ui && ui->plainTextEdit)
            ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Saved point cloud: ") + filePath);
        return true;
    }

    if(meshIndex < 0)
        return false;

    QString selectedFilter = tr("STL Files (*.stl)");
    QString filePath = QFileDialog::getSaveFileName(
        this,
        tr("Save Mesh"),
        suggestedBaseName + QStringLiteral(".stl"),
        tr("STL Files (*.stl);;VTK Files (*.vtk);;PLY Files (*.ply);;STEP Files (*.stp *.step)"),
        &selectedFilter);
    if(filePath.isEmpty())
        return false;

    const bool useStl = selectedFilter.contains(QStringLiteral("*.stl"), Qt::CaseInsensitive);
    const bool useVtk = selectedFilter.contains(QStringLiteral("*.vtk"), Qt::CaseInsensitive);
    const bool usePly = selectedFilter.contains(QStringLiteral("*.ply"), Qt::CaseInsensitive);
    const bool useStp = selectedFilter.contains(QStringLiteral("*.stp"), Qt::CaseInsensitive)
                     || selectedFilter.contains(QStringLiteral("*.step"), Qt::CaseInsensitive);
    if(useStl && !filePath.endsWith(QStringLiteral(".stl"), Qt::CaseInsensitive))
        filePath += QStringLiteral(".stl");
    else if(useVtk && !filePath.endsWith(QStringLiteral(".vtk"), Qt::CaseInsensitive))
        filePath += QStringLiteral(".vtk");
    else if(usePly && !filePath.endsWith(QStringLiteral(".ply"), Qt::CaseInsensitive))
        filePath += QStringLiteral(".ply");
    else if(useStp && !filePath.endsWith(QStringLiteral(".stp"), Qt::CaseInsensitive)
                 && !filePath.endsWith(QStringLiteral(".step"), Qt::CaseInsensitive))
        filePath += QStringLiteral(".stp");

    const QString ext = QFileInfo(filePath).suffix().toLower();
    if(ext != QStringLiteral("stl") && ext != QStringLiteral("vtk")
       && ext != QStringLiteral("ply") && ext != QStringLiteral("stp")
       && ext != QStringLiteral("step"))
    {
        QMessageBox::warning(this, tr("Save Mesh"),
                             tr("Please select stl, vtk, ply or step format."));
        return false;
    }

    const PCLMesh mesh = ui->DicGLWidget->meshByIndex(meshIndex);
    if(mesh.polygons.empty())
        return false;

    // STEP 导出
    if(ext == QStringLiteral("stp") || ext == QStringLiteral("step"))
    {
        showStepProgress(tr("Exporting STEP..."));

        QEventLoop el;
        QFutureWatcher<bool> fw;
        connect(&fw, &QFutureWatcher<bool>::finished, this, &MainWindow::hideStepProgress);
        connect(&fw, &QFutureWatcher<bool>::finished, &el, &QEventLoop::quit);
        fw.setFuture(QtConcurrent::run([this, mesh, filePath]() {
            auto progressFn = [this](int pct, const char* msg) {
                QMetaObject::invokeMethod(this, [this, pct, msg = QString::fromLatin1(msg)]() {
                    updateStepProgress(pct, msg);
                }, Qt::QueuedConnection);
            };
            return StepExporter::exportMeshToStep(mesh, filePath, progressFn);
        }));
        el.exec();
        if(!fw.result())
        {
            QMessageBox::warning(this, tr("Save Mesh"),
                                 tr("STEP export failed."));
            return false;
        }
        if(ui && ui->plainTextEdit)
            ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Saved STEP: ") + filePath);
        return true;
    }

    int rc = -1;
    if(ext == QStringLiteral("stl"))
        rc = pcl::io::savePolygonFileSTL(filePath.toStdString(), mesh);
    else if(ext == QStringLiteral("ply"))
        rc = pcl::io::savePLYFile(filePath.toStdString(), mesh);
    else
        rc = pcl::io::savePolygonFileVTK(filePath.toStdString(), mesh);

    if(rc < 0)
        return false;

    if(ui && ui->plainTextEdit)
        ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Saved mesh: ") + filePath);
    return true;
}

bool MainWindow::savePointCloudPackage(const QModelIndexList& selectedRows)
{
    if(!ui || !ui->DicGLWidget || !pointCloudListModel || selectedRows.isEmpty())
        return false;

    const QString packagePath = QFileDialog::getSaveFileName(
        this,
        tr("Save Point Cloud Package"),
        QString(),
        tr("Point Cloud Package (*.pcpkg)"));
    if(packagePath.isEmpty())
        return false;

    QFile out(packagePath);
    if(!out.open(QIODevice::WriteOnly))
        return false;

    QDataStream ds(&out);
    ds.setByteOrder(QDataStream::LittleEndian);
    ds.setVersion(QDataStream::Qt_5_15);

    const quint32 magic = 0x474B4350; // "PCKG"
    const quint16 version = 3;
    ds << magic << version;

    quint32 validCount = 0;
    for(const QModelIndex& idx : selectedRows)
    {
        const int row = idx.row();
        if(row < 0 || row >= pointCloudListModel->rowCount())
            continue;
        if(const QStandardItem* item = pointCloudListModel->item(row))
        {
            const int type = item->data(RoleEntryType).toInt();
            if(type == static_cast<int>(PointCloudEntryType::PointCloud) && item->data(RolePointCloudIndex).toInt() >= 0)
                ++validCount;
            else if(type == static_cast<int>(PointCloudEntryType::FittedAxisLine) && item->data(RoleLineIndex).toInt() >= 0)
                ++validCount;
            else if(type != static_cast<int>(PointCloudEntryType::PointCloud)
                    && type != static_cast<int>(PointCloudEntryType::FittedAxisLine)
                    && item->data(RoleMeshIndex).toInt() >= 0)
                ++validCount;
        }
    }
    ds << validCount;

    for(const QModelIndex& idx : selectedRows)
    {
        const int row = idx.row();
        if(row < 0 || row >= pointCloudListModel->rowCount())
            continue;

        const QStandardItem* item = pointCloudListModel->item(row);
        if(!item)
            continue;

        const int entryType = item->data(RoleEntryType).toInt();
        const int cloudIndex = item->data(RolePointCloudIndex).toInt();
        const int meshIndex = item->data(RoleMeshIndex).toInt();
        const int lineIndex = item->data(RoleLineIndex).toInt();
        const QString displayName = item->text();

        if(entryType == static_cast<int>(PointCloudEntryType::PointCloud))
        {
            if(cloudIndex < 0)
                continue;
            PCLCloud::Ptr cloud = ui->DicGLWidget->pointCloudByIndex(cloudIndex);
            if(!cloud)
                cloud.reset(new PCLCloud);

            ds << static_cast<qint32>(entryType) << displayName;
            ds << static_cast<quint32>(cloud->points.size());
            for(const auto& p : cloud->points)
                ds << p.x << p.y << p.z;
        }
        else if(entryType == static_cast<int>(PointCloudEntryType::FittedAxisLine))
        {
            // 回转轴线：仅保存线段端点（点云），不含拓扑
            if(lineIndex < 0)
                continue;
            PCLCloud::Ptr line = ui->DicGLWidget->lineByIndex(lineIndex);
            if(!line)
                line.reset(new PCLCloud);

            ds << static_cast<qint32>(entryType) << displayName;
            ds << static_cast<quint32>(line->points.size());
            for(const auto& p : line->points)
                ds << p.x << p.y << p.z;
        }
        else
        {
            if(meshIndex < 0)
                continue;
            const PCLMesh mesh = ui->DicGLWidget->meshByIndex(meshIndex);
            ds << static_cast<qint32>(entryType) << displayName;

            pcl::PointCloud<pcl::PointXYZ> meshPoints;
            pcl::fromPCLPointCloud2(mesh.cloud, meshPoints);
            ds << static_cast<quint32>(meshPoints.size());
            for(const auto& p : meshPoints.points)
                ds << p.x << p.y << p.z;

            ds << static_cast<quint32>(mesh.polygons.size());
            for(const auto& poly : mesh.polygons)
            {
                ds << static_cast<quint32>(poly.vertices.size());
                for(std::uint32_t v : poly.vertices)
                    ds << static_cast<quint32>(v);
            }
        }
    }

    if(ui && ui->plainTextEdit)
        ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Saved point cloud package: ") + packagePath);
    return true;
}

bool MainWindow::loadPointCloudPackage(const QString& packagePath)
{
    if(!ui || !ui->DicGLWidget)
        return false;

    QFile in(packagePath);
    if(!in.open(QIODevice::ReadOnly))
        return false;

    QDataStream ds(&in);
    ds.setByteOrder(QDataStream::LittleEndian);
    ds.setVersion(QDataStream::Qt_5_15);

    quint32 magic = 0;
    quint16 version = 0;
    quint32 itemCount = 0;
    ds >> magic >> version >> itemCount;
    if(magic != 0x474B4350 || (version != 1 && version != 2 && version != 3))
        return false;

    if(version == 1)
    {
        for(quint32 i = 0; i < itemCount; ++i)
        {
            QString displayName;
            quint32 pointCount = 0;
            ds >> displayName >> pointCount;

            PCLCloud cloud;
            cloud.points.reserve(pointCount);
            for(quint32 j = 0; j < pointCount; ++j)
            {
                float x = 0.0f, y = 0.0f, z = 0.0f;
                ds >> x >> y >> z;
                cloud.points.emplace_back(x, y, z);
            }
            cloud.width = static_cast<std::uint32_t>(cloud.points.size());
            cloud.height = 1;
            cloud.is_dense = false;

            ui->DicGLWidget->add_pointcloud(cloud);
            addPointCloudFileListItem(packagePath, displayName,
                                      static_cast<int>(PointCloudEntryType::PointCloud), -1,
                                      ui->DicGLWidget->pointCloudCount() - 1);
        }
    }
    else
    {
        for(quint32 i = 0; i < itemCount; ++i)
        {
            qint32 entryType = static_cast<qint32>(PointCloudEntryType::PointCloud);
            QString displayName;
            ds >> entryType >> displayName;

            if(entryType == static_cast<qint32>(PointCloudEntryType::PointCloud))
            {
                quint32 pointCount = 0;
                ds >> pointCount;

                PCLCloud cloud;
                cloud.points.reserve(pointCount);
                for(quint32 j = 0; j < pointCount; ++j)
                {
                    float x = 0.0f, y = 0.0f, z = 0.0f;
                    ds >> x >> y >> z;
                    cloud.points.emplace_back(x, y, z);
                }
                cloud.width = static_cast<std::uint32_t>(cloud.points.size());
                cloud.height = 1;
                cloud.is_dense = false;

                ui->DicGLWidget->add_pointcloud(cloud);
                addPointCloudFileListItem(packagePath, displayName,
                                          static_cast<int>(PointCloudEntryType::PointCloud), -1,
                                          ui->DicGLWidget->pointCloudCount() - 1);
            }
            else if(entryType == static_cast<qint32>(PointCloudEntryType::FittedAxisLine))
            {
                // 回转轴线：读取线段端点并重建线段
                quint32 pointCount = 0;
                ds >> pointCount;

                PCLCloud::Ptr line(new PCLCloud);
                line->points.reserve(pointCount);
                for(quint32 j = 0; j < pointCount; ++j)
                {
                    float x = 0.0f, y = 0.0f, z = 0.0f;
                    ds >> x >> y >> z;
                    line->points.emplace_back(x, y, z);
                }
                line->width = static_cast<std::uint32_t>(line->points.size());
                line->height = 1;
                line->is_dense = true;

                const int newLineIndex = ui->DicGLWidget->addLine(line);
                addPointCloudFileListItem(packagePath, displayName,
                                          static_cast<int>(PointCloudEntryType::FittedAxisLine),
                                          -1, -1, newLineIndex);
            }
            else
            {
                quint32 pointCount = 0;
                ds >> pointCount;

                pcl::PointCloud<pcl::PointXYZ> meshPoints;
                meshPoints.reserve(pointCount);
                for(quint32 j = 0; j < pointCount; ++j)
                {
                    float x = 0.0f, y = 0.0f, z = 0.0f;
                    ds >> x >> y >> z;
                    meshPoints.emplace_back(x, y, z);
                }

                quint32 polygonCount = 0;
                ds >> polygonCount;
                std::vector<pcl::Vertices> polygons;
                polygons.reserve(polygonCount);
                for(quint32 pi = 0; pi < polygonCount; ++pi)
                {
                    quint32 vcount = 0;
                    ds >> vcount;
                    pcl::Vertices poly;
                    poly.vertices.resize(vcount);
                    for(quint32 vi = 0; vi < vcount; ++vi)
                    {
                        quint32 idx = 0;
                        ds >> idx;
                        poly.vertices[vi] = idx;
                    }
                    polygons.push_back(poly);
                }

                PCLMesh mesh;
                pcl::toPCLPointCloud2(meshPoints, mesh.cloud);
                mesh.polygons = std::move(polygons);

                const int newMeshIndex = ui->DicGLWidget->addMesh(mesh);
                addPointCloudFileListItem(packagePath, displayName,
                                          entryType,
                                          newMeshIndex,
                                          -1);
            }
        }
    }

    if(ui && ui->plainTextEdit)
        ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Loaded point cloud package: ") + packagePath);
    return true;
}

bool MainWindow::loadPointCloudFileOrPackage(const QString& path)
{
    const QString ext = QFileInfo(path).suffix().toLower();
    if(ext == QStringLiteral("pcpkg"))
        return loadPointCloudPackage(path);

    if(!pointCloudWorker)
        initPointCloudThread();
    if(!pointCloudWorker)
        return false;

    pendingPointCloudLoadCount += 1;
    pointCloudLoadInProgress = true;
    if(ui && ui->cadImport)
        ui->cadImport->setEnabled(false);

    QMetaObject::invokeMethod(pointCloudWorker,
                              "loadFile",
                              Qt::QueuedConnection,
                              Q_ARG(QString, path));
    return true;
}






void MainWindow::on_analysis_clicked()
{
    if(!ui || !ui->DicGLWidget || !pointCloudListModel)
        return;

    // 获取选中的点云
    PCLCloud::Ptr selected_cloud = ui->DicGLWidget->selectedPointsCloud();
    if (!selected_cloud || selected_cloud->empty()) {
        const QModelIndex current = ui->listWidgetPointCloudFiles->currentIndex();
        if (current.isValid()) {
            const int cloudIndex = pointCloudListModel->item(current.row())->data(RolePointCloudIndex).toInt();
            if (cloudIndex >= 0)
                selected_cloud = ui->DicGLWidget->pointCloudByIndex(cloudIndex);
        }
    }
    if (!selected_cloud || selected_cloud->size() < 10) {
        QMessageBox::warning(this, tr("Analysis"), tr("No valid point cloud selected."));
        return;
    }

    // 参数对话框
    QDialog dlg(this);
    // dialog built programmatically below
    dlg.setWindowTitle(tr("Curvature Analysis"));
    dlg.setMinimumWidth(400);
    auto* lay = new QVBoxLayout(&dlg);
    auto* lblK = new QLabel(tr("Neighborhood size (k):"));
    lay->addWidget(lblK);
    auto* spinK = new QSpinBox();
    spinK->setRange(10, 200); spinK->setValue(30); spinK->setSingleStep(10);
    lay->addWidget(spinK);
    auto* hintK = new QLabel(tr("Larger k = wider search radius = better for gentle curves."));
    hintK->setWordWrap(true); lay->addWidget(hintK);
    auto* lblThr = new QLabel(tr("Curvature threshold (0=flat, ~0.02=curved, ~0.1+=sharp):"));
    lay->addWidget(lblThr);
    auto* spinThr = new QDoubleSpinBox();
    spinThr->setDecimals(6); spinThr->setRange(0.0, 0.333333);
    spinThr->setValue(0.00002); spinThr->setSingleStep(0.00001);
    lay->addWidget(spinThr);
    auto* hintThr = new QLabel(tr("Check log for curvature stats to calibrate this value."));
    hintThr->setWordWrap(true); lay->addWidget(hintThr);
    auto* btnBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(btnBox, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btnBox, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    lay->addWidget(btnBox);
    if (dlg.exec() != QDialog::Accepted) return;
    const int kNeighbors = spinK->value();
    const double curvThreshold = spinThr->value();

    const std::size_t N = selected_cloud->size();
    ui->plainTextEdit->appendPlainText(
        systemTime() + " " + tr("Computing curvature (k=%1, threshold=%2)...")
            .arg(kNeighbors).arg(curvThreshold));

    // 计算每个点的曲率
    std::vector<float> curvatures = Point_Algorithm::computePointCurvature(
        selected_cloud, kNeighbors);

    const float threshold = static_cast<float>(curvThreshold);

    // 提取高曲率点
    PCLCloud highCurvCloud;
    std::size_t highCount = 0;
    for (std::size_t i = 0; i < N; ++i) {
        if (curvatures[i] >= threshold) {
            highCurvCloud.push_back(selected_cloud->points[i]);
            ++highCount;
        }
    }
    highCurvCloud.width = static_cast<uint32_t>(highCurvCloud.size());
    highCurvCloud.height = 1;
    highCurvCloud.is_dense = false;

    // 统计曲率分布
    std::vector<float> sorted = curvatures;
    std::sort(sorted.begin(), sorted.end());
    float curvMin = sorted.front(), curvMax = sorted.back();
    float curvMean = 0.0f, curvMedian = sorted[sorted.size() / 2];
    for (float c : curvatures) curvMean += c;
    curvMean /= static_cast<float>(curvatures.size());

    // 添加到列表 — 红色
    const int idx = ui->DicGLWidget->pointCloudCount();
    ui->DicGLWidget->add_pointcloud(highCurvCloud);
    ui->DicGLWidget->setPointCloudColor(idx, QColor(255, 40, 40));
    addPointCloudFileListItem(QString(),
        QStringLiteral("曲率≥%1_%2pts_k%3").arg(curvThreshold).arg(highCount).arg(kNeighbors),
        static_cast<int>(PointCloudEntryType::PointCloud), -1, idx, -1);

    ui->plainTextEdit->appendPlainText(
        systemTime() + " " +
        tr("Curvature stats: min=%1, max=%2, mean=%3, median=%4")
            .arg(curvMin, 0, 'f', 6).arg(curvMax, 0, 'f', 6)
            .arg(curvMean, 0, 'f', 6).arg(curvMedian, 0, 'f', 6));
    ui->plainTextEdit->appendPlainText(
        systemTime() + " " +
        tr("Curvature >= %1: %2/%3 points (%4%)")
            .arg(threshold, 0, 'f', 6)
            .arg(highCount).arg(N)
            .arg(100.0 * highCount / N, 0, 'f', 1));
}

void MainWindow::on_reconstructSurface_clicked()
{
    // ---------- 1. 选择重建方法 ----------
    QDialog dlg(this);
    Ui::ReconstSurfaceDialog dlgUi;
    dlgUi.setupUi(&dlg);
    connect(dlgUi.buttonBox, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(dlgUi.buttonBox, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    // 切换重建方法时禁用无关参数面板，同时更新方法说明
    auto toggleMethodSettings = [&](int index) {
        dlgUi.groupBoxMesh->setEnabled(index == 0);
        dlgUi.groupBoxPoisson->setEnabled(index == 1);
        dlgUi.groupBoxNurbs->setEnabled(index == 2);
        dlgUi.groupBoxLoft->setEnabled(index == 3);
        switch (index) {
        case 0: dlgUi.labelMethodDesc->setText(
            QStringLiteral("\u8D2A\u5A6A\u4E09\u89D2\u5316\u5C06\u5C40\u90E8\u90BB\u57DF\u6295\u5F71\u5230\u5207\u5E73\u9762\u5E76\u8FDE\u63A5\uFF0C\u901F\u5EA6\u5FEB\uFF0C\u9002\u5408\u53F6\u7247\u7B49\u5355\u4FA7\u626B\u63CF\u7684\u5F00\u653E\u66F2\u9762\u3002")); break;
        case 1: dlgUi.labelMethodDesc->setText(
            QStringLiteral("Poisson \u901A\u8FC7\u516B\u53C9\u6811\u4E0A\u7684\u9690\u5F0F\u51FD\u6570\u91CD\u5EFA\u5149\u6ED1\u5C01\u95ED\u7684\u6C34\u5BC6\u66F2\u9762\uFF0C\u9002\u5408\u5168\u65B9\u4F4D\u626B\u63CF\u7684\u7269\u4F53\u3002")); break;
        case 2: dlgUi.labelMethodDesc->setText(
            QStringLiteral("NURBS \u5C06\u5149\u6ED1\u7684\u53C2\u6570\u66F2\u9762\u62DF\u5408\u5230\u70B9\u4E91\uFF0C\u751F\u6210\u6570\u5B66\u4E0A\u5E73\u6ED1\u3001\u53EF\u7F16\u8F91\u7684\u66F2\u9762\uFF0C\u4F46\u901F\u5EA6\u8F83\u6162\u3002")); break;
        case 3: dlgUi.labelMethodDesc->setText(
            QStringLiteral("\u8499\u76AE\uFF08\u653E\u6837\uFF09\u5C06\u591A\u4E2A\u6709\u5E8F\u622A\u9762\u6309\u5F27\u957F\u91CD\u91C7\u6837\u540E\u9010\u7247\u8FDE\u6210\u56DB\u8FB9\u5F62\u7F51\u683C\uFF0C\u9002\u5408\u7FFC\u578B\u7B49\u6CBF\u4E3B\u8F74\u5207\u7247\u7684\u53F6\u7247\u3002\u53EF\u76F4\u63A5\u591A\u9009\u5DF2\u6709\u622A\u9762\uFF0C\u6216\u5BF9\u5355\u70B9\u4E91\u73B0\u5207\u3002")); break;
        }
    };
    connect(dlgUi.comboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            toggleMethodSettings);
    toggleMethodSettings(dlgUi.comboBox->currentIndex());

    if(dlg.exec() != QDialog::Accepted)
        return;

    const int methodIndex = dlgUi.comboBox->currentIndex();
    const bool usePoisson = (methodIndex == 1);
    const bool useNurbs = (methodIndex == 2);
    // Greedy params
    const float voxelLeafSize = dlgUi.spinBoxVoxel->value();
    const float maxSurfaceAngleDeg = dlgUi.spinBoxMaxAngle->value();
    // NURBS params
    const int nurbsOrder = dlgUi.spinBoxNurbsOrder->value();
    const int nurbsRefinement = dlgUi.spinBoxNurbsRefinement->value();
    const int nurbsResolution = dlgUi.spinBoxNurbsResolution->value();
    // Poisson params
    const int poissonDepth = dlgUi.spinBoxPoissonDepth->value();
    const int poissonSolverDivide = dlgUi.spinBoxPoissonSolverDivide->value();
    const int poissonIsoDivide = dlgUi.spinBoxPoissonIsoDivide->value();
    // Loft params
    const bool useLoft = (methodIndex == 3);
    const bool loftMultiSection = dlgUi.checkBoxMultiSection->isChecked();
    const int loftSlices = dlgUi.spinBoxLoftSlices->value(); // 仅单点云切片模式使用
    const int loftResolution = dlgUi.spinBoxLoftResolution->value(); // 0=自动

    // ---------- 2. 收集待重建点云 ----------
    // 蒙皮法需要多个有序截面，走独立的收集流程；其余方法收集单个合并点云。
    std::vector<PCLCloud::Ptr> loftSections;   // 仅蒙皮法使用
    pcl::PointCloud<pcl::PointXYZ>::Ptr selected_cloud;

    if (useLoft)
    {
        // 收集左侧列表中选中的点云条目（按列表行序，通常即 slice_00/01/... 的主轴顺序）
        std::vector<int> cloudIndices;
        if (ui->listWidgetPointCloudFiles && ui->listWidgetPointCloudFiles->selectionModel())
        {
            const QModelIndexList rows = ui->listWidgetPointCloudFiles->selectionModel()->selectedRows();
            for (const QModelIndex& idx : rows)
            {
                const QStandardItem* item = pointCloudListModel ? pointCloudListModel->item(idx.row()) : nullptr;
                if(item && item->data(RoleEntryType).toInt() == static_cast<int>(PointCloudEntryType::PointCloud))
                {
                    const int cloudIndex = item->data(RolePointCloudIndex).toInt();
                    if(cloudIndex >= 0)
                        cloudIndices.push_back(cloudIndex);
                }
            }
        }

        if (loftMultiSection)
        {
            // 收集选中的点云（每个条目一片）
            std::vector<PCLCloud::Ptr> picked;
            for (int ci : cloudIndices)
            {
                PCLCloud::Ptr c = ui->DicGLWidget->pointCloudByIndex(ci);
                if (c && c->size() >= 2)
                    picked.push_back(c);
            }

            if (picked.size() >= 2)
            {
                // 情况 A：选了多个条目，每个条目直接作为一个截面
                loftSections = std::move(picked);
            }
            else if (picked.size() == 1)
            {
                // 情况 B：只选了一个点云，但它内部已含多个离散截面 →
                // 沿主轴按间隙自动切分（截面数由数据决定，不用手填）
                loftSections = Point_Algorithm::splitCloudIntoSections(picked.front());
                if (loftSections.size() >= 2)
                    ui->plainTextEdit->appendPlainText(systemTime() + " " +
                        tr("Loft: auto-split single cloud into %1 sections.")
                            .arg(loftSections.size()));
            }

            if (loftSections.size() < 2)
            {
                ui->plainTextEdit->appendPlainText(systemTime() + " " +
                    tr("Loft: could not obtain >=2 sections. Select multiple section clouds, "
                       "or select one cloud that contains separated sections."));
                return;
            }
        }
        else
        {
            // 对单个选中点云沿主轴 PCA 切片，生成有序截面
            PCLCloud::Ptr single = ui->DicGLWidget->mergedPointCloudByIndices(cloudIndices, true);
            if (!single || single->size() < static_cast<std::size_t>(loftSlices))
            {
                ui->plainTextEdit->appendPlainText(systemTime() + " " +
                    tr("Loft slicing needs a single point cloud with enough points."));
                return;
            }
            const Point_Algorithm::PCAResult pr = Point_Algorithm::pcaAnalysis(single);
            if (!pr.success)
            {
                ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("PCA failed for loft slicing."));
                return;
            }
            const Eigen::Vector3f pc1 = pr.eigenvectors.col(0).normalized();
            float tMin = std::numeric_limits<float>::max(), tMax = -tMin;
            for (const auto& p : single->points)
            {
                const float t = (Eigen::Vector3f(p.x, p.y, p.z) - pr.centroid).dot(pc1);
                tMin = std::min(tMin, t); tMax = std::max(tMax, t);
            }
            const float span = std::max(tMax - tMin, 1e-4f);
            const float halfTol = (span / static_cast<float>(loftSlices)) * 0.75f; // 相邻切片略有重叠，保证连续
            for (int i = 0; i < loftSlices; ++i)
            {
                const float tc = tMin + span * static_cast<float>(i) / static_cast<float>(loftSlices - 1);
                PCLCloud::Ptr sc(new PCLCloud);
                for (const auto& p : single->points)
                {
                    const Eigen::Vector3f pt(p.x, p.y, p.z);
                    const float proj = (pt - pr.centroid).dot(pc1);
                    if (std::abs(proj - tc) <= halfTol)
                    {
                        const Eigen::Vector3f projected = pt - pc1 * (proj - tc);
                        sc->points.emplace_back(projected.x(), projected.y(), projected.z());
                    }
                }
                if (sc->size() >= 2)
                {
                    sc->width = static_cast<uint32_t>(sc->points.size());
                    sc->height = 1;
                    loftSections.push_back(sc);
                }
            }
            if (loftSections.size() < 2)
            {
                ui->plainTextEdit->appendPlainText(systemTime() + " " +
                    tr("Loft slicing produced too few non-empty sections."));
                return;
            }
        }

        // 合并所有截面点，供后续裁剪/日志使用
        selected_cloud.reset(new PCLCloud);
        for (const auto& s : loftSections) *selected_cloud += *s;
    }
    else
    {
        // 优先取 3D 视图中手动框选的单点，为空则取左侧列表选中的完整点云（排除隐藏条目）
        selected_cloud = ui->DicGLWidget->selectedPointsCloud();

        if (!selected_cloud || selected_cloud->empty())
        {
            std::vector<int> cloudIndices;
            if (ui->listWidgetPointCloudFiles && ui->listWidgetPointCloudFiles->selectionModel())
            {
                const QModelIndexList rows = ui->listWidgetPointCloudFiles->selectionModel()->selectedRows();
                cloudIndices.reserve(rows.size());
                for (const QModelIndex& idx : rows)
                {
                    const QStandardItem* item = pointCloudListModel ? pointCloudListModel->item(idx.row()) : nullptr;
                    if(item && item->data(RoleEntryType).toInt() == static_cast<int>(PointCloudEntryType::PointCloud))
                    {
                        const int cloudIndex = item->data(RolePointCloudIndex).toInt();
                        if(cloudIndex >= 0)
                            cloudIndices.push_back(cloudIndex);
                    }
                }
            }

            // mergedPointCloudByIndices 第二个参数为 true 表示跳过隐藏点云
            selected_cloud = ui->DicGLWidget->mergedPointCloudByIndices(cloudIndices, true);
        }

        // 至少 3 个点才能构成一个三角面片
        if (!selected_cloud || selected_cloud->size() < 3)
        {
            ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("No valid selected points/clouds for reconstruction (hidden clouds excluded)."));
            return;
        }
    }

    // ---------- 2.5 曲率自适应降采样（仅 Poisson：水密重建能从非均匀采样中受益） ----------
    if (usePoisson) {
        const float featureLeaf = std::max(voxelLeafSize, 0.01f);
        const float flatLeaf = featureLeaf * 4.0f;
        PCLCloud::Ptr adapted = Point_Algorithm::adaptiveCurvatureDownsample(
            selected_cloud, featureLeaf, flatLeaf, 20.0f);
        if (adapted && adapted->size() >= 10) {
            selected_cloud = adapted;
            ui->plainTextEdit->appendPlainText(
                systemTime() + " " + tr("Curvature-adaptive downsample: %1 points (feature leaf=%2, flat leaf=%3)")
                    .arg(selected_cloud->size()).arg(featureLeaf).arg(flatLeaf));
        }
    }

    // ---------- 3. 调用重建算法 ----------
    if(ui && ui->plainTextEdit)
    {
        const QString methodLabel = useLoft ? QStringLiteral("Loft")
            : useNurbs ? QStringLiteral("NURBS")
            : usePoisson ? QStringLiteral("Poisson") : QStringLiteral("Mesh");
        ui->plainTextEdit->appendPlainText(
            systemTime() + " " + tr("Starting %1 reconstruction with %2 points...")
                .arg(methodLabel).arg(selected_cloud->size()));
    }

    // ===== 蒙皮法重建 =====
    if (useLoft)
    {
        // loftResolution: 每个面的插值点数，<=0 时默认 1000
        const int nInterpPerSurface = (loftResolution > 0) ? loftResolution : 1000;

        ui->plainTextEdit->appendPlainText(
            systemTime() + " " + tr("Loft: %1 个截面, 开始网格重建...")
                .arg(loftSections.size()));

        pcl::PolygonMesh reconstructed_mesh = Point_Algorithm::loftReconstruction(loftSections, nInterpPerSurface);

        if (reconstructed_mesh.polygons.empty())
        {
            ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Loft: 网格重建失败。"));
            return;
        }

        ui->DicGLWidget->add_mesh(reconstructed_mesh);
        addPointCloudFileListItem(QStringLiteral("loft_mesh"),
                                  QStringLiteral("loft_mesh(蒙皮网格)"),
                                  static_cast<int>(PointCloudEntryType::Reconstruction),
                                  ui->DicGLWidget->meshCount() - 1,  // meshIndex
                                  -1);                                // pointCloudIndex
        ui->DicGLWidget->update();
        ui->plainTextEdit->appendPlainText(
            systemTime() + " " + tr("Loft: 完成, %1 个三角面。")
                .arg(static_cast<qulonglong>(reconstructed_mesh.polygons.size())));
        return;
    }

    pcl::PolygonMesh reconstructed_mesh;
    if (false)
    {
    }
    else
    {
        const auto reconMethod = useNurbs ? Point_Algorithm::ReconMethod::Nurbs
            : usePoisson ? Point_Algorithm::ReconMethod::Poisson
            : Point_Algorithm::ReconMethod::Greedy;
        reconstructed_mesh = Point_Algorithm::reconstructMesh(
            selected_cloud, reconMethod, voxelLeafSize, maxSurfaceAngleDeg,
            nurbsOrder, nurbsRefinement, nurbsResolution,
            poissonDepth, poissonSolverDivide, poissonIsoDivide);
    }

    if (reconstructed_mesh.polygons.empty()) {
        ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Surface reconstruction failed or produced an empty mesh."));
        return;
    }

    // ---------- 3.5 修复网格（去退化面片、并重复顶点） ----------
    {
        const int nRemoved = Point_Algorithm::repairMesh(reconstructed_mesh);
        if (nRemoved > 0) {
            ui->plainTextEdit->appendPlainText(
                systemTime() + " " + tr("Mesh repair: removed %1 degenerate faces.").arg(nRemoved));
        }
        if (reconstructed_mesh.polygons.empty()) {
            ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("All faces removed during repair."));
            return;
        }
    }

    // 蒙皮网格本身就是有序规整的四边形网格，裁剪/精简会破坏其结构，故跳过。
    if (!useLoft)
    {
        // ---------- 4. 裁剪越界三角面片 ----------
        // Poisson 生成水密网格超出点云边界 → 负容差收缩包围盒来裁掉越界部分
        // Greedy 只在有点的区域三角化 → 正容差保护边界三角不被误裁
        {
            const float clipMargin = usePoisson ? -0.05f : 0.03f;
            Point_Algorithm::clipMeshToCloudBounds(reconstructed_mesh, selected_cloud, clipMargin);
        }
        if(reconstructed_mesh.polygons.empty()) {
            ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("All triangles clipped — reconstruction result empty."));
            return;
        }

        // ---------- 4.5 精简到 ≤3000 面 ----------
        {
            const int nBefore = static_cast<int>(reconstructed_mesh.polygons.size());
            Point_Algorithm::decimateMesh(reconstructed_mesh, 3000);
            if(reconstructed_mesh.polygons.empty()) {
                ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Decimation produced empty mesh."));
                return;
            }
            const int nAfter = static_cast<int>(reconstructed_mesh.polygons.size());
            if (nBefore != nAfter) {
                ui->plainTextEdit->appendPlainText(
                    systemTime() + " " + tr("Mesh decimated: %1 → %2 faces.").arg(nBefore).arg(nAfter));
            }
        }
    }

    // ---------- 5. 命名结果（循环直至合法） ----------
    // 限制 ASCII-only 是为了避免 PCL 内部文件 I/O 和 OpenGL 标签在非英文系统上出现编码问题
    QString resultName;
    while (true)
    {
        bool ok = false;
        const QString inputName = QInputDialog::getText(
            this,
            tr("Reconstruction Name"),
            tr("Please enter result name (ASCII only, unique):"),
            QLineEdit::Normal,
            QStringLiteral("recon_%1").arg(pointCloudListModel ? pointCloudListModel->rowCount() + 1 : 1),
            &ok).trimmed();

        if(!ok)
            return;

        if(inputName.isEmpty())
        {
            QMessageBox::warning(this, tr("Invalid Name"), tr("Name cannot be empty."));
            continue;
        }

        // PCL 的文件保存/加载路径不支持中文，OpenGL 的显示标签在非 UTF-8 locale 下也会乱码
        const QRegularExpression chineseRegex(QStringLiteral("[\\x{4e00}-\\x{9fff}]"));
        if(chineseRegex.match(inputName).hasMatch())
        {
            QMessageBox::warning(this, tr("Invalid Name"), tr("Name cannot contain Chinese characters."));
            continue;
        }

        bool duplicated = false;
        if(pointCloudListModel)
        {
            for(int r = 0; r < pointCloudListModel->rowCount(); ++r)
            {
                QStandardItem* item = pointCloudListModel->item(r);
                if(item && item->text().compare(inputName, Qt::CaseInsensitive) == 0)
                {
                    duplicated = true;
                    break;
                }
            }
        }

        if(duplicated)
        {
            QMessageBox::warning(this, tr("Invalid Name"), tr("Name already exists. Please use a unique name."));
            continue;
        }

        resultName = inputName;
        break;
    }

    // ---------- 6. 加入 3D 场景和文件列表 ----------
    const int reconstructionMeshIndex = ui->DicGLWidget->addMesh(reconstructed_mesh);
    if(reconstructionMeshIndex >= 0)
        ui->DicGLWidget->setMeshDisplayTag(reconstructionMeshIndex,
            useLoft ? QStringLiteral("reconstruction_loft")
                : useNurbs ? QStringLiteral("reconstruction_nurbs")
                : usePoisson ? QStringLiteral("reconstruction_poisson")
                : QStringLiteral("reconstruction"));

    pcl::PointCloud<pcl::PointXYZ> reconstructed_points;
    pcl::fromPCLPointCloud2(reconstructed_mesh.cloud, reconstructed_points);
    if(!reconstructed_points.empty())
    {
        addPointCloudFileListItem(resultName,
                                  resultName,
                                  static_cast<int>(PointCloudEntryType::Reconstruction),
                                  reconstructionMeshIndex,
                                  -1);
    }

    const QString methodLabelEnd = useNurbs ? QStringLiteral("NURBS")
        : usePoisson ? QStringLiteral("Poisson") : QStringLiteral("Mesh");
    ui->plainTextEdit->appendPlainText(
        systemTime() + " " + tr("%1 reconstruction completed. Triangles: %2, name: %3")
            .arg(methodLabelEnd)
            .arg(reconstructed_mesh.polygons.size())
            .arg(resultName));
}

// 对选中点云执行主成分分析，绘制主轴。
void MainWindow::on_PCA_Method_clicked()
{
    if(!ui || !ui->DicGLWidget || !ui->listWidgetPointCloudFiles || !ui->listWidgetPointCloudFiles->selectionModel() || !pointCloudListModel)
        return;

    const QModelIndexList selectedRows = ui->listWidgetPointCloudFiles->selectionModel()->selectedRows();
    if(selectedRows.isEmpty())
    {
        QMessageBox::information(this, tr("PCA"), tr("Please select one point cloud entry first."));
        return;
    }

    // 只处理第一个选中项
    const QStandardItem* item = pointCloudListModel->item(selectedRows.first().row());
    if(!item || item->data(RoleEntryType).toInt() != static_cast<int>(PointCloudEntryType::PointCloud))
        return;

    const int cloudIndex = item->data(RolePointCloudIndex).toInt();
    if(cloudIndex < 0)
        return;

    PCLCloud::Ptr input = ui->DicGLWidget->pointCloudByIndex(cloudIndex);
    if(!input || input->empty())
        return;

    // 显示 PCA 参数对话框
    QDialog dlg(this);
    Ui::PCADialog dlgUi;
    dlgUi.setupUi(&dlg);
    dlg.setWindowTitle(tr("PCA - Principal Component Analysis"));
    dlg.setMinimumWidth(420);
    dlg.setStyleSheet(QStringLiteral(
        "QDialog {"
        "  background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #f5f7fa, stop:1 #e4e9f0);"
        "}"
        "QLabel { color: #2c3e50; font-size: 13px; }"
        "QCheckBox { color: #2c3e50; font-size: 13px; spacing: 6px; }"
        "QCheckBox::indicator { width: 16px; height: 16px; }"
        "QDoubleSpinBox, QSpinBox {"
        "  background: white;"
        "  border: 1px solid #c0c8d0;"
        "  border-radius: 4px;"
        "  padding: 4px 8px;"
        "  font-size: 13px;"
        "  color: #2c3e50;"
        "}"
        "QDoubleSpinBox:focus, QSpinBox:focus {"
        "  border-color: #3a7ce0;"
        "}"
        "QDialogButtonBox QPushButton {"
        "  background: #3a7ce0;"
        "  color: white;"
        "  border: none;"
        "  border-radius: 4px;"
        "  padding: 6px 18px;"
        "  font-size: 13px;"
        "  min-width: 75px;"
        "}"
        "QDialogButtonBox QPushButton:hover { background: #5090f0; }"
        "QDialogButtonBox QPushButton:last-child {"
        "  background: #e0e0e0;"
        "  color: #555;"
        "}"
        "QDialogButtonBox QPushButton:last-child:hover { background: #d0d0d0; }"));
    dlgUi.sliceWidget->setVisible(dlgUi.checkSlice->isChecked());
    connect(dlgUi.buttonBox, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(dlgUi.buttonBox, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    if(dlg.exec() != QDialog::Accepted) return;

    const double axisScale = dlgUi.spinScale->value();
    const bool doSlice = dlgUi.checkSlice->isChecked();
    const int numSlices = dlgUi.spinNumSlices->value();
    const float tolerance = static_cast<float>(dlgUi.spinTolerance->value());

    // 计算 PCA
    const Point_Algorithm::PCAResult r = Point_Algorithm::pcaAnalysis(input);
    if(!r.success)
        return;

    const QString baseName = item->text().trimmed().isEmpty()
        ? QStringLiteral("cloud_%1").arg(cloudIndex + 1) : item->text().trimmed();
    const QString axisNames[3] = { QStringLiteral("PC1"), QStringLiteral("PC2"), QStringLiteral("PC3") };

    // 绘制三条主轴（仅在未启用切片模式时显示）
    if(!doSlice)
    {
        for(int k = 0; k < 3; ++k)
        {
            const float eigVal = r.eigenvalues[k];
            const float halfLen = std::sqrt(std::max(eigVal, 0.0f)) * static_cast<float>(axisScale);
            const Eigen::Vector3f dir = r.eigenvectors.col(k).normalized();

            PCLCloud::Ptr axisLine(new PCLCloud);
            const int samples = 80;
            for(int s = 0; s < samples; ++s)
            {
                const float t = -halfLen + 2.0f * halfLen * static_cast<float>(s) / static_cast<float>(samples - 1);
                const Eigen::Vector3f pt = r.centroid + dir * t;
                axisLine->points.emplace_back(pt.x(), pt.y(), pt.z());
            }
            axisLine->width = static_cast<uint32_t>(axisLine->points.size());
            axisLine->height = 1;
            axisLine->is_dense = true;

            const int lineIndex = ui->DicGLWidget->addLine(axisLine);
            const QString axisName = baseName + QStringLiteral("_") + axisNames[k];
            addPointCloudFileListItem(axisName, axisName,
                                      static_cast<int>(PointCloudEntryType::FittedAxisLine),
                                      -1, -1, lineIndex);
        }
    }

    // 日志
    if(ui && ui->plainTextEdit)
    {
        ui->plainTextEdit->appendPlainText(
            systemTime() + " " + tr("PCA: %1, centroid=(%2, %3, %4)")
                .arg(baseName).arg(r.centroid.x(), 0, 'f', 4)
                .arg(r.centroid.y(), 0, 'f', 4).arg(r.centroid.z(), 0, 'f', 4));
        for(int k = 0; k < 3; ++k)
            ui->plainTextEdit->appendPlainText(
                QString("  %1: eig=%2  dir=(%3, %4, %5)").arg(axisNames[k])
                    .arg(r.eigenvalues[k], 0, 'f', 4)
                    .arg(r.eigenvectors(0, k), 0, 'f', 4)
                    .arg(r.eigenvectors(1, k), 0, 'f', 4)
                    .arg(r.eigenvectors(2, k), 0, 'f', 4));
    }

    ui->DicGLWidget->update();

    // 进入切片模式
    if(doSlice)
        enterPCASlicingMode(r, input, cloudIndex, numSlices, tolerance);
}

// ========== PCA 切片模式 ==========

void MainWindow::enterPCASlicingMode(const Point_Algorithm::PCAResult& r,
                                      PCLCloud::Ptr cloud,
                                      int cloudIndex,
                                      int numSlices,
                                      float tolerance)
{
    pcaSlicingResult = r;
    pcaSliceNum = numSlices;
    pcaSliceTolerance = tolerance;
    pcaSlicingActive = true;
    pcaOriginalCloudIndex = cloudIndex;

    // 计算点云沿 PC1 的范围
    const Eigen::Vector3f pc1 = r.eigenvectors.col(0).normalized();
    pcaSliceMinT = std::numeric_limits<float>::max();
    pcaSliceMaxT = -std::numeric_limits<float>::max();
    for(const auto& p : cloud->points)
    {
        const Eigen::Vector3f pt(p.x, p.y, p.z);
        const float t = (pt - r.centroid).dot(pc1);
        pcaSliceMinT = std::min(pcaSliceMinT, t);
        pcaSliceMaxT = std::max(pcaSliceMaxT, t);
    }
    const float span = std::max(pcaSliceMaxT - pcaSliceMinT, 1e-4f);
    pcaSliceMinT -= span * 0.02f;
    pcaSliceMaxT += span * 0.02f;

    // 初始化每个切片位置（均匀分布）
    pcaActiveSliceIndex = 0;
    pcaSlicePositions.resize(numSlices);
    const float initSpan = pcaSliceMaxT - pcaSliceMinT;
    for(int i = 0; i < numSlices; ++i)
        pcaSlicePositions[i] = pcaSliceMinT + initSpan * static_cast<float>(i) / static_cast<float>(numSlices - 1);

    // 计算 PC2 / PC3 方向上的范围（用于切片平面的尺寸）
    const Eigen::Vector3f pc2 = r.eigenvectors.col(1).normalized();
    const Eigen::Vector3f pc3 = r.eigenvectors.col(2).normalized();
    float pc2Min = std::numeric_limits<float>::max(), pc2Max = -pc2Min;
    float pc3Min = pc2Min, pc3Max = pc2Max;
    for(const auto& p : cloud->points)
    {
        const Eigen::Vector3f pt(p.x, p.y, p.z);
        const Eigen::Vector3f d = pt - r.centroid;
        pc2Min = std::min(pc2Min, d.dot(pc2)); pc2Max = std::max(pc2Max, d.dot(pc2));
        pc3Min = std::min(pc3Min, d.dot(pc3)); pc3Max = std::max(pc3Max, d.dot(pc3));
    }
    const float pcExtent = std::max(pc2Max - pc2Min, pc3Max - pc3Min) * 0.6f;
    ui->DicGLWidget->setPCASlicePlaneExtent(pcExtent);

    // 只修正 up 向量为 PC1 方向（保证鼠标 Y→t 映射方向正确），不改变位置和视线
    ui->DicGLWidget->camera()->setUpVector(qglviewer::Vec(pc1.x(), pc1.y(), pc1.z()));

    // 启用 DicGLWidget 的 PCA 切片拖动
    ui->DicGLWidget->setMouseTracking(true);
    ui->DicGLWidget->enterPCASliceMode(pc1, r.centroid, pcaSliceMinT, pcaSliceMaxT,
                                        pcaSlicePositions, pcaActiveSliceIndex, tolerance * 0.5f);

    // 连接信号：鼠标拖拽时同步
    connect(ui->DicGLWidget, &DICQGLViewer::pcaSlicePositionChanged,
            this, [this](int sliceIndex, float tVal) {
                if(sliceIndex >= 0 && sliceIndex < static_cast<int>(pcaSlicePositions.size()))
                {
                    pcaSlicePositions[sliceIndex] = tVal;
                    updatePCASlice();
                }
            }, Qt::UniqueConnection);

    // 连接信号：点击 3D 视图中的截面来切换激活切片
    connect(ui->DicGLWidget, &DICQGLViewer::pcaActiveSliceChanged,
            this, [this](int sliceIndex) {
                pcaActiveSliceIndex = sliceIndex;
                const float t = pcaSlicePositions[sliceIndex];
                if(QSpinBox* sp = pcaSlicePanel->findChild<QSpinBox*>(QStringLiteral("pcaSliceSpin"))) {
                    QSignalBlocker blk(sp);
                    sp->setValue(sliceIndex + 1);
                }
                if(QDoubleSpinBox* vs = pcaSlicePanel->findChild<QDoubleSpinBox*>(QStringLiteral("pcaValSpin"))) {
                    QSignalBlocker blk(vs);
                    vs->setRange(static_cast<double>(pcaSliceMinT), static_cast<double>(pcaSliceMaxT));
                    vs->setValue(static_cast<double>(t));
                }
                if(QSlider* sl = pcaSlicePanel->findChild<QSlider*>(QStringLiteral("pcaSliceSlider"))) {
                    QSignalBlocker blk(sl);
                    const float frac = (t - pcaSliceMinT) / std::max(1e-6f, pcaSliceMaxT - pcaSliceMinT);
                    sl->setValue(static_cast<int>(frac * 1000.0f));
                }
                if(QLabel* lb = pcaSlicePanel->findChild<QLabel*>(QStringLiteral("pcaPosLabel")))
                    lb->setText(QStringLiteral("t = %1").arg(t, 0, 'f', 3));
                updatePCASlice();
            }, Qt::UniqueConnection);

    // 创建浮动确认/取消面板
    if(!pcaSlicePanel)
    {
        pcaSlicePanel = new QWidget(this);
        pcaSlicePanel->setObjectName(QStringLiteral("pcaSlicePanel"));
        pcaSlicePanel->setAttribute(Qt::WA_ShowWithoutActivating);
        pcaSlicePanel->setMouseTracking(true);
        pcaSlicePanel->installEventFilter(this);
        pcaSlicePanel->setStyleSheet(
            "QWidget#pcaSlicePanel {"
            "  background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #2a3a55, stop:1 #1a2538);"
            "  border: 1px solid #3a5070;"
            "  border-radius: 10px;"
            "  padding: 10px;"
            "}"
            "QLabel { color: #c8d8f0; font-size: 12px; }"
            "QSpinBox, QDoubleSpinBox {"
            "  background: #1e2d3d;"
            "  color: #c8d8f0;"
            "  border: 1px solid #3a5070;"
            "  border-radius: 4px;"
            "  padding: 3px 6px;"
            "  font-size: 12px;"
            "}"
            "QSpinBox::up-button, QSpinBox::down-button,"
            "QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {"
            "  background: #2a3a50;"
            "  border-radius: 2px;"
            "}"
            "QSlider::groove:horizontal {"
            "  background: #2a3a50;"
            "  height: 6px;"
            "  border-radius: 3px;"
            "}"
            "QSlider::handle:horizontal {"
            "  background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #5b9cf5, stop:1 #2d6fcf);"
            "  width: 14px;"
            "  margin: -5px 0;"
            "  border-radius: 7px;"
            "}"
            "QSlider::sub-page:horizontal {"
            "  background: #3a6090;"
            "  border-radius: 3px;"
            "}"
            "QPushButton {"
            "  background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #3a7ce0, stop:1 #2563c7);"
            "  color: white;"
            "  border: none;"
            "  border-radius: 5px;"
            "  padding: 6px 14px;"
            "  min-width: 65px;"
            "  font-size: 12px;"
            "}"
            "QPushButton:hover { background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #5090f0, stop:1 #3a7ce0); }"
            "QPushButton#pcaCancelBtn { background: #4a4a5a; }"
            "QPushButton#pcaCancelBtn:hover { background: #6a6a7a; }");

        QVBoxLayout* lay = new QVBoxLayout(pcaSlicePanel);
        lay->setContentsMargins(12, 10, 12, 10);
        lay->setSpacing(8);

        QLabel* title = new QLabel(tr("PCA Slice Control"));
        title->setStyleSheet("font-weight: bold; font-size: 13px; color: #e8f0ff;");
        lay->addWidget(title);

        // Slice selector row
        QHBoxLayout* sliceRow = new QHBoxLayout;
        QLabel* sliceLbl = new QLabel(tr("Slice:"));
        sliceLbl->setFixedWidth(32);
        sliceRow->addWidget(sliceLbl);
        QSpinBox* sliceSpin = new QSpinBox;
        sliceSpin->setObjectName(QStringLiteral("pcaSliceSpin"));
        sliceSpin->setMinimum(1);
        sliceSpin->setMaximum(1);
        sliceSpin->setValue(1);
        sliceSpin->setToolTip(tr("Select which slice to adjust"));
        sliceRow->addWidget(sliceSpin);
        QLabel* ofLbl = new QLabel(QStringLiteral("/ 1"));
        ofLbl->setObjectName(QStringLiteral("pcaSliceOfLabel"));
        ofLbl->setStyleSheet("color: #90b0d0;");
        sliceRow->addWidget(ofLbl);
        sliceRow->addStretch();
        lay->addLayout(sliceRow);

        QLabel* tip = new QLabel(tr("Drag on viewport or use slider"));
        tip->setWordWrap(true);
        lay->addWidget(tip);

        QHBoxLayout* valRow = new QHBoxLayout;
        QLabel* valLbl = new QLabel(tr("t ="));
        valLbl->setStyleSheet("color: #c8d8f0; font-size: 12px;");
        valRow->addWidget(valLbl);
        QDoubleSpinBox* valSpin = new QDoubleSpinBox;
        valSpin->setObjectName(QStringLiteral("pcaValSpin"));
        valSpin->setDecimals(4);
        valSpin->setRange(-10000.0, 10000.0);
        valSpin->setSingleStep(0.1);
        valSpin->setToolTip(tr("Enter exact t-value for active slice"));
        valRow->addWidget(valSpin);
        lay->addLayout(valRow);

        QSlider* sliceSlider = new QSlider(Qt::Horizontal);
        sliceSlider->setObjectName(QStringLiteral("pcaSliceSlider"));
        sliceSlider->setRange(0, 1000);
        sliceSlider->setValue(500);
        lay->addWidget(sliceSlider);

        QLabel* posLabel = new QLabel(QStringLiteral("t = 0.000"));
        posLabel->setObjectName(QStringLiteral("pcaPosLabel"));
        posLabel->setAlignment(Qt::AlignCenter);
        posLabel->setStyleSheet("color: #90b0d0; font-size: 11px;");
        lay->addWidget(posLabel);

        QHBoxLayout* btnRow = new QHBoxLayout;
        QPushButton* confirmBtn = new QPushButton(tr("Confirm"));
        confirmBtn->setObjectName(QStringLiteral("pcaConfirmBtn"));
        QPushButton* cancelBtn = new QPushButton(tr("Cancel"));
        cancelBtn->setObjectName(QStringLiteral("pcaCancelBtn"));
        btnRow->addWidget(confirmBtn);
        btnRow->addWidget(cancelBtn);
        lay->addLayout(btnRow);

        // SpinBox: switch active slice
        connect(sliceSpin, QOverload<int>::of(&QSpinBox::valueChanged), this,
            [this, sliceSpin, sliceSlider, valSpin, posLabel](int val) {
                const int idx = val - 1; // 1-indexed → 0-indexed
                if(idx < 0 || idx >= static_cast<int>(pcaSlicePositions.size())) return;
                pcaActiveSliceIndex = idx;
                ui->DicGLWidget->setActiveSliceIndex(idx);
                if(idx < static_cast<int>(pcaSlicePositions.size())) {
                    const float t = pcaSlicePositions[idx];
                    QSignalBlocker b1(sliceSlider);
                    sliceSlider->setValue(static_cast<int>(
                        (t - pcaSliceMinT) / std::max(1e-6f, pcaSliceMaxT - pcaSliceMinT) * 1000.0f));
                    QSignalBlocker b2(valSpin);
                    valSpin->setValue(static_cast<double>(t));
                    posLabel->setText(QStringLiteral("t = %1").arg(t, 0, 'f', 3));
                }
                updatePCASlice();
            });

        // valSpin: manual numeric t-value input
        connect(valSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            [this, sliceSlider, posLabel](double val) {
                if(!pcaSlicingActive) return;
                if(pcaActiveSliceIndex < 0 || pcaActiveSliceIndex >= static_cast<int>(pcaSlicePositions.size()))
                    return;
                const float t = static_cast<float>(val);
                pcaSlicePositions[pcaActiveSliceIndex] = t;
                ui->DicGLWidget->setActiveSlicePosition(t);
                QSignalBlocker blk(sliceSlider);
                const float frac = (t - pcaSliceMinT) / std::max(1e-6f, pcaSliceMaxT - pcaSliceMinT);
                sliceSlider->setValue(static_cast<int>(frac * 1000.0f));
                updatePCASlice();
                posLabel->setText(QStringLiteral("t = %1").arg(t, 0, 'f', 3));
            });

        // Slider: move active slice
        connect(sliceSlider, &QSlider::valueChanged, this,
            [this, valSpin, posLabel](int val) {
                if(!pcaSlicingActive) return;
                if(pcaActiveSliceIndex < 0 || pcaActiveSliceIndex >= static_cast<int>(pcaSlicePositions.size()))
                    return;
                const float frac = static_cast<float>(val) / 1000.0f;
                const float t = pcaSliceMinT + frac * (pcaSliceMaxT - pcaSliceMinT);
                pcaSlicePositions[pcaActiveSliceIndex] = t;
                ui->DicGLWidget->setActiveSlicePosition(t);
                QSignalBlocker blk(valSpin);
                valSpin->setValue(static_cast<double>(t));
                updatePCASlice();
                posLabel->setText(QStringLiteral("t = %1").arg(t, 0, 'f', 3));
            });

        // Sync slider, spinbox and valSpin when mouse drag happens
        connect(ui->DicGLWidget, &DICQGLViewer::pcaSlicePositionChanged,
                this, [this, sliceSlider, valSpin, posLabel](int sliceIndex, float tVal) {
            if(sliceIndex >= 0 && sliceIndex < static_cast<int>(pcaSlicePositions.size()))
            {
                pcaSlicePositions[sliceIndex] = tVal;
                if(sliceIndex == pcaActiveSliceIndex) {
                    QSignalBlocker b1(sliceSlider);
                    const float frac = (tVal - pcaSliceMinT) / std::max(1e-6f, pcaSliceMaxT - pcaSliceMinT);
                    sliceSlider->setValue(static_cast<int>(frac * 1000.0f));
                    QSignalBlocker b2(valSpin);
                    valSpin->setValue(static_cast<double>(tVal));
                }
                posLabel->setText(QStringLiteral("t = %1").arg(
                    pcaSlicePositions[pcaActiveSliceIndex], 0, 'f', 3));
            }
        });

        connect(confirmBtn, &QPushButton::clicked, this, [this]() {
            // 先移除预览点云，保证后续生成的切片索引不偏移
            if(pcaCurrentSliceCloudIndex >= 0)
            {
                ui->DicGLWidget->removePointCloud(pcaCurrentSliceCloudIndex);
                pcaCurrentSliceCloudIndex = -1;
            }

            PCLCloud::Ptr orig = ui->DicGLWidget->pointCloudByIndex(pcaOriginalCloudIndex);
            if(!orig || orig->empty()) return;

            const Eigen::Vector3f pc1c = pcaSlicingResult.eigenvectors.col(0).normalized();
            const Eigen::Vector3f cc = pcaSlicingResult.centroid;
            const float halfTc = pcaSliceTolerance * 0.5f;

            for(int i = 0; i < static_cast<int>(pcaSlicePositions.size()); ++i)
            {
                const float t = pcaSlicePositions[i];
                PCLCloud::Ptr sliceCloud(new PCLCloud);
                for(const auto& p : orig->points)
                {
                    const Eigen::Vector3f pt(p.x, p.y, p.z);
                    const float proj = (pt - cc).dot(pc1c);
                    if(std::abs(proj - t) <= halfTc)
                    {
                        const Eigen::Vector3f projected = pt - pc1c * (proj - t);
                        sliceCloud->points.emplace_back(projected.x(), projected.y(), projected.z());
                    }
                }
                if(sliceCloud->empty()) continue;
                sliceCloud->width = static_cast<uint32_t>(sliceCloud->points.size());
                sliceCloud->height = 1;
                ui->DicGLWidget->add_pointcloud(*sliceCloud);
                const int newIdx = ui->DicGLWidget->pointCloudCount() - 1;
                const QString name = QStringLiteral("slice_%1").arg(i, 2, 10, QLatin1Char('0'));
                addPointCloudFileListItem(name, name,
                    static_cast<int>(PointCloudEntryType::PointCloud), -1, newIdx);
            }
            exitPCASlicingMode();
        });

        connect(cancelBtn, &QPushButton::clicked, this, [this]() {
            exitPCASlicingMode();
        });
    }

    pcaSlicePanel->setVisible(true);
    pcaSlicePanel->resize(230, 305);
    pcaSlicePanel->move(width() - pcaSlicePanel->width() - 24, 50);

    // 初始化 spinbox 范围和滑块位置
    if(QSpinBox* sp = pcaSlicePanel->findChild<QSpinBox*>(QStringLiteral("pcaSliceSpin"))) {
        sp->setRange(1, static_cast<int>(pcaSlicePositions.size()));
        sp->setValue(pcaActiveSliceIndex + 1);
    }
    if(QLabel* lb = pcaSlicePanel->findChild<QLabel*>(QStringLiteral("pcaSliceOfLabel")))
        lb->setText(QStringLiteral("/ %1").arg(static_cast<int>(pcaSlicePositions.size())));
    if(QDoubleSpinBox* vs = pcaSlicePanel->findChild<QDoubleSpinBox*>(QStringLiteral("pcaValSpin"))) {
        QSignalBlocker blk(vs);
        vs->setRange(static_cast<double>(pcaSliceMinT), static_cast<double>(pcaSliceMaxT));
        vs->setValue(static_cast<double>(pcaSlicePositions[pcaActiveSliceIndex]));
    }
    if(QSlider* sl = pcaSlicePanel->findChild<QSlider*>(QStringLiteral("pcaSliceSlider"))) {
        QSignalBlocker blk(sl);
        const float frac = (pcaSlicePositions[pcaActiveSliceIndex] - pcaSliceMinT)
            / std::max(1e-6f, pcaSliceMaxT - pcaSliceMinT);
        sl->setValue(static_cast<int>(frac * 1000.0f));
    }
    if(QLabel* lb = pcaSlicePanel->findChild<QLabel*>(QStringLiteral("pcaPosLabel")))
        lb->setText(QStringLiteral("t = %1").arg(pcaSlicePositions[pcaActiveSliceIndex], 0, 'f', 3));

    updatePCASlice();
    ui->DicGLWidget->update();
}

void MainWindow::exitPCASlicingMode()
{
    pcaSlicingActive = false;
    ui->DicGLWidget->exitPCASliceMode();
    if(pcaSlicePanel)
        pcaSlicePanel->setVisible(false);

    if(pcaCurrentSliceCloudIndex >= 0)
    {
        ui->DicGLWidget->removePointCloud(pcaCurrentSliceCloudIndex);
        pcaCurrentSliceCloudIndex = -1;
    }

    ui->DicGLWidget->update();
}

void MainWindow::updatePCASlice()
{
    if(!pcaSlicingActive) return;

    const Eigen::Vector3f pc1 = pcaSlicingResult.eigenvectors.col(0).normalized();
    const Eigen::Vector3f c = pcaSlicingResult.centroid;

    if(pcaCurrentSliceCloudIndex >= 0)
    {
        ui->DicGLWidget->removePointCloud(pcaCurrentSliceCloudIndex);
        pcaCurrentSliceCloudIndex = -1;
    }

    if(pcaActiveSliceIndex < 0 || pcaActiveSliceIndex >= static_cast<int>(pcaSlicePositions.size()))
    {
        ui->DicGLWidget->update();
        return;
    }

    const float tCenter = pcaSlicePositions[pcaActiveSliceIndex];
    const float halfT = pcaSliceTolerance * 0.5f;

    PCLCloud::Ptr sliceCloud(new PCLCloud);
    PCLCloud::Ptr origCloud = ui->DicGLWidget->pointCloudByIndex(pcaOriginalCloudIndex);
    if(!origCloud || origCloud->empty()) { ui->DicGLWidget->update(); return; }

    for(const auto& p : origCloud->points)
    {
        const Eigen::Vector3f pt(p.x, p.y, p.z);
        const float t = (pt - c).dot(pc1);
        if(std::abs(t - tCenter) <= halfT)
        {
            const float offset = t - tCenter;
            const Eigen::Vector3f proj = pt - pc1 * offset;
            sliceCloud->points.emplace_back(proj.x(), proj.y(), proj.z());
        }
    }

    sliceCloud->width = static_cast<uint32_t>(sliceCloud->points.size());
    sliceCloud->height = 1;

    if(!sliceCloud->empty())
    {
        ui->DicGLWidget->add_pointcloud(*sliceCloud);
        pcaCurrentSliceCloudIndex = ui->DicGLWidget->pointCloudCount() - 1;
    }

    ui->DicGLWidget->update();
}

// 对选中点云执行法向估计并一致化。
void MainWindow::on_NormalConsistency_clicked()
{
    if(!ui || !ui->DicGLWidget || !ui->listWidgetPointCloudFiles || !ui->listWidgetPointCloudFiles->selectionModel() || !pointCloudListModel)
        return;

    const QModelIndexList selectedRows = ui->listWidgetPointCloudFiles->selectionModel()->selectedRows();
    if(selectedRows.isEmpty())
    {
        QMessageBox::information(this, tr("Normal Consistency"), tr("Please select one point cloud entry first."));
        return;
    }

    QDialog dlg(this);
    Ui::NormalConsistencyDialog dlgUi;
    dlgUi.setupUi(&dlg);
    connect(dlgUi.buttonBox, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(dlgUi.buttonBox, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    if(dlg.exec() != QDialog::Accepted) return;

    const int kSearch = dlgUi.spinK->value();

    int processedCount = 0;
    for(const QModelIndex& idx : selectedRows)
    {
        const QStandardItem* item = pointCloudListModel->item(idx.row());
        if(!item || item->data(RoleEntryType).toInt() != static_cast<int>(PointCloudEntryType::PointCloud))
            continue;

        const int cloudIndex = item->data(RolePointCloudIndex).toInt();
        if(cloudIndex < 0)
            continue;

        PCLCloud::Ptr input = ui->DicGLWidget->pointCloudByIndex(cloudIndex);
        if(!input || input->empty())
            continue;

        pcl::PointCloud<pcl::Normal>::Ptr normals = Point_Algorithm::estimateConsistentNormals(input, kSearch);
        if(!normals || normals->empty())
            continue;

        // 生成新点云条目（携带一致化法向）
        PCLCloud::Ptr output(new PCLCloud(*input));
        ui->DicGLWidget->add_pointcloud(*output);
        const int newIndex = ui->DicGLWidget->pointCloudCount() - 1;
        ui->DicGLWidget->setPointCloudNormals(newIndex, normals);

        const QString baseName = item->text().trimmed().isEmpty() ? QStringLiteral("cloud_%1").arg(cloudIndex + 1) : item->text().trimmed();
        const QString resultName = baseName + QStringLiteral("_Normal");
        addPointCloudFileListItem(resultName, resultName,
                                  static_cast<int>(PointCloudEntryType::PointCloud), -1, newIndex);
        ++processedCount;
    }

    if(processedCount <= 0)
    {
        QMessageBox::warning(this, tr("Normal Consistency"), tr("No selected point cloud could be processed."));
        return;
    }

    if(ui && ui->plainTextEdit)
        ui->plainTextEdit->appendPlainText(
            systemTime() + " " + tr("Normal consistency complete: %1 point cloud(s) processed. Ready for Poisson reconstruction.")
                .arg(processedCount));

    ui->DicGLWidget->update();
}

// 对当前选中点云执行表面重建并新增结果条目。
void MainWindow::on_Centroid_Method_clicked()
{
    if(!ui || !ui->DicGLWidget || !ui->listWidgetPointCloudFiles || !ui->listWidgetPointCloudFiles->selectionModel() || !pointCloudListModel)
        return;

    const QModelIndexList selectedRows = ui->listWidgetPointCloudFiles->selectionModel()->selectedRows();
    if(selectedRows.isEmpty())
    {
        QMessageBox::information(this, tr("Voxel Grid"), tr("Please select one point cloud entry first."));
        return;
    }

    QDialog dlg(this);
    Ui::VoxelGridDialog dlgUi;
    dlgUi.setupUi(&dlg);
    connect(dlgUi.buttonBox, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(dlgUi.buttonBox, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    if(dlg.exec() != QDialog::Accepted) return;

    const double leaf = dlgUi.spinLeaf->value();

    int processedCount = 0;
    for(const QModelIndex& idx : selectedRows)
    {
        const QStandardItem* item = pointCloudListModel->item(idx.row());
        if(!item || item->data(RoleEntryType).toInt() != static_cast<int>(PointCloudEntryType::PointCloud))
            continue;

        const int cloudIndex = item->data(RolePointCloudIndex).toInt();
        if(cloudIndex < 0)
            continue;

        PCLCloud::Ptr input = ui->DicGLWidget->pointCloudByIndex(cloudIndex);
        if(!input || input->empty())
            continue;

        PCLCloud::Ptr output = Point_Algorithm::voxelGridDownsample(input,
                                                                     static_cast<float>(leaf),
                                                                     static_cast<float>(leaf),
                                                                     static_cast<float>(leaf));
        if(!output || output->empty())
            continue;

        ui->DicGLWidget->add_pointcloud(*output);
        const int newIndex = ui->DicGLWidget->pointCloudCount() - 1;
        const QString baseName = item->text().trimmed().isEmpty() ? QStringLiteral("cloud_%1").arg(cloudIndex + 1) : item->text().trimmed();
        const QString resultName = baseName + QStringLiteral("_Voxel");
        addPointCloudFileListItem(resultName, resultName,
                                  static_cast<int>(PointCloudEntryType::PointCloud), -1, newIndex);
        ++processedCount;
    }

    if(processedCount <= 0)
    {
        QMessageBox::warning(this, tr("Voxel Grid"), tr("No selected point cloud could be processed."));
        return;
    }

    ui->DicGLWidget->update();
}

// 对选中点云执行 MLS 平滑重采样。
void MainWindow::on_MLS_Method_clicked()
{
    if(!ui || !ui->DicGLWidget || !ui->listWidgetPointCloudFiles || !ui->listWidgetPointCloudFiles->selectionModel() || !pointCloudListModel)
        return;

    const QModelIndexList selectedRows = ui->listWidgetPointCloudFiles->selectionModel()->selectedRows();
    if(selectedRows.isEmpty())
    {
        QMessageBox::information(this, tr("MLS Smoothing"), tr("Please select one point cloud entry first."));
        return;
    }

    QDialog dlg(this);
    Ui::MLSDialog dlgUi;
    dlgUi.setupUi(&dlg);
    connect(dlgUi.buttonBox, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(dlgUi.buttonBox, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    if(dlg.exec() != QDialog::Accepted) return;

    const double searchRadius = dlgUi.spinRadius->value();
    const int polyOrder = dlgUi.spinOrder->value();

    int processedCount = 0;
    for(const QModelIndex& idx : selectedRows)
    {
        const QStandardItem* item = pointCloudListModel->item(idx.row());
        if(!item || item->data(RoleEntryType).toInt() != static_cast<int>(PointCloudEntryType::PointCloud))
            continue;

        const int cloudIndex = item->data(RolePointCloudIndex).toInt();
        if(cloudIndex < 0)
            continue;

        PCLCloud::Ptr input = ui->DicGLWidget->pointCloudByIndex(cloudIndex);
        if(!input || input->empty())
            continue;

        PCLCloud::Ptr output = Point_Algorithm::mlsSmoothing(input, searchRadius, polyOrder);
        if(!output || output->empty())
            continue;

        ui->DicGLWidget->add_pointcloud(*output);
        const int newIndex = ui->DicGLWidget->pointCloudCount() - 1;
        const QString baseName = item->text().trimmed().isEmpty() ? QStringLiteral("cloud_%1").arg(cloudIndex + 1) : item->text().trimmed();
        const QString resultName = baseName + QStringLiteral("_MLS");
        addPointCloudFileListItem(resultName, resultName,
                                  static_cast<int>(PointCloudEntryType::PointCloud), -1, newIndex);
        ++processedCount;
    }

    if(processedCount <= 0)
    {
        QMessageBox::warning(this, tr("MLS Smoothing"), tr("No selected point cloud could be processed."));
        return;
    }

    ui->DicGLWidget->update();
}

// 对选中点云执行体素下采样，结果追加保存。
void MainWindow::on_SOR_Method_clicked()
{
    if(!ui || !ui->DicGLWidget || !ui->listWidgetPointCloudFiles || !ui->listWidgetPointCloudFiles->selectionModel() || !pointCloudListModel)
        return;

    const QModelIndexList selectedRows = ui->listWidgetPointCloudFiles->selectionModel()->selectedRows();
    if(selectedRows.isEmpty())
    {
        QMessageBox::information(this, tr("SOR Filter"), tr("Please select one point cloud entry first."));
        return;
    }

    QDialog dlg(this);
    Ui::SORDialog dlgUi;
    dlgUi.setupUi(&dlg);
    connect(dlgUi.buttonBox, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(dlgUi.buttonBox, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    if(dlg.exec() != QDialog::Accepted) return;

    const int meanK = dlgUi.spinMeanK->value();
    const double stddev = dlgUi.spinStddev->value();

    int processedCount = 0;
    for(const QModelIndex& idx : selectedRows)
    {
        const QStandardItem* item = pointCloudListModel->item(idx.row());
        if(!item || item->data(RoleEntryType).toInt() != static_cast<int>(PointCloudEntryType::PointCloud))
            continue;

        const int cloudIndex = item->data(RolePointCloudIndex).toInt();
        if(cloudIndex < 0)
            continue;

        PCLCloud::Ptr input = ui->DicGLWidget->pointCloudByIndex(cloudIndex);
        if(!input || input->empty())
            continue;

        PCLCloud::Ptr output = Point_Algorithm::sorFilter(input, meanK, stddev);
        if(!output || output->empty())
            continue;

        ui->DicGLWidget->add_pointcloud(*output);
        const int newIndex = ui->DicGLWidget->pointCloudCount() - 1;
        const QString baseName = item->text().trimmed().isEmpty() ? QStringLiteral("cloud_%1").arg(cloudIndex + 1) : item->text().trimmed();
        const QString resultName = baseName + QStringLiteral("_SOR");
        addPointCloudFileListItem(resultName, resultName,
                                  static_cast<int>(PointCloudEntryType::PointCloud), -1, newIndex);
        ++processedCount;
    }

    if(processedCount <= 0)
    {
        QMessageBox::warning(this, tr("SOR Filter"), tr("No selected point cloud could be processed."));
        return;
    }

    ui->DicGLWidget->update();
}

// 对选中点云执行 SOR 滤波，结果追加保存。
void MainWindow::on_recon_ball_clicked(){
    if(!ui || !ui->DicGLWidget || !ui->listWidgetPointCloudFiles || !ui->listWidgetPointCloudFiles->selectionModel() || !pointCloudListModel)
        return;

    const QModelIndexList selectedRows = ui->listWidgetPointCloudFiles->selectionModel()->selectedRows();
    if(selectedRows.isEmpty())
    {
        QMessageBox::information(this, tr("Recon Ball"), tr("Please select one point cloud entry first."));
        return;
    }

    int successCount = 0;
    for(const QModelIndex& idx : selectedRows)
    {
        const QStandardItem* item = pointCloudListModel->item(idx.row());
        if(!item || item->data(RoleEntryType).toInt() != static_cast<int>(PointCloudEntryType::PointCloud))
            continue;

        const int cloudIndex = item->data(RolePointCloudIndex).toInt();
        if(cloudIndex < 0)
            continue;

        PCLCloud::Ptr input = ui->DicGLWidget->pointCloudByIndex(cloudIndex);
        if(!input || input->size() < 10)
            continue;

        const auto r = Point_Algorithm::fitSphere3D(input, 0.5);
        if(!r.success || !r.inliers || r.inliers->empty())
            continue;

        ui->DicGLWidget->add_pointcloud(*r.inliers);
        const int newIndex = ui->DicGLWidget->pointCloudCount() - 1;

        const QString baseName = item->text().trimmed().isEmpty() ? QStringLiteral("cloud_%1").arg(cloudIndex + 1) : item->text().trimmed();
        const QString resultName = baseName + QStringLiteral("_ReconBall");
        addPointCloudFileListItem(resultName, resultName,
                                  static_cast<int>(PointCloudEntryType::PointCloud), -1, newIndex);

        if(ui->plainTextEdit)
        {
            ui->plainTextEdit->appendPlainText(
                systemTime() + " " + tr("ReconBall %1: inliers=%2, center=(%3,%4,%5), r=%6")
                    .arg(baseName)
                    .arg(static_cast<int>(r.inliers->size()))
                    .arg(r.center.x(), 0, 'f', 3)
                    .arg(r.center.y(), 0, 'f', 3)
                    .arg(r.center.z(), 0, 'f', 3)
                    .arg(r.radius, 0, 'f', 3));
        }

        ++successCount;
    }

    if(successCount <= 0)
    {
        QMessageBox::warning(this, tr("Recon Ball"), tr("No selected point cloud could be recognized as sphere."));
        return;
    }

    ui->DicGLWidget->update();
}


// 球面识别：保留 RANSAC 内点并新增点云。
// 对选中的点云执行几何体拟合（球/圆柱/平面/圆），并将拟合结果网格化显示。
void MainWindow::on_Fit_clicked()
{
    if(!ui || !ui->DicGLWidget || !ui->listWidgetPointCloudFiles || !ui->listWidgetPointCloudFiles->selectionModel() || !pointCloudListModel)
        return;

    QComboBox* fitSelector = findChild<QComboBox*>(QStringLiteral("Fit_select"));
    if(!fitSelector)
        fitSelector = findChild<QComboBox*>(QStringLiteral("Fit_solution"));
    if(!fitSelector)
        fitSelector = findChild<QComboBox*>(QStringLiteral("comboBox_fit"));

    QString fitKey;
    if(fitSelector)
    {
        for(int i = 0; i < fitSelector->count(); ++i)
        {
            const QString text = fitSelector->itemText(i).trimmed();
            if(text == QStringLiteral("球"))
                fitSelector->setItemData(i, QStringLiteral("sphere"));
            else if(text == QStringLiteral("圆柱"))
                fitSelector->setItemData(i, QStringLiteral("cylinder"));
            else if(text == QStringLiteral("平面"))
                fitSelector->setItemData(i, QStringLiteral("plane"));
            else if(text == QStringLiteral("圆"))
                fitSelector->setItemData(i, QStringLiteral("circle3d"));
        }

        fitKey = fitSelector->currentData().toString().trimmed().toLower();
        if(fitKey.isEmpty())
        {
            const QString txt = fitSelector->currentText().trimmed();
            if(txt == QStringLiteral("球"))
                fitKey = QStringLiteral("sphere");
            else if(txt == QStringLiteral("圆柱"))
                fitKey = QStringLiteral("cylinder");
            else if(txt == QStringLiteral("平面"))
                fitKey = QStringLiteral("plane");
            else if(txt == QStringLiteral("圆"))
                fitKey = QStringLiteral("circle3d");
        }
    }

    if(fitKey.isEmpty())
        fitKey = QStringLiteral("sphere");

    PCLCloud::Ptr areaSelected = ui->DicGLWidget->selectedPointsCloud();
    QModelIndexList selectedRows = ui->listWidgetPointCloudFiles->selectionModel()->selectedRows();
    std::sort(selectedRows.begin(), selectedRows.end(), [](const QModelIndex& a, const QModelIndex& b){
        return a.row() < b.row();
    });

    struct FitInputEntry { QString baseName; PCLCloud::Ptr cloud; };
    std::vector<FitInputEntry> fitInputs;
    fitInputs.reserve(static_cast<size_t>(selectedRows.size()) + 1);

    if(!selectedRows.isEmpty())
    {
        for(const QModelIndex& idx : selectedRows)
        {
            if(const QStandardItem* item = pointCloudListModel->item(idx.row()))
            {
                if(item->data(RoleEntryType).toInt() != static_cast<int>(PointCloudEntryType::PointCloud))
                    continue;

                const int cloudIndex = item->data(RolePointCloudIndex).toInt();
                if(cloudIndex < 0)
                    continue;

                if(PCLCloud::Ptr cloud = ui->DicGLWidget->pointCloudByIndex(cloudIndex))
                {
                    if(!cloud->empty())
                    {
                        QString baseName = item->text().trimmed();
                        if(baseName.isEmpty())
                            baseName = QStringLiteral("cloud_%1").arg(cloudIndex + 1);
                        fitInputs.push_back({baseName, cloud});
                    }
                }
            }
        }
    }
    else if(areaSelected && !areaSelected->empty())
    {
        fitInputs.push_back({QStringLiteral("Selection"), areaSelected});
    }

    if(fitInputs.empty())
    {
        QMessageBox::information(this, tr("Fit"), tr("Please box-select points in DicGLWidget or select point clouds in the list first."));
        return;
    }

    int successCount = 0;
    QModelIndex lastAddedIndex;

    // Batch naming: ask for base name when fitting multiple items
    QString fitBaseName;
    if(fitInputs.size() > 1)
    {
        const QString defaultName = fitInputs.front().baseName + QStringLiteral("_Sphere");
        bool ok = false;
        fitBaseName = QInputDialog::getText(this, tr("Batch Fit Naming"),
                                             tr("Enter base name for fitted spheres:"),
                                             QLineEdit::Normal, defaultName, &ok);
        if(!ok)
            return;
        if(fitBaseName.trimmed().isEmpty())
            fitBaseName = defaultName;
    }

    for(const FitInputEntry& entry : fitInputs)
    {
        auto buildSphereMesh = [](const Eigen::Vector3f& center, float radius, int latSteps, int lonSteps) {
            pcl::PointCloud<pcl::PointXYZ> pts;
            std::vector<pcl::Vertices> polys;
            pts.reserve((latSteps + 1) * (lonSteps + 1));

            for(int i = 0; i <= latSteps; ++i)
            {
                const float v = static_cast<float>(i) / static_cast<float>(latSteps);
                const float theta = static_cast<float>(M_PI) * v;
                const float st = std::sin(theta);
                const float ct = std::cos(theta);
                for(int j = 0; j <= lonSteps; ++j)
                {
                    const float u = static_cast<float>(j) / static_cast<float>(lonSteps);
                    const float phi = static_cast<float>(2.0 * M_PI) * u;
                    const float sp = std::sin(phi);
                    const float cp = std::cos(phi);
                    pts.emplace_back(center.x() + radius * st * cp,
                                     center.y() + radius * st * sp,
                                     center.z() + radius * ct);
                }
            }

            const int row = lonSteps + 1;
            for(int i = 0; i < latSteps; ++i)
            {
                for(int j = 0; j < lonSteps; ++j)
                {
                    const int a = i * row + j;
                    const int b = a + 1;
                    const int c = (i + 1) * row + j;
                    const int d = c + 1;
                    pcl::Vertices t1; t1.vertices = {static_cast<int>(a), static_cast<int>(c), static_cast<int>(b)};
                    pcl::Vertices t2; t2.vertices = {static_cast<int>(b), static_cast<int>(c), static_cast<int>(d)};
                    polys.push_back(t1);
                    polys.push_back(t2);
                }
            }

            PCLMesh mesh;
            pcl::toPCLPointCloud2(pts, mesh.cloud);
            mesh.polygons = std::move(polys);
            return mesh;
        };

        auto buildPlaneMesh = [](const Eigen::Vector4f& coeff,
                                 const PCLCloud::Ptr& refCloud,
                                 int gridN) {
            Eigen::Vector3f n(coeff.x(), coeff.y(), coeff.z());
            n.normalize();
            Eigen::Vector3f helper = std::fabs(n.z()) < 0.9f ? Eigen::Vector3f(0,0,1) : Eigen::Vector3f(0,1,0);
            Eigen::Vector3f u = n.cross(helper).normalized();
            Eigen::Vector3f v = n.cross(u).normalized();
            const Eigen::Vector3f origin = -coeff.w() * n;

            float minU = -10.f, maxU = 10.f, minV = -10.f, maxV = 10.f;
            if(refCloud && !refCloud->empty())
            {
                minU = minV = std::numeric_limits<float>::max();
                maxU = maxV = -std::numeric_limits<float>::max();
                for(const auto& p : refCloud->points)
                {
                    const Eigen::Vector3f q(p.x, p.y, p.z);
                    const Eigen::Vector3f r = q - origin;
                    const float pu = r.dot(u);
                    const float pv = r.dot(v);
                    minU = std::min(minU, pu); maxU = std::max(maxU, pu);
                    minV = std::min(minV, pv); maxV = std::max(maxV, pv);
                }
                const float padU = std::max((maxU - minU) * 0.08f, 1.0f);
                const float padV = std::max((maxV - minV) * 0.08f, 1.0f);
                minU -= padU; maxU += padU;
                minV -= padV; maxV += padV;
            }

            pcl::PointCloud<pcl::PointXYZ> pts;
            std::vector<pcl::Vertices> polys;
            pts.reserve((gridN + 1) * (gridN + 1));
            for(int i = 0; i <= gridN; ++i)
            {
                const float su = static_cast<float>(i) / static_cast<float>(gridN);
                const float uu = minU + (maxU - minU) * su;
                for(int j = 0; j <= gridN; ++j)
                {
                    const float sv = static_cast<float>(j) / static_cast<float>(gridN);
                    const float vv = minV + (maxV - minV) * sv;
                    const Eigen::Vector3f p = origin + uu * u + vv * v;
                    pts.emplace_back(p.x(), p.y(), p.z());
                }
            }

            const int row = gridN + 1;
            for(int i = 0; i < gridN; ++i)
            {
                for(int j = 0; j < gridN; ++j)
                {
                    const int a = i * row + j;
                    const int b = a + 1;
                    const int c = (i + 1) * row + j;
                    const int d = c + 1;
                    pcl::Vertices t1; t1.vertices = {static_cast<int>(a), static_cast<int>(c), static_cast<int>(b)};
                    pcl::Vertices t2; t2.vertices = {static_cast<int>(b), static_cast<int>(c), static_cast<int>(d)};
                    polys.push_back(t1);
                    polys.push_back(t2);
                }
            }

            PCLMesh mesh;
            pcl::toPCLPointCloud2(pts, mesh.cloud);
            mesh.polygons = std::move(polys);
            return mesh;
        };

        auto buildCylinderMesh = [](const Eigen::Vector3f& axisPoint,
                                    Eigen::Vector3f axisDir,
                                    float radius,
                                    const PCLCloud::Ptr& refCloud,
                                    int hSteps,
                                    int radialSteps) {
            axisDir.normalize();
            Eigen::Vector3f helper = std::fabs(axisDir.z()) < 0.9f ? Eigen::Vector3f(0,0,1) : Eigen::Vector3f(0,1,0);
            Eigen::Vector3f u = axisDir.cross(helper).normalized();
            Eigen::Vector3f v = axisDir.cross(u).normalized();

            float minT = -radius;
            float maxT = radius;
            if(refCloud && !refCloud->empty())
            {
                minT = std::numeric_limits<float>::max();
                maxT = -std::numeric_limits<float>::max();
                for(const auto& p : refCloud->points)
                {
                    const Eigen::Vector3f q(p.x, p.y, p.z);
                    const float t = (q - axisPoint).dot(axisDir);
                    minT = std::min(minT, t);
                    maxT = std::max(maxT, t);
                }
                if(!std::isfinite(minT) || !std::isfinite(maxT) || maxT - minT < 1e-4f)
                {
                    minT = -radius;
                    maxT = radius;
                }
            }

            pcl::PointCloud<pcl::PointXYZ> pts;
            std::vector<pcl::Vertices> polys;
            pts.reserve((hSteps + 1) * (radialSteps + 1));

            for(int i = 0; i <= hSteps; ++i)
            {
                const float t = minT + (maxT - minT) * (static_cast<float>(i) / static_cast<float>(hSteps));
                const Eigen::Vector3f c = axisPoint + axisDir * t;
                for(int j = 0; j <= radialSteps; ++j)
                {
                    const float ang = static_cast<float>(2.0 * M_PI) * (static_cast<float>(j) / static_cast<float>(radialSteps));
                    const Eigen::Vector3f p = c + radius * (std::cos(ang) * u + std::sin(ang) * v);
                    pts.emplace_back(p.x(), p.y(), p.z());
                }
            }

            const int row = radialSteps + 1;
            for(int i = 0; i < hSteps; ++i)
            {
                for(int j = 0; j < radialSteps; ++j)
                {
                    const int a = i * row + j;
                    const int b = a + 1;
                    const int c = (i + 1) * row + j;
                    const int d = c + 1;
                    pcl::Vertices t1; t1.vertices = {static_cast<int>(a), static_cast<int>(c), static_cast<int>(b)};
                    pcl::Vertices t2; t2.vertices = {static_cast<int>(b), static_cast<int>(c), static_cast<int>(d)};
                    polys.push_back(t1);
                    polys.push_back(t2);
                }
            }

            PCLMesh mesh;
            pcl::toPCLPointCloud2(pts, mesh.cloud);
            mesh.polygons = std::move(polys);
            return mesh;
        };

        auto buildCircleDiskMesh = [](const Eigen::Vector3f& center,
                                      Eigen::Vector3f normal,
                                      float radius,
                                      int radialSteps,
                                      int ringSteps) {
            normal.normalize();
            Eigen::Vector3f helper = std::fabs(normal.z()) < 0.9f ? Eigen::Vector3f(0,0,1) : Eigen::Vector3f(0,1,0);
            Eigen::Vector3f u = normal.cross(helper).normalized();
            Eigen::Vector3f v = normal.cross(u).normalized();

            pcl::PointCloud<pcl::PointXYZ> pts;
            std::vector<pcl::Vertices> polys;
            pts.reserve(1 + ringSteps * (radialSteps + 1));
            pts.emplace_back(center.x(), center.y(), center.z());

            for(int r = 1; r <= ringSteps; ++r)
            {
                const float rr = radius * (static_cast<float>(r) / static_cast<float>(ringSteps));
                for(int j = 0; j <= radialSteps; ++j)
                {
                    const float ang = static_cast<float>(2.0 * M_PI) * (static_cast<float>(j) / static_cast<float>(radialSteps));
                    const Eigen::Vector3f p = center + rr * (std::cos(ang) * u + std::sin(ang) * v);
                    pts.emplace_back(p.x(), p.y(), p.z());
                }
            }

            const int row = radialSteps + 1;
            for(int j = 0; j < radialSteps; ++j)
            {
                pcl::Vertices t;
                t.vertices = {0u, static_cast<int>(1 + j), static_cast<int>(1 + j + 1)};
                polys.push_back(t);
            }

            for(int r = 1; r < ringSteps; ++r)
            {
                const int base0 = 1 + (r - 1) * row;
                const int base1 = 1 + r * row;
                for(int j = 0; j < radialSteps; ++j)
                {
                    const int a = base0 + j;
                    const int b = a + 1;
                    const int c = base1 + j;
                    const int d = c + 1;
                    pcl::Vertices t1; t1.vertices = {static_cast<int>(a), static_cast<int>(c), static_cast<int>(b)};
                    pcl::Vertices t2; t2.vertices = {static_cast<int>(b), static_cast<int>(c), static_cast<int>(d)};
                    polys.push_back(t1);
                    polys.push_back(t2);
                }
            }

            PCLMesh mesh;
            pcl::toPCLPointCloud2(pts, mesh.cloud);
            mesh.polygons = std::move(polys);
            return mesh;
        };
        if(fitKey == QStringLiteral("sphere"))
        {
            const auto result = Point_Algorithm::fitSphere3D(entry.cloud, 0.5);
            if(!result.success)
                continue;

            auto buildPlaneMesh = [](const Eigen::Vector4f& coeff,
                                     const PCLCloud::Ptr& refCloud,
                                     int gridN) {
                Eigen::Vector3f n(coeff.x(), coeff.y(), coeff.z());
                n.normalize();
                Eigen::Vector3f helper = std::fabs(n.z()) < 0.9f ? Eigen::Vector3f(0,0,1) : Eigen::Vector3f(0,1,0);
                Eigen::Vector3f u = n.cross(helper).normalized();
                Eigen::Vector3f v = n.cross(u).normalized();
                const Eigen::Vector3f origin = -coeff.w() * n;

                float minU = -10.f, maxU = 10.f, minV = -10.f, maxV = 10.f;
                if(refCloud && !refCloud->empty())
                {
                    minU = minV = std::numeric_limits<float>::max();
                    maxU = maxV = -std::numeric_limits<float>::max();
                    for(const auto& p : refCloud->points)
                    {
                        const Eigen::Vector3f q(p.x, p.y, p.z);
                        const Eigen::Vector3f r = q - origin;
                        const float pu = r.dot(u);
                        const float pv = r.dot(v);
                        minU = std::min(minU, pu); maxU = std::max(maxU, pu);
                        minV = std::min(minV, pv); maxV = std::max(maxV, pv);
                    }
                    const float padU = std::max((maxU - minU) * 0.08f, 1.0f);
                    const float padV = std::max((maxV - minV) * 0.08f, 1.0f);
                    minU -= padU; maxU += padU;
                    minV -= padV; maxV += padV;
                }

                pcl::PointCloud<pcl::PointXYZ> pts;
                std::vector<pcl::Vertices> polys;
                pts.reserve((gridN + 1) * (gridN + 1));
                for(int i = 0; i <= gridN; ++i)
                {
                    const float su = static_cast<float>(i) / static_cast<float>(gridN);
                    const float uu = minU + (maxU - minU) * su;
                    for(int j = 0; j <= gridN; ++j)
                    {
                        const float sv = static_cast<float>(j) / static_cast<float>(gridN);
                        const float vv = minV + (maxV - minV) * sv;
                        const Eigen::Vector3f p = origin + uu * u + vv * v;
                        pts.emplace_back(p.x(), p.y(), p.z());
                    }
                }

                const int row = gridN + 1;
                for(int i = 0; i < gridN; ++i)
                {
                    for(int j = 0; j < gridN; ++j)
                    {
                        const int a = i * row + j;
                        const int b = a + 1;
                        const int c = (i + 1) * row + j;
                        const int d = c + 1;
                        pcl::Vertices t1; t1.vertices = {static_cast<int>(a), static_cast<int>(c), static_cast<int>(b)};
                        pcl::Vertices t2; t2.vertices = {static_cast<int>(b), static_cast<int>(c), static_cast<int>(d)};
                        polys.push_back(t1);
                        polys.push_back(t2);
                    }
                }

                PCLMesh mesh;
                pcl::toPCLPointCloud2(pts, mesh.cloud);
                mesh.polygons = std::move(polys);
                return mesh;
            };

            auto buildCylinderMesh = [](const Eigen::Vector3f& axisPoint,
                                        Eigen::Vector3f axisDir,
                                        float radius,
                                        const PCLCloud::Ptr& refCloud,
                                        int hSteps,
                                        int radialSteps) {
                axisDir.normalize();
                Eigen::Vector3f helper = std::fabs(axisDir.z()) < 0.9f ? Eigen::Vector3f(0,0,1) : Eigen::Vector3f(0,1,0);
                Eigen::Vector3f u = axisDir.cross(helper).normalized();
                Eigen::Vector3f v = axisDir.cross(u).normalized();

                float minT = -radius;
                float maxT = radius;
                if(refCloud && !refCloud->empty())
                {
                    minT = std::numeric_limits<float>::max();
                    maxT = -std::numeric_limits<float>::max();
                    for(const auto& p : refCloud->points)
                    {
                        const Eigen::Vector3f q(p.x, p.y, p.z);
                        const float t = (q - axisPoint).dot(axisDir);
                        minT = std::min(minT, t);
                        maxT = std::max(maxT, t);
                    }
                    if(!std::isfinite(minT) || !std::isfinite(maxT) || maxT - minT < 1e-4f)
                    {
                        minT = -radius;
                        maxT = radius;
                    }
                }

                pcl::PointCloud<pcl::PointXYZ> pts;
                std::vector<pcl::Vertices> polys;
                pts.reserve((hSteps + 1) * (radialSteps + 1));

                for(int i = 0; i <= hSteps; ++i)
                {
                    const float t = minT + (maxT - minT) * (static_cast<float>(i) / static_cast<float>(hSteps));
                    const Eigen::Vector3f c = axisPoint + axisDir * t;
                    for(int j = 0; j <= radialSteps; ++j)
                    {
                        const float ang = static_cast<float>(2.0 * M_PI) * (static_cast<float>(j) / static_cast<float>(radialSteps));
                        const Eigen::Vector3f p = c + radius * (std::cos(ang) * u + std::sin(ang) * v);
                        pts.emplace_back(p.x(), p.y(), p.z());
                    }
                }

                const int row = radialSteps + 1;
                for(int i = 0; i < hSteps; ++i)
                {
                    for(int j = 0; j < radialSteps; ++j)
                    {
                        const int a = i * row + j;
                        const int b = a + 1;
                        const int c = (i + 1) * row + j;
                        const int d = c + 1;
                        pcl::Vertices t1; t1.vertices = {static_cast<int>(a), static_cast<int>(c), static_cast<int>(b)};
                        pcl::Vertices t2; t2.vertices = {static_cast<int>(b), static_cast<int>(c), static_cast<int>(d)};
                        polys.push_back(t1);
                        polys.push_back(t2);
                    }
                }

                PCLMesh mesh;
                pcl::toPCLPointCloud2(pts, mesh.cloud);
                mesh.polygons = std::move(polys);
                return mesh;
            };

            auto buildCircleDiskMesh = [](const Eigen::Vector3f& center,
                                          Eigen::Vector3f normal,
                                          float radius,
                                          int radialSteps,
                                          int ringSteps) {
                normal.normalize();
                Eigen::Vector3f helper = std::fabs(normal.z()) < 0.9f ? Eigen::Vector3f(0,0,1) : Eigen::Vector3f(0,1,0);
                Eigen::Vector3f u = normal.cross(helper).normalized();
                Eigen::Vector3f v = normal.cross(u).normalized();

                pcl::PointCloud<pcl::PointXYZ> pts;
                std::vector<pcl::Vertices> polys;
                pts.reserve(1 + ringSteps * (radialSteps + 1));
                pts.emplace_back(center.x(), center.y(), center.z());

                for(int r = 1; r <= ringSteps; ++r)
                {
                    const float rr = radius * (static_cast<float>(r) / static_cast<float>(ringSteps));
                    for(int j = 0; j <= radialSteps; ++j)
                    {
                        const float ang = static_cast<float>(2.0 * M_PI) * (static_cast<float>(j) / static_cast<float>(radialSteps));
                        const Eigen::Vector3f p = center + rr * (std::cos(ang) * u + std::sin(ang) * v);
                        pts.emplace_back(p.x(), p.y(), p.z());
                    }
                }

                const int row = radialSteps + 1;
                for(int j = 0; j < radialSteps; ++j)
                {
                    pcl::Vertices t;
                    t.vertices = {0u, static_cast<int>(1 + j), static_cast<int>(1 + j + 1)};
                    polys.push_back(t);
                }

                for(int r = 1; r < ringSteps; ++r)
                {
                    const int base0 = 1 + (r - 1) * row;
                    const int base1 = 1 + r * row;
                    for(int j = 0; j < radialSteps; ++j)
                    {
                        const int a = base0 + j;
                        const int b = a + 1;
                        const int c = base1 + j;
                        const int d = c + 1;
                        pcl::Vertices t1; t1.vertices = {static_cast<int>(a), static_cast<int>(c), static_cast<int>(b)};
                        pcl::Vertices t2; t2.vertices = {static_cast<int>(b), static_cast<int>(c), static_cast<int>(d)};
                        polys.push_back(t1);
                        polys.push_back(t2);
                    }
                }

                PCLMesh mesh;
                pcl::toPCLPointCloud2(pts, mesh.cloud);
                mesh.polygons = std::move(polys);
                return mesh;
            };

            const PCLMesh mesh = buildSphereMesh(result.center, result.radius, 72, 144);
            if(mesh.polygons.empty())
                continue;

            const int meshIndex = ui->DicGLWidget->addMesh(mesh);
            if(meshIndex < 0)
                continue;
            ui->DicGLWidget->setMeshDisplayTag(meshIndex, QStringLiteral("sphere"));

            const QString resultName = fitBaseName.isEmpty()
                ? entry.baseName + QStringLiteral("_FitSphere")
                : QString("%1_%2").arg(fitBaseName).arg(successCount + 1);
            addPointCloudFileListItem(resultName,
                                      resultName,
                                      static_cast<int>(PointCloudEntryType::FittedGeometry),
                                      meshIndex,
                                      -1,
                                      -1);

            sphereFitResults[meshIndex] = { result.center, result.radius };

            if(pointCloudListModel)
                lastAddedIndex = pointCloudListModel->index(pointCloudListModel->rowCount() - 1, 0);

            if(ui->plainTextEdit)
                ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Sphere fit: center=(%1,%2,%3), r=%4")
                                                   .arg(result.center.x(), 0, 'f', 3)
                                                   .arg(result.center.y(), 0, 'f', 3)
                                                   .arg(result.center.z(), 0, 'f', 3)
                                                   .arg(result.radius, 0, 'f', 3));

            if(fitInputs.size() == 1)
                QMessageBox::information(this, tr("Sphere Fit Result"),
                                         tr("Sphere fit succeeded.\nCenter: (%1, %2, %3)\nRadius: %4")
                                            .arg(result.center.x(), 0, 'f', 6)
                                            .arg(result.center.y(), 0, 'f', 6)
                                            .arg(result.center.z(), 0, 'f', 6)
                                            .arg(result.radius, 0, 'f', 6));
        }
        else if(fitKey == QStringLiteral("plane"))
        {
            const auto result = Point_Algorithm::fitPlane3D(entry.cloud, 0.5);
            if(!result.success)
                continue;

            const PCLMesh mesh = buildPlaneMesh(result.coefficients, entry.cloud, 80);
            if(mesh.polygons.empty())
                continue;

            const int meshIndex = ui->DicGLWidget->addMesh(mesh);
            if(meshIndex < 0)
                continue;
            ui->DicGLWidget->setMeshDisplayTag(meshIndex, QStringLiteral("plane"));

            const QString resultName = entry.baseName + QStringLiteral("_FitPlane");
            addPointCloudFileListItem(resultName, resultName,
                                      static_cast<int>(PointCloudEntryType::FittedGeometry), meshIndex, -1, -1);

            if(pointCloudListModel)
                lastAddedIndex = pointCloudListModel->index(pointCloudListModel->rowCount() - 1, 0);
        }
        else if(fitKey == QStringLiteral("cylinder"))
        {
            const auto result = Point_Algorithm::fitCylinder3D(entry.cloud, 0.1, 2000, 0.5, 0.1f, 1e6f);
            if(!result.success)
                continue;

            const PCLMesh mesh = buildCylinderMesh(result.axisPoint, result.axisDirection, result.radius, entry.cloud, 100, 120);
            if(mesh.polygons.empty())
                continue;

            const int meshIndex = ui->DicGLWidget->addMesh(mesh);
            if(meshIndex < 0)
                continue;
            ui->DicGLWidget->setMeshDisplayTag(meshIndex, QStringLiteral("cylinder"));

            const QString resultName = entry.baseName + QStringLiteral("_FitCylinder");
            addPointCloudFileListItem(resultName, resultName,
                                      static_cast<int>(PointCloudEntryType::FittedGeometry), meshIndex, -1, -1);

            if(pointCloudListModel)
                lastAddedIndex = pointCloudListModel->index(pointCloudListModel->rowCount() - 1, 0);
        }
        else if(fitKey == QStringLiteral("circle3d"))
        {
            const auto result = Point_Algorithm::fitCircle3D(entry.cloud, 2000, 0.5, 0.1f, 1e6f);
            if(!result.success)
                continue;

            const PCLMesh mesh = buildCircleDiskMesh(result.center, result.normal, result.radius, 180, 32);
            if(mesh.polygons.empty())
                continue;

            const int meshIndex = ui->DicGLWidget->addMesh(mesh);
            if(meshIndex >= 0)
                ui->DicGLWidget->setMeshDisplayTag(meshIndex, QStringLiteral("circle3d"));

            const QString circleName = entry.baseName + QStringLiteral("_FitCircle3D");
            addPointCloudFileListItem(circleName, circleName,
                                      static_cast<int>(PointCloudEntryType::FittedGeometry), meshIndex, -1, -1);

            PCLCloud::Ptr axisLine(new PCLCloud);
            const int samples = 100;
            const float halfLen = std::max(result.radius * 1.5f, 10.0f);
            axisLine->points.reserve(static_cast<std::size_t>(samples));
            for(int i = 0; i < samples; ++i)
            {
                const float u = static_cast<float>(i) / static_cast<float>(samples - 1);
                const float t = -halfLen + 2.0f * halfLen * u;
                const Eigen::Vector3f p = result.center + result.normal * t;
                axisLine->points.emplace_back(p.x(), p.y(), p.z());
            }
            axisLine->width = static_cast<uint32_t>(axisLine->points.size());
            axisLine->height = 1;
            axisLine->is_dense = true;

            const int lineIndex = ui->DicGLWidget->addLine(axisLine);
            const QString axisName = entry.baseName + QStringLiteral("_RotationAxis");
            addPointCloudFileListItem(axisName, axisName,
                                      static_cast<int>(PointCloudEntryType::FittedAxisLine), -1, -1, lineIndex);

            if(pointCloudListModel)
                lastAddedIndex = pointCloudListModel->index(pointCloudListModel->rowCount() - 1, 0);
        }
        else
        {
            QMessageBox::warning(this, tr("Fit"), tr("Unsupported fit type key: %1").arg(fitKey));
            return;
        }

        ++successCount;
    }

    if(successCount <= 0)
    {
        QMessageBox::warning(this, tr("Fit"), tr("No selected point cloud could be fitted with current option."));
        return;
    }

    if(lastAddedIndex.isValid() && ui->listWidgetPointCloudFiles && ui->listWidgetPointCloudFiles->selectionModel())
    {
        ui->listWidgetPointCloudFiles->selectionModel()->setCurrentIndex(
            lastAddedIndex, QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);
    }

    ui->DicGLWidget->update();
}





void MainWindow::on_AxisCalibrate_clicked()
{
    if(!ui || !ui->DicGLWidget || !ui->listWidgetPointCloudFiles || !ui->listWidgetPointCloudFiles->selectionModel() || !pointCloudListModel)
        return;

    QModelIndexList selectedRows = ui->listWidgetPointCloudFiles->selectionModel()->selectedRows();
    if(selectedRows.isEmpty())
    {
        const QModelIndex current = ui->listWidgetPointCloudFiles->currentIndex();
        if(current.isValid())
            selectedRows << current;
    }

    if(selectedRows.isEmpty())
    {
        QMessageBox::information(this, tr("Axis Calibrate"), tr("Please select fitted sphere result(s) first."));
        return;
    }

    std::vector<Eigen::Vector3f> centers;
    centers.reserve(static_cast<std::size_t>(selectedRows.size()) * 8);

    auto meshCentroid = [](const PCLMesh& mesh, Eigen::Vector3f& c) -> bool {
        pcl::PointCloud<pcl::PointXYZ> pts;
        pcl::fromPCLPointCloud2(mesh.cloud, pts);
        if(pts.empty())
            return false;
        Eigen::Vector3f sum = Eigen::Vector3f::Zero();
        for(const auto& p : pts.points)
            sum += Eigen::Vector3f(p.x, p.y, p.z);
        c = sum / static_cast<float>(pts.size());
        return true;
    };

    auto appendCloudPointsAsCenters = [&centers](const PCLCloud::Ptr& cloud) {
        if(!cloud || cloud->empty())
            return;
        centers.reserve(centers.size() + cloud->size());
        for(const auto& p : cloud->points)
            centers.emplace_back(p.x, p.y, p.z);
    };

    for(const QModelIndex& idx : selectedRows)
    {
        const QStandardItem* item = pointCloudListModel->item(idx.row());
        if(!item)
            continue;

        const int entryType = item->data(RoleEntryType).toInt();

        // 球拟合结果：优先使用 sphereFitResults 中存储的球心，否则用网格中心。
        if(entryType == static_cast<int>(PointCloudEntryType::FittedGeometry))
        {
            const QString itemName = item->text().trimmed();
            const int meshIndex = item->data(RoleMeshIndex).toInt();
            if(meshIndex < 0)
                continue;

            // 优先使用存储的球心
            if(sphereFitResults.contains(meshIndex))
            {
                centers.push_back(sphereFitResults[meshIndex].center);
            }
            else
            {
                // 回退：使用网格质心
                Eigen::Vector3f center;
                if(meshCentroid(ui->DicGLWidget->meshByIndex(meshIndex), center))
                    centers.push_back(center);
            }
            continue;
        }

        // 普通点云（pcd/txt 导入）：直接将点云点作为拟合输入。
        if(entryType == static_cast<int>(PointCloudEntryType::PointCloud))
        {
            const int cloudIndex = item->data(RolePointCloudIndex).toInt();
            if(cloudIndex < 0)
                continue;
            appendCloudPointsAsCenters(ui->DicGLWidget->pointCloudByIndex(cloudIndex));
            continue;
        }
    }

    if(centers.size() < 3)
    {
        QMessageBox::warning(this, tr("Axis Calibrate"), tr("At least 3 valid points are required (from sphere centers or selected point cloud points)."));
        return;
    }

    ui->AxisCalibrate->setEnabled(false);

    auto* watcher = new QFutureWatcher<Point_Algorithm::RotationAxisFitResult>(this);
    connect(watcher, &QFutureWatcher<Point_Algorithm::RotationAxisFitResult>::finished,
            this, [this, watcher, centers]() {
                const Point_Algorithm::RotationAxisFitResult r = watcher->result();
                watcher->deleteLater();
                if(ui && ui->AxisCalibrate)
                    ui->AxisCalibrate->setEnabled(true);

                if(!r.success)
                {
                    QMessageBox::warning(this, tr("Axis Calibrate"), tr("Axis fitting failed."));
                    return;
                }

                // 保存回转轴结果供后续旋转使用
                m_hasRotationAxis = true;
                m_axisPoint = r.axisPoint;
                m_axisDirection = r.axisDirection;
                m_axisRadius = r.radius;
                if(ui && ui->axisrotate)
                    ui->axisrotate->setEnabled(true);

                // 显示拟合圆轨迹（来自 Point_Algorithm 输出）
                if(r.circle3D && !r.circle3D->empty())
                {
                    ui->DicGLWidget->add_pointcloud(*r.circle3D);
                    const int circleCloudIndex = ui->DicGLWidget->pointCloudCount() - 1;
                    const QString circleName = QStringLiteral("AxisCalibrate_Circle");
                    addPointCloudFileListItem(circleName,
                                              circleName,
                                              static_cast<int>(PointCloudEntryType::PointCloud),
                                              -1,
                                              circleCloudIndex,
                                              -1);
                }

                // 由球心范围构造轴线段
                float minT = std::numeric_limits<float>::max();
                float maxT = -std::numeric_limits<float>::max();
                for(const auto& c : centers)
                {
                    const float t = (c - r.axisPoint).dot(r.axisDirection);
                    minT = std::min(minT, t);
                    maxT = std::max(maxT, t);
                }
                if(!std::isfinite(minT) || !std::isfinite(maxT) || maxT - minT < 1e-4f)
                {
                    minT = -50.f;
                    maxT = 50.f;
                }
                const float pad = std::max((maxT - minT) * 0.8f, 20.0f);
                minT -= pad;
                maxT += pad;

                PCLCloud::Ptr axisLine(new PCLCloud);
                const int samples = 100;
                axisLine->points.reserve(static_cast<std::size_t>(samples));
                for(int i = 0; i < samples; ++i)
                {
                    const float u = static_cast<float>(i) / static_cast<float>(samples - 1);
                    const float t = minT + (maxT - minT) * u;
                    const Eigen::Vector3f p = r.axisPoint + r.axisDirection * t;
                    axisLine->points.emplace_back(p.x(), p.y(), p.z());
                }
                axisLine->width = static_cast<uint32_t>(axisLine->points.size());
                axisLine->height = 1;
                axisLine->is_dense = true;

                const int lineIndex = ui->DicGLWidget->addLine(axisLine);
                const QString axisName = QStringLiteral("AxisCalibrate_RotationAxis");
                addPointCloudFileListItem(axisName,
                                          axisName,
                                          static_cast<int>(PointCloudEntryType::FittedAxisLine),
                                          -1,
                                          -1,
                                          lineIndex);

                if(ui && ui->plainTextEdit)
                {
                    ui->plainTextEdit->appendPlainText(
                        systemTime() + " " + tr("Axis fit: point=(%1,%2,%3), dir=(%4,%5,%6), radius=%7, planeRMSE=%8, circleRMSE=%9")
                            .arg(r.axisPoint.x(), 0, 'f', 6)
                            .arg(r.axisPoint.y(), 0, 'f', 6)
                            .arg(r.axisPoint.z(), 0, 'f', 6)
                            .arg(r.axisDirection.x(), 0, 'f', 6)
                            .arg(r.axisDirection.y(), 0, 'f', 6)
                            .arg(r.axisDirection.z(), 0, 'f', 6)
                            .arg(r.radius, 0, 'f', 6)
                            .arg(r.planeRMSE, 0, 'f', 6)
                            .arg(r.circleRMSE, 0, 'f', 6));
                }

                ui->DicGLWidget->update();
            }, Qt::QueuedConnection);

    watcher->setFuture(QtConcurrent::run([centers]() {
        return Point_Algorithm::fitRotationAxisFromCenters(centers);
    }));
}

void MainWindow::on_axisrotate_clicked()
{
    if(!ui || !ui->DicGLWidget || !ui->listWidgetPointCloudFiles || !ui->listWidgetPointCloudFiles->selectionModel() || !pointCloudListModel)
        return;

    // 收集所有可用的回转轴线和点云
    QMap<int, QString> availableAxes;
    QMap<int, QString> availableClouds;

    for(int row = 0; row < pointCloudListModel->rowCount(); ++row)
    {
        const QStandardItem* item = pointCloudListModel->item(row);
        if(!item)
            continue;

        const int entryType = item->data(RoleEntryType).toInt();

        if(entryType == static_cast<int>(PointCloudEntryType::FittedAxisLine))
        {
            const QString itemName = item->text().trimmed();
            if(itemName.contains(QStringLiteral("AxisCalibrate"), Qt::CaseInsensitive) ||
               itemName.contains(QStringLiteral("RotationAxis"), Qt::CaseInsensitive))
            {
                const int lineIndex = item->data(RoleLineIndex).toInt();
                if(lineIndex >= 0)
                    availableAxes[lineIndex] = itemName;
            }
        }
        else if(entryType == static_cast<int>(PointCloudEntryType::PointCloud))
        {
            const int cloudIndex = item->data(RolePointCloudIndex).toInt();
            if(cloudIndex >= 0)
                availableClouds[cloudIndex] = item->text();
        }
    }

    if(availableAxes.isEmpty())
    {
        QMessageBox::warning(this, tr("Axis Rotate"),
            tr("No rotation axis found. Please run Axis Calibrate first or import a saved rotation axis."));
        return;
    }

    if(availableClouds.isEmpty())
    {
        QMessageBox::warning(this, tr("Axis Rotate"), tr("No point clouds available for rotation."));
        return;
    }

    // 创建非模态对话框
    QDialog* dlg = new QDialog(this);
    dlg->setAttribute(Qt::WA_DeleteOnClose);
    dlg->setWindowModality(Qt::NonModal);

    Ui::AxisRotateDialog* dlgUi = new Ui::AxisRotateDialog();
    dlgUi->setupUi(dlg);
    dlg->setMinimumWidth(450);
    dlg->setStyleSheet(QStringLiteral(
        "QDialog {"
        "  background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #f5f7fa, stop:1 #e4e9f0);"
        "}"
        "QGroupBox {"
        "  border: 1px solid #c0c8d0;"
        "  border-radius: 6px;"
        "  margin-top: 8px;"
        "  padding-top: 12px;"
        "  font-size: 13px;"
        "  font-weight: bold;"
        "  color: #2c3e50;"
        "}"
        "QGroupBox::title {"
        "  subcontrol-origin: margin;"
        "  left: 12px;"
        "  padding: 0 4px;"
        "}"
        "QLabel { color: #2c3e50; font-size: 13px; }"
        "QCheckBox { color: #2c3e50; font-size: 13px; spacing: 6px; }"
        "QCheckBox::indicator { width: 16px; height: 16px; }"
        "QPushButton#btnResetAllAngles {"
        "  background: #f97316;"
        "  color: white;"
        "  border: none;"
        "  border-radius: 4px;"
        "  padding: 4px 12px;"
        "  font-size: 13px;"
        "  font-weight: 600;"
        "}"
        "QPushButton#btnResetAllAngles:hover { background: #fb923c; }"
        "QDoubleSpinBox, QSpinBox, QComboBox {"
        "  background: white;"
        "  border: 1px solid #c0c8d0;"
        "  border-radius: 4px;"
        "  padding: 4px 8px;"
        "  font-size: 13px;"
        "  color: #2c3e50;"
        "}"
        "QDoubleSpinBox:focus, QSpinBox:focus, QComboBox:focus {"
        "  border-color: #3a7ce0;"
        "}"
        "QListWidget {"
        "  background: white;"
        "  border: 1px solid #c0c8d0;"
        "  border-radius: 4px;"
        "  font-size: 13px;"
        "  color: #2c3e50;"
        "}"
        "QListWidget::item:selected {"
        "  background: #3a7ce0;"
        "  color: white;"
        "}"
        "QSlider::groove:horizontal {"
        "  border: 1px solid #c0c8d0;"
        "  height: 6px;"
        "  background: #e0e0e0;"
        "  border-radius: 3px;"
        "}"
        "QSlider::handle:horizontal {"
        "  background: #3a7ce0;"
        "  border: 1px solid #2a6cc0;"
        "  width: 16px;"
        "  margin: -5px 0;"
        "  border-radius: 8px;"
        "}"
        "QSlider::handle:horizontal:hover {"
        "  background: #5090f0;"
        "}"
        "QDialogButtonBox QPushButton {"
        "  background: #3a7ce0;"
        "  color: white;"
        "  border: none;"
        "  border-radius: 4px;"
        "  padding: 6px 18px;"
        "  font-size: 13px;"
        "  min-width: 75px;"
        "}"
        "QDialogButtonBox QPushButton:hover { background: #5090f0; }"
        "QDialogButtonBox QPushButton:last-child {"
        "  background: #e0e0e0;"
        "  color: #555;"
        "}"
        "QDialogButtonBox QPushButton:last-child:hover { background: #d0d0d0; }"));

    // 填充轴线下拉框
    for(auto it = availableAxes.constBegin(); it != availableAxes.constEnd(); ++it)
        dlgUi->comboBoxAxis->addItem(it.value(), it.key());

    // 填充点云列表
    QModelIndexList selectedRows = ui->listWidgetPointCloudFiles->selectionModel()->selectedRows();
    QSet<int> preSelectedCloudIndices;
    for(const QModelIndex& idx : selectedRows)
    {
        const QStandardItem* item = pointCloudListModel->item(idx.row());
        if(item && item->data(RoleEntryType).toInt() == static_cast<int>(PointCloudEntryType::PointCloud))
        {
            const int cloudIndex = item->data(RolePointCloudIndex).toInt();
            if(cloudIndex >= 0)
                preSelectedCloudIndices.insert(cloudIndex);
        }
    }

    for(auto it = availableClouds.constBegin(); it != availableClouds.constEnd(); ++it)
    {
        QListWidgetItem* listItem = new QListWidgetItem(it.value(), dlgUi->listWidgetClouds);
        listItem->setData(Qt::UserRole, it.key());
        if(preSelectedCloudIndices.contains(it.key()))
            listItem->setSelected(true);
    }

    // 为每个点云独立存储角度和预览索引
    QMap<int, double>* cloudAngles = new QMap<int, double>();  // cloudIndex -> angle
    QMap<int, bool>* cloudClockwise = new QMap<int, bool>();   // cloudIndex -> isClockwise
    QMap<int, int>* previewCloudIndices = new QMap<int, int>(); // cloudIndex -> previewCloudIndex

    // 初始化所有点云角度为0
    for(int cloudIndex : availableClouds.keys())
    {
        (*cloudAngles)[cloudIndex] = 0.0;
        (*cloudClockwise)[cloudIndex] = false;
    }

    dlg->setProperty("cloudAngles", QVariant::fromValue(reinterpret_cast<quintptr>(cloudAngles)));
    dlg->setProperty("cloudClockwise", QVariant::fromValue(reinterpret_cast<quintptr>(cloudClockwise)));
    dlg->setProperty("previewCloudIndices", QVariant::fromValue(reinterpret_cast<quintptr>(previewCloudIndices)));

    // 标志：阻止递归更新。对话框是非模态的，函数会立即返回，
    // 因此必须堆分配，不能用栈变量按引用捕获（否则悬垂引用导致读到随机值）。
    bool* isUpdatingFromSelection = new bool(false);

    // 同步滑块和输入框
    connect(dlgUi->spinBoxAngle, QOverload<double>::of(&QDoubleSpinBox::valueChanged), dlg,
        [dlgUi, cloudAngles, isUpdatingFromSelection](double angle) {
            if(*isUpdatingFromSelection) return;

            // 阻止递归
            bool wasBlocking = dlgUi->sliderAngle->blockSignals(true);
            dlgUi->sliderAngle->setValue(static_cast<int>(angle * 100.0));
            dlgUi->sliderAngle->blockSignals(wasBlocking);

            // 保存当前选中点云的角度
            QList<QListWidgetItem*> selected = dlgUi->listWidgetClouds->selectedItems();
            for(QListWidgetItem* item : selected)
            {
                int cloudIndex = item->data(Qt::UserRole).toInt();
                (*cloudAngles)[cloudIndex] = angle;
            }
        });

    connect(dlgUi->sliderAngle, &QSlider::valueChanged, dlg,
        [dlgUi, cloudAngles, isUpdatingFromSelection](int sliderValue) {
            if(*isUpdatingFromSelection) return;

            double angle = sliderValue / 100.0;

            // 阻止递归
            bool wasBlocking = dlgUi->spinBoxAngle->blockSignals(true);
            dlgUi->spinBoxAngle->setValue(angle);
            dlgUi->spinBoxAngle->blockSignals(wasBlocking);

            // 保存当前选中点云的角度
            QList<QListWidgetItem*> selected = dlgUi->listWidgetClouds->selectedItems();
            for(QListWidgetItem* item : selected)
            {
                int cloudIndex = item->data(Qt::UserRole).toInt();
                (*cloudAngles)[cloudIndex] = angle;
            }
        });

    // 顺时针复选框变化时保存状态
    connect(dlgUi->checkBoxClockwise, &QCheckBox::toggled, dlg,
        [dlgUi, cloudClockwise](bool checked) {
            QList<QListWidgetItem*> selected = dlgUi->listWidgetClouds->selectedItems();
            for(QListWidgetItem* item : selected)
            {
                int cloudIndex = item->data(Qt::UserRole).toInt();
                (*cloudClockwise)[cloudIndex] = checked;
            }
        });

    // 实时预览函数
    auto updatePreview = [this, dlg, dlgUi, cloudAngles, cloudClockwise, previewCloudIndices]() {
        const int selectedAxisLineIndex = dlgUi->comboBoxAxis->currentData().toInt();
        PCLCloud::Ptr axisLine = ui->DicGLWidget->lineByIndex(selectedAxisLineIndex);
        if(!axisLine || axisLine->points.size() < 2)
            return;

        const auto& p0 = axisLine->points.front();
        const auto& p1 = axisLine->points.back();
        const Eigen::Vector3f axisPoint(p0.x, p0.y, p0.z);
        const Eigen::Vector3f p1_vec(p1.x, p1.y, p1.z);
        const Eigen::Vector3f axisDirection = (p1_vec - axisPoint).normalized();

        // 只对角度非零的点云生成预览；角度为 0 的点云不需要预览（预览副本会与原始点云完全重叠）
        QSet<int> cloudsToPreview;
        for(int i = 0; i < dlgUi->listWidgetClouds->count(); ++i)
        {
            QListWidgetItem* item = dlgUi->listWidgetClouds->item(i);
            int cloudIndex = item->data(Qt::UserRole).toInt();
            if(qAbs((*cloudAngles)[cloudIndex]) >= 0.01)
                cloudsToPreview.insert(cloudIndex);
        }

        // 删除不再需要的预览点云。删除会使高于该索引的点云整体前移，
        // 因此按索引降序删除，并同步修正剩余预览记录中受影响的索引。
        QList<int> keysToRemove;
        for(auto it = previewCloudIndices->constBegin(); it != previewCloudIndices->constEnd(); ++it)
        {
            if(!cloudsToPreview.contains(it.key()))
                keysToRemove.append(it.key());
        }

        // 按待删除的 viewer 索引降序处理
        QList<QPair<int, int>> removals; // (viewerIndex, cloudKey)
        for(int key : keysToRemove)
            removals.append(qMakePair((*previewCloudIndices)[key], key));
        std::sort(removals.begin(), removals.end(),
                  [](const QPair<int,int>& a, const QPair<int,int>& b){ return a.first > b.first; });

        for(const QPair<int,int>& r : removals)
        {
            const int removedViewerIndex = r.first;
            ui->DicGLWidget->removePointCloud(removedViewerIndex);
            previewCloudIndices->remove(r.second);

            // 修正剩余预览中大于被删除索引的记录（它们已整体前移 1 位）
            for(auto it = previewCloudIndices->begin(); it != previewCloudIndices->end(); ++it)
            {
                if(it.value() > removedViewerIndex)
                    it.value() -= 1;
            }
        }

        // 为每个需要预览的点云应用其独立的旋转角度
        for(int cloudIndex : cloudsToPreview)
        {
            PCLCloud::Ptr originalCloud = ui->DicGLWidget->pointCloudByIndex(cloudIndex);
            if(!originalCloud || originalCloud->empty())
                continue;

            double angleDegrees = (*cloudAngles)[cloudIndex];
            bool isClockwise = (*cloudClockwise)[cloudIndex];
            if(isClockwise)
                angleDegrees = -angleDegrees;
            const double angleRadians = angleDegrees * M_PI / 180.0;

            // 构建旋转矩阵
            const Eigen::Vector3f axis = axisDirection.normalized();
            const float c = std::cos(static_cast<float>(angleRadians));
            const float s = std::sin(static_cast<float>(angleRadians));
            const float t = 1.0f - c;

            Eigen::Matrix3f R;
            R <<
                t * axis.x() * axis.x() + c,           t * axis.x() * axis.y() - s * axis.z(), t * axis.x() * axis.z() + s * axis.y(),
                t * axis.x() * axis.y() + s * axis.z(), t * axis.y() * axis.y() + c,           t * axis.y() * axis.z() - s * axis.x(),
                t * axis.x() * axis.z() - s * axis.y(), t * axis.y() * axis.z() + s * axis.x(), t * axis.z() * axis.z() + c;

            Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
            transform.block<3, 3>(0, 0) = R;
            transform.block<3, 1>(0, 3) = axisPoint - R * axisPoint;

            PCLCloud::Ptr rotatedCloud(new PCLCloud);
            pcl::transformPointCloud(*originalCloud, *rotatedCloud, transform);

            if(previewCloudIndices->contains(cloudIndex))
            {
                const int previewIndex = (*previewCloudIndices)[cloudIndex];
                ui->DicGLWidget->replacePointCloud(previewIndex, *rotatedCloud);
            }
            else
            {
                ui->DicGLWidget->add_pointcloud(*rotatedCloud);
                const int newPreviewIndex = ui->DicGLWidget->pointCloudCount() - 1;
                (*previewCloudIndices)[cloudIndex] = newPreviewIndex;
            }
        }

        ui->DicGLWidget->update();
    };

    // 点云选择变化时，显示选中点云的角度
    connect(dlgUi->listWidgetClouds, &QListWidget::itemSelectionChanged, dlg,
        [dlgUi, cloudAngles, cloudClockwise, isUpdatingFromSelection, updatePreview]() {
            QList<QListWidgetItem*> selected = dlgUi->listWidgetClouds->selectedItems();
            if(selected.isEmpty())
                return;

            *isUpdatingFromSelection = true;

            // 显示第一个选中点云的角度
            int firstCloudIndex = selected.first()->data(Qt::UserRole).toInt();
            double angle = (*cloudAngles)[firstCloudIndex];
            bool isClockwise = (*cloudClockwise)[firstCloudIndex];

            dlgUi->spinBoxAngle->setValue(angle);
            dlgUi->sliderAngle->setValue(static_cast<int>(angle * 100.0));
            dlgUi->checkBoxClockwise->setChecked(isClockwise);

            *isUpdatingFromSelection = false;

            // 选择变化后立即刷新预览，反映新选中点云的角度
            updatePreview();
        });

    // 角度、滑块、复选框、轴线变化时更新预览
    connect(dlgUi->spinBoxAngle, QOverload<double>::of(&QDoubleSpinBox::valueChanged), dlg, updatePreview);
    connect(dlgUi->sliderAngle, &QSlider::valueChanged, dlg, updatePreview);
    connect(dlgUi->checkBoxClockwise, &QCheckBox::toggled, dlg, updatePreview);
    connect(dlgUi->comboBoxAxis, QOverload<int>::of(&QComboBox::currentIndexChanged), dlg, updatePreview);

    // "全部归零"按钮
    connect(dlgUi->btnResetAllAngles, &QPushButton::clicked, dlg,
        [dlgUi, cloudAngles, cloudClockwise, updatePreview]() {
            for(auto it = cloudAngles->begin(); it != cloudAngles->end(); ++it)
                it.value() = 0.0;
            for(auto it = cloudClockwise->begin(); it != cloudClockwise->end(); ++it)
                it.value() = false;

            dlgUi->spinBoxAngle->setValue(0.0);
            dlgUi->sliderAngle->setValue(0);
            dlgUi->checkBoxClockwise->setChecked(false);

            updatePreview();
        });

    // 对话框关闭时的处理
    connect(dlg, &QDialog::finished, this,
        [this, dlg, dlgUi, availableAxes, availableClouds, cloudAngles, cloudClockwise, previewCloudIndices, isUpdatingFromSelection](int result) {

        // 清理所有预览点云（按索引降序删除，避免删除后索引偏移导致漏删）
        QList<int> previewViewerIndices = previewCloudIndices->values();
        std::sort(previewViewerIndices.begin(), previewViewerIndices.end(), std::greater<int>());
        for(int previewIndex : previewViewerIndices)
            ui->DicGLWidget->removePointCloud(previewIndex);
        previewCloudIndices->clear();
        ui->DicGLWidget->update();

        if(result == QDialog::Accepted)
        {
            // 用户确认，执行最终旋转并保存结果
            const int selectedAxisLineIndex = dlgUi->comboBoxAxis->currentData().toInt();
            PCLCloud::Ptr axisLine = ui->DicGLWidget->lineByIndex(selectedAxisLineIndex);

            if(axisLine && axisLine->points.size() >= 2)
            {
                const auto& p0 = axisLine->points.front();
                const auto& p1 = axisLine->points.back();
                const Eigen::Vector3f axisPoint(p0.x, p0.y, p0.z);
                const Eigen::Vector3f p1_vec(p1.x, p1.y, p1.z);
                const Eigen::Vector3f axisDirection = (p1_vec - axisPoint).normalized();

                int rotatedCount = 0;
                for(auto it = cloudAngles->constBegin(); it != cloudAngles->constEnd(); ++it)
                {
                    int cloudIndex = it.key();
                    double angleDegrees = it.value();

                    // 跳过角度为0的点云
                    if(qAbs(angleDegrees) < 0.01)
                        continue;

                    PCLCloud::Ptr cloud = ui->DicGLWidget->pointCloudByIndex(cloudIndex);
                    if(!cloud || cloud->empty())
                        continue;

                    bool isClockwise = (*cloudClockwise)[cloudIndex];
                    if(isClockwise)
                        angleDegrees = -angleDegrees;
                    const double angleRadians = angleDegrees * M_PI / 180.0;

                    const Eigen::Vector3f axis = axisDirection.normalized();
                    const float c = std::cos(static_cast<float>(angleRadians));
                    const float s = std::sin(static_cast<float>(angleRadians));
                    const float t = 1.0f - c;

                    Eigen::Matrix3f R;
                    R <<
                        t * axis.x() * axis.x() + c,           t * axis.x() * axis.y() - s * axis.z(), t * axis.x() * axis.z() + s * axis.y(),
                        t * axis.x() * axis.y() + s * axis.z(), t * axis.y() * axis.y() + c,           t * axis.y() * axis.z() - s * axis.x(),
                        t * axis.x() * axis.z() - s * axis.y(), t * axis.y() * axis.z() + s * axis.x(), t * axis.z() * axis.z() + c;

                    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
                    transform.block<3, 3>(0, 0) = R;
                    transform.block<3, 1>(0, 3) = axisPoint - R * axisPoint;

                    PCLCloud::Ptr rotatedCloud(new PCLCloud);
                    pcl::transformPointCloud(*cloud, *rotatedCloud, transform);

                    ui->DicGLWidget->add_pointcloud(*rotatedCloud);
                    const int newCloudIndex = ui->DicGLWidget->pointCloudCount() - 1;

                    const QString newName = availableClouds[cloudIndex] +
                        QStringLiteral("_rotated_%1deg").arg(angleDegrees, 0, 'f', 1);
                    addPointCloudFileListItem(newName, newName,
                                              static_cast<int>(PointCloudEntryType::PointCloud),
                                              -1, newCloudIndex, -1);

                    rotatedCount++;
                }

                if(rotatedCount > 0 && ui && ui->plainTextEdit)
                {
                    ui->plainTextEdit->appendPlainText(
                        systemTime() + " " + tr("Rotated %1 point cloud(s) around axis")
                            .arg(rotatedCount));
                }

                ui->DicGLWidget->update();
            }
        }

        // 释放动态分配的内存
        delete cloudAngles;
        delete cloudClockwise;
        delete previewCloudIndices;
        delete isUpdatingFromSelection;
        delete dlgUi;
    });

    // 初始预览
    updatePreview();

    // 显示非模态对话框
    dlg->show();
}

void MainWindow::on_ICP_Trans_clicked()
{
    if(!ui || !ui->DicGLWidget || !ui->listWidgetPointCloudFiles || !pointCloudListModel)
        return;

    // 收集所有可用点云
    QVector<QString> availableClouds;
    QVector<int> cloudIndices;

    for(int i = 0; i < pointCloudListModel->rowCount(); ++i)
    {
        QStandardItem* item = pointCloudListModel->item(i);
        if(!item)
            continue;

        int entryType = item->data(RoleEntryType).toInt();
        if(entryType == static_cast<int>(PointCloudEntryType::PointCloud))
        {
            int cloudIndex = item->data(RolePointCloudIndex).toInt();
            PCLCloud::Ptr cloud = ui->DicGLWidget->pointCloudByIndex(cloudIndex);
            if(cloud && !cloud->empty())
            {
                availableClouds.append(item->data(Qt::DisplayRole).toString());
                cloudIndices.append(cloudIndex);
            }
        }
    }

    if(availableClouds.size() < 2)
    {
        if(ui->plainTextEdit)
            ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Need at least 2 point clouds for ICP registration"));
        return;
    }

    // 创建 ICP 对话框
    QDialog* dlg = new QDialog(this, Qt::Window);
    dlg->setAttribute(Qt::WA_DeleteOnClose);
    dlg->setWindowModality(Qt::NonModal);

    Ui::ICPDialog* dlgUi = new Ui::ICPDialog;
    dlgUi->setupUi(dlg);

    // 填充基准点云列表（多选，可合并）
    for(int i = 0; i < availableClouds.size(); ++i)
    {
        QListWidgetItem* item = new QListWidgetItem(availableClouds[i]);
        item->setData(Qt::UserRole, cloudIndices[i]);
        dlgUi->listWidgetReference->addItem(item);
    }

    // 填充待变换点云列表
    for(int i = 0; i < availableClouds.size(); ++i)
    {
        QListWidgetItem* item = new QListWidgetItem(availableClouds[i]);
        item->setData(Qt::UserRole, cloudIndices[i]);
        dlgUi->listWidgetSource->addItem(item);
    }

    // 对话框确认后在子线程执行 ICP
    connect(dlg, &QDialog::accepted, this,
        [this, dlg, dlgUi, availableClouds]() {

        // 收集选中的基准点云索引（多片合并为一个整体目标）
        QVector<int> referenceCloudIndices;
        for(QListWidgetItem* item : dlgUi->listWidgetReference->selectedItems())
            referenceCloudIndices.append(item->data(Qt::UserRole).toInt());

        if(referenceCloudIndices.isEmpty())
        {
            if(ui->plainTextEdit)
                ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("No reference point cloud selected"));
            return;
        }

        // 合并基准点云
        PCLCloud::Ptr referenceCloud(new PCLCloud);
        for(int idx : referenceCloudIndices)
        {
            PCLCloud::Ptr c = ui->DicGLWidget->pointCloudByIndex(idx);
            if(c && !c->empty())
                *referenceCloud += *c;
        }

        if(referenceCloud->empty())
        {
            if(ui->plainTextEdit)
                ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Reference point cloud is invalid"));
            return;
        }

        // 获取选中的源点云
        QList<QListWidgetItem*> selectedItems = dlgUi->listWidgetSource->selectedItems();
        if(selectedItems.isEmpty())
        {
            if(ui->plainTextEdit)
                ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("No source point clouds selected"));
            return;
        }

        // ICP 参数
        int maxIterations = dlgUi->spinBoxMaxIterations->value();
        double maxCorrespondenceDistance = dlgUi->spinBoxMaxDistance->value();
        double transformationEpsilon = dlgUi->spinBoxTransformEpsilon->value();
        double fitnessEpsilon = dlgUi->spinBoxFitnessEpsilon->value();
        const bool wholeTransform = dlgUi->checkBoxWholeTransform->isChecked();

        // 收集需要处理的源点云信息（排除已作为基准的点云）
        QVector<int> sourceCloudIndices;
        QVector<QString> sourceCloudNames;
        for(QListWidgetItem* item : selectedItems)
        {
            int sourceCloudIndex = item->data(Qt::UserRole).toInt();
            if(!referenceCloudIndices.contains(sourceCloudIndex))
            {
                sourceCloudIndices.append(sourceCloudIndex);
                sourceCloudNames.append(item->text());
            }
        }

        if(sourceCloudIndices.isEmpty())
        {
            if(ui->plainTextEdit)
                ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("No valid source point clouds to align"));
            return;
        }

        // 显示开始信息
        if(ui->plainTextEdit)
            ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Starting ICP registration in background thread..."));

        // 在子线程中执行 ICP
        QtConcurrent::run([this, referenceCloud, sourceCloudIndices, sourceCloudNames, wholeTransform,
                          maxIterations, maxCorrespondenceDistance, transformationEpsilon, fitnessEpsilon]() {

            QVector<PCLCloud::Ptr> alignedClouds;
            QVector<QString> alignedNames;
            QVector<float> fitnessScores;
            QVector<int> iterationCounts;
            QVector<bool> convergedFlags;

            if(wholeTransform)
            {
                // 整体变换：多片源点云合并求一个统一变换矩阵，再用同一矩阵分别变换每片。
                PCLCloud::Ptr mergedSource(new PCLCloud);
                QVector<PCLCloud::Ptr> srcClouds;
                for(int i = 0; i < sourceCloudIndices.size(); ++i)
                {
                    PCLCloud::Ptr c = ui->DicGLWidget->pointCloudByIndex(sourceCloudIndices[i]);
                    srcClouds.append(c);
                    if(c && !c->empty())
                        *mergedSource += *c;
                }

                if(mergedSource->empty())
                {
                    convergedFlags.append(false);
                }
                else
                {
                    ICP_CUDA::ICPResult icpResult = ICP_CUDA::performICP(
                        mergedSource,
                        referenceCloud,
                        maxIterations,
                        static_cast<float>(maxCorrespondenceDistance),
                        static_cast<float>(transformationEpsilon),
                        static_cast<float>(fitnessEpsilon)
                    );

                    if(icpResult.converged)
                    {
                        // 用同一矩阵变换每一片，保持各片独立成项但作为刚体一起运动
                        for(int i = 0; i < srcClouds.size(); ++i)
                        {
                            if(!srcClouds[i] || srcClouds[i]->empty())
                            {
                                convergedFlags.append(false);
                                continue;
                            }
                            PCLCloud::Ptr alignedCloud(new PCLCloud);
                            pcl::transformPointCloud(*srcClouds[i], *alignedCloud, icpResult.transformation);
                            alignedClouds.append(alignedCloud);
                            alignedNames.append(sourceCloudNames[i] + QStringLiteral("_ICP"));
                            fitnessScores.append(icpResult.fitness_score);
                            iterationCounts.append(icpResult.iterations);
                            convergedFlags.append(true);
                        }
                    }
                    else
                    {
                        convergedFlags.append(false);
                    }
                }
            }
            else
            {
                // 每片各自独立配准（原有行为）
                for(int i = 0; i < sourceCloudIndices.size(); ++i)
                {
                    int sourceCloudIndex = sourceCloudIndices[i];
                    PCLCloud::Ptr sourceCloud = ui->DicGLWidget->pointCloudByIndex(sourceCloudIndex);

                    if(!sourceCloud || sourceCloud->empty())
                    {
                        convergedFlags.append(false);
                        continue;
                    }

                    ICP_CUDA::ICPResult icpResult = ICP_CUDA::performICP(
                        sourceCloud,
                        referenceCloud,
                        maxIterations,
                        static_cast<float>(maxCorrespondenceDistance),
                        static_cast<float>(transformationEpsilon),
                        static_cast<float>(fitnessEpsilon)
                    );

                    if(icpResult.converged)
                    {
                        PCLCloud::Ptr alignedCloud(new PCLCloud);
                        pcl::transformPointCloud(*sourceCloud, *alignedCloud, icpResult.transformation);

                        alignedClouds.append(alignedCloud);
                        alignedNames.append(sourceCloudNames[i] + QStringLiteral("_ICP"));
                        fitnessScores.append(icpResult.fitness_score);
                        iterationCounts.append(icpResult.iterations);
                        convergedFlags.append(true);
                    }
                    else
                    {
                        convergedFlags.append(false);
                    }
                }
            }

            // 回到主线程更新 UI
            QMetaObject::invokeMethod(this, [this, alignedClouds, alignedNames, fitnessScores,
                                             iterationCounts, convergedFlags, sourceCloudNames]() {
                int alignedCount = 0;

                for(int i = 0; i < alignedClouds.size(); ++i)
                {
                    ui->DicGLWidget->add_pointcloud(*alignedClouds[i]);
                    const int newCloudIndex = ui->DicGLWidget->pointCloudCount() - 1;

                    addPointCloudFileListItem(alignedNames[i], alignedNames[i],
                                              static_cast<int>(PointCloudEntryType::PointCloud),
                                              -1, newCloudIndex, -1);

                    if(ui->plainTextEdit)
                    {
                        ui->plainTextEdit->appendPlainText(
                            systemTime() + " " +
                            tr("CUDA ICP converged for '%1': fitness score = %2, iterations = %3")
                                .arg(sourceCloudNames[i])
                                .arg(fitnessScores[i], 0, 'e', 6)
                                .arg(iterationCounts[i]));
                    }

                    alignedCount++;
                }

                // 输出失败信息
                for(int i = 0; i < convergedFlags.size(); ++i)
                {
                    if(!convergedFlags[i] && i < sourceCloudNames.size())
                    {
                        if(ui->plainTextEdit)
                        {
                            ui->plainTextEdit->appendPlainText(
                                systemTime() + " " +
                                tr("CUDA ICP failed to converge for '%1'").arg(sourceCloudNames[i]));
                        }
                    }
                }

                if(alignedCount > 0)
                {
                    ui->DicGLWidget->update();
                    if(ui->plainTextEdit)
                    {
                        ui->plainTextEdit->appendPlainText(
                            systemTime() + " " +
                            tr("ICP registration completed: %1 point cloud(s) aligned")
                                .arg(alignedCount));
                    }
                }
            }, Qt::QueuedConnection);
        });
    });

    // 对话框关闭时释放 UI
    connect(dlg, &QDialog::finished, this,
        [dlgUi]() {
            delete dlgUi;
        });

    dlg->show();
}

//把STL转换成PCD
void MainWindow::on_ToPcd_clicked()
{
    if(!ui || !ui->DicGLWidget || !ui->listWidgetPointCloudFiles || !pointCloudListModel)
        return;

    const QModelIndex current = ui->listWidgetPointCloudFiles->currentIndex();
    if(!current.isValid())
    {
        QMessageBox::information(this, tr("STL to PCD"), tr("Please select an STL mesh file in the list first."));
        return;
    }

    QStandardItem* item = pointCloudListModel->itemFromIndex(current);
    if(!item)
        return;

    const int entryType = item->data(RoleEntryType).toInt();
    if(entryType != static_cast<int>(PointCloudEntryType::Reconstruction))
    {
        QMessageBox::information(this, tr("STL to PCD"), tr("Please select an STL mesh file (not a point cloud or geometry entry)."));
        return;
    }

    const int meshIndex = item->data(RoleMeshIndex).toInt();
    if(meshIndex < 0)
    {
        QMessageBox::information(this, tr("STL to PCD"), tr("Invalid mesh index."));
        return;
    }

    const PCLMesh mesh = ui->DicGLWidget->meshByIndex(meshIndex); //获取网格数据
    if(mesh.polygons.empty())
    {
        QMessageBox::information(this, tr("STL to PCD"), tr("The selected mesh data is empty."));
        return;
    }

    QDialog dlg(this);
    Ui::StlToPcdDialog dlgUi;
    dlgUi.setupUi(&dlg);
    connect(dlgUi.buttonBox, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(dlgUi.buttonBox, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    if(dlg.exec() != QDialog::Accepted)
        return;
    //设置采样点数
    const int numPoints = dlgUi.spinBox->value();

    if(ui && ui->plainTextEdit)
        ui->plainTextEdit->appendPlainText(
            systemTime() + " " + tr("Starting STL to PCD sampling: %1 points...").arg(numPoints));
    //调用samplePointsFromMesh
    PCLCloud::Ptr sampled = Point_Algorithm::samplePointsFromMesh(mesh, numPoints);

    if(!sampled || sampled->empty())
    {
        if(ui && ui->plainTextEdit)
            ui->plainTextEdit->appendPlainText(
                systemTime() + " " + tr("Sampling failed, generated point cloud is empty."));
        return;
    }

    ui->DicGLWidget->add_pointcloud(*sampled);
    const int newCloudIndex = ui->DicGLWidget->pointCloudCount() - 1;

    const QFileInfo fi(item->data(Qt::UserRole).toString());
    const QString displayName = fi.completeBaseName().isEmpty()
        ? fi.fileName()
        : fi.completeBaseName();
    const QString cloudName = displayName + tr("_pcd");

    addPointCloudFileListItem(
        fi.absoluteFilePath(),
        cloudName,
        static_cast<int>(PointCloudEntryType::PointCloud),
        -1,
        newCloudIndex,
        -1);

    if(ui && ui->plainTextEdit)
        ui->plainTextEdit->appendPlainText(
            systemTime() + " " + tr("STL to PCD complete: %1 points").arg(sampled->size()));
}





void MainWindow::on_RecordY_clicked()
{
    double y = plcSnapshot.m_new_sensor1_Y;
    double sensor = g_Global->g_measureResult[g_Global->singleChannelIndex];

    // 参数校验，无效时提前返回。
    if (sensor == 888888) {
        QMessageBox::warning(this, tr("Invalid Data"), tr("The current confocal data is invalid. Please remeasure."));
        return;
    }

    QFile file(ball_A_path);
    if (!file.open(QIODevice::Append | QIODevice::Text)) {
        QMessageBox::warning(this, tr("Write Failed"), tr("Failed to open file: ") + ball_A_path);
        return;
    }

    QTextStream out(&file);
    out << QString::number(y, 'f', 6) << "\n";
    file.close();
}


void MainWindow::on_RecordSensor_clicked()
{
    double sensor = g_Global->g_measureResult[g_Global->singleChannelIndex];

    // 参数校验，无效时提前返回。
    if (sensor == 888888) {
        QMessageBox::warning(this, tr("Invalid Data"), tr("The current confocal data is invalid. Please remeasure."));
        return;
    }

    QFile file(ball_B_path);
    if (!file.open(QIODevice::Append | QIODevice::Text)) {
        QMessageBox::warning(this, tr("Write Failed"), tr("Failed to open file: ") + ball_B_path);
        return;
    }

    QTextStream out(&file);
    out << QString::number(sensor, 'f', 6) << "\n";
    file.close();
}


void MainWindow::on_lcdNumberSingleChannelResult_overflow()
{

}



void MainWindow::mahrSensor77()
{
    jingtai[jishu]=g_Global->g_measureResult[g_Global->singleChannelIndex];
    //jiaodu[jishu]=plcController->newangle;
    jishu++;
    if(jishu > jingtai.size()-2)
    {
        mahrTimer77->stop();
        //QString savePath = "E:/Code/QT/auto_measurement_cmake_521-77/data/jingtai_data.txt";
        QString timeStr = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
        QString basePath = ui->cali_data_root->text().trimmed();

        // 参数校验，无效时提前返回。
        if (basePath.isEmpty()) {
            qWarning() << "鏍囧畾璺緞涓虹┖";
            return; // 终止后续操作，防止崩溃
        }

        // 参数校验，无效时提前返回。
        // 参数校验，无效时提前返回。
        if (!basePath.endsWith(QDir::separator())) {
            basePath += QDir::separator();
        }
        QString savename = QString("jingtai_%1.txt").arg(timeStr);
        QString savePath = basePath + savename;
        //QString savePath = QString("E:/Code/QT/auto_measurement_cmake_521-77/data/jingtai_data_%1.txt")
        saveJingtaiToFile(jingtai, savePath);
        ui->plainTextEdit->appendPlainText(QString("jingtai save succeed!%1").arg(timeStr));
    }

}


void MainWindow::on_Motionless_clicked()
{
    // mahrTimer77->start();
    jishu = 0 ;

    mahrTimer77 = new QTimer(this);
    mahrTimer77->start(100);
    connect(mahrTimer77,&QTimer::timeout,this,&MainWindow::mahrSensor77);
    QString uu="jingtai is begin!";
    ui->plainTextEdit->appendPlainText(uu);

}

void MainWindow::saveJingtaiToFile( QVector<double>& jingtai, const QString& filePath) {
    QFile file(filePath);

    // 参数校验，无效时提前返回。
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qDebug() << "fail" << file.errorString();
        return;
    }

    QTextStream out(&file);

    for (double value : jingtai) {
        out << value << "\n"; // 写入数据并换行
    }

    file.close();
    qDebug() << "数据保存成功，路径：" << filePath;
}


/*************************************************************************************
  * ************************************************************************************/
/*************************************************************************************
  * ************************************************************************************/
void MainWindow::on_Motion_clicked()
{
    jishu = 0 ;

    mahrTimer77 = new QTimer(this);
    mahrTimer77->start(100);
    connect(mahrTimer77,&QTimer::timeout,this,&MainWindow::mahrSensor772);
    QString uu="dongtai is begin!";
    ui->plainTextEdit->appendPlainText(uu);
}

void MainWindow::mahrSensor772()
{
    // 参数校验，无效时提前返回。
    if(abs((plcSnapshot.m_new_angle)-jishu)<1)
    {
        jingtai[jishu]=g_Global->g_measureResult[g_Global->singleChannelIndex];
        jiaodu[jishu]=plcSnapshot.m_new_angle;
        jishu++;
        if(jishu > jingtai.size()-2)
        {
            mahrTimer77->stop();
            //QString savePath = "E:/Code/QT/auto_measurement_cmake_521-77/data/jingtai_data.txt";
            QString timeStr = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
            QString basePath = ui->cali_data_root->text().trimmed();

            // 参数校验，无效时提前返回。
            if (basePath.isEmpty()) {
                qWarning() << "fail:cali_data_root is black";
                return; // 终止后续操作，防止崩溃
            }

            // 参数校验，无效时提前返回。
            // 参数校验，无效时提前返回。
            if (!basePath.endsWith(QDir::separator())) {
                basePath += QDir::separator();
            }
            QString savename = QString("dongtai_%1.txt").arg(timeStr);
            QString savePath = basePath + savename;
            //QString savePath = QString("E:/Code/QT/auto_measurement_cmake_521-77/data/dongtai_data_%1.txt")

            saveTwoArraysToFile(jingtai,jiaodu, savePath);
            ui->plainTextEdit->appendPlainText(QString("dongtai save succeed! %1").arg(timeStr));
        }
    }

}

void MainWindow::saveTwoArraysToFile(
    const QVector<double>& jingtai,
    const QVector<double>& jiaodu,
    const QString& filePath
    ) {
    // 参数校验，无效时提前返回。
    if (jingtai.size() != jiaodu.size()) {
        qDebug() << "fail";
        return;
    }

    QFile file(filePath);

    // 参数校验，无效时提前返回。
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qDebug() << "fail" << file.errorString();
        return;
    }

    QTextStream out(&file);

    for (int i = 0; i < jingtai.size(); ++i) {
        out << jingtai[i] << " " << jiaodu[i] << "\n";
    }

    file.close();
    qDebug() << "write yes" << filePath;
}


/*************************************************************************************
  * ************************************************************************************/

/***********************************************************************************************************************************************************************************************
 * ********************************************************************************************************************************************************************************************/
void MainWindow::on_SingleCenterMeasure_clicked()
{

    jishu = 0 ;
    mahrTimer77 = new QTimer(this);
    mahrTimer77->start(30);
    connect(mahrTimer77,&QTimer::timeout,this,&MainWindow::mahrSensor773);
    QString uu="dancipianxincaijikais";
    ui->plainTextEdit->appendPlainText(uu);
    ui->progressBar->setValue(0);

}





void MainWindow::mahrSensor773()
{
    ui->progressBar->setValue(jishu*100/1022);
    if(abs((plcSnapshot.m_new_angle)-jishu*0.35)<0.5)
    {
        if(g_Global->g_measureResult[g_Global->singleChannelIndex]<2.0)
        {
        cgq2[jishu]=g_Global->g_measureResult[g_Global->singleChannelIndex];
        jiaodu2[jishu]=plcSnapshot.m_new_angle;

        jishu++;
        if(jishu > cgq2.size()-1)
        {
            mahrTimer77->stop();
            //QString savePath = "E:/Code/QT/auto_measurement_cmake_pxqingxie/data/pianxin.txt";
            QString timeStr = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");

            //QString savePath = QString("E:/Code/QT/auto_measurement_cmake_pxqingxie/data/pianxin_%1.txt")

            //saveTwoArraysToFile(cgq2,jiaodu2, savePath);
            //ui->plainTextEdit->appendPlainText(QString("dongtaicaijijieshu%1").arg(timeStr));
            QVector<double> center = fitCircleCenter(cgq2, jiaodu2);
            ui->pianxinX->display(QString::number(center[0], 'f', 4));
            ui->pianxinY->display(QString::number(center[1], 'f', 4));
            //QThread::msleep(500);
            // mahrTimer->stop();
            QTimer::singleShot(500, this, [=]() {
                emit emit_move_motor(center[0],center[1],52,24,56,25); // 缁惧灝婀€5鐟欙箑褰傞崥顖氬З
            });
            // mahrTimer->start();
            // QTimer::singleShot(500, this, [=]() {
            // });

            // QThread::msleep(2000);

            // (double center,double reg_adress,double xian_adress)
            // QTimer::singleShot(500, this, [=]() {});
            // plcController->HoldingRegisters(center[0], 52);
            // QTimer::singleShot(500, this, [=]() {
            // });



            // plcController->HoldingRegisters(center[1], 56);
            // QTimer::singleShot(500, this, [=]() {
            // });

            //cgq2.clear();
            //jiaodu2.clear();

            ui->plainTextEdit->appendPlainText(QString("pianxin is succeed!%1").arg(timeStr));
             //std::cout<<center[0]<<std::endl;
             //std::cout<<center[1]<<std::endl;


            QLabel *label = findChild<QLabel*>("label_56");
            if (!label) return; // 防止Label不存在

            // 参数校验，无效时提前返回。
            if (isYellow) {
                label->setStyleSheet("background-color: green;"); // 鍒囨崲涓虹豢鑹?
            } else {
                label->setStyleSheet("background-color: yellow;"); // 鍒囨崲涓洪粍鑹?
            }
            isYellow = !isYellow; // 翻转标记

        }
        }
    }

}



QVector<double> MainWindow::fitCircleCenter(const QVector<double>& l, const QVector<double>& a)
{
    /*// 1. 涓ユ牸鐨勮緭鍏ュ悎娉曟€ф牎楠?
    if (l.isEmpty() || a.isEmpty() || l.size() != a.size()) {
        qWarning() << "杈撳叆鏁扮粍涓虹┖鎴栭暱搴︿笉鍖归厤锛?;
        return {0.0, 0.0};
    }
    const int pointCount = l.size();
    if (pointCount < 3) {
        qWarning() << "鑷冲皯闇€瑕?涓偣鎵嶈兘鎷熷悎鍦嗗績锛?;
        return {0.0, 0.0};
    }

    for (double len : l) {
        if (len < 0) {
            qWarning() << "闀垮害鍊间笉鑳戒负璐熸暟锛? << len;
            return {0.0, 0.0};
        }
    }
*/
    const int pointCount = l.size();
    QVector<double> xList, yList;
    for (int i = 0; i < pointCount; ++i) {
        double angleDeg = fmod(270.0 - a[i], 360.0);
        if (angleDeg < 0) angleDeg += 360.0;

        double angleRad = qDegreesToRadians(angleDeg);
        double radius = l[i] + 20; // 避免重复计算
        double x = -radius * cos(angleRad);
        double y = -radius * sin(angleRad);

        xList.append(x);
        yList.append(y);
    }

    double n = pointCount;
    double sumX = 0.0, sumY = 0.0;
    double sumX2 = 0.0, sumY2 = 0.0, sumXY = 0.0;
    double sumX3y2 = 0.0, sumX2y3 = 0.0, sumX2y2 = 0.0;

    for (int i = 0; i < pointCount; ++i) {
        double x = xList[i];
        double y = yList[i];
        double x2 = x * x;
        double y2 = y * y;
        double x2y2 = x2 + y2; // x+y

        sumX += x;
        sumY += y;
        sumX2 += x2;
        sumY2 += y2;
        sumXY += x * y;
        sumX3y2 += x * x2y2;  // x*(x+y) = x + xy
        sumX2y3 += y * x2y2;  // y*(x+y) = xy + y
        sumX2y2 += x2y2;      // sum(x+y)
    }

    // M * [A,B,C]^T = V
    double M11 = sumX2;
    double M12 = sumXY;
    double M13 = sumX;
    double M21 = sumXY;
    double M22 = sumY2;
    double M23 = sumY;
    double M31 = sumX;
    double M32 = sumY;
    double M33 = n;

    double V1 = sumX3y2;
    double V2 = sumX2y3;
    double V3 = sumX2y2;

    double detM = M11*(M22*M33 - M23*M32)
                  - M12*(M21*M33 - M23*M31)
                  + M13*(M21*M32 - M22*M31);

    const double EPS = 1e-8;
    if (fabs(detM) < EPS) {
        qWarning() << "鐐归泦鍏辩嚎鎴栨暟鍊煎寮傦紝鏃犳硶鎷熷悎鍦嗗績锛佽鍒楀紡鍊硷細" << detM;
        return {0.0, 0.0};
    }

    double detA = V1*(M22*M33 - M23*M32)
                  - M12*(V2*M33 - M23*V3)
                  + M13*(V2*M32 - M22*V3);

    double detB = M11*(V2*M33 - M23*V3)
                  - V1*(M21*M33 - M23*M31)
                  + M13*(M21*V3 - V2*M31);

    double A = detA / detM;
    double B = detB / detM;

    double x0 = A / 2.0;
    double y0 = B / 2.0;

    // double C = (V3 - M31*A - M32*B) / M33;
    // double r = sqrt(x0*x0 + y0*y0 - C);

    return {x0, y0};
}









/*******************************************************************************************************************************************************************************
 * **********************************************************************************************************************************************************************************/


void MainWindow::on_ReadDistance_clicked()
{
    QString filePath = QFileDialog::getOpenFileName(
        this,
        "閫夋嫨TXT鏂囦欢",
        QDir::homePath(), // 默认路径：用户主目录
        "鏂囨湰鏂囦欢 (*.txt);;鎵€鏈夋枃浠?(*.*)"
        );

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) { // 鎵撳紑澶辫触
        QMessageBox::critical(this, "閿欒", "文件打开失败:" + file.errorString());
        return;
    }

    dataVector.clear();

    QTextStream in(&file);
    int lineNum = 0; // 琛屽彿锛岀敤浜庡畾浣嶉敊璇?
    while (!in.atEnd()) {
        lineNum++;
        QString line = in.readLine().trimmed(); // 璇诲彇涓€琛屽苟鍘婚櫎棣栧熬绌烘牸
        if (line.isEmpty()) continue; // 跳过空行

        QStringList numStrList = line.split(QRegExp("[,\\s\\t]+"), Qt::SkipEmptyParts);
        for (const QString &numStr : numStrList) {
            bool ok;
            double num = numStr.toDouble(&ok); // 转换为double
            if (ok) {
                dataVector.append(num); // 转换成功，加入vector
            } else {
            }
        }
    }

    file.close();
    QString timeStr = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    ui->plainTextEdit->appendPlainText(QString("wenjianduquchenggong%1").arg(timeStr));
}

void MainWindow::on_Readangle_clicked()
{
    QString filePath = QFileDialog::getOpenFileName(
        this,
        "閫夋嫨TXT鏂囦欢",
        QDir::homePath(), // 默认路径：用户主目录
        "鏂囨湰鏂囦欢 (*.txt);;鎵€鏈夋枃浠?(*.*)"
        );

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) { // 鎵撳紑澶辫触
        QMessageBox::critical(this, "閿欒", "文件打开失败:" + file.errorString());
        return;
    }

    dataVector2.clear();

    QTextStream in(&file);
    int lineNum = 0; // 琛屽彿锛岀敤浜庡畾浣嶉敊璇?
    while (!in.atEnd()) {
        lineNum++;
        QString line = in.readLine().trimmed(); // 璇诲彇涓€琛屽苟鍘婚櫎棣栧熬绌烘牸
        if (line.isEmpty()) continue; // 跳过空行

        QStringList numStrList = line.split(QRegExp("[,\\s\\t]+"), Qt::SkipEmptyParts);
        for (const QString &numStr : numStrList) {
            bool ok;
            double num = numStr.toDouble(&ok); // 转换为double
            if (ok) {
                dataVector2.append(num); // 转换成功，加入vector
            } else {
            }
        }
    }

    file.close();
    QString timeStr = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    ui->plainTextEdit->appendPlainText(QString("file read succeed!%1").arg(timeStr));
    QVector<double> center = fitCircleCenter(dataVector, dataVector2);
    ui->pianxinX->display(QString::number(center[0], 'f', 4));
    ui->pianxinY->display(QString::number(center[1], 'f', 4));
    std::cout<<center[0]<<std::endl;
    std::cout<<center[1]<<std::endl;
    ui->plainTextEdit->appendPlainText(QString("yunaxin is succeed!%1").arg(timeStr));
}

/*******************************************************************************************************************************************************************************
 * **********************************************************************************************************************************************************************************/
void MainWindow::on_sensorE_move_clicked()
{
    QString target =  ui->txtSensorE_target->text();
    double Target = target.toDouble() ;
    sendRegistersAndVar(Target, addressxin_reg_1, addressxin_cyl_1 , 500) ;
}

void MainWindow::on_sensorF_move_clicked()
{
    QString target =  ui->txtSensorF_target->text();
    double Target = target.toDouble() ;
    sendRegistersAndVar(Target, addressxin_reg_2, addressxin_cyl_2 , 500) ;
}

void MainWindow::on_sensorA_move_clicked()
{
  QString target =  ui->txtSensorA_target->text();
  double Target = target.toDouble() ;
  sendRegistersAndVar(Target, addressqing_reg_1, addressqing_cyl_1 , 500) ;

  // QTimer::singleShot(400, this, [=]() {
  //     sendRegistersAndVar(0.0-Target, addressqing_reg_3, addressqing_cyl_3 , 500) ;
  // });
}

void MainWindow::on_sensorB_move_clicked()
{
    QString target =  ui->txtSensorB_target->text();
    double Target = target.toDouble() ;
    sendRegistersAndVar(Target,addressqing_reg_2, addressqing_cyl_2 , 500) ;

    // QTimer::singleShot(400, this, [=]() {
    //     sendRegistersAndVar(0.0-Target, addressqing_reg_4, addressqing_cyl_4 , 500) ;
    // });
}

void MainWindow::on_sensorC_move_clicked()
{
    QString target =  ui->txtSensorC_target->text();
    double Target = target.toDouble() ;
    sendRegistersAndVar(Target, addressqing_reg_3, addressqing_cyl_3 , 500) ;
    // QTimer::singleShot(400, this, [=]() {
    //     sendRegistersAndVar(Target, addressqing_reg_3, addressqing_cyl_3 , 500) ;
    // });
}

void MainWindow::on_sensorD_move_clicked()
{
    QString target =  ui->txtSensorD_target->text();
    double Target = target.toDouble() ;
    sendRegistersAndVar(Target, addressqing_reg_4, addressqing_cyl_4 , 500) ;
}

void MainWindow::on_smallRotate_angle_move_clicked()
{
     QString target =  ui->txtsmallRotate_angle_target->text();
    double Target = target.toDouble() ;
    sendRegistersAndVar(Target, 60, 19 , 500) ;
}


/*******************************************************************************************************************************************************************************
 * **********************************************************************************************************************************************************************************/
void MainWindow::on_RecordMeasure_clicked()
{
    jishu = 0 ;

    mahrTimer77 = new QTimer(this);
    mahrTimer77->start(30);
    connect(mahrTimer77,&QTimer::timeout,this,&MainWindow::mahrSensor774);
    QString uu="danciqingxie is begin!";
    ui->plainTextEdit->appendPlainText(uu);
    ui->progressBar_2->setValue(0);
}


void MainWindow::mahrSensor774()
{
    ui->progressBar_2->setValue(jishu*100/1022);
    if(abs((plcSnapshot.m_new_angle)-jishu*0.35)<0.5)
    {
        if(g_Global->g_measureResult[g_Global->singleChannelIndex]<3.0)
        {
        cgq3[jishu]=g_Global->g_measureResult[g_Global->singleChannelIndex];
        jiaodu3[jishu]=plcSnapshot.m_new_angle;
        jishu++;
        if(jishu > cgq3.size()-1)
        {
            mahrTimer77->stop();
            //QString savePath = "E:/Code/QT/auto_measurement_cmake_pxqingxie/data/pianxin.txt";
            QString timeStr = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");

            //QString savePath = QString("E:/Code/QT/auto_measurement_cmake_pxqingxie/data/qingxie_%1.txt")

            //saveTwoArraysToFile(cgq3,jiaodu3, savePath);
            //ui->plainTextEdit->appendPlainText(QString("dongtaicaijijieshu%1").arg(timeStr));
            QString target =  ui->txtSensorD_target_2->text();
            banjing = target.toDouble() ;
            QVector<double> center = fitPlaneNormal(cgq3, jiaodu3,banjing);
            double xweiyi= atan(center[0])*182.528765;// X轴电机正方向一侧
            double yweiyi= atan(center[1])*182.528765;// Y轴电机正方向一侧
            ui->qingxieX->display(QString::number(center[0], 'f', 5));
            ui->qingxieY->display(QString::number(center[1], 'f', 5));
            ui->lcdNumber->display(QString::number(center[0]*180.0, 'f', 4));
            ui->lcdNumber_2->display(QString::number(center[1]*180.0, 'f', 4));
            ui->plainTextEdit->appendPlainText(QString("xianglaing is succeed!%1").arg(timeStr));



            QLabel *label = findChild<QLabel*>("label_71");
            if (!label) return; // 防止Label不存在

            // 参数校验，无效时提前返回。
            if (isYellow1) {
                label->setStyleSheet("background-color: green;"); // 鍒囨崲涓虹豢鑹?
            } else {
                label->setStyleSheet("background-color: yellow;"); // 鍒囨崲涓洪粍鑹?
            }
            isYellow1 = !isYellow1; // 翻转标记

        }
        }
    }

}

QVector<double> MainWindow::fitPlaneNormal(const QVector<double>& l, const QVector<double>& a, double r)
{
    // 将极坐标(l, a)转换为笛卡尔坐标(x, y, z)，通过 SVD 拟合平面法向量
    const int pointCount = l.size();
    if (pointCount < 3) { // 至少3个点才能拟合平面
        qWarning() << "[fitPlaneNormal] 鐐规暟閲忎笉瓒?鏃犳硶鎷熷悎骞抽潰!";
        return {0.0, 0.0};
    }

    QVector<double> xList, yList, zList;
    for (int i = 0; i < pointCount; ++i) {
        double angleRaw = 270.0 - a[i];
        double angleNorm = fmod(angleRaw, 360.0);
        if (angleNorm < 0.0) {
            angleNorm += 360.0;
        }

        double angleRad = qDegreesToRadians(angleNorm);
        double x = r * cos(angleRad);
        double y = r * sin(angleRad);
        double z = 10.0 - l[i]; // 若物理意义错误，需根据实际场景调整

        xList.append(x);
        yList.append(y);
        zList.append(z);
    }

    double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
    for (int i = 0; i < pointCount; ++i) {
        sumX += xList[i];
        sumY += yList[i];
        sumZ += zList[i];
    }
    double xMean = sumX / pointCount;
    double yMean = sumY / pointCount;
    double zMean = sumZ / pointCount;

    double Sxx = 0.0, Syy = 0.0, Szz = 0.0;
    double Sxy = 0.0, Sxz = 0.0, Syz = 0.0;
    for (int i = 0; i < pointCount; ++i) {
        double dx = xList[i] - xMean;
        double dy = yList[i] - yMean;
        double dz = zList[i] - zMean;

        Sxx += dx * dx;
        Syy += dy * dy;
        Szz += dz * dz;
        Sxy += dx * dy;
        Sxz += dx * dz;
        Syz += dy * dz;
    }

    // Sxx*A + Sxy*B = -Sxz*C
    // Sxy*A + Syy*B = -Syz*C
    double denominator = Sxx * Syy - Sxy * Sxy; // 鍒嗘瘝锛屼笉鑳戒负0
    const double EPS = 1e-8;
    if (fabs(denominator) < EPS) {
        qWarning() << "[fitPlaneNormal] 鍗忔柟宸煩闃佃鍒楀紡鎺ヨ繎0锛岀偣闆嗗叡绾匡紒";
        return {0.0, 0.0};
    }

    double A = (Sxy * Syz - Syy * Sxz) / denominator;
    double B = (Sxy * Sxz - Sxx * Syz) / denominator;

    double nx = A;
    double ny = B;

    return {nx, ny};
}




/*******************************************************************************************************************************************************************************
 * **********************************************************************************************************************************************************************************/
void MainWindow::on_ReadHeight_clicked()
{
    QString filePath = QFileDialog::getOpenFileName(
        this,
        "閫夋嫨TXT鏂囦欢",
        QDir::homePath(), // 默认路径：用户主目录
        "鏂囨湰鏂囦欢 (*.txt);;鎵€鏈夋枃浠?(*.*)"
        );

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) { // 鎵撳紑澶辫触
        QMessageBox::critical(this, "閿欒", "文件打开失败:" + file.errorString());
        return;
    }

    dataVector.clear();

    QTextStream in(&file);
    int lineNum = 0; // 琛屽彿锛岀敤浜庡畾浣嶉敊璇?

    while (!in.atEnd()) {
        lineNum++;

        QString line = in.readLine().trimmed(); // 璇诲彇涓€琛屽苟鍘婚櫎棣栧熬绌烘牸
        if (line.isEmpty()) {
            continue; // 跳过空行
        }

        QStringList numStrList = line.split(QRegExp("[,\\s\\t]+"), Qt::SkipEmptyParts);

        for (const QString &numStr : numStrList) {
            bool ok;
            double num = numStr.toDouble(&ok); // 转换为double

            if (ok) {
                dataVector.append(num); // 转换成功，加入数据缓存
            } else {
                // ui->plainTextEdit_log->appendPlainText(
                // );
            }
        }
    }

    file.close();

    QString timeStr = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    ui->plainTextEdit->appendPlainText(QString("鏂囦欢璇诲彇鎴愬姛锛?1").arg(timeStr));
}


void MainWindow::on_ReadAngle_clicked()
{
    QString filePath = QFileDialog::getOpenFileName(
        this,
        "閫夋嫨TXT鏂囦欢",
        QDir::homePath(), // 默认路径：用户主目录
        "鏂囨湰鏂囦欢 (*.txt);;鎵€鏈夋枃浠?(*.*)"
        );

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) { // 鎵撳紑澶辫触
        QMessageBox::critical(this, "閿欒", "文件打开失败:" + file.errorString());
        return;
    }

    dataVector2.clear();

    QTextStream in(&file);
    int lineNum = 0; // 琛屽彿锛岀敤浜庡畾浣嶉敊璇?

    while (!in.atEnd()) {
        lineNum++;

        QString line = in.readLine().trimmed(); // 璇诲彇涓€琛屽苟鍘婚櫎棣栧熬绌烘牸
        if (line.isEmpty()) {
            continue; // 跳过空行
        }

        QStringList numStrList = line.split(QRegExp("[,\\s\\t]+"), Qt::SkipEmptyParts);

        for (const QString &numStr : numStrList) {
            bool ok;
            double num = numStr.toDouble(&ok); // 转换为double

            if (ok) {
                dataVector2.append(num); // 转换成功，加入数据缓存
            } else {
                // ui->plainTextEdit_log->appendPlainText(
                // );
            }
        }
    }

    file.close();

    QString timeStr = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    ui->plainTextEdit->appendPlainText(QString("鏂囦欢璇诲彇鎴愬姛锛?1").arg(timeStr));

    QString target = ui->txtSensorD_target_2->text();
    banjing = target.toDouble();

    QVector<double> center = fitPlaneNormal(dataVector, dataVector2, banjing);

    double xweiyi = atan(center[0]) * 182.528765; // X轴电机正方向一侧的位移
    double yweiyi = atan(center[1]) * 182.528765; // Y轴电机正方向一侧的位移

    ui->qingxieX->display(QString::number(center[0], 'f', 5));
    ui->qingxieY->display(QString::number(center[1], 'f', 5));

    ui->lcdNumber->display(QString::number(center[0] * 180, 'f', 4));
    ui->lcdNumber_2->display(QString::number(center[1] * 180, 'f', 4));

    ui->plainTextEdit->appendPlainText(QString("娉曞悜閲忚绠楀畬鎴愶細%1").arg(timeStr));
}


/*******************************************************************************************************************************************************************************
 * **********************************************************************************************************************************************************************************/
void MainWindow::on_RecordCenter_clicked()
{
    QString timeStr = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");

    QString basePath = ui->cali_data_root->text().trimmed();

    // 参数校验，无效时提前返回。
    if (basePath.isEmpty()) {
        qWarning() << "閿欒:cali_data_root 涓虹┖!";
        return; // 终止后续操作，防止崩溃
    }

    // 参数校验，无效时提前返回。
    // 参数校验，无效时提前返回。
    if (!basePath.endsWith(QDir::separator())) {
        basePath += QDir::separator();
    }

    QString savename = QString("pianxin_%1.txt").arg(timeStr);
    QString savePath = basePath + savename;

    saveTwoArraysToFile(cgq2, jiaodu2, savePath);

    ui->plainTextEdit->appendPlainText(QString("偏心数据保存成功，?1").arg(timeStr));
}


void MainWindow::on_RecordTilt_clicked()
{
    QString timeStr = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");

    QString basePath = ui->cali_data_root->text().trimmed();

    // 参数校验，无效时提前返回。
    if (basePath.isEmpty()) {
        qWarning() << "閿欒:cali_data_root 涓虹┖!";
        return; // 终止后续操作，防止崩溃
    }

    // 参数校验，无效时提前返回。
    // 参数校验，无效时提前返回。
    if (!basePath.endsWith(QDir::separator())) {
        basePath += QDir::separator();
    }

    QString savename = QString("qingxie_%1.txt").arg(timeStr);
    QString savePath = basePath + savename;

    saveTwoArraysToFile(cgq3, jiaodu3, savePath);

    ui->plainTextEdit->appendPlainText(QString("倾斜数据保存成功，?1").arg(timeStr));
}


void MainWindow::on_emergency_stop_clicked()
{
    postPlcMoveToAbsolutePosition(1, 18);
}


/***************************************************************************************************************
 * *********************************************************************************/
void MainWindow::initChartContainer()
{
    m_chartView = new QChartView();
    m_chartView->setRenderHint(QPainter::Antialiasing); // 抗锯齿，使曲线更平滑

    QGridLayout *layout = qobject_cast<QGridLayout*>(ui->chartWidget->layout());
    if (!layout) {
        layout = new QGridLayout(ui->chartWidget);
        layout->setContentsMargins(0, 0, 0, 0);
    }

    layout->addWidget(m_chartView);
}


void MainWindow::Draw()
{
    QVector<double> cgqcd;
    QVector<double> jiaoducd;

    cgqcd = cgq2.mid(0, jishu);
    jiaoducd = jiaodu2.mid(0, jishu);

    updatePolarChart(cgqcd, jiaoducd);

    // qDebug() << "update";
}



void MainWindow::updatePolarChart(const QVector<double> &cgq33, const QVector<double> &jiaodu33)
{
    QVector<double> cgq22;
    for (double val : cgq33) {
        cgq22.append(val + 0.3); // 每个元素加 0.3
    }

    m_polarChart = new QPolarChart();
    m_polarChart->setTitle("pianxin tuxiang");

    QScatterSeries *scatterSeries = new QScatterSeries();
    scatterSeries->setName("cgq-jiaodu");
    scatterSeries->setMarkerShape(QScatterSeries::MarkerShapeCircle); // 圆形散点
    scatterSeries->setMarkerSize(4); // 散点大小
    scatterSeries->setPen(QPen(Qt::red, 1)); // 散点边框
    scatterSeries->setBrush(Qt::red); // 散点填充色

    for (int i = 0; i < jiaodu33.size(); ++i) {
        double convertedAngle = 270.0 - jiaodu33[i];
        convertedAngle = fmod(convertedAngle + 360.0, 360.0);

        scatterSeries->append(convertedAngle, cgq22[i]);
    }

    m_polarChart->addSeries(scatterSeries);

    QValueAxis *angleAxis = new QValueAxis();
    angleAxis->setTickCount(9);
    angleAxis->setLabelFormat("%d");
    angleAxis->setRange(0, 360);
    m_polarChart->addAxis(angleAxis, QPolarChart::PolarOrientationAngular);

    QValueAxis *radiusAxis = new QValueAxis();
    radiusAxis->setTickCount(6);
    radiusAxis->setLabelFormat("%.3f");
    double minDistance = *std::min_element(cgq22.begin(), cgq22.end());
    double maxDistance = *std::max_element(cgq22.begin(), cgq22.end());
    radiusAxis->setRange(qMax(0.0, minDistance * 0.7), maxDistance * 1.1);
    m_polarChart->addAxis(radiusAxis, QPolarChart::PolarOrientationRadial);

    scatterSeries->attachAxis(angleAxis);
    scatterSeries->attachAxis(radiusAxis);

    m_chartView->setChart(m_polarChart);
    m_chartView->setRenderHint(QPainter::Antialiasing);
}





void MainWindow::mahrSensorpx()
{
    updatePolarChart(cgq2, jiaodu2);
}


 void MainWindow::move_motor(double center,double center2,double reg_adress,double xian_adress,double reg_adress2,double xian_adress2)
 {  std::cout<<center<<std::endl;
     postPlcWriteHoldingRegisters(center, reg_adress);
     QTimer::singleShot(500, this, [=]() {
         postPlcMoveToAbsolutePosition(1, xian_adress); // 缁惧灝婀€5鐟欙箑褰傞崥顖氬З
     });

     QTimer::singleShot(1000, this, [=]() {
         postPlcWriteHoldingRegisters(center2, reg_adress2);
         QTimer::singleShot(500, this, [=]() {
             postPlcMoveToAbsolutePosition(1, xian_adress2); // 缁惧灝婀€5鐟欙箑褰傞崥顖氬З
         });
     });
    mahrTimer->start(100);
 }

void MainWindow::move_motor_z(double bottom_position_z,double reg_adress_z,double xian_adress_z)
 {
    std::cout<<bottom_position_z<<std::endl;
    postPlcWriteHoldingRegisters(bottom_position_z, reg_adress_z);
    QTimer::singleShot(500, this, [=]() {
        postPlcMoveToAbsolutePosition(1, xian_adress_z); // 缁惧灝婀€5鐟欙箑褰傞崥顖氬З
    });

    mahrTimer->start(100);
 }

 void MainWindow::move_motor_t(double tiltX, double tiltY, double eccX, double eccY,
                               double reg_adress_t_x, double xian_adress_t_x,
                               double reg_adress_t_y, double xian_adress_t_y,
                               double reg_adress_c_x, double xian_adress_c_x,
                               double reg_adress_c_y, double xian_adress_c_y)
 {
     double targetTiltX = plcSnapshot.m_tiaoqing1 + (tiltX * 120.0);
     //double targetTiltX = 0.5;
     std::cout << "Tilt-X target: " << targetTiltX << std::endl;
     postPlcWriteHoldingRegisters(targetTiltX, reg_adress_t_x);
     QTimer::singleShot(500, this, [=]() {
         postPlcMoveToAbsolutePosition(1, xian_adress_t_x);
     });

     QTimer::singleShot(1000, this, [=]() {
        double targetTiltY = plcSnapshot.m_tiaoqing2 + (tiltY * 80.0);
        //double targetTiltY = 0.5;
         std::cout << "Tilt-Y target: " << targetTiltY << std::endl;
         postPlcWriteHoldingRegisters(targetTiltY, reg_adress_t_y);
         QTimer::singleShot(500, this, [=]() {
             postPlcMoveToAbsolutePosition(1, xian_adress_t_y);
         });
    });

     mahrTimer->start(100);
 }

void MainWindow::stop_timer()
 {
    mahrTimer->stop();
 }

 void MainWindow::start_timer()
 {
     mahrTimer->start(100);
 }






 void MainWindow::on_btnShowJogPanel_clicked()
 {
     if (!jogPanel) {
         return;
     }

     if (jogPanel->isVisible()) {
         jogPanel->hide();
     } else {
         jogPanel->show();
         jogPanel->raise();
         jogPanel->activateWindow();
         this->activateWindow();
         this->setFocus();
     }
 }


 void MainWindow::setJogButtonDown(JogAxis axis, JogDir dir, bool down)
 {
     if (!jogPanel)
         return;

     if (axis == JogAxis::X && dir == JogDir::Positive) {
         jogPanel->setXPositiveDown(down);
     } else if (axis == JogAxis::X && dir == JogDir::Negative) {
         jogPanel->setXNegativeDown(down);
     } else if (axis == JogAxis::Y && dir == JogDir::Positive) {
         jogPanel->setYPositiveDown(down);
     } else if (axis == JogAxis::Y && dir == JogDir::Negative) {
         jogPanel->setYNegativeDown(down);
     } else if (axis == JogAxis::Z && dir == JogDir::Positive) {
         jogPanel->setZPositiveDown(down);
     } else if (axis == JogAxis::Z && dir == JogDir::Negative) {
         jogPanel->setZNegativeDown(down);
     }
 }


 void MainWindow::emergencyStopJog()
 {
     postPlcMoveToAbsolutePosition(1, 18);

     QTimer::singleShot(ESTOP_PULSE_MS, this, [=]() {
         postPlcMoveToAbsolutePosition(0, 18);
     });
 }


 void MainWindow::clearAllJogKeyStates()
 {
     aKeyPressed = false;
     dKeyPressed = false;
     leftKeyPressed = false;
     rightKeyPressed = false;
     upKeyPressed = false;
     downKeyPressed = false;

     setJogButtonDown(JogAxis::X, JogDir::Positive, false);
     setJogButtonDown(JogAxis::X, JogDir::Negative, false);
     setJogButtonDown(JogAxis::Y, JogDir::Positive, false);
     setJogButtonDown(JogAxis::Y, JogDir::Negative, false);
     setJogButtonDown(JogAxis::Z, JogDir::Positive, false);
     setJogButtonDown(JogAxis::Z, JogDir::Negative, false);
 }


 void MainWindow::requestStartJog(JogAxis axis, JogDir dir)
 {
     // 参数校验，无效时提前返回。
     if (jogActive && currentJogAxis == axis && currentJogDir == dir)
         return;

     pendingJog.valid = true;
     pendingJog.axis = axis;
     pendingJog.dir = dir;

     // 参数校验，无效时提前返回。
     if (jogBusy)
         return;

     // 参数校验，无效时提前返回。
     if (!jogActive) {
         PendingJogCommand cmd = pendingJog;
         pendingJog.valid = false;
         doStartJog(cmd.axis, cmd.dir);
         return;
     }

     doStopJog();
 }


 void MainWindow::doStartJog(JogAxis axis, JogDir dir)
 {
     jogBusy = true;
     jogStopping = false;

     ++jogCommandId;
     ++speedRestoreId;

     currentJogAxis = axis;
     currentJogDir = dir;
     jogActive = true;

     // mahrTimer->stop();

     double jogVelocity = ui->keyPosition_Velocity->text().toDouble();
     postPlcWriteHoldingRegisters(jogVelocity, 28);

     // double target = (dir == JogDir::Positive) ? 1000.0 : -1000.0;

     // int addr1 = 0;
     int addr2 = 0;

     switch (axis) {
     case JogAxis::X:
         // addr1 = address_x1;
         addr2 = (dir == JogDir::Positive) ? 12 : 17;
         break;
     case JogAxis::Y:
         // addr1 = address_y1;
         addr2 = (dir == JogDir::Positive) ? 8 : 13;
         break;
     case JogAxis::Z:
         // addr1 = address_z1;
         addr2 = (dir == JogDir::Positive) ? 10 : 15;
         break;
         // 后续需要看情况补充调心调倾部分控制
     // case JogAxis::AdjustA:
     //     // addr1 = addressxin_reg_1;
     //     addr2 = addressxin_cyl_1;
     //     break;
     // case JogAxis::AdjustB:
     //     // addr1 = addressxin_reg_2;
     //     addr2 = addressxin_cyl_2;
     //     break;
     default:
         jogBusy = false;
         return;
     }


     int currentId = jogCommandId;

     QTimer::singleShot(START_DELAY_MS, this, [=]() {
         if (currentId != jogCommandId)
             return;

         if (!jogActive)
             return;

         if (currentJogAxis != axis || currentJogDir != dir)
             return;

         // sendRegistersAndVar(target, addr1, addr2, 500);
         postPlcMoveToAbsolutePosition(1, addr2); // 线圈触发启动
         jogBusy = false;
         // mahrTimer->start(100);

         // 参数校验，无效时提前返回。
         if (pendingJog.valid &&
             (pendingJog.axis != currentJogAxis || pendingJog.dir != currentJogDir)) {
             doStopJog();
         }
     });
 }


 void MainWindow::doStopJog()
 {
     if (!jogActive && !jogBusy)
         return;

     jogBusy = true;
     jogStopping = true;
     jogActive = false;

     ++jogCommandId;

     // mahrTimer->stop();

     // 根据当前轴和方向计算停止线圈地址（与 doStartJog 一致）
    int addr2 = 0;
    switch (currentJogAxis) {
    case JogAxis::X:
        addr2 = (currentJogDir == JogDir::Positive) ? 12 : 17;
        break;
    case JogAxis::Y:
        addr2 = (currentJogDir == JogDir::Positive) ? 8 : 13;
        break;
    case JogAxis::Z:
        addr2 = (currentJogDir == JogDir::Positive) ? 10 : 15;
        break;
    default:
        break;
    }

    if (addr2 != 0)
        postPlcMoveToAbsolutePosition(0, addr2);

     int restoreId = ++speedRestoreId;
     QTimer::singleShot(RESTORE_SPEED_DELAY_MS, this, [=]() {
         if (restoreId == speedRestoreId && !jogActive) {
             double velocityNormal = ui->txtPosition_Velocity->text().toDouble();
             postPlcWriteHoldingRegisters(velocityNormal, 28);
         }
     });

     // mahrTimer->start(100);

     QTimer::singleShot(STOP_SETTLE_MS, this, [=]() {
         finishStopAndStartPending();

     });
 }


 // 停止稳定后的收尾：清除停止状态，若停止期间收到换向请求（pendingJog）则立即启动新方向。
 void MainWindow::finishStopAndStartPending()
 {
     currentJogAxis = JogAxis::None;
     jogStopping = false;
     jogBusy = false;

     if (pendingJog.valid) {
         PendingJogCommand cmd = pendingJog;
         pendingJog.valid = false;
         doStartJog(cmd.axis, cmd.dir);
     }
 }


 void MainWindow::X_positive_pressed()
 {
     requestStartJog(JogAxis::X, JogDir::Positive);
 }

 void MainWindow::X_positive_released()
 {
     if (currentJogAxis == JogAxis::X && currentJogDir == JogDir::Positive) {
         pendingJog.valid = false;
         doStopJog();
     }
 }

 void MainWindow::X_negative_pressed()
 {
     requestStartJog(JogAxis::X, JogDir::Negative);
 }

 void MainWindow::X_negative_released()
 {
     if (currentJogAxis == JogAxis::X && currentJogDir == JogDir::Negative) {
         pendingJog.valid = false;
         doStopJog();
     }
 }

 void MainWindow::Y_positive_pressed()
 {
     requestStartJog(JogAxis::Y, JogDir::Positive);
 }

 void MainWindow::Y_positive_released()
 {
     if (currentJogAxis == JogAxis::Y && currentJogDir == JogDir::Positive) {
         pendingJog.valid = false;
         doStopJog();
     }
 }

 void MainWindow::Y_negative_pressed()
 {
     requestStartJog(JogAxis::Y, JogDir::Negative);
 }

 void MainWindow::Y_negative_released()
 {
     if (currentJogAxis == JogAxis::Y && currentJogDir == JogDir::Negative) {
         pendingJog.valid = false;
         doStopJog();
     }
 }

 void MainWindow::Z_positive_pressed()
 {
     requestStartJog(JogAxis::Z, JogDir::Positive);
 }

 void MainWindow::Z_positive_released()
 {
     if (currentJogAxis == JogAxis::Z && currentJogDir == JogDir::Positive) {
         pendingJog.valid = false;
         doStopJog();
     }
 }

 void MainWindow::Z_negative_pressed()
 {
     requestStartJog(JogAxis::Z, JogDir::Negative);
 }

 void MainWindow::Z_negative_released()
 {
     if (currentJogAxis == JogAxis::Z && currentJogDir == JogDir::Negative) {
         pendingJog.valid = false;
         doStopJog();
     }
 }


 void MainWindow::adjustA_positive_pressed()
 {
     requestStartJog(JogAxis::AdjustA, JogDir::Positive);
 }

 void MainWindow::adjustA_positive_released()
 {
     if (currentJogAxis == JogAxis::AdjustA &&
         currentJogDir == JogDir::Positive) {
         pendingJog.valid = false;
         doStopJog();
     }
 }

 void MainWindow::adjustA_negative_pressed()
 {
     requestStartJog(JogAxis::AdjustA, JogDir::Negative);
 }

 void MainWindow::adjustA_negative_released()
 {
     if (currentJogAxis == JogAxis::AdjustA &&
         currentJogDir == JogDir::Negative) {
         pendingJog.valid = false;
         doStopJog();
     }
 }

 void MainWindow::adjustB_positive_pressed()
 {
     requestStartJog(JogAxis::AdjustB, JogDir::Positive);
 }

 void MainWindow::adjustB_positive_released()
 {
     if (currentJogAxis == JogAxis::AdjustB &&
         currentJogDir == JogDir::Positive) {
         pendingJog.valid = false;
         doStopJog();
     }
 }

 void MainWindow::adjustB_negative_pressed()
 {
     requestStartJog(JogAxis::AdjustB, JogDir::Negative);
 }

 void MainWindow::adjustB_negative_released()
 {
     if (currentJogAxis == JogAxis::AdjustB &&
         currentJogDir == JogDir::Negative) {
         pendingJog.valid = false;
         doStopJog();
     }
 }





 void MainWindow::on_RT_Matrix_clicked()
 {
     if(!m_hasCoarseRegResult)
     {
         QMessageBox::information(this, tr("RT Matrix"),
             tr("No coarse registration result available. Please run coarse registration first."));
         return;
     }

     if(!ui || !ui->DicGLWidget || !pointCloudListModel)
         return;

     // Collect selected data: first from listWidget, then from DicGLWidget box selection
     struct TransformTarget {
         enum Type { PointCloud, Mesh, Line };
         Type type;
         int index;
         QString displayName;
     };
     std::vector<TransformTarget> targets;

     // 1. Check listWidgetPointCloudFiles selection
     if(ui->listWidgetPointCloudFiles && ui->listWidgetPointCloudFiles->selectionModel())
     {
         const QModelIndexList selectedRows = ui->listWidgetPointCloudFiles->selectionModel()->selectedRows();
         for(const QModelIndex& idx : selectedRows)
         {
             if(const QStandardItem* item = pointCloudListModel->item(idx.row()))
             {
                 const int entryType = item->data(RoleEntryType).toInt();
                 const QString name = item->text();

                 if(entryType == static_cast<int>(PointCloudEntryType::PointCloud))
                 {
                     const int ci = item->data(RolePointCloudIndex).toInt();
                     if(ci >= 0)
                         targets.push_back({TransformTarget::PointCloud, ci, name});
                 }
                 else if(entryType == static_cast<int>(PointCloudEntryType::FittedGeometry) ||
                         entryType == static_cast<int>(PointCloudEntryType::Reconstruction))
                 {
                     const int mi = item->data(RoleMeshIndex).toInt();
                     if(mi >= 0)
                         targets.push_back({TransformTarget::Mesh, mi, name});
                 }
                 else if(entryType == static_cast<int>(PointCloudEntryType::FittedAxisLine))
                 {
                     const int li = item->data(RoleLineIndex).toInt();
                     if(li >= 0)
                         targets.push_back({TransformTarget::Line, li, name});
                 }
             }
         }
     }

     // 2. If nothing selected in list, check DicGLWidget box selection
     if(targets.empty())
     {
         PCLCloud::Ptr boxSelected = ui->DicGLWidget->selectedPointsCloud();
         if(boxSelected && !boxSelected->empty())
         {
             PCLCloud transformed;
             pcl::transformPointCloud(*boxSelected, transformed, m_coarseRegTransform);
             ui->DicGLWidget->add_pointcloud(transformed);
             const int newIdx = ui->DicGLWidget->pointCloudCount() - 1;
             const QString resultName = QStringLiteral("RT_Transformed_Selection");
             addPointCloudFileListItem(resultName, resultName,
                                       static_cast<int>(PointCloudEntryType::PointCloud),
                                       -1, newIdx, -1);
             ui->DicGLWidget->update();
             if(ui->plainTextEdit)
                 ui->plainTextEdit->appendPlainText(
                     systemTime() + " " + tr("RT matrix applied to box selection. %1 points.")
                         .arg(boxSelected->size()));
             return;
         }
     }

     if(targets.empty())
     {
         QMessageBox::information(this, tr("RT Matrix"),
             tr("Please select data in the point cloud list or box-select points in the viewer first."));
         return;
     }

     // 3. Apply transform to each target
     int transformedCount = 0;
     for(const TransformTarget& t : targets)
     {
         if(t.type == TransformTarget::PointCloud)
         {
             PCLCloud::Ptr src = ui->DicGLWidget->pointCloudByIndex(t.index);
             if(!src || src->empty())
                 continue;
             PCLCloud transformed;
             pcl::transformPointCloud(*src, transformed, m_coarseRegTransform);
             ui->DicGLWidget->add_pointcloud(transformed);
             const QString resultName = t.displayName + QStringLiteral("_RT");
             addPointCloudFileListItem(resultName, resultName,
                                       static_cast<int>(PointCloudEntryType::PointCloud),
                                       -1, -1, -1);
             ++transformedCount;
         }
         else if(t.type == TransformTarget::Mesh)
         {
             const PCLMesh mesh = ui->DicGLWidget->meshByIndex(t.index);
             pcl::PointCloud<pcl::PointXYZ> pts;
             pcl::fromPCLPointCloud2(mesh.cloud, pts);
             if(pts.empty())
                 continue;
             PCLCloud transformed;
             pcl::transformPointCloud(pts, transformed, m_coarseRegTransform);
             PCLMesh newMesh;
             pcl::toPCLPointCloud2(transformed, newMesh.cloud);
             newMesh.polygons = mesh.polygons;
             const int newMeshIdx = ui->DicGLWidget->addMesh(newMesh);
             if(newMeshIdx >= 0)
             {
                 const QString resultName = t.displayName + QStringLiteral("_RT");
                 addPointCloudFileListItem(resultName, resultName,
                                           static_cast<int>(PointCloudEntryType::FittedGeometry),
                                           newMeshIdx, -1, -1);
                 ++transformedCount;
             }
         }
         else if(t.type == TransformTarget::Line)
         {
             PCLCloud::Ptr srcLine = ui->DicGLWidget->lineByIndex(t.index);
             if(!srcLine || srcLine->points.empty())
                 continue;
             PCLCloud transformed;
             pcl::transformPointCloud(*srcLine, transformed, m_coarseRegTransform);
             transformed.width = transformed.points.size();
             transformed.height = 1;
             transformed.is_dense = true;
             const int newLineIdx = ui->DicGLWidget->addLine(
                 std::make_shared<PCLCloud>(transformed));
             if(newLineIdx >= 0)
             {
                 const QString resultName = t.displayName + QStringLiteral("_RT");
                 addPointCloudFileListItem(resultName, resultName,
                                           static_cast<int>(PointCloudEntryType::FittedAxisLine),
                                           -1, -1, newLineIdx);
                 ++transformedCount;
             }
         }
     }

     ui->DicGLWidget->update();
     if(ui->plainTextEdit)
         ui->plainTextEdit->appendPlainText(
             systemTime() + " " + tr("RT matrix applied to %1 item(s).").arg(transformedCount));
 }


 void MainWindow::on_coarseregistration_clicked()
 {

         if(!ui || !ui->DicGLWidget || !pointCloudListModel)
             return;

         // 1. Collect all _FitSphere entries with sphere center data
         struct SphereEntry {
             int meshIndex;
             QString displayName;
             Eigen::Vector3f center;
             float radius;
         };
         std::vector<SphereEntry> sphereEntries;

         for(int row = 0; row < pointCloudListModel->rowCount(); ++row)
         {
             QStandardItem* item = pointCloudListModel->item(row);
             if(!item) continue;

             const int entryType = item->data(RoleEntryType).toInt();
             if(entryType != static_cast<int>(PointCloudEntryType::FittedGeometry))
                 continue;

             const QString name = item->text();
             const int meshIndex = item->data(RoleMeshIndex).toInt();
             if(meshIndex < 0)
                 continue;

             // 优先使用 sphereFitResults 中存储的球心数据
             Eigen::Vector3f center = Eigen::Vector3f::Zero();
             float radius = 0.0f;
             if(sphereFitResults.contains(meshIndex))
             {
                 center = sphereFitResults[meshIndex].center;
                 radius = sphereFitResults[meshIndex].radius;
             }
             else
             {
                 // 回退：使用网格质心
                 const PCLMesh mesh = ui->DicGLWidget->meshByIndex(meshIndex);
                 pcl::PointCloud<pcl::PointXYZ> pts;
                 pcl::fromPCLPointCloud2(mesh.cloud, pts);
                 if(pts.empty()) continue;
                 Eigen::Vector3f sum = Eigen::Vector3f::Zero();
                 for(const auto& p : pts.points)
                     sum += Eigen::Vector3f(p.x, p.y, p.z);
                 center = sum / static_cast<float>(pts.size());
             }
             sphereEntries.push_back({ meshIndex, name, center, radius });
         }

         if(sphereEntries.size() < 3)
         {
             QMessageBox::information(this, tr("Coarse Registration"),
                 tr("Need at least 3 fitted sphere results. Currently found: %1").arg(sphereEntries.size()));
             return;
         }

         // 2. Build dialog
         QDialog dlg(this);
         Ui::CoarseRegistrationDialog dlgUi;
         dlgUi.setupUi(&dlg);

         // Wire up Reorder buttons (defined in .ui file)
         auto makeReorder = [](QPushButton* btn, QListWidget* list, int delta) {
             QObject::connect(btn, &QPushButton::clicked, [list, delta]() {
                 const int r = list->currentRow();
                 if(r < 0) return;
                 const int target = r + delta;
                 if(target < 0 || target >= list->count()) return;
                 QListWidgetItem* item = list->takeItem(r);
                 list->insertItem(target, item);
                 list->setCurrentRow(target);
             });
         };
         makeReorder(dlgUi.btnRefUp,   dlgUi.listReference, -1);
         makeReorder(dlgUi.btnRefDown, dlgUi.listReference, +1);
         makeReorder(dlgUi.btnSrcUp,   dlgUi.listSource,    -1);
         makeReorder(dlgUi.btnSrcDown, dlgUi.listSource,    +1);

         auto populateList = [&sphereEntries](QListWidget* list) {
             list->clear();
             for(size_t i = 0; i < sphereEntries.size(); ++i)
             {
                 const auto& e = sphereEntries[i];
                 const QString text = QString("%1  |  c=(%2,%3,%4)  r=%5")
                     .arg(e.displayName)
                     .arg(e.center.x(), 0, 'f', 3)
                     .arg(e.center.y(), 0, 'f', 3)
                     .arg(e.center.z(), 0, 'f', 3)
                     .arg(e.radius, 0, 'f', 3);
                 QListWidgetItem* litem = new QListWidgetItem(text);
                 litem->setData(Qt::UserRole, static_cast<int>(i)); // store index into sphereEntries
                 list->addItem(litem);
             }
         };

         // Initially put all entries in Reference list
         populateList(dlgUi.listReference);

         auto updateInfo = [&dlgUi]() {
             dlgUi.labelInfo->setText(QString("Ref: %1 items  |  Src: %2 items  (need equal count >= 3)")
                 .arg(dlgUi.listReference->count())
                 .arg(dlgUi.listSource->count()));
         };
         updateInfo();

         // Move selected items to Source
         QObject::connect(dlgUi.btnMoveToSrc, &QPushButton::clicked, [&]() {
             QList<QListWidgetItem*> selected = dlgUi.listReference->selectedItems();
             for(QListWidgetItem* sel : selected)
             {
                 QListWidgetItem* moved = dlgUi.listReference->takeItem(dlgUi.listReference->row(sel));
                 dlgUi.listSource->addItem(moved);
             }
             updateInfo();
         });

         // Move selected items to Reference
         QObject::connect(dlgUi.btnMoveToRef, &QPushButton::clicked, [&]() {
             QList<QListWidgetItem*> selected = dlgUi.listSource->selectedItems();
             for(QListWidgetItem* sel : selected)
             {
                 QListWidgetItem* moved = dlgUi.listSource->takeItem(dlgUi.listSource->row(sel));
                 dlgUi.listReference->addItem(moved);
             }
             updateInfo();
         });

         QObject::connect(dlgUi.buttonBox, &QDialogButtonBox::accepted, [&]() {
             const int nRef = dlgUi.listReference->count();
             const int nSrc = dlgUi.listSource->count();
             if(nRef < 3 || nSrc < 3)
             {
                 QMessageBox::warning(&dlg, tr("Coarse Registration"),
                     tr("Each group must have at least 3 items. Ref: %1, Src: %2").arg(nRef).arg(nSrc));
                 return;
             }
             if(nRef != nSrc)
             {
                 QMessageBox::warning(&dlg, tr("Coarse Registration"),
                     tr("Both groups must have the same number of items. Ref: %1, Src: %2").arg(nRef).arg(nSrc));
                 return;
             }
             dlg.accept();
         });
         QObject::connect(dlgUi.buttonBox, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

         if(dlg.exec() != QDialog::Accepted)
             return;

         // 3. Read final lists and compute transform
         std::vector<Eigen::Vector3f> refCenters, srcCenters;
         refCenters.reserve(dlgUi.listReference->count());
         srcCenters.reserve(dlgUi.listSource->count());

         for(int i = 0; i < dlgUi.listReference->count(); ++i)
         {
             const int idx = dlgUi.listReference->item(i)->data(Qt::UserRole).toInt();
             refCenters.push_back(sphereEntries[idx].center);
         }
         for(int i = 0; i < dlgUi.listSource->count(); ++i)
         {
             const int idx = dlgUi.listSource->item(i)->data(Qt::UserRole).toInt();
             srcCenters.push_back(sphereEntries[idx].center);
         }

         const auto regResult = Point_Algorithm::estimateRigidTransformSVD(srcCenters, refCenters);
         if(!regResult.success)
         {
             QMessageBox::warning(this, tr("Coarse Registration"), tr("SVD transform computation failed."));
             return;
         }

         // 4. Log result
         if(ui->plainTextEdit)
         {
             ui->plainTextEdit->appendPlainText(systemTime() + " " + tr("Coarse registration (SVD):"));
             ui->plainTextEdit->appendPlainText(
                 QString("  Transform: [%1 %2 %3 %4; %5 %6 %7 %8; %9 %10 %11 %12; 0 0 0 1]")
                     .arg(regResult.transformation(0,0), 0, 'f', 6)
                     .arg(regResult.transformation(0,1), 0, 'f', 6)
                     .arg(regResult.transformation(0,2), 0, 'f', 6)
                     .arg(regResult.transformation(0,3), 0, 'f', 6)
                     .arg(regResult.transformation(1,0), 0, 'f', 6)
                     .arg(regResult.transformation(1,1), 0, 'f', 6)
                     .arg(regResult.transformation(1,2), 0, 'f', 6)
                     .arg(regResult.transformation(1,3), 0, 'f', 6)
                     .arg(regResult.transformation(2,0), 0, 'f', 6)
                     .arg(regResult.transformation(2,1), 0, 'f', 6)
                     .arg(regResult.transformation(2,2), 0, 'f', 6)
                     .arg(regResult.transformation(2,3), 0, 'f', 6));
             ui->plainTextEdit->appendPlainText(
                 QString("  RMS error: %1 mm").arg(regResult.rmsError, 0, 'f', 6));
         }

         // 5. Store RT matrix and enable RT_Matrix button
         m_coarseRegTransform = regResult.transformation;
         m_hasCoarseRegResult = true;
         if(ui->RT_Matrix)
             ui->RT_Matrix->setEnabled(true);
         if(ui->plainTextEdit)
             ui->plainTextEdit->appendPlainText(
                 systemTime() + " " + tr("Coarse registration completed. RT matrix stored."));
 }

