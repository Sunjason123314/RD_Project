#ifndef DICQGLVIEWER_H
#define DICQGLVIEWER_H
#define OCTREE_RESOLUTION 128.0f
//OPENGL(QT自带，如果使用opengl相关的第三方库，需要注释掉)
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLTexture>
#include <QOpenGLBuffer>
//Qt
#include <QWheelEvent>
#include <QMouseEvent>
#include <QDebug>
#include <QImage>
#include <QColor>
#include <QMenu>
#include <QPolygon>
//QGLViewer——qt第三方opengl显示控件库
#include <QGLViewer/qglviewer.h>
//pcl——点云库
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
//eigen
#include <Eigen/Dense>
//opencv
#include <opencv2/opencv.hpp>
//stl
#include <iostream>
#include <vector>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <iomanip>
#include <omp.h>
#include <chrono>
//assimp库
//#include <assimp/cimport.h>
//#include <assimp/scene.h>
//#include <assimp/postprocess.h>

using namespace std;

typedef pcl::PolygonMesh PCLMesh;//网格
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;//点云
typedef pcl::PointCloud<pcl::PointXYZRGB> PCLRGBCloud;//彩色点云

class DICQGLViewer : public QGLViewer
{
    Q_OBJECT
public:
    explicit DICQGLViewer(QWidget *parent = nullptr);
    ~DICQGLViewer();

    struct Plane {
        Eigen::Vector3f point;
        Eigen::Vector3f normal;
        Plane(const Eigen::Vector3f& p, const Eigen::Vector3f& n) : point(p), normal(n) {}
    };

public:
    void add_mesh(PCLMesh mesh);//添加网格
    void add_line(pcl::PointCloud<pcl::PointXYZ>::Ptr& line);//添加线段
    int addLine(const PCLCloud::Ptr& line);//添加线段并返回索引
    int lineCount() const;//读取当前线段数量
    bool removeLine(int index);//按索引删除线段
    void setLineVisible(int index, bool visible);//设置指定线段显示/隐藏
    bool lineVisible(int index) const;//读取指定线段显示状态
    void setSelectedLineIndex(int index);//设置当前选中的线段索引
    int selectedLineIndex() const;//读取当前选中的线段索引
    PCLCloud::Ptr lineByIndex(int index) const;//读取指定索引线段副本
    void add_pointcloud(PCLCloud cloud);//添加点云
    int pointCloudCount() const;//当前已导入点云数量
    int selectedPointCloudIndex() const;//当前列表选中的点云索引
    QColor pointCloudColor(int index) const;//读取指定点云显示颜色
    void setSelectedPointCloudIndex(int index);//设置列表选中点云，用于高亮显示
    void setSelectedPointCloudIndices(const std::vector<int>& indices);//设置列表多选点云，用于同步高亮
    void setPointCloudColor(int index, const QColor& color);//设置指定点云显示颜色
    void setPointCloudVisible(int index, bool visible);//设置指定点云显示/隐藏
    bool pointCloudVisible(int index) const;//读取指定点云显示状态
    void removePointCloud(int index);//按索引删除点云
    int addMesh(PCLMesh mesh);//添加网格并返回索引
    void setMeshDisplayTag(int index, const QString& tag);//设置网格显示标签（用于渲染风格）
    bool removeMesh(int index);//按索引删除网格
    int meshCount() const;//读取当前网格数量
    void setMeshVisible(int index, bool visible);//设置指定网格显示/隐藏
    bool meshVisible(int index) const;//读取指定网格显示状态
    PCLMesh meshByIndex(int index) const;//读取指定索引网格副本
    const std::vector<int>& selectedIndices() const;//读取当前框选点索引（合并点集下的索引）
    int removeAreaSelectedPoints();//删除当前框选命中的点，返回删除数量
    int removeAreaUnselectedPoints();//删除未框选点（仅保留框选点），返回删除数量
    PCLCloud::Ptr selectedPointsCloud() const;//读取当前框选出的所有可见点云点
    PCLCloud::Ptr mergedPointCloudByIndices(const std::vector<int>& indices, bool onlyVisible = true) const;//按列表索引合并点云
    PCLCloud::Ptr pointCloudByIndex(int index) const;//读取指定索引点云副本
    bool replacePointCloud(int index, const PCLCloud& cloud);//按索引替换点云
    void setPointCloudNormals(int index, const pcl::PointCloud<pcl::Normal>::Ptr& normals);//设置点云法向
    pcl::PointCloud<pcl::Normal>::Ptr pointCloudNormals(int index) const;//读取点云法向
    bool hasConsistentNormals(int index) const;//检查是否有已一致化的法向
    void add_axes(const Eigen::Matrix4f &transform);//添加坐标轴
    void add_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &plane);//添加平面
    void setMultiCornerSelectionEnabled(bool enabled);//启用/禁用多边形多点选择模式
    void setMeshWireframeEnabled(bool enabled) { mesh_wireframe_enabled = enabled; update(); }//是否叠加网格线框
    //点云导入
    PCLCloud::Ptr source_cloud;//源点云(点云1)
    PCLCloud::Ptr target_cloud;//目标点云(点云2)
    //两片点云配准
    std::vector<pcl::PointXYZ> selected_source_points;//源点云选择点
    std::vector<pcl::PointXYZ> selected_target_points;//目标点云选择点
    PCLCloud::Ptr transformed_source_cloud;//粗配准之后的点云
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;//KD树
    //3-2-1对齐建立坐标系
    std::vector<pcl::PointXYZ> selected_align1_points;//前三点
    Eigen::Vector3f plane_p1;//前三点构成平面
    Eigen::Vector3f plane_p2;
    Eigen::Vector3f plane_p3;
    std::vector<pcl::PointXYZ> selected_align2_points;//后三点
    // PCA 切片模式
    void enterPCASliceMode(const Eigen::Vector3f& pc1Dir, const Eigen::Vector3f& origin,
                           float tMin, float tMax,
                           const std::vector<float>& slicePositions,
                           int activeSlice, float sliceHalfThickness);
    void exitPCASliceMode();
    bool isPCASlicing() const { return pca_slicing_active; }
    void setActiveSlicePosition(float t);
    void setActiveSliceIndex(int idx);
    void setPCASlicePlaneExtent(float extent) { pca_pc2Max = pca_pc3Max = extent; }
    std::vector<float> slicePositions() const { return pca_slice_positions; }
    int activeSliceIndex() const { return pca_active_slice; }

    int findSliceAtPixel(const QPoint& pixel) const;

signals:
    void pointSelected();//开始选点信号
    void saveSelectedPointsRequested();//右键菜单请求：将当前选中点保存为新的点云条目
    void lineSelected(int lineIndex);//鼠标点击选中线段
    void pcaSlicePositionChanged(int sliceIndex, float tValue);//PCA 切片位置变化
    void pcaActiveSliceChanged(int sliceIndex);//鼠标点击切换激活切片
protected:
    //重写绘制函数
    virtual void draw() override;
    virtual void postDraw() override;
    //重写鼠标事件函数
    virtual void mousePressEvent(QMouseEvent *e) override;
    virtual void mouseMoveEvent(QMouseEvent *e) override;
    virtual void mouseReleaseEvent(QMouseEvent *e) override;
    //重写键盘事件函数
    virtual void keyPressEvent(QKeyEvent *e) override;
    //重写初始化opengl函数(为了VBO)
    virtual void initializeGL() override;
    //qglviewer世界坐标轴绘制函数
    virtual void drawAxis(float length);

private:
    /*******************网格绘制*****************/
    vector<PCLMesh> mesh_plot;
    vector<PCLCloud> mesh_cloud;
    vector<bool> mesh_visibility;
    vector<QString> mesh_display_tags;
    bool mesh_wireframe_enabled = false;  // 是否叠加三角网格线框(默认关，避免展向环边显成"一横一横")
    void draw_mesh();
    /*******************线段绘制*****************/
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> line_plot;
    std::vector<bool> line_visibility;
    int selected_line_index = -1;
    void drawPointCloudLines();
    /*******************平面绘制*****************/
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> plane_plot;
    void drawPointCloudPlanes();
    /*******************点云绘制*****************/
    vector<PCLCloud> pointclouds_plot;
    vector<QColor> pointcloud_colors;
    vector<pcl::PointCloud<pcl::Normal>::Ptr> pointcloud_normals;
    vector<PCLRGBCloud> rgbpointclouds_plot;
    vector<bool> pointcloud_visibility;
    int selected_pointcloud_index = -1;
    std::vector<int> selected_pointcloud_indices;
    int pointsize=2;//可以不需要
    void draw_pointcloud();
    QColor defaultPointCloudColor(int index) const;
    QColor colorForPointCloud(int index) const;
    void drawSelectedPointCloudHighlight();
    void syncPrimaryPointCloudCache();
    QOpenGLBuffer vbo1;
    QOpenGLBuffer vbo3;
    /*******************鼠标单击选点*****************/
    //高亮所选点
    void highlightSelectedPoints(const std::vector<pcl::PointXYZ>& points);
    //选择点云数据上的点
    void selectPoint(int x, int y, const PCLCloud::Ptr cloud, std::vector<pcl::PointXYZ>& selected_points);
    //选择三角形平面上的点
    void selectPointOnTriangle(int x, int y, const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Eigen::Vector3f& p3, std::vector<pcl::PointXYZ>& selected_points);
    /***************新建局部坐标轴绘制************/
    void draw_axes();
    std::vector<Eigen::Matrix4f> axes_transforms;
    /*******************鼠标框选*****************/
    bool selecting; //框选操作标识符
    QPoint selection_start; //记录框选操作的起始位置
    QPoint selection_end; //记录框选操作的结束位置
    QRect selected_region; //框选区域
    std::vector<int> selected_indices; //框选的点索引（相对合并输入点集）
    std::vector<pcl::PointXYZ> selected_points_cache; //框选命中的实际点（跨点云）
    QOpenGLBuffer vbo2;
    bool middle_panning = false; //中键拖动平移标记
    QPoint middle_last_pos; //中键平移上一帧鼠标位置
    bool right_dragging = false; //右键是否发生拖动（用于区分右键点击菜单和右键拖动旋转）
    QPoint right_press_pos; //右键按下位置
    /****************PCA 切片拖动***************/
    qglviewer::Vec initialCameraPos, initialLookAt, initialUp;
    bool pca_slicing_active = false;
    bool pca_slice_dragging = false;
    Eigen::Vector3f pca_pc1_dir = Eigen::Vector3f::UnitZ();
    Eigen::Vector3f pca_pc2_dir = Eigen::Vector3f::UnitX();
    Eigen::Vector3f pca_pc3_dir = Eigen::Vector3f::UnitY();
    Eigen::Vector3f pca_origin = Eigen::Vector3f::Zero();
    float pca_tMin = 0.0f, pca_tMax = 1.0f;
    float pca_pc2Min = -100.0f, pca_pc2Max = 100.0f;
    float pca_pc3Min = -100.0f, pca_pc3Max = 100.0f;
    int pca_numSlices = 20;
    int pca_active_slice = 0;
    float pca_halfThickness = 0.5f;
    std::vector<float> pca_slice_positions;
    QPoint pca_drag_last_pos;
    void drawPCASlicePlanes();
    /*******************多边形多点选择*****************/
    bool multi_corner_selection_enabled = false;
    std::vector<QPoint> multi_corner_vertices;
    QPoint multi_corner_hover_pos;
    void performPolygonSelection();
    int pickLineAtScreen(int x, int y, float maxPixelDistance = 8.0f) const;
    /*******************平面拟合*****************/
    void DICQGLViewer::performAreaSelection();
    /*******************CUDA-VBO*****************/
    //cudaGraphicsResource *cuda_vbo_resource; // CUDA VBO 资源（如果你计划使用 CUDA 进行点云数据处理（例如数据转换、滤波、计算等），然后再绘制处理后的数据，那么你需要使用 cudaGraphicsResource）
    /*******************表面重建*****************/
    PCLMesh reconstructed_mesh;
};

#endif // DICQGLVIEWER_H
