#include "dicqglviewer.h"
#include <QMouseEvent>
#include <cmath>
#include <Eigen/Dense>
#include <vector>
//CUDA
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include "draw.h"
//#include <cuda_gl_interop.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <qglviewer/qglviewer.h>
#include <qglviewer/manipulatedCameraFrame.h>
#include <vector>
#include <algorithm>
#include <unordered_set>
#include <iostream>
#include <QString>
#include <limits>

using namespace std;

namespace {

void drawViewportGradient(int viewportWidth, int viewportHeight)
{
    GLboolean depthTestEnabled = GL_FALSE;
    GLboolean lightingEnabled = GL_FALSE;
    GLint previousMatrixMode = GL_MODELVIEW;
    glGetBooleanv(GL_DEPTH_TEST, &depthTestEnabled);
    glGetBooleanv(GL_LIGHTING, &lightingEnabled);
    glGetIntegerv(GL_MATRIX_MODE, &previousMatrixMode);

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, viewportWidth, 0, viewportHeight, -1, 1);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glBegin(GL_QUADS);
    glColor3f(0.92f, 0.92f, 0.92f);
    glVertex2f(0.0f, 0.0f);
    glVertex2f(static_cast<float>(viewportWidth), 0.0f);
    glColor3f(1.0f, 1.0f, 1.0f);
    glVertex2f(static_cast<float>(viewportWidth), static_cast<float>(viewportHeight));
    glVertex2f(0.0f, static_cast<float>(viewportHeight));
    glEnd();

    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(previousMatrixMode);

    if (lightingEnabled)
        glEnable(GL_LIGHTING);
    if (depthTestEnabled)
        glEnable(GL_DEPTH_TEST);
}

void drawReferenceGrid(float radius)
{
    const float gridExtent = std::max(200.0f, radius * 0.75f);
    const int divisions = 20;
    const float step = gridExtent * 2.0f / static_cast<float>(divisions);

    GLboolean lightingEnabled = GL_FALSE;
    glGetBooleanv(GL_LIGHTING, &lightingEnabled);
    glDisable(GL_LIGHTING);

    glLineWidth(1.0f);
    glBegin(GL_LINES);
    for (int i = 0; i <= divisions; ++i) {
        const float p = -gridExtent + step * static_cast<float>(i);
        if (i == divisions / 2) {
            glColor4f(0.50f, 0.50f, 0.50f, 0.80f);
        } else {
            glColor4f(0.75f, 0.75f, 0.75f, 0.40f);
        }

        glVertex3f(p, -gridExtent, 0.0f);
        glVertex3f(p,  gridExtent, 0.0f);
        glVertex3f(-gridExtent, p, 0.0f);
        glVertex3f( gridExtent, p, 0.0f);
    }
    glEnd();

    if (lightingEnabled)
        glEnable(GL_LIGHTING);
}

}

DICQGLViewer::DICQGLViewer(QWidget *parent)
    : QGLViewer{parent}
{
    //初始设置
    camera()->setPosition(qglviewer::Vec(1000, 1000, 300)); // MATLAB view(45,30) 标准3D视角
    camera()->lookAt(qglviewer::Vec(0, -500, 0)); // 设置相机的观察点
    camera()->setUpVector(qglviewer::Vec(0, 0, 1)); // Z轴朝上
    initialCameraPos = camera()->position();
    initialLookAt = qglviewer::Vec(0, -500, 0);
    initialUp = qglviewer::Vec(0, 0, 1);
    setSceneRadius(1500);
    setAxisIsDrawn(false);
    setTextIsEnabled(false);
    setAutoFillBackground(false);
    setMouseTracking(true);
    selecting = false;
    // 鼠标交互通过事件函数控制：
    // 左键拖动用于框选；右键拖动用于旋转；中键拖动用于平移；滚轮用于缩放
    //两块点云配准
    source_cloud.reset(new PCLCloud);
    target_cloud.reset(new PCLCloud);
    transformed_source_cloud.reset(new PCLCloud);
}

DICQGLViewer::~DICQGLViewer(){
    if (vbo1.isCreated()) {
        vbo1.destroy();
    }
    if (vbo2.isCreated()) {
        vbo2.destroy();
    }
    if (vbo3.isCreated()) {
        vbo3.destroy();
    }
}

//初始化函数重载
void DICQGLViewer::initializeGL() {
    QGLViewer::initializeGL();//继承父类
    glEnable(GL_DEPTH_TEST);//深度测试
    /***********VBO的初始化************/
    //点云1绘制
    vbo1.create();
    vbo1.bind();
    vbo1.allocate(nullptr, sizeof(float) * 3 * 400000000);//分配空间（根据点的多少设置上限）
    vbo1.release();
    //点云2绘制
    vbo3.create();
    vbo3.bind();
    vbo3.allocate(nullptr, sizeof(float) * 3 * 400000000);//分配空间（根据点的多少设置上限）
    vbo3.release();
    //高亮框选点绘制
    vbo2.create();
    vbo2.bind();
    vbo2.allocate(nullptr, sizeof(float) * 3 * 400000000);
    vbo2.release();
    //使能透明度与抗锯齿，让点云、坐标轴和选择框边缘更柔和
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
#ifdef GL_MULTISAMPLE
    glEnable(GL_MULTISAMPLE);
#endif
}

//绘制函数重载
void DICQGLViewer::draw()
{
    glDisable(GL_LIGHTING);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // MATLAB风格白色背景
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // 清除颜色和深度缓冲区
    glEnable(GL_DEPTH_TEST);
    drawReferenceGrid(sceneRadius());
    drawAxis(sceneRadius()/5.0);
    if(pca_slicing_active)
        drawPCASlicePlanes();

//    // 绘制点云（VBO）（带透明度->比不带透明度版本要卡）
//    if (!pointclouds_plot.empty()) {
//        const PCLCloud& cloud = pointclouds_plot[0];
//        int num_points = cloud.points.size();
//        if (num_points > 0) {
//            // 绑定顶点VBO
//            vbo1.bind();
//            // 将VBO映射到内存中(CPU->GPU)
//            float* vbo_data = static_cast<float*>(vbo1.map(QOpenGLBuffer::WriteOnly));
//            if (vbo_data) {
//                for (int i = 0; i < num_points; ++i) {
//                    // 将点云数据复制到VBO
//                    vbo_data[i * 3] = cloud.points[i].x;
//                    vbo_data[i * 3 + 1] = cloud.points[i].y;
//                    vbo_data[i * 3 + 2] = cloud.points[i].z;
//                }
//                vbo1.unmap(); // 解映射
//            } else {
//                qWarning() << "Failed to map VBO";
//                return;
//            }
//            vbo1.release(); // 解绑顶点VBO

//            // 创建并绑定颜色VBO
//            QOpenGLBuffer colorVbo(QOpenGLBuffer::VertexBuffer);
//            colorVbo.create();
//            colorVbo.bind();
//            colorVbo.allocate(num_points * 4 * sizeof(float)); // RGBA每个点4个float

//            // 将颜色数据映射到VBO
//            float* color_data = static_cast<float*>(colorVbo.map(QOpenGLBuffer::WriteOnly));
//            if (color_data) {
//                for (int i = 0; i < num_points; ++i) {
//                    // 设置颜色为白色，透明度为0.5
//                    color_data[i * 4] = 1.0f; // R
//                    color_data[i * 4 + 1] = 1.0f; // G
//                    color_data[i * 4 + 2] = 1.0f; // B
//                    color_data[i * 4 + 3] = 0.1f; // A
//                }
//                colorVbo.unmap(); // 解映射
//            } else {
//                qWarning() << "Failed to map color VBO";
//                return;
//            }
//            colorVbo.release(); // 解绑颜色VBO

//            // 启用混合（用于透明度）
//            glEnable(GL_BLEND);
//            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

//            // 设置顶点数组并绘制点
//            glEnableClientState(GL_VERTEX_ARRAY); // 启用顶点数组功能
//            vbo1.bind();
//            glVertexPointer(3, GL_FLOAT, 0, nullptr);
//            vbo1.release();

//            // 设置颜色数组并绘制点
//            glEnableClientState(GL_COLOR_ARRAY); // 启用颜色数组功能
//            colorVbo.bind();
//            glColorPointer(4, GL_FLOAT, 0, nullptr);
//            colorVbo.release();

//            // 绘制点
//            glPointSize(1.0f);
//            glDrawArrays(GL_POINTS, 0, num_points);

//            // 禁用顶点和颜色数组功能
//            glDisableClientState(GL_VERTEX_ARRAY);
//            glDisableClientState(GL_COLOR_ARRAY);

//            // 关闭混合
//            glDisable(GL_BLEND);
//        } else {
//            qWarning() << "No points to draw";
//        }
//    }

    //绘制点云1（VBO1）
    if (!pointclouds_plot.empty() && (pointcloud_visibility.empty() || pointcloud_visibility[0])) {
        const PCLCloud& cloud = pointclouds_plot[0];
        int num_points = cloud.points.size();
        if (num_points > 0) {
            // 绑定VBO
            vbo1.bind();
            // 将VBO映射到内存中(CPU->GPU)
            float* vbo_data = static_cast<float*>(vbo1.map(QOpenGLBuffer::WriteOnly));
            if (vbo_data) {
                //memcpy(vbo_data, cloud.points.data(), sizeof(float) * num_points * 3);//这种方法绘制有误
                for (int i = 0; i < num_points; ++i) {
                    // 将点云数据复制到VBO
                    vbo_data[i * 3] = cloud.points[i].x;
                    vbo_data[i * 3 + 1] = cloud.points[i].y;
                    vbo_data[i * 3 + 2] = cloud.points[i].z;
                }
                vbo1.unmap(); // 解映射
            } else {
                qWarning() << "Failed to map VBO";
                return;
            }
            // 设置顶点数组并绘制点(VBO)：源点云颜色可由标签栏和颜色下拉框修改
            const QColor cloudColor = colorForPointCloud(0);
            glColor3f(cloudColor.redF(), cloudColor.greenF(), cloudColor.blueF());
            glPointSize(2.0f);
            glEnableClientState(GL_VERTEX_ARRAY); // 启用顶点数组功能
            glVertexPointer(3, GL_FLOAT, 0, nullptr);
            glDrawArrays(GL_POINTS, 0, num_points);
            glDisableClientState(GL_VERTEX_ARRAY); // 禁用顶点数组功能
            // 解绑VBO
            vbo1.release();
        }
    }
    //绘制点云2（VBO3）
    if (pointclouds_plot.size() > 1 &&
        (pointcloud_visibility.size() <= 1 || pointcloud_visibility[1])) {
        const PCLCloud& cloud = pointclouds_plot[1];
        int num_points = cloud.points.size();
        if (num_points > 0) {
            // 绑定VBO
            vbo3.bind();
            // 将VBO映射到内存中(CPU->GPU)
            float* vbo_data = static_cast<float*>(vbo3.map(QOpenGLBuffer::WriteOnly));
            if (vbo_data) {
                //memcpy(vbo_data, cloud.points.data(), sizeof(float) * num_points * 3);//这种方法绘制有误
                for (int i = 0; i < num_points; ++i) {
                    // 将点云数据复制到VBO
                    vbo_data[i * 3] = cloud.points[i].x;
                    vbo_data[i * 3 + 1] = cloud.points[i].y;
                    vbo_data[i * 3 + 2] = cloud.points[i].z;
                }
                vbo3.unmap(); // 解映射
            } else {
                qWarning() << "Failed to map VBO";
                return;
            }
            // 设置顶点数组并绘制点(VBO)：目标点云颜色可由标签栏和颜色下拉框修改
            const QColor cloudColor = colorForPointCloud(1);
            glColor3f(cloudColor.redF(), cloudColor.greenF(), cloudColor.blueF());
            glPointSize(2.0f);
            glEnableClientState(GL_VERTEX_ARRAY); // 启用顶点数组功能
            glVertexPointer(3, GL_FLOAT, 0, nullptr);
            glDrawArrays(GL_POINTS, 0, num_points);
            glDisableClientState(GL_VERTEX_ARRAY); // 禁用顶点数组功能
            // 解绑VBO
            vbo3.release();
        }
    }

    // 批量导入时，第 3 个及之后的点云也要显示出来。
    // 前两个点云仍用 VBO 绘制，便于后续源/目标点云配准；额外点云用可修改配色区分。
    for (int cloudIndex = 2; cloudIndex < static_cast<int>(pointclouds_plot.size()); ++cloudIndex) {
        const PCLCloud& cloud = pointclouds_plot[cloudIndex];
        if (cloud.points.empty())
            continue;
        if (cloudIndex < static_cast<int>(pointcloud_visibility.size()) && !pointcloud_visibility[cloudIndex])
            continue;

        const QColor cloudColor = colorForPointCloud(cloudIndex);
        glColor3f(cloudColor.redF(), cloudColor.greenF(), cloudColor.blueF());
        glPointSize(2.0f);
        glBegin(GL_POINTS);
        for (const auto& point : cloud.points) {
            glVertex3f(point.x, point.y, point.z);
        }
        glEnd();
    }

    // 第 3 个及之后的“拟合/重建结果”同样会作为普通点云条目加入左侧列表，
    // 这里只负责把它们按点云方式画出来；如果想显示成真正的3D几何体，需通过 mesh/plane 的专门绘制接口。

    // 标签栏中选中的点云用更大的亮色点覆盖一遍，形成高亮效果。
    drawSelectedPointCloudHighlight();

    if(axes_transforms.size()!=0)//绘制新建坐标轴
    {
        draw_axes();
    }

    if(plane_plot.size()!=0)//绘制平面
    {
        drawPointCloudPlanes();
    }

    if(mesh_plot.size()!=0)//绘制重建曲面/网格
    {
        glEnable(GL_DEPTH_TEST);
        draw_mesh();
    }

    // 注意：当前拟合结果如果只是以“采样点云”形式加入列表，那么这里仍然会按点云绘制。
    // 若要在界面中看到球/圆柱/平面这种“真正的3D图形”，需要把拟合结果单独存入 mesh/plane 结构并在此处调用对应绘制函数。

    if(line_plot.size()!= 0)//绘制线段
    {
        drawPointCloudLines();
    }

    //高亮单击选点
    highlightSelectedPoints(selected_source_points);
    highlightSelectedPoints(selected_target_points);
    highlightSelectedPoints(selected_align1_points);
    highlightSelectedPoints(selected_align2_points);

    // 高亮框选选点（VBO2）
    if (!selected_points_cache.empty()) {
        glDisable(GL_DEPTH_TEST);
        int selected_num_points = static_cast<int>(selected_points_cache.size());
        vbo2.bind();
        float* selected_vbo_data = static_cast<float*>(vbo2.map(QOpenGLBuffer::WriteOnly));
        if (selected_vbo_data) {
            for (int i = 0; i < selected_num_points; ++i) {
                const auto& point = selected_points_cache[i];
                selected_vbo_data[i * 3] = point.x;
                selected_vbo_data[i * 3 + 1] = point.y;
                selected_vbo_data[i * 3 + 2] = point.z;
            }
            vbo2.unmap();
        } else {
            qWarning() << "Failed to map selected points VBO";
            vbo2.release();
            return;
        }
        glColor3f(1.0f, 0.24f, 0.36f); //亮红色，突出框选结果
        glPointSize(5.0f); // 设置点的大小
        glEnableClientState(GL_VERTEX_ARRAY); // 启用顶点数组功能
        glVertexPointer(3, GL_FLOAT, 0, nullptr); // 设置顶点指针
        glDrawArrays(GL_POINTS, 0, selected_num_points); // 绘制选中的点
        glDisableClientState(GL_VERTEX_ARRAY); // 禁用顶点数组功能
        vbo2.release();
        glEnable(GL_DEPTH_TEST);
    }

    // 绘制拟合平面
    if (fitted_plane_coefficients.size()==4) {
        drawFittedPlane();
    }

    // 绘制框选框 / 多边形选框
    if (selecting || multi_corner_selection_enabled) {
        // 切换到2D绘制模式
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0, width(), height(), 0, -1, 1);

        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();

        glDisable(GL_DEPTH_TEST); //禁用深度测试以确保选择框绘制在最前面

        if (selecting) {
            glColor4f(0.10f, 0.58f, 1.00f, 0.16f); // 半透明填充，拖拽区域更直观
            glBegin(GL_QUADS);
            glVertex2i(selection_start.x(), selection_start.y());
            glVertex2i(selection_end.x(), selection_start.y());
            glVertex2i(selection_end.x(), selection_end.y());
            glVertex2i(selection_start.x(), selection_end.y());
            glEnd();

            glColor3f(0.24f, 0.78f, 1.0f); //设置选择框颜色为亮蓝色
            glLineWidth(2.0f); // 设置线宽

            glBegin(GL_LINE_LOOP);
            glVertex2i(selection_start.x(), selection_start.y());
            glVertex2i(selection_end.x(), selection_start.y());
            glVertex2i(selection_end.x(), selection_end.y());
            glVertex2i(selection_start.x(), selection_end.y());
            glEnd();
        }

        if (multi_corner_selection_enabled && !multi_corner_vertices.empty()) {
            glColor3f(0.24f, 0.90f, 0.45f);
            glLineWidth(2.0f);
            glBegin(GL_LINE_STRIP);
            for (const QPoint& p : multi_corner_vertices)
                glVertex2i(p.x(), p.y());
            glVertex2i(multi_corner_hover_pos.x(), multi_corner_hover_pos.y());
            glEnd();

            glPointSize(6.0f);
            glBegin(GL_POINTS);
            for (const QPoint& p : multi_corner_vertices)
                glVertex2i(p.x(), p.y());
            glEnd();
        }

        glEnable(GL_DEPTH_TEST); // 恢复深度测试

        // 恢复3D绘制模式
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
        glPopMatrix();
    }
}

void DICQGLViewer::postDraw()
{
    // 不调用 QGLViewer::postDraw()，避免显示默认快捷键提示
    glColor4f(0.3f, 0.3f, 0.3f, 0.85f);
    drawText(width() - 320, 20, tr("Left: Select  |  Right: Rotate  |  Middle: Pan"));
}

//绘制世界坐标系函数重载
void DICQGLViewer::drawAxis(float length) {
    const qreal charWidth = length / 40.0;
    const qreal charHeight = length / 30.0;
    const qreal charShift = 1.04 * length;

    GLboolean lighting, colorMaterial;
    glGetBooleanv(GL_LIGHTING, &lighting);
    glGetBooleanv(GL_COLOR_MATERIAL, &colorMaterial);

    glDisable(GL_LIGHTING);

    // 设置线条宽度
    glLineWidth(0.001f);
    glColor3f(0.74f, 0.85f, 1.0f);

    glBegin(GL_LINES);
    // The X
    glVertex3d(charShift, charWidth, -charHeight);
    glVertex3d(charShift, -charWidth, charHeight);
    glVertex3d(charShift, -charWidth, -charHeight);
    glVertex3d(charShift, charWidth, charHeight);
    // The Y
    glVertex3d(charWidth, charShift, charHeight);
    glVertex3d(0.0, charShift, 0.0);
    glVertex3d(-charWidth, charShift, charHeight);
    glVertex3d(0.0, charShift, 0.0);
    glVertex3d(0.0, charShift, 0.0);
    glVertex3d(0.0, charShift, -charHeight);
    // The Z
    glVertex3d(-charWidth, charHeight, charShift);
    glVertex3d(charWidth, charHeight, charShift);
    glVertex3d(charWidth, charHeight, charShift);
    glVertex3d(-charWidth, -charHeight, charShift);
    glVertex3d(-charWidth, -charHeight, charShift);
    glVertex3d(charWidth, -charHeight, charShift);
    glEnd();

    glEnable(GL_LIGHTING);
    glDisable(GL_COLOR_MATERIAL);

    float color[4];
    color[0] = 0.22f;
    color[1] = 0.59f;
    color[2] = 1.0f;
    color[3] = 1.0f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
    QGLViewer::drawArrow(length, 0.014 * length);

    color[0] = 1.0f;
    color[1] = 0.31f;
    color[2] = 0.37f;
    color[3] = 1.0f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
    glPushMatrix();
    glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
    QGLViewer::drawArrow(length, 0.014 * length);
    glPopMatrix();

    color[0] = 0.22f;
    color[1] = 0.82f;
    color[2] = 0.57f;
    color[3] = 1.0f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
    glPushMatrix();
    glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
    QGLViewer::drawArrow(length, 0.014 * length);
    glPopMatrix();

    if (colorMaterial)
        glEnable(GL_COLOR_MATERIAL);
    if (!lighting)
        glDisable(GL_LIGHTING);
}

int DICQGLViewer::pickLineAtScreen(int x, int y, float maxPixelDistance) const
{
    // 屏幕空间线段拾取：在鼠标附近寻找最近可见线段，命中后返回 line 索引。
    if(line_plot.empty())
        return -1;

    const float maxDist2 = maxPixelDistance * maxPixelDistance;
    float bestDist2 = maxDist2;
    int bestIndex = -1;

    auto dist2PointToSegment = [](float px, float py, float ax, float ay, float bx, float by) {
        const float vx = bx - ax;
        const float vy = by - ay;
        const float wx = px - ax;
        const float wy = py - ay;
        const float vv = vx * vx + vy * vy;
        float t = 0.0f;
        if(vv > 1e-6f)
            t = std::max(0.0f, std::min(1.0f, (wx * vx + wy * vy) / vv));
        const float cx = ax + t * vx;
        const float cy = ay + t * vy;
        const float dx = px - cx;
        const float dy = py - cy;
        return dx * dx + dy * dy;
    };

    for(int i = 0; i < static_cast<int>(line_plot.size()); ++i)
    {
        if(i < static_cast<int>(line_visibility.size()) && !line_visibility[i])
            continue;
        const auto& cloud = line_plot[i];
        if(!cloud || cloud->points.size() < 2)
            continue;

        for(std::size_t k = 1; k < cloud->points.size(); ++k)
        {
            const auto& p0 = cloud->points[k - 1];
            const auto& p1 = cloud->points[k];
            const qglviewer::Vec s0 = camera()->projectedCoordinatesOf(qglviewer::Vec(p0.x, p0.y, p0.z));
            const qglviewer::Vec s1 = camera()->projectedCoordinatesOf(qglviewer::Vec(p1.x, p1.y, p1.z));
            const float d2 = dist2PointToSegment(static_cast<float>(x), static_cast<float>(y),
                                                 static_cast<float>(s0.x), static_cast<float>(s0.y),
                                                 static_cast<float>(s1.x), static_cast<float>(s1.y));
            if(d2 < bestDist2)
            {
                bestDist2 = d2;
                bestIndex = i;
            }
        }
    }

    return bestIndex;
}

//鼠标事件处理函数重载
void DICQGLViewer::mousePressEvent(QMouseEvent *e)
{
    // PCA 切片模式：左键点击选中切片 / 拖动
    if(pca_slicing_active && e->button() == Qt::LeftButton && e->modifiers() == Qt::NoModifier)
    {
        const int hitSlice = findSliceAtPixel(e->pos());
        if(hitSlice >= 0 && hitSlice != pca_active_slice)
        {
            pca_active_slice = hitSlice;
            emit pcaActiveSliceChanged(hitSlice);
            update();
        }
        if(hitSlice >= 0)
        {
            pca_slice_dragging = true;
            pca_drag_last_pos = e->pos();
        }
        return;
    }

    if (multi_corner_selection_enabled && e->modifiers() == Qt::NoModifier) {
        if (e->button() == Qt::LeftButton) {
            multi_corner_vertices.push_back(e->pos());
            multi_corner_hover_pos = e->pos();
            update();
            return;
        }

        if (e->button() == Qt::RightButton) {
            performPolygonSelection();
            multi_corner_vertices.clear();
            update();
            return;
        }
    }

    if (e->button() == Qt::LeftButton && e->modifiers() == Qt::NoModifier) {//左键：框选
        selecting = true;
        selection_start = e->pos();
        selection_end = e->pos();
        return;
    }

    // 中键按下：进入手动平移模式
    if (e->button() == Qt::MidButton && e->modifiers() == Qt::NoModifier) {
        middle_panning = true;
        middle_last_pos = e->pos();
        return;
    }

    // 右键：记录按下位置。拖动时用于旋转；仅点击时用于弹菜单。
    if (e->button() == Qt::RightButton && e->modifiers() == Qt::NoModifier) {
        right_dragging = false;
        right_press_pos = e->pos();
        QMouseEvent remappedEvent(
            e->type(),
            e->localPos(),
            e->windowPos(),
            e->screenPos(),
            Qt::LeftButton,
            (e->buttons() & ~Qt::RightButton) | Qt::LeftButton,
            e->modifiers(),
            e->source());
        QGLViewer::mousePressEvent(&remappedEvent);
        return;
    }

    QGLViewer::mousePressEvent(e);
}
void DICQGLViewer::mouseMoveEvent(QMouseEvent *e) {
    // PCA 切片拖动中：沿 PC1 轴移动当前激活切片位置
    if(pca_slicing_active && pca_slice_dragging && (e->buttons() & Qt::LeftButton))
    {
        if(pca_active_slice >= 0 && pca_active_slice < static_cast<int>(pca_slice_positions.size()))
        {
            const float frac = 1.0f - static_cast<float>(e->pos().y()) / static_cast<float>(std::max(1, height()));
            float t = pca_tMin + frac * (pca_tMax - pca_tMin);
            t = std::max(pca_tMin, std::min(pca_tMax, t));
            pca_slice_positions[pca_active_slice] = t;
            emit pcaSlicePositionChanged(pca_active_slice, t);
        }
        return;
    }

    if (multi_corner_selection_enabled) {
        multi_corner_hover_pos = e->pos();
        update();
        return;
    }

    if (selecting) {
        selection_end = e->pos();
        update();
        return;
    }

    // 中键拖动手动平移（按像素位移换算到世界坐标）
    if (middle_panning && (e->buttons() & Qt::MidButton) && e->modifiers() == Qt::NoModifier) {
        const QPoint delta = e->pos() - middle_last_pos;
        middle_last_pos = e->pos();

        qglviewer::Camera* cam = camera();
        const qglviewer::Vec pos = cam->position();
        const qglviewer::Vec view = cam->viewDirection();
        const qglviewer::Vec pivot = cam->pivotPoint();
        const double distance = std::fabs((pos - pivot) * view);
        const double worldPerPixel = (2.0 * distance * std::tan(cam->fieldOfView() * 0.5)) / std::max(1, height());

        const qglviewer::Vec right = cam->rightVector();
        const qglviewer::Vec up = cam->upVector();
        const qglviewer::Vec t = (-right * static_cast<double>(delta.x()) + up * static_cast<double>(delta.y())) * worldPerPixel;

        cam->frame()->translate(t);
        update();
        return;
    }

    // 右键拖动改为“旋转”：把移动事件中的右键状态转换为左键状态
    if ((e->buttons() & Qt::RightButton) && e->modifiers() == Qt::NoModifier) {
        if((e->pos() - right_press_pos).manhattanLength() > 3)
            right_dragging = true;

        QMouseEvent remappedEvent(
            e->type(),
            e->localPos(),
            e->windowPos(),
            e->screenPos(),
            Qt::NoButton,
            (e->buttons() & ~Qt::RightButton) | Qt::LeftButton,
            e->modifiers(),
            e->source());
        QGLViewer::mouseMoveEvent(&remappedEvent);
        return;
    }

    QGLViewer::mouseMoveEvent(e);
}
void DICQGLViewer::mouseReleaseEvent(QMouseEvent *e) {
    // PCA 切片拖动释放
    if(pca_slice_dragging && e->button() == Qt::LeftButton)
    {
        pca_slice_dragging = false;
        return;
    }

    if (multi_corner_selection_enabled && e->modifiers() == Qt::NoModifier) {
        if (e->button() == Qt::LeftButton || e->button() == Qt::RightButton) {
            update();
            return;
        }
    }

    if (e->button() == Qt::LeftButton && selecting) {
        selecting = false;
        selection_end = e->pos();

        if((selection_end - selection_start).manhattanLength() <= 3)
        {
            // 左键微小位移视为点击：优先做回转轴线拾取，再决定是否执行框选。
            const int hitLine = pickLineAtScreen(selection_end.x(), selection_end.y(), 8.0f);
            if(hitLine >= 0)
            {
                setSelectedLineIndex(hitLine);
                emit lineSelected(hitLine);
                update();
                return;
            }
        }

        // 开始计时
        auto start = std::chrono::high_resolution_clock::now();
        performAreaSelection();
        // 结束计时
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::cout << "performAreaSelection took " << elapsed.count() << " seconds." << std::endl;
        update();
        return;
    }

    if (e->button() == Qt::MidButton && middle_panning) {
        middle_panning = false;
        update();
        return;
    }

    // 右键释放：若发生拖动则作为旋转结束；若未拖动则弹出框选点菜单。
    if (e->button() == Qt::RightButton && e->modifiers() == Qt::NoModifier) {
        QMouseEvent remappedEvent(
            e->type(),
            e->localPos(),
            e->windowPos(),
            e->screenPos(),
            Qt::LeftButton,
            (e->buttons() & ~Qt::RightButton),
            e->modifiers(),
            e->source());
        QGLViewer::mouseReleaseEvent(&remappedEvent);
        camera()->frame()->stopSpinning(); // 禁止惯性

        if(!right_dragging && !selected_points_cache.empty()) {
            QMenu menu(this);
            QAction* deleteAction = menu.addAction(tr("Delete selected points"));
            QAction* deleteOthersAction = menu.addAction(tr("Delete unselected points"));
            QAction* saveCurrentAction = menu.addAction(tr("Save selected points to list"));
            QAction* chosen = menu.exec(mapToGlobal(e->pos()));
            if(chosen == deleteAction)
                removeAreaSelectedPoints();
            else if(chosen == deleteOthersAction)
                removeAreaUnselectedPoints();
            else if(chosen == saveCurrentAction)
                emit saveSelectedPointsRequested();
        }

        right_dragging = false;
        update();
        return;
    }

    QGLViewer::mouseReleaseEvent(e);
    update();
}
//键盘事件处理函数重载
void DICQGLViewer::keyPressEvent(QKeyEvent *e)
{
    // 空格键：回到初始视角
    if(e->key() == Qt::Key_Space && e->modifiers() == Qt::NoModifier)
    {
        camera()->frame()->setPositionAndOrientation(initialCameraPos, qglviewer::Quaternion());
        camera()->lookAt(initialLookAt);
        camera()->setUpVector(initialUp);
        update();
        return;
    }

    //禁用ESC
    if(e->key() != Qt::Key_Escape)
    {
        QGLViewer::keyPressEvent(e);
    }
}
//****************鼠标选点（单击、拉框）***************//
void DICQGLViewer::performAreaSelection() {
    selected_indices.clear();
    selected_points_cache.clear();

    std::vector<float> points;
    points.reserve(10000000);

    for (int cloudIndex = 0; cloudIndex < static_cast<int>(pointclouds_plot.size()); ++cloudIndex) {
        if (cloudIndex < static_cast<int>(pointcloud_visibility.size()) && !pointcloud_visibility[cloudIndex])
            continue;

        const PCLCloud& cloud = pointclouds_plot[cloudIndex];
        for (const auto& point : cloud.points) {
            points.push_back(point.x);
            points.push_back(point.y);
            points.push_back(point.z);
        }
    }

    if (points.empty())
        return;

    selected_indices.reserve(points.size() / 3);
    int min_x = std::min(selection_start.x(), selection_end.x());
    int max_x = std::max(selection_start.x(), selection_end.x());
    int min_y = std::min(selection_start.y(), selection_end.y());
    int max_y = std::max(selection_start.y(), selection_end.y());

    // 获取模型视图矩阵
    qglviewer::Camera* cam = camera();
    GLdouble model_view_matrix[16];
    GLdouble projection_matrix[16];
    GLint viewport[4];
    makeCurrent();
    //cam->getModelViewMatrix(model_view_matrix);
    glGetDoublev(GL_MODELVIEW_MATRIX, model_view_matrix);
    glGetDoublev(GL_PROJECTION_MATRIX, projection_matrix);
    //cam->getProjectionMatrix(projection_matrix);

    //cam->getViewport(viewport);
    glGetIntegerv(GL_VIEWPORT, viewport);

    std::cout << "ModelView Matrix: " << std::endl;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            std::cout << std::setw(12) << model_view_matrix[i * 4 + j] << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "Projection Matrix: " << std::endl;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            std::cout << std::setw(12) << projection_matrix[i * 4 + j] << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "Viewport: " << viewport[0] << ", " << viewport[1] << ", " << viewport[2] << ", " << viewport[3] << std::endl;

    // 将矩阵转换为float数组
    std::vector<float> model_view_matrix_float(16);
    std::vector<float> projection_matrix_float(16);
    for (int i = 0; i < 16; ++i) {
        model_view_matrix_float[i] = static_cast<float>(model_view_matrix[i]);
        projection_matrix_float[i] = static_cast<float>(projection_matrix[i]);
    }

    // 调用CUDA函数：在“所有可见点云合并后”的点集上进行框选
    performAreaSelectionCuda(points, model_view_matrix_float.data(), projection_matrix_float.data(), viewport, min_x, max_x, min_y, max_y, selected_indices);

    selected_points_cache.reserve(selected_indices.size());
    for (int idx : selected_indices) {
        const int base = idx * 3;
        if (base + 2 < static_cast<int>(points.size())) {
            selected_points_cache.push_back(pcl::PointXYZ(points[base], points[base + 1], points[base + 2]));
        }
    }

    std::cout << "选中点的个数: " << selected_points_cache.size() << std::endl;
}

void DICQGLViewer::performPolygonSelection() {
    selected_indices.clear();
    selected_points_cache.clear();

    if (multi_corner_vertices.size() < 3)
        return;

    std::vector<float> points;
    points.reserve(10000000);

    for (int cloudIndex = 0; cloudIndex < static_cast<int>(pointclouds_plot.size()); ++cloudIndex) {
        if (cloudIndex < static_cast<int>(pointcloud_visibility.size()) && !pointcloud_visibility[cloudIndex])
            continue;

        const PCLCloud& cloud = pointclouds_plot[cloudIndex];
        for (const auto& point : cloud.points) {
            points.push_back(point.x);
            points.push_back(point.y);
            points.push_back(point.z);
        }
    }

    if (points.empty())
        return;

    selected_indices.reserve(points.size() / 3);

    GLdouble model_view_matrix[16];
    GLdouble projection_matrix[16];
    GLint viewport[4];
    makeCurrent();
    glGetDoublev(GL_MODELVIEW_MATRIX, model_view_matrix);
    glGetDoublev(GL_PROJECTION_MATRIX, projection_matrix);
    glGetIntegerv(GL_VIEWPORT, viewport);

    QPolygon polygon;
    for (const QPoint& p : multi_corner_vertices)
        polygon << p;

    for (int idx = 0; idx < static_cast<int>(points.size() / 3); ++idx) {
        GLdouble sx = 0.0, sy = 0.0, sz = 0.0;
        gluProject(points[idx * 3], points[idx * 3 + 1], points[idx * 3 + 2],
                   model_view_matrix, projection_matrix, viewport,
                   &sx, &sy, &sz);

        const int screenX = static_cast<int>(std::lround(sx));
        const int screenY = static_cast<int>(std::lround(viewport[3] - sy));
        if (polygon.containsPoint(QPoint(screenX, screenY), Qt::OddEvenFill)) {
            selected_indices.push_back(idx);
            selected_points_cache.push_back(pcl::PointXYZ(points[idx * 3], points[idx * 3 + 1], points[idx * 3 + 2]));
        }
    }
}

//单击选择点云点
void DICQGLViewer::selectPoint(int x, int y, const PCLCloud::Ptr cloud, std::vector<pcl::PointXYZ>& selected_points)
{
    //获取相机对象
    qglviewer::Camera* cam = camera();
    if (!cam) {
        std::cerr << "Camera not initialized!" << std::endl;
        return;
    }
    //获取鼠标点击位置的世界坐标
    QPoint mousePos(x, y);//鼠标屏幕点击点
    bool found = true;//指示是否找到了对应的点的三维信息
    qglviewer::Vec point = cam->pointUnderPixel(mousePos,found);
    if (!found) {
        std::cerr << "Failed to get 3D point from mouse click." << std::endl;
        return;
    }
    std::cout<<  point.x << "," << point.y << "," << point.z << std::endl;
    // 射线起点为相机位置，方向为相机位置到鼠标点击位置
    Eigen::Vector3f ray_origin(cam->position().x, cam->position().y, cam->position().z);
    Eigen::Vector3f ray_direction(point.x - ray_origin.x(), point.y - ray_origin.y(), point.z - ray_origin.z());
    ray_direction.normalize();
    //将最小距离值初始化为很大的数
    float min_dist = std::numeric_limits<float>::max();
    pcl::PointXYZ selected_point;
    // 遍历点云，找到射线最近的点
    for (const auto& p : *cloud)
    {
        Eigen::Vector3f point(p.x, p.y, p.z);
        Eigen::Vector3f to_point = point - ray_origin;

        // 计算点到射线的距离
        float dist = to_point.cross(ray_direction).norm();

        if (dist < min_dist)
        {
            min_dist = dist;
            selected_point = p;
        }
    }
    // 如果找到了最近的点，则将其添加到选中的点的列表中
    if (min_dist < std::numeric_limits<float>::max())
    {
        selected_points.push_back(selected_point);
        std::cout << "Selected point: (" << selected_point.x << ", " << selected_point.y << ", " << selected_point.z << ")" << std::endl;
    }
    else
    {
        std::cerr << "No point found close to the ray." << std::endl;
    }
}

//单击选择平面点
void DICQGLViewer::selectPointOnTriangle(int x, int y, const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Eigen::Vector3f& p3, std::vector<pcl::PointXYZ> &selected_points)
{
    // 获取相机对象
    qglviewer::Camera* cam = camera();
    if (!cam) {
        std::cerr << "Camera not initialized!" << std::endl;
        return;
    }
    // 获取鼠标点击位置的世界坐标
    QPoint mousePos(x, y);
    bool found = true;
    qglviewer::Vec point = cam->pointUnderPixel(mousePos, found);
    if (!found) {
        std::cerr << "Failed to get 3D point from mouse click." << std::endl;
        return;
    }
    // 射线起点为鼠标点击位置的世界坐标
    Eigen::Vector3f ray_origin(point.x, point.y, point.z);
    // 构造射线方向（从相机位置指向鼠标点击位置）
    Eigen::Vector3f cam_position(cam->position().x, cam->position().y, cam->position().z);
    Eigen::Vector3f ray_direction = (ray_origin - cam_position).normalized();
    // 计算平面的法向量
    Eigen::Vector3f normal = (p2 - p1).cross(p3 - p1).normalized();
    // 计算射线与平面的交点
    float denom = ray_direction.dot(normal);
    if (fabs(denom) > 1e-6) {
        // 射线与平面相交
        float t = (p1 - ray_origin).dot(normal) / denom;
        Eigen::Vector3f intersection_point = ray_origin + t * ray_direction;
        // 检查交点是否在三角形内部
        Eigen::Vector3f v0 = p2 - p1;
        Eigen::Vector3f v1 = p3 - p1;
        Eigen::Vector3f v2 = intersection_point - p1;
        float dot00 = v0.dot(v0);
        float dot01 = v0.dot(v1);
        float dot02 = v0.dot(v2);
        float dot11 = v1.dot(v1);
        float dot12 = v1.dot(v2);
        float invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
        float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        float v = (dot00 * dot12 - dot01 * dot02) * invDenom;
        if (u >= 0 && v >= 0 && u + v <= 1) {
            // 交点在三角形内部
            selected_points.push_back(pcl::PointXYZ(intersection_point.x(), intersection_point.y(), intersection_point.z()));
            std::cout << "Selected point on triangle: (" << intersection_point.x() << ", " << intersection_point.y() << ", " << intersection_point.z() << ")" << std::endl;
        }
    } else {
        std::cerr << "Ray is parallel to the triangle plane." << std::endl;
    }
}
//高亮所选点
void DICQGLViewer::highlightSelectedPoints(const std::vector<pcl::PointXYZ>& points) {
    if (!points.empty()) {
        glColor3f(1.0f, 0.24f, 0.36f);//亮红色
        glPointSize(5.0f);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < points.size(); ++i) {
            glVertex3f(points[i].x, points[i].y, points[i].z);
        }
        glEnd();
    }
}

//7.19 平面拟合测试
void DICQGLViewer::drawFittedPlane()
{
    if (fitted_plane_coefficients.size() == 4) {
        glColor4f(0.10f, 0.62f, 1.0f, 0.28f);//蓝色半透明
        float a = fitted_plane_coefficients[0];
        float b = fitted_plane_coefficients[1];
        float c = fitted_plane_coefficients[2];
        float d = fitted_plane_coefficients[3];

        // 打印拟合平面的系数
        //std::cout << "Plane coefficients: a=" << a << ", b=" << b << ", c=" << c << ", d=" << d << std::endl;

        glBegin(GL_QUADS);

        // 定义平面的四个角点，确保足够大以避免过度倾斜
        float x1 = -100.0f, y1 = -100.0f;
        float z1 = -(a * x1 + b * y1 + d) / c;

        float x2 = 100.0f, y2 = -100.0f;
        float z2 = -(a * x2 + b * y2 + d) / c;

        float x3 = 100.0f, y3 = 100.0f;
        float z3 = -(a * x3 + b * y3 + d) / c;

        float x4 = -100.0f, y4 = 100.0f;
        float z4 = -(a * x4 + b * y4 + d) / c;

        glVertex3f(x1, y1, z1);
        glVertex3f(x2, y2, z2);
        glVertex3f(x3, y3, z3);
        glVertex3f(x4, y4, z4);
        glEnd();

        //        // 打印顶点坐标
        //        std::cout << "Vertex1: x=" << x1 << ", y=" << y1 << ", z=" << z1 << std::endl;
        //        std::cout << "Vertex2: x=" << x2 << ", y=" << y2 << ", z=" << z2 << std::endl;
        //        std::cout << "Vertex3: x=" << x3 << ", y=" << y3 << ", z=" << z3 << std::endl;
        //        std::cout << "Vertex4: x=" << x4 << ", y=" << y4 << ", z=" << z4 << std::endl;
    }
}
//平面拟合
void DICQGLViewer::fitPlaneToSelection() {
    if (selected_indices.size() < 3) {
        std::cerr << "Not enough points to fit a plane" << std::endl;
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr selected_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int index : selected_indices) {
        selected_cloud->points.push_back(source_cloud->points[index]);
    }

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(selected_cloud);
    seg.segment(*inliers, *coefficients);

    // 打印拟合信息
    std::cout << "Number of selected points: " << selected_indices.size() << std::endl;
    std::cout << "Number of inliers: " << inliers->indices.size() << std::endl;

    if (coefficients->values.size() == 4) {
        fitted_plane_coefficients = {coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]};
        std::cout << "Fitted Plane Coefficients: "
                  << fitted_plane_coefficients[0] << " "
                  << fitted_plane_coefficients[1] << " "
                  << fitted_plane_coefficients[2] << " "
                  << fitted_plane_coefficients[3] << std::endl;

        // 检查平面系数是否有效
        if (std::isnan(fitted_plane_coefficients[0]) || std::isnan(fitted_plane_coefficients[1]) ||
            std::isnan(fitted_plane_coefficients[2]) || std::isnan(fitted_plane_coefficients[3])) {
            std::cerr << "Invalid plane coefficients, cannot draw the plane" << std::endl;
            fitted_plane_coefficients.clear();
        }
    } else {
        std::cerr << "Failed to fit a plane" << std::endl;
        fitted_plane_coefficients.clear();
    }
}

/***********************绘制相关***********************/
//网格绘制
void DICQGLViewer::add_mesh(PCLMesh mesh)
{
    addMesh(std::move(mesh));
}

int DICQGLViewer::addMesh(PCLMesh mesh)
{
    PCLCloud tempcloud;
    pcl::fromPCLPointCloud2(mesh.cloud ,tempcloud);
    mesh_plot.push_back(std::move(mesh));
    mesh_cloud.push_back(std::move(tempcloud));
    mesh_visibility.push_back(true);
    mesh_display_tags.push_back(QString());
    std::cout << mesh_plot.back().polygons.size() << std::endl;
    update();
    return static_cast<int>(mesh_plot.size()) - 1;
}

void DICQGLViewer::setMeshDisplayTag(int index, const QString& tag)
{
    if(index < 0 || index >= static_cast<int>(mesh_plot.size()))
        return;

    while(mesh_display_tags.size() < mesh_plot.size())
        mesh_display_tags.push_back(QString());

    mesh_display_tags[index] = tag.trimmed();
    update();
}

bool DICQGLViewer::removeMesh(int index)
{
    if(index < 0 || index >= static_cast<int>(mesh_plot.size()))
        return false;

    mesh_plot.erase(mesh_plot.begin() + index);
    if(index < static_cast<int>(mesh_cloud.size()))
        mesh_cloud.erase(mesh_cloud.begin() + index);
    if(index < static_cast<int>(mesh_visibility.size()))
        mesh_visibility.erase(mesh_visibility.begin() + index);
    if(index < static_cast<int>(mesh_display_tags.size()))
        mesh_display_tags.erase(mesh_display_tags.begin() + index);
    while(mesh_cloud.size() > mesh_plot.size())
        mesh_cloud.pop_back();
    while(mesh_visibility.size() > mesh_plot.size())
        mesh_visibility.pop_back();
    while(mesh_display_tags.size() > mesh_plot.size())
        mesh_display_tags.pop_back();

    update();
    return true;
}

void DICQGLViewer::draw_mesh()
{
    struct MeshStyle
    {
        float baseR;
        float baseG;
        float baseB;
        float lineR;
        float lineG;
        float lineB;
        float alpha;
        float lineAlpha;
        float lineWidth;
        float ambient;
        float diffuse;
        float specular;
    };

    auto styleForTag = [](const QString& rawTag) -> MeshStyle {
        const QString tag = rawTag.trimmed().toLower();
        if(tag == QStringLiteral("sphere")) {
            return {0.45f, 0.66f, 1.00f, 0.72f, 0.86f, 1.00f, 0.92f, 0.36f, 0.95f, 0.45f, 0.48f, 0.28f};
        }
        if(tag == QStringLiteral("cylinder")) {
            return {0.56f, 0.74f, 1.00f, 0.80f, 0.90f, 1.00f, 0.90f, 0.34f, 0.95f, 0.43f, 0.50f, 0.24f};
        }
        if(tag == QStringLiteral("plane")) {
            return {0.64f, 0.82f, 1.00f, 0.82f, 0.93f, 1.00f, 0.86f, 0.42f, 1.15f, 0.40f, 0.46f, 0.12f};
        }
        if(tag == QStringLiteral("circle3d")) {
            return {0.48f, 0.90f, 0.80f, 0.78f, 0.98f, 0.88f, 0.88f, 0.40f, 1.10f, 0.42f, 0.48f, 0.18f};
        }
        if(tag == QStringLiteral("reconstruction")) {
            return {0.58f, 0.79f, 1.00f, 0.86f, 0.95f, 1.00f, 0.86f, 0.38f, 1.05f, 0.42f, 0.50f, 0.16f};
        }
        return {0.72f, 0.57f, 0.34f, 1.00f, 0.82f, 0.42f, 0.90f, 0.24f, 0.80f, 0.38f, 0.42f, 0.10f};
    };

    GLboolean lightingEnabled = GL_FALSE;
    glGetBooleanv(GL_LIGHTING, &lightingEnabled);
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0f, 1.0f);

    for (int j = 0; j < mesh_plot.size(); ++j)
    {
        if (j < static_cast<int>(mesh_visibility.size()) && !mesh_visibility[j])
            continue;
        if (mesh_cloud[j].points.empty())
            continue;

        while(mesh_display_tags.size() < mesh_plot.size())
            mesh_display_tags.push_back(QString());

        const MeshStyle style = styleForTag(j < static_cast<int>(mesh_display_tags.size()) ? mesh_display_tags[j] : QString());

        float minZ = mesh_cloud[j].points.front().z;
        float maxZ = mesh_cloud[j].points.front().z;
        for (const auto& point : mesh_cloud[j].points) {
            minZ = std::min(minZ, point.z);
            maxZ = std::max(maxZ, point.z);
        }
        const float zRange = std::max(maxZ - minZ, 1.0f);

        glBegin(GL_TRIANGLES);
        for (int i = 0; i < mesh_plot[j].polygons.size(); ++i)
        {
            if (mesh_plot[j].polygons[i].vertices.size() < 3)
                continue;

            const int p0 = mesh_plot[j].polygons[i].vertices[0];
            const int p1 = mesh_plot[j].polygons[i].vertices[1];
            const int p2 = mesh_plot[j].polygons[i].vertices[2];

            if (p0 < 0 || p1 < 0 || p2 < 0 ||
                p0 >= mesh_cloud[j].points.size() ||
                p1 >= mesh_cloud[j].points.size() ||
                p2 >= mesh_cloud[j].points.size()) {
                continue;
            }

            const auto& a = mesh_cloud[j].points[p0];
            const auto& b = mesh_cloud[j].points[p1];
            const auto& c = mesh_cloud[j].points[p2];

            const Eigen::Vector3f va(a.x, a.y, a.z);
            const Eigen::Vector3f vb(b.x, b.y, b.z);
            const Eigen::Vector3f vc(c.x, c.y, c.z);
            Eigen::Vector3f normal = (vb - va).cross(vc - va);
            if (normal.norm() > 1e-6f)
                normal.normalize();
            else
                normal = Eigen::Vector3f(0.0f, 0.0f, 1.0f);

            const Eigen::Vector3f lightDir = Eigen::Vector3f(0.25f, -0.35f, 0.90f).normalized();
            const float diffuseTerm = std::max(0.0f, static_cast<float>(std::fabs(normal.dot(lightDir))));
            const float avgZ = (a.z + b.z + c.z) / 3.0f;
            const float height = (avgZ - minZ) / zRange;
            const float shade = style.ambient + style.diffuse * diffuseTerm + style.specular * (0.35f + 0.65f * height);

            const float r = std::min(style.baseR * shade, 1.0f);
            const float g = std::min(style.baseG * shade, 1.0f);
            const float bcol = std::min(style.baseB * shade, 1.0f);
            glColor4f(r, g, bcol, style.alpha);
            glNormal3f(normal.x(), normal.y(), normal.z());
            glVertex3f(a.x, a.y, a.z);
            glVertex3f(b.x, b.y, b.z);
            glVertex3f(c.x, c.y, c.z);
        }
        glEnd();
    }

    glDisable(GL_POLYGON_OFFSET_FILL);

    for (int j = 0; j < mesh_plot.size(); ++j)
    {
        if (j < static_cast<int>(mesh_visibility.size()) && !mesh_visibility[j])
            continue;

        while(mesh_display_tags.size() < mesh_plot.size())
            mesh_display_tags.push_back(QString());
        const MeshStyle style = styleForTag(j < static_cast<int>(mesh_display_tags.size()) ? mesh_display_tags[j] : QString());

        glLineWidth(style.lineWidth);
        glColor4f(style.lineR, style.lineG, style.lineB, style.lineAlpha);
        glBegin(GL_LINES);
        for (int i = 0; i < mesh_plot[j].polygons.size(); ++i)
        {
            if (mesh_plot[j].polygons[i].vertices.size() < 3)
                continue;

            const int p0 = mesh_plot[j].polygons[i].vertices[0];
            const int p1 = mesh_plot[j].polygons[i].vertices[1];
            const int p2 = mesh_plot[j].polygons[i].vertices[2];

            if (p0 < 0 || p1 < 0 || p2 < 0 ||
                p0 >= mesh_cloud[j].points.size() ||
                p1 >= mesh_cloud[j].points.size() ||
                p2 >= mesh_cloud[j].points.size()) {
                continue;
            }

            const auto& a = mesh_cloud[j].points[p0];
            const auto& b = mesh_cloud[j].points[p1];
            const auto& c = mesh_cloud[j].points[p2];

            glVertex3f(a.x, a.y, a.z); glVertex3f(b.x, b.y, b.z);
            glVertex3f(b.x, b.y, b.z); glVertex3f(c.x, c.y, c.z);
            glVertex3f(c.x, c.y, c.z); glVertex3f(a.x, a.y, a.z);
        }
        glEnd();
    }

    if (lightingEnabled)
        glEnable(GL_LIGHTING);
}

//线段绘制
void DICQGLViewer::add_line(pcl::PointCloud<pcl::PointXYZ>::Ptr &line)
{
    addLine(line);
}

int DICQGLViewer::addLine(const PCLCloud::Ptr& line)
{
    if(!line)
        return -1;

    line_plot.push_back(line);
    line_visibility.push_back(true);
    update();
    return static_cast<int>(line_plot.size()) - 1;
}

int DICQGLViewer::lineCount() const
{
    return static_cast<int>(line_plot.size());
}

bool DICQGLViewer::removeLine(int index)
{
    if(index < 0 || index >= static_cast<int>(line_plot.size()))
        return false;

    line_plot.erase(line_plot.begin() + index);
    if(index < static_cast<int>(line_visibility.size()))
        line_visibility.erase(line_visibility.begin() + index);
    while(line_visibility.size() > line_plot.size())
        line_visibility.pop_back();

    if(selected_line_index == index)
        selected_line_index = -1;
    else if(selected_line_index > index)
        --selected_line_index;

    update();
    return true;
}

void DICQGLViewer::setLineVisible(int index, bool visible)
{
    if(index < 0 || index >= static_cast<int>(line_plot.size()))
        return;

    while(line_visibility.size() < line_plot.size())
        line_visibility.push_back(true);

    line_visibility[index] = visible;
    update();
}

bool DICQGLViewer::lineVisible(int index) const
{
    if(index < 0 || index >= static_cast<int>(line_plot.size()))
        return false;
    if(index >= static_cast<int>(line_visibility.size()))
        return true;
    return line_visibility[index];
}

void DICQGLViewer::setSelectedLineIndex(int index)
{
    if(index < 0 || index >= static_cast<int>(line_plot.size()))
        selected_line_index = -1;
    else
        selected_line_index = index;

    update();
}

int DICQGLViewer::selectedLineIndex() const
{
    return selected_line_index;
}

PCLCloud::Ptr DICQGLViewer::lineByIndex(int index) const
{
    if(index < 0 || index >= static_cast<int>(line_plot.size()))
        return PCLCloud::Ptr(new PCLCloud);
    return line_plot[index];
}

void DICQGLViewer::drawPointCloudLines()
{
    for (int i = 0; i < line_plot.size(); ++i)
    {
        if(i < static_cast<int>(line_visibility.size()) && !line_visibility[i])
            continue;
        if(!line_plot[i] || line_plot[i]->empty())
            continue;

        const bool selected = (i == selected_line_index);
        if(selected)
        {
            glColor3f(1.0f, 0.30f, 0.30f);
            glPointSize(6.0f);
        }
        else
        {
            glColor3f(1.0f, 0.86f, 0.35f);
            glPointSize(4.0f);
        }

        glBegin(GL_POINTS);
        for (const auto& point : line_plot[i]->points) {
            glVertex3f(point.x, point.y, point.z);
        }
        glEnd();

        if(selected)
        {
            glColor3f(1.0f, 0.30f, 0.30f);
            glLineWidth(4.0f);
        }
        else
        {
            glColor3f(1.0f, 0.86f, 0.35f);
            glLineWidth(2.0f);
        }

        glBegin(GL_LINE_STRIP);
        for (const auto& point : line_plot[i]->points) {
            glVertex3f(point.x, point.y, point.z);
        }
        glEnd();
    }
}

//点云绘制
void DICQGLViewer::add_pointcloud(PCLCloud cloud)
{
    const int newIndex = static_cast<int>(pointclouds_plot.size());
    pointclouds_plot.push_back(cloud);
    pointcloud_colors.push_back(defaultPointCloudColor(newIndex));
    pointcloud_visibility.push_back(true);
    pointcloud_normals.push_back(nullptr);

    // 只有第 1/2 片导入点云作为配准用 source/target 缓存；后续显示点云不覆盖配准数据。
    if(!source_cloud)
        source_cloud.reset(new PCLCloud);
    if(!target_cloud)
        target_cloud.reset(new PCLCloud);
    if(newIndex == 0)
        *source_cloud = cloud;
    else if(newIndex == 1)
        *target_cloud = cloud;

    update();
}

int DICQGLViewer::pointCloudCount() const
{
    return static_cast<int>(pointclouds_plot.size());
}

int DICQGLViewer::selectedPointCloudIndex() const
{
    return selected_pointcloud_index;
}

const std::vector<int>& DICQGLViewer::selectedIndices() const
{
    return selected_indices;
}

QColor DICQGLViewer::defaultPointCloudColor(int index) const
{
    static const QColor defaultColors[] = {
        QColor(92, 214, 255),   // 青蓝：默认源点云
        QColor(255, 184, 71),   // 橙色：默认目标点云
        QColor(199, 143, 255),  // 紫色
        QColor(56, 209, 145),   // 绿色
        QColor(255, 120, 168),  // 粉色
        QColor(242, 230, 92),   // 黄色
        QColor(133, 189, 255)   // 淡蓝
    };
    const int colorCount = static_cast<int>(sizeof(defaultColors) / sizeof(defaultColors[0]));
    if(index < 0)
        index = 0;
    return defaultColors[index % colorCount];
}

QColor DICQGLViewer::colorForPointCloud(int index) const
{
    if(index >= 0 && index < static_cast<int>(pointcloud_colors.size()) && pointcloud_colors[index].isValid())
        return pointcloud_colors[index];

    return defaultPointCloudColor(index);
}

QColor DICQGLViewer::pointCloudColor(int index) const
{
    return colorForPointCloud(index);
}

void DICQGLViewer::setMultiCornerSelectionEnabled(bool enabled)
{
    multi_corner_selection_enabled = enabled;
    selecting = false;
    right_dragging = false;
    multi_corner_vertices.clear();
    multi_corner_hover_pos = QPoint(width() / 2, height() / 2);
    if(enabled)
        selected_points_cache.clear();
    update();
}

void DICQGLViewer::setSelectedPointCloudIndex(int index)
{
    if(index < 0 || index >= static_cast<int>(pointclouds_plot.size()))
        selected_pointcloud_index = -1;
    else
        selected_pointcloud_index = index;

    selected_pointcloud_indices.clear();
    if(selected_pointcloud_index >= 0)
        selected_pointcloud_indices.push_back(selected_pointcloud_index);

    update();
}

void DICQGLViewer::setSelectedPointCloudIndices(const std::vector<int>& indices)
{
    selected_pointcloud_indices.clear();
    for(const int idx : indices)
    {
        if(idx >= 0 && idx < static_cast<int>(pointclouds_plot.size()))
            selected_pointcloud_indices.push_back(idx);
    }

    if(!selected_pointcloud_indices.empty())
        selected_pointcloud_index = selected_pointcloud_indices.back();
    else
        selected_pointcloud_index = -1;

    update();
}

void DICQGLViewer::setPointCloudColor(int index, const QColor& color)
{
    if(index < 0 || index >= static_cast<int>(pointclouds_plot.size()))
        return;

    while(pointcloud_colors.size() < pointclouds_plot.size())
        pointcloud_colors.push_back(defaultPointCloudColor(static_cast<int>(pointcloud_colors.size())));

    pointcloud_colors[index] = color.isValid() ? color : defaultPointCloudColor(index);
    update();
}

void DICQGLViewer::removePointCloud(int index)
{
    if(index < 0 || index >= static_cast<int>(pointclouds_plot.size()))
        return;

    pointclouds_plot.erase(pointclouds_plot.begin() + index);
    if(index < static_cast<int>(pointcloud_colors.size()))
        pointcloud_colors.erase(pointcloud_colors.begin() + index);
    if(index < static_cast<int>(pointcloud_visibility.size()))
        pointcloud_visibility.erase(pointcloud_visibility.begin() + index);
    if(index < static_cast<int>(pointcloud_normals.size()))
        pointcloud_normals.erase(pointcloud_normals.begin() + index);
    while(pointcloud_colors.size() > pointclouds_plot.size())
        pointcloud_colors.pop_back();
    while(pointcloud_visibility.size() > pointclouds_plot.size())
        pointcloud_visibility.pop_back();
    while(pointcloud_normals.size() > pointclouds_plot.size())
        pointcloud_normals.pop_back();

    if(selected_pointcloud_index == index)
        selected_pointcloud_index = -1;
    else if(selected_pointcloud_index > index)
        --selected_pointcloud_index;

    if(index == 0)
        selected_source_points.clear();
    if(index <= 1)
        selected_target_points.clear();
    selected_indices.clear();
    selected_points_cache.clear();

    syncPrimaryPointCloudCache();
    update();
}

int DICQGLViewer::removeAreaSelectedPoints()
{
    if(selected_indices.empty())
        return 0;

    std::unordered_set<int> selectedSet(selected_indices.begin(), selected_indices.end());
    int globalVisibleIndex = 0;
    int removedCount = 0;

    for (int cloudIndex = 0; cloudIndex < static_cast<int>(pointclouds_plot.size()); ++cloudIndex)
    {
        if (cloudIndex < static_cast<int>(pointcloud_visibility.size()) && !pointcloud_visibility[cloudIndex])
            continue;

        PCLCloud& cloud = pointclouds_plot[cloudIndex];
        PCLCloud filtered;
        filtered.points.reserve(cloud.points.size());

        for (const auto& p : cloud.points)
        {
            if(selectedSet.find(globalVisibleIndex) == selectedSet.end())
                filtered.points.push_back(p);
            else
                ++removedCount;
            ++globalVisibleIndex;
        }

        filtered.width = static_cast<uint32_t>(filtered.points.size());
        filtered.height = filtered.points.empty() ? 0 : 1;
        filtered.is_dense = cloud.is_dense;
        cloud.swap(filtered);
    }

    selected_indices.clear();
    selected_points_cache.clear();
    syncPrimaryPointCloudCache();
    update();
    return removedCount;
}

int DICQGLViewer::removeAreaUnselectedPoints()
{
    if(selected_indices.empty())
        return 0;

    std::unordered_set<int> selectedSet(selected_indices.begin(), selected_indices.end());
    int globalVisibleIndex = 0;
    int removedCount = 0;

    for (int cloudIndex = 0; cloudIndex < static_cast<int>(pointclouds_plot.size()); ++cloudIndex)
    {
        if (cloudIndex < static_cast<int>(pointcloud_visibility.size()) && !pointcloud_visibility[cloudIndex])
            continue;

        PCLCloud& cloud = pointclouds_plot[cloudIndex];
        PCLCloud filtered;
        filtered.points.reserve(cloud.points.size());

        for (const auto& p : cloud.points)
        {
            if(selectedSet.find(globalVisibleIndex) != selectedSet.end())
                filtered.points.push_back(p);
            else
                ++removedCount;
            ++globalVisibleIndex;
        }

        filtered.width = static_cast<uint32_t>(filtered.points.size());
        filtered.height = filtered.points.empty() ? 0 : 1;
        filtered.is_dense = cloud.is_dense;
        cloud.swap(filtered);
    }

    selected_indices.clear();
    selected_points_cache.clear();
    syncPrimaryPointCloudCache();
    update();
    return removedCount;
}

void DICQGLViewer::syncPrimaryPointCloudCache()
{
    if(!source_cloud)
        source_cloud.reset(new PCLCloud);
    if(!target_cloud)
        target_cloud.reset(new PCLCloud);

    source_cloud->clear();
    target_cloud->clear();

    if(!pointclouds_plot.empty())
        *source_cloud = pointclouds_plot[0];
    if(pointclouds_plot.size() > 1)
        *target_cloud = pointclouds_plot[1];
}

void DICQGLViewer::drawSelectedPointCloudHighlight()
{
    if(selected_pointcloud_indices.empty() &&
       (selected_pointcloud_index < 0 || selected_pointcloud_index >= static_cast<int>(pointclouds_plot.size())))
        return;

    std::vector<int> highlightIndices = selected_pointcloud_indices;
    if(highlightIndices.empty() && selected_pointcloud_index >= 0)
        highlightIndices.push_back(selected_pointcloud_index);

    GLboolean depthTestEnabled = GL_FALSE;
    glGetBooleanv(GL_DEPTH_TEST, &depthTestEnabled);
    glDisable(GL_DEPTH_TEST);

    for(const int cloudIndex : highlightIndices)
    {
        if(cloudIndex < 0 || cloudIndex >= static_cast<int>(pointclouds_plot.size()))
            continue;
        if(cloudIndex < static_cast<int>(pointcloud_visibility.size()) && !pointcloud_visibility[cloudIndex])
            continue;

        const PCLCloud& cloud = pointclouds_plot[cloudIndex];
        if(cloud.points.empty())
            continue;

        const QColor cloudColor = colorForPointCloud(cloudIndex);
        glColor4f(cloudColor.redF(), cloudColor.greenF(), cloudColor.blueF(), 0.95f);
        glPointSize(4.5f);
        glBegin(GL_POINTS);
        for(const auto& point : cloud.points)
            glVertex3f(point.x, point.y, point.z);
        glEnd();

        pcl::PointXYZ minPoint = cloud.points.front();
        pcl::PointXYZ maxPoint = cloud.points.front();
        for(const auto& point : cloud.points)
        {
            minPoint.x = std::min(minPoint.x, point.x);
            minPoint.y = std::min(minPoint.y, point.y);
            minPoint.z = std::min(minPoint.z, point.z);
            maxPoint.x = std::max(maxPoint.x, point.x);
            maxPoint.y = std::max(maxPoint.y, point.y);
            maxPoint.z = std::max(maxPoint.z, point.z);
        }

        glColor4f(1.0f, 0.86f, 0.20f, 0.95f);
        glLineWidth(2.0f);
        glBegin(GL_LINES);
        glVertex3f(minPoint.x, minPoint.y, minPoint.z); glVertex3f(maxPoint.x, minPoint.y, minPoint.z);
        glVertex3f(maxPoint.x, minPoint.y, minPoint.z); glVertex3f(maxPoint.x, maxPoint.y, minPoint.z);
        glVertex3f(maxPoint.x, maxPoint.y, minPoint.z); glVertex3f(minPoint.x, maxPoint.y, minPoint.z);
        glVertex3f(minPoint.x, maxPoint.y, minPoint.z); glVertex3f(minPoint.x, minPoint.y, minPoint.z);

        glVertex3f(minPoint.x, minPoint.y, maxPoint.z); glVertex3f(maxPoint.x, minPoint.y, maxPoint.z);
        glVertex3f(maxPoint.x, minPoint.y, maxPoint.z); glVertex3f(maxPoint.x, maxPoint.y, maxPoint.z);
        glVertex3f(maxPoint.x, maxPoint.y, maxPoint.z); glVertex3f(minPoint.x, maxPoint.y, maxPoint.z);
        glVertex3f(minPoint.x, maxPoint.y, maxPoint.z); glVertex3f(minPoint.x, minPoint.y, maxPoint.z);

        glVertex3f(minPoint.x, minPoint.y, minPoint.z); glVertex3f(minPoint.x, minPoint.y, maxPoint.z);
        glVertex3f(maxPoint.x, minPoint.y, minPoint.z); glVertex3f(maxPoint.x, minPoint.y, maxPoint.z);
        glVertex3f(maxPoint.x, maxPoint.y, minPoint.z); glVertex3f(maxPoint.x, maxPoint.y, maxPoint.z);
        glVertex3f(minPoint.x, maxPoint.y, minPoint.z); glVertex3f(minPoint.x, maxPoint.y, maxPoint.z);
        glEnd();
    }

    if(depthTestEnabled)
        glEnable(GL_DEPTH_TEST);
}

void DICQGLViewer::setPointCloudVisible(int index, bool visible)
{
    if(index < 0 || index >= static_cast<int>(pointclouds_plot.size()))
        return;

    while(pointcloud_visibility.size() < pointclouds_plot.size())
        pointcloud_visibility.push_back(true);

    pointcloud_visibility[index] = visible;
    update();
}

bool DICQGLViewer::pointCloudVisible(int index) const
{
    if(index < 0 || index >= static_cast<int>(pointclouds_plot.size()))
        return false;
    if(index >= static_cast<int>(pointcloud_visibility.size()))
        return true;
    return pointcloud_visibility[index];
}

int DICQGLViewer::meshCount() const
{
    return static_cast<int>(mesh_plot.size());
}

void DICQGLViewer::setMeshVisible(int index, bool visible)
{
    if(index < 0 || index >= static_cast<int>(mesh_plot.size()))
        return;

    while(mesh_visibility.size() < mesh_plot.size())
        mesh_visibility.push_back(true);

    mesh_visibility[index] = visible;
    update();
}

bool DICQGLViewer::meshVisible(int index) const
{
    if(index < 0 || index >= static_cast<int>(mesh_plot.size()))
        return false;
    if(index >= static_cast<int>(mesh_visibility.size()))
        return true;
    return mesh_visibility[index];
}

PCLMesh DICQGLViewer::meshByIndex(int index) const
{
    if(index < 0 || index >= static_cast<int>(mesh_plot.size()))
        return PCLMesh();
    return mesh_plot[index];
}

PCLCloud::Ptr DICQGLViewer::selectedPointsCloud() const
{
    PCLCloud::Ptr cloud(new PCLCloud);
    cloud->points.assign(selected_points_cache.begin(), selected_points_cache.end());
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
}

PCLCloud::Ptr DICQGLViewer::mergedPointCloudByIndices(const std::vector<int>& indices, bool onlyVisible) const
{
    PCLCloud::Ptr merged(new PCLCloud);
    for (int index : indices)
    {
        if(index < 0 || index >= static_cast<int>(pointclouds_plot.size()))
            continue;
        if(onlyVisible && index < static_cast<int>(pointcloud_visibility.size()) && !pointcloud_visibility[index])
            continue;

        const PCLCloud& cloud = pointclouds_plot[index];
        merged->points.insert(merged->points.end(), cloud.points.begin(), cloud.points.end());
    }

    merged->width = static_cast<uint32_t>(merged->points.size());
    merged->height = 1;
    merged->is_dense = false;
    return merged;
}

PCLCloud::Ptr DICQGLViewer::pointCloudByIndex(int index) const
{
    if(index < 0 || index >= static_cast<int>(pointclouds_plot.size()))
        return PCLCloud::Ptr(new PCLCloud);

    return PCLCloud::Ptr(new PCLCloud(pointclouds_plot[index]));
}

bool DICQGLViewer::replacePointCloud(int index, const PCLCloud& cloud)
{
    if(index < 0 || index >= static_cast<int>(pointclouds_plot.size()))
        return false;

    pointclouds_plot[index] = cloud;
    selected_indices.clear();
    selected_points_cache.clear();
    syncPrimaryPointCloudCache();
    // 替换后清除旧法向，新点云需要重新估计
    if(index < static_cast<int>(pointcloud_normals.size()))
        pointcloud_normals[index] = nullptr;
    update();
    return true;
}

void DICQGLViewer::setPointCloudNormals(int index, const pcl::PointCloud<pcl::Normal>::Ptr& normals)
{
    if(index < 0 || index >= static_cast<int>(pointclouds_plot.size()))
        return;
    while(pointcloud_normals.size() < pointclouds_plot.size())
        pointcloud_normals.push_back(nullptr);
    pointcloud_normals[index] = normals;
}

pcl::PointCloud<pcl::Normal>::Ptr DICQGLViewer::pointCloudNormals(int index) const
{
    if(index < 0 || index >= static_cast<int>(pointcloud_normals.size()))
        return nullptr;
    return pointcloud_normals[index];
}

bool DICQGLViewer::hasConsistentNormals(int index) const
{
    if(index < 0 || index >= static_cast<int>(pointcloud_normals.size()))
        return false;
    return pointcloud_normals[index] != nullptr && !pointcloud_normals[index]->empty();
}

void DICQGLViewer::enterPCASliceMode(const Eigen::Vector3f& pc1Dir, const Eigen::Vector3f& origin,
                                       float tMin, float tMax,
                                       const std::vector<float>& slicePositions,
                                       int activeSlice, float sliceHalfThickness)
{
    pca_slicing_active = true;
    pca_slice_dragging = false;
    pca_pc1_dir = pc1Dir.normalized();
    pca_origin = origin;
    pca_tMin = tMin;
    pca_tMax = tMax;
    pca_numSlices = static_cast<int>(slicePositions.size());
    pca_slice_positions = slicePositions;
    pca_active_slice = std::max(0, std::min(activeSlice, pca_numSlices - 1));
    pca_halfThickness = sliceHalfThickness;
}

void DICQGLViewer::exitPCASliceMode()
{
    pca_slicing_active = false;
    pca_slice_dragging = false;
}

void DICQGLViewer::setActiveSlicePosition(float t)
{
    if(pca_active_slice >= 0 && pca_active_slice < static_cast<int>(pca_slice_positions.size()))
    {
        pca_slice_positions[pca_active_slice] = t;
        update();
    }
}

void DICQGLViewer::setActiveSliceIndex(int idx)
{
    if(idx >= 0 && idx < static_cast<int>(pca_slice_positions.size()))
    {
        pca_active_slice = idx;
        update();
    }
}

int DICQGLViewer::findSliceAtPixel(const QPoint& pixel) const
{
    if(pca_slice_positions.empty()) return -1;

    qglviewer::Vec orig, dir;
    camera()->convertClickToLine(pixel, orig, dir);

    const Eigen::Vector3f rayOrig(orig[0], orig[1], orig[2]);
    const Eigen::Vector3f rayDir(dir[0], dir[1], dir[2]);
    const Eigen::Vector3f planeNormal = pca_pc1_dir;

    const Eigen::Vector3f ref = (std::abs(planeNormal.z()) < 0.9f)
        ? Eigen::Vector3f::UnitZ() : Eigen::Vector3f::UnitX();
    const Eigen::Vector3f pc2 = planeNormal.cross(ref).normalized();
    const Eigen::Vector3f pc3 = planeNormal.cross(pc2).normalized();
    const float hw = pca_pc2Max;
    const float hh = pca_pc3Max;

    int bestSlice = -1;
    float bestDist = std::numeric_limits<float>::max();

    for(int i = 0; i < static_cast<int>(pca_slice_positions.size()); ++i)
    {
        const Eigen::Vector3f planePoint = pca_origin + planeNormal * pca_slice_positions[i];

        const float denom = rayDir.dot(planeNormal);
        if(std::abs(denom) < 1e-6f) continue;

        const float t = (planePoint - rayOrig).dot(planeNormal) / denom;
        if(t < 0.0f) continue;

        const Eigen::Vector3f hitPoint = rayOrig + rayDir * t;
        const Eigen::Vector3f local = hitPoint - planePoint;
        if(std::abs(local.dot(pc2)) <= hw && std::abs(local.dot(pc3)) <= hh)
        {
            if(t < bestDist) { bestDist = t; bestSlice = i; }
        }
    }
    return bestSlice;
}

void DICQGLViewer::drawPCASlicePlanes()
{
    const Eigen::Vector3f& pc1 = pca_pc1_dir;
    const Eigen::Vector3f  o  = pca_origin;

    // 构建 PC2/PC3 方向：取与 pc1 正交的两个方向
    const Eigen::Vector3f ref = (std::abs(pc1.z()) < 0.9f)
        ? Eigen::Vector3f::UnitZ() : Eigen::Vector3f::UnitX();
    const Eigen::Vector3f pc2 = pc1.cross(ref).normalized();
    const Eigen::Vector3f pc3 = pc1.cross(pc2).normalized();
    const float hw = pca_pc2Max;
    const float hh = pca_pc3Max;

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    auto planeVtx = [&](const Eigen::Vector3f& c, float su, float sv) -> Eigen::Vector3f {
        return c + pc2 * su + pc3 * sv;
    };

    const int nSlices = static_cast<int>(pca_slice_positions.size());
    for(int i = 0; i < nSlices; ++i)
    {
        const float t = pca_slice_positions[i];
        const Eigen::Vector3f center = o + pc1 * t;
        const bool isActive = (i == pca_active_slice);

        const Eigen::Vector3f v00 = planeVtx(center, -hw, -hh);
        const Eigen::Vector3f v10 = planeVtx(center,  hw, -hh);
        const Eigen::Vector3f v11 = planeVtx(center,  hw,  hh);
        const Eigen::Vector3f v01 = planeVtx(center, -hw,  hh);

        if(isActive)
        {
            // Active slice: brighter fill + thick outline
            glColor4f(0.1f, 0.5f, 0.9f, 0.55f);
            glBegin(GL_QUADS);
            glVertex3f(v00.x(), v00.y(), v00.z());
            glVertex3f(v10.x(), v10.y(), v10.z());
            glVertex3f(v11.x(), v11.y(), v11.z());
            glVertex3f(v01.x(), v01.y(), v01.z());
            glEnd();

            glLineWidth(3.0f);
            glColor4f(0.1f, 0.5f, 0.9f, 0.9f);
            glBegin(GL_LINE_LOOP);
            glVertex3f(v00.x(), v00.y(), v00.z());
            glVertex3f(v10.x(), v10.y(), v10.z());
            glVertex3f(v11.x(), v11.y(), v11.z());
            glVertex3f(v01.x(), v01.y(), v01.z());
            glEnd();
        }
        else
        {
            // Inactive slices: lighter fill + thin outline, still visible
            glColor4f(0.25f, 0.65f, 0.95f, 0.35f);
            glBegin(GL_QUADS);
            glVertex3f(v00.x(), v00.y(), v00.z());
            glVertex3f(v10.x(), v10.y(), v10.z());
            glVertex3f(v11.x(), v11.y(), v11.z());
            glVertex3f(v01.x(), v01.y(), v01.z());
            glEnd();

            glLineWidth(1.2f);
            glColor4f(0.35f, 0.65f, 0.9f, 0.55f);
            glBegin(GL_LINE_LOOP);
            glVertex3f(v00.x(), v00.y(), v00.z());
            glVertex3f(v10.x(), v10.y(), v10.z());
            glVertex3f(v11.x(), v11.y(), v11.z());
            glVertex3f(v01.x(), v01.y(), v01.z());
            glEnd();
        }
    }
}

void DICQGLViewer::draw_pointcloud()
{
    glPointSize(pointsize);
    glBegin(GL_POINTS);
    for (int j = 0; j < pointclouds_plot.size(); ++j)
    {
        int pcCount = pointclouds_plot[j].size();
        for (int i = 0; i < pcCount; i++)
        {
            qglviewer::Vec Pt;
            Pt.x = pointclouds_plot[j].points[i].x;
            Pt.y = pointclouds_plot[j].points[i].y;
            Pt.z = pointclouds_plot[j].points[i].z;
            glColor3f(0.36f, 0.84f, 1.00f);
            glVertex3fv(Pt);
        }
    }
    for (int j = 0; j < rgbpointclouds_plot.size(); ++j)
    {
        int pcCount = rgbpointclouds_plot[j].size();
        for (int i = 0; i < pcCount; ++i)
        {
            qglviewer::Vec Pt;
            Pt.x = rgbpointclouds_plot[j].points[i].x;
            Pt.y = rgbpointclouds_plot[j].points[i].y;
            Pt.z = rgbpointclouds_plot[j].points[i].z;
            float r=rgbpointclouds_plot[j].points[i].r/255.0;
            float g=rgbpointclouds_plot[j].points[i].g/255.0;
            float b=rgbpointclouds_plot[j].points[i].b/255.0;
            glColor3f(r,g,b);
            glVertex3fv(Pt);
        }
    }
    glEnd();
}

//平面绘制
void DICQGLViewer::add_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &plane)
{
    plane_plot.push_back(plane);
    update();
}
void DICQGLViewer::drawPointCloudPlanes()
{
    for (const auto &plane : plane_plot)
    {
        if (plane->points.size() < 3)
        {
            continue; // 确保有足够的点来绘制平面
        }
        glColor4f(0.22f, 0.82f, 0.57f, 0.45f);//绿色半透明
        glPointSize(3.0f);
        glBegin(GL_POINTS);
        for (const auto &point : plane->points)
        {
            glVertex3f(point.x, point.y, point.z);
        }
        glEnd();
        glColor4f(0.22f, 0.82f, 0.57f, 0.28f);
        glLineWidth(1.5f);
        glBegin(GL_POLYGON);
        for (const auto &point : plane->points)
        {
            glVertex3f(point.x, point.y, point.z);
        }
        glEnd();
    }
}

//坐标轴绘制（3-2-1）
void DICQGLViewer::add_axes(const Eigen::Matrix4f &transform)
{
    axes_transforms.push_back(transform);
}
void DICQGLViewer::draw_axes()
{
    for (const auto &transform : axes_transforms)
    {
        // 从变换矩阵中提取旋转和平移部分
        Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
        Eigen::Vector3f translation = transform.block<3, 1>(0, 3);
        // 设置坐标轴的长度
        float axisLength = 200.0f;
        float arrowLength = 10.0f;
        float arrowWidth = 5.0f;
        // 定义局部坐标系中的坐标轴端点
        Eigen::Vector3f xAxis = rotation * Eigen::Vector3f(axisLength, 0, 0) + translation;
        Eigen::Vector3f yAxis = rotation * Eigen::Vector3f(0, axisLength, 0) + translation;
        Eigen::Vector3f zAxis = rotation * Eigen::Vector3f(0, 0, axisLength) + translation;
        glLineWidth(2.0f);
        // 绘制X轴（红色）
        glColor3f(1.0f, 0.31f, 0.37f);
        glBegin(GL_LINES);
        glVertex3f(translation.x(), translation.y(), translation.z());
        glVertex3f(xAxis.x(), xAxis.y(), xAxis.z());
        glEnd();
        // 绘制X轴箭头
        Eigen::Vector3f xArrow1 = rotation * Eigen::Vector3f(axisLength - arrowLength, arrowWidth, 0) + translation;
        Eigen::Vector3f xArrow2 = rotation * Eigen::Vector3f(axisLength - arrowLength, -arrowWidth, 0) + translation;
        glBegin(GL_TRIANGLES);
        glVertex3f(xAxis.x(), xAxis.y(), xAxis.z());
        glVertex3f(xArrow1.x(), xArrow1.y(), xArrow1.z());
        glVertex3f(xArrow2.x(), xArrow2.y(), xArrow2.z());
        glEnd();
        // 绘制Y轴（绿色）
        glColor3f(0.22f, 0.82f, 0.57f);
        glBegin(GL_LINES);
        glVertex3f(translation.x(), translation.y(), translation.z());
        glVertex3f(yAxis.x(), yAxis.y(), yAxis.z());
        glEnd();
        // 绘制Y轴箭头
        Eigen::Vector3f yArrow1 = rotation * Eigen::Vector3f(arrowWidth, axisLength - arrowLength, 0) + translation;
        Eigen::Vector3f yArrow2 = rotation * Eigen::Vector3f(-arrowWidth, axisLength - arrowLength, 0) + translation;
        glBegin(GL_TRIANGLES);
        glVertex3f(yAxis.x(), yAxis.y(), yAxis.z());
        glVertex3f(yArrow1.x(), yArrow1.y(), yArrow1.z());
        glVertex3f(yArrow2.x(), yArrow2.y(), yArrow2.z());
        glEnd();
        // 绘制Z轴（蓝色）
        glColor3f(0.22f, 0.59f, 1.0f);
        glBegin(GL_LINES);
        glVertex3f(translation.x(), translation.y(), translation.z());
        glVertex3f(zAxis.x(), zAxis.y(), zAxis.z());
        glEnd();
        // 绘制Z轴箭头
        Eigen::Vector3f zArrow1 = rotation * Eigen::Vector3f(arrowWidth, 0, axisLength - arrowLength) + translation;
        Eigen::Vector3f zArrow2 = rotation * Eigen::Vector3f(-arrowWidth, 0, axisLength - arrowLength) + translation;
        glBegin(GL_TRIANGLES);
        glVertex3f(zAxis.x(), zAxis.y(), zAxis.z());
        glVertex3f(zArrow1.x(), zArrow1.y(), zArrow1.z());
        glVertex3f(zArrow2.x(), zArrow2.y(), zArrow2.z());
        glEnd();
    }
}
