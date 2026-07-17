#ifndef GL_IMAGE_H
#define GL_IMAGE_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLTexture>
#include <QWheelEvent>
#include <QMouseEvent>

class GL_Image : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:

    GL_Image(QWidget* parent = nullptr);
    ~GL_Image();

    // 设置实时显示的数据源
    void setImageData(uchar* imageSrc, uint width, uint height);
    void setImageData(const QImage* img);

    //设置图片尺寸（不设置默认为0，无法显示）
    void setImageSize(QSize size);

    //清除尺寸和图像指针（关闭相机前不清除会导致闪退）
    void Clear();

public slots:
    void refresh(BYTE* pBuffer);

protected:
    // 重写QGLWidget类的接口
    void initializeGL();
    void paintGL();
    void resizeGL(int w, int h);

    //缩放操作
    double zoom=1;//缩放率
    double zommrate=0.05;//缩放变化速率
    double maxzoom=3;//最大缩放率
    double minzoom=0.05;//最小缩放率
    int size_w;
    int size_h;
    QSize zoomsize;            //图片尺寸
    void wheelEvent(QWheelEvent *event);

    //平移操作
    QPoint position;
    int x=0;//平移的x坐标
    int y=0;//平移的y坐标
    int x_=0;//拖拽记录的x坐标
    int y_=0;//拖拽记录的y坐标
    bool drag=false;//记录鼠标是否按下
    QPoint mousePressPosition;////记录鼠标按下时的坐标位置
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

private:
    uchar* imageData_;           //纹理显示的数据源
    QSize imageSize_;            //图片尺寸
    QSize Ortho2DSize_;          //窗口尺寸
    QOpenGLTexture* texture_;
    GLuint textureId_;           //纹理对象ID

    bool initTextureFlag=false;

};


#endif // GL_IMAGE_H
