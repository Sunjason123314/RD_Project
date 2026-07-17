#include "gl_image.h"

#include <QDebug>
#include <QPainter>
#include <iostream>

using namespace std;

GL_Image::GL_Image(QWidget* parent):
    QOpenGLWidget(parent)
{
//    makeCurrent();
    imageData_ = nullptr;
    setAutoFillBackground(false);
    position=QPoint(0,0);
    imageSize_=QSize(5120,5120);
//    doneCurrent();
}

GL_Image::~GL_Image()
{
    makeCurrent();
    texture_->destroy();
    doneCurrent();
}

// 设置待显示的数据源
void GL_Image::setImageData(uchar* imageSrc, uint width, uint height)
{
    imageData_ = imageSrc;
    imageSize_.setWidth(width);
    imageSize_.setHeight(height);
    update();
}

void GL_Image::setImageData(const QImage* img)
{
//    m_img = img;
    imageData_ = (uchar*)img->bits();
    imageSize_.setWidth(img->width());
    imageSize_.setHeight(img->height());
    zoomsize.setWidth(zoom*img->width());
    zoomsize.setHeight(zoom*img->height());
    update();
}

void GL_Image::setImageSize(QSize size)
{
    imageSize_.setWidth(size.width());
    imageSize_.setHeight(size.height());
    zoomsize.setWidth(zoom*size.width());
    zoomsize.setHeight(zoom*size.height());
    update();
}

void GL_Image::Clear()
{
    imageSize_=QSize(0,0);
    imageData_=nullptr;
    initTextureFlag=false;
    update();
}

void GL_Image::refresh(BYTE *pBuffer)
{
    imageData_=(uchar*) pBuffer;
    update();
}

void GL_Image::initializeGL()
{
    makeCurrent();
    initializeOpenGLFunctions();
    texture_ = new QOpenGLTexture(QOpenGLTexture::Target2D);

    texture_->create();
    textureId_ = texture_->textureId();
    glBindTexture(GL_TEXTURE_2D, textureId_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    doneCurrent();
}

// cpu:4% gpu:10%
void GL_Image::paintGL()
{
    makeCurrent();
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if(imageData_ == nullptr){
        return;
    }

    //QByteArray ba((const char*)imageData_,64);
    //qDebug() <<imageData_ << ba;
    glBindTexture(GL_TEXTURE_2D, textureId_);

    if(!initTextureFlag)
    {
        // 首次显示纹理GL_ALPHA  GL_LUMINANCE
        glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, imageSize_.width(), imageSize_.height(), 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, imageData_);
        initTextureFlag = true;
    }
    else
    {
        // 动态修改纹理数据
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, imageSize_.width(), imageSize_.height(), GL_LUMINANCE, GL_UNSIGNED_BYTE, imageData_);
        //glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, imageSize_.width(), imageSize_.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, imageData_);
    }

    glEnable(GL_TEXTURE_2D);
    glBegin(GL_POLYGON);
    //顶点坐标和纹理坐标必须一一对应
    glTexCoord2d(0.0f, 0.0f);
    glVertex2d(x-zoomsize.width()/2, y-zoomsize.height()/2);
    glTexCoord2d(0.0f, 1.0f);
    glVertex2d(x-zoomsize.width()/2, zoomsize.height()/2+y);
    glTexCoord2d(1.0f, 1.0f);
    glVertex2d(zoomsize.width()/2+x, zoomsize.height()/2+y);
    glTexCoord2d(1.0f, 0.0f);
    glVertex2d(zoomsize.width()/2+x, y-zoomsize.height()/2);
    glEnd();
    glDisable(GL_TEXTURE_2D);

    doneCurrent();
}

void GL_Image::resizeGL(int w, int h)
{
    makeCurrent();
    Ortho2DSize_.setWidth(w);
    Ortho2DSize_.setHeight(h);
    glViewport(0,0, w, h);
    glMatrixMode(GL_PROJECTION);//将后续矩阵操作应用于投影矩阵堆栈。
    glLoadIdentity();
    glOrtho(0, Ortho2DSize_.width(), Ortho2DSize_.height(), 0, -1, 1);
    glMatrixMode(GL_MODELVIEW);//将后续矩阵操作应用于 modelview 矩阵堆栈。
    doneCurrent();
}

void GL_Image::wheelEvent(QWheelEvent *event)
{
    if(event->angleDelta().y() > 0)
    {
        zoom += zommrate;
    }
    else
    {
        zoom -= zommrate;
    }
    if(zoom >= maxzoom)
    {
        zoom = maxzoom;
    }
    else if(zoom <= minzoom)
    {
        zoom = minzoom;
    }
    zoomsize.setHeight(imageSize_.height()*zoom);
    zoomsize.setWidth(imageSize_.width()*zoom);

    update();

}

void GL_Image::mousePressEvent(QMouseEvent *event)
{
    drag=true;
    mousePressPosition=event->pos();
    x_=x;
    y_=y;
}

void GL_Image::mouseMoveEvent(QMouseEvent *event)
{
    if(drag==false)
    {
        return;
    }
    x=x_+event->pos().x()-mousePressPosition.x();
    y=y_+event->pos().y()-mousePressPosition.y();
    update();
}

void GL_Image::mouseReleaseEvent(QMouseEvent *event)
{
    drag=false;
}
