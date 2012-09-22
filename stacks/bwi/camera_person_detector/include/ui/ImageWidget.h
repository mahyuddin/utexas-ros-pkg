#ifndef IMAGE_WIDGET_H
#define IMAGE_WIDGET_H

#include <QtGui>
#include <QWidget>
#include <QImage>

#include <opencv/cv.h>

class QPaintEvent;

class ImageWidget : public QWidget {
  Q_OBJECT

  public:
    ImageWidget(QWidget*);
    void setImageSize(int,int);
    void paintEvent(QPaintEvent*);
    void setImage(const cv::Mat&);
    
  private:
    QImage _img;
};

#endif
