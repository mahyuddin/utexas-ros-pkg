#include "ImageWidget.h"

ImageWidget::ImageWidget(QWidget *parent) : QWidget(parent) {
    setImageSize(640, 480);
}

void ImageWidget::setImageSize(int width, int height){
    _img = QImage(width, height,QImage::Format_RGB32);
    _img.fill(Qt::black);
}

void ImageWidget::paintEvent(QPaintEvent *event) {
    QPainter painter;
    painter.begin(this);
    painter.drawImage(event->rect(), _img);
    painter.end();
}

void ImageWidget::setImage(const cv::Mat &src) {
  QImage dest(src.cols, src.rows, QImage::Format_ARGB32);
  for (int y = 0; y < src.rows; ++y) {
    QRgb *destrow = (QRgb*)dest.scanLine(y);
    for(int x = 0; x < src.cols; ++x) {
      const cv::Vec3b v = src.at<cv::Vec3b>(y,x);
      destrow[x] = qRgba(v[2], v[1], v[0], 255);
    }
  }
  _img = dest;
  repaint();
}
