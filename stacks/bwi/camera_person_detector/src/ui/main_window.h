#ifndef camera_person_detector_MAIN_WINDOW_H
#define camera_person_detector_MAIN_WINDOW_H

#include "Detector.h"
#include <bwi_msgs/PersonDetection.h>

#include <boost/foreach.hpp>

#include <QtGui>
#include <QtGui/QMainWindow>
#include <QMessageBox>

#include "ui_main_window.h"

#define FRAME_INTERVAL 10
#define NODE "camera_transform_producer"

typedef bwi_msgs::PersonDetection Detection;

namespace camera_person_detector {
  class MainWindow : public QMainWindow {
    Q_OBJECT

    public:
      MainWindow(int argc, char** argv, QWidget *parent = 0);
      ~MainWindow();

      void draw(std::vector<Detection>&, cv::Mat&, cv::Mat&);

    public slots:
      void rosLoop();
    private:
      Ui::MainWindowDesign ui;
      void init(int,char**);
      cv::Scalar getColorFromId(unsigned);
      void markDetection(Detection&, cv::Mat&);
      void displayStats(std::vector<Detection>& outputs);
      int _frameCount;
      ros::Time _frameTime;
  };
}
#endif // bwi_person_controller_MAIN_WINDOW_H
