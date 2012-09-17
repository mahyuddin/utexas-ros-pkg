#ifndef camera_person_detector_MAIN_WINDOW_H
#define camera_person_detector_MAIN_WINDOW_H

#include "Detector.h"
#include "DetectorOutput.h"
#define NODE "camera_transform_producer"
#include <boost/foreach.hpp>

#include <QtGui>
#include <QtGui/QMainWindow>
#include <QMessageBox>

#include "ui_main_window.h"

#define FRAME_INTERVAL 10

namespace camera_person_detector {
  class MainWindow : public QMainWindow {
    Q_OBJECT

    public:
      MainWindow(int argc, char** argv, QWidget *parent = 0);
      ~MainWindow();

      void draw(std::vector<DetectorOutput>&, cv::Mat&, cv::Mat&);

    public slots:
      void rosLoop();
    private:
      Ui::MainWindowDesign ui;
      void init(int,char**);
      cv::Scalar getColorFromId(unsigned);
      void markDetections(DetectorOutput&, cv::Mat&);
      void displayStats(std::vector<DetectorOutput>& outputs);
      int _frameCount;
      ros::Time _frameTime;
  };
}
#endif // bwi_person_controller_MAIN_WINDOW_H
