#ifndef camera_person_detector_MAIN_WINDOW_H
#define camera_person_detector_MAIN_WINDOW_H

#include "Detector.h"
#include "Signin.h"
#include <bwi_msgs/PersonDetection.h>

#include <boost/foreach.hpp>

#include <QtGui>
#include <QtGui/QMainWindow>
#include <QMessageBox>

#include "ui_DetectorViewer.h"

#define FRAME_INTERVAL 10
#define NODE "camera_transform_producer"

typedef bwi_msgs::PersonDetection Detection;

namespace camera_person_detector {
  class DetectorViewer : public QMainWindow {
    Q_OBJECT

    public:
      DetectorViewer(int argc, char** argv, QWidget *parent = 0);
      ~DetectorViewer();

      void draw(std::vector<Detection>&, cv::Mat&, cv::Mat&);

    public slots:
      void rosLoop();
      void setRegisterAll(int);
      void setRegisterPerson(int);
      void cameraSelected(int);
    signals:
      void rosShutdown();
    private:
      Ui::DetectorViewerDesign ui;
      
      void init(int,char**);
      void setConnections();
      cv::Scalar getColorFromId(unsigned);
      void markDetection(Detection&, cv::Mat&);
      void displayStats(std::vector<Detection>& outputs);
      int _frameCount;
      ros::Time _frameTime;

      Detector* _detector;
      std::map<std::string, Detector*> _detectors;
      Signin* _signin;
      std::vector<std::string> _cameras;
  };
}
#endif // bwi_person_controller_MAIN_WINDOW_H
