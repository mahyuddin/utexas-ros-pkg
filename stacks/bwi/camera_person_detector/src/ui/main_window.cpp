#include "main_window.h"

namespace camera_person_detector {

  using namespace Qt;

  MainWindow::MainWindow(int argc, char** argv, QWidget *parent) : QMainWindow(parent) {
    ui.setupUi(this);
    init(argc,argv);
  }

  void MainWindow::init(int argc, char** argv) {
    ros::init(argc, argv, NODE);
    ros::NodeHandle *node = new ros::NodeHandle(), *nh_param = new ros::NodeHandle("~");
    Detector* detector = new Detector();
    detector->setCallback(boost::bind(&MainWindow::draw, this, _1, _2, _3));
    detector->run(*node, *nh_param);
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(rosLoop()));
    timer->start(10);
    _frameCount = 0;
    _frameTime = ros::Time::now();
  }
  MainWindow::~MainWindow() {}

  cv::Scalar MainWindow::getColorFromId(unsigned id) {
    uchar r = (id * id % 255);
    uchar g = ((id + 1) * (id + 3)) % 255;
    uchar b = ((id + 5) * (id + 7)) % 255;
    return cv::Scalar(b,g,r);
  }

  void MainWindow::draw(std::vector<DetectorOutput>& outputs, cv::Mat& image, cv::Mat& foreground) {
    _frameCount++;
    QString frameStr;
    if(_frameCount % FRAME_INTERVAL == 0) {
      double t = (ros::Time::now() - _frameTime).toSec();
      double rate = 0;
      if(t > 0)
        rate = FRAME_INTERVAL / t;
      frameStr = QString::number(rate, 'g', 2) + " Hz";
      ui.lblFrameRate->setText(frameStr);
      _frameTime = ros::Time::now();
    }
    ui.lblDetections->setText(QString::number(outputs.size()));
    BOOST_FOREACH(DetectorOutput& output, outputs) {
      markDetections(output,image);
      markDetections(output,foreground);
    } 
    ui.imgScene->setImage(image);
    ui.imgForeground->setImage(foreground);
    displayStats(outputs);
  }

  void MainWindow::displayStats(std::vector<DetectorOutput>& outputs) {
    QString stats;
    int i = 0;
    BOOST_FOREACH(DetectorOutput& output, outputs) {
      i++;
      stats += "Detection " + QString::number(i) + "\n";
      stats += "ID: " + QString::number(output.reading.id) + "\n";
      stats += 
        QString("Feet Position: (%1,%2)\n")
        .arg(QString::number(output.reading.x, 'f', 3))
        .arg(QString::number(output.reading.y, 'f', 3));
      stats += QString("Height: %1\n").arg(QString::number(output.reading.height, 'f', 3));
      stats += "----------------\n";
    }
    ui.txtStats->setText(stats);
  }

  void MainWindow::markDetections(DetectorOutput& output, cv::Mat& image) {
    cv::Point textPoint(output.boundingBox.x + output.boundingBox.width, output.boundingBox.y);

    cv::rectangle(image, output.boundingBox, getColorFromId(output.reading.id), 3);
    std::stringstream ss; ss << output.reading.id;
    cv::putText(image, ss.str(), textPoint, 0, 0.5, cv::Scalar(255));
    cv::circle(image, output.feetImage, 1, cv::Scalar(128,128,0), 1);
  }

  void MainWindow::rosLoop() {
    ros::spinOnce();
  }
}
