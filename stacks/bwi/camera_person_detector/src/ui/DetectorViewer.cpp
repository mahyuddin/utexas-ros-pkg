#include "DetectorViewer.h"

namespace camera_person_detector {

  using namespace Qt;

  DetectorViewer::DetectorViewer(int argc, char** argv, QWidget *parent) : QMainWindow(parent) {
    ui.setupUi(this);
    init(argc,argv);
  }

  void DetectorViewer::init(int argc, char** argv) {
    ros::init(argc, argv, NODE);
    ros::NodeHandle *node = new ros::NodeHandle(), *nh_param = new ros::NodeHandle("~");
    _signin = new Signin(*node, *nh_param);
    _detector = new Detector();
    _detector->setCallback(boost::bind(&DetectorViewer::draw, this, _1, _2, _3));
    _detector->run(*node, *nh_param);
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(rosLoop()));
    timer->start(10);
    _frameCount = 0;
    _frameTime = ros::Time::now();

    XmlRpc::XmlRpcValue cameras;
    nh_param->getParam("cameras", cameras);
    for(int i = 0; i < cameras.size(); i++) {
      std::string camera = static_cast<std::string>(cameras[i]);
      _cameras.push_back(camera);
      ui.cbCameras->addItem(QString::fromStdString(camera));
    }

    setConnections();
  }

  void DetectorViewer::setConnections() {
    connect(ui.cbxRegisterAll, SIGNAL(stateChanged(int)), this, SLOT(setRegisterAll(int)));
    connect(ui.cbxRegisterPerson, SIGNAL(stateChanged(int)), this, SLOT(setRegisterPerson(int)));
    connect(ui.cbCameras, SIGNAL(currentIndexChanged(int)), this, SLOT(cameraSelected(int)));
  }
  DetectorViewer::~DetectorViewer() {}

  void DetectorViewer::cameraSelected(int index) {
    _detector->setCamera(_cameras[index]);
    _signin->setCamera(_cameras[index]);
  }

  cv::Scalar DetectorViewer::getColorFromId(unsigned id) {
    uchar r = (id * id % 255);
    uchar g = ((id + 1) * (id + 3)) % 255;
    uchar b = ((id + 5) * (id + 7)) % 255;
    return cv::Scalar(b,g,r);
  }

  void DetectorViewer::draw(std::vector<Detection>& detections, cv::Mat& image, cv::Mat& foreground) {
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
    ui.lblDetections->setText(QString::number(detections.size()));
    BOOST_FOREACH(Detection& detection, detections) {
      markDetection(detection,image);
      markDetection(detection,foreground);
    } 
    ui.imgScene->setImage(image);
    ui.imgForeground->setImage(foreground);
    displayStats(detections);
  }

  void DetectorViewer::displayStats(std::vector<Detection>& detections) {
    QString stats;
    int i = 0;
    BOOST_FOREACH(Detection& detection, detections) {
      i++;
      stats += "Detection " + QString::number(i) + "\n";
      stats += "ID: " + QString::number(detection.id) + "\n";
      stats += 
        QString("Feet Position: (%1,%2)\n")
        .arg(QString::number(detection.feet.x, 'f', 3))
        .arg(QString::number(detection.feet.y, 'f', 3));
      stats += QString("Height: %1\n").arg(QString::number(detection.height, 'f', 3));
      stats += "----------------\n";
    }
    ui.txtStats->setText(stats);
  }

  void DetectorViewer::markDetection(Detection& detection, cv::Mat& image) {
    cv::Point textPoint(detection.imageBox.x + detection.imageBox.width, detection.imageBox.y);
    bwi_msgs::BoundingBox bb = detection.imageBox;
    cv::Rect rect(bb.x,bb.y,bb.width,bb.height);
    cv::rectangle(image, rect, getColorFromId(detection.id), 3);
    std::stringstream ss; ss << detection.id;
    cv::putText(image, ss.str(), textPoint, 0, 0.5, cv::Scalar(255));
    cv::Point feet(detection.imageFeet.x, detection.imageFeet.y);
    cv::circle(image, feet, 1, cv::Scalar(128,128,0), 1);
  }

  void DetectorViewer::rosLoop() {
    if(!ros::ok()) emit rosShutdown();
    ros::spinOnce();
  }

  void DetectorViewer::setRegisterAll(int state) {
    if(state == 0)
      _detector->setRegisterAll(false);
    else
      _detector->setRegisterAll(true);
  }

  void DetectorViewer::setRegisterPerson(int state) {
    if(state == 0)
      _signin->stop();
    else
      _signin->collect();
  }
}
