#include "Detector.h"
#include <ros/ros.h>
#define NODE "camera_transform_producer"

double _frameRate = 0;
int _frameCount = 0;
ros::Time _startTime;

cv::Scalar generateColorFromId(unsigned int id) {
  uchar r = (id * id % 255);
  uchar g = ((id + 1) * (id + 3)) % 255;
  uchar b = ((id + 5) * (id + 7)) % 255;
  return cv::Scalar(b,g,r);
}

void draw(std::vector<DetectorOutput>& outputs, cv::Mat& image, cv::Mat& foreground) {
  _frameCount++;
  if(_frameCount % 10 == 0) {
    _frameRate = (double)_frameCount / (ros::Time::now() - _startTime).toSec();
    _frameCount = 0;
    _startTime = ros::Time::now();
  }
  
  std::stringstream ss; ss << "Frame Rate: " << _frameRate;
  cv::putText(image, ss.str(), cv::Point(0,15), 0, 0.5, cv::Scalar(255));
  BOOST_FOREACH(DetectorOutput& output, outputs) {
    cv::Point textPoint(output.boundingBox.x + output.boundingBox.width, output.boundingBox.y);
    
    cv::rectangle(image, output.boundingBox, generateColorFromId(output.reading.id), 3);
    std::stringstream ss; ss << output.reading.id;
    cv::putText(image, ss.str(), textPoint, 0, 0.5, cv::Scalar(255));
    cv::circle(image, output.feetImage, 1, cv::Scalar(128,128,0), 1);
  }
  cv::imshow("Display", image);
  cv::imshow("Foreground", foreground);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, NODE);
  ros::NodeHandle node, nh_param("~");
  _startTime = ros::Time::now();
  Detector detector;
  detector.setCallback(&draw);
  detector.run(node, nh_param);
  
  // Start OpenCV display window
  cv::namedWindow("Display");
  cv::namedWindow("Foreground");

  cvStartWindowThread();
  ros::spin();
  return 0;
}
