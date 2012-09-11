/**
 * \file  detector.cc
 * \brief  
 *
 * Copyright (C) 2012, UT Austin
 */

#include <stdlib.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <boost/foreach.hpp>

#include "TransformProvider.h"
#include "ekf/Ekf.h"
#include "sp/SegmentationProcessor.h"
#include "Level.h"
#include "PersonIdentifier.h"
#include "MultiscaleHogDetector.h"

#define NODE "camera_transform_producer"

// The hog detector needs a window that's slightly larger 
// than the detected person. Because we calculate height 
// based on this window size, we need to adjust the height 
// after detection.
#define BS_HEIGHT_ADJUSTMENT 1.2

namespace {

  cv_bridge::CvImageConstPtr camera_image_ptr; 
  sensor_msgs::CameraInfoConstPtr camera_info_ptr;
  std::string map_frame_id;

  cv::BackgroundSubtractorMOG2 mog;

  boost::shared_ptr<cv::CascadeClassifier> haar;

  bool use_hog_descriptor;
  bool use_haar_cascade;
  std::string haar_cascade_file;

  TransformProvider _transform;
  sp::SegmentationProcessor _processor;
  EkfManager _manager;
  PersonIdentifier _identifier;
  MultiscaleHogDetector* _detector;
  EkfModel *_hogModel, *_bsModel;

  double _frameRate;
  ros::Time _startTime;
  int _frameCount;
  double _minPersonHeight;
}

cv::Scalar generateColorFromId(unsigned int id) {
  uchar r = (id * id % 255);
  uchar g = ((id + 1) * (id + 3)) % 255;
  uchar b = ((id + 5) * (id + 7)) % 255;
  return cv::Scalar(r,g,b);
}

cv::Rect correctForImage(cv::Rect rect, cv::Mat& image) {
  if(rect.x < 0) rect.x = 0;
  if(rect.y < 0) rect.y = 0;
  if(rect.x > image.cols - rect.width) rect.x = image.cols - rect.width;
  if(rect.y > image.rows - rect.height) rect.y = image.rows - rect.height;
  return rect;
}

void drawEKF(cv::Mat &img, cv::Mat& orig, cv::Mat& foreground) {
  std::stringstream ss; ss << "Frame Rate: " << _frameRate;
  cv::putText(img, ss.str(), cv::Point(0,15), 0, 0.5, cv::Scalar(255));
  BOOST_FOREACH(PersonEkf* filter, _manager.getValidEstimates()) {  
    BFL::Pdf<MatrixWrapper::ColumnVector>* posterior = filter->PostGet();
    MatrixWrapper::ColumnVector mean = posterior->ExpectedValueGet();
    
    double x = mean(1), y = mean(2), height = mean(5);
    tf::Point head(x,y,height);
    tf::Point feet(x,y,0);
    cv::Point top = _transform.getImageProjection(head);
    cv::Point bottom = _transform.getImageProjection(feet);
    
    int imageHeight = abs(top.y - bottom.y);
    cv::Rect rect(top.x - imageHeight / 4, top.y, imageHeight / 2, imageHeight); 
    rect = correctForImage(rect,orig);
    int id = filter->getId();
    cv::rectangle(img, rect, generateColorFromId(id), 3);
    std::stringstream ss; ss << id;
    cv::putText(img, ss.str(), cv::Point(top.x + imageHeight / 4 + 2, top.y), 0, 0.5, cv::Scalar(255));
    cv::circle(img, bottom, 1, cv::Scalar(128,128,0), 1);
  }
}

std::vector<cv::Rect> detectBackground(cv::Mat& img) {
  std::vector<cv::Rect> locations;
  std::vector<sp::Blob*> blobs = _processor.constructBlobs(img);
  BOOST_FOREACH(sp::Blob* blob, blobs) {
    if(blob->getArea() < 30 * 120) continue;
    cv::Rect rect(
      blob->getLeft(), 
      blob->getBottom(), 
      blob->getWidth(), 
      std::min((int)(blob->getHeight() * BS_HEIGHT_ADJUSTMENT), img.rows - blob->getBottom())
    );
    locations.push_back(rect);
  }
  return locations;
}

std::vector<PersonReading> getReadingsFromDetections(cv::Mat& image, cv::Mat& foreground, std::vector<cv::Rect> detections, bool getId = false) {
  std::vector<PersonReading> readings;
  BOOST_FOREACH(cv::Rect& detection, detections) {
    cv::Point bottom(detection.x + detection.width / 2, detection.y + detection.height);
    cv::Point top(detection.x + detection.width / 2, detection.y);    
    float height = _transform.getWorldHeight(top,bottom);
    if(height < _minPersonHeight) continue;
    tf::Point feet = _transform.getWorldProjection(bottom);
    PersonReading reading(feet.x(), feet.y(), height);
    if(getId) reading.id = _identifier.getPersonId(image,foreground,detection);
    readings.push_back(reading);
  }
  return readings;
}

void processImage(const sensor_msgs::ImageConstPtr& msg,
    const sensor_msgs::CameraInfoConstPtr& cam_info) {
  _frameCount++;
  if(_frameCount % 10 == 0) {
    _frameRate = (double)_frameCount / (ros::Time::now() - _startTime).toSec();
    _frameCount = 0;
    _startTime = ros::Time::now();
  }
  camera_image_ptr = cv_bridge::toCvShare(msg, "bgr8");
  cv::Mat original(camera_image_ptr->image); 
  camera_info_ptr = cam_info;
  _transform.computeModel(cam_info);

  // Apply background subtraction along with some filtering to detect person
  cv::Mat foreground;
  mog(camera_image_ptr->image, foreground, -1);
  cv::threshold(foreground, foreground, 128, 255, CV_THRESH_BINARY);
  cv::medianBlur(foreground, foreground, 9);
  cv::erode(foreground, foreground, cv::Mat());
  cv::dilate(foreground, foreground, cv::Mat());
 
  // Get ground plane and form the search rectangle list
  if (!_transform.isGroundPlaneAvailable()) {
    _transform.computeGroundPlane(msg->header.frame_id);
  }
  _detector->calculateSearchSpace(original.rows, original.cols);  

  cv::Mat gray_image(original.rows, original.cols, CV_8UC1);
  cv::cvtColor(original, gray_image, CV_RGB2GRAY);

  std::vector<cv::Rect> hog_locations, bs_locations, haar_locations;
  if (use_hog_descriptor) {
    //hog_locations = _detector->detectMultiScale(gray_image);
    bs_locations = detectBackground(foreground);
  } else {
    //haar->detectMultiScale(gray_image, haar_locations, window_scale, min_group_rectangles);
  }

  cv::Mat display_image = original.clone();
  cv::Mat display_foreground = original.clone();
  for(int i = 0; i < display_image.rows; i++) {
    for(int j = 0; j < display_image.cols; j++) {
      if(!foreground.at<bool>(i,j))
        display_foreground.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
    }
  }
  _manager.updateFilters(getReadingsFromDetections(original, foreground, hog_locations), _hogModel);
  _manager.updateFilters(getReadingsFromDetections(original, foreground, bs_locations, true), _bsModel);
  drawEKF(display_image, original, foreground);
  
  cv::imshow("Display", display_image);
  cv::imshow("Foreground", display_foreground);
}

void getParams(ros::NodeHandle& nh) {

  nh.param<bool>("use_hog_descriptor", use_hog_descriptor, true);

  nh.param<bool>("use_haar_cascade", use_haar_cascade, false);
  nh.param<std::string>("haar_cascade_file", haar_cascade_file, "");

  nh.param<std::string>("map_frame_id", map_frame_id, "/map");
  
  nh.param<double>("min_person_height", _minPersonHeight, 1.37f);
}


int main(int argc, char *argv[]) {
  ros::init(argc, argv, NODE);
  ros::NodeHandle node, nh_param("~");
  _startTime = ros::Time::now(); _frameRate = 0;
  getParams(nh_param);
  _transform = TransformProvider(map_frame_id);
  _hogModel = new HogModel();
  _bsModel = new BsModel();

  if (use_hog_descriptor) {
    _detector = new MultiscaleHogDetector(_transform,nh_param);
  } else {
    haar.reset(new cv::CascadeClassifier(haar_cascade_file));
  }
  
  // subscribe to the camera image stream to setup correspondences
  image_transport::ImageTransport it(node);
  std::string image_topic = node.resolveName("image_raw");

  image_transport::CameraSubscriber image_subscriber = 
     it.subscribeCamera(image_topic, 1, &processImage);

  // Start OpenCV display window
  cv::namedWindow("Display");
  cv::namedWindow("Foreground");

  cvStartWindowThread();

  ros::spin();

  return 0;
}
