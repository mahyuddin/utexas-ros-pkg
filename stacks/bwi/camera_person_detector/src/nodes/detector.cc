/**
 * \file  detector.cc
 * \brief  
 *
 * Copyright (C) 2012, UT Austin
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <boost/foreach.hpp>

#define NODE "camera_transform_producer"

namespace {

  cv_bridge::CvImageConstPtr camera_image_ptr; 
  sensor_msgs::CameraInfoConstPtr camera_info_ptr; 

  cv::BackgroundSubtractorMOG2 mog;
  cv::Mat foreground;

  cv::HOGDescriptor hog;
}

void processImage(const sensor_msgs::ImageConstPtr& msg,
    const sensor_msgs::CameraInfoConstPtr& cam_info) {

  camera_image_ptr = cv_bridge::toCvShare(msg, "bgr8");
  camera_info_ptr = cam_info;

  // Apply background subtraction along with some filtering to detect person
  // mog(camera_image_ptr->image, foreground, -1);
  // cv::threshold(foreground, foreground, 128, 255, CV_THRESH_BINARY);
  // cv::medianBlur(foreground, foreground, 9);
  // cv::erode(foreground, foreground, cv::Mat());
  // cv::dilate(foreground, foreground, cv::Mat());
  
  std::vector<cv::Rect> locations;
  cv::Mat gray_image(camera_image_ptr->image.rows, camera_image_ptr->image.cols,
      CV_8UC1);
  cv::cvtColor(camera_image_ptr->image, gray_image, CV_RGB2GRAY);
  hog.detectMultiScale(gray_image, locations);

  BOOST_FOREACH(cv::Rect& rect, locations) {
    cv::rectangle(gray_image, rect, cv::Scalar(0, 255, 0), 3); 
  }

  ROS_INFO_STREAM(locations.size());

  cv::imshow("Display", gray_image);
}

void getParams(ros::NodeHandle& nh) {
}

int main(int argc, char *argv[]) {
  
  ros::init(argc, argv, NODE);
  ros::NodeHandle node, nh_param("~");
  getParams(nh_param);

  // subscribe to the camera image stream to setup correspondences
  image_transport::ImageTransport it(node);
  std::string image_topic = node.resolveName("image_raw");
  image_transport::CameraSubscriber image_subscriber = 
     it.subscribeCamera(image_topic, 1, &processImage);

  // Start OpenCV display window
  cv::namedWindow("Display");

  cvStartWindowThread();

  // Apply HOG Detector
  hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
  ros::spin();

  return 0;
}
