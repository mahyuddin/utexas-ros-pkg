/**
 * \file  camera_transform_producer.cpp
 * \brief  Computes camera transform from correspondences provided by user 
 * in camera stream and a map of the world
 *
 * Copyright (C) 2012, UT Austin
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Image.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#define NODE "camera_transform_producer"

namespace {

  bool save_launch_file = true;
  std::string launch_file_path = "default.launch";

  nav_msgs::OccupancyGrid::ConstPtr map;
  cv::Mat map_image;
  bool map_received;

  cv_bridge::CvImageConstPtr camera_image_ptr; 
}

void occupancyGridToImage(const nav_msgs::OccupancyGrid::ConstPtr grid, 
    cv::Mat& image) {

  image = cv::Mat::zeros(grid->info.height, grid->info.width, CV_8UC3);
  for (int row = grid->info.height - 1; row >= 0; row--) {
    for (unsigned int col = 0; col < grid->info.width; col++) {
      int grid_row = grid->info.height - 1 - row;
      int grid_data = grid->data[grid_row * grid->info.width + col];
      if (grid_data != -1) {
        image.at<cv::Vec3b>(row, col)[0] = image.at<cv::Vec3b>(row, col)[1] = 
            image.at<cv::Vec3b>(row, col)[2] = 255 - (255 * grid_data) / 100;
      } else {
        image.at<cv::Vec3b>(row, col)[0] = image.at<cv::Vec3b>(row, col)[1] = 
            image.at<cv::Vec3b>(row, col)[2] = 128;
      }
    }
  }

}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr map_msg) {
  map = map_msg;
  occupancyGridToImage(map, map_image);
  map_received = true;
  ROS_INFO_STREAM("Received map from map_server");
}

void constructConcatenatedImage(
    const cv::Mat& mat1, const cv::Mat& mat2, cv::Mat& mat3) {

  int max_rows = (mat1.rows > mat2.rows) ? mat1.rows : mat2.rows;
  cv::Mat mat4 = mat1.clone();
  cv::Mat mat5 = mat2.clone();
  cv::Scalar zero_scalar(0,0,0);
  //zero_scalar[0] = zero_scalar[1] = zero_scalar[2] = 0;
  mat4.resize(max_rows, zero_scalar);
  mat5.resize(max_rows, zero_scalar);

  cv::hconcat(mat4, mat5, mat3);
}

void processImage(const sensor_msgs::ImageConstPtr &msg) {
  if (!map_received) {
    ROS_INFO_STREAM("Waiting for map...");
    return;
  }
  
  camera_image_ptr = cv_bridge::toCvShare(msg, "bgr8");
  cv::Mat concat_image;
  constructConcatenatedImage(camera_image_ptr->image, map_image, concat_image);
  cv::imshow("Display", concat_image);

}

void getParams(ros::NodeHandle& nh) {
  nh.getParam("save_launch_file", save_launch_file);
  nh.getParam("launch_file_path", launch_file_path);
}

int main(int argc, char *argv[]) {
  
  ros::init(argc, argv, NODE);
  ros::NodeHandle node, nh_param("~");
  getParams(nh_param);

  // subscribe to the map data
  ros::Subscriber sub_map = 
      node.subscribe<nav_msgs::OccupancyGrid>("map", 1, mapCallback);

  // subscribe to the camera image stream to setup correspondences
  image_transport::ImageTransport it(node);
  std::string image_topic = node.resolveName("image_raw");
  image_transport::Subscriber image_subscriber = 
     it.subscribe(image_topic, 1, &processImage);

  // Start OpenCV display window
  cv::namedWindow("Display");

  cvStartWindowThread();

  ros::spin();

  return 0;
}
