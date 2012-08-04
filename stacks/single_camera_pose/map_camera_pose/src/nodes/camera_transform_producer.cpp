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
#include <image_geometry/pinhole_camera_model.h>

#include <boost/thread/mutex.hpp>

#define NODE "camera_transform_producer"

namespace {

  bool save_launch_file = true;
  std::string launch_file_path = "default.launch";

  nav_msgs::OccupancyGrid::ConstPtr map;
  cv::Mat map_image;
  bool map_received;

  cv_bridge::CvImageConstPtr camera_image_ptr; 
  sensor_msgs::CameraInfoConstPtr camera_info_ptr; 

  enum ClickState {
    NONE_CLICKED,
    CAM_CLICKED,
    MAP_CLICKED
  } clickState = NONE_CLICKED;

  std::vector<cv::Point2f> cam_image_pxls;
  std::vector<cv::Point3f> map_image_pts; // of the form a,b,c
  std::vector<cv::Point2f> map_image_pxls;

  boost::mutex mutex_image_pxls;

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
  mat4.resize(max_rows, zero_scalar);
  mat5.resize(max_rows, zero_scalar);

  cv::hconcat(mat4, mat5, mat3);
}

void drawCorrespondences(cv::Mat& img) {
  boost::mutex::scoped_lock lock(mutex_image_pxls);

  cv::Scalar green(0,255,0);
  cv::Scalar red(0,0,255);

  unsigned int count = 0;
  for (; count < map_image_pxls.size() &&
      count < cam_image_pxls.size(); count++) {
    cv::circle(img, map_image_pxls[count], 5, green, 2);
    cv::circle(img, cam_image_pxls[count], 5, green, 2);
    cv::line(img, map_image_pxls[count], cam_image_pxls[count], green, 2);
  }

  if (count == map_image_pxls.size() - 1) {
    cv::circle(img, map_image_pxls[count], 5, red, 2);
  }

  if (count == cam_image_pxls.size() - 1) {
    cv::circle(img, cam_image_pxls[count], 5, red, 2);
  }
}

void processImage(const sensor_msgs::ImageConstPtr& msg,
    const sensor_msgs::CameraInfoConstPtr& cam_info) {

  if (!map_received) {
    ROS_INFO_STREAM("Waiting for map...");
    return;
  }
  
  camera_image_ptr = cv_bridge::toCvShare(msg, "bgr8");
  camera_info_ptr = cam_info;

  cv::Mat concat_image;
  constructConcatenatedImage(camera_image_ptr->image, map_image, concat_image);
  drawCorrespondences(concat_image);

  cv::imshow("Display", concat_image);

}

bool isCamImage(int x, int y) {
  return (x < camera_image_ptr->image.cols);
}

cv::Point3f getMapImagePt(int x, int y) {
  // Fix point so in the space of the map
  x -= camera_image_ptr->image.cols;
  // Flip y axis
  y = map_image.rows - y;

  // TODO Assumes no rotation in the map -- not well supported feature
  float zFinal = map->info.origin.position.z;
  float xFinal = x * map->info.resolution + map->info.origin.position.x;
  float yFinal = y * map->info.resolution + map->info.origin.position.y;

  return cv::Point3f(xFinal, yFinal, zFinal);
}

void generateTransformation() {

  if (map_image_pts.size() < 4) 
    return;

  std::cout << "Have 4 or more points. Generating transformation...\n";

  cv::Mat camera_matrix(3,3,CV_32FC1);
  camera_matrix.at<float>(0,0) = camera_info_ptr->K[0];
  camera_matrix.at<float>(0,1) = camera_info_ptr->K[1];
  camera_matrix.at<float>(0,2) = camera_info_ptr->K[2];
  camera_matrix.at<float>(1,0) = camera_info_ptr->K[3];
  camera_matrix.at<float>(1,1) = camera_info_ptr->K[4];
  camera_matrix.at<float>(1,2) = camera_info_ptr->K[5];
  camera_matrix.at<float>(2,0) = camera_info_ptr->K[6];
  camera_matrix.at<float>(2,1) = camera_info_ptr->K[7];
  camera_matrix.at<float>(2,2) = camera_info_ptr->K[8];
  std::cout << camera_matrix;

  std::vector<float> distortion_coefficients;
  distortion_coefficients.resize(5);
  distortion_coefficients[0] = camera_info_ptr->D[0];
  distortion_coefficients[1] = camera_info_ptr->D[1];
  distortion_coefficients[2] = camera_info_ptr->D[2];
  distortion_coefficients[3] = camera_info_ptr->D[3];
  distortion_coefficients[4] = camera_info_ptr->D[4];

  cv::Mat rotation_matrix(3,3,CV_32FC1);
  cv::Mat translation_vector(3,1,CV_32FC1);

  for (unsigned int i = 0; i < map_image_pts.size(); i++) {
    std::cout << "Map point selected: (" << map_image_pts[i].x << "," << map_image_pts[i].y << "," << map_image_pts[i].z  << ") --> Cam pixel (" << cam_image_pxls[i].x << "," << cam_image_pxls[i].y << ")" << std::endl;
  }

  cv::solvePnP(map_image_pts, cam_image_pxls, camera_matrix, 
  //    camera_info_ptr->D, rotation_matrix, translation_vector, false, CV_EPNP);
      distortion_coefficients, rotation_matrix, translation_vector, false, CV_EPNP);

  std::cout << "Translation (" << translation_vector.at<float>(0,0) << ","
            << translation_vector.at<float>(1,0) << ","
            << translation_vector.at<float>(2,0) << ")" << std::endl;
}

void undoPrevious() {
  boost::mutex::scoped_lock lock(mutex_image_pxls);
  switch (clickState) {
    case NONE_CLICKED:
      if (map_image_pts.size() != 0) {
        map_image_pts.pop_back();
        map_image_pxls.pop_back();
        clickState = CAM_CLICKED;
      }
      break;
    case CAM_CLICKED:
      cam_image_pxls.pop_back();
      clickState = NONE_CLICKED;
      break;
    case MAP_CLICKED:
      map_image_pts.pop_back();
      map_image_pxls.pop_back();
      clickState = NONE_CLICKED;
      break;
  }
}

void processClick(int x, int y) {
  boost::mutex::scoped_lock lock(mutex_image_pxls);
  switch (clickState) {
    case NONE_CLICKED:
      if (isCamImage(x,y)) {
        cam_image_pxls.push_back(cv::Point(x,y));
        clickState = CAM_CLICKED;
      } else {
        map_image_pts.push_back(getMapImagePt(x,y));
        map_image_pxls.push_back(cv::Point(x,y));
        clickState = MAP_CLICKED;
      }
      break;
    case CAM_CLICKED:
      if (isCamImage(x,y)) {
        cam_image_pxls.pop_back();
        cam_image_pxls.push_back(cv::Point(x,y));
      } else {
        map_image_pts.push_back(getMapImagePt(x,y));
        map_image_pxls.push_back(cv::Point(x,y));
        clickState = NONE_CLICKED;
        generateTransformation();
      }
      break;
    case MAP_CLICKED:
      if (isCamImage(x,y)) {
        cam_image_pxls.push_back(cv::Point(x,y));
        clickState = NONE_CLICKED;
      } else {
        map_image_pts.pop_back();
        map_image_pxls.pop_back();
        map_image_pts.push_back(getMapImagePt(x,y));
        map_image_pxls.push_back(cv::Point(x,y));
        generateTransformation();
      }
      break;
  }
}

static void mouseCallback(int event, int x, int y, int, void *) {
  switch(event) {
    case CV_EVENT_LBUTTONDOWN:
      processClick(x,y);
      break;
    case CV_EVENT_RBUTTONDOWN:
      undoPrevious();
      break;
  }
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
  image_transport::CameraSubscriber image_subscriber = 
     it.subscribeCamera(image_topic, 1, &processImage);

  // Start OpenCV display window
  cv::namedWindow("Display");
  cvSetMouseCallback("Display", mouseCallback);

  cvStartWindowThread();

  ros::spin();

  return 0;
}
