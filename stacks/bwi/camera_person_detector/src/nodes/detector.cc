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
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <boost/foreach.hpp>

#define NODE "camera_transform_producer"

namespace {

  cv_bridge::CvImageConstPtr camera_image_ptr; 
  sensor_msgs::CameraInfoConstPtr camera_info_ptr;
  image_geometry::PinholeCameraModel model;

  cv::BackgroundSubtractorMOG2 mog;
  cv::Mat foreground;

  // cv::HOGDescriptor hog(cv::Size(64, 128), cv::Size(16, 16), cv::Size(8, 8), 
  //     cv::Size(8, 8), 9, DEFAULT_WIN_SIGMA, 0.2, true, 0);
  cv::HOGDescriptor hog;

  std::vector<cv::Rect> rectangle_list;
  std::vector<float> level_scale;
  std::vector<int> level_min_y;
  std::vector<int> level_max_y;

  double  min_height;
  double max_height;
  std::string map_frame_id;
  int win_stride;
  double win_scale;
  int max_levels;

  bool ground_plane_available;
  tf::Point ground_point;
  tf::Point ground_normal;
  tf::StampedTransform transform_cam_from_map;
  tf::Transform transform_map_from_cam;
  
}

void computeGroundPlane(std::string camera_frame_id) {
  
  // Obtain transformation to camera
  tf::TransformListener listener;
  bool transform_found = 
    listener.waitForTransform(camera_frame_id, map_frame_id,
                              ros::Time(), ros::Duration(1.0));
  if (transform_found) {
    try {
      listener.lookupTransform(camera_frame_id, "/map",
                               ros::Time(), transform_cam_from_map);
    } catch (tf::TransformException ex) {
      ROS_ERROR_STREAM("Transform unavailable (Exception): " << ex.what());
    }
  } else {
    ROS_ERROR_STREAM("Transform unavailable: lookup failed");
  }

  transform_map_from_cam = transform_cam_from_map.inverse();

  tf::Point o_map(0,0,0);
  tf::Point p_map(1,0,0);
  tf::Point q_map(0,1,0);

  ground_point = transform_cam_from_map * o_map;
  tf::Point p_cam(transform_cam_from_map * p_map);
  tf::Point q_cam(transform_cam_from_map * q_map);

  ground_normal = (p_cam - ground_point).cross(q_cam - ground_point);

  ground_plane_available = true;

}

tf::Point getGroundProjection(cv::Point pt, float height = 0) {
  
  cv::Point2d image_point(pt.x, pt.y);
  cv::Point2d rectified_point(model.rectifyPoint(image_point));
  cv::Point3d ray = model.projectPixelTo3dRay(rectified_point);

  tf::Point ray_1(0, 0, 0);
  tf::Point ray_2(ray.x, ray.y, ray.z);
  tf::Point ground_origin = transform_cam_from_map * tf::Point(0,0,height);
  float t = (ground_origin - ray_1).dot(ground_normal)
          / (ray_2 - ray_1).dot(ground_normal);
  tf::Point point_cam = ray_1 + t * (ray_2 - ray_1);
  //std::cout << "  " << "pc: " << point_cam.x() << "," << point_cam.y() << "," << point_cam.z() << std::endl;
  return transform_map_from_cam * point_cam;

}

cv::Point getImageProjection(tf::Point pt) {
  tf::Point point_cam = transform_cam_from_map * pt;
  //std::cout << "  " << "pc: " << point_cam.x() << "," << point_cam.y() << "," << point_cam.z() << std::endl;
  cv::Point3d xyz(point_cam.x(), point_cam.y(), point_cam.z());
  cv::Point2d rectified_point(model.project3dToPixel(xyz));
  return model.unrectifyPoint(rectified_point);
}

bool populateRectangleList() {
  rectangle_list.clear();

  // Compute max window size
  int j = camera_image_ptr->image.rows - 1;
  int i = camera_image_ptr->image.cols / 2;
  tf::Point ground_point = getGroundProjection(cv::Point(i, j));
  /* std::cout << "  " << "pc: " << ground_point.x() << "," << ground_point.y() << "," << ground_point.z() << std::endl; */
  tf::Point max_point = ground_point + tf::Point(0, 0, max_height);
  cv::Point max_image_point = getImageProjection(max_point);
  int upper_bound = (max_image_point.y > 0) ? max_image_point.y : 0;
  int max_window_size = j - upper_bound;
  
  // Compute min window size
  j = 0;
  i = camera_image_ptr->image.cols / 2;
  tf::Point min_point = getGroundProjection(cv::Point(i,j), min_height);
  ground_point = min_point - tf::Point(0, 0, min_height);
  /* std::cout << "  " << "pc: " << ground_point.x() << "," << ground_point.y() << "," << ground_point.z() << std::endl; */
  cv::Point min_image_point = getImageProjection(ground_point);
  /* std::cout << "  img: " << min_image_point.x << " " << min_image_point.y << std::endl; */
  int lower_bound = (min_image_point.y < camera_image_ptr->image.rows) ? 
    min_image_point.y : camera_image_ptr->image.rows;
  int min_window_size = lower_bound;

  ROS_INFO_STREAM("Using max size = " << max_window_size << 
                  ", min size = " << min_window_size);

  int num_levels = max_levels;

  if (min_window_size > max_window_size) {
    ROS_ERROR_STREAM("Minimum computed window size greater than maximum. " <<
                     "Is the camera upside-down?");
    return false;
  } else {
    int max_scaled_size = (int) (pow(win_scale, max_levels) * min_window_size);
    if (max_scaled_size > max_window_size) {
      num_levels = log(max_window_size / min_window_size) / log(win_scale);
    } else {
      win_scale = exp(log(max_window_size / min_window_size) / max_levels);
    }
  }

  ROS_INFO_STREAM("Using scale = " << win_scale << ", levels = " << num_levels);
  return false;

  // Start at the bottom of the image and go up, assuming the bottom is always
  // closer to the camera
  for (int i = camera_image_ptr->image.rows - 1; i >= 0; i -= win_stride) {

    // Compute max and min height in pixels for the bounding box
    int j = camera_image_ptr->image.cols / 2;
    tf::Point ground_point = getGroundProjection(cv::Point(j, i));
    //std::cout << "row " << i << ": ground projection " << ground_point.x() << "," << ground_point.y() << "," << ground_point.z() << std::endl;

    tf::Point max_point = ground_point + tf::Point(0,0,max_height);
    tf::Point min_point = ground_point + tf::Point(0,0,min_height);

    cv::Point max_image_point = getImageProjection(max_point);
    //std::cout << "row" << i << ": max_image_pt " << max_image_point.x << "," << max_image_point.y << std::endl;
    cv::Point min_image_point = getImageProjection(min_point);

    int lower_bound = (min_image_point.y > 0) ? min_image_point.y : 0;
    int upper_bound = (max_image_point.y > 0) ? max_image_point.y : 0;

    std::cout << "row " << i << ": from " << i - upper_bound << " to " << i - lower_bound << std::endl;
    std::cout << "  evaluating for heights: "; 
    for (float height = i - lower_bound; height <= i - upper_bound; 
           height *= win_scale) {
      int height_in_int = (int) height;
      //std::cout << height_in_int << " ";
      for (j = 0; j < camera_image_ptr->image.cols; j += win_stride) {
        int right_bound = j + height_in_int / 2;
        if (right_bound >= camera_image_ptr->image.cols) 
          break;
        
        cv::Rect r(j, i - height_in_int, height_in_int / 2, height_in_int);
        rectangle_list.push_back(r);
      }
    }
    //std::cout << std::endl;

    if (lower_bound == 0) {
      break;
    }

  }

  ROS_INFO_STREAM("Number of rectangles to check: " << rectangle_list.size());

}

void detect(cv::Mat& img, std::vector<cv::Rect>& locations) {
  BOOST_FOREACH(cv::Rect& r, rectangle_list) {
    cv::Mat window = img(r);
    cv::Mat resized_window;
    cv::Size window_size(128, 64);
    cv::resize(window, resized_window, window_size);
    std::vector<cv::Point> window_locations;
    //hog.detect(resized_window, window_locations);
  }
}

void processImage(const sensor_msgs::ImageConstPtr& msg,
    const sensor_msgs::CameraInfoConstPtr& cam_info) {

  camera_image_ptr = cv_bridge::toCvShare(msg, "bgr8");
  camera_info_ptr = cam_info;
  model.fromCameraInfo(cam_info);

  // Apply background subtraction along with some filtering to detect person
  // mog(camera_image_ptr->image, foreground, -1);
  // cv::threshold(foreground, foreground, 128, 255, CV_THRESH_BINARY);
  // cv::medianBlur(foreground, foreground, 9);
  // cv::erode(foreground, foreground, cv::Mat());
  // cv::dilate(foreground, foreground, cv::Mat());
 
  // Get ground plane and form the search rectangle list
  if (!ground_plane_available) {
    computeGroundPlane(msg->header.frame_id);
  }
  if (rectangle_list.empty()) {
    if (!populateRectangleList()) {
      ros::shutdown();
      return;
    }
  }
  
  std::vector<cv::Rect> locations;
  cv::Mat gray_image(camera_image_ptr->image.rows, camera_image_ptr->image.cols,
      CV_8UC1);
  cv::cvtColor(camera_image_ptr->image, gray_image, CV_RGB2GRAY);

  //detect(gray_image, locations);
  BOOST_FOREACH(cv::Rect& rect, locations) {
    cv::rectangle(gray_image, rect, cv::Scalar(0, 255, 0), 3); 
  }

  ROS_INFO_STREAM(locations.size());

  //cv::imshow("Display", gray_image);
  cv::imshow("Display", camera_image_ptr->image);
}

void getParams(ros::NodeHandle& nh) {
  nh.param<double>("min_expected_height", min_height, 1.22f); // 4 feet in m
  nh.param<double>("max_expected_height", max_height, 2.13f); // 7 feet in m
  nh.param<int>("win_stride", win_stride, 8); // default opencv stride (i think)
  nh.param<double>("win_scale", win_scale, 1.05f); // default opencv 
  nh.param<int>("max_levels", max_levels, 64); 
  nh.param<std::string>("map_frame_id", map_frame_id, "/map");
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
