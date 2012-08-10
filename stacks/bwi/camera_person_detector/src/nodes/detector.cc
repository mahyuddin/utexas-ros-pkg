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

  std::vector<float> level_scale;
  std::vector<int> level_min_y;
  std::vector<int> level_max_y;
  std::vector<bool> level_found;
  bool search_space_calculated = false;

  double  min_height;
  double max_height;
  std::string map_frame_id;
  int win_stride;
  double win_scale;
  int max_levels;
  int min_window_size = 64;
  int max_window_size = 256;

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

bool calculateSearchSpace() {

  level_scale.clear();
  level_min_y.clear();
  level_max_y.clear();

  // Compute max window size
  int j = camera_image_ptr->image.rows - 1;
  int i = camera_image_ptr->image.cols / 2;
  tf::Point ground_point = getGroundProjection(cv::Point(i, j));
  /* std::cout << "  " << "pc: " << ground_point.x() << "," << ground_point.y() << "," << ground_point.z() << std::endl; */
  tf::Point max_point = ground_point + tf::Point(0, 0, max_height);
  cv::Point max_image_point = getImageProjection(max_point);
  int upper_bound = (max_image_point.y > 0) ? max_image_point.y : 0;
  max_window_size = j - upper_bound;
  // if (max_window_size > 25)
  //   max_window_size = 256;
  
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
  min_window_size = lower_bound;
  if (min_window_size < 96)
    min_window_size = 96;

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
      num_levels = log(max_window_size / min_window_size) / log(win_scale) + 1;
    } else {
      win_scale = exp(log(max_window_size / min_window_size) / max_levels);
    }
  }

  ROS_INFO_STREAM("Using scale = " << win_scale << ", levels = " << num_levels);

  // Calculate all the different scales
  float scale = (float) min_window_size / 128;
  for (int n = 0; n < num_levels; n++) {
    level_scale.push_back(scale);
    scale *= win_scale;
  }

  // Calculate all the min and max values for each scale in the original image
  BOOST_FOREACH(float scale, level_scale) {

    // Image size at this scale
    cv::Size img_size((int) std::ceil(camera_image_ptr->image.cols / scale),
                     (int) std::ceil(camera_image_ptr->image.rows / scale));

    ROS_INFO_STREAM(" For scale " << scale << " -> " << img_size.width << "," 
                    << img_size.height);

    // Window size at this scale
    int level_start_y = img_size.height;
    int level_finish_y = -1;
    bool found = false;
    for (j = img_size.height - 1; j >= 128; j -= win_stride) {
      i = img_size.width / 2;

      // Get real image pixel coordinates
      int real_j = j * scale;
      int real_i = i * scale;

      ground_point = getGroundProjection(cv::Point(real_i, real_j));
      max_point = ground_point + tf::Point(0,0,max_height);
      min_point = ground_point + tf::Point(0,0,min_height);

      max_image_point = getImageProjection(max_point);
      max_image_point.x /= scale;
      max_image_point.y /= scale;
      min_image_point = getImageProjection(min_point);
      min_image_point.x /= scale;
      min_image_point.y /= scale;

      if (j - 128 >= std::floor(max_image_point.y) &&
          j - 128 <= std::ceil(min_image_point.y)) {
        // This level is applicable at this height
        if (j > level_finish_y)
          level_finish_y = j;
        if (j < level_start_y)
          level_start_y = j - 128;
        found = true;
      }
    }

    ROS_INFO_STREAM("    min: " << level_start_y << " max: " << level_finish_y);

    level_min_y.push_back(level_start_y);
    level_max_y.push_back(level_finish_y);
    level_found.push_back(found);
  }

  search_space_calculated = true;
  return true;

}

void detect(cv::Mat& img, std::vector<cv::Rect>& locations) {
  // Calculate all the min and max values for each scale in the original image
  int i = 0;
  BOOST_FOREACH(float scale, level_scale) {

    if (!level_found[i]) {
      i++;
      continue;
    }

    // Image size at this scale
    cv::Size img_size((int) std::ceil(camera_image_ptr->image.cols / scale),
                      (int) std::ceil(camera_image_ptr->image.rows / scale));
    cv::Mat resized_img;
    cv::resize(img, resized_img, img_size);

    // Cropped image based on search space
    cv::Rect crop_region(0, level_min_y[i], img_size.width, level_max_y[i] - level_min_y[i]);
    cv::Mat cropped_img = resized_img(crop_region);

    // detect
    std::vector<cv::Point> level_locations;
    hog.detect(cropped_img, level_locations);

    // fix locations appropriately
    BOOST_FOREACH(cv::Point& level_loc, level_locations) {
      locations.push_back(cv::Rect(cvRound(level_loc.x * scale),
                                   cvRound((level_loc.y + level_min_y[i]) * scale),
                                   cvRound(64 * scale),
                                   cvRound(128 * scale)));
    }

    i++;
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
  if (!search_space_calculated) {
    if (!calculateSearchSpace()) {
      ros::shutdown();
      return;
    }
  }
  
  std::vector<cv::Rect> locations;
  cv::Mat gray_image(camera_image_ptr->image.rows, camera_image_ptr->image.cols,
      CV_8UC1);
  cv::cvtColor(camera_image_ptr->image, gray_image, CV_RGB2GRAY);

  detect(gray_image, locations);
  //hog.detectMultiScale(gray_image,locations);
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
