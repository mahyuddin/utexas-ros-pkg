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
#include "sp/SegmentationProcessor.h"

#include <image_geometry/pinhole_camera_model.h>
#include <boost/foreach.hpp>

#include "TransformProvider.h"
#include "ekf/Ekf.h"
#include "Level.h"
#include "PersonIdentifier.h"

#define NODE "camera_transform_producer"

// The hog detector needs a window that's slightly larger 
// than the detected person. Because we calculate height 
// based on this window size, we need to adjust the height 
// after detection.
#define HOG_HEIGHT_ADJUSTMENT .87 
#define BS_HEIGHT_ADJUSTMENT 1.2

namespace {

  cv_bridge::CvImageConstPtr camera_image_ptr; 
  sensor_msgs::CameraInfoConstPtr camera_info_ptr;
  std::string map_frame_id;

  cv::BackgroundSubtractorMOG2 mog;
  cv::Mat foreground;

  boost::shared_ptr<cv::HOGDescriptor> hog;
  boost::shared_ptr<cv::CascadeClassifier> haar;

  bool use_hog_descriptor;
  std::string hog_descriptor_file;
  bool use_haar_cascade;
  std::string haar_cascade_file;

  bool search_space_calculated = false;
  double min_person_height;
  double max_person_height;
  int window_stride;
  int window_height;
  int window_width;
  double window_scale;
  int max_levels;
  int min_window_height;
  int max_window_height;

  int min_group_rectangles;
  double group_eps;

  int hog_deriv_aperture;
  double hog_l2hys_threshold;
  double hog_win_sigma;
  bool hog_gamma_correction;
  double hog_hit_threshold;
  double hog_weight_threshold;

  TransformProvider _transform;
  sp::SegmentationProcessor _processor;
  EkfManager _manager;
  PersonIdentifier _identifier;
  EkfModel *_hogModel, *_bsModel;

  double _frameRate;
  ros::Time _startTime;
  int _frameCount;
  std::vector<Level> levels;
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

void drawEKF(cv::Mat &img, cv::Mat& orig) {
  std::map<int,bool> found;
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
    int id = _identifier.getBestPersonId(orig,rect,found);
    found[id] = true;
    cv::rectangle(img, rect, generateColorFromId(id), 3);
    std::stringstream ss; ss << id;
    cv::putText(img, ss.str(), cv::Point(top.x + imageHeight / 4 + 2, top.y), 0, 0.5, cv::Scalar(255));
    //ROS_INFO("Person detected at (%2.2f, %2.2f, %2.2f)", mean(1), mean(2), mean(5));
    cv::circle(img, bottom, 1, cv::Scalar(128,128,0), 1);
    
    ROS_INFO("feet world position: (%2.2f, %2.2f, %2.2f)", feet.x(), feet.y(), feet.z());
  }
}

bool calculateSearchSpace() {

  // Initialize some variables
  int image_center;
  tf::Point ground_point,world_point;
  cv::Point image_point;
  int window_top, window_bottom;

  // Compute overall max window size (will be at bottom of the image)
  window_bottom = camera_image_ptr->image.rows - 1;
  image_center = camera_image_ptr->image.cols / 2;
  ground_point = _transform.getWorldProjection(cv::Point(image_center, window_bottom));
  world_point = ground_point + tf::Point(0, 0, max_person_height);
  image_point = _transform.getImageProjection(world_point);
  window_top = (image_point.y > 0) ? image_point.y : 0;
  max_window_height = (window_bottom - window_top > max_window_height) ? 
    max_window_height : window_bottom - window_top;
  
  // Compute overall min window size (will be at top of the image)
  window_top = 0;
  image_center = camera_image_ptr->image.cols / 2;
  world_point = 
    _transform.getWorldProjection(cv::Point(image_center, window_top), min_person_height);
  ground_point = world_point - tf::Point(0, 0, min_person_height);
  image_point = _transform.getImageProjection(ground_point);
  window_bottom = (image_point.y < camera_image_ptr->image.rows) ? 
    image_point.y : camera_image_ptr->image.rows;
  min_window_height = (window_bottom - window_top < min_window_height) ?
    min_window_height : window_bottom - window_top;

  ROS_INFO_STREAM("Estimated maximum window size = " << max_window_height << 
                  ", min size = " << min_window_height);

  // Now compute scaling between these window sizes
  int num_levels = max_levels;
  if (min_window_height > max_window_height) {
    ROS_ERROR_STREAM("Minimum computed window size greater than maximum. " <<
                     "Is the camera upside-down?");
    return false;
  } else {
    int max_scaled_size = 
      (int) (pow(window_scale, max_levels) * min_window_height);
    if (max_scaled_size > max_window_height) {
      num_levels = 
        log(max_window_height / min_window_height) / log(window_scale) + 1;
    } else {
      window_scale = 
        exp(log(max_window_height / min_window_height) / max_levels);
    }
  }

  ROS_INFO_STREAM("Using scale = " << window_scale << 
                     ", levels = " << num_levels);

  // Calculate information about all the different levels
  levels.clear();

  // Calculate all the different scales - start at scale such that the minimum
  // window becomes equal to the detector window size
  float scale = (float) min_window_height / window_height;
  for (int n = 0; n < num_levels; n++) {

    Level level;
    level.scale = scale;

    level.image_width = cvCeil(camera_image_ptr->image.cols / level.scale);
    level.image_height = cvCeil(camera_image_ptr->image.rows / level.scale);
    level.orig_window_height = cvRound(window_height * scale);

    // Now, for this level, calculate search space in original image

    ROS_DEBUG_STREAM("Level " << level.scale << " with win size " << 
                    level.orig_window_height <<
                    " will have effective img size: " << level.image_width <<
                    "x" << level.image_height);

    // Now, let's assume that due to some minor deviances in the calculation,
    // this level won't actually have any search space inside it.
    level.search_space_found = false;

    // Now let's check whether a window of this height fits into the image at
    // different locations
    for (window_bottom = level.image_height - 1; window_bottom >= window_height; 
         window_bottom -= window_stride) {

      int window_top = window_bottom - window_height;

      // Get these image coordinates in the original image
      int orig_window_bottom = cvFloor(window_bottom * scale);
      if (orig_window_bottom >= camera_image_ptr->image.rows)
        orig_window_bottom = camera_image_ptr->image.rows - 1;
      int orig_image_center = camera_image_ptr->image.cols / 2;

      ground_point = 
        _transform.getWorldProjection(cv::Point(orig_image_center, orig_window_bottom));

      // Get upper point by assuming max height
      world_point = ground_point + tf::Point(0,0,max_person_height);
      image_point = _transform.getImageProjection(world_point);
      int upper_window_top = cvFloor(image_point.y / scale);

      // Get lower point by assuming min height
      world_point = ground_point + tf::Point(0,0,min_person_height);
      image_point = _transform.getImageProjection(world_point);
      int lower_window_top = cvFloor(image_point.y / scale);

      // This location is good for this level if the window top is in between
      // these upper and lower ranges
      if (window_top >= upper_window_top && window_top <= lower_window_top) {
        if (!level.search_space_found) {
          level.resized_start_y = window_top;
          level.resized_end_y = window_bottom;
          level.search_space_found = true;
        } else {
          if (window_top < level.resized_start_y) 
            level.resized_start_y = window_top;
          if (window_bottom > level.resized_end_y)
            level.resized_end_y = window_bottom;
        }
      }
    }

    if (level.search_space_found) {
      level.orig_start_y = cvFloor(level.resized_start_y * level.scale);
      level.orig_end_y = cvFloor(level.resized_end_y * level.scale);

      ROS_DEBUG_STREAM("  Search from " << level.orig_start_y << " to " <<
          level.orig_end_y);
      ROS_DEBUG_STREAM("  in resize img " << level.resized_start_y << " to " <<
          level.resized_end_y);
    }
    levels.push_back(level);
    scale *= window_scale;
  }

  search_space_calculated = true;
  return true;

}

void detect(cv::Mat& img, Level& level, 
    std::vector<cv::Rect>& locations, std::vector<double>& weights) {

  if (!level.search_space_found) {
    return;
  }

  // Image size at this scale
  cv::Size img_size(level.image_width, level.image_height);
  cv::Mat resized_img;
  cv::resize(img, resized_img, img_size);

  // Cropped image based on search space
  cv::Rect crop_region(0, level.resized_start_y, img_size.width, 
      level.resized_end_y - level.resized_start_y);
  cv::Mat cropped_img = resized_img(crop_region);

  // detect
  std::vector<cv::Point> level_locations;
  hog->detect(cropped_img, level_locations, weights, hog_hit_threshold, 
      cv::Size(window_stride, window_stride));

  locations.clear();
  locations.reserve(level_locations.size());

  // Fix locations appropriately
  BOOST_FOREACH(cv::Point& level_loc, level_locations) {
    locations.push_back(
        cv::Rect(cvRound(level_loc.x * level.scale),
          cvRound((level_loc.y + level.resized_start_y) * level.scale),
          level.orig_window_height / 2,
          level.orig_window_height * HOG_HEIGHT_ADJUSTMENT));
  }

}

std::vector<cv::Rect> detectBackground(cv::Mat& img) {
  std::vector<cv::Rect> locations;
  std::vector<sp::Blob*> blobs = _processor.constructBlobs(foreground);
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

std::vector<cv::Rect> detectMultiScale(cv::Mat& img) {

  boost::thread level_threads[levels.size()];
  std::vector<std::vector<cv::Rect> > level_locations;
  std::vector<std::vector<double> > level_weights;
  level_locations.resize(levels.size());
  level_weights.resize(levels.size());

  // start all the threads
  int i = 0;
  BOOST_FOREACH(Level& level, levels) {
    if (!level.search_space_found) {
      continue;
    }
    level_threads[i] = boost::thread(
        boost::bind(&detect, boost::ref(img), 
        boost::ref(level), boost::ref(level_locations[i]),
        boost::ref(level_weights[i])));
    i++;
  }

  // end all the threads
  i = 0;
  int num_total_locations = 0;
  BOOST_FOREACH(Level& level, levels) {
    if (!level.search_space_found) {
      continue;
    }
    level_threads[i].join();
    num_total_locations += level_locations[i].size();
    i++;
  }

  // concatenate all the locations
  std::vector<cv::Rect> all_locations;
  std::vector<double> weights;
  i = 0;
  BOOST_FOREACH(std::vector<cv::Rect>& level_location, level_locations) {
    all_locations.insert(all_locations.end(), 
        level_location.begin(), level_location.end());
    weights.insert(weights.end(), level_weights[i].begin(),
        level_weights[i].end());
    i++;
  }

  
  // group similar rectangles together
  cv::groupRectangles(all_locations, min_group_rectangles - 1, group_eps);
  std::vector<cv::Rect> locations;
  for(size_t i = 0; i < all_locations.size(); i++) {
    if(weights[i] >= hog_weight_threshold)
      locations.push_back(all_locations[i]);
  }
  return locations;
}

std::vector<PersonReading> getReadingsFromDetections(cv::Mat& image, std::vector<cv::Rect> detections) {
  std::vector<PersonReading> readings;
  BOOST_FOREACH(cv::Rect& detection, detections) {
    cv::Point bottom(detection.x + detection.width / 2, detection.y + detection.height);
    cv::Point top(detection.x + detection.width / 2, detection.y);    
    float height = _transform.getWorldHeight(top,bottom);
    if(height < min_person_height) continue;
    tf::Point feet = _transform.getWorldProjection(bottom);
    int id = _identifier.getPersonId(image, detection);
    PersonReading reading(feet.x(), feet.y(), height, id);
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
  camera_info_ptr = cam_info;
  _transform.computeModel(cam_info);

  // Apply background subtraction along with some filtering to detect person
  mog(camera_image_ptr->image, foreground, -1);
  cv::threshold(foreground, foreground, 128, 255, CV_THRESH_BINARY);
  cv::medianBlur(foreground, foreground, 9);
  cv::erode(foreground, foreground, cv::Mat());
  cv::dilate(foreground, foreground, cv::Mat());
 
  // Get ground plane and form the search rectangle list
  if (!_transform.isGroundPlaneAvailable()) {
    _transform.computeGroundPlane(msg->header.frame_id);
  }
  if (!search_space_calculated) {
    if (!calculateSearchSpace()) {
      ros::shutdown();
      return;
    }
  }
  

  cv::Mat gray_image(camera_image_ptr->image.rows, camera_image_ptr->image.cols,
      CV_8UC1);
  cv::cvtColor(camera_image_ptr->image, gray_image, CV_RGB2GRAY);

  std::vector<cv::Rect> hog_locations, bs_locations, haar_locations;
  if (use_hog_descriptor) {
    hog_locations = detectMultiScale(gray_image);
    bs_locations = detectBackground(foreground);
  } else {
    haar->detectMultiScale(gray_image, haar_locations, window_scale, 
        min_group_rectangles);
  }

  cv::Mat display_image(camera_image_ptr->image);
  cv::Mat original_image(camera_image_ptr->image); 
  //BOOST_FOREACH(cv::Rect& rect, bs_locations)
    //cv::rectangle(display_image, rect, cv::Scalar(0,0,255), 3);
  //BOOST_FOREACH(cv::Rect& rect, hog_locations)
    //cv::rectangle(display_image, rect, cv::Scalar(0,255,0), 3);

  _manager.updateFilters(getReadingsFromDetections(original_image, hog_locations), _hogModel);
  _manager.updateFilters(getReadingsFromDetections(original_image, bs_locations), _bsModel);
  drawEKF(display_image, original_image);
  
  cv::imshow("Display", display_image);
  cv::imshow("Foreground", foreground);
}

void getParams(ros::NodeHandle& nh) {

  nh.param<double>("min_person_height", min_person_height, 1.37f); // 4.5 feet
  nh.param<double>("max_person_height", max_person_height, 2.13f); // 7 feet

  nh.param<int>("window_stride", window_stride, 8);
  nh.param<double>("window_scale", window_scale, 1.05);
  nh.param<int>("min_window_height", min_window_height, 64);
  nh.param<int>("max_window_height", max_window_height, 512);
  nh.param<int>("max_levels", max_levels, 64);
  nh.param<int>("min_group_rectangles", min_group_rectangles, 2);
  nh.param<double>("group_eps", group_eps, 1);

  nh.param<int>("window_height", window_height, 128);
  nh.param<int>("window_width", window_width, 64);

  nh.param<bool>("use_hog_descriptor", use_hog_descriptor, true);
  nh.param<std::string>("hog_descriptor_file", hog_descriptor_file, "");
  nh.param<int>("hog_deriv_aperture", hog_deriv_aperture, 1);
  nh.param<double>("hog_win_sigma", hog_win_sigma, -1);
  nh.param<double>("hog_l2hys_threshold", hog_l2hys_threshold, 0.2);
  nh.param<bool>("hog_gamma_correction", hog_gamma_correction, true);
  nh.param<double>("hog_hit_threshold", hog_hit_threshold, 0.4);
  nh.param<double>("hog_weight_threshold", hog_weight_threshold, 0.05);

  nh.param<bool>("use_haar_cascade", use_haar_cascade, false);
  nh.param<std::string>("haar_cascade_file", haar_cascade_file, "");

  nh.param<std::string>("map_frame_id", map_frame_id, "/map");
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
    cv::Size window_size(window_width, window_height);
    cv::Size block_size(16, 16);
    cv::Size block_stride(8, 8);
    cv::Size cell_size(8, 8);
    int nbins = 9;
    int deriv_aperture = hog_deriv_aperture;
    double win_sigma = hog_win_sigma;
    int histogram_type = cv::HOGDescriptor::L2Hys;
    double l2hys_threshold = hog_l2hys_threshold;
    bool gamma_correction = hog_gamma_correction;
    int nlevels = 64;
    hog.reset(new cv::HOGDescriptor(window_size, block_size, block_stride, 
         cell_size, nbins, deriv_aperture, win_sigma, histogram_type, 
         l2hys_threshold, gamma_correction, nlevels));
    hog.reset(new cv::HOGDescriptor());
    hog->setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
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
