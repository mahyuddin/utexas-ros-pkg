#include "MultiscaleHogDetector.h"

MultiscaleHogDetector::MultiscaleHogDetector(TransformProvider& transform, ros::NodeHandle& nh) 
  : _transform(transform), _nh(nh) {
  initialize();
  _searchSpaceCalculated = false;
}

void MultiscaleHogDetector::calculateSearchSpace(int rows, int cols) {
  if(_searchSpaceCalculated) return;
  // Initialize some variables
  int image_center;
  tf::Point ground_point,world_point;
  cv::Point image_point;
  int window_top, window_bottom;
  std::vector<Level> levels;
  
  // Pull variables from the node handle
  
  double min_person_height;
  double max_person_height;

  double window_scale;
  int max_levels;
  int min_window_height;
  int max_window_height;
  
  _nh.param<double>("min_person_height", min_person_height, 1.37f); // 4.5 feet
  _nh.param<double>("max_person_height", max_person_height, 2.13f); // 7 feet
  
  _nh.param<int>("window_stride", _windowStride, 8);
  _nh.param<int>("window_height", _windowHeight, 128);
  _nh.param<int>("window_width", _windowWidth, 64);
  _nh.param<double>("window_scale", window_scale, 1.05);
  
  _nh.param<int>("max_levels", max_levels, 64);
  _nh.param<int>("min_window_height", min_window_height, 64);
  _nh.param<int>("max_window_height", max_window_height, 512);

  // Compute overall max window size (will be at bottom of the image)
  window_bottom = rows - 1;
  image_center = cols / 2;
  ground_point = _transform.getWorldProjection(cv::Point(image_center, window_bottom));
  world_point = ground_point + tf::Point(0, 0, max_person_height);
  image_point = _transform.getImageProjection(world_point);
  window_top = (image_point.y > 0) ? image_point.y : 0;
  max_window_height = (window_bottom - window_top > max_window_height) ? 
    max_window_height : window_bottom - window_top;
  
  // Compute overall min window size (will be at top of the image)
  window_top = 0;
  image_center = cols / 2;
  world_point = 
    _transform.getWorldProjection(cv::Point(image_center, window_top), min_person_height);
  ground_point = world_point - tf::Point(0, 0, min_person_height);
  image_point = _transform.getImageProjection(ground_point);
  window_bottom = (image_point.y < rows) ? 
    image_point.y : rows;
  min_window_height = (window_bottom - window_top < min_window_height) ?
    min_window_height : window_bottom - window_top;

  ROS_INFO_STREAM("Estimated maximum window size = " << max_window_height << 
                  ", min size = " << min_window_height);

  // Now compute scaling between these window sizes
  int num_levels = max_levels;
  if (min_window_height > max_window_height) {
    throw std::invalid_argument("Minimum window height is greater than maximum. Is the camera upside-down?");
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

  // Calculate all the different scales - start at scale such that the minimum
  // window becomes equal to the detector window size
  float scale = (float) min_window_height / _windowHeight;
  for (int n = 0; n < num_levels; n++) {

    Level level;
    level.scale = scale;

    level.image_width = cvCeil(cols / level.scale);
    level.image_height = cvCeil(rows / level.scale);
    level.orig_window_height = cvRound(_windowHeight * scale);

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
    for (window_bottom = level.image_height - 1; window_bottom >= _windowHeight; 
         window_bottom -= _windowStride) {

      int window_top = window_bottom - _windowHeight;

      // Get these image coordinates in the original image
      int orig_window_bottom = cvFloor(window_bottom * scale);
      if (orig_window_bottom >= rows)
        orig_window_bottom = rows - 1;
      int orig_image_center = cols / 2;

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

  _levels = levels;
  _searchSpaceCalculated = true;
}

std::vector<cv::Rect> MultiscaleHogDetector::detectMultiScale(cv::Mat& img) {

  boost::thread level_threads[_levels.size()];
  std::vector<std::vector<cv::Rect> > level_locations;
  std::vector<std::vector<double> > level_weights;
  level_locations.resize(_levels.size());
  level_weights.resize(_levels.size());

  tpool pool;

  // start all the threads
  int i = 0;
  BOOST_FOREACH(Level& level, _levels) {
    if (!level.search_space_found) {
      continue;
    }
    level_threads[i] = boost::thread(
        boost::bind(&MultiscaleHogDetector::detect, this, boost::ref(img), 
        boost::ref(level), boost::ref(level_locations[i]),
        boost::ref(level_weights[i])));
    i++;
  }

  // end all the threads
  i = 0;
  int num_total_locations = 0;
  BOOST_FOREACH(Level& level, _levels) {
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
  cv::groupRectangles(all_locations, _minGroupRectangles - 1, _groupEps);
  std::vector<cv::Rect> locations;
  for(size_t i = 0; i < all_locations.size(); i++) {
    if(weights[i] >= _weightThreshold)
      locations.push_back(all_locations[i]);
  }
  return locations;
}

void MultiscaleHogDetector::detect(cv::Mat& img, Level& level, 
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
  _hog->detect(cropped_img, level_locations, weights, _hitThreshold, 
      cv::Size(_windowStride, _windowStride));

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

void MultiscaleHogDetector::initialize() {
    cv::Size window_size(_windowWidth, _windowHeight);
    cv::Size block_size(16, 16);
    cv::Size block_stride(8, 8);
    cv::Size cell_size(8, 8);
    int nbins = 9;
    int deriv_aperture;
    double win_sigma;
    int histogram_type = cv::HOGDescriptor::L2Hys;
    double l2hys_threshold;
    bool gamma_correction;
    int nlevels = 64;
    
    _nh.param<int>("hog_deriv_aperture", deriv_aperture, 1);
    _nh.param<double>("hog_win_sigma", win_sigma, -1);
    _nh.param<double>("hog_l2hys_threshold", l2hys_threshold, 0.2);
    _nh.param<bool>("hog_gamma_correction", gamma_correction, true);
    
    _nh.param<int>("min_group_rectangles", _minGroupRectangles, 2);
    _nh.param<double>("group_eps", _groupEps, 1);
    
    _nh.param<double>("hog_weight_threshold", _weightThreshold, 0.05);
    _nh.param<double>("hog_hit_threshold", _hitThreshold, 0.4);
  
    _hog.reset(new cv::HOGDescriptor(window_size, block_size, block_stride, 
         cell_size, nbins, deriv_aperture, win_sigma, histogram_type, 
         l2hys_threshold, gamma_correction, nlevels));
    _hog.reset(new cv::HOGDescriptor());
    _hog->setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
}

bool MultiscaleHogDetector::isSearchSpaceCalculated() {
  return _searchSpaceCalculated;
}
