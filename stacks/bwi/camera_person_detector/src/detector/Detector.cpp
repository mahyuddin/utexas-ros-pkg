#include "Detector.h"

BwiSubtractor::BwiSubtractor() : cv::BackgroundSubtractorMOG2() {
  fVarInit = 32;
  backgroundRatio = 0.45;
}

Detector::Detector(ros::NodeHandle& nh, ros::NodeHandle& nhParam) : _nh(nh), _nhParam(nhParam), _transport(0) {
  init();
}

bwi_msgs::BoundingBox Detector::getBB(int x, int y, int width, int height, cv::Mat& image) {
  x = std::max(x, 0);
  y = std::max(y, 0);
  x = std::min(x, image.cols - width);
  y = std::min(y, image.rows - height);
  bwi_msgs::BoundingBox bb;
  bb.x = x; bb.y = y; bb.width = width; bb.height = height;
  return bb;
}

bool Detector::isForegroundEmpty(cv::Mat& foreground) {
  int minPixels = 5, pixelCount = 0;
  int step = 4;
  int xmax = foreground.cols - (foreground.cols % step);
  int ymax = foreground.rows - (foreground.rows % step);
  for(int y = 0; y < ymax; y += step) {
    for(int x = 0; x < xmax; x += step) {
      if(pixelCount == minPixels) return false;
      if(foreground.at<bool>(y,x)) pixelCount++;
    }
  }
  return true;
}

std::vector<PersonReading> Detector::removeOverlaps(std::vector<PersonReading> readings, cv::Mat& image) {
  std::vector<PersonReading> keep;
  if(readings.size() < 2)
    return readings;
  static int size = image.rows * image.cols;
  static bool* pixels = new bool[size];
  if(image.rows * image.cols != size) {
    size = image.rows * image.cols;
    pixels = new bool[size];
  }
  memset(pixels, false, size);
  BOOST_FOREACH(PersonReading reading, readings) {
    int used = 0, total = reading.box.width * reading.box.height;
    for(int x = reading.box.x; x < reading.box.x + reading.box.width; x++) {
      for(int y = reading.box.y; y < reading.box.y + reading.box.height; y++) {
        if(pixels[y * image.cols + x])
          used++;
        pixels[y * image.cols + x] = true;
      }
    }
    if((float)used / total <= .5)
      keep.push_back(reading);
  }
  return keep;
}

void Detector::broadcast(cv::Mat& image, cv::Mat& foreground_mask, cv::Mat& foreground_display) {
  bwi_msgs::PersonDetectionArray detections;
  BOOST_FOREACH(PersonEkf* filter, _manager.getValidEstimates()) {  
    bwi_msgs::PersonDetection detection;
     
    BFL::Pdf<MatrixWrapper::ColumnVector>* posterior = filter->PostGet();
    MatrixWrapper::ColumnVector mean = posterior->ExpectedValueGet();
    
    double x = mean(1), y = mean(2), height = mean(5);
    detection.feet.x = x; detection.feet.y = y;
    detection.height = height;
    
    tf::Point head(x,y,height);
    tf::Point feet(x,y,0);
    cv::Point top = _transform.getImageProjection(head);
    cv::Point bottom = _transform.getImageProjection(feet);
    int imageHeight = abs(top.y - bottom.y);
    bwi_msgs::BoundingBox bb = getBB(
      bottom.x - imageHeight / 4, 
      bottom.y - imageHeight, 
      imageHeight / 2, 
      imageHeight,
      image); 
    detection.imageBox = bb;
    detection.imageFeet.x = bottom.x; detection.imageFeet.y = bottom.y;    
    ColorSignature signature(image, foreground_mask, cv::Rect(bb.x, bb.y, bb.width, bb.height));
    double sigdist;
    detection.id = _identifier.getSignatureId(signature, sigdist);
    detection.level_id = _levelId;
    if(detection.id)
      filter->setId(detection.id);
    else
      detection.id = filter->getId();
    detection.signatureDistance = sigdist;
    detection.signature = signature.getMsg();
    detections.detections.push_back(detection);
  }
  _publisher.publish(detections);
  if(_callback) _callback(detections.detections,image,foreground_display);
}

std::vector<cv::Rect> Detector::detectBackground(cv::Mat& img) {
  std::vector<cv::Rect> locations;
  std::vector<sp::Blob*> blobs = _processor.constructBlobs(img);
  BOOST_FOREACH(sp::Blob* blob, blobs) {
    double minArea = (double)img.cols / 20 * (double)img.rows / 4;
    if(blob->getArea() < minArea) continue;
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

std::vector<PersonReading> Detector::getReadingsFromDetections(cv::Mat& image, cv::Mat& foreground_mask, std::vector<cv::Rect> detections, EkfModel* model, bool registerSignature) {
  std::vector<PersonReading> readings;
  BOOST_FOREACH(cv::Rect& detection, detections) {
    cv::Point bottom(detection.x + detection.width / 2, detection.y + detection.height);
    cv::Point top(detection.x + detection.width / 2, detection.y);    
    float height = _transform.getWorldHeight(top,bottom);
    if(height < _minPersonHeight) continue;
    tf::Point feet = _transform.getWorldProjection(bottom);
    PersonReading reading(feet.x(), feet.y(), height, detection, model);
    readings.push_back(reading);
    if(registerSignature && _registerAll) _identifier.registerSignature(image, foreground_mask, detection);
  }
  return readings;
}

cv::Mat Detector::backgroundSubtract(cv::Mat& original) {
  cv::Mat foreground_mask;
  _mog(original, foreground_mask, -1);
  cv::threshold(foreground_mask, foreground_mask, 128, 255, CV_THRESH_BINARY);
  cv::medianBlur(foreground_mask, foreground_mask, 9);
  cv::erode(foreground_mask, foreground_mask, cv::Mat());
  cv::dilate(foreground_mask, foreground_mask, cv::Mat());
  return foreground_mask;
}

void Detector::processImage(const sensor_msgs::ImageConstPtr& msg,
    const sensor_msgs::CameraInfoConstPtr& cam_info) {
  if(_paused) return;

  cv_bridge::CvImageConstPtr cameraImagePtr = cv_bridge::toCvShare(msg, "bgr8");
  cv::Mat original(cameraImagePtr->image), foreground_mask; 
  // Apply background subtraction
  foreground_mask = backgroundSubtract(original);
  cv::Mat foreground_display = original.clone();
  for(int i = 0; i < original.rows; i++) {
    for(int j = 0; j < original.cols; j++) {
      if(!foreground_mask.at<bool>(i,j))
        foreground_display.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
    }
  }
  if(isForegroundEmpty(foreground_mask)) {
    _manager.updateFilters(std::vector<PersonReading>());
    broadcast(original, foreground_mask, foreground_display);
    return;
  }
  
  // Get ground plane and form the search rectangle list
  _transform.computeModel(cam_info);
  if (!_transform.isGroundPlaneAvailable()) {
    _transform.computeGroundPlane(msg->header.frame_id);
  }
  _detector->calculateSearchSpace(original.rows, original.cols);  

  // Detect People
  cv::Mat gray_image(original.rows, original.cols, CV_8UC1);
  cv::cvtColor(original, gray_image, CV_RGB2GRAY);

  std::vector<cv::Rect> hog_locations, bs_locations;
  hog_locations = _detector->detectMultiScale(gray_image);
  bs_locations = detectBackground(foreground_mask);

  // Get combined readings
  std::vector<PersonReading> hog_readings, bs_readings;
  hog_readings = getReadingsFromDetections(original, foreground_mask, hog_locations, _hogModel, false);
  bs_readings = getReadingsFromDetections(original, foreground_mask, bs_locations, _bsModel, true);
  std::vector<PersonReading> combined;
  BOOST_FOREACH(PersonReading& r, hog_readings)
    combined.push_back(r);
  BOOST_FOREACH(PersonReading& r, bs_readings)
    combined.push_back(r);
  combined = removeOverlaps(combined, original);

  // Update
  _manager.updateFilters(combined);
  broadcast(original, foreground_mask, foreground_display);
}

void Detector::processDetections(const bwi_msgs::PersonDetectionArray& detections) {
}

void Detector::getParams(ros::NodeHandle& nh) {
  nh.param<std::string>("level_id", _levelId, "demo1");
  _mapFrameId = bwi_utils::frameIdFromLevelId(_levelId);
  nh.param<double>("min_person_height", _minPersonHeight, 1.37f);
  nh.param<std::string>("camname", _camera, "camera1");
  nh.param<bool>("register_all", _registerAll, false);
}

void Detector::init() {
  getParams(_nhParam);
  _transform = TransformProvider(_mapFrameId);
  _hogModel = new HogModel();
  _bsModel = new BsModel();

  _detector = new MultiscaleHogDetector(_transform,_nhParam);
  
  _transport = new image_transport::ImageTransport(_nh);
  setCamera(_camera);

  _globalSub = _nh.subscribe("/global/person_detections", 1000, &Detector::processDetections, this);
  _registrationSub = _nh.subscribe("/global/registered_persons", 1000, &Detector::processPersonRegistration, this);
}

void Detector::processPersonRegistration(const bwi_msgs::PersonDescriptor descriptor) {
  _identifier.registerDescriptor(descriptor);
}

void Detector::setCallback(boost::function<void (CALLBACK_ARGS)> callback) {
  _callback = callback;
}

void Detector::pause() {
  _paused = true;
}

void Detector::unpause() {
  _paused = false;
}

std::string Detector::getImageTopic(std::string camera) {
  std::stringstream topic;
  topic << camera << "/image_raw";
  return topic.str();
}

void Detector::setCamera(std::string camera) {
  _camSub.shutdown();
  _camera = camera;
  std::string topic = getImageTopic(_camera);
  _camSub = _transport->subscribeCamera(topic, 1, &Detector::processImage, this);
  _publisher.shutdown();
  _publisher = _nh.advertise<bwi_msgs::PersonDetectionArray&>(_camera + "/person_detections", 1000);
  ROS_INFO("Subscribed to camera topic %s", _camSub.getTopic().c_str());
  ROS_INFO("Publishing to %s", _publisher.getTopic().c_str());
}

void Detector::setRegisterAll(bool value) {
  _registerAll = value;
}
