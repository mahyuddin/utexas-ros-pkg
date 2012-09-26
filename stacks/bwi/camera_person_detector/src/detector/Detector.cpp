#include "Detector.h"

Detector::Detector() : _transport(0) {
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
      top.x - imageHeight / 4, 
      top.y, 
      imageHeight / 2, 
      imageHeight,
      image); 
    detection.imageBox = bb;
    detection.imageFeet.x = bottom.x; detection.imageFeet.y = bottom.y;    
    ColorSignature signature(image, foreground_mask, cv::Rect(bb.x, bb.y, bb.width, bb.height));
    detection.id = _identifier.getSignatureId(signature);
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

std::vector<PersonReading> Detector::getReadingsFromDetections(cv::Mat& image, cv::Mat& foreground_mask, std::vector<cv::Rect> detections, bool registerSignature = false) {
  std::vector<PersonReading> readings;
  BOOST_FOREACH(cv::Rect& detection, detections) {
    cv::Point bottom(detection.x + detection.width / 2, detection.y + detection.height);
    cv::Point top(detection.x + detection.width / 2, detection.y);    
    float height = _transform.getWorldHeight(top,bottom);
    if(height < _minPersonHeight) continue;
    tf::Point feet = _transform.getWorldProjection(bottom);
    PersonReading reading(feet.x(), feet.y(), height);
    readings.push_back(reading);
    if(registerSignature && _registerAll) _identifier.registerSignature(image, foreground_mask, detection);
  }
  return readings;
}

void Detector::processImage(const sensor_msgs::ImageConstPtr& msg,
    const sensor_msgs::CameraInfoConstPtr& cam_info) {
  if(_paused) return;
  cv_bridge::CvImageConstPtr cameraImagePtr = cv_bridge::toCvShare(msg, "bgr8");
  cv::Mat original(cameraImagePtr->image); 
  _transform.computeModel(cam_info);

  // Apply background subtraction along with some filtering to detect person
  cv::Mat foreground_mask(_foreground);
  _mog(original, foreground_mask, -1);
  cv::threshold(foreground_mask, foreground_mask, 128, 255, CV_THRESH_BINARY);
  cv::medianBlur(foreground_mask, foreground_mask, 9);
  cv::erode(foreground_mask, foreground_mask, cv::Mat());
  cv::dilate(foreground_mask, foreground_mask, cv::Mat());
 
  // Get ground plane and form the search rectangle list
  if (!_transform.isGroundPlaneAvailable()) {
    _transform.computeGroundPlane(msg->header.frame_id);
  }
  _detector->calculateSearchSpace(original.rows, original.cols);  

  cv::Mat gray_image(original.rows, original.cols, CV_8UC1);
  cv::cvtColor(original, gray_image, CV_RGB2GRAY);

  std::vector<cv::Rect> hog_locations, bs_locations;
  hog_locations = _detector->detectMultiScale(gray_image);
  bs_locations = detectBackground(foreground_mask);

  cv::Mat foreground_display = original.clone();
  for(int i = 0; i < original.rows; i++) {
    for(int j = 0; j < original.cols; j++) {
      if(!foreground_mask.at<bool>(i,j))
        foreground_display.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
    }
  }
  _manager.updateFilters(getReadingsFromDetections(original, foreground_mask, hog_locations), _hogModel);
  _manager.updateFilters(getReadingsFromDetections(original, foreground_mask, bs_locations, true), _bsModel);
  broadcast(original, foreground_mask, foreground_display);
}

void Detector::processDetections(const bwi_msgs::PersonDetectionArray& detections) {
}

void Detector::getParams(ros::NodeHandle& nh) {
  nh.param<std::string>("map_frame_id", _mapFrameId, "/map");
  nh.param<double>("min_person_height", _minPersonHeight, 1.37f);
  nh.param<std::string>("camera", _camera, "camera1");
  nh.param<bool>("register_all", _registerAll, false);
}

void Detector::run(ros::NodeHandle& node, ros::NodeHandle& nh_param) {
  getParams(nh_param);
  _transform = TransformProvider(_mapFrameId);
  _hogModel = new HogModel();
  _bsModel = new BsModel();

  _detector = new MultiscaleHogDetector(_transform,nh_param);
  
  _transport = new image_transport::ImageTransport(node);
  setCamera(_camera);

  _globalSub = node.subscribe("/bwi/person_detections/global", 1000, &Detector::processDetections, this);
  _registrationSub = node.subscribe("/bwi/registered_persons", 1000, &Detector::processPersonRegistration, this);
  _publisher = node.advertise<bwi_msgs::PersonDetectionArray&>("/bwi/person_detections/" + _camera, 1000);
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
  topic << "/" << camera << "/image_raw";
  return topic.str();
}

void Detector::setCamera(std::string camera) {
  _camSub.shutdown();
  _camera = camera;
  std::string topic = getImageTopic(_camera);
  _camSub = _transport->subscribeCamera(topic, 1, &Detector::processImage, this);
  ROS_INFO("Subscribed to camera topic %s", _camSub.getTopic().c_str());
}

void Detector::setRegisterAll(bool value) {
  _registerAll = value;
}
