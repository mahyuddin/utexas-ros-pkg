#include "Detector.h"

bwi_msgs::BoundingBox Detector::getBB(int x, int y, int width, int height, cv::Mat& image) {
  x = std::max(x, 0);
  y = std::max(y, 0);
  x = std::min(x, image.cols - width);
  y = std::min(y, image.rows - height);
  bwi_msgs::BoundingBox bb;
  bb.x = x; bb.y = y; bb.width = width; bb.height = height;
  return bb;
}

void Detector::broadcast(cv::Mat& image, cv::Mat& foreground) {
  std::vector<bwi_msgs::PersonDetection> detections;
  BOOST_FOREACH(PersonEkf* filter, _manager.getValidEstimates()) {  
    bwi_msgs::PersonDetection detection;
    
    BFL::Pdf<MatrixWrapper::ColumnVector>* posterior = filter->PostGet();
    MatrixWrapper::ColumnVector mean = posterior->ExpectedValueGet();
    
    double x = mean(1), y = mean(2), height = mean(5);
    int id = filter->getId();
    //detection.reading = PersonReading(x,y,height,id);
    detection.id = filter->getId();
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
    detection.signatures = _identifier.getSignaturesById(id);
    detections.push_back(detection);
    
    _publisher.publish(detection);
  }
  if(_callback) _callback(detections,image,foreground);
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

std::vector<PersonReading> Detector::getReadingsFromDetections(cv::Mat& image, cv::Mat& foreground, std::vector<cv::Rect> detections, bool getId = false) {
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

void Detector::processImage(const sensor_msgs::ImageConstPtr& msg,
    const sensor_msgs::CameraInfoConstPtr& cam_info) {
  cv_bridge::CvImageConstPtr cameraImagePtr = cv_bridge::toCvShare(msg, "bgr8");
  cv::Mat original(cameraImagePtr->image); 
  _transform.computeModel(cam_info);

  // Apply background subtraction along with some filtering to detect person
  cv::Mat foreground(_foreground);
  _mog(original, foreground, -1);
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

  std::vector<cv::Rect> hog_locations, bs_locations;
  hog_locations = _detector->detectMultiScale(gray_image);
  bs_locations = detectBackground(foreground);

  cv::Mat display_foreground = original.clone();
  for(int i = 0; i < original.rows; i++) {
    for(int j = 0; j < original.cols; j++) {
      if(!foreground.at<bool>(i,j))
        display_foreground.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
    }
  }
  _manager.updateFilters(getReadingsFromDetections(original, foreground, hog_locations), _hogModel);
  _manager.updateFilters(getReadingsFromDetections(original, foreground, bs_locations, true), _bsModel);
  broadcast(original, display_foreground);
}

void Detector::processDetection(const bwi_msgs::PersonDetection& detection) {
  _identifier.registerSignatures(detection); 
}

void Detector::getParams(ros::NodeHandle& nh) {
  nh.param<std::string>("map_frame_id", _mapFrameId, "/map");
  nh.param<double>("min_person_height", _minPersonHeight, 1.37f);
}

void Detector::run(ros::NodeHandle& node, ros::NodeHandle& nh_param) {
  getParams(nh_param);
  _transform = TransformProvider(_mapFrameId);
  _hogModel = new HogModel();
  _bsModel = new BsModel();

  _detector = new MultiscaleHogDetector(_transform,nh_param);
  
  // subscribe to the camera image stream to setup correspondences
  image_transport::ImageTransport it(node);
  std::string image_topic = node.resolveName("image_raw");

  image_transport::CameraSubscriber image_subscriber = 
     it.subscribeCamera(image_topic, 1, &Detector::processImage, this);
  node.subscribe("bwi/global_detections", 1000, &Detector::processDetection, this);
  _publisher = node.advertise<bwi_msgs::PersonDetection>("bwi/local_detections", 1000);
}

void Detector::setCallback(boost::function<void (CALLBACK_ARGS)> callback) {
  _callback = callback;
}

