#include "ColorSignature.h"

int ColorSignature::_ID = 1;

ColorSignature::ColorSignature(cv::Mat& image, cv::Rect detection) {
  // fill up _means
  for(int i = 0; i < SIGNATURE_SLICES; i++) {
    int x = detection.x;
    int y = detection.y + i * detection.height / SIGNATURE_SLICES;
    int width = detection.width;
    int height = detection.height / SIGNATURE_SLICES;
    Color c(getAverageColor(image,cv::Rect(x,y,width,height)));
    _means.push_back(c);
  }
  _id = _ID++;
  _stamp = ros::Time::now();
}

Color ColorSignature::getAverageColor(cv::Mat& image, cv::Rect slice) {
  uint g = 0, b = 0, r = 0;
  int count = 0;
  for(int x = slice.x; x < slice.x + slice.width; x++) {
    for(int y = slice.y; y < slice.y + slice.height; y++) {
      count++;
      Color pixel = image.at<Color>(y,x);
      g += pixel[0]; b += pixel[1]; r += pixel[2];
    }
  }
  g /= count; b /= count; r /= count;
  Color c(g,b,r);
  return c;
}


bool ColorSignature::operator==(const ColorSignature &other) const {
  //ROS_INFO("Beginning comparison, sig: %i vs other: %i", _id, other._id);
  for(size_t i = 0; i < SIGNATURE_SLICES; i++) {
    //ROS_INFO("sig: %i with other: %i, threshold %i", _means[i][0], other._means[i][0], SIMILARITY_THRESHOLD(i));
    //ROS_INFO("sig: %i with other: %i, threshold %i", _means[i][1], other._means[i][1], SIMILARITY_THRESHOLD(i));
    //ROS_INFO("sig: %i with other: %i, threshold %i", _means[i][2], other._means[i][2], SIMILARITY_THRESHOLD(i));
    //ROS_INFO("Similarity: %2.2f", COLOR_DISTANCE(_means[i],other._means[i],i));
    if(!ARE_SIMILAR(_means[i],other._means[i],i)) {
      return false;
    }
  }
  return true;
}

int ColorSignature::getId() {
  return _id;
}

ros::Time ColorSignature::getStamp() {
  return _stamp;
}

void ColorSignature::update(const ColorSignature &other) {
  _means = other._means;
  _stamp = other._stamp;
}
