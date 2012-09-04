#include "ColorSignature.h"

int ColorSignature::_ID = 1;

ColorSignature::ColorSignature(cv::Mat& image, cv::Rect detection) {
  for(int i = 0; i < SIGNATURE_SLICES; i++) {
    int x = detection.x;
    int y = detection.y + i * detection.height / SIGNATURE_SLICES;
    int width = detection.width;
    int height = detection.height / SIGNATURE_SLICES;
    SigItem item = getSigItem(image,cv::Rect(x,y,width,height));
    _means.push_back(item);
  }
  _id = _ID++;
  _stamp = ros::Time::now();
}

Color ColorSignature::getAverageColor(cv::Mat& image, cv::Rect slice) {
  uint g = 0, b = 0, r = 0;
  int count = 0;
  int w = slice.width / 5;
  int cx = slice.x + slice.width / 2;
  int h = slice.height;
  for(int x = cx - w; x < cx + w; x++) {
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

SigItem ColorSignature::getSigItem(cv::Mat& image, cv::Rect slice) {
  Color color = getAverageColor(image,slice);
  double g = color[0], b = color[1], r = color[2];
  SigItem item(g - b, b - r);
  return item;
}

bool ColorSignature::operator==(const ColorSignature &other) const {
  float totalDistance = 0;
  for(size_t i = 0; i < USED_SLICES; i++) {
    totalDistance += sqrt(
      (_means[i][0] - other._means[i][0]) *
      (_means[i][0] - other._means[i][0]) +
      (_means[i][1] - other._means[i][1]) *
      (_means[i][1] - other._means[i][1])
    );
  }
  float avg = totalDistance / USED_SLICES;
  //ROS_INFO("%i vs %i: avg %2.2f", _id, other._id, avg); 
  if(fabs(avg) < 150) return true;
  return false;
}

int ColorSignature::getId() {
  return _id;
}

ros::Time ColorSignature::getStamp() {
  return _stamp;
}

void ColorSignature::update(const ColorSignature &other) {
  //_means = other._means;
  _stamp = other._stamp;
}
