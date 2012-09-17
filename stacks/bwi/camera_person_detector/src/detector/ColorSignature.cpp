#include "ColorSignature.h"

ColorSignature::ColorSignature(const bwi_msgs::ColorSignature& msg, GUID id) {
  _sigmsg = msg;
  _id = id;
}

ColorSignature::ColorSignature(cv::Mat& image, cv::Mat& mask, cv::Rect detection, GUID id) {
  for(int i = 0; i < SIGNATURE_SLICES; i++) _sigmsg.histograms.push_back(SigItem());
  FOREACH_SLICE(i) {
    int x = detection.x;
    int y = detection.y + i * detection.height / SIGNATURE_SLICES;
    int width = detection.width;
    int height = detection.height / SIGNATURE_SLICES;
    SigItem item = getSigItem(image, mask, cv::Rect(x,y,width,height));
    _sigmsg.histograms[i] = item;
  }
  _sigmsg.stamp = ros::Time::now();
  _sigmsg.rbins = HISTOGRAM_R;
  _sigmsg.gbins = HISTOGRAM_G;
  _sigmsg.bbins = HISTOGRAM_B;
  _id = id;
}

Color ColorSignature::getAverageColor(cv::Mat& image, cv::Rect slice) {
  uint g = 0, b = 0, r = 0;
  int count = 0;
  int w = slice.width / 2;
  int cx = slice.x + slice.width / 2;
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

SigItem ColorSignature::getSigItem(cv::Mat& image, cv::Mat& mask, cv::Rect slice) {
  SigItem item;
  for(int i = 0; i < HISTOGRAM_BINS; i++) item.bins.push_back(0);
  int total = 1;
  int w = slice.width / 3;
  int h = slice.height;
  int cx = slice.x + slice.width / 2;
  for(int x = cx - w; x < cx + w; x++) {
    for(int y = slice.y; y < slice.y + h; y++) {
      bool allow = mask.at<bool>(y,x);
      if(!allow) continue;
      Color pixel = image.at<Color>(y,x);
      total++;
      uchar b = pixel[0], g = pixel[1], r = pixel[2];
      int rbin = r / INTERVAL_R;
      int gbin = g / INTERVAL_G;
      int bbin = b / INTERVAL_B;
      item.bins[rbin + HISTOGRAM_R * gbin + HISTOGRAM_R * HISTOGRAM_G * bbin]++;
    }
  }
  for(int i = 0; i < HISTOGRAM_BINS; i++) item.bins[i] /= total;
  
  return item; 
}

bool ColorSignature::operator==(const ColorSignature &other) const {
  if(distance(other) < 40) return true;
  return false;
}

double ColorSignature::distance(const ColorSignature& other) const {
  double sqSum = 0;
  FOREACH_SLICE(i) {
    SigItem leftItem(_sigmsg.histograms[i]), rightItem(other._sigmsg.histograms[i]);
    for(int j = 0; j < HISTOGRAM_BINS; j++) {
      double lbin = leftItem.bins[j], rbin = rightItem.bins[j];
      if(lbin > 0 || rbin > 0)
        sqSum += pow(10 * (lbin - rbin), 2) / (lbin + rbin);
    }
  }
  double distance = sqSum;
  //ROS_INFO("dist between %i and %i: %2.2f", _sigmsg.id, other._sigmsg.id, distance);
  return distance; 
}

GUID ColorSignature::getId() {
  return _id;
}

void ColorSignature::setId(GUID id) {
  _id = id;
}

ros::Time ColorSignature::getStamp() {
  return _sigmsg.stamp;
}

void ColorSignature::update(const ColorSignature &other) {
  //_sigmsg.histograms = other._sigmsg.histograms;
  _sigmsg.stamp = other._sigmsg.stamp;
}

bwi_msgs::ColorSignature ColorSignature::getMsg() const {
  return _sigmsg;
}
