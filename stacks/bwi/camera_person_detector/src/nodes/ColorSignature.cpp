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
  uint g,b,r;
  int count = 0;
  for(int x = slice.x; x <= slice.x + slice.width; x++) {
    for(int y = slice.y; y <= slice.y + slice.height; y++) {
      count++;
      Color pixel = image.at<Color>(x,y);
      g += pixel[0]; b += pixel[1]; r += pixel[2];
    }
  }
  Color c(g,b,r);
  return c;
}


bool ColorSignature::operator==(const ColorSignature &other) const {
  for(size_t i = 0; i < SIGNATURE_SLICES; i++)
    if(!ARE_SIMILAR(_means[i],other._means[i],i))
      return false;
  return true;
}

int ColorSignature::getId() {
  return _id;
}

ros::Time ColorSignature::getStamp() {
  return _stamp;
}
