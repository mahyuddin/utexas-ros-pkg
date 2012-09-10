#include "ColorSignature.h"

int ColorSignature::_ID = 1;

//ColorSignature::ColorSignature(cv::Mat& image, cv::Rect detection) {
  ////cv::Mat yuvImage(image.rows,image.cols, image.type());
  ////cvtColor(image, yuvImage, CV_BGR2YCrCb);
  //for(int i = 0; i < SIGNATURE_SLICES; i++) _items.push_back(SigItem());
  //FOREACH_SLICE(i) {
    //int x = detection.x;
    //int y = detection.y + i * detection.height / SIGNATURE_SLICES;
    //int width = detection.width;
    //int height = detection.height / SIGNATURE_SLICES;
    //SigItem item = getSigItem(image, cv::Rect(x,y,width,height));
    //_items[i] = item;
    ////for(int y = 0; y < HISTOGRAM_R; y++)
      ////for(int u = 0; u < HISTOGRAM_G; u++)
        ////for(int v = 0; v < HISTOGRAM_B; v++)
          ////if(item[y + u * HISTOGRAM_R + v * HISTOGRAM_G * HISTOGRAM_Y] > 0)
            ////ROS_INFO("%i[%i,%i,%i] = %2.2f", _ID + 1, y, u, v, item[y + u * HISTOGRAM_R + v * HISTOGRAM_G * HISTOGRAM_Y]);
  //}
  //_id = _ID++;
  //_stamp = ros::Time::now();
//}

ColorSignature::ColorSignature(cv::Mat& image, cv::Mat& mask, cv::Rect detection) {
  //cv::Mat yuvImage(image.rows,image.cols, image.type());
  //cvtColor(image, yuvImage, CV_BGR2YCrCb);
  for(int i = 0; i < SIGNATURE_SLICES; i++) _items.push_back(SigItem());
  FOREACH_SLICE(i) {
    int x = detection.x;
    int y = detection.y + i * detection.height / SIGNATURE_SLICES;
    int width = detection.width;
    int height = detection.height / SIGNATURE_SLICES;
    SigItem item = getSigItem(image, mask, cv::Rect(x,y,width,height));
    _items[i] = item;
    //for(int y = 0; y < HISTOGRAM_R; y++)
      //for(int u = 0; u < HISTOGRAM_G; u++)
        //for(int v = 0; v < HISTOGRAM_B; v++)
          //if(item[y + u * HISTOGRAM_R + v * HISTOGRAM_R * HISTOGRAM_G] > 0)
            //ROS_INFO("%i[%i,%i,%i] = %2.2f", _ID + 1, y, u, v, item[y + u * HISTOGRAM_R + v * HISTOGRAM_G * HISTOGRAM_G]);
  }
  _id = _ID++;
  _stamp = ros::Time::now();
}

Color ColorSignature::getAverageColor(cv::Mat& image, cv::Rect slice) {
  uint g = 0, b = 0, r = 0;
  int count = 0;
  int w = slice.width / 2;
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


//SigItem ColorSignature::getSigItem(cv::Mat& image, cv::Rect slice) {
  //static cv::Mat* mask = 0;
  //if(!mask) {
    //mask = new cv::Mat(image.rows,image.cols,CV_8U);
    //for(int i = 0; i < image.rows; i++)
      //for(int j = 0; j < image.cols; j++)
        //mask->at<bool>(i,j) = true;
  //}
  //return getSigItem(image,*mask,slice);
//}

SigItem ColorSignature::getSigItem(cv::Mat& image, cv::Mat& mask, cv::Rect slice) {
  SigItem item;
  for(int i = 0; i < HISTOGRAM_BINS; i++) item.push_back(0);
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
      //ROS_INFO("yuv: (%i,%i,%i), ncrementing bin (%i,%i,%i)", 
          ////r, g, b,
          //y,u,v,
          //ybin, ubin, vbin);
      item[rbin + HISTOGRAM_R * gbin + HISTOGRAM_R * HISTOGRAM_G * bbin]++;
    }
  }
  for(int i = 0; i < HISTOGRAM_BINS; i++) item[i] /= total;
  
  return item; 
}

bool ColorSignature::operator==(const ColorSignature &other) const {
  if(distance(other) < 100) return true;
  return false;
}

double ColorSignature::distance(const ColorSignature& other) const {
  double sqSum = 0;
  FOREACH_SLICE(i) {
    SigItem left(_items[i]), right(other._items[i]);
    for(int j = 0; j < HISTOGRAM_BINS; j++)
      if(left[j] > 0 || right[j] > 0)
        sqSum +=  pow(10 * (left[j] - right[j]),2) / (left[j] + right[j]);
  }
  double distance = sqSum;
  ROS_INFO("dist between %i and %i: %2.2f", _id, other._id, distance);
  return distance; 
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
