#ifndef COLOR_SIGNATURE_H
#define COLOR_SIGNATURE_H

#include <opencv/cv.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>

#define SIMILARITY_THRESHOLD(slice) (5 + slice)
#define SIGNATURE_SLICES 5
#define ARE_SIMILAR(c1,c2,slice) (\
  abs(c1[0] - c2[0]) <= SIMILARITY_THRESHOLD(slice) && \
  abs(c1[1] - c2[1]) <= SIMILARITY_THRESHOLD(slice) && \
  abs(c1[2] - c2[2]) <= SIMILARITY_THRESHOLD(slice) \
  )
typedef cv::Vec3b Color;

class ColorSignature {
  public:
    ColorSignature(cv::Mat&,cv::Rect);
    bool operator==(const ColorSignature&) const;
    int getId();
    ros::Time getStamp();
  private:
    Color getAverageColor(cv::Mat&, cv::Rect);
    std::vector<Color> _means;
    ros::Time _stamp;
    int _id;
    static int _ID;
};

#endif
