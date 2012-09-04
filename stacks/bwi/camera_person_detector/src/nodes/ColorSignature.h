#ifndef COLOR_SIGNATURE_H
#define COLOR_SIGNATURE_H

#include <opencv/cv.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>

#define SIMILARITY_THRESHOLD(slice)  (slice <= 1 ? 10 : slice * 10)
#define SIGNATURE_SLICES 5
#define USED_SLICES 3
#define COLOR_DISTANCE(c1,c2,slice) (\
  sqrt((c1[0] - c2[0]) * (c1[0] - c2[0]) + (c1[1] - c2[1]) * (c1[1] - c2[1]) + (c1[2] - c2[2]) * (c1[2] - c2[2])) \
)
#define ARE_SIMILAR(c1,c2,slice) (COLOR_DISTANCE(c1,c2,slice) < SIMILARITY_THRESHOLD(slice))

typedef cv::Vec3b Color;
typedef cv::Vec2b SigItem;

class ColorSignature {

  public:
    ColorSignature(cv::Mat&,cv::Rect);
    bool operator==(const ColorSignature&) const;
    int getId();
    ros::Time getStamp();
    void update(const ColorSignature&);
  private:
    Color getAverageColor(cv::Mat&, cv::Rect);
    SigItem getSigItem(cv::Mat&, cv::Rect);
    std::vector<SigItem> _means;
    ros::Time _stamp;
    int _id;
    static int _ID;
};

#endif
