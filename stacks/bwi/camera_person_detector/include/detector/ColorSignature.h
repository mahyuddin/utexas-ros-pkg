#ifndef COLOR_SIGNATURE_H
#define COLOR_SIGNATURE_H

#include <opencv/cv.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <bwi_msgs/ColorSignature.h>
#include <bwi_msgs/SignatureHistogram.h>
#include <boost/random/mersenne_twister.hpp>

#include "DetectorTypeDefs.h"

#define SIGNATURE_SLICES 1
#define START_SLICE 0
#define END_SLICE 1 
#define NUM_SLICES (START_SLICE - END_SLICE )
#define FOREACH_SLICE(i) for(int i=START_SLICE;i<END_SLICE;i++)

#define HISTOGRAM_R 4
#define HISTOGRAM_G 4
#define HISTOGRAM_B 4
#define INTERVAL_R (256.0 / HISTOGRAM_R)
#define INTERVAL_G (256.0 / HISTOGRAM_G)
#define INTERVAL_B (256.0 / HISTOGRAM_B)
#define HISTOGRAM_BINS (HISTOGRAM_R * HISTOGRAM_G * HISTOGRAM_B)

typedef cv::Vec3b Color;
typedef bwi_msgs::SignatureHistogram SigItem;

class ColorSignature {

  public:
    ColorSignature(const bwi_msgs::ColorSignature&, GUID);
    ColorSignature(cv::Mat&, cv::Mat&, cv::Rect, GUID);
    bool operator==(const ColorSignature&) const;
    double distance(const ColorSignature&) const;
    GUID getId();
    void setId(GUID);
    ros::Time getStamp();
    void update(const ColorSignature&);
    bwi_msgs::ColorSignature getMsg() const;
  private:
    Color getAverageColor(cv::Mat&, cv::Rect);
    SigItem getSigItem(cv::Mat&, cv::Mat&, cv::Rect);
    bwi_msgs::ColorSignature _sigmsg;
    GUID _id;
};

#endif
