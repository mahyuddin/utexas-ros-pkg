#ifndef MULTISCALE_HOG_DETECTOR
#define MULTISCALE_HOG_DETECTOR

#include <tf/tf.h>
#include <opencv/cv.h>
#include <boost/foreach.hpp>

#include "TransformProvider.h"
#include "Level.h"

// The hog detector needs a window that's slightly larger 
// than the detected person. Because we calculate height 
// based on this window size, we need to adjust the height 
// after detection.
#define HOG_HEIGHT_ADJUSTMENT .87 

class MultiscaleHogDetector {
  private:
    boost::shared_ptr<cv::HOGDescriptor> _hog;
    TransformProvider& _transform;
    std::vector<Level> _levels;
    ros::NodeHandle& _nh;
    bool _searchSpaceCalculated;
    void detect(cv::Mat&,Level&,std::vector<cv::Rect>&,std::vector<double>&);
    void initialize();
    
    int _windowStride, _windowWidth, _windowHeight, _minGroupRectangles;
    double _weightThreshold, _hitThreshold, _groupEps;
  public:
    MultiscaleHogDetector(TransformProvider&,ros::NodeHandle&);
    std::vector<cv::Rect> detectMultiScale(cv::Mat&);
    bool isSearchSpaceCalculated();
    void calculateSearchSpace(int,int);
};

#endif

