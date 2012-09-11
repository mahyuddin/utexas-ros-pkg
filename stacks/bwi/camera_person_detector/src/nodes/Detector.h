#ifndef DETECTOR_H
#define DETECTOR_H

#include <stdlib.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <boost/foreach.hpp>

#include "TransformProvider.h"
#include "ekf/Ekf.h"
#include "sp/SegmentationProcessor.h"
#include "Level.h"
#include "PersonIdentifier.h"
#include "MultiscaleHogDetector.h"

#define NODE "camera_transform_producer"
#define BS_HEIGHT_ADJUSTMENT 1.2

class Detector {
  private:
    cv::Mat _foreground;
    cv::BackgroundSubtractorMOG2 _mog;

    TransformProvider _transform;
    sp::SegmentationProcessor _processor;
    EkfManager _manager;
    PersonIdentifier _identifier;
    MultiscaleHogDetector* _detector;
    EkfModel *_hogModel, *_bsModel;

    double _frameRate;
    ros::Time _startTime;
    int _frameCount;
    double _minPersonHeight;
    std::string _mapFrameId;
    
    cv::Scalar generateColorFromId(unsigned int);
    cv::Rect correctForImage(cv::Rect, cv::Mat&);
    void drawEKF(cv::Mat&, cv::Mat&, cv::Mat&);
    std::vector<cv::Rect> detectBackground(cv::Mat&);
    std::vector<PersonReading> getReadingsFromDetections(cv::Mat&, cv::Mat&, std::vector<cv::Rect>, bool);
    void processImage(const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&);
    void getParams(ros::NodeHandle&);
  public:
    void run();
};

#endif
