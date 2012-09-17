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
#include <boost/function.hpp>

#include "TransformProvider.h"
#include "ekf/Ekf.h"
#include "sp/SegmentationProcessor.h"
#include "Level.h"
#include "PersonIdentifier.h"
#include "MultiscaleHogDetector.h"

#include <bwi_msgs/PersonDetection.h>
#include <bwi_msgs/BoundingBox.h>

#define BS_HEIGHT_ADJUSTMENT 1.2
#define CALLBACK_ARGS std::vector<bwi_msgs::PersonDetection>&, cv::Mat&, cv::Mat&

class Detector {
  private:
    cv::Mat _foreground;
    cv::BackgroundSubtractorMOG2 _mog;

    ros::Publisher _publisher;

    TransformProvider _transform;
    sp::SegmentationProcessor _processor;
    EkfManager _manager;
    PersonIdentifier _identifier;
    MultiscaleHogDetector* _detector;
    EkfModel *_hogModel, *_bsModel;

    double _minPersonHeight;
    std::string _mapFrameId;
    
    boost::function<void (CALLBACK_ARGS)> _callback;
    
    bwi_msgs::BoundingBox getBB(int x, int y, int width, int height, cv::Mat&);
    std::vector<cv::Rect> detectBackground(cv::Mat&);
    std::vector<PersonReading> getReadingsFromDetections(cv::Mat&, cv::Mat&, std::vector<cv::Rect>, bool);
    void processImage(const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&);
    void processDetection(const bwi_msgs::PersonDetection&);
    void getParams(ros::NodeHandle&);
    void broadcast(cv::Mat&, cv::Mat&);
  public:
    void run(ros::NodeHandle&,ros::NodeHandle&);
    void setCallback(boost::function<void (CALLBACK_ARGS)>);
};

#endif
