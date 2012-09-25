#ifndef SIGNIN_H
#define SIGNIN_H

#include <ros/ros.h>
#include <bwi_msgs/PersonDetectionArray.h>
#include <bwi_msgs/PersonDetection.h>
#include <bwi_msgs/PersonDescriptor.h>
#include <bwi_msgs/ColorSignature.h>

#include "PersonIdentifier.h"
#include "Detector.h"

class Signin {

  public:
    Signin(ros::NodeHandle&, ros::NodeHandle&);
    void attachToCamera(std::string);
    void collectSignatures(ros::Duration);
    void processDetections(const bwi_msgs::PersonDetectionArray&);
    void collect();
    void stop();
  private:
    ros::Subscriber _subscriber;
    ros::Publisher _publisher;
    std::string _camera;
    ros::NodeHandle& _nh;

    PersonIdentifier _identifier;
    Detector* _detector;

    bool _collect;

    std::vector<bwi_msgs::ColorSignature> _signatures;
};

#endif
