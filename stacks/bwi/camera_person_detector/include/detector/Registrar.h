#ifndef SIGNIN_H
#define SIGNIN_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <bwi_msgs/PersonDetectionArray.h>
#include <bwi_msgs/PersonDetection.h>
#include <bwi_msgs/PersonDescriptor.h>
#include <bwi_msgs/ColorSignature.h>

#include "PersonIdentifier.h"
#include "Detector.h"

class Registrar {

  public:
    Registrar(ros::NodeHandle&, ros::NodeHandle&);
    void setCamera(std::string);
    void collectSignatures(ros::Duration);
    void processDetections(const bwi_msgs::PersonDetectionArray&);
    void start();
    bool register_start(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
    void stop();
    bool register_stop(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
  private:
    ros::Subscriber _subscriber;
    ros::Publisher _publisher;
    ros::ServiceServer _collectService, _stopService;
    std::string _camera;
    ros::NodeHandle& _nh;

    PersonIdentifier _identifier;

    bool _collect;

    std::vector<bwi_msgs::ColorSignature> _signatures;
};

#endif
