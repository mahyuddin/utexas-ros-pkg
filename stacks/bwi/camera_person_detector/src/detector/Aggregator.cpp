#include "Aggregator.h"

#include <std_msgs/String.h>

typedef std::pair<const std::string, XmlRpc::XmlRpcValue> XmlRpcPair; 

Aggregator::Aggregator() {
}

void Aggregator::run(ros::NodeHandle& nh, ros::NodeHandle& nh_param) {
  XmlRpc::XmlRpcValue cameras;
  nh_param.getParam("cameras", cameras);
  for(int i = 0; i < cameras.size(); i++) {
    std::string camera = static_cast<std::string>(cameras[i]);
    ros::Subscriber s = nh.subscribe("/bwi/person_detections/" + camera, 1000, &Aggregator::processDetections, this);
    ROS_INFO("aggregator subscribed to %s", s.getTopic().c_str());
    _subscribers.push_back(s);
  }
  _publisher = nh.advertise<bwi_msgs::PersonDetectionArray>("/bwi/person_detections/global", 1000);
  ros::spin();
}

void Aggregator::processDetections(const bwi_msgs::PersonDetectionArray& detections) {
  _publisher.publish(detections);
}
