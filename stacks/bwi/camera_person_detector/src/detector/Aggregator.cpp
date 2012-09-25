#include "Aggregator.h"

#include <std_msgs/String.h>

typedef std::pair<const std::string, XmlRpc::XmlRpcValue> XmlRpcPair; 

Aggregator::Aggregator() {
}

void Aggregator::run(ros::NodeHandle& node) {
  XmlRpc::XmlRpcValue cameras;
  node.getParam("~cameras", cameras);
  //BOOST_FOREACH(XmlRpcPair xrv, cameras) {
  for(int i = 0; i < cameras.size(); i++) {
    std::string camera = static_cast<std::string>(cameras[i]);
    ros::Subscriber s = node.subscribe("/bwi/person_detections/" + camera, 1000, &Aggregator::processDetections, this);
  }
  _publisher = node.advertise<bwi_msgs::PersonDetectionArray>("/bwi/person_detections/global", 1000);
  ros::spin();
}

void Aggregator::processDetections(const bwi_msgs::PersonDetectionArray& detections) {
  _publisher.publish(detections);
}
