#include "Aggregator.h"

Aggregator::Aggregator() {
}

void Aggregator::run(ros::NodeHandle& node) {
  ros::Subscriber s = node.subscribe("bwi/person_detections/local", 1000, &Aggregator::processDetections, this);
  ROS_INFO("sub topic: %s, %i publishers", s.getTopic().c_str(), s.getNumPublishers());
  _publisher = node.advertise<bwi_msgs::PersonDetectionArray>("/bwi/person_detections/global", 1000);
  ROS_INFO("running aggregator");
}

void Aggregator::processDetections(const bwi_msgs::PersonDetectionArray& detections) {
  //_publisher.publish(detections);
  ROS_INFO("publishing detections");
}
