#ifndef AGGREGATOR_H
#define AGGREGATOR_H

#include <ros/ros.h>
#include <bwi_msgs/PersonDetectionArray.h>
#include <bwi_msgs/PersonDetection.h>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

class Aggregator {

  public:
    Aggregator();
    void run(ros::NodeHandle&, ros::NodeHandle&);
    void processDetections(const bwi_msgs::PersonDetectionArray&);

  private:
    ros::Publisher _publisher;
    std::vector<ros::Subscriber> _subscribers;
};

#endif
