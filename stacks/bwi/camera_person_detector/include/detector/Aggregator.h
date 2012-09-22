#ifndef AGGREGATOR_H
#define AGGREGATOR_H

#include <ros/ros.h>
#include <bwi_msgs/PersonDetectionArray.h>
#include <bwi_msgs/PersonDetection.h>

class Aggregator {

  public:
    Aggregator();
    void run(ros::NodeHandle&);
    void processDetections(const bwi_msgs::PersonDetectionArray&);

  private:
    ros::Publisher _publisher;

};

#endif
