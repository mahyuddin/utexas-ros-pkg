#include "Aggregator.h"
#include <ros/ros.h>

#define NODE "camera_person_aggregator"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, NODE);
  ros::NodeHandle node;
  
  Aggregator aggregator;
  aggregator.run(node);
  
  ros::spin();
  return 0;
}
