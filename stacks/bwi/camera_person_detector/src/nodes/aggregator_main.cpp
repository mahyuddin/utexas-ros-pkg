#include "Aggregator.h"
#include <ros/ros.h>

#define NODE "camera_person_aggregator"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, NODE);
  ros::NodeHandle node, nh_param("~");
  
  Aggregator aggregator;
  aggregator.run(node, nh_param);
  
  ros::spin();
  return 0;
}
