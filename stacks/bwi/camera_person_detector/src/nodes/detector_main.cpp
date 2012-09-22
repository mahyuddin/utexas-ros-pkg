#include "Detector.h"
#include <ros/ros.h>

#define NODE "camera_transform_producer"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, NODE);
  ros::NodeHandle node, nh_param("~");
  
  Detector detector;
  detector.run(node, nh_param);
  
  ros::spin();
  return 0;
}
