#include "Detector.h"
#include <ros/ros.h>

#define NODE "camera_person_detector"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, NODE);
  ros::NodeHandle node, nh_param("~");
  
  Detector* detector = new Detector();
  detector->run(node, nh_param);
  
  ros::spin();
  return 0;
}
