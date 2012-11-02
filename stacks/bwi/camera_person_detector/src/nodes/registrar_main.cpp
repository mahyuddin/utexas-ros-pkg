#include "Registrar.h"
#include <ros/ros.h>

#define NODE "person_registrar"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, NODE);
  ros::NodeHandle node, nh_param("~");
  
  Registrar registrar(node, nh_param);
  
  ros::spin();
  return 0;
}
