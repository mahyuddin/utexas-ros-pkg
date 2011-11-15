#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Point.h>
#include "Detector.h"

int main(int argc, char **argv) {
  ros::init(argc,argv,"glados_person_detection");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  ros::Publisher pub = nh.advertise<geometry_msgs::Point>("glados_person_detection/person",5);
  Detector detector(&pub,true);
  image_transport::Subscriber sub = it.subscribe("glados_person_detection/image_raw",1,&Detector::detect,&detector);
 
  return 0;
}
