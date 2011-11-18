#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <glados_person_detection/Detector.h>

int main(int argc, char **argv) {
  ros::init(argc,argv,"glados_person_detection");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("glados_person_detection/person",5);
  bool doDisplay;
  ros::param::param("~display",doDisplay,true);
  Detector detector(&pub,doDisplay);
  image_transport::CameraSubscriber sub = it.subscribeCamera("glados_person_detection/image_raw",1,&Detector::detect,&detector);
  ros::spin();
  return 0;
}
