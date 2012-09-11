#include "Detector.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, NODE);
  Detector detector;
  detector.run();
  ros::spin();
  return 0;
}
