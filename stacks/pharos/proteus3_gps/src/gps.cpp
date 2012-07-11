/**
 * This ROS node does the following:
 *
 * 1. Reads in GPS data from a serial port
 * 2. Saves the data within a GPS message
 * 3. Publishes the message through ROS topic /gps/measurement
 *
 * The GPS is a LS23060 receiver.
 *
 * @author Chien-Liang Fok
 */
#include <string>
#include <iostream>
#include <cstdio>

#include "serial/serial.h"

#include "ros/ros.h"
#include "proteus3_gps/gps.h"
#include <sstream>

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

int run(int argc, char **argv) {
  ros::init(argc, argv, "gps");

  // This node handle's namespace is "gps".
  // See:  http://www.ros.org/wiki/roscpp/Overview/NodeHandles#Namespaces
  ros::NodeHandle node("gps");

  // Get the parameters
  // See: http://www.ros.org/wiki/roscpp/Overview/Parameter%20Server
  std::string port;
  node.param<std::string>("port", port, "/dev/ttyUSB0");
  cout << "Port: " << port << endl;
  
  int baud = 57600;
  node.param<int>("baud", baud, 57600);
  cout << "Baud: " << baud << endl;

  serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

  cout << "Is the serial port open?";
  if(my_serial.isOpen())
    cout << " Yes." << endl;
  else
    cout << " No." << endl;

  /*
   * Tell ROS that this node is going to publish messages on topic "measurement".
   * The buffer size is 1000, meaning up to 1000 messages will be stored  
   * before throwing any away.
   */
  ros::Publisher chatter_pub = node.advertise<proteus3_gps::gps>("measurement", 1000);
  
  /*
   * Loop at 20Hz.
   */
  ros::Rate loop_rate(20);
  
  //int count = 0;
  //uint8_t *buff = new uint8_t[COMPASS_MESSAGE_SIZE];
  bool readMsg = false;

  while (ros::ok()) {
    if (my_serial.available() >= 8) {
      string result = my_serial.readline(100, "\n");
      cout << "Read line: " << result << endl;
    }

    ros::spinOnce();
    
    // only sleep if a message was successfully read
    if (readMsg) 
      loop_rate.sleep();
  }

  return 0;
}

int main(int argc, char **argv) {
  try {
    return run(argc, argv);
  } catch (exception &e) {
    cerr << "Unhandled Exception: " << e.what() << endl;
  }
  return 0;
}
