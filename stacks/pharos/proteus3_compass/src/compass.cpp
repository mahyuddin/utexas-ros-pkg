#include <string>
#include <iostream>
#include <cstdio>

// OS Specific sleep
//#ifdef _WIN32
//#include <windows.h>
//#else
//#include <unistd.h>
//#endif

#include "serial/serial.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

int run(int argc, char **argv) {

  string port("/dev/ttyACM0");
  unsigned long baud = 115200;

  serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

  cout << "Is the serial port open?";
  if(my_serial.isOpen())
    cout << " Yes." << endl;
  else
    cout << " No." << endl;

  ros::init(argc, argv, "compass");
  ros::NodeHandle node;

  /*
   * Tell ROS that this node is going to publish messages on topic "compass".
   * The buffer size is 1000, meaning up to 1000 messages will be stored  
   * before throwing some away.
   */
  ros::Publisher chatter_pub = node.advertise<std_msgs::String>("compass", 1000);
  
  /*
   * Loop at 10Hz.
   */
  ros::Rate loop_rate(10);
  
  int count = 0;
  uint8_t *buff = new uint8_t[8];
  bool readMsg = 0;

  while (ros::ok()) {
    if (my_serial.available() >= 8) {
      size_t numBytes = my_serial.read(buff, 1);
      if (numBytes == 1) {
        if (buff[0] == 0x24) {
          readMsg = 1;
          numBytes = my_serial.read(&buff[1], 7);
          printf("Read %i bytes: ", numBytes);
          if (numBytes == 7) {
            for (uint8_t i = 0; i < 8; i++) {
              printf("0x%x ", buff[i]);
            }
            printf("\n"); 
            uint8_t checksum = 0;
            for (uint8_t i = 0; i < 7; i++) {
              checksum ^= buff[i];
            }
            printf("Checksum = 0x%x\n", checksum);  
            if (checksum == buff[7]) {
              printf("Checksum matched!\n");
            } else {
              printf("Checksum missmatch!\n");
            }
          }
        } else {
          printf("First byte not start byte, discard it...\n");
          readMsg = 0;
        }
      }
    }

    //string result = my_serial.readline(20, "\t");
    //cout << "Read " << result.length() << " bytes: " << result << endl;

    /*std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);
    */
    ros::spinOnce();
    
    // only sleep if a message was successfully read
    if (readMsg) 
      loop_rate.sleep();
    ++count;
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
