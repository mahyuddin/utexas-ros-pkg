/**
 * This ROS node does the following:
 *
 * 1. Reads in GPS data from a serial port
 * 2. Saves the data within a GPS message
 * 3. Publishes the message through ROS topic /gps/measurement
 *
 * The GPS is a LS23060 receiver, which generates NMEA sentences.
 * Some of the code below was taken from the Player GPS driver
 * under server/drivers/gps/garminnmea.cc.
 *
 * @author Chien-Liang Fok
 */
#include <string>
#include <iostream>
#include <cstdio>
#include <vector>
#include <sstream>

#include "serial/serial.h"

#include "ros/ros.h"
#include "proteus3_gps/gps.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

#define NMEA_GPGGA "$GPGGA"

// Filtered GPS geodetic coords; for outlier rejection
double filter_a, filter_thresh;
double filter_lat, filter_lon;
bool filter_good;

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while(std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}


std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    return split(s, delim, elems);
}

void parseGPGGA(const std::vector<std::string> tokens) {
  cout << "parseGPGGA called with the following tokens:" << endl;
  // Print the tokens (for debugging)
  for(std::vector<int>::size_type i = 0; i != tokens.size(); i++) {
    std::cout << "\t" << i << "\t" << tokens[i] << endl;
  }
  
  char tmp[8]; // A temporary working buffer for holding token fragments
  double degrees, minutes, arcseconds;
  double lat, lon;
  double utm_e, utm_n;

  // Field 1 is UTC Time in hhmmss.sss

  // Field 2 is the Latitude in ddmm.mmmm
  tokens[2].copy(tmp, 2); 
  tmp[2]='\0';
  degrees = atoi(tmp);
  minutes = atof(tokens[2].substr(2).c_str());
  arcseconds = ((degrees * 60.0) + minutes) * 60.0;

  // Field 3 is 'N' or 'S' for north or south. Adjust sign accordingly.
  if(tokens[3][0] == 'S')
    arcseconds *= -1;

  lat = arcseconds / 3600.0;
  cout << "\tlatitude: " << lat << endl;

  // Field 4 is the Longitude in dddmm.mmmm
  tokens[4].copy(tmp, 3);
  tmp[3]='\0';
  degrees = atoi(tmp);
  minutes = atof(tokens[4].substr(3).c_str());
  arcseconds = ((degrees * 60.0) + minutes) * 60.0;

  // Field 5 is 'E' or 'W' for east or west. Adjust sign accordingly.
  if(tokens[5][0] == 'W')
    arcseconds *= -1;

  lon = arcseconds / 3600.0;
  cout << "\tlongitude: " << lon << endl;
 
  // Field 6 is the fix indicator
  int fixInd = atoi(tokens[6].c_str());
  cout << "\tPosition fix indicator: " << fixInd << endl;
 
  // Field 7 is the number of satellites used
  int numSats = atoi(tokens[7].c_str());
  cout << "\tNum satellites: " << numSats << endl;

  // Field 8 is the HDOP (Horizontal Dilution of Precision)
  float hdop = atof(tokens[8].c_str());
  cout << "\tHDOP: " << hdop << endl;

  // Field 9 is the altitude in meters
  float altitude = atof(tokens[9].c_str());
  cout << "\tAltitude: " << altitude << endl;

  // Field 10 is the altitude's reference point, e.g., 'M' is
  // mean sea level.  Ignore it.

  // Field 11 is "geoid separation".  Ignore it.

  // Field 12 is the reference point for the above geoid separation.  Ignore it.

  // Field 13 is the differential GPS reference station ID.  Ignore it.

  // Field 14 is the checksum.  Ignore it.

  // Update the filtered lat/lon and see if the new values are any good
  filter_lat = filter_a * lat + (1 - filter_a) * filter_lat;
  filter_lon = filter_a * lon + (1 - filter_a) * filter_lon;

  // Reject outliers
  filter_good = true;
  if (fabs(lat - filter_lat) > filter_thresh)
    filter_good = false;
  if (fabs(lon - filter_lon) > filter_thresh)
    filter_good = false;

  if (!filter_good) {
    printf("Rejected: (%f, %f), expected (%f, %f)\n", lat, lon, filter_lat, filter_lon);
    return;
  } else {

    // Compute the UTM coordindates
    //UTM(lat, lon, &utm_e, &utm_n);
    //printf("utm: %.3f %.3f\n", utm_e, utm_n);

    //data.utm_e = (int32_t) rint(utm_e * 100);
    //data.utm_n = (int32_t) rint(utm_n * 100);
  // publish the data here!
  }
}

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

  //Initialize the GPS outlier detection variables
  filter_a = 0.80;
  filter_thresh = 1.0;
  filter_lat = 0;
  filter_lon = 0;

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
  bool readLine = false;

  while (ros::ok()) {
    if (my_serial.available() > 0) {
      string currLine = my_serial.readline(100, "\n");
      cout << "Read line: " << currLine;
      readLine = true;

      std::vector<std::string> tokens = split(currLine, ',');
      
      // Print the tokens (for debugging) 
      //for(std::vector<int>::size_type i = 0; i != tokens.size(); i++) {
      //  std::cout << "\t" << i << "\t" << tokens[i] << endl;
      //}

      if (!tokens[0].compare(NMEA_GPGGA))
        parseGPGGA(tokens);

    } else {
      readLine = false;
    }

    ros::spinOnce();
    
    // only sleep if a message was successfully read
    if (readLine) 
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
