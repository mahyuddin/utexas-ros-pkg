/**
 * \file  calibrate.cc
 * \brief  Calculates position and orientation of any cameras using
 *         ARTags placed in known locations
 *
 * The ar_multi node is laucnched automatically using roslaunch, and the
 * configuration is provided using a YAML file. ar_multi should already be
 * publishing this information on tf.
 *
 * A static transformation publisher should then be publishing the transform
 * between marker[i] -> world[i] (since the parent of the tree structure 
 * will be base_link on the camera, we require multiple worlds here)
 *
 * In this node we read the yaml file to figure out the number of markers
 * available, and read the corresponding transformations available from the
 * tf tree (i.e. world[i] -> camera_base_link).
 *
 * We average accross all of these to reduce error, and write out the
 * /world -> /camera_base_link transformation to file. The camera is
 * calibrated!
 *
 * \author  Piyush Khandelwal (piyushk), piyushk@cs.utexas.edu
 * Copyright (C) 2011, The University of Texas at Austin, Piyush Khandelwal
 *
 * License: Modified BSD License
 *
 * $ Id: 09/30/2011 11:25:52 AM piyushk $
 */


#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/package.h>

#include <boost/foreach.hpp>

#include <ar_nav_common/PatternParser.h>

#define NODE "ar_localization_calibrate"

namespace {
  std::string patternFile;
  std::string cameraBaseFrame;
  std::string cameraId;
}

void getParams(ros::NodeHandle& nh) {
  nh.getParam("patternFile", patternFile);
  nh.getParam("cameraBaseFrame", cameraBaseFrame);
  nh.getParam("cameraId", cameraId);
}

int main (int argc, char **argv) {

  ros::init(argc, argv, NODE);
  ros::NodeHandle nh, nhParam("~");
  getParams(nhParam);

  // Get information about the patterns
  std::vector<ar_nav::Pattern> patterns;
  ar_nav::readPatternFile(patternFile, patterns);

  //tf::TransformListener listener(nh);
  tf::TransformListener listener;
  tf::Vector3 averageTranslation(0,0,0);
  tf::Quaternion averageRotation(0,0,0,0);
  int numTransformations = 0;

  // Get transformations for every marker
  BOOST_FOREACH(const ar_nav::Pattern& pattern, patterns) {

    std::cout << pattern.name << std::endl;

    // Get transform from /marker to /camera
    tf::StampedTransform transformCamToMarker;
    listener.waitForTransform(pattern.name, cameraBaseFrame, ros::Time(), ros::Duration(1.0)); 
    try {
      listener.lookupTransform(cameraBaseFrame, pattern.name,
                               ros::Time(), transformCamToMarker);
    } catch (tf::TransformException ex) {
      ROS_ERROR("Error getting transformation for %s: %s", pattern.name.c_str(), ex.what());
      continue;
    }

    tf::Quaternion q = transformCamToMarker.getRotation();
    tf::Vector3 v = transformCamToMarker.getOrigin();
    std::cout << "  CamToMarker" << std::endl;
    std::cout << "  - Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
    std::cout << "  - Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
	  << q.getZ() << ", " << q.getW() << "]" << std::endl;

    // Compute transform of /map to /marker
    tf::Quaternion noRotation(0, 0, 0, 1);
    tf::Vector3 markerPosition(-pattern.location.x, -pattern.location.y, -pattern.location.z);
    tf::Transform transformMarkerToMap(noRotation, markerPosition);

    q = transformMarkerToMap.getRotation();
    v = transformMarkerToMap.getOrigin();
    std::cout << "  MarkerToMap" << std::endl;
    std::cout << "  - Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
    std::cout << "  - Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
	  << q.getZ() << ", " << q.getW() << "]" << std::endl;

    // Compute transformation from /map to /camera
    tf::Transform transformCamToMap(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0));
    transformCamToMap.mult(transformCamToMarker, transformMarkerToMap);
    q = transformCamToMap.getRotation();
    v = transformCamToMap.getOrigin();
    std::cout << "  CamToMap" << std::endl;
    std::cout << "  - Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
    std::cout << "  - Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
	  << q.getZ() << ", " << q.getW() << "]" << std::endl;

    tf::Transform transformMapToCam = transformCamToMap.inverse();
    q = transformMapToCam.getRotation();
    v = transformMapToCam.getOrigin();
    std::cout << "  MapToCam" << std::endl;
    std::cout << "  - Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
    std::cout << "  - Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
	  << q.getZ() << ", " << q.getW() << "]" << std::endl;

    numTransformations++;

    if (numTransformations == 1) {
      averageTranslation = transformMapToCam.getOrigin();
      averageRotation = transformMapToCam.getRotation();  
    } else {
      averageTranslation += transformMapToCam.getOrigin();
      averageRotation += transformMapToCam.getRotation();  
    }

  }

  if (numTransformations == 0) {
    ROS_ERROR("Unable to get any viable transforms from markers");
    return 0;
  }

  averageTranslation *= 1.0 / numTransformations;
  averageRotation *= 1.0 / numTransformations;

  std::ofstream fout((ros::package::getPath("ar_localization") + "/launch/" + cameraId + "-tf.launch").c_str());

  fout << "<launch>" << std::endl;
  fout << "  <node pkg=\"tf\" type=\"static_transform_publisher\" name=\"link_" << cameraId << "_broadcaster\" "
       << "args=\"" << averageTranslation.x() << " " << averageTranslation.y() << " " << averageTranslation.z()
       << " " << averageRotation.getX() << " " << averageRotation.getY() << " " << averageRotation.getZ() << " "
       << averageRotation.getW() << " map " << cameraBaseFrame << " 100\" />" << std::endl;
  fout << "</launch>" << std::endl;
  fout.close();

  return 0;
}
