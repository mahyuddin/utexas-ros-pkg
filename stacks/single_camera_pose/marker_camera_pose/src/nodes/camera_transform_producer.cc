/**
 * \file  camera_transform_producer.cc
 * \brief  Calculates position and orientation of any cameras using
 *         fiducial markers placed in known locations
 *
 * It is assumed that the fiducial marker system is publishing the information
 * on the tf tree for markers corresponding to the ones obtained from file
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

#include <camera_pose/pattern.h>

#define NODE "extrinsic_camera_calibrate"

namespace {
  std::string pattern_file;
  std::string camera_base_frame;
  std::string launch_file_path;
  std::string launch_camera_frame;
}

void getParams(ros::NodeHandle& nh) {
  nh.getParam("pattern_file", pattern_file);
  nh.getParam("camera_base_frame", camera_base_frame);
  nh.getParam("launch_file_path", launch_file_path);
  nh.getParam("launch_camera_frame", launch_camera_frame);
}

int main (int argc, char **argv) {

  ros::init(argc, argv, NODE);
  ros::NodeHandle nh, nhParam("~");
  getParams(nhParam);

  // Get information about the patterns
  std::vector<camera_pose::Pattern> patterns;
  camera_pose::readPatternFile(pattern_file, patterns);

  //tf::TransformListener listener(nh);
  tf::TransformListener listener;
  tf::Vector3 avg_translation(0,0,0);
  tf::Quaternion avg_rotation(0,0,0,0);
  int num_transformations = 0;

  // Get transformations for every marker
  BOOST_FOREACH(const camera_pose::Pattern& pattern, patterns) {

    ROS_DEBUG_STREAM("Evaulating marker: " << pattern.name);

    // Get transform from /marker to /camera, - published by the fiducial 
    // marker system
    tf::StampedTransform transform_cam_to_marker;
    listener.waitForTransform(
        pattern.name, camera_base_frame, ros::Time(), ros::Duration(1.0)); 
    try {
      listener.lookupTransform(camera_base_frame, pattern.name,
                               ros::Time(), transform_cam_to_marker);
    } catch (tf::TransformException ex) {
      ROS_ERROR_STREAM("Error getting transformation for " << 
          pattern.name.c_str() << ": " << ex.what());
      continue;
    }

    tf::Quaternion q = transform_cam_to_marker.getRotation();
    tf::Vector3 v = transform_cam_to_marker.getOrigin();
    ROS_DEBUG_STREAM("CamToMarker");
    ROS_DEBUG_STREAM("  - Translation: [" << v.getX() << ", " 
        << v.getY() << ", " << v.getZ() << "]");
    ROS_DEBUG_STREAM("  - Rotation: in Quaternion [" << q.getX() << ", " 
        << q.getY() << ", " << q.getZ() << ", " << q.getW() << "]");

    // Compute transform of /map to /marker
    tf::Quaternion no_rotation(0, 0, 0, 1);
    tf::Vector3 marker_position(
        -pattern.location.x, -pattern.location.y, -pattern.location.z);
    tf::Transform transform_marker_to_map(no_rotation, marker_position);

    q = transform_marker_to_map.getRotation();
    v = transform_marker_to_map.getOrigin();
    ROS_DEBUG_STREAM("  MarkerToMap");
    ROS_DEBUG_STREAM("  - Translation: [" << v.getX() << ", " 
        << v.getY() << ", " << v.getZ() << "]");
    ROS_DEBUG_STREAM("  - Rotation: in Quaternion [" << q.getX() << ", " 
        << q.getY() << ", " << q.getZ() << ", " << q.getW() << "]");

    // Compute transformation from /map to /camera
    tf::Transform transform_cam_to_map(
        tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0));
    transform_cam_to_map.mult(transform_cam_to_marker, transform_marker_to_map);
    q = transform_cam_to_map.getRotation();
    v = transform_cam_to_map.getOrigin();
    ROS_DEBUG_STREAM("  CamToMap");
    ROS_DEBUG_STREAM("  - Translation: [" << v.getX() << ", " 
        << v.getY() << ", " << v.getZ() << "]");
    ROS_DEBUG_STREAM("  - Rotation: in Quaternion [" << q.getX() << ", " 
        << q.getY() << ", " << q.getZ() << ", " << q.getW() << "]");

    tf::Transform transform_map_to_cam = transform_cam_to_map.inverse();
    q = transform_map_to_cam.getRotation();
    v = transform_map_to_cam.getOrigin();
    ROS_DEBUG_STREAM("  MapToCam");
    ROS_DEBUG_STREAM("  - Translation: [" << v.getX() << ", " 
        << v.getY() << ", " << v.getZ() << "]");
    ROS_DEBUG_STREAM("  - Rotation: in Quaternion [" << q.getX() << ", " 
        << q.getY() << ", " << q.getZ() << ", " << q.getW() << "]");

    num_transformations++;

    if (num_transformations == 1) {
      avg_translation = transform_map_to_cam.getOrigin();
      avg_rotation = transform_map_to_cam.getRotation();  
    } else {
      avg_translation += transform_map_to_cam.getOrigin();
      avg_rotation += transform_map_to_cam.getRotation();  
    }

  }

  if (num_transformations == 0) {
    ROS_ERROR("Unable to get any viable transforms from markers");
    return 0;
  }

  avg_translation *= 1.0 / num_transformations;
  avg_rotation *= 1.0 / num_transformations;

  // Write compute transformation as a launch file
  if (!launch_file_path.empty()) {
    ROS_INFO_STREAM(" Writing camera launch file: " << launch_file_path);
    // Write to file if provided
    std::ofstream fout(launch_file_path.c_str());
    fout << "<launch>" << std::endl;
    fout << "  <node pkg=\"tf\" type=\"static_transform_publisher\"\n"
         << "        name=\"link_" << launch_camera_frame << "_broadcaster\"\n"
         << "        args=\"" << avg_translation.x() << " " 
         << avg_translation.y() << " " << avg_translation.z() << " " 
         << avg_rotation.getX() << " " << avg_rotation.getY() << " " 
         << avg_rotation.getZ() << " " << avg_rotation.getW() << " /map /" 
         << launch_camera_frame << " 100\" />" << std::endl;
    fout << "</launch>" << std::endl;
    fout.close();
  }

  return 0;
}
