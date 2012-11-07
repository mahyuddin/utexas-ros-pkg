/**
 * \file  tf_to_pose.cc
 * \brief  This node converts a transform grabbed from the tf tree to a pose
 * message. Useful with fiducial marker systems.
 *
 * \author  Piyush Khandelwal (piyushk), piyushk@cs.utexas.edu
 * Copyright (C) 2011, The University of Texas at Austin, Piyush Khandelwal
 *
 * License: Modified BSD License
 *
 * $ Id: 11/03/2011 01:35:01 PM piyushk $
 */

#include <fstream>
#include <ros/ros.h>
#include <ros/assert.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <bwi_msgs/RobotDetectionArray.h>

#define NODE "robot_location_aggregator"

namespace {
  std::string robot_list_file;
  std::string parent_frame;
  double publish_frequency;
}

void getParams(ros::NodeHandle& nh) {
  nh.param<std::string>("robot_list_file", robot_list_file, "");
  nh.param<double>("publish_frequency", publish_frequency, 10);
}

int main (int argc, char **argv) {

  ros::init(argc, argv, NODE);
  ros::NodeHandle nh, nhParam("~");
  getParams(nhParam);

  tf::TransformListener listener;
  ros::Publisher posePublisher = 
      nh.advertise<bwi_msgs::RobotDetectionArray>("robot_detections", 1);

  // Get list of robots to monitor
  if (robot_list_file.empty()) {
    ROS_FATAL_STREAM("Robot list needs to be provided (~robot_list_file)");
  }
  // YAML::Node robot_list = YAML::LoadFile(robot_list_file); // Does this throw an exception if the file is not found?
  std::ifstream fin(robot_list_file.c_str());
  YAML::Parser parser(fin);
  YAML::Node robot_list;
  parser.GetNextDocument(robot_list);
  ROS_ASSERT(robot_list.Type() == YAML::NodeType::Map);

  ros::Rate rate(publish_frequency);

  while (ros::ok()) {

    rate.sleep();
    bwi_msgs::RobotDetectionArray out_msg;
 
    // for(YAML::const_iterator it = robot_list.begin(); it != robot_list.end(); ++it) {
    for(YAML::Iterator it = robot_list.begin(); it != robot_list.end(); ++it) {
      bwi_msgs::RobotDetection robot_msg;
      // std::string robot(it->first.as<std::string>()); // robot names are unique
      // std::string level(it->second.as<std::string>());
      std::string robot,level;
      it.first() >> robot;
      it.second() >> level;
      robot_msg.id = robot;
      robot_msg.level_id = level;

      std::string parent_frame = "/" + level + "/map";
      std::string child_frame = "/" + robot + "/base_footprint";

      // Get the latest transform from /parent to /child 
      tf::StampedTransform transform_parent_to_child;
      listener.waitForTransform(
          parent_frame, child_frame, ros::Time(0), ros::Duration(1.0)); 
      try {
        listener.lookupTransform(parent_frame, child_frame,
                                 ros::Time(), transform_parent_to_child);
      } catch (tf::TransformException ex) {
        ROS_INFO_STREAM("Transformation unavailable: " << ex.what());
        continue;
      }

      // tf::Quaternion q = transform_parent_to_child.getRotation();
      // tf::Quaternion yaw;
      // yaw.setEuler(tf::getYaw(q), 0, 0);
      tf::Vector3 v = transform_parent_to_child.getOrigin();

      // Fill in the pose message
      robot_msg.point.header.stamp = transform_parent_to_child.stamp_;
      robot_msg.point.header.frame_id = parent_frame;
      robot_msg.point.point.x = v.getX();
      robot_msg.point.point.y = v.getY();
      robot_msg.point.point.z = v.getZ();
      // pose.pose.pose.orientation.x = yaw.getX();
      // pose.pose.pose.orientation.y = yaw.getY();
      // pose.pose.pose.orientation.z = yaw.getZ();
      // pose.pose.pose.orientation.w = yaw.getW();
      out_msg.robots.push_back(robot_msg);
    }
    posePublisher.publish(out_msg);

  }

  return 0;
}
