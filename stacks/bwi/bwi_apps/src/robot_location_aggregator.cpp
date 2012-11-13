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

#include <boost/bind.hpp>
#include <boost/foreach.hpp>

#include <bwi_msgs/RobotDetection.h>
#include <bwi_msgs/RobotDetectionArray.h>
#include <bwi_utils/utils.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define NODE "robot_location_aggregator"

typedef std::pair<std::string, bwi_msgs::RobotDetection> DetectionPair;

namespace {
  std::string robot_list_file;
  std::string parent_frame;
  double publish_frequency;
  ros::Publisher pub;
  std::map<std::string, bwi_msgs::RobotDetection> detected_robots;
}

void getParams(ros::NodeHandle& nh) {
  nh.param<std::string>("robot_list_file", robot_list_file, "");
  nh.param<double>("publish_frequency", publish_frequency, 10);
}

void handleRobotDetections(std::string id, const geometry_msgs::PoseWithCovarianceStamped pose) {
  bwi_msgs::RobotDetection detection;
  detection.id = id;
  ROS_INFO("frame id: %s", pose.header.frame_id.c_str());
  detection.level_id = bwi_utils::levelIdFromFrameId(pose.header.frame_id);
  detection.point.header = pose.header;
  detection.point.point = pose.pose.pose.position;
  detected_robots[id] = detection;
}

void publishDetections() {
  bwi_msgs::RobotDetectionArray detections;
  BOOST_FOREACH(DetectionPair p, detected_robots) {
    detections.detections.push_back(p.second);
  }
  pub.publish(detections);
}  

int main (int argc, char **argv) {

  ros::init(argc, argv, NODE);
  ros::NodeHandle nh, nhParam("~");
  getParams(nhParam);

  if (robot_list_file.empty()) {
    ROS_FATAL_STREAM("Robot list needs to be provided (~robot_list_file)");
  }
  
  pub = nh.advertise<bwi_msgs::RobotDetectionArray>("/global/robot_detections", 1);
  std::vector<ros::Subscriber> subscribers;

  std::ifstream fin(robot_list_file.c_str());
  YAML::Parser parser(fin);
  YAML::Node robot_list;
  parser.GetNextDocument(robot_list);
  ROS_ASSERT(robot_list.Type() == YAML::NodeType::Sequence);
  for(YAML::Iterator it = robot_list.begin(); it != robot_list.end(); ++it) {
    std::string robot_id;
    *it >> robot_id;
    boost::function<void(const geometry_msgs::PoseWithCovarianceStamped)> f = boost::bind(&handleRobotDetections, robot_id, _1);
    ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(std::string("/") + robot_id + "/amcl_pose", 1000, f);
    subscribers.push_back(sub);
  }
  
  ros::Rate rate(publish_frequency);
  while(ros::ok()) {
    rate.sleep();
    ros::spinOnce();
    publishDetections();
  }
  return 0;
}
