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

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define NODE "tf_to_pose"

namespace {
  std::string child_frame;
  std::string parent_frame;
  double publish_frequency;
  geometry_msgs::PoseWithCovarianceStamped pose;
}

void getParams(ros::NodeHandle& nh) {
  nh.param<std::string>("child_frame", child_frame, "/child");
  nh.param<std::string>("parent_frame", parent_frame, "/parent");
  nh.param<double>("publish_frequency", publish_frequency, 5);
}

int main (int argc, char **argv) {

  ros::init(argc, argv, NODE);
  ros::NodeHandle nh, nhParam("~");
  getParams(nhParam);

  tf::TransformListener listener;
  ros::Publisher posePublisher = 
      nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(child_frame, 1);

  ros::Rate rate(publish_frequency);

  while (ros::ok()) {

    rate.sleep();

    // Get the latest transform from /parent to /child 
    tf::StampedTransform transform_parent_to_child;
    listener.waitForTransform(
        parent_frame, child_frame, ros::Time(), ros::Duration(1.0)); 
    try {
      listener.lookupTransform(parent_frame, child_frame,
                               ros::Time(), transform_parent_to_child);
    } catch (tf::TransformException ex) {
      ROS_INFO_STREAM("Transformation unavailable: " << ex.what());
      continue;
    }

    tf::Quaternion q = transform_parent_to_child.getRotation();
    tf::Quaternion yaw;
    yaw.setEuler(tf::getYaw(q), 0, 0);
    tf::Vector3 v = transform_parent_to_child.getOrigin();

    // Fill in the pose message
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = parent_frame;
    pose.pose.pose.position.x = v.getX();
    pose.pose.pose.position.y = v.getY();
    pose.pose.pose.position.z = v.getZ();
    pose.pose.pose.orientation.x = yaw.getX();
    pose.pose.pose.orientation.y = yaw.getY();
    pose.pose.pose.orientation.z = yaw.getZ();
    pose.pose.pose.orientation.w = yaw.getW();

    // TODO: do something intelligent about the covariance?
    float covariance[] = {
      1e-3, 0, 0, 0, 0, 0,
      0, 1e-3, 0, 0, 0, 0,
      0, 0, 1e-3, 0, 0, 0,
      0, 0, 0, 1e-3, 0, 0,
      0, 0, 0, 0, 1e-3, 0,
      0, 0, 0, 0, 0, 1e-3
    };
    for (unsigned int i = 0; i < pose.pose.covariance.size(); i++) {
      pose.pose.covariance[i] = covariance[i];
    }

    posePublisher.publish(pose);

  }

  return 0;
}
