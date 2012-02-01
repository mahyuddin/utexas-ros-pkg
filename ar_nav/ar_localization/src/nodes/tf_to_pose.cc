/**
 * \file  tf_to_pose.cc
 * \brief  This file grabs the tf information published by the artoolkit object
 * detector and publishes out a readable message. The advantage of publishing this 
 * message is that it can be read by localization modules such as amcl to reinitialize
 * a particle filter.
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

#define NODE "ar_tf_to_pose"

namespace {
  std::string robotBaseFrame;
  std::string mapFrame;
  double publishFrequency;
  geometry_msgs::PoseWithCovarianceStamped pose;
}

void getParams(ros::NodeHandle& nh) {
  nh.param<std::string>("robotBaseFrame", robotBaseFrame, "robot");
  nh.param<std::string>("mapFrame", mapFrame, "map");
  nh.param<double>("publishFrequency", publishFrequency, 5);
}

int main (int argc, char **argv) {

  ros::init(argc, argv, NODE);
  ros::NodeHandle nh, nhParam("~");
  getParams(nhParam);

  tf::TransformListener listener;
  ros::Publisher posePublisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(robotBaseFrame, 1);

  ros::Rate rate(publishFrequency);

  while (ros::ok()) {

    rate.sleep();

    // Get the latest transform from /map to /marker 
    tf::StampedTransform transformMapToMarker;
    listener.waitForTransform(mapFrame, robotBaseFrame, ros::Time(), ros::Duration(1.0)); 
    try {
      listener.lookupTransform(mapFrame, robotBaseFrame,
                               ros::Time(), transformMapToMarker);
    } catch (tf::TransformException ex) {
      ROS_INFO("Transfromation unavailable: %s", ex.what());
      continue;
    }

    tf::Quaternion q = transformMapToMarker.getRotation();
    tf::Quaternion yaw;
    yaw.setEuler(tf::getYaw(q), 0, 0);
    tf::Vector3 v = transformMapToMarker.getOrigin();

    // Fill in the pose message - the covriance is currently not used
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = mapFrame;
    pose.pose.pose.position.x = v.getX();
    pose.pose.pose.position.y = v.getY();
    pose.pose.pose.position.z = 0; // For now remove the z component
    pose.pose.pose.orientation.x = yaw.getX();
    pose.pose.pose.orientation.y = yaw.getY();
    pose.pose.pose.orientation.z = yaw.getZ();
    pose.pose.pose.orientation.w = yaw.getW();
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
