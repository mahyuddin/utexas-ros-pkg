/**
 * \file  camera_transform_producer.cpp
 * \brief  Computes camera transform from correspondences provided by user 
 * in camera stream and a map of the world
 *
 * Copyright (C) 2012, UT Austin
 */

#include <ros/ros.h>

#define NODE "camera_transform_producer"



void getParams(ros::NodeHandle& nh) {
  nh.getParam("patternFile", map_file);
  nh.getParam("save_launch_file", save_launch_file);
  nh.getParam("launch_file_path", launch_file_path);
}

int main(int argc, const char *argv[]) {
  
  ros init(argc, argv, NODE);
  ros::NodeHandle nh, nh_param("~");
  getParams(nhParam);

  return 0;
}
