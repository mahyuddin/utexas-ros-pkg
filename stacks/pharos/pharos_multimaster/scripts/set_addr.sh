#!/bin/sh

multimaster_path=`rospack find pharos_multimaster`
export ROS_IP=`${multimaster_path}/scripts/get_addr.py $1`
