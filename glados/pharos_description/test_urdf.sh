#!/bin/sh

rosrun xacro xacro.py urdf/pharos1.urdf.xacro > pharos.urdf
roslaunch urdf_tutorial display.launch model:=pharos.urdf
