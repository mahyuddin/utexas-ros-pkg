#!/bin/bash

#ros + source control
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get upgrade --yes && sudo apt-get install ros-fuerte-desktop-full python-rospkg python-rosdep python-rosinstall python-rosrelease ros-fuerte-camera-umd ros-fuerte-qt-ros ros-fuerte-turtlebot ros-fuerte-brown-remotelab ros-fuerte-freenect-stack ros-fuerte-zeroconf-avahi-suite python-netifaces vim subversion git mercurial openssh-server --yes

#ros code built from scratch
cd
mkdir ros
cd ~/ros
rosws init
rosws merge /opt/ros/fuerte/.rosinstall -y
rosws merge http://utexas-ros-pkg.googlecode.com/svn/trunk/rosinstall/experimental.rosinstall -y
rosws merge http://utexas-ros-pkg.googlecode.com/svn/trunk/rosinstall/segway_rmp.rosinstall -y
rosws merge https://github.com/robotics-in-concert/rocon_multimaster/blob/master/rocon_multimaster_fuerte.rosinstall -y
rosws update
echo "source ~/ros/setup.bash" >> ~/.bashrc
source ~/.bashrc && rosmake rocon_gateway rocon_gateway_hub segway_rmp segbot_run segbot_navigation
echo "source \`rospack find bwi_multimaster\`/data/setup.bash" >> ~/.bashrc
