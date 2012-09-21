/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/bwi_person_controller/qnode.hpp"
#include <geometry_msgs/Twist.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace bwi_person_controller {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{
    init();
  }

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"bwi_person_controller");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
  ros::NodeHandle private_nh("~");
  private_nh.param<std::string>("id", person_id, "person");
	// Add your ros communications here.
  navigate_client = n.serviceClient<bwi_msgs::NavigatePerson>("/navigate");
  vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	start();
	return true;
}

void QNode::run() {
  initialized = true;
  ros::Rate rate(10);
  while (ros::ok()) {
    bool plan_available = false;
    bwi_msgs::NavigatePerson plan;
    navigation_mutex.lock();
    if (navigation_queue.size() > 0) {
      plan_available = true;
      plan = navigation_queue[0];
      navigation_queue.erase(navigation_queue.begin());
    }
    navigation_mutex.unlock();
    if (plan_available) {
      navigate_client.call(plan);
    }
    rate.sleep();
  }
	emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::navigate(double x, double y, uint32_t level, std::string &error) {
  bwi_msgs::NavigatePerson plan;
  plan.request.person_id = person_id;
  plan.request.goal.level_id = level;
  plan.request.goal.point.x = x;
  plan.request.goal.point.y = y;
  plan.request.goal.point.z = 0;
  navigation_mutex.lock();
  navigation_queue.push_back(plan);
  navigation_mutex.unlock();
}

void QNode::move(double x, double theta) {
  geometry_msgs::Twist v;
  v.linear.x = x;
  v.linear.y = v.linear.z = 0; //HACK!!
  v.angular.x = v.angular.y = 0;
  v.angular.z = theta;
  vel_pub.publish(v);
}

}  // namespace bwi_person_controller
