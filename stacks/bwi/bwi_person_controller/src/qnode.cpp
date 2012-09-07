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
	{}

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
	// Add your ros communications here.
	start();
	return true;
}

void QNode::run() {
  ros::spin();
	// ros::Rate loop_rate(1);
	// int count = 0;
	// while ( ros::ok() ) {

	// 	std_msgs::String msg;
	// 	std::stringstream ss;
	// 	ss << "hello world " << count;
	// 	msg.data = ss.str();
	// 	chatter_publisher.publish(msg);
	// 	log(Info,std::string("I sent: ")+msg.data);
	// 	ros::spinOnce();
	// 	loop_rate.sleep();
	// 	++count;
	// }
	// std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

}  // namespace bwi_person_controller
