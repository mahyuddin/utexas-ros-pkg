#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>

#include <boost/thread/mutex.hpp>

#define NODE "glados_follower_app"

namespace {
  ros::Publisher goal_publisher_;
  boost::mutex mutex_robot_pose_; // Not actually running on 2 threads - but I have put this in case I forget later
  float robot_x_;
  float robot_y_;
}

void processPerson(geometry_msgs::PoseStamped::ConstPtr person_msg) {

  float person_x = person_msg->pose.position.x;
  float person_y = person_msg->pose.position.y;

  mutex_robot_pose_.lock();
  float robot_x = robot_x_;
  float robot_y = robot_y_;
  mutex_robot_pose_.unlock();

  float goal_orientation = atan2f(person_y - robot_y, person_x - robot_x);
  float distance = sqrtf((person_y - robot_y)*(person_y - robot_y) + (person_x - robot_x)*(person_x - robot_x));

  if (distance < 1) {
    return;
  }

  float next_distance = distance - 0.5;
  float goal_x = robot_x + (next_distance / distance) * person_x;
  float goal_y = robot_y + (next_distance / distance) * person_y;

  geometry_msgs::PoseStamped next_goal;
  next_goal.header.frame_id = "map";
  next_goal.header.stamp = ros::Time::now();
  next_goal.pose.position.x = goal_x;
  next_goal.pose.position.y = goal_y;
  next_goal.pose.position.z = 0;
  next_goal.pose.orientation = tf::createQuaternionMsgFromYaw(goal_orientation);  

  goal_publisher_.publish(next_goal);
}

void processRobotPose(geometry_msgs::PoseWithCovarianceStamped::ConstPtr robot_pose_msg) {
  mutex_robot_pose_.lock();
  robot_x_ = robot_pose_msg->pose.pose.position.x;
  robot_y_ = robot_pose_msg->pose.pose.position.y;
  mutex_robot_pose_.unlock();
}

int main(int argc, char *argv[]) {

  ros::init(argc, argv, NODE);
  ros::NodeHandle node;

  int q_depth = 1;

  // Le Subscribers
  ros::TransportHints no_delay = ros::TransportHints().tcpNoDelay(true);
  ros::Subscriber person_subscriber_ =
    node.subscribe("glados_person_detection/person", q_depth,
                   &processPerson, no_delay);
  ros::Subscriber robot_pose_subscriber_ = 
    node.subscribe("amcl_pose", q_depth, 
                   &processRobotPose, no_delay);

  // Le Publishers
  goal_publisher_ = 
    node.advertise <geometry_msgs::PoseStamped>("move_base_simple/goal2", q_depth);

  // Le spin
  ros::spin();

  // Le exit
  return 0;
}

