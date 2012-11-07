/*
    Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <algorithm>
#include <assert.h>

#include <bwi_gazebo_plugins/personcontroller_plugin.h>

#include <common/common.h>
#include <math/gzmath.h>
#include <physics/physics.h>
#include <sdf/sdf.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>

namespace gazebo
{

enum
{
  RIGHT,
  LEFT,
};

// Constructor
PersonControllerPlugin::PersonControllerPlugin()
{
}

// Destructor
PersonControllerPlugin::~PersonControllerPlugin()
{
  delete rosnode_;
}

// Load the controller
void PersonControllerPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->parent = _parent;
  this->world = _parent->GetWorld();

  if (!_sdf->HasElement("topicName")) {
    ROS_WARN("Object controller plugin missing <topicName>, defaults to cmd_vel");
    this->topicName = "cmd_vel";
  } else {
    this->topicName = _sdf->GetElement("topicName")->GetValueString();
  }

  this->robotNamespace = "";
  if (_sdf->HasElement("robotNamespace")) {
    this->robotNamespace = _sdf->GetElement("robotNamespace")->GetValueString() + "/";
  }

  alive_ = true;

  // Initialize the ROS node and subscribe to cmd_vel
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "diff_drive_plugin", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle(this->robotNamespace);
  ros::NodeHandle nh_private("~");

  ROS_INFO("starting object controller plugin in ns: %s", this->robotNamespace.c_str());

  // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(topicName, 1,
          boost::bind(&PersonControllerPlugin::cmdVelCallback, this, _1),
          ros::VoidPtr(), &queue_);
  sub_ = rosnode_->subscribe(so);
  pub_ = rosnode_->advertise<geometry_msgs::PointStamped>("person_detections", 1);

  // start custom queue for diff drive
  this->callback_queue_thread_ = 
    boost::thread(boost::bind(&PersonControllerPlugin::QueueThread, this));

  // listen to the update event (broadcast every simulation iteration)
  this->updateConnection = event::Events::ConnectWorldUpdateStart(boost::bind(&PersonControllerPlugin::UpdateChild, this));
}

// Update the controller
void PersonControllerPlugin::UpdateChild() {
  double stepTime = this->world->GetPhysicsEngine()->GetStepTime();

  math::Pose new_pose = this->parent->GetWorldPose();
  new_pose.pos.x += stepTime * forward_vel * cos(new_pose.rot.GetYaw());
  new_pose.pos.y += stepTime * forward_vel * sin(new_pose.rot.GetYaw());
  new_pose.rot.SetFromEuler(math::Vector3(0,0,new_pose.rot.GetYaw() + stepTime * angular_vel));
  this->parent->SetWorldPose(new_pose);

  // Collisions might impart some velocity to it, nullify said velocity
  this->parent->SetLinearVel(math::Vector3(0,0,0));
  this->parent->SetLinearAccel(math::Vector3(0,0,0));
  this->parent->SetAngularVel(math::Vector3(0,0,0));
  this->parent->SetAngularAccel(math::Vector3(0,0,0));

  bwi_msgs::PersonDetection detection;
  //TODO find out the best way to suppy the global frame id, maybe a global param
  detection.header.frame_id = "/map";
  detection.header.stamp = ros::Time::now();
  detection.feet.x = new_pose.pos.x;
  detection.feet.y = new_pose.pos.y;
  detection.feet.z = new_pose.pos.z;
  pub_.publish(detection);
}

// Finalize the controller
void PersonControllerPlugin::FiniChild() {
  alive_ = false;
  queue_.clear();
  queue_.disable();
  rosnode_->shutdown();
  callback_queue_thread_.join();
}

void PersonControllerPlugin::cmdVelCallback
    (const geometry_msgs::Twist::ConstPtr& cmd_msg) {
  lock.lock();
  forward_vel = cmd_msg->linear.x;
  angular_vel = cmd_msg->angular.z;
  lock.unlock();
}

void PersonControllerPlugin::QueueThread() {
  static const double timeout = 0.01;
  while (alive_ && rosnode_->ok())
  {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

GZ_REGISTER_MODEL_PLUGIN(PersonControllerPlugin)
}

