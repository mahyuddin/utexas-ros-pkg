#include <ros/ros.h>
/*********************************************************************
* Based on navfn_node from the navfn package. See original copyright
* notice below:
* 
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Bhaskara Marthi
*********************************************************************/

#include <navfn/navfn_ros.h>
#include <tf/transform_datatypes.h>

class NavfnWithLocalCostmap : public navfn::NavfnROS {
  public:
    NavfnWithLocalCostmap(std::string name, std::string frame_id, costmap_2d::Costmap2DROS* cmap);
    bool makePlan2(const geometry_msgs::Point &start, const geometry_msgs::Point &goal, 
        std::vector<geometry_msgs::PoseStamped>& plan);

  private:
    boost::shared_ptr<costmap_2d::Costmap2D> cmap_;
    std::string frame_id_;
};

NavfnWithLocalCostmap::NavfnWithLocalCostmap(std::string name, std::string frame_id, costmap_2d::Costmap2DROS* cmap) : NavfnROS(name, cmap) {
    inscribed_radius_ = 0.0;
    circumscribed_radius_ = 0.0;
    inflation_radius_ = 0.0;
    frame_id_ = frame_id;
}

bool NavfnWithLocalCostmap::makePlan2(const geometry_msgs::Point& start, const geometry_msgs::Point& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
  geometry_msgs::PoseStamped start_pose;
  start_pose.header.frame_id = frame_id_;
  start_pose.header.stamp = ros::Time::now();
  start_pose.pose.position = start;
  start_pose.pose.orientation =  tf::createQuaternionMsgFromRollPitchYaw(0,0,0); 

  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.frame_id = frame_id_;
  goal_pose.header.stamp = ros::Time::now();
  goal_pose.pose.position = goal;
  goal_pose.pose.orientation =  tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

  return makePlan(start_pose, goal_pose, plan);
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "navfn_node");

  ros::NodeHandle n("~");
  tf::TransformListener tf;

  // Set params
  /* n.setParam("dummy_costmap/global_frame", "/ens1/map"); */
  n.setParam("dummy_costmap/map_topic", "/ens1/map");
  n.setParam("dummy_costmap/robot_base_frame", "/ens1/map"); // Do this so it doesn't complain about unavailable transform 
  n.setParam("dummy_costmap/publish_frequency", 0.0);
  n.setParam("dummy_costmap/observation_sources", std::string(""));
  n.setParam("dummy_costmap/static_map", true);
  /* n.setParam("dummy_costmap/save_debug_pgm", true); */
  n.setParam("dummy_costmap/robot_radius", 0.1);
  n.setParam("dummy_costmap/inflation_radius", 0.1);
  
  costmap_2d::Costmap2DROS dummy_costmap("dummy_costmap", tf);
  NavfnWithLocalCostmap navfn("navfn_planner", "/ens1/map", &dummy_costmap);

  geometry_msgs::Point start;
  start.x = 10;
  start.y = 5;
  start.z = 0;

  geometry_msgs::Point goal;
  goal.x = 19.3;
  goal.y = 2.5;
  goal.z = 0;

  std::vector<geometry_msgs::PoseStamped> plan;
  bool success = navfn.makePlan2(start, goal, plan);

  if (success) { 
    std::cout << "Success!!" << std::endl;
    BOOST_FOREACH(geometry_msgs::PoseStamped& point, plan) {
      std::cout << "[" << point.pose.position.x << "," << point.pose.position.y << "]" << std::endl;
    }
  }

  return 0;
}
