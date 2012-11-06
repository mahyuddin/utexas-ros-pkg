/*********************************************************************
* Extension of the navfn node to do human path planning in multi level
* maps. Based on navfn_node from the navfn package. See original copyright
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

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <navfn/navfn_ros.h>
#include <tf/transform_datatypes.h>

#include <bwi_msgs/MakeNavPlan.h>
#include <bwi_msgs/MultiLevelMapData.h>
#include <bwi_utils/utils.h>

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

namespace {
  bwi_msgs::MultiLevelMapData::ConstPtr map_data_ptr;
  bool initialized = false;
  std::map<std::string, boost::shared_ptr<NavfnWithLocalCostmap> > navfn_map; 
  std::map<std::string, boost::shared_ptr<costmap_2d::Costmap2DROS> > costmap_map; 
  std::map<std::string, boost::shared_ptr<tf::TransformListener> > tf_map;
}

double computePlanCost(std::vector<geometry_msgs::PoseStamped>& plan) {
  bool first = true;
  double current_x = 0;
  double current_y = 0;
  double cost = 0;
  BOOST_FOREACH(geometry_msgs::PoseStamped& pose, plan) {
    if (!first) {
      double dist_x = pose.pose.position.x - current_x;
      double dist_y = pose.pose.position.y - current_y;
      cost += sqrtf(dist_x * dist_x + dist_y * dist_y);
    }
    current_x = pose.pose.position.x;
    current_y = pose.pose.position.y;
    first = false;
  }
  return cost;
}

bool makePlanService(bwi_msgs::MakeNavPlan::Request& req, bwi_msgs::MakeNavPlan::Response& resp) {

  if (!initialized) {
    resp.plan_found = false;
    resp.error_message = "The planner has not yet been initialized";
    return true;
  }

  std::string from_level = req.start.level_id;
  std::string to_level = req.goal.level_id;

  if (from_level == to_level) { // No fancy stuff here, use regular planning
    resp.plan_found = true;
    resp.use_local_planning = true;
    return true;
  }
  resp.use_local_planning = false;
 
  // Search through links to links in between these 2 levels
  std::vector<geometry_msgs::Point> start_level_goals;
  std::vector<geometry_msgs::Point> goal_level_starts;
  std::vector<bool> reverse_transitions;
  std::vector<bwi_msgs::MultiLevelMapLink> transitions;
  BOOST_FOREACH(const bwi_msgs::MultiLevelMapLink& link, map_data_ptr->links) {
    if (link.from_point.level_id == from_level && link.to_point.level_id == to_level) {
      start_level_goals.push_back(link.from_point.point);
      goal_level_starts.push_back(link.to_point.point);
      transitions.push_back(link);
      reverse_transitions.push_back(false);
    } else if (link.from_point.level_id == to_level && link.to_point.level_id == from_level) {
      start_level_goals.push_back(link.to_point.point);
      goal_level_starts.push_back(link.from_point.point);
      transitions.push_back(link);
      reverse_transitions.push_back(true);
    }
  }

  if (start_level_goals.size() == 0) {
    resp.plan_found = false;
    resp.error_message = "It is not possible to go from level " + from_level + " to level " + to_level + " as no links exist between these 2 levels";
    return true;
  }

  int best_plan_id = -1;
  double best_plan_cost = std::numeric_limits<double>::max();

  for (size_t i = 0; i < start_level_goals.size(); i++) {
    std::vector<geometry_msgs::PoseStamped> plan1;
    bool success1 = navfn_map[from_level]->makePlan2(req.start.point, start_level_goals[i], plan1);
    if (!success1)
      continue;
    std::vector<geometry_msgs::PoseStamped> plan2;
    bool success2 = navfn_map[to_level]->makePlan2(goal_level_starts[i], req.goal.point, plan2);
    if (!success2)
      continue;

    // Planning succeeded
    // TODO add link cost
    double cost = computePlanCost(plan1) + computePlanCost(plan2);
    if (cost < best_plan_cost) {
      best_plan_id = i;
      best_plan_cost = cost;
    }
  }

  if (best_plan_id == -1) {
    resp.plan_found = false;
    resp.error_message = "Links between start and goal destination exist, but planner unable to find free path";
    return true;
  }

  resp.plan_found = true;
  resp.start_level_goal = start_level_goals[best_plan_id];
  resp.goal_level_start = goal_level_starts[best_plan_id];
  resp.reverse_transition = reverse_transitions[best_plan_id];
  resp.transition = transitions[best_plan_id]; 

  return true;
}

void initializeLevel(bwi_msgs::LevelMetaData& level) {

  std::string map_frame = bwi_utils::frameIdFromLevelId(level.level_id);
  std::string map_topic = map_frame;
  /* All of this is commented out because it doesn't work with the current message setup, needs to be redone
  std::string costmap = level.level.level_namespace + "/local_costmap";
  std::string planner = level.level.level_namespace + "/navfn_planner";
  
  ros::NodeHandle n("~");
  // n.setParam(costmap + "/global_frame", map_topic);
  n.setParam(costmap + "/map_topic", map_topic);
  n.setParam(costmap + "/robot_base_frame", map_frame); // Do this so it doesn't complain about unavailable transform 
  n.setParam(costmap + "/publish_frequency", 0.0);
  n.setParam(costmap + "/observation_sources", std::string(""));
  n.setParam(costmap + "/static_map", true);
  // n.setParam(costmap + "/save_debug_pgm", true);
  n.setParam(costmap + "/robot_radius", 0.1);  // Don't do point planning, incase there is a hole in the wall
  n.setParam(costmap + "/inflation_radius", 0.1);
 
  boost::shared_ptr<tf::TransformListener> tf_ptr;
  tf_ptr.reset(new tf::TransformListener());
  tf_map[id] = tf_ptr;

  boost::shared_ptr<costmap_2d::Costmap2DROS> costmap_ptr;
  costmap_ptr.reset(new costmap_2d::Costmap2DROS(costmap, *tf_ptr));
  costmap_map[id] = costmap_ptr;

  boost::shared_ptr<NavfnWithLocalCostmap> navfn_ptr;
  navfn_ptr.reset(new NavfnWithLocalCostmap(planner, map_topic, costmap_ptr.get()));
  navfn_map[id] = navfn_ptr;

  name_map[id] = level.level.level_name;
  */
}

void getMapData(const bwi_msgs::MultiLevelMapData::ConstPtr& map_data_msg) {
  map_data_ptr = map_data_msg;
  BOOST_FOREACH(bwi_msgs::LevelMetaData level, map_data_ptr->levels) {
    ROS_INFO_STREAM("Initializing level: " << level);
    initializeLevel(level);
  }
  initialized = true;
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "path_provider");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<bwi_msgs::MultiLevelMapData>("/global/map_metadata", 1, getMapData);
  ros::NodeHandle private_n("~");
  ros::ServiceServer service = 
    private_n.advertiseService("make_plan", makePlanService);

  ros::spin();

  return 0;
}
