#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sstream>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/features2d/features2d.hpp"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include "CloudProcessor.h"

#define PI 3.1415926535
class PersonDetector {
  private:
    ros::Publisher& _posPub, _cmdPub;
    tf::TransformListener _listener;
    CloudProcessor _processor;
  public:
    PersonDetector(ros::Publisher& posPub, ros::Publisher& cmdPub) : _posPub(posPub), _cmdPub(cmdPub), _listener(ros::Duration(30.0)) {
      // +x right, +y down, +z forward ... may want to xform the cloud first so we can put coords in base_link frame
      _processor.setBoundingBox(-.7,.7,-2,.3,0,2);
    }
    
    void handleCloudMessage(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
      _processor.setCloud(cloud);
      _processor.processSegments();
  
      pcl::PointXYZ centroid;
      if (_processor.getCentroid(centroid)) {
        geometry_msgs::PointStamped source; 
        source.point.x = centroid.x;
        source.point.y = centroid.y;
        source.point.z = centroid.z;
        ROS_INFO("centroid_cam: (%2.2f,%2.2f,%2.2f)", 
          source.point.x, source.point.y, source.point.z
        );
        geometry_msgs::PointStamped target;
        if(transformToBase(source, target))
          approachPoint(target);
      }
      else
      {
        _cmdPub.publish(geometry_msgs::Twist());
      }

    }

    void approachPoint(geometry_msgs::PointStamped target) {
      double linear_scale = 1.0, angular_scale = 0.8, goal_distance = .6;
      double maxLinear = .4, maxAngular = PI / 6;
      double angularOffset = .07;
      
      geometry_msgs::Twist cmd;
      cmd.linear.x = (target.point.x - goal_distance) * linear_scale;
      cmd.angular.z = (target.point.y + angularOffset) * angular_scale;
      ROS_INFO("requesting %2.2f m/s forward, %2.2f rad %s", 
        cmd.linear.x, 
        cmd.angular.z, 
        cmd.angular.z > 0 ? "left" : "right");
      if(cmd.linear.x < -maxLinear) cmd.linear.x = -maxLinear;
      if(cmd.linear.x > maxLinear) cmd.linear.x = maxLinear;
      if(cmd.angular.z < -maxAngular) cmd.angular.z = -maxAngular;
      if(cmd.angular.z > maxAngular) cmd.angular.z = maxAngular;
      _cmdPub.publish(cmd);

      ROS_INFO("centroid_base: (%2.2f,%2.2f,%2.2f), %2.2f m/s forward, %2.2f rad %s", 
        target.point.x, target.point.y, target.point.z,
        cmd.linear.x, cmd.angular.z,
        cmd.angular.z > 0 ? "left" : "right"
      );
    }

    bool transformToBase(geometry_msgs::PointStamped source, geometry_msgs::PointStamped& target) {
        bool success = false;
        ros::Rate sleeper(1);
        ros::Time now;
        while(!success) {
          try {        
            now = ros::Time::now();
            _listener.waitForTransform("camera_depth_optical_frame","base_link",now,ros::Duration(0.1));
            tf::StampedTransform transform;
            _listener.lookupTransform("camera_depth_optical_frame","base_link",now, transform);
            success = true;
         } catch(tf::ExtrapolationException &e) {
            
            ROS_INFO("extrapolation error, continuing: %s", e.what());
            return false;
          } catch(tf::LookupException &e) {
            ROS_INFO("lookup failed, continuing");
            return false;
          }
          if(!success)
            sleeper.sleep();
        }
        geometry_msgs::PointStamped pt;
        source.header = _processor.getCloud()->header;
        source.header.stamp = now;
        
        _listener.transformPoint("base_link", source, target);
        return true;
    }

    void handleImageMessage(const sensor_msgs::Image::ConstPtr& msg) {
      //ROS_INFO("got image message: %i", msg->header.seq);
    }
};

int main(int argc, char **argv)
{
  cv::Mat target_img;
  ros::init(argc, argv, "person_detector");
  ros::NodeHandle n;
  ros::Publisher cmdPub = n.advertise<geometry_msgs::Twist>("disabled_cmd_vel",5);
  ros::Publisher posPub = n.advertise<geometry_msgs::PoseStamped>("person_detector/person",5);
  PersonDetector* detector = new PersonDetector(posPub, cmdPub);
  ros::Subscriber image_sub = n.subscribe("camera/rgb/image_color", 1000, &PersonDetector::handleImageMessage, detector);
  ros::Subscriber pc_sub = n.subscribe("camera/depth/points", 1, &PersonDetector::handleCloudMessage, detector);
  ros::spin();

  return 0;
}


