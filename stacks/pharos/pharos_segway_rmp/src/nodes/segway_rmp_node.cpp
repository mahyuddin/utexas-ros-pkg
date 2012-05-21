/**
 * \file  segway_rmp_node.cpp
 * \brief Main driver class for the segway 
 */

#define NODE "segway_rmp_node"

namespace {

  std::string serial_port;
  float segway_motor_timeout;
  std::string frame_id;
  std::string segway_rmp_type;

  float linear_pos_accel_limit;
  float linear_neg_accel_limit;
  float angular_pos_accel_limit;
  float angular_neg_accel_limit;

  float linear_odom_scale;
  float angular_odom_scale;

  float dps_to_counts;
  float mps_to_counts;
  float meters_to_counts;
  float rev_to_counts;
  float torque_to_counts;

}

int getParameters(const ros::NodeHandle &private_nh) {

  private_nh.param("serial_port", serial_port, std::string("/dev/ttyUSB0"));
  private_nh.param("motor_timeout", segway_motor_timeout, 0.5);

  private_nh.param("frame_id", frame_id, std::string("base_link"));
  private_nh.param("segway_rmp_type", serial_port, std::string("50/100"));

  // Get the linear acceleration limits in m/s^2.  Zero means infinite.
  private_nh.param("linear_pos_accel_limit", linear_pos_accel_limit, 0.0);
  private_nh.param("linear_neg_accel_limit", linear_neg_accel_limit, 0.0);

  // Get the angular acceleration limits in deg/s^2.  Zero means infinite.
  private_nh.param("angular_pos_accel_limit", angular_pos_accel_limit, 0.0);
  private_nh.param("angular_neg_accel_limit", angular_neg_accel_limit, 0.0);

  n->param("linear_odom_scale", this->linear_odom_scale, 1.0);
  n->param("angular_odom_scale", this->angular_odom_scale, 1.0);

  return 1;
}

int setupSegwayRMP() {
  if (segway_rmp_type == "50/100") {
    dps_to_counts = 7.8;
    mps_to_counts = 401.0;
    meters_to_counts = 40181.0;
    rev_to_counts = 117031.0;
    torque_to_counts = 1463.0;
  } else {
    dps_to_counts = 7.8;
    mps_to_counts = 332.0;
    meters_to_counts = 33215.0;
    rev_to_counts = 112644.0;
    torque_to_counts = 1094.0;
  }
  return 1;
}

int main(int argc, const char *argv[]) {

  ros::init(argc, argv, NODE);
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  if (!getParameters(private_nh)) {
    ROS_FATAL("Parameters incorrect. Exitting...");
    return -1;
  }

  // Setup the Segway RMP
  if (!setupSegwayRMP()) {
    ROS_FATAL("Unable to setup Segway RMP. Exitting...");
    return -1;
  }
  
  // Setup ROS Communications

  // Single threaded spin
  ROS_INFO("Starting spin...");
  ros::spin();
  ROS_INFO("Ending spin...");

  return 0;
}

