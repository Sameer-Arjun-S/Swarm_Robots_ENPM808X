#include "../include/master.hpp"
#include "../include/custom_trajectory.hpp"

using std::cin;
using std::cout;
using std::endl;
using std::vector;
using std::placeholders::_1;
using namespace std::chrono_literals;
using TWIST = geometry_msgs::msg::Twist;
using ODOM = nav_msgs::msg::Odometry;
using RCL_NODE_PTR = std::shared_ptr<rclcpp::Node>;

Master::Master(std::vector<std::shared_ptr<RobotMove>> const &robot_array)
    : Node("master_node") {
  this->robot_array = robot_array;
  auto processCallback = std::bind(&Master::process_callback, this);
  this->timer_ = this->create_wall_timer(100ms, processCallback);
  this->traj();
}

void Master::process_callback() {
  // Stub: Implementing the process callback logic
  // Implementing the logic to control the robots based on trajectory
}

void Master::traj() {
// STUB 
}