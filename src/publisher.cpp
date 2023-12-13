/**
 * @file publisher.cpp
 * @author Driver: Manav Nagda  Navigator: Sameer Arjun Design Keeper: Ishaan Parikh
 * @brief  Master class for the project which creates a master node and executes
 * the trajectory algorithm
 * @version 0.1
 * @copyright Copyright (c) 2023
 *
 */
#include "../include/publisher.hpp"
#include "../include/custom_trajectory.hpp"
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::cin;
using std::cout;
using std::endl;
using std::placeholders::_1;

using LaserScanMsg = sensor_msgs::msg::LaserScan;

void RobotMove::subscribe_callback(const ODOM &msg) {
  pose.first = msg.pose.pose.position.x;
  pose.second = msg.pose.pose.position.y;
  theta = asin(msg.pose.pose.orientation.z) * 2;
}

void RobotMove::move(double linear, double angular) {
  geometry_msgs::msg::Twist msg;
  msg.linear.x = linear;
  msg.angular.z = angular;
  publisher_velocity->publish(msg);
}

void RobotMove::stop() {
  geometry_msgs::msg::Twist cmd_vel_msg;
  cmd_vel_msg.linear.x = 0;
  cmd_vel_msg.angular.z = 0;
  publisher_velocity->publish(cmd_vel_msg);
}

void RobotMove::process_callback() {
  std::pair<double, double> goal{goal_x, goal_y};
  double K_linear = 0.05;
  double K_angular = 0.7;
  double distance = abs(
      sqrt(pow((goal_x - (pose.first)), 2) + pow((goal_y - (pose.second)), 2)));
  double linear_speed = distance * K_linear;
  double desired_angle_goal = atan2(goal_y - pose.second, goal_x - pose.first);
  double angular_speed = (desired_angle_goal - theta) * K_angular;

  if (checkObstacleAhead()) {
    angular_speed = 0.5; // Rotate to the right if an obstacle is detected
  }

  move(linear_speed, angular_speed);

  if (distance < 0.1) {
    stop();
    RCLCPP_INFO(this->get_logger(), "Reached target successfully");
  }
}

bool RobotMove::checkObstacleAhead() {
  if (!laser_scan_data_available) {
    return false;
  }

  double obstacle_distance_threshold = 0.8;

  for (double range : laser_scan_data) {
    if (range < obstacle_distance_threshold) {
      return true;
    }
  }

  return false;
}