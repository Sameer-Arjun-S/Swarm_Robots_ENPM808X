#ifndef INCLUDE_PUBLISHER_HPP_
#define INCLUDE_PUBLISHER_HPP_

#include <string>
#include <utility>
#include <chrono>
#include <iostream>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

using TWIST = geometry_msgs::msg::Twist;
using ODOM = nav_msgs::msg::Odometry;

class RobotMove : public rclcpp::Node {

  void stop() {
    // Implementing stub for stopping robot movement
  }

  bool checkObstacleAhead() {
    // Implementing stub for checking obstacles
    return false;
  }

private:
  std::string robot_ns;
  double goal_x;
  double goal_y;

 
  void subscribe_callback(const ODOM &msg) {
    // Stubbed subscribe_callback function
    // Implementing stub for subscribing to messages
  }

  void process_callback() {
    // Stubbed process_callback function
    // Implementing stub for processing callbacks
  }

  void move(double linear, double angular) {
    // Stubbed move function
    // Implementing stub for moving the robot
  }
};

#endif // INCLUDE_PUBLISHER_HPP_