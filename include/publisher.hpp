/**
 * @file master.cpp
 * @author Driver: Manav Nagda  Navigator: Sameer Arjun Design Keeper: Ishaan Parikh
 * @brief  Master class for the project which creates a master node and executes
 * the trajectory algorithm
 * @version 0.1
 * @copyright Copyright (c) 2023
 *
 */
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
public:
  RobotMove(std::string const &node_name, std::string const &robot_name)
      : Node(node_name), robot_ns{robot_name}, goal_x{0.0}, goal_y{0.0}, laser_scan_data_available{false} {
    callback_grp = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    auto cmd_vel_topic = "/" + robot_ns + "/cmd_vel";
    auto pose_topic = "/" + robot_ns + "/odom";

    publisher_velocity = this->create_publisher<TWIST>(cmd_vel_topic, 1);

    auto subCallback0 = std::bind(&RobotMove::subscribe_callback, this, _1);
    subscription_pose =
        this->create_subscription<ODOM>(pose_topic, 1, subCallback0);

    auto processCallback = std::bind(&RobotMove::process_callback, this);
    timer_ = this->create_wall_timer(100ms, processCallback, callback_grp);
  }

  void set_goal(double x, double y) {
    goal_x = x;
    goal_y = y;
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Navigating to target: [" << goal_x << "," << goal_y << "]");
  }

  void stop();

  bool checkObstacleAhead();

private:
  std::string robot_ns;
  double goal_x;
  double goal_y;

  rclcpp::CallbackGroup::SharedPtr callback_grp;
  rclcpp::Subscription<ODOM>::SharedPtr subscription_pose;
  rclcpp::Publisher<TWIST>::SharedPtr publisher_velocity;
  rclcpp::TimerBase::SharedPtr timer_;
  std::pair<double, double> pose;
  double theta;
  bool laser_scan_data_available;
  std::vector<double> laser_scan_data;

  void subscribe_callback(const ODOM &msg);

  void process_callback();

  void move(double linear, double angular);
};

#endif // INCLUDE_PUBLISHER_HPP_