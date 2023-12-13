#ifndef INCLUDE_MASTER_HPP_
#define INCLUDE_MASTER_HPP_

#include <vector>
#include <memory>
#include "../include/publisher.hpp"

using TWIST = geometry_msgs::msg::Twist;
using ODOM = nav_msgs::msg::Odometry;

/**
 * @brief Master class to control the robots and spawn nodes
 *
 */
class Master : public rclcpp::Node {
 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<TWIST>::SharedPtr publisher_;
  std::vector<std::shared_ptr<RobotMove>> robot_array;

 public:
  /**
   * @brief Construct a new Master object
   *
   * @param robot_array  Vector of robot objects
   */
  explicit Master(std::vector<std::shared_ptr<RobotMove>> const &robot_array);

  /**
   * @brief Empty function to be called by publisher
   *
   */
  void process_callback();

  /**
   * @brief Function to select the robot trajectory to be followed and call the
   * respective function
   *
   */
  void traj();
};
#endif // INCLUDE_MASTER_HPP_
