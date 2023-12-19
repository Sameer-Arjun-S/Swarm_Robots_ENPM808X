/**
 * @file main.cpp
 * @author Driver:  Navigator:
 * @brief  Main file for the project which creates the nodes
 * @version 0.1
 * @copyright Copyright (c) 2023
 *
 */

// Importing the necessary libraries
#include "../include/master.hpp"
#include "../include/custom_trajectory.hpp"
#include <cstdlib>
#include <ctime>

using std::cout;
using std::endl;
using std::vector;

/**
 * @brief Main function for the project
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @return Integer value indicating the execution status
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // Seeding the random number generator
  std::srand(static_cast<unsigned int>(std::time(nullptr)));
  rclcpp::executors::MultiThreadedExecutor exec;
  std::vector<std::shared_ptr<RobotMove>> robot_array;
  // Implementing loop for instances of RoboMove object
  for (int i = 0; i < 24; i++) {
    auto r_namespace = "tb" + std::to_string(i);
    auto nodename = "tb" + std::to_string(i) + "_node";
    // Shared pointer for RoboMove object
    auto robot = std::make_shared<RobotMove>(nodename, r_namespace);
    exec.add_node(robot);
    robot_array.push_back(robot);
  }

  auto node = std::make_shared<Master>(robot_array);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}