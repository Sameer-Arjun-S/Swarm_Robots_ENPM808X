/**
 * @file main.cpp
 * @brief  Main file for the project to create nodes and control the robots
 * @version 0.1
 * @date 2023
 */

#include "../include/master.hpp"
#include "../include/custom_trajectory.hpp"
#include <cstdlib>
#include <ctime>

using std::cout;
using std::endl;
using std::vector;

int main(int argc, char **argv) {
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Seed the random number generator with the current time
  std::srand(static_cast<unsigned int>(std::time(nullptr)));

//   // Create a MultiThreadedExecutor to handle multiple nodes
//   rclcpp::executors::MultiThreadedExecutor exec;

//   // Create an array to hold robot nodes
//   std::vector<std::shared_ptr<RobotMove>> robot_array;

  // Create and add robot nodes to the executor
//   for (int i = 0; i < 24; i++) {
//     auto r_namespace = "tb" + std::to_string(i);
//     auto nodename = "tb" + std::to_string(i) + "_node";
//     auto robot = std::make_shared<RobotMove>(nodename, r_namespace);
//     exec.add_node(robot);
//     robot_array.push_back(robot);
//   }

//   // Create and add the master node to the executor
//   auto node = std::make_shared<Master>(robot_array);
//   exec.add_node(node);

  // Spin the executor to run all nodes
//   exec.spin();

//   // Shutdown ROS
//   rclcpp::shutdown();

  return 0;
}