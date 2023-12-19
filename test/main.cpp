/**
 * @file main.cpp
 * @author Driver: Navigator: 
 * @brief  Main file for the project which runs the tests
 * @version 0.1
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>


/**
 * @brief Main function for the test suite.
 * 
 * Initializes ROS2 node and runs all available tests using Google Test.
 * 
 * @param argc Number of arguments.
 * @param argv Arguments.
 * @return int Returns the result of the tests.
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  std::cout << "DONE SHUTTING DOWN" << std::endl;
  rclcpp::shutdown();
  return result;
}