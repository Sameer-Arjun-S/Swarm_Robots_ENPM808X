/**
 * @file publisher_test.cpp
 * @author Driver:  Navigator: 
 * @brief  Test file for the project which tests the publisher and subscriber
 * @version 0.1
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <gtest/gtest.h>
#include <stdlib.h>
#include "std_msgs/msg/string.hpp"
#include "../include/publisher.hpp"

/**
 * @brief Test fixture class for the project
 * 
 */
class TestPublisher : public testing::Test {
 protected:
  rclcpp::Node::SharedPtr node_;
};
/**
 * @brief Construct a new test f object to test the publisher
 * 
 */
TEST_F(TestPublisher, test_num_publishers) {
  node_ = rclcpp::Node::make_shared("test_pub");
  auto test_pub = node_->create_publisher<std_msgs::msg::String>
                    ("Publisher active", 5.0);
    
  auto pub = node_->count_publishers("Publisher active");
  EXPECT_EQ(1, static_cast<int>(pub));
}
/**
 * @brief Construct a new test f object to test the subscriber
 * 
 */
TEST_F(TestPublisher, test_num_subscribers) {
  node_ = rclcpp::Node::make_shared("test_sub");
  auto test_sub = node_->create_subscription<std_msgs::msg::String>("Publisher active", 10.0,
    [](const std_msgs::msg::String::SharedPtr msg) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received message: '%s'", msg->data.c_str());
    });

  auto sub = node_->count_subscribers("Subscriber active");
  EXPECT_EQ(0, static_cast<int>(sub));
}