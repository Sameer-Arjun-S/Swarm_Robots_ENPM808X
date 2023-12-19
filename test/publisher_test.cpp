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
 * @brief Test fixture class for testing publisher and subscriber
 * 
 */
class TestPublisher : public testing::Test {
 protected:
  rclcpp::Node::SharedPtr node_;
};
/**
 * @brief Test the number of publishers created
 * 
 */
TEST_F(TestPublisher, test_num_publishers) {
  node_ = rclcpp::Node::make_shared("testpub");
  auto test_pub = node_->create_publisher<std_msgs::msg::String>
                    ("publisher_active", 10.0);
    
  auto pub = node_->count_publishers("publisher_active");
  EXPECT_EQ(1, static_cast<int>(pub));
}
/**
 * @brief Test the number of subscribers created
 * 
 */
TEST_F(TestPublisher, test_num_subscribers) {
  node_ = rclcpp::Node::make_shared("testsub");
  auto test_sub = node_->create_subscription<std_msgs::msg::String>("publisher_active", 10.0,
    [](const std_msgs::msg::String::SharedPtr msg) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I heard: '%s", msg->data.c_str());
    });

  auto sub = node_->count_subscribers("subscriber_active");
  EXPECT_EQ(0, static_cast<int>(sub));
}