#include <gtest/gtest.h>
#include <memory>
#include <utility>

# include "../include/custom_trajectory.hpp"

TEST(AlgorithmTest,CircleTest) {
  CustomTrajectory traj;
  traj.setCenter(0,0);
  traj.setRobotCount(20);
  std::vector<std::vector<double>> circle = traj.generateCirclePath();
  EXPECT_EQ(circle.size(),20);
}

TEST(AlgorithmTest,SquareTest) {
  CustomTrajectory traj;
  traj.setCenter(0,0);
  traj.setRobotCount(20);
  std::vector<std::vector<double>> square = traj.generateSquarePath();
  EXPECT_EQ(square.size(),20);
}
