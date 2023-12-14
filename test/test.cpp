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

TEST(CustomTrajectoryTest, RandomizeCenterTest) {
  CustomTrajectory traj;
  int result = traj.randomizeCenter();
  // Check that the result is one of the expected values (0, 1, or 2)
  EXPECT_TRUE(result == 0 || result == 1 || result == 2);
}

TEST(CustomTrajectoryTest, AssignStationNumberTest) {
  CustomTrajectory traj;
  
  // Test cases for each station
  EXPECT_EQ(traj.assignStationNumber(20.0, 20.0), 1);
  EXPECT_EQ(traj.assignStationNumber(-20.0, 20.0), 2);
  EXPECT_EQ(traj.assignStationNumber(-20.0, -20.0), 3);
  EXPECT_EQ(traj.assignStationNumber(20.0, -20.0), 4);

}
