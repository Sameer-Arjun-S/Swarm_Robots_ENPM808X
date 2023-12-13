#ifndef INCLUDE_CUSTOM_TRAJECTORY_HPP_
#define INCLUDE_CUSTOM_TRAJECTORY_HPP_

#include <cmath>
#include <iostream>
#include <vector>

class CustomTrajectory {
 public:
  double xCenter;
  double yCenter;

 private:
  double centersArray[4][2];  
  double circleRadius;
  double sideLength;
  double robotCount;
  double circumRadius;

 public:
  CustomTrajectory();
  void setCenter(double x, double y);
  void setRobotCount(double n);
  int getRobotCount();
  int randomizeCenter();
  int assignStationNumber(double x, double y);
  std::vector<std::vector<double>> generateCirclePath();
  std::vector<std::vector<double>> generateSquarePath();
};

#endif  // INCLUDE_CUSTOM_TRAJECTORY_HPP_
