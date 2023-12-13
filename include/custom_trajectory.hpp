#ifndef INCLUDE_CUSTOM_TRAJECTORY_HPP_
#define INCLUDE_CUSTOM_TRAJECTORY_HPP_

#include <vector>
#include <cmath>

class CustomTrajectory {
public:
  CustomTrajectory();

  int randomizeCenter();

  int assignStationNumber(double x, double y);

  void setCenter(double x, double y);

  void setRobotCount(double n);

  int getRobotCount();

  std::vector<std::vector<double>> generateCirclePath();

  std::vector<std::vector<double>> generateSquarePath();

private:
  double centersArray[4][2];
  double xCenter;
  double yCenter;
  double circleRadius;
  double circumRadius;
  double sideLength;
  double robotCount;
};

#endif // INCLUDE_CUSTOM_TRAJECTORY_HPP_