#include "../include/custom_trajectory.hpp"

using std::vector;

CustomTrajectory::CustomTrajectory() {

  centersArray[0][0] = 20.0;
  centersArray[0][1] = 20.0;
  centersArray[1][0] = -20.0;
  centersArray[1][1] = 20.0;
  centersArray[2][0] = 20.0;
  centersArray[2][1] = -20.0;
  centersArray[3][0] = -20.0;
  centersArray[3][1] = -20.0;

  xCenter = 0;
  yCenter = 0;
  circleRadius = 9.6;
  circumRadius = 9.6;
  sideLength = 9.6;
  robotCount = 12;
}

int CustomTrajectory::randomizeCenter() {

  int randomIndex = rand() % 4;  
  xCenter = centersArray[randomIndex][0];
  yCenter = centersArray[randomIndex][1];

  if ((xCenter == 20.0 && yCenter == 20.0) || (xCenter == 20.0 && yCenter == -20.0)) {
    return 2;
  } else if ((xCenter == -20.0 && yCenter == 20.0) || (xCenter == -20.0 && yCenter == -20.0)) {
    return 1;
  } else {
    return 0;
  }
}

int CustomTrajectory::assignStationNumber(double x, double y) {

  if ((x == 20.0 && y == 20.0)) {
    return 1;
  } else if ((x == -20.0 && y == 20.0)) {
    return 2;
  } else if ((x == -20.0 && y == -20.0)) {
    return 3;
  } else if ((x == 20.0 && y == -20.0)) {
    return 4;
  }
}

void CustomTrajectory::setCenter(double x, double y) {
  xCenter = x;
  yCenter = y;
}

void CustomTrajectory::setRobotCount(double n) {
  robotCount = n;
}

int CustomTrajectory::getRobotCount() {
  return robotCount;
}

vector<vector<double>> CustomTrajectory::generateCirclePath() {
  vector<vector<double>> circlePath(robotCount, vector<double>(2, 0));
  double angle = 0;
  double angleIncrement = 2 * M_PI / robotCount;
  for (int i = 0; i < robotCount; i++) {
    circlePath[i][0] = xCenter + circleRadius * cos(angle);
    circlePath[i][1] = yCenter + circleRadius * sin(angle);
    angle += angleIncrement;
  }
  return circlePath;
}

vector<vector<double>> CustomTrajectory::generateSquarePath() {
  vector<vector<double>> squarePath(robotCount, vector<double>(2, 0));
  double x = xCenter - sideLength / 2;
  double y = yCenter - sideLength / 2;
  for (int i = 0; i < robotCount; i++) {
    if (i < robotCount / 4) {
      x = x + sideLength / (robotCount / 4);
    } else if (i < robotCount / 2) {
      y = y + sideLength / (robotCount / 4);
    } else if (i < 3 * robotCount / 4) {
      x = x - sideLength / (robotCount / 4);
    } else {
      y = y - sideLength / (robotCount / 4);
    }
    squarePath[i][0] = x;
    squarePath[i][1] = y;
  }
  return squarePath;
}
