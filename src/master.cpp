/**
 * @file master.cpp
 * @author Driver: Sameer Arjun S  Navigator: Ishaan Parikh  Design Keeper: Manav Nagda
 * @brief  Master class for the project which creates a master node and executes
 * the trajectory algorithm
 * @version 0.1
 * @copyright Copyright (c) 2023
 *
 */
#include "../include/master.hpp"
#include "../include/custom_trajectory.hpp"

using std::cin;
using std::cout;
using std::endl;
using std::vector;
using std::placeholders::_1;
using namespace std::chrono_literals;
using TWIST = geometry_msgs::msg::Twist;
using ODOM = nav_msgs::msg::Odometry;
using RCL_NODE_PTR = std::shared_ptr<rclcpp::Node>;

/**
 * @brief Constructor for Master class.
 * @param robot_array Vector of shared pointers to RobotMove objects.
 */

Master::Master(std::vector<std::shared_ptr<RobotMove>> const &robot_array)
    : Node("master_node") {
  this->robot_array = robot_array;
  auto processCallback = std::bind(&Master::process_callback, this);
  this->timer_ = this->create_wall_timer(100ms, processCallback);
  this->traj();
}

/**
 * @brief Empty process callback method.
 */
void Master::process_callback() {}
/**
 * @brief Executes the trajectory algorithm based on random choice.
 * Generates path and sets goals for robots accordingly.
 */
void Master::traj() {
  CustomTrajectory trajectory;
  int choice = trajectory.randomizeCenter();
  int stationNumber = trajectory.assignStationNumber(trajectory.xCenter, trajectory.yCenter); 
  trajectory.setRobotCount(20);
  vector<vector<double>> circleTrajectory = trajectory.generateCirclePath();
  vector<vector<double>> squareTrajectory = trajectory.generateSquarePath();

  cout << "There is a fire at station number: " << stationNumber << endl;

  switch (choice) {
  case 1:
    cout << "Executing Circular Formation" << endl;
    for (int i = 0; i < static_cast<int>(circleTrajectory.size()); i++) {
      this->robot_array[i]->set_goal(circleTrajectory[i][0],
                                     circleTrajectory[i][1]);
    }
    break;
  case 2:
    cout << "Executing Square Formation" << endl;
    for (int i = 0; i < static_cast<int>(squareTrajectory.size()); i++) {
      this->robot_array[i]->set_goal(squareTrajectory[i][0],
                                     squareTrajectory[i][1]);
    }
    break;
  default:
    cout << "Invalid choice" << endl;
    break;
  }
}