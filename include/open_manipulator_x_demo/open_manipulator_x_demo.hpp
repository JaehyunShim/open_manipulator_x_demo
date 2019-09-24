/*******************************************************************************
* Copyright 2019 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Ryan Shim */

#ifndef OPEN_MANIPULATOR_X_DEMO_HPP_
#define OPEN_MANIPULATOR_X_DEMO_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "open_manipulator_msgs/srv/set_joint_position.hpp"
#include "open_manipulator_msgs/srv/set_kinematics_pose.hpp"

#define PI 3.14159265359

namespace open_manipulator_x_demo
{
class OpenManipulatorXDemo : public rclcpp::Node
{
 public:
  OpenManipulatorXDemo();
  virtual ~OpenManipulatorXDemo();

 private:
  /*****************************************************************************
  ** ROS Clients
  *****************************************************************************/
  rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_joint_space_path_client_;
  rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_tool_control_client_;

  /*****************************************************************************
  ** Callback Functions and Relevant Functions
  *****************************************************************************/
  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void kinematics_pose_callback(const open_manipulator_msgs::msg::KinematicsPose::SharedPtr msg);

  bool set_joint_space_path(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool set_tool_control(std::vector<double> joint_angle);

};
}  // namespace open_manipulator_x_teleop_joystick
#endif  // OPEN_MANIPULATOR_X_TELEOP_JOYSTICK_HPP_
