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

#include "open_manipulator_x_demo/open_manipulator_x_demo.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;


OpenManipulatorXDemo::OpenManipulatorXDemo()
: Node("open_manipulator_x_demo")
{
  /********************************************************************************
  ** Initialise ROS clients
  ********************************************************************************/
  // Initialise ROS clients
  goal_joint_space_path_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>("open_manipulator_x/goal_joint_space_path");
  goal_tool_control_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>("open_manipulator_x/goal_tool_control");

  /********************************************************************************
  ** Initialise timers
  ********************************************************************************/
  process_timer_ = this->create_wall_timer(2.5s, std::bind(&OpenManipulatorXDemo::process_callback, this));

  RCLCPP_INFO(this->get_logger(), "OpenManipulator-X demo has been initialised.");
}

OpenManipulatorXDemo::~OpenManipulatorXDemo() 
{
  RCLCPP_INFO(this->get_logger(), "OpenManipulator-X demo has been terminated.");
}

/********************************************************************************
** Callback Functions
********************************************************************************/
bool OpenManipulatorXDemo::set_joint_space_path(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
  request->joint_position.joint_name = joint_name;
  request->joint_position.position = joint_angle;
  request->path_time = path_time;
  
  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) 
  {
    auto result = future.get();
    return result->is_planned;
  };
  auto future_result = goal_joint_space_path_client_->async_send_request(request, response_received_callback);

  return false;
}

bool OpenManipulatorXDemo::set_tool_control(std::vector<double> joint_angle)
{
  auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
  request->joint_position.joint_name.push_back("gripper");
  request->joint_position.position = joint_angle;

  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) 
  {
    auto result = future.get();
    return result->is_planned;
  };
  auto future_result = goal_tool_control_client_->async_send_request(request, response_received_callback);

  return false;
}

void OpenManipulatorXDemo::process_callback()
{
  std::vector<std::string> joint_name;
  std::vector<double> joint_angle;
  double path_time;
  if (state_ == 0)
  {
    path_time = 2.0;  // unit: s
    joint_name.push_back("joint1"); joint_angle.push_back(1.57);
    joint_name.push_back("joint2"); joint_angle.push_back(-1.2);
    joint_name.push_back("joint3"); joint_angle.push_back(0.3);
    joint_name.push_back("joint4"); joint_angle.push_back(0.9);
    set_joint_space_path(joint_name, joint_angle, path_time);
    state_++;
  }
  else if (state_ == 1)
  {
    path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(-1.57);
    joint_name.push_back("joint2"); joint_angle.push_back(-1.2);
    joint_name.push_back("joint3"); joint_angle.push_back(0.3);
    joint_name.push_back("joint4"); joint_angle.push_back(0.9);
    set_joint_space_path(joint_name, joint_angle, path_time);
    state_++;
  }
  else if (state_ == 2)
  {
    path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(0.5);
    joint_name.push_back("joint3"); joint_angle.push_back(1.0);
    joint_name.push_back("joint4"); joint_angle.push_back(-1.5);
    set_joint_space_path(joint_name, joint_angle, path_time);
    state_++;
  }
  else if (state_ == 3)
  {
    path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(0.7);
    joint_name.push_back("joint3"); joint_angle.push_back(0.4);
    joint_name.push_back("joint4"); joint_angle.push_back(-1.1);
    set_joint_space_path(joint_name, joint_angle, path_time);
    state_++;
  }
  else if (state_ == 4)
  {
    joint_angle.push_back(0.01);
    set_tool_control(joint_angle);
    state_++;
  }
  else if (state_ == 5)
  {
    joint_angle.push_back(-0.01);
    set_tool_control(joint_angle);
    state_++;
  }
  else if (state_ == 6)
  {
    path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(0.5);
    joint_name.push_back("joint3"); joint_angle.push_back(1.0);
    joint_name.push_back("joint4"); joint_angle.push_back(-1.5);
    set_joint_space_path(joint_name, joint_angle, path_time);
    state_ = 0;
  }
}

/********************************************************************************
** Main
********************************************************************************/
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpenManipulatorXDemo>());
  rclcpp::shutdown();

  return 0;
}
