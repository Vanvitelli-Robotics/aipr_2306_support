/*
    robot_joint_vel_control node ROS2

    Copyright 2024 Università degli studi della Campania "Luigi Vanvitelli"
    Author: Marco Costanzo <marco.costanzo@unicampania.it>
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/set_bool.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <rclcpp/wait_for_message.hpp>

bool vel_mode = false;
sensor_msgs::msg::JointState cmd_joint_position, cmd_joint_vel;
std::shared_ptr<rclcpp::Node> node = nullptr;

void initIntegrator()
{
  rclcpp::wait_for_message<sensor_msgs::msg::JointState>(cmd_joint_position, node, "/joint_states");
  cmd_joint_position.name.pop_back();
  cmd_joint_position.name.pop_back();
  cmd_joint_position.position.pop_back();
  cmd_joint_position.position.pop_back();

  cmd_joint_vel = cmd_joint_position;
  cmd_joint_vel.velocity.resize(7);
  for (std::size_t i = 0; i < cmd_joint_vel.velocity.size(); i++)
  {
    cmd_joint_vel.velocity[i] = 0.0;
  }
  RCLCPP_INFO_STREAM(node->get_logger(), "[ROBOT CONTROL]: robot state is\n"
                                             << sensor_msgs::msg::to_yaml(cmd_joint_position));
}

void set_mode_vel_cmd_srv_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                             std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  if (req->data)
  {
    // vel mode
    initIntegrator();
    RCLCPP_INFO_STREAM(node->get_logger(), "[ROBOT CONTROL]: joint_velocity");
  }
  else
  {
    // position mode
    RCLCPP_INFO_STREAM(node->get_logger(), "[ROBOT CONTROL]: joint_position");
  }

  vel_mode = req->data;

  res->success = true;
}

void cmd_joint_vel_cb(const sensor_msgs::msg::JointState::ConstSharedPtr& msg)
{
  if (!vel_mode)
  {
    RCLCPP_ERROR_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                                 "[ROBOT CONTROL]: the robot is not in velocity mode!");
    return;
  }
  if (msg->velocity.size() == cmd_joint_vel.velocity.size())
  {
    cmd_joint_vel.velocity = msg->velocity;
  }
  else
  {
    RCLCPP_ERROR_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                                 "[ROBOT CONTROL]: invalid command velocity vector size!");
  }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  node = rclcpp::Node::make_shared("robot_joint_vel_control");

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_mode_vel =
      node->create_service<std_srvs::srv::SetBool>("/velocity_mode", set_mode_vel_cmd_srv_cb);

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_cmd_joint_position =
      node->create_publisher<sensor_msgs::msg::JointState>("/cmd/joint_position", 1);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_cmd_joint_vel =
      node->create_subscription<sensor_msgs::msg::JointState>("/cmd/joint_velocity", 1, cmd_joint_vel_cb);

  RCLCPP_INFO_STREAM(node->get_logger(), "[ROBOT CONTROL] Initialized");

  double DT = 0.01;
  rclcpp::Rate loop_rate(1.0 / DT);
  while (rclcpp::ok())
  {
    if (!vel_mode)
    {
      loop_rate.sleep();
      rclcpp::spin_some(node);
      continue;
    }

    for (std::size_t i = 0; i < cmd_joint_position.position.size(); i++)
    {
      cmd_joint_position.position[i] += DT * cmd_joint_vel.velocity[i];
    }

    pub_cmd_joint_position->publish(cmd_joint_position);
    rclcpp::spin_some(node);
  }

  return 0;
}