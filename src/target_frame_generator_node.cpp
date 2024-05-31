/*
    target_frame_generator node ROS2

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
#include "tf2_ros/transform_broadcaster.h"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("target_frame_generator");

  tf2_ros::TransformBroadcaster tf_br(node);

  RCLCPP_INFO_STREAM(node->get_logger(), "[TARGET FRAME GENERATOR] Initialized");

  rclcpp::Rate loop_rate(10);
  while (rclcpp::ok())
  {
    loop_rate.sleep();
    rclcpp::spin_some(node);

    rclcpp::Time t = node->now();

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = t;
    transform.header.frame_id = "world";
    transform.child_frame_id = "target_frame";
    // translation
    transform.transform.translation.x = 0.5545;
    transform.transform.translation.y = 0.0 + 0.2 * sin(2.0 * M_PI * (1.0 / 20.0) * t.seconds());
    transform.transform.translation.z = 0.5211;
    // rotation
    transform.transform.rotation.x = 1.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 0.0;

    tf_br.sendTransform(transform);
  }

  return 0;
}
