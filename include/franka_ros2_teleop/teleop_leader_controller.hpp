// Copyright (c) 2025 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <string>
#include <vector>
#include <memory>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_ros2_teleop
{
/**
 * The gravity compensation controller only sends zero torques so that the robot
 * does gravity compensation
 */
class TeleopLeaderController : public controller_interface::ControllerInterface
{
public:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_init() override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
  const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
  const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::JointState>>
  external_joint_torques_from_follower_buffer_ptr_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    external_joint_torques_from_follower_subscriber_;

private:
  std::string arm_id_;
  std::string input_topic_;
  int64_t input_topic_timeout_ = 2500000;
  bool use_input_topic_ = true;
  std::vector<double> feedback_avoidance_alpha_{18.75, 15, 13.5, 9, 5.25, 3, 1.5};

  std::vector<double> leader_velocity_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  void updateJointStates();
};
}  // namespace franka_ros2_teleop
