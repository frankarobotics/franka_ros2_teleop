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

#include <string>
#include <cassert>
#include <cmath>
#include <exception>

#include <Eigen/Eigen>
#include <franka_ros2_teleop/teleop_follower_controller.hpp>
#include <franka_ros2_teleop/utils.hpp>

namespace franka_ros2_teleop
{

controller_interface::InterfaceConfiguration
TeleopFollowerController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (unsigned int i = 1; i <= teleop_utils::NUM_JOINTS; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
TeleopFollowerController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (unsigned int i = 1; i <= teleop_utils::NUM_JOINTS; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type TeleopFollowerController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  updateJointStates();
  Vector7d q_goal = initial_q_;

  const auto input = measured_joint_states_from_leader_buffer_ptr_.readFromRT();

  if (teleop_utils::check_if_rt_buffer_data_is_valid(input)) {
    if (teleop_utils::check_if_message_too_old(get_node(), input, input_topic_timeout_)) {
      teleop_utils::set_command_interfaces_to_gravity_compensation(command_interfaces_);

      RCLCPP_ERROR(this->get_node()->get_logger(), "Latest message is too old");
      return controller_interface::return_type::ERROR;
    }

    Eigen::Map<Eigen::VectorXd>(q_goal.data(), teleop_utils::NUM_JOINTS) =
      Eigen::Map<Eigen::VectorXd>((*input)->position.data(), teleop_utils::NUM_JOINTS);
  }

  // standard joint impedance controller from franka example controllers
  // the target position comes from the leader
  constexpr double kAlpha = 0.99;
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
  Vector7d tau_d_calculated =
    k_gains_.cwiseProduct(q_goal - q_) + d_gains_.cwiseProduct(-dq_filtered_);
  for (unsigned int i = 0; i < teleop_utils::NUM_JOINTS; ++i) {
    command_interfaces_[i].set_value(tau_d_calculated(i));
  }
  return controller_interface::return_type::OK;
}

CallbackReturn TeleopFollowerController::on_init()
{
  try {
    auto_declare<std::string>("arm_id", "");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_node()->get_logger(), "Exception thrown during init stage with message: %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn TeleopFollowerController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  input_topic_ = get_node()->get_parameter("input_topic").as_string();
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  if (k_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (k_gains.size() != static_cast<uint>(teleop_utils::NUM_JOINTS)) {
    RCLCPP_FATAL(
      get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
      teleop_utils::NUM_JOINTS, k_gains.size());
    return CallbackReturn::FAILURE;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (d_gains.size() != static_cast<uint>(teleop_utils::NUM_JOINTS)) {
    RCLCPP_FATAL(
      get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
      teleop_utils::NUM_JOINTS, d_gains.size());
    return CallbackReturn::FAILURE;
  }
  for (unsigned int i = 0; i < teleop_utils::NUM_JOINTS; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }
  dq_filtered_.setZero();

  auto parameters_client =
    std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  arm_id_ = teleop_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());
  input_topic_ = get_node()->get_parameter("input_topic").as_string();
  RCLCPP_INFO(this->get_node()->get_logger(), "Using input_topic: %s", input_topic_.c_str());
  input_topic_timeout_ = get_node()->get_parameter("input_topic_timeout").as_int();
  RCLCPP_INFO(
    this->get_node()->get_logger(), "Using input_topic_timeout: %ld", input_topic_timeout_);

  measured_joint_states_from_leader_subscriber_ =
    this->get_node()->create_subscription<sensor_msgs::msg::JointState>(
    input_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      // check if message is correct size, if not ignore
      if (msg->position.size() == teleop_utils::NUM_JOINTS) {
        measured_joint_states_from_leader_buffer_ptr_.writeFromNonRT(msg);
      } else {
        RCLCPP_ERROR(
          this->get_node()->get_logger(),
          "Invalid command received for %zu joints, expected "
          "command for %u size",
          msg->position.size(), teleop_utils::NUM_JOINTS);
      }
    });

  return CallbackReturn::SUCCESS;
}

CallbackReturn TeleopFollowerController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  updateJointStates();
  dq_filtered_.setZero();
  initial_q_ = q_;
  elapsed_time_ = 0.0;

  measured_joint_states_from_leader_buffer_ptr_ =
    realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::JointState>>(nullptr);

  return CallbackReturn::SUCCESS;
}

CallbackReturn TeleopFollowerController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  measured_joint_states_from_leader_buffer_ptr_ =
    realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::JointState>>(nullptr);

  return controller_interface::CallbackReturn::SUCCESS;
}

void TeleopFollowerController::updateJointStates()
{
  for (unsigned int i = 0; i < teleop_utils::NUM_JOINTS; ++i) {
    const auto & position_interface = state_interfaces_.at(2 * i);
    const auto & velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

}  // namespace franka_ros2_teleop

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(
  franka_ros2_teleop::TeleopFollowerController, controller_interface::ControllerInterface)
