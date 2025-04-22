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

#include <algorithm>
#include <exception>
#include <string>

#include <franka_ros2_teleop/teleop_leader_controller.hpp>
#include <franka_ros2_teleop/utils.hpp>

namespace franka_ros2_teleop
{

controller_interface::InterfaceConfiguration
TeleopLeaderController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (unsigned int i = 1; i <= teleop_utils::NUM_JOINTS; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

template<typename T>
auto sgn(T val) -> int
{
  // returns the sign of the value: -1, 0 or +1
  return (T(0) < val) - (val < T(0));
}

controller_interface::InterfaceConfiguration TeleopLeaderController::state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (unsigned int i = 1; i <= teleop_utils::NUM_JOINTS; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type TeleopLeaderController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto input = external_joint_torques_from_follower_buffer_ptr_.readFromRT();

  std::vector<double> tau_in{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  updateJointStates();

  if (!use_input_topic_) {
    teleop_utils::set_command_interfaces_to_gravity_compensation(command_interfaces_);
    return controller_interface::return_type::OK;
  }

  if (use_input_topic_ && teleop_utils::check_if_rt_buffer_data_is_valid(input)) {
    if (teleop_utils::check_if_message_too_old(get_node(), input, input_topic_timeout_)) {
      teleop_utils::set_command_interfaces_to_gravity_compensation(command_interfaces_);

      RCLCPP_ERROR(this->get_node()->get_logger(), "Latest message is too old");
      return controller_interface::return_type::ERROR;
    }

    tau_in = (*input)->effort;
  }

  // takes and inverts the external joints from the follower
  // has an extra that avoids a feedback loop that results in the leader
  // "jumping away" after contact of the follower
  std::vector<double> leader_torque_commands(teleop_utils::NUM_JOINTS);
  for (unsigned int i = 0; i < teleop_utils::NUM_JOINTS; ++i) {
    const double power_in = leader_velocity_[i] * tau_in[i];

    double feedback_avoidance_term = 0.0;

    if (power_in < 0) {
      feedback_avoidance_term =
        -feedback_avoidance_alpha_[i] * sgn(leader_velocity_[i]) * fabs(power_in);
    }

    leader_torque_commands[i] = -tau_in[i] + feedback_avoidance_term;
  }

  teleop_utils::set_command_interfaces(command_interfaces_, leader_torque_commands);

  return controller_interface::return_type::OK;
}

CallbackReturn TeleopLeaderController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  RCLCPP_INFO(this->get_node()->get_logger(), "Using arm_id: %s", arm_id_.c_str());

  input_topic_ = get_node()->get_parameter("input_topic").as_string();
  RCLCPP_INFO(this->get_node()->get_logger(), "Using input_topic: %s", input_topic_.c_str());

  input_topic_timeout_ = get_node()->get_parameter("input_topic_timeout").as_int();
  RCLCPP_INFO(
    this->get_node()->get_logger(), "Using input_topic_timeout: %ld", input_topic_timeout_);

  use_input_topic_ = get_node()->get_parameter("use_input_topic").as_bool();
  RCLCPP_INFO(
    this->get_node()->get_logger(), "Using use_input_topic: %s",
    use_input_topic_ ? "True" : "False");

  feedback_avoidance_alpha_ = get_node()->get_parameter("alpha").as_double_array();
  std::stringstream stream;
  stream << "Using alpha: [";
  for (unsigned int i = 0; i < teleop_utils::NUM_JOINTS - 1; ++i) {
    stream << feedback_avoidance_alpha_[i] << ", ";
  }
  stream << feedback_avoidance_alpha_[teleop_utils::NUM_JOINTS - 1] << "]";
  RCLCPP_INFO(this->get_node()->get_logger(), stream.str().c_str());

  if (use_input_topic_) {
    external_joint_torques_from_follower_subscriber_ =
      this->get_node()->create_subscription<sensor_msgs::msg::JointState>(
      input_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->effort.size() == teleop_utils::NUM_JOINTS) {
          external_joint_torques_from_follower_buffer_ptr_.writeFromNonRT(msg);
        } else {
          RCLCPP_ERROR(
            this->get_node()->get_logger(),
            "Invalid command received for %zu joints, "
            "expected command for %u size",
            msg->effort.size(), teleop_utils::NUM_JOINTS);
        }
      });
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn TeleopLeaderController::on_init()
{
  try {
    auto_declare<std::string>("arm_id", "fr3");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_node()->get_logger(), "Exception thrown during init stage with message: %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }

  external_joint_torques_from_follower_buffer_ptr_ =
    realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::JointState>>(nullptr);

  return CallbackReturn::SUCCESS;
}

CallbackReturn TeleopLeaderController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/
)
{
  external_joint_torques_from_follower_buffer_ptr_ =
    realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::JointState>>(nullptr);

  return controller_interface::CallbackReturn::SUCCESS;
}

void TeleopLeaderController::updateJointStates()
{
  for (unsigned int i = 0; i < teleop_utils::NUM_JOINTS; ++i) {
    const auto & velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(velocity_interface.get_interface_name() == "velocity");

    leader_velocity_[i] = velocity_interface.get_value();
  }
}

}  // namespace franka_ros2_teleop
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(
  franka_ros2_teleop::TeleopLeaderController, controller_interface::ControllerInterface)
