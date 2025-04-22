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

#include <chrono>
#include <memory>
#include <string>
#include <functional>

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <franka_ros2_teleop/utils.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

constexpr auto WAIT_TIME = 1s;
constexpr int SWITCH_CONTROLLER_BEST_EFFORT = 1;
constexpr int SWITCH_CONTROLLER_STRICT = 2;
const std::string MOVE_TO_START_CONTROLLER = "move_to_start_example_controller";
const std::string TARGET_POSITION_REACHED_PARAMETER = "process_finished";

class ControllerCoordinator
{
public:
  ControllerCoordinator(
    std::shared_ptr<rclcpp::Node> node, const std::string & controller_namespace,
    const std::string & target_controller)
  {
    controller_namespace_ = controller_namespace;
    node_handle_ = node;
    target_controller_ = target_controller;

    move_to_start_controller_parameter_client_ = std::make_shared<rclcpp::SyncParametersClient>(
      node, controller_namespace_ + "/" + MOVE_TO_START_CONTROLLER);

    controller_manager_switch_service_client_ =
      node->create_client<controller_manager_msgs::srv::SwitchController>(
      controller_namespace_ + "/controller_manager/switch_controller");
  }

  bool wait_for_connection() const
  {
    return teleop_utils::sync_wait_for_service(
      move_to_start_controller_parameter_client_, "move_to_start parameter client",
      controller_namespace_, node_handle_->get_logger(), WAIT_TIME) &&
           teleop_utils::sync_wait_for_service(
      controller_manager_switch_service_client_, "switch_controller service client",
      controller_namespace_, node_handle_->get_logger(), WAIT_TIME);
  }

  bool start_move_to_start_controller() const
  {
    return switch_controller({MOVE_TO_START_CONTROLLER}, {});
  }

  bool start_target_controller() const
  {
    return switch_controller({target_controller_}, {MOVE_TO_START_CONTROLLER});
  }

  bool has_reached_target_position() const
  {
    std::stringstream ss;
    ss << "[" << controller_namespace_ << "]";

    bool target_position_reached = false;

    for (auto & parameter : move_to_start_controller_parameter_client_->get_parameters(
        {TARGET_POSITION_REACHED_PARAMETER}))
    {
      ss << " has_reached_target_position: " << parameter.value_to_string();

      target_position_reached = parameter.as_bool();
    }

    RCLCPP_DEBUG(node_handle_->get_logger(), "%s", ss.str().c_str());

    return target_position_reached;
  }

private:
  bool switch_controller(
    const std::vector<std::string> & controllers_to_activate,
    const std::vector<std::string> & controllers_to_deactivate) const
  {
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->activate_controllers = controllers_to_activate;
    request->deactivate_controllers = controllers_to_deactivate;
    request->strictness = SWITCH_CONTROLLER_STRICT;
    request->activate_asap = true;
    request->timeout = rclcpp::Duration(1.0s);

    auto result = controller_manager_switch_service_client_->async_send_request(request);

    rclcpp::spin_until_future_complete(node_handle_, result);

    const bool succeeded = result.get()->ok;

    RCLCPP_INFO(
      node_handle_->get_logger(), "[%s] switch controller %s", controller_namespace_.c_str(),
      (succeeded ? "succeded" : "failed"));

    return succeeded;
  }

  rclcpp::Node::SharedPtr node_handle_;
  std::string controller_namespace_;
  std::string target_controller_;
  rclcpp::SyncParametersClient::SharedPtr move_to_start_controller_parameter_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
    controller_manager_switch_service_client_;
};

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  std::vector<std::string> arguments = rclcpp::init_and_remove_ros_arguments(argc, argv);
  arguments.erase(arguments.begin());

  auto node = rclcpp::Node::make_shared("teleop_coordinator");

  if (arguments.size() % 2 != 0) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Arguments are not even! Usage: ros2 run franka_ros2_teleop teleop_coordinator "
      "<controller_namespace_1> "
      "<target_controller_name_1> [<controller_namespace_2> <target_controller_name_2> ...] ");
    return 1;
  }

  std::vector<ControllerCoordinator> coordinatedControllers;

  for (std::size_t i = 0; i < arguments.size(); i += 2) {
    coordinatedControllers.emplace_back(node, arguments[i], arguments[i + 1]);
    RCLCPP_INFO(
      node->get_logger(), "Controller Interface: %s %s", arguments[i].c_str(),
      arguments[i + 1].c_str());
  }

  rclcpp::Rate loop_rate(10);

  bool running = true;

  for (const ControllerCoordinator & coordinator : coordinatedControllers) {
    running = coordinator.wait_for_connection();
  }

  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(0.5s));

  for (const ControllerCoordinator & coordinator : coordinatedControllers) {
    running = coordinator.start_move_to_start_controller();
  }

  while (rclcpp::ok() && running) {
    bool all_reached = true;
    for (const ControllerCoordinator & coordinator : coordinatedControllers) {
      all_reached = all_reached && coordinator.has_reached_target_position();
    }

    if (all_reached) {
      running = false;
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  for (const ControllerCoordinator & coordinator : coordinatedControllers) {
    coordinator.start_target_controller();
  }

  rclcpp::shutdown();

  return 0;
}
