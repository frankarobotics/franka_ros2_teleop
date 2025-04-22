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

#include <algorithm>
#include <chrono>
#include <string>
#include <memory>
#include <vector>

#include <tinyxml2.h>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace teleop_utils
{

constexpr unsigned int NUM_JOINTS = 7;

inline std::string getRobotNameFromDescription(
  const std::string & robot_description, const rclcpp::Logger & logger)
{
  std::string robot_name;
  tinyxml2::XMLDocument doc;

  if (doc.Parse(robot_description.c_str()) != tinyxml2::XML_SUCCESS) {
    RCLCPP_ERROR(logger, "Failed to parse robot_description.");
    return "";
  }

  tinyxml2::XMLElement * robot_xml = doc.FirstChildElement("robot");
  if (robot_xml) {
    robot_name = robot_xml->Attribute("name");
    if (robot_name.empty()) {
      RCLCPP_ERROR(logger, "Failed to get robot name from XML.");
      return "";
    }
    RCLCPP_INFO(logger, "Extracted Robot Name: %s", robot_name.c_str());
  } else {
    RCLCPP_ERROR(logger, "Robot element not found in XML.");
  }
  return robot_name;
}

inline void limit(std::vector<double> & values, double min, double max)
{
  for (double & value : values) {
    value = std::clamp(value, min, max);
  }
}

template<typename T>
bool check_if_rt_buffer_data_is_valid(const std::shared_ptr<T> * data)
{
  return !(!data || !(*data));
}

template<typename T>
bool check_if_message_too_old(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, const std::shared_ptr<T> * data,
  const int64_t threshold_in_nanoseconds = 2500000)
{
  rclcpp::Time time_now = node->get_clock()->now();
  int64_t time_delta = (time_now - (*data)->header.stamp).nanoseconds();

  return time_delta > threshold_in_nanoseconds;
}

inline void set_command_interfaces(
  std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces,
  const std::vector<double> values)
{
  for (auto i = 0; i < 7; ++i) {
    command_interfaces[i].set_value(values[i]);
  }
}

inline void set_command_interfaces_to_gravity_compensation(
  std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces)
{
  for (auto i = 0; i < 7; ++i) {
    command_interfaces[i].set_value(0.0);
  }
}

template<typename T>
bool sync_wait_for_action_result(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
  typename rclcpp_action::Client<T>::SharedPtr action_client,
  std::shared_future<typename rclcpp_action::ClientGoalHandle<T>::SharedPtr> & future,
  const std::string & action_name, const rclcpp::Logger & logger,
  const std::chrono::duration<long double, std::milli> & timeout =
  std::chrono::duration<long double, std::milli>(-1))
{
  RCLCPP_INFO(logger, "Wait for %s action result", action_name.c_str());
  RCLCPP_DEBUG(
    logger, "Wait for the %s action goal to be accepted by the server", action_name.c_str());
  auto future_return_code = rclcpp::spin_until_future_complete(node_ptr, future, timeout);

  if (future_return_code != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(
      logger, "The %s action goal was not accepted by the action server", action_name.c_str());
    return false;
  }

  RCLCPP_DEBUG(logger, "The %s action goal was accepted by the server", action_name.c_str());
  auto result = action_client->async_get_result(future.get());

  RCLCPP_DEBUG(logger, "Waiting for the %s action result", action_name.c_str());
  future_return_code = rclcpp::spin_until_future_complete(node_ptr, result, timeout);

  if (future_return_code != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(logger, "The %s action was aborted", action_name.c_str());
    return false;
  }

  RCLCPP_DEBUG(logger, "The %s action result was received", action_name.c_str());
  return result.get().code == rclcpp_action::ResultCode::SUCCEEDED;
}

template<typename T>
bool sync_wait_for_service(
  T service, const std::string & type, const std::string & controller_namespace,
  const rclcpp::Logger & logger,
  const std::chrono::duration<long double, std::milli> & timeout =
  std::chrono::duration<long double, std::milli>(-1))
{
  while (!service->wait_for_service(timeout)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        logger, "[%s] %s interrupted while waiting for connection. Exiting.",
        controller_namespace.c_str(), type.c_str());
      return false;
    }
    RCLCPP_DEBUG(
      logger, "[%s] %s not available yet, waiting again ...", controller_namespace.c_str(),
      type.c_str());
  }
  return true;
}

template<typename T>
bool sync_send_goal(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
  typename rclcpp_action::Client<T>::SharedPtr action_client, const typename T::Goal & goal,
  const std::string & action_name, const rclcpp::Logger & logger,
  const std::chrono::duration<long double, std::milli> & timeout =
  std::chrono::duration<long double, std::milli>(-1))
{
  RCLCPP_DEBUG(logger, "Send %s action goal", action_name.c_str());
  auto future = action_client->async_send_goal(goal);

  RCLCPP_DEBUG(logger, "Start waiting for %s action result", action_name.c_str());
  return sync_wait_for_action_result<T>(
    node_ptr, action_client, future, action_name, logger, timeout);
}

template<typename T>
bool sync_wait_for_action_server(
  typename rclcpp_action::Client<T>::SharedPtr action_client, const std::string & action_name,
  const rclcpp::Logger & logger,
  const std::chrono::duration<long double, std::milli> & timeout =
  std::chrono::duration<long double, std::milli>(-1))
{
  const auto result = action_client->wait_for_action_server(timeout);

  if (!result) {
    RCLCPP_ERROR(logger, "The %s action server not available after waiting", action_name.c_str());
  }

  return result;
}

}  // namespace teleop_utils
