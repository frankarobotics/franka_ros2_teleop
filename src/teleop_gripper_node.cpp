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

#include <cassert>
#include <cmath>

#include <exception>
#include <memory>
#include <string>
#include <variant>

#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/homing.hpp>
#include <franka_msgs/action/move.hpp>
#include <franka_ros2_teleop/utils.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

constexpr auto SERVICE_AVAILABLE_WAIT_DURATION = 2.0s;
constexpr auto HOMING_ACTION_WAIT_DURATION = 10.0s;
constexpr auto ACTION_WAIT_DURATION = 5.0s;

const std::string GRIPPER_HOMED_PARAM_NAME = "gripper_homed";
const std::string GRASP_FORCE_PARAM_NAME = "grasp_force";
const std::string MOVE_SPEED_PARAM_NAME = "move_speed";
const std::string GRIPPER_STATE_TOPIC_NAME = "~/leader/franka_gripper/joint_states";

class TeleopGripperNode : public rclcpp::Node
{
public:
  explicit TeleopGripperNode(const std::string & node_name)
  : Node(node_name)
  {
    joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      GRIPPER_STATE_TOPIC_NAME, 1,
      std::bind(&TeleopGripperNode::gripper_position_callback, this, _1));

    leader_homing_client_ = rclcpp_action::create_client<franka_msgs::action::Homing>(
      this, "~/leader/franka_gripper/homing");
    follower_homing_client_ = rclcpp_action::create_client<franka_msgs::action::Homing>(
      this, "~/follower/franka_gripper/homing");
    grasp_client_ = rclcpp_action::create_client<franka_msgs::action::Grasp>(
      this, "~/follower/franka_gripper/grasp");
    move_client_ = rclcpp_action::create_client<franka_msgs::action::Move>(
      this, "~/follower/franka_gripper/move");

    this->declare_parameter(GRIPPER_HOMED_PARAM_NAME, gripper_homed_);
    this->declare_parameter(GRASP_FORCE_PARAM_NAME, grasp_force_);
    this->declare_parameter(MOVE_SPEED_PARAM_NAME, move_speed_);

    parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&TeleopGripperNode::parameter_update_callback, this, std::placeholders::_1));

    grasping_ = false;
  }

  rcl_interfaces::msg::SetParametersResult parameter_update_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    for (const rclcpp::Parameter & parameter : parameters) {
      if (parameter.get_name() == MOVE_SPEED_PARAM_NAME) {
        if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
          move_speed_ = parameter.as_double();
        }
      }

      if (parameter.get_name() == GRASP_FORCE_PARAM_NAME) {
        if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
          grasp_force_ = parameter.as_double();
        }
      }
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    return result;
  }

  bool init()
  {
    grasping_ = false;
    gripper_homed_ = this->get_parameter(GRIPPER_HOMED_PARAM_NAME).as_bool();

    bool homing_success(false);
    if (!gripper_homed_) {
      RCLCPP_INFO(this->get_logger(), "Homing gripper");
      homing_success = homingGripper();
    }

    const bool move_server_available =
      teleop_utils::sync_wait_for_action_server<franka_msgs::action::Move>(
      move_client_, "move", this->get_logger(), SERVICE_AVAILABLE_WAIT_DURATION);
    const bool grasp_server_available =
      teleop_utils::sync_wait_for_action_server<franka_msgs::action::Grasp>(
      grasp_client_, "grasp", this->get_logger(), SERVICE_AVAILABLE_WAIT_DURATION);

    auto message = sensor_msgs::msg::JointState();
    const bool message_received = rclcpp::wait_for_message(
      message, this->shared_from_this(), GRIPPER_STATE_TOPIC_NAME, SERVICE_AVAILABLE_WAIT_DURATION);

    return (gripper_homed_ || homing_success) && move_server_available && grasp_server_available &&
           message_received;
  }

  bool homingGripper()
  {
    const bool leader_homing_service_available =
      teleop_utils::sync_wait_for_action_server<franka_msgs::action::Homing>(
      leader_homing_client_, "leader homing", this->get_logger(),
      SERVICE_AVAILABLE_WAIT_DURATION);
    const bool follower_homing_service_available =
      teleop_utils::sync_wait_for_action_server<franka_msgs::action::Homing>(
      follower_homing_client_, "follower homing", this->get_logger(),
      SERVICE_AVAILABLE_WAIT_DURATION);

    if (!(leader_homing_service_available && follower_homing_service_available)) {
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Homing the leader and follower gripper");
    auto leader_future =
      leader_homing_client_->async_send_goal(franka_msgs::action::Homing::Goal());
    auto follower_future =
      follower_homing_client_->async_send_goal(franka_msgs::action::Homing::Goal());

    const bool leader_homed =
      teleop_utils::sync_wait_for_action_result<franka_msgs::action::Homing>(
      this->get_node_base_interface(), leader_homing_client_, leader_future, "leader_homing",
      this->get_logger(), HOMING_ACTION_WAIT_DURATION);

    const bool follower_homed =
      teleop_utils::sync_wait_for_action_result<franka_msgs::action::Homing>(
      this->get_node_base_interface(), follower_homing_client_, follower_future,
      "follower_homing", this->get_logger(), HOMING_ACTION_WAIT_DURATION);

    RCLCPP_INFO(this->get_logger(), "Homing leader and follower gripper done");

    return leader_homed && follower_homed;
  }

  void update()
  {
    if (!latest_msg_) {
      return;
    }

    const double gripper_width = 2.0 * latest_msg_->position[0];
    RCLCPP_DEBUG(this->get_logger(), "Gripper width: %f", gripper_width);

    if (!gripper_homed_) {
      RCLCPP_INFO(this->get_logger(), "Max Width: %f", gripper_width);

      max_width_ = gripper_width;

      RCLCPP_INFO(this->get_logger(), "Grasp Threshold: %f", get_grasping_threshold());
      RCLCPP_INFO(this->get_logger(), "Open Threshold: %f", get_opening_threshold());

      gripper_homed_ = true;
    }

    if (gripper_width < get_grasping_threshold() && !grasping_) {
      if (grasp()) {
        grasping_ = true;
      }
    } else if (gripper_width > get_opening_threshold() && grasping_) {
      if (open()) {
        grasping_ = false;
      }
    }
  }

private:
  double get_grasping_threshold() {return relative_grasping_threshold_ * max_width_;}
  double get_opening_threshold() {return relative_opening_threshold_ * max_width_;}

  bool grasp()
  {
    return teleop_utils::sync_send_goal<franka_msgs::action::Grasp>(
      this->get_node_base_interface(), grasp_client_, get_grasp_goal(), "grasping",
      this->get_logger(), ACTION_WAIT_DURATION);
  }

  bool open()
  {
    return teleop_utils::sync_send_goal<franka_msgs::action::Move>(
      this->get_node_base_interface(), move_client_, get_move_goal(), "moving", this->get_logger(),
      ACTION_WAIT_DURATION);
  }

  franka_msgs::action::Grasp::Goal get_grasp_goal()
  {
    franka_msgs::action::Grasp::Goal grasp_goal;

    grasp_goal.force = grasp_force_;
    grasp_goal.speed = move_speed_;
    grasp_goal.epsilon.inner = grasp_epsilon_inner_;
    grasp_goal.epsilon.outer = grasp_epsilon_outer_scaling_ * max_width_;

    return grasp_goal;
  }

  franka_msgs::action::Move::Goal get_move_goal()
  {
    franka_msgs::action::Move::Goal move_goal;

    move_goal.speed = move_speed_;
    move_goal.width = max_width_;

    return move_goal;
  }

  void gripper_position_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    latest_msg_ = msg;
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  sensor_msgs::msg::JointState::SharedPtr latest_msg_;

  rclcpp_action::Client<franka_msgs::action::Homing>::SharedPtr leader_homing_client_;
  rclcpp_action::Client<franka_msgs::action::Homing>::SharedPtr follower_homing_client_;
  rclcpp_action::Client<franka_msgs::action::Grasp>::SharedPtr grasp_client_;
  rclcpp_action::Client<franka_msgs::action::Move>::SharedPtr move_client_;

  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  bool gripper_homed_ = false;

  bool grasping_ = false;
  double max_width_ = 0.07;                  // [m]
  double move_speed_ = 0.5;                  // [m/s]
  double grasp_force_ = 1.0;                 // [N]
  double relative_grasping_threshold_{0.5};  // think %
  double relative_opening_threshold_{0.6};   // think %
  double grasp_epsilon_inner_{0.001};        // [m]
  double grasp_epsilon_outer_scaling_{100.0};
};

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;

  std::shared_ptr<TeleopGripperNode> node =
    std::make_shared<TeleopGripperNode>("teleop_gripper_node");

  rclcpp::Rate loop_rate(30);

  auto executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();

  const bool initialized = node->init();

  if (initialized) {
    RCLCPP_INFO(node->get_logger(), "Initialization successful");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Initialization failed");
  }

  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(0.5s));

  while (rclcpp::ok() && initialized) {
    rclcpp::spin_some(node);
    node->update();
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
