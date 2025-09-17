#  Copyright (c) 2025 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import os
import base64
from typing import Any, Dict, List, Optional
import yaml
from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


LEADER_NAMESPACE = "leader"
FOLLOWER_NAMESPACE = "follower"

LEADER_CONTROLLER_NAME = "leader_controller"
FOLLOWER_CONTROLLER_NAME = "follower_controller"

LEADER_INPUT_TOPIC = "franka_robot_state_broadcaster/external_joint_torques"
FOLLOWER_INPUT_TOPIC = "franka_robot_state_broadcaster/measured_joint_states"

LOWER_TORQUE_THRESHOLDS_ACCELERATION = [20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0]
LOWER_TORQUE_THRESHOLD_NOMINAL = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
LOWER_FORCE_THRESHOLDS_ACCELERATION = [20.0, 20.0, 20.0, 20.0, 20.0, 20.0]
LOWER_FORCE_THRESHOLDS_NOMINAL = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]

TIMEOUT_KEY = 'input_topic_timeout'
ALPHA_KEY = 'alpha'
K_GAINS_KEY = 'k_gains'
D_GAINS_KEY = 'd_gains'
MAX_TORQUE_ACCELERATION_KEY = 'upper_torque_thresholds_acceleration'
MAX_TORQUE_NOMINAL_KEY = 'upper_torque_thresholds_nominal'
MAX_FORCE_ACCELERATION_KEY = 'upper_force_thresholds_acceleration'
MAX_FORCE_NOMINAL_KEY = 'upper_force_thresholds_nominal'
LOAD_GRIPPER_KEY = 'load_gripper'
FAKE_HARDWARE_KEY = 'fake_hardware'
URDF_KEY = 'urdf_file'
NAMESPACE_KEY = 'namespace'
BASE_NAMESPACE_KEY = 'base_namespace'
ROBOT_IP_KEY = 'robot_ip'
ARM_ID_KEY = 'arm_id'
ARM_PREFIX_KEY = 'arm_prefix'

default_parameters = {
    TIMEOUT_KEY: 2500000,
    ALPHA_KEY: [3, 3, 3, 3, 1, 1, 1],
    K_GAINS_KEY: [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0],
    D_GAINS_KEY: [30.0, 30.0, 30.0, 30.0, 10.0, 10.0, 5.0],
    MAX_TORQUE_ACCELERATION_KEY: [85.0, 85.0, 85.0, 85.0, 11.0, 11.0, 11.0],
    MAX_TORQUE_NOMINAL_KEY: [85.0, 85.0, 85.0, 85.0, 11.0, 11.0, 11.0],
    MAX_FORCE_ACCELERATION_KEY: [85.0, 85.0, 85.0, 85.0, 11.0, 11.0],
    MAX_FORCE_NOMINAL_KEY: [85.0, 85.0, 85.0, 85.0, 11.0, 11.0],
    LOAD_GRIPPER_KEY: False,
    FAKE_HARDWARE_KEY: False,
    URDF_KEY: "fr3/fr3.urdf.xacro",
    ARM_ID_KEY: "",
    ARM_PREFIX_KEY: "",
    BASE_NAMESPACE_KEY: None,
    ROBOT_IP_KEY: "you_need_to_the_configure_robot_address",
}


def load_yaml(file_path: str) -> Dict[str, Any]:
    """Load a YAML file and return its content as a dictionary."""
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"File not found: {file_path}")
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)


def construct_namespace(*names):
    """Construct a namespace from the provided names, filtering out empty strings."""
    return "/".join(filter(None, names))


def cvt_to_string(input_list: List) -> str:
    """Convert a list of numbers to a comma-separated string."""
    return ', '.join(map(str, map(float, input_list)))


def cvt_to_float_list(input_list: List) -> List[float]:
    """Convert a list of strings or numbers to a list of floats."""
    return list(map(float, input_list))


def get_action_remapping(action: str, target_action: str) -> list:
    """To remap an action one has to remap all topics associated with that action."""
    return [
        (f'{action}/_action/feedback', f'{target_action}/_action/feedback'),
        (f'{action}/_action/status', f'{target_action}/_action/status'),
        (f'{action}/_action/cancel_goal', f'{target_action}/_action/cancel_goal'),
        (f'{action}/_action/get_result', f'{target_action}/_action/get_result'),
        (f'{action}/_action/send_goal', f'{target_action}/_action/send_goal'),
    ]


def create_controller_config(
    input_topic_name: str,
    config
) -> str:
    """Create a YAML configuration file for the teleoperation controllers."""
    random_string: str = base64.urlsafe_b64encode(os.urandom(6)).decode().lower()
    target_file_name = f'/tmp/launch_params_{random_string}'

    input_topic_timeout = config[TIMEOUT_KEY]
    alpha: List[float] = cvt_to_float_list(config[ALPHA_KEY])
    k_gains: List[float] = cvt_to_float_list(config[K_GAINS_KEY])
    d_gains: List[float] = cvt_to_float_list(config[D_GAINS_KEY])

    config_data = {
        "/**": {
            "leader_controller": {
                "ros__parameters": {
                    "arm_id": "fr3",
                    "input_topic": input_topic_name,
                    "input_topic_timeout": input_topic_timeout,
                    "use_input_topic": True,
                    "alpha": alpha,
                }
            },
            "follower_controller": {
                "ros__parameters": {
                    "arm_id": "fr3",
                    "input_topic": input_topic_name,
                    "input_topic_timeout": input_topic_timeout,
                    "k_gains": k_gains,
                    "d_gains": d_gains,
                }
            },
        },
    }

    with open(target_file_name, 'w') as param_file:
        param_file.write(yaml.dump(config_data, default_flow_style=False))

    return target_file_name


def add_robot_launch_config(
        robot_config: Dict[str, Any],
        namespace: str,
        input_topic_name: str,
        controller_name: str,
) -> List[LaunchDescriptionEntity]:
    """Create the shared launch configuration for a single robot."""
    launch_config = []

    config_file = create_controller_config(
        input_topic_name,
        robot_config
    )

    launch_config.append(ExecuteProcess(
        cmd=[[
            FindExecutable(name="ros2"),
            " service call ",
            f"/{namespace}/service_server/set_full_collision_behavior ",
            "franka_msgs/srv/SetFullCollisionBehavior ",
            "\"{ ",
            "lower_torque_thresholds_acceleration: ",
            f"[{cvt_to_string(LOWER_TORQUE_THRESHOLDS_ACCELERATION)}], ",
            "upper_torque_thresholds_acceleration: "
            f"[{cvt_to_string(robot_config[MAX_TORQUE_ACCELERATION_KEY])}], ",
            "lower_torque_thresholds_nominal: ",
            f"[{cvt_to_string(LOWER_TORQUE_THRESHOLD_NOMINAL)}], ",
            "upper_torque_thresholds_nominal: ",
            f"[{cvt_to_string(robot_config[MAX_TORQUE_NOMINAL_KEY])}], ",
            "lower_force_thresholds_acceleration: ",
            f"[{cvt_to_string(LOWER_FORCE_THRESHOLDS_ACCELERATION)}], ",
            "upper_force_thresholds_acceleration: ",
            f"[{cvt_to_string(robot_config[MAX_FORCE_ACCELERATION_KEY])}], ",
            "lower_force_thresholds_nominal: ",
            f"[{cvt_to_string(LOWER_FORCE_THRESHOLDS_NOMINAL)}], ",
            "upper_force_thresholds_nominal: ",
            f"[{cvt_to_string(robot_config[MAX_FORCE_NOMINAL_KEY])}] ",
            "}\"",
        ]],
        shell=True,
        name='set_robot_collision_behavior',
        output='both',
    ))

    launch_config.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('franka_ros2_teleop'),
                    'launch',
                    'teleop_single_robot.launch.py'
                ])
            ),
            launch_arguments={
                'arm_id': robot_config[ARM_ID_KEY],
                'arm_prefix': robot_config[ARM_PREFIX_KEY],
                'namespace': namespace,
                'urdf_file': robot_config[URDF_KEY],
                'robot_ip': robot_config[ROBOT_IP_KEY],
                'load_gripper': str(robot_config[LOAD_GRIPPER_KEY]),
                'use_fake_hardware': str(robot_config[FAKE_HARDWARE_KEY]),
                'fake_sensor_commands': str(robot_config[FAKE_HARDWARE_KEY]),
                'joint_sources': ','.join(["joint_states", "franka_gripper/joint_states"]),
                'joint_state_rate': str(30),
            }.items(),
        )
    )

    controller_manager = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=[
            'move_to_start_example_controller',
            controller_name,
            '--controller-manager-timeout', '30',
            '--inactive',
            '--param-file', config_file
        ],
        parameters=[PathJoinSubstitution([
            FindPackageShare('franka_ros2_teleop'), 'config', "teleop_controllers.yaml",
        ])],
        output='screen',
    )
    launch_config.append(controller_manager)

    return launch_config


def add_pair(pair_config) -> List[LaunchDescriptionEntity]:
    """Create the launch configuration for a pair of robots (leader and follower)."""
    try:
        pair_namespace: str = pair_config['namespace']
    except KeyError:
        raise Exception(
            "You need to specify a namespace for each pair!"
        )

    base_namespace: Optional[str] = pair_config.get(BASE_NAMESPACE_KEY, None)

    leader_namespace = construct_namespace(base_namespace, pair_namespace, "leader")
    follower_namespace = construct_namespace(base_namespace, pair_namespace, "follower")

    teleop_coordinator_node = Node(
        package='franka_ros2_teleop',
        executable='teleop_coordinator',
        arguments=[
            leader_namespace, LEADER_CONTROLLER_NAME, follower_namespace, FOLLOWER_CONTROLLER_NAME
        ],
        output='both',
    )

    launch_config = [teleop_coordinator_node]

    leader_config = pair_config.pop('leader')
    follower_config = pair_config.pop('follower')

    # Create the launch configuration for the leader robot
    resolved_leader_config = pair_config | leader_config

    launch_config.extend(
        add_robot_launch_config(
            resolved_leader_config,
            leader_namespace,
            f"/{follower_namespace}/{LEADER_INPUT_TOPIC}",
            LEADER_CONTROLLER_NAME,
        )
    )

    # Create the launch configuration for the follower robot
    resolved_follower_config = pair_config | follower_config

    launch_config.extend(
        add_robot_launch_config(
            resolved_follower_config,
            follower_namespace,
            f"/{leader_namespace}/{FOLLOWER_INPUT_TOPIC}",
            FOLLOWER_CONTROLLER_NAME
        )
    )

    if resolved_follower_config[LOAD_GRIPPER_KEY]:
        teleop_gripper_node = Node(
            package='franka_ros2_teleop',
            executable='teleop_gripper_node',
            output='both',
            namespace=follower_namespace,
            remappings=[
                (
                    "~/leader/franka_gripper/joint_states",
                    f"/{leader_namespace}/franka_gripper/joint_states"
                ),
                *get_action_remapping(
                    "~/leader/franka_gripper/homing", f"/{leader_namespace}/franka_gripper/homing"
                ),
                *get_action_remapping(
                    "~/follower/franka_gripper/homing",
                    f"/{follower_namespace}/franka_gripper/homing"
                ),
                *get_action_remapping(
                    "~/follower/franka_gripper/grasp",
                    f"/{follower_namespace}/franka_gripper/grasp"
                ),
                *get_action_remapping(
                    "~/follower/franka_gripper/move",
                    f"/{follower_namespace}/franka_gripper/move"
                ),
            ],
        )
        launch_config.append(teleop_gripper_node)

    return launch_config


def generate_launch_configuration_for_all_robots(context) -> List[LaunchDescriptionEntity]:
    """
    Generate the launch configuration for all robots.

    Configuration is based on the provided robot configuration file.
    """
    config_file = LaunchConfiguration('robot_config_file').perform(context)
    base_config = load_yaml(config_file)

    pairs = base_config.pop('pairs', None)

    assert pairs, "You need to add teleoperation pairs (leader and follower) to your config file!"

    launch_config = []

    resolved_base_config = default_parameters | base_config

    for pair_config in pairs:

        resolved_pair_config = resolved_base_config | pair_config

        launch_config.extend(add_pair(resolved_pair_config))

    return launch_config


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('franka_ros2_teleop'), 'config', 'fr3_teleop_config.yaml'
            ]),
            description='Path to the robot configuration file to load',
        ),
        OpaqueFunction(function=generate_launch_configuration_for_all_robots),
    ])
