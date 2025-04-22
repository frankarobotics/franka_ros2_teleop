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


import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Generates the "default" nodes (controller_manager, robot_state_publisher, etc.)
# for the Franka robot. This function is called by the main launch file.
# It uses the xacro library to process the URDF file and generate the robot description.


def generate_robot_nodes(context):
    urdf_path = PathJoinSubstitution(
        [
            FindPackageShare("franka_description"),
            "robots",
            LaunchConfiguration("urdf_file"),
        ]
    ).perform(context)
    robot_description = xacro.process_file(
        urdf_path,
        mappings={
            "ros2_control": "true",
            "arm_id": LaunchConfiguration("arm_id").perform(context),
            "arm_prefix": LaunchConfiguration("arm_prefix").perform(context),
            "robot_ip": LaunchConfiguration("robot_ip").perform(context),
            "hand": LaunchConfiguration("load_gripper").perform(context),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware").perform(
                context
            ),
            "fake_sensor_commands": LaunchConfiguration("fake_sensor_commands").perform(
                context
            ),
        },
    ).toprettyxml(indent="  ")

    namespace = LaunchConfiguration("namespace").perform(context)
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("franka_ros2_teleop"), "config", "teleop_controllers.yaml"]
    ).perform(context)

    joint_sources_str = LaunchConfiguration("joint_sources").perform(context)
    joint_sources = joint_sources_str.split(",")
    joint_state_rate = int(LaunchConfiguration("joint_state_rate").perform(context))

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=namespace,
            parameters=[controllers_yaml, {"robot_description": robot_description}],
            output="screen",
            on_exit=Shutdown(),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            namespace=namespace,
            parameters=[
                {
                    "joints": joint_sources,
                    "rate": joint_state_rate,
                    "use_robot_description": False,
                }
            ],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=["joint_state_broadcaster"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=["franka_robot_state_broadcaster"],
            parameters=[{"arm_id": LaunchConfiguration("arm_id").perform(context)}],
            condition=UnlessCondition(LaunchConfiguration("use_fake_hardware")),
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("franka_gripper"),
                            "launch",
                            "gripper.launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments={
                "namespace": namespace,
                "robot_ip": LaunchConfiguration("robot_ip").perform(context),
                "use_fake_hardware": LaunchConfiguration("use_fake_hardware").perform(
                    context
                ),
            }.items(),
            condition=IfCondition(LaunchConfiguration("load_gripper")),
        ),
    ]

    return nodes


# The generate_launch_description function is the entry point (like "main")
# We use it to declare the launch arguments and call the generate_robot_nodes function.


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            "arm_id", default_value="", description="ID of the type of arm used"
        ),
        DeclareLaunchArgument(
            "arm_prefix", default_value="", description="Prefix for arm topics"
        ),
        DeclareLaunchArgument(
            "namespace", default_value="", description="Namespace for the robot"
        ),
        DeclareLaunchArgument(
            "urdf_file",
            default_value="fr3/fr3.urdf.xacro",
            description="Path to URDF file",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="172.16.0.3",
            description="Hostname or IP address of the robot",
        ),
        DeclareLaunchArgument(
            "load_gripper",
            default_value="false",
            description="Use Franka Gripper as an end-effector",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware", default_value="false", description="Use fake hardware"
        ),
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Fake sensor commands",
        ),
        DeclareLaunchArgument(
            "joint_sources",
            default_value="joint_states,franka_gripper/joint_states",
            description="Comma-separated list of joint state topics",
        ),
        DeclareLaunchArgument(
            "joint_state_rate",
            default_value="30",
            description="Rate for joint state publishing (Hz)",
        ),
    ]

    return LaunchDescription(
        launch_args + [OpaqueFunction(function=generate_robot_nodes)]
    )
