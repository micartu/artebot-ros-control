# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# This file was last modified by Michael Artuerhof
# (michael.artuerhof@gmail.com), on 2024-05-26.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )

    # Initialize Arguments
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("artebot_control"), "urdf", "artebot.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("artebot_control"),
            "config",
            "artebot_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "artebot_diff_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    joint_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "artebot_joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    nodes = [
        control_node,
        robot_controller_spawner,
        joint_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
