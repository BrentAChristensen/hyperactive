# Copyright (C) 2023 Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#!/usr/bin/env python3
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node  # Import the Node class


def load_yaml(package_name, file_name):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_name)
    with open(absolute_file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    include_gripper = LaunchConfiguration("include_gripper")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    ar_model_config = LaunchConfiguration("ar_model")
    
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Make MoveIt use simulation time. This is needed "+\
                "for trajectory planing in simulation.",
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "include_gripper",
            default_value="True",
            description="Run the servo gripper",
            choices=["True", "False"],
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="moveit_with_yolo.rviz",
            description="Full path to the RViz configuration file to use",
        ))
    declared_arguments.append(
        DeclareLaunchArgument("ar_model",
                              default_value="mk3",
                              choices=["mk1", "mk2", "mk3"],
                              description="Model of AR4"))

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare("annin_ar4_description"), "urdf", "ar.urdf.xacro"]),
        " ",
        "ar_model:=",
        ar_model_config,
        " ",
        "include_gripper:=",
        include_gripper,
    ])
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare("annin_ar4_moveit_config"), "srdf", "ar.srdf.xacro"]),
        " ",
        "name:=",
        ar_model_config,
        " ",
        "include_gripper:=",
        include_gripper,
    ])
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    robot_description_kinematics = {
        "robot_description_kinematics":
        load_yaml(
            "annin_ar4_moveit_config",
            os.path.join("config", "kinematics.yaml"),
        )
    }

  # MoveIt Configuration (Adding cartesian_limits here)
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            "annin_ar4_moveit_config",
            os.path.join("config", "joint_limits.yaml"),
        ),
        "robot_description_planning.cartesian_limits.max_trans_vel": 1.0,  # Example value
        "robot_description_planning.cartesian_limits.max_trans_acc": 0.5,   # Example value
        "robot_description_planning.cartesian_limits.max_trans_dec": 0.01,
        "robot_description_planning.cartesian_limits.max_rot_vel": 0.08727   
    }

    # Planning Configuration
    ompl_planning_yaml = load_yaml("annin_ar4_moveit_config",
                                   "config/ompl_planning.yaml")
    pilz_planning_yaml = load_yaml("annin_ar4_moveit_config",
                                   "config/pilz_planning.yaml")
    planning_pipeline_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl", "pilz"],
        "ompl": ompl_planning_yaml,
        "pilz": pilz_planning_yaml,
    }

    # Trajectory Execution Configuration
    controllers_yaml = load_yaml("annin_ar4_moveit_config", "config/controllers.yaml")

    moveit_controllers = {
        "moveit_simple_controller_manager":
        controllers_yaml,
        "moveit_controller_manager":
        "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        # Added due to https://github.com/moveit/moveit2_tutorials/issues/528
        "publish_robot_description_semantic": True,
    }

    # Starts Pilz Industrial Motion Planner MoveGroupSequenceAction and MoveGroupSequenceService servers
    move_group_capabilities = {
        "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService"
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_capabilities,
            {
                "use_sim_time": use_sim_time
            },
        ],
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("annin_ar4_moveit_config"), "rviz", rviz_config_file])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
            {"use_sim_time": use_sim_time},
        ],
    )
    
    # Define the static transform publisher node
    camera_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
       # arguments=['0.0', '-0.7122', '1.1176','0.0','0.0','0.0', 'base_link', 'camera_link'],
        arguments=['0.0','-0.75','0.70','1.076','0.885','0.0', 'base_link', 'camera_link'],  # Corrected to use 'arguments' instead of 'parameters'
    )
    
        # Add the joint_state_publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            "use_sim_time": use_sim_time
        }],
    )
    # Create the camera node
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("realsense2_camera"),
                         "launch", "rs_launch.py")
        ])
    )

    
    # Add DetectionToPoseMapper Node
    detection_to_pose_mapper_node = Node(
        package='feather_tracker',  # Replace with your actual package name
        executable='detection_to_pose_node.py',  # The script/executable name
        name='detection_to_pose_node',
        output='screen',
        parameters=[
            {'target_class_name': 'bare peacock quill'},
            {'detection_topic': '/yolo/detections_3d'}
        ]
    )

    nodes_to_start = [
        move_group_node, joint_state_publisher_node, rviz_node, detection_to_pose_mapper_node,
        realsense, camera_tf_publisher 
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
