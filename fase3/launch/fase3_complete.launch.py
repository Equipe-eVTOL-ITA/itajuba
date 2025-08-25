#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Get package directories
    pkg_fase3 = get_package_share_directory('itajuba_fase3')
    pkg_cv_utils = get_package_share_directory('itajuba_cv_utils')
    
    # Configuration files
    fsm_params = os.path.join(pkg_fase3, "config", "fsm.yaml")
    detector_params = os.path.join(pkg_cv_utils, 'fase3', 'hsv_fase3.yaml')
    rviz_cfg = os.path.join(pkg_fase3, 'launch', 'drone_viz.rviz')

    # Launch arguments
    exec_arg = DeclareLaunchArgument(
        "mission",
        default_value="fase3",
        description="Executable that implements the mission FSM")

    # Color detector node (updated to use itajuba_cv_utils)
    detector_node = Node(
        package='itajuba_cv_utils',
        executable='fase3_color_detector',
        name='fase3_color_detector',
        parameters=[detector_params],
        output='screen'
    )

    # Main fase3 mission node
    fase3_node = Node(
        package='itajuba_fase3',
        executable=LaunchConfiguration("mission"),
        parameters=[fsm_params],
        output='screen'
    )

    # Position bridge for rviz visualization
    bridge_node = Node(
        package='sae_drone_lib',
        executable='pos_to_rviz',
        name='pos_to_rviz',
        output='screen'
    )

    # RViz visualization (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen'
    )

    # Delay the main mission node to allow detector to start first
    delayed_fase3_node = TimerAction(period=3.0, actions=[fase3_node])

    return LaunchDescription([
        exec_arg,
        detector_node,
        bridge_node,
        # rviz_node,  # Uncomment if you want RViz to start automatically
        delayed_fase3_node
    ])
