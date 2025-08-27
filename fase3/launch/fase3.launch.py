from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_fase3   = get_package_share_directory('itajuba_fase3')
    pkg_cv_utils = get_package_share_directory('itajuba_cv_utils')
    params      = os.path.join(pkg_fase3, "config", "fsm.yaml")
    detector_params = os.path.join(pkg_fase3, 'config', 'onboard.yaml')
    rviz_cfg = os.path.join(pkg_fase3, 'launch', 'drone_viz.rviz')

    exec_arg = DeclareLaunchArgument(
        "mission",
        default_value="fase3",
        description="Executable that implements the mission FSM")

    detector_node = Node(
        package='itajuba_cv_utils',
        executable='fase3_color_detector',
        name='fase3_color_detector',
        parameters=[detector_params],
        output='screen'
    )

    fase3_node = Node(
        package='itajuba_fase3',
        executable=LaunchConfiguration("mission"),
        parameters=[params],
        output='screen'
    )

    bridge_node = Node(
        package='itajuba_drone_lib',
        executable='pos_to_rviz',
        output='screen'
    )

    delayed_fase3_node = TimerAction(period=5.0, actions=[fase3_node])

    return LaunchDescription([
        exec_arg,
        detector_node,
        bridge_node,
        delayed_fase3_node
    ])
