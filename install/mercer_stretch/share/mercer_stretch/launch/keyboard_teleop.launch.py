import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    mercer_keyboard = Node(
        package='mercer_stretch',
        executable='keyboard_teleop',
        )

    return LaunchDescription([
        mercer_keyboard,
        ])