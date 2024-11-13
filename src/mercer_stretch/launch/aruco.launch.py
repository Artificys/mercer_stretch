import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    dict_file_path = os.path.join(get_package_share_directory('mercer_stretch'), 'config', 'stretch_marker_dict.yaml')

    mercer_aruco = Node(
        package='mercer_stretch',
        executable='mercer_aruco',
        output='screen',
        parameters=[dict_file_path],
        )
    
    stretch_core_path = get_package_share_directory('stretch_core')

    stretch_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(stretch_core_path), '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'trajectory', 'broadcast_odom_tf': 'False', 'fail_out_of_range_goal': 'True'}.items(),
    )

    d435i_launch = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               stretch_core_path, 'launch'),
               '/stretch_realsense.launch.py'])
          )
    
    rviz_config_path = os.path.join(stretch_core_path, 'rviz', 'stretch_simple_test.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        )
    
    return LaunchDescription([
        mercer_aruco,
        d435i_launch,
        stretch_driver,
        rviz_node
    ])