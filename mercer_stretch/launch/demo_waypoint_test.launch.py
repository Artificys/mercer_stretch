import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    stretch_navigation_path = get_package_share_directory('stretch_nav2')
    
    teleop_type_param = DeclareLaunchArgument(
        'teleop_type', default_value="joystick", description="how to teleop ('keyboard', 'joystick' or 'none')")
    
    use_sim_time_param = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    autostart_param = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Whether to autostart lifecycle nodes on launch')

    map_path_param = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(stretch_navigation_path,
                                   'map', '/home/hello-robot/stretch_user/maps/CII_8th_Floor_w_RRC_09_22_24.yaml'),
        description='Full path to the map.yaml file to use for navigation')

    route_param = DeclareLaunchArgument(
        'route_file',
        default_value=os.path.join(stretch_navigation_path,
                                   'route_file', '/home/hello-robot/stretch_user/navigation_config/RRC_Test.json'),
        description='Full route file to use for navigation')

    params_file_param = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(stretch_navigation_path, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_navigation_path, '/launch/navigation.launch.py']),
        launch_arguments={'teleop_type': LaunchConfiguration('teleop_type'),
                          'use_sim_time': LaunchConfiguration('use_sim_time'), 
                          'autostart': LaunchConfiguration('autostart'),
                          'map': LaunchConfiguration('map'),
                          'params_file': LaunchConfiguration('params_file')}.items())

    audio_node = Node(
        package='mercer_stretch',
        executable='mercer_audio',
        name='mercer_audio',
        output='screen'
    )

    demo_security_node = Node(
            package='mercer_stretch',
            executable='mercer_nav',
            name='mercer_nav',
            output='screen',
            parameters=[{'route_file': LaunchConfiguration('route_file')}]
    )

    return LaunchDescription([
        teleop_type_param,
        use_sim_time_param,
        autostart_param,
        map_path_param,
        route_param,
        params_file_param,
        navigation_launch,
        demo_security_node,
        audio_node
    ])