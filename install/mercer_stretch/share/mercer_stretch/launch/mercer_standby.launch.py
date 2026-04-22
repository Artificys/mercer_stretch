import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    stretch_navigation_path = get_package_share_directory('stretch_nav2')

    use_depth_scan = LaunchConfiguration('use_depth_scan')
    declare_use_depth_scan = DeclareLaunchArgument(
        'use_depth_scan',
        default_value = "true",
        description = "use depth camera to detect obstacles ('true' or 'false')"
    )

    scan_sources = "scan"
    if use_depth_scan == "true":
        scan_sources = "scan scan_depth"

    param_rewrites = {
        'observation_sources': scan_sources
    }

    configured_params = RewrittenYaml(
        source_file = os.path.join(stretch_navigation_path, "config/nav2_params.yaml"),
        root_key = "",
        param_rewrites = param_rewrites,
        convert_types = True
    )

    
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
        default_value=os.path.join("/home/hello-robot/stretch_user/maps", "Mercer_Lab_2026.yaml"),
        description='Full path to the map.yaml file to use for navigation')

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

    depth_camera_parameters = [{'name': 'depth_module.profile',         'default': '424x240x15', 'description': 'depth module profile'},
                           {'name': 'depth_module.depth_profile',   'default': '424x240x15', 'description': 'depth module profile'},
                           {'name': 'depth_module.infra_profile',   'default': '424x240x15', 'description': 'depth module profile'},
                           {'name': 'rgb_camera.profile',           'default': '424x240x15', 'description': 'color image width'},
                           {'name': 'rgb_camera.color_profile',     'default': '424x240x15', 'description': 'color image width'},
                           {'name': 'align_depth.enable',           'default': 'true',       'description': 'whether to publish aligned_depth_to_color feed'},
                           {'name': 'device_type',                  'default': 'd435', 'description': "''"}
                           ]

    def declare_configurable_parameters(parameters):
        return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

    d435i_basic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("stretch_core"), "launch"),
        "/d435i_basic.launch.py"])
    )


    depth_camera_launch = LaunchDescription(declare_configurable_parameters(depth_camera_parameters) + [
          d435i_basic_launch
    ])

    depth_camera_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[
            ('depth', '/camera/depth/image_rect_raw'),
            ('depth_camera_info', 'camera/depth/camera_info'),
            ('scan', '/scan_depth')
        ],
        parameters=[{
            'range_min': 0.5,
            'range_max': 2.5,
            'scan_height': 200,
            'output_frame_id': 'base_link'
        }]
    )

    stow_arm = ExecuteProcess(
        cmd=[FindExecutable(name="ros2"), "service", "call", "/stow_the_robot", "std_srvs/srv/Trigger", "{}"],
        output="screen",
        shell="false"
    )

    return LaunchDescription([
        teleop_type_param,
        use_sim_time_param,
        autostart_param,
        map_path_param,
        params_file_param,
        navigation_launch,
        audio_node,
        depth_camera_launch,
        declare_use_depth_scan,
        depth_camera_node,
        stow_arm,
    ])