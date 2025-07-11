import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    my_robot_path = get_package_share_directory('mercer_stretch')
    urdf = os.path.join(my_robot_path , 'mercer_stretch.urdf')
    rviz_config = os.path.join(my_robot_path, 'mercer_stretch.rviz')
    #rviz_config = '/home/hello-robot/mercer_ws/src/mercer_stretch/rviz/mercer_stretch.rviz'
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    sim_time_arg = DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false', choices=['true','false'],
            description='Use simulation (Gazebo) clock if true')
    
    gui_arg = DeclareLaunchArgument(name='gui', 
            default_value='true', choices=['true', 'false'], 
            description='Flag to enable joint_state_publisher_gui')
            
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(rviz_config),
                                     description='Absolute path to rviz config file')

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
        
        # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))    )
        
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
        arguments=[urdf])
        
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')])

    return LaunchDescription([
        sim_time_arg,
        gui_arg,
        rviz_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
