import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    #pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/simple_tracked/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )
    
    controller_node = Node(
        name='dtv_controller_node',
        package='dtv_core',
        executable='dtv_controller',
        output='screen'
    )
    
    joystick_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("teleop_twist_joy"), '/launch', '/teleop-launch.py']),
        launch_arguments={
            'joy_config': 'xbox'
        }.items(),
    )
    
    return LaunchDescription([
        bridge,
        controller_node,
        joystick_node
    ])

