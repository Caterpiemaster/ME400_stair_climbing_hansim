from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    joy_params = os.path.join(get_package_share_directory('main_ctrl'),'config','joystick.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            # parameters=[joy_params, {'use_sim_time': use_sim_time}],
            parameters=[joy_params],
         )

    main_control_node = Node(
            package='main_ctrl',
            executable='listener',
            name='listener'
        )

    publisher_node = Node(
            package='main_ctrl',
            executable='talker',
            name='talker'
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        joy_node,
        publisher_node,
        main_control_node,        
    ])