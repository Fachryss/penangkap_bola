import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('bouncing_ball_catcher')
    params = os.path.join(pkg, 'config', 'params.yaml')

    source = LaunchConfiguration('source')

    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='0',
                              description='Indeks webcam atau path file video'),
        Node(package='bouncing_ball_catcher', executable='camera_node',
             name='camera_node', output='screen',
             parameters=[params, {'source': source}]),
        Node(package='bouncing_ball_catcher', executable='game_node',
             name='game_node', output='screen',
             parameters=[params]),
    ])
