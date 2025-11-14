from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Path to turtlebot4_gz_bringup launch file
    tb4_gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('turtlebot4_gz_bringup'),
                'launch',
                'turtlebot4_gz.launch.py'
            )
        ]),
        launch_arguments={'world': 'maze'}.items()
    )

    # rosbridge_server node
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': 9090,
            'address': '0.0.0.0'
        }]
    )

    return LaunchDescription([
        tb4_gz_launch,
        rosbridge_node
    ])
