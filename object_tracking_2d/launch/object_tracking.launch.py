import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    this_pkg = get_package_share_path('object_tracking_2d')
    param = os.path.join(this_pkg, 'config/param.yaml')

    return LaunchDescription([
        Node(
            package='object_tracking_2d',
            executable='object_tracking_2d_node',
            name='lidar_object_tracker',
            output='screen',
            namespace='tracking', 
            parameters=[
                os.path.join(this_pkg, 'config/param.yaml'), 
                os.path.join(this_pkg, 'config/qos.yaml')
            ],
            emulate_tty=True,
        ),
    ])
