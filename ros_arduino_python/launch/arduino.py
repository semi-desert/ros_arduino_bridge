import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # FIXME yaml file parameters, it's not working now.
    config = os.path.join(
        get_package_share_directory('ros_arduino_python'),
        'config',
        'arduino_params.yaml'
    )
    arduino_cmd = Node(
            package='ros_arduino_python',
            executable='ros_arduino_python',
            name='arduino',
            output='screen',
            parameters=[
                config
            ],
        )
    
    ld = LaunchDescription()
    ld.add_action(arduino_cmd)
    return ld