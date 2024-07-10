from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    arduino_cmd = Node(
            package='ros_arduino_python',
            executable='arduino_node',
            name='arduino',
            output='screen',
            parameters=[
                {'path_to_params_file': 'config/my_arduino_params.yaml'}
            ],
            # clear_params=True
        )
    
    ld = LaunchDescription()
    ld.add_action(arduino_cmd)

    return ld