from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='challenge_1',
            executable='process',
        ),
        Node(
            package='challenge_1',
            executable='signal_generator',
        ),
        Node(
            package='rqt_graph',
            executable='rqt_graph',
            output='screen'
        ),
        Node(
            package='rqt_plot',
            executable='rqt_plot',
            output='screen',
            arguments=['/proc_signal/data', '/proc_signal_2/data']
        )
    ])
