import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    
    chit_chat_ns_launch = DeclareLaunchArgument(
        'test_group', default_value=TextSubstitution(text='test_group')
    )

    chit_chat_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('basic_comms'),
                'launch/test_launch.py'
            )
        )
    )

    test_group_launch = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('test_group')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('basic_comms'),
                        'launch/test_launch.py'
                    )
                )
            )
        ]
    )

    l_d = LaunchDescription([chit_chat_ns_launch, chit_chat_launch, test_group_launch])
    return l_d