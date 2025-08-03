from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    tb3_bringup_dir = get_package_share_directory('turtlebot3_bringup')
    tb3_controller_dir = get_package_share_directory('tb3_controller')

    rviz_config_file = os.path.join(tb3_controller_dir, 'rviz', 'model.rviz')

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )

    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        namespace_arg,

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_gui': False}]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_bringup_dir, 'launch', 'turtlebot3_state_publisher.launch.py')
            ),
            launch_arguments={'namespace': namespace}.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=namespace,
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
    ])