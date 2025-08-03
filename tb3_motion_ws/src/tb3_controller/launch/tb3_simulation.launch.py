from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import subprocess

def generate_trajectory(context, *args, **kwargs):
    # Extract launch args
    x = context.launch_configurations['x']
    y = context.launch_configurations['y']
    yaw = context.launch_configurations['yaw']
    yaml_path = context.launch_configurations['waypoint_file']

    # Use ros2 run instead of calling script directly
    try:
        subprocess.run([
            'ros2', 'run', 'tb3_controller', 'trajectory_generator',
            x, y, yaw, yaml_path
        ], check=True)
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Failed to generate trajectory: {e}")
        raise

    return []

def generate_launch_description():
    tb3_controller_dir = get_package_share_directory('tb3_controller')

    # Construct paths
    default_yaml = os.path.join(tb3_controller_dir, 'config', 'waypoints.yaml')
    trajectory_file = os.path.join(os.path.expanduser('~'), 'tb3_motion_ws', 'data', 'trajectory.npz')

    # Declare arguments
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Initial X position')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Initial Y position')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Initial Yaw (radians)')
    yaml_arg = DeclareLaunchArgument('waypoint_file', default_value=default_yaml, description='Waypoint YAML file')

    # LaunchConfigurations
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    yaml_file = LaunchConfiguration('waypoint_file')

    return LaunchDescription([
        x_arg,
        y_arg,
        yaw_arg,
        yaml_arg,

        # Step 0: Generate trajectory before anything else
        OpaqueFunction(function=generate_trajectory),

        # Step 1: Start Gazebo
        ExecuteProcess(
            cmd=[
                'gazebo',
                '/opt/ros/humble/share/turtlebot3_gazebo/worlds/empty.world',
                '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),

        # Step 2: Spawn robot at initial pose
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-entity', 'burger',
                '-file', '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf',
                '-x', x,
                '-y', y,
                '-z', '0.01',
                '-Y', yaw,
            ],
            output='screen'
        ),

        # Step 3: Start controller
        Node(
            package='tb3_controller',
            executable='pure_pursuit_node',
            name='pure_pursuit_pid',
            output='screen',
            parameters=[{'trajectory_path': trajectory_file}]
        ),

        # Step 4: Start plotter
        Node(
            package='tb3_controller',
            executable='plotter_node',
            name='plotter_node',
            output='screen',
            parameters=[{'trajectory_path': trajectory_file}]
        ),
    ])
