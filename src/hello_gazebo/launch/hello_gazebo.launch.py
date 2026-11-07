import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world = os.path.join(
        get_package_share_directory('hello_gazebo'), 'worlds', 'hello.sdf')
    gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    headless = LaunchConfiguration('headless')

    # The GUI starts paused so the fall can be watched after pressing play.
    # Headless mode uses -r to run immediately and -s to skip the GUI.
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': world}.items(),
        condition=UnlessCondition(headless),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': f'-r -s {world}'}.items(),
        condition=IfCondition(headless),
    )

    # Forward the box odometry from Gazebo to ROS 2 (Gazebo -> ROS only).
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/box/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        output='screen',
    )

    watcher = Node(
        package='hello_gazebo',
        executable='box_watcher',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo without the GUI'),
        gazebo_gui,
        gazebo_headless,
        bridge,
        watcher,
    ])
