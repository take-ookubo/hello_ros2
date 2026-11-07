import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

JOINTS = ['joint1', 'joint2', 'joint3']


def generate_launch_description():
    share = get_package_share_directory('hello_arm')
    world = os.path.join(share, 'worlds', 'arm_world.sdf')
    with open(os.path.join(share, 'urdf', 'arm.urdf')) as f:
        robot_description = f.read()
    gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    def gazebo(context):
        # -r runs the simulation immediately. -s runs the server without the GUI.
        headless = IfCondition(LaunchConfiguration('headless')).evaluate(context)
        gz_args = f'-r -s {world}' if headless else f'-r {world}'
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={'gz_args': gz_args}.items(),
        )]

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # Without -world, create waits until Gazebo has loaded a world before spawning.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'hello_arm', '-topic', 'robot_description'],
        output='screen',
    )

    # ROS -> Gazebo: joint targets. Gazebo -> ROS: joint states and clock.
    bridge_args = [
        f'/arm/{j}/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double'
        for j in JOINTS
    ] + [
        '/world/arm_world/model/hello_arm/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
    ]
    remappings = [('/world/arm_world/model/hello_arm/joint_state', '/joint_states')]
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        remappings=remappings,
        output='screen',
    )

    mover = Node(
        package='hello_arm',
        executable='arm_mover',
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('auto_move')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo without the GUI'),
        DeclareLaunchArgument(
            'auto_move', default_value='true',
            description='Swing the joints automatically with arm_mover'),
        OpaqueFunction(function=gazebo),
        robot_state_publisher,
        spawn,
        bridge,
        mover,
    ])
