import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('agri_arm')
    gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    def gazebo_and_model(context):
        headless = IfCondition(LaunchConfiguration('headless')).evaluate(context)
        world = LaunchConfiguration('world').perform(context)
        gz_args = f'-r -s {world}' if headless else f'-r {world}'
        payload = str(float(LaunchConfiguration('payload').perform(context)))
        mode = LaunchConfiguration('control_mode').perform(context)
        if mode not in ('position', 'effort'):
            raise ValueError(f'control_mode must be position or effort, not {mode}')
        controllers = os.path.join(
            share, 'config', 'controllers.yaml' if mode == 'position' else 'controllers_effort.yaml')
        robot_description = xacro.process_file(
            os.path.join(share, 'urdf', 'agri_arm.urdf.xacro'),
            mappings={'payload': payload, 'control_mode': mode,
                      'controllers': controllers,
                      'grasp_objects': LaunchConfiguration('grasp_objects').perform(context),
                      }).toxml()
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim_launch),
                launch_arguments={'gz_args': gz_args}.items()),
            # gz_ros2_control reads robot_description from this node.
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            ),
        ]
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'agri_arm', '-topic', 'robot_description'],
        output='screen',
    )
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # The controller manager starts inside Gazebo after the arm is spawned.
    # Software rendering can make that slow, so wait up to two minutes.
    def spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager-timeout', '120'],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )

    pose_cycle = Node(
        package='agri_arm',
        executable='pose_cycle',
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
            description='Cycle through harvesting poses with pose_cycle'),
        DeclareLaunchArgument(
            'world', default_value=os.path.join(share, 'worlds', 'agri_world.sdf'),
            description='SDF world file to load'),
        DeclareLaunchArgument(
            'control_mode', default_value='position',
            description='position (ideal servo) or effort (PID torque control)'),
        DeclareLaunchArgument(
            'payload', default_value='0.0',
            description='Extra mass [kg] at the tool tip, such as a fruit'),
        DeclareLaunchArgument(
            'grasp_objects', default_value='',
            description='Gazebo models the hand can hold (space separated); '
            'see agri_arm.urdf.xacro'),
        OpaqueFunction(function=gazebo_and_model),
        spawn,
        clock_bridge,
        spawner('joint_state_broadcaster'),
        spawner('arm_controller'),
        spawner('hand_controller'),
        pose_cycle,
    ])
