"""Gazebo + ros2_control + MoveIt 2 for agri_arm, with an optional demo."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    arm_share = get_package_share_directory('agri_arm')
    share = get_package_share_directory('agri_arm_moveit_config')

    def moveit_nodes(context):
        payload = str(float(LaunchConfiguration('payload').perform(context)))
        # Use the same model and payload as the Gazebo side.
        moveit_config = (
            MoveItConfigsBuilder('agri_arm', package_name='agri_arm_moveit_config')
            .robot_description(
                file_path=os.path.join(arm_share, 'urdf', 'agri_arm.urdf.xacro'),
                mappings={'payload': payload})
            .robot_description_semantic(file_path='config/agri_arm.srdf')
            .robot_description_kinematics(file_path='config/kinematics.yaml')
            .joint_limits(file_path='config/joint_limits.yaml')
            .trajectory_execution(file_path='config/moveit_controllers.yaml')
            .planning_pipelines(
                pipelines=['ompl', 'pilz_industrial_motion_planner'],
                default_planning_pipeline='ompl')
            .pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
            # robot_state_publisher already publishes robot_description.
            .planning_scene_monitor(
                publish_robot_description=False,
                publish_robot_description_semantic=True)
            .to_moveit_configs()
        )
        params = [moveit_config.to_dict(), {'use_sim_time': True}]
        rviz = Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(share, 'config', 'moveit.rviz')],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
                {'use_sim_time': True},
            ],
            output='log',
            condition=IfCondition(LaunchConfiguration('rviz')),
        )
        return [
            Node(package='moveit_ros_move_group', executable='move_group',
                 parameters=params, output='screen'),
            rviz,
        ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm_share, 'launch', 'agri_arm_gazebo.launch.py')),
        launch_arguments={
            'world': os.path.join(share, 'worlds', 'harvest_world.sdf'),
            'headless': LaunchConfiguration('headless'),
            'payload': LaunchConfiguration('payload'),
            'auto_move': 'false',
        }.items())

    # Always started: it adds the post and floor to the planning scene.
    # With demo:=false it exits after that instead of moving the arm.
    demo = Node(
        package='agri_arm_moveit_config',
        executable='harvest_demo',
        parameters=[{
            'use_sim_time': True,
            'run_motions': LaunchConfiguration('demo'),
            'cycles': LaunchConfiguration('cycles'),
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Gazebo without the GUI'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz with the MotionPlanning panel'),
        DeclareLaunchArgument('demo', default_value='true',
                              description='Run the harvest_demo motion sequence'),
        DeclareLaunchArgument('cycles', default_value='0',
                              description='Demo cycles, 0 repeats forever'),
        DeclareLaunchArgument('payload', default_value='0.0',
                              description='Extra mass [kg] at the tool tip'),
        gazebo,
        OpaqueFunction(function=moveit_nodes),
        demo,
    ])
