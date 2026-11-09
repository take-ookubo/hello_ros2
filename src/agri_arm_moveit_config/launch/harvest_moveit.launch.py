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


# Ripe fruit models in worlds/harvest_world.sdf that the hand can pick.
FRUIT_MODELS = ['fruit_left', 'fruit_right']


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
            'control_mode': LaunchConfiguration('control_mode'),
            'auto_move': 'false',
            'grasp_objects': ' '.join(FRUIT_MODELS),
        }.items())

    # Hand and plant joints in Gazebo (std_msgs/Empty -> gz.msgs.Empty),
    # fruit poses for checking where they ended up, and set_pose to hang
    # the fruit back on the plant before the next cycle.
    gz_topics = [f'/{m}/{part}/{action}@std_msgs/msg/Empty]gz.msgs.Empty'
                 for m in FRUIT_MODELS for part in ('hand', 'stem')
                 for action in ('attach', 'detach')]
    sim_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=gz_topics + [
            f'/model/{m}/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose' for m in FRUIT_MODELS
        ] + ['/world/harvest_world/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Always started: it adds the post and floor to the planning scene.
    # With demo:=false it exits after that instead of moving the arm.
    def demo_node(context):
        side = LaunchConfiguration('side_views').perform(context).lower() == 'true'
        return [Node(
            package='agri_arm_moveit_config',
            executable='harvest_demo',
            parameters=[{
                'use_sim_time': True,
                'run_motions': LaunchConfiguration('demo'),
                'cycles': LaunchConfiguration('cycles'),
                'use_camera': LaunchConfiguration('camera'),
                'approach': LaunchConfiguration('approach'),
                'use_wrist_camera': LaunchConfiguration('wrist_camera'),
                'wrist_view_yaws': [0.0, 45.0, -45.0] if side else [0.0],
                'if_unseen': LaunchConfiguration('if_unseen'),
                'align_roll': LaunchConfiguration('align_roll'),
            }],
            output='screen',
        )]

    # Field camera: Gazebo images to ROS, its pose as TF, and the detector.
    camera = IfCondition(LaunchConfiguration('camera'))
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/field_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/field_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/field_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=camera,
    )
    # Same pose as the field_camera model in worlds/harvest_world.sdf.
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '1.0', '--y', '0', '--z', '0.75',
                   '--roll', '0', '--pitch', '0.48', '--yaw', '3.14159265',
                   '--frame-id', 'world', '--child-frame-id', 'field_camera_link'],
        condition=camera,
    )
    # Gazebo's camera looks along +x; ROS images use z forward, x right, y down.
    optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--roll', '-1.57079633', '--yaw', '-1.57079633',
                   '--frame-id', 'field_camera_link',
                   '--child-frame-id', 'field_camera_optical'],
        condition=camera,
    )
    detector = Node(
        package='agri_perception',
        executable='fruit_detector',
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=camera,
    )

    # Leaves in front of the stems (worlds/leaves.sdf), to test occlusion.
    leaves = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'leaves', '-file', os.path.join(share, 'worlds', 'leaves.sdf')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('leaves')),
    )

    # Wrist camera on the hand: images to ROS and the stem detector.
    wrist = IfCondition(LaunchConfiguration('wrist_camera'))
    wrist_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/wrist_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/wrist_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/wrist_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=wrist,
    )
    stem_detector = Node(
        package='agri_perception',
        executable='stem_detector',
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=wrist,
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
        DeclareLaunchArgument('control_mode', default_value='position',
                              description='position (ideal servo) or effort (PID torque control)'),
        DeclareLaunchArgument('payload', default_value='0.0',
                              description='Extra mass [kg] at the tool tip'),
        DeclareLaunchArgument('approach', default_value='lin',
                              description='lin: straight Pilz LIN approach from 10 cm, '
                              'joint: OMPL straight to the pick pose'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='Find the fruit with the field camera '
                              '(false: use the fixed positions)'),
        DeclareLaunchArgument('wrist_camera', default_value='true',
                              description='Correct the grasp point from the wrist camera '
                              '(false: assume the stem goes straight up)'),
        DeclareLaunchArgument('side_views', default_value='true',
                              description='Look again from 45° to each side when the wrist '
                              'camera does not see the stem'),
        DeclareLaunchArgument('if_unseen', default_value='skip',
                              description='Stem not seen from any view: skip the fruit, or '
                              'guess (grasp as if the stem went straight up)'),
        DeclareLaunchArgument('align_roll', default_value='true',
                              description='Turn the hand (joint6) to the lean of the stem the '
                              'wrist camera sees'),
        DeclareLaunchArgument('leaves', default_value='true',
                              description='Add leaves that hide the stems (worlds/leaves.sdf)'),
        gazebo,
        leaves,
        sim_bridge,
        wrist_bridge,
        stem_detector,
        camera_bridge,
        camera_tf,
        optical_tf,
        detector,
        OpaqueFunction(function=moveit_nodes),
        OpaqueFunction(function=demo_node),
    ])
