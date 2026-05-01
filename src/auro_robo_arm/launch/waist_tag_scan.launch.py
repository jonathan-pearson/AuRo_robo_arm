"""Bring up camera/tag detection and run a waist-only AprilTag scan."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context, *args, **kwargs):
    share_dir = Path(get_package_share_directory('auro_robo_arm'))

    robot_name = LaunchConfiguration('robot_name').perform(context)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share_dir / 'launch' / 'usb_bringup.launch.py')),
            launch_arguments={
                'robot_model': LaunchConfiguration('robot_model'),
                'robot_name': LaunchConfiguration('robot_name'),
                'port': LaunchConfiguration('port'),
                'load_configs': LaunchConfiguration('load_configs'),
                'use_rviz': LaunchConfiguration('use_rviz'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('start_robot')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share_dir / 'launch' / 'tag_detector.launch.py')),
            launch_arguments={
                'video_device': LaunchConfiguration('video_device'),
                'publish_annotated': LaunchConfiguration('publish_annotated'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('start_camera')),
        ),
        Node(
            package='auro_robo_arm',
            executable='waist_tag_scanner',
            name='waist_tag_scanner',
            output='screen',
            parameters=[{
                'robot_name': robot_name,
                'joint_name': 'waist',
                'tag_id': ParameterValue(LaunchConfiguration('tag_id'), value_type=int),
                'scan_positions': LaunchConfiguration('scan_positions'),
                'preset_arm_positions': LaunchConfiguration('preset_arm_positions'),
                'arm_joint_names': LaunchConfiguration('arm_joint_names'),
                'preset_settle_sec': ParameterValue(
                    LaunchConfiguration('preset_settle_sec'),
                    value_type=float,
                ),
                'dwell_sec': ParameterValue(
                    LaunchConfiguration('dwell_sec'),
                    value_type=float,
                ),
                'settle_sec': ParameterValue(
                    LaunchConfiguration('settle_sec'),
                    value_type=float,
                ),
                'max_cycles': ParameterValue(
                    LaunchConfiguration('max_cycles'),
                    value_type=int,
                ),
                'max_abs_position_rad': ParameterValue(
                    LaunchConfiguration('max_abs_position_rad'),
                    value_type=float,
                ),
                'return_to_start_on_timeout': ParameterValue(
                    LaunchConfiguration('return_to_start_on_timeout'),
                    value_type=bool,
                ),
                'return_to_start_on_found': ParameterValue(
                    LaunchConfiguration('return_to_start_on_found'),
                    value_type=bool,
                ),
                'stop_on_first_detection': ParameterValue(
                    LaunchConfiguration('stop_on_first_detection'),
                    value_type=bool,
                ),
                'track_on_detection': ParameterValue(
                    LaunchConfiguration('track_on_detection'),
                    value_type=bool,
                ),
                'track_deadband_px': ParameterValue(
                    LaunchConfiguration('track_deadband_px'),
                    value_type=float,
                ),
                'waist_px_gain': ParameterValue(
                    LaunchConfiguration('waist_px_gain'),
                    value_type=float,
                ),
                'wrist_px_gain': ParameterValue(
                    LaunchConfiguration('wrist_px_gain'),
                    value_type=float,
                ),
                'max_track_step_rad': ParameterValue(
                    LaunchConfiguration('max_track_step_rad'),
                    value_type=float,
                ),
                'min_log_interval_sec': ParameterValue(
                    LaunchConfiguration('min_log_interval_sec'),
                    value_type=float,
                ),
                'log_path': LaunchConfiguration('log_path'),
                'detections_topic': LaunchConfiguration('detections_topic'),
                'command_topic': LaunchConfiguration('command_topic'),
                'start_active': ParameterValue(
                    LaunchConfiguration('start_active'),
                    value_type=bool,
                ),
                'stow_on_turn_off': ParameterValue(
                    LaunchConfiguration('stow_on_turn_off'),
                    value_type=bool,
                ),
                'staged_home': ParameterValue(
                    LaunchConfiguration('staged_home'),
                    value_type=bool,
                ),
                'staged_home_step_wait_sec': ParameterValue(
                    LaunchConfiguration('staged_home_step_wait_sec'),
                    value_type=float,
                ),
                'halt_hold_sec': ParameterValue(
                    LaunchConfiguration('halt_hold_sec'),
                    value_type=float,
                ),
                'home_wait_sec': ParameterValue(
                    LaunchConfiguration('home_wait_sec'),
                    value_type=float,
                ),
                'sleep_wait_sec': ParameterValue(
                    LaunchConfiguration('sleep_wait_sec'),
                    value_type=float,
                ),
                'home_positions': LaunchConfiguration('home_positions'),
                'sleep_positions': LaunchConfiguration('sleep_positions'),
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_robot', default_value='true'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('robot_model', default_value='vx300'),
        DeclareLaunchArgument('robot_name', default_value='vx300'),
        DeclareLaunchArgument('port', default_value='/dev/ttyDXL'),
        DeclareLaunchArgument('load_configs', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('video_device', default_value='/dev/video4'),
        DeclareLaunchArgument('publish_annotated', default_value='true'),
        DeclareLaunchArgument('tag_id', default_value='0'),
        DeclareLaunchArgument(
            'preset_arm_positions',
            default_value='0,0,0,1.047,0',
            description=(
                'Optional comma-separated full arm group positions in radians. '
                'For vx300 order defaults to waist,shoulder,elbow,wrist_angle,wrist_rotate.'
            ),
        ),
        DeclareLaunchArgument(
            'arm_joint_names',
            default_value='waist,shoulder,elbow,wrist_angle,wrist_rotate',
        ),
        DeclareLaunchArgument('preset_settle_sec', default_value='2.0'),
        DeclareLaunchArgument(
            'scan_positions',
            default_value='-0.45,-0.25,0.0,0.25,0.45',
            description='Comma-separated absolute waist angles in radians.',
        ),
        DeclareLaunchArgument('dwell_sec', default_value='1.0'),
        DeclareLaunchArgument('settle_sec', default_value='0.4'),
        DeclareLaunchArgument(
            'max_cycles',
            default_value='0',
            description='Number of scan passes. Use 0 to scan until interrupted.',
        ),
        DeclareLaunchArgument('max_abs_position_rad', default_value='0.75'),
        DeclareLaunchArgument('return_to_start_on_timeout', default_value='true'),
        DeclareLaunchArgument('return_to_start_on_found', default_value='false'),
        DeclareLaunchArgument('stop_on_first_detection', default_value='false'),
        DeclareLaunchArgument('track_on_detection', default_value='true'),
        DeclareLaunchArgument('track_deadband_px', default_value='30.0'),
        DeclareLaunchArgument('waist_px_gain', default_value='-0.0008'),
        DeclareLaunchArgument('wrist_px_gain', default_value='0.0008'),
        DeclareLaunchArgument('max_track_step_rad', default_value='0.06'),
        DeclareLaunchArgument('min_log_interval_sec', default_value='1.0'),
        DeclareLaunchArgument('detections_topic', default_value='/detections'),
        DeclareLaunchArgument('command_topic', default_value='/auro/command'),
        DeclareLaunchArgument('start_active', default_value='false'),
        DeclareLaunchArgument('stow_on_turn_off', default_value='true'),
        DeclareLaunchArgument('staged_home', default_value='true'),
        DeclareLaunchArgument('staged_home_step_wait_sec', default_value='1.5'),
        DeclareLaunchArgument('halt_hold_sec', default_value='0.5'),
        DeclareLaunchArgument('home_wait_sec', default_value='3.0'),
        DeclareLaunchArgument('sleep_wait_sec', default_value='3.0'),
        DeclareLaunchArgument('home_positions', default_value='0,0,0,0,0'),
        DeclareLaunchArgument('sleep_positions', default_value='0,-1.85,1.55,0.8,0'),
        DeclareLaunchArgument('log_path', default_value='/tmp/auro_tag_scan.jsonl'),
        OpaqueFunction(function=_launch_setup),
    ])
