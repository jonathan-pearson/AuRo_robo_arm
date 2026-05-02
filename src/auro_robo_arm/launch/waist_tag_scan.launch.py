"""Bring up camera/tag detection and run a waist-only AprilTag scan."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from interbotix_xs_modules.xs_launch import (
    construct_interbotix_xsarm_semantic_robot_description_command,
    declare_interbotix_xsarm_robot_description_launch_arguments,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def _load_yaml(package_name: str, file_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r', encoding='utf-8') as file:
        return yaml.safe_load(file)


def _position_only_kinematics():
    kinematics = _load_yaml('interbotix_xsarm_moveit', 'config/kinematics.yaml')
    params = kinematics['/**']['ros__parameters']
    params['robot_description_kinematics']['interbotix_arm']['position_only_ik'] = True
    return params


def _launch_setup(context, *args, **kwargs):
    share_dir = Path(get_package_share_directory('auro_robo_arm'))

    robot_model = LaunchConfiguration('robot_model').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)

    robot_description = {
        'robot_description': LaunchConfiguration('robot_description'),
    }
    config_path = PathJoinSubstitution([
        FindPackageShare('interbotix_xsarm_moveit'),
        'config',
    ])
    robot_description_semantic = {
        'robot_description_semantic':
            construct_interbotix_xsarm_semantic_robot_description_command(
                robot_model=robot_model,
                config_path=config_path,
            ),
    }
    ompl_planning_pipeline_config = {
        'planning_pipelines': {
            'pipeline_names': ['ompl'],
        },
        'ompl': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision',
                'default_planning_request_adapters/CheckForStackedConstraints',
            ],
            'response_adapters': [
                'default_planning_response_adapters/AddTimeOptimalParameterization',
                'default_planning_response_adapters/AddRuckigTrajectorySmoothing',
                'default_planning_response_adapters/ValidateSolution',
                'default_planning_response_adapters/DisplayMotionPath',
            ],
            'start_state_max_bounds_error': 0.1,
        },
    }
    ompl_planning_pipeline_config['ompl'].update(
        _load_yaml('interbotix_xsarm_moveit', 'config/ompl_planning.yaml')
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': _load_yaml(
            'interbotix_xsarm_moveit',
            f'config/controllers/{robot_model}_controllers.yaml',
        ),
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

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
            parameters=[
                {
                    'planning_scene_monitor_options': {
                        'robot_description': 'robot_description',
                        'joint_state_topic': f'/{robot_name}/joint_states',
                    },
                    'plan_request_params': {
                        'planning_pipeline': 'ompl',
                        'planner_id': 'RRTConnect',
                        'planning_time': 5.0,
                        'planning_attempts': 10,
                        'max_velocity_scaling_factor': 0.2,
                        'max_acceleration_scaling_factor': 0.2,
                    },
                    'robot_name': robot_name,
                    'joint_name': 'waist',
                    'moveit_group_name': 'interbotix_arm',
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
                    'use_moveit_for_stow': ParameterValue(
                        LaunchConfiguration('use_moveit_for_stow'),
                        value_type=bool,
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
                },
                robot_description,
                robot_description_semantic,
                _position_only_kinematics(),
                ompl_planning_pipeline_config,
                moveit_controllers,
                {
                    'robot_description_planning': _load_yaml(
                        'interbotix_xsarm_moveit',
                        f'config/joint_limits/{robot_model}_joint_limits.yaml',
                    ),
                    'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                    'trajectory_execution.allowed_goal_duration_margin': 0.5,
                    'trajectory_execution.allowed_start_tolerance': 0.01,
                    'publish_planning_scene': True,
                    'publish_geometry_updates': True,
                    'publish_state_updates': True,
                    'publish_transforms_updates': True,
                    'sensors': [''],
                },
            ],
            remappings=[
                ('joint_states', f'/{robot_name}/joint_states'),
                ('/arm_controller/follow_joint_trajectory',
                 f'/{robot_name}/arm_controller/follow_joint_trajectory'),
                ('/gripper_controller/follow_joint_trajectory',
                 f'/{robot_name}/gripper_controller/follow_joint_trajectory'),
            ],
        ),
    ]


def generate_launch_description():
    declared_arguments = [
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
        DeclareLaunchArgument('use_moveit_for_stow', default_value='true'),
        DeclareLaunchArgument('halt_hold_sec', default_value='0.5'),
        DeclareLaunchArgument('home_wait_sec', default_value='3.0'),
        DeclareLaunchArgument('sleep_wait_sec', default_value='3.0'),
        DeclareLaunchArgument('home_positions', default_value='0,0,0,0,0'),
        DeclareLaunchArgument('sleep_positions', default_value='0,-1.85,1.55,0.8,0'),
        DeclareLaunchArgument('log_path', default_value='/tmp/auro_tag_scan.jsonl'),
        DeclareLaunchArgument(
            'external_srdf_loc',
            default_value=TextSubstitution(text=''),
            description='Optional custom semantic description include.',
        ),
    ]
    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments(
            show_gripper_bar='true',
            show_gripper_fingers='true',
            hardware_type='actual',
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=_launch_setup)])
