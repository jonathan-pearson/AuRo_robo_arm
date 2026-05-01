"""Launch arm_tag_follower with the MoveIt parameters it needs."""

import os

from ament_index_python.packages import get_package_share_directory
from interbotix_xs_modules.xs_common import get_interbotix_xsarm_models
from interbotix_xs_modules.xs_launch import (
    construct_interbotix_xsarm_semantic_robot_description_command,
    declare_interbotix_xsarm_robot_description_launch_arguments,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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

    # MoveItPy uses MoveItCpp internally. Its pipeline loader expects
    # planning_pipelines.pipeline_names, while move_group's launch file uses a
    # flatter planning_pipelines list. Keep this wrapper in MoveItCpp format.
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
        Node(
            package='auro_robo_arm',
            executable='arm_tag_follower',
            name='arm_tag_follower',
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
                    'tag_id': ParameterValue(
                        LaunchConfiguration('tag_id'),
                        value_type=int,
                    ),
                    'tag_size': ParameterValue(
                        LaunchConfiguration('tag_size'),
                        value_type=float,
                    ),
                    'hover_height': ParameterValue(
                        LaunchConfiguration('hover_height'),
                        value_type=float,
                    ),
                    'cam_x': ParameterValue(LaunchConfiguration('cam_x'), value_type=float),
                    'cam_y': ParameterValue(LaunchConfiguration('cam_y'), value_type=float),
                    'cam_z': ParameterValue(LaunchConfiguration('cam_z'), value_type=float),
                    'cam_roll': ParameterValue(
                        LaunchConfiguration('cam_roll'),
                        value_type=float,
                    ),
                    'cam_pitch': ParameterValue(
                        LaunchConfiguration('cam_pitch'),
                        value_type=float,
                    ),
                    'cam_yaw': ParameterValue(
                        LaunchConfiguration('cam_yaw'),
                        value_type=float,
                    ),
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
        DeclareLaunchArgument(
            'robot_model',
            default_value='vx300',
            choices=get_interbotix_xsarm_models(),
            description='Interbotix arm model.',
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value=LaunchConfiguration('robot_model'),
            description='ROS namespace and TF prefix for the arm.',
        ),
        DeclareLaunchArgument(
            'external_srdf_loc',
            default_value=TextSubstitution(text=''),
            description='Optional custom semantic description include.',
        ),
        DeclareLaunchArgument('tag_id', default_value='0'),
        DeclareLaunchArgument('tag_size', default_value='0.10'),
        DeclareLaunchArgument('hover_height', default_value='0.15'),
        DeclareLaunchArgument('cam_x', default_value='0.0'),
        DeclareLaunchArgument('cam_y', default_value='0.0'),
        DeclareLaunchArgument('cam_z', default_value='0.3'),
        DeclareLaunchArgument('cam_roll', default_value='0.0'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0'),
        DeclareLaunchArgument('cam_yaw', default_value='0.0'),
    ]
    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments(
            show_gripper_bar='true',
            show_gripper_fingers='true',
            hardware_type='actual',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=_launch_setup)])
