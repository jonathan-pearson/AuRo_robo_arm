"""Launch the Interbotix X-Series arm driver with a USB port override."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool_text(value: str) -> str:
    return 'true' if value.lower() in {'1', 'true', 'yes', 'on'} else 'false'


def _safe_filename(value: str) -> str:
    return ''.join(
        char if char.isalnum() or char in {'-', '_'} else '_'
        for char in value
    )


def _write_mode_config(robot_name: str, port: str) -> str:
    mode_config = (
        Path('/tmp') / f'auro_robo_arm_{_safe_filename(robot_name)}_usb_modes.yaml'
    )
    mode_config.write_text(
        '\n'.join(
            [
                f'port: {port}',
                '',
                'groups:',
                '  arm:',
                '    operating_mode: position',
                '    profile_type: time',
                '    profile_velocity: 2000',
                '    profile_acceleration: 1000',
                '    torque_enable: true',
                '',
                'singles:',
                '  gripper:',
                '    operating_mode: pwm',
                '    torque_enable: true',
                '',
            ]
        ),
        encoding='utf-8',
    )
    return str(mode_config)


def _launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration('robot_model').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context) or robot_model
    port = LaunchConfiguration('port').perform(context)
    motor_configs = LaunchConfiguration('motor_configs').perform(context)
    rvizconfig = LaunchConfiguration('rvizconfig').perform(context)
    use_rviz = _bool_text(LaunchConfiguration('use_rviz').perform(context))
    use_sim = _bool_text(LaunchConfiguration('use_sim').perform(context))
    use_sim_time = _bool_text(
        LaunchConfiguration('use_sim_time').perform(context)
    )
    load_configs = _bool_text(LaunchConfiguration('load_configs').perform(context))

    auro_share_dir = get_package_share_directory('auro_robo_arm')
    xsarm_control_dir = get_package_share_directory('interbotix_xsarm_control')
    xsarm_launch = Path(xsarm_control_dir) / 'launch' / 'xsarm_control.launch.py'
    if not motor_configs:
        local_motor_configs = Path(auro_share_dir) / 'config' / f'{robot_model}.yaml'
        if local_motor_configs.is_file():
            motor_configs = str(local_motor_configs)
        else:
            motor_configs = str(
                Path(xsarm_control_dir) / 'config' / f'{robot_model}.yaml'
            )
    if not rvizconfig:
        rvizconfig = str(Path(auro_share_dir) / 'rviz' / 'vx300.rviz')
    mode_configs = _write_mode_config(robot_name, port)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(xsarm_launch)),
            launch_arguments={
                'robot_model': robot_model,
                'robot_name': robot_name,
                'motor_configs': motor_configs,
                'mode_configs': mode_configs,
                'load_configs': load_configs,
                'use_rviz': 'false',
                'use_sim': use_sim,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=robot_name,
            arguments=['-d', rvizconfig],
            parameters=[{'use_sim_time': use_sim_time == 'true'}],
            output={'both': 'log'},
        ),
    ]


def generate_launch_description():
    """Generate the USB-backed Interbotix driver launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_model',
                default_value='vx300',
                description=(
                    'Interbotix arm model. This package is configured for '
                    'the 5-DOF vx300.'
                ),
            ),
            DeclareLaunchArgument(
                'robot_name',
                default_value='',
                description='ROS namespace for the arm. Defaults to robot_model.',
            ),
            DeclareLaunchArgument(
                'port',
                default_value='/dev/ttyDXL',
                description='USB serial device for the U2D2/DYNAMIXEL bus.',
            ),
            DeclareLaunchArgument(
                'motor_configs',
                default_value='',
                description=(
                    'Optional motor config YAML. Leave empty to use this '
                    "package's config when available, otherwise the "
                    'Interbotix config for robot_model.'
                ),
            ),
            DeclareLaunchArgument(
                'rvizconfig',
                default_value='',
                description=(
                    'Optional RViz config. Leave empty to use the AuRo VX300 '
                    'visualization config.'
                ),
            ),
            DeclareLaunchArgument(
                'load_configs',
                default_value='false',
                description=(
                    'Write motor EEPROM configs on startup. Existing '
                    'Interbotix arms should normally leave this false.'
                ),
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='false',
                description='Start RViz with the AuRo visualization config.',
            ),
            DeclareLaunchArgument(
                'use_sim',
                default_value='false',
                description=(
                    'Use the Interbotix simulated xs_sdk instead of physical '
                    'USB hardware.'
                ),
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulation time.',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
