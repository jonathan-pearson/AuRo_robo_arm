"""Launch the AuRo VX300 with MoveIt and MoveIt RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _safe_filename(value: str) -> str:
    return ''.join(
        char if char.isalnum() or char in {'-', '_'} else '_'
        for char in value
    )


def _write_moveit_mode_config(robot_name: str, port: str) -> str:
    mode_config = (
        Path('/tmp')
        / f'auro_robo_arm_{_safe_filename(robot_name)}_moveit_modes.yaml'
    )
    mode_config.write_text(
        '\n'.join(
            [
                f'port: {port}',
                '',
                'groups:',
                '  arm:',
                '    operating_mode: position',
                '    profile_type: velocity',
                '    profile_velocity: 131',
                '    profile_acceleration: 25',
                '    torque_enable: true',
                '',
                'singles:',
                '  gripper:',
                '    operating_mode: linear_position',
                '    profile_type: velocity',
                '    profile_velocity: 131',
                '    profile_acceleration: 15',
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
    load_configs = LaunchConfiguration('load_configs').perform(context)
    use_moveit_rviz = LaunchConfiguration('use_moveit_rviz').perform(context)
    hardware_type = LaunchConfiguration('hardware_type').perform(context)

    auro_share_dir = get_package_share_directory('auro_robo_arm')
    xsarm_moveit_dir = get_package_share_directory('interbotix_xsarm_moveit')
    if not motor_configs:
        local_motor_configs = Path(auro_share_dir) / 'config' / f'{robot_model}.yaml'
        if local_motor_configs.is_file():
            motor_configs = str(local_motor_configs)

    mode_configs = _write_moveit_mode_config(robot_name, port)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(xsarm_moveit_dir) / 'launch' / 'xsarm_moveit.launch.py')
            ),
            launch_arguments={
                'robot_model': robot_model,
                'robot_name': robot_name,
                'motor_configs': motor_configs,
                'mode_configs': mode_configs,
                'load_configs': load_configs,
                'use_moveit_rviz': use_moveit_rviz,
                'hardware_type': hardware_type,
                'use_world_frame': 'true',
                'rviz_frame': 'world',
            }.items(),
        ),
    ]


def generate_launch_description():
    """Generate the MoveIt-backed physical-arm launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_model',
                default_value='vx300',
                description='Interbotix arm model to use with MoveIt.',
            ),
            DeclareLaunchArgument(
                'robot_name',
                default_value='vx300',
                description='ROS namespace and TF prefix for the arm.',
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
                    "package's config when available."
                ),
            ),
            DeclareLaunchArgument(
                'load_configs',
                default_value='false',
                description=(
                    'Write motor EEPROM configs on startup. Use true only '
                    'when deliberately initializing or changing servo config.'
                ),
            ),
            DeclareLaunchArgument(
                'use_moveit_rviz',
                default_value='true',
                description="Start RViz with MoveIt's MotionPlanning plugin.",
            ),
            DeclareLaunchArgument(
                'hardware_type',
                default_value='actual',
                description="Use 'actual' hardware or 'fake' MoveIt hardware.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
