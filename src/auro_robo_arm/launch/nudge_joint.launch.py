"""Launch the one-shot single-joint nudge helper."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate a launch description for the nudge_joint node."""
    return LaunchDescription(
        [
            DeclareLaunchArgument('robot_name', default_value='vx300'),
            DeclareLaunchArgument('joint_name', default_value='waist'),
            DeclareLaunchArgument('delta_rad', default_value='0.10'),
            DeclareLaunchArgument('return_to_start', default_value='true'),
            DeclareLaunchArgument('torque_enable', default_value='true'),
            DeclareLaunchArgument('settle_sec', default_value='2.0'),
            DeclareLaunchArgument('wait_timeout_sec', default_value='10.0'),
            Node(
                package='auro_robo_arm',
                executable='nudge_joint',
                name='nudge_joint',
                output='screen',
                parameters=[
                    {
                        'robot_name': LaunchConfiguration('robot_name'),
                        'joint_name': LaunchConfiguration('joint_name'),
                        'delta_rad': ParameterValue(
                            LaunchConfiguration('delta_rad'),
                            value_type=float,
                        ),
                        'return_to_start': ParameterValue(
                            LaunchConfiguration('return_to_start'),
                            value_type=bool,
                        ),
                        'torque_enable': ParameterValue(
                            LaunchConfiguration('torque_enable'),
                            value_type=bool,
                        ),
                        'settle_sec': ParameterValue(
                            LaunchConfiguration('settle_sec'),
                            value_type=float,
                        ),
                        'wait_timeout_sec': ParameterValue(
                            LaunchConfiguration('wait_timeout_sec'),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
