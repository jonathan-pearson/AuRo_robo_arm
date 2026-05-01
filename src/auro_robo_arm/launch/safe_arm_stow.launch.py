"""Run the Home -> Sleep safe stow sequence."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='vx300'),
        DeclareLaunchArgument('group_name', default_value='arm'),
        DeclareLaunchArgument('home_positions', default_value='0,0,0,0,0'),
        DeclareLaunchArgument('sleep_positions', default_value='0,-1.85,1.55,0.8,0'),
        DeclareLaunchArgument('staged_home', default_value='true'),
        DeclareLaunchArgument('staged_home_step_wait_sec', default_value='1.5'),
        DeclareLaunchArgument('home_wait_sec', default_value='3.0'),
        DeclareLaunchArgument('sleep_wait_sec', default_value='3.0'),
        DeclareLaunchArgument('torque_off_after_sleep', default_value='false'),
        Node(
            package='auro_robo_arm',
            executable='safe_arm_stow',
            name='safe_arm_stow',
            output='screen',
            parameters=[{
                'robot_name': LaunchConfiguration('robot_name'),
                'group_name': LaunchConfiguration('group_name'),
                'home_positions': LaunchConfiguration('home_positions'),
                'sleep_positions': LaunchConfiguration('sleep_positions'),
                'staged_home': ParameterValue(
                    LaunchConfiguration('staged_home'),
                    value_type=bool,
                ),
                'staged_home_step_wait_sec': ParameterValue(
                    LaunchConfiguration('staged_home_step_wait_sec'),
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
                'torque_off_after_sleep': ParameterValue(
                    LaunchConfiguration('torque_off_after_sleep'),
                    value_type=bool,
                ),
            }],
        ),
    ])
