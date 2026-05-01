"""Launch a view-only RViz model of the Interbotix arm."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate a launch description for RViz-only arm visualization."""
    auro_share_dir = get_package_share_directory('auro_robo_arm')
    xsarm_description_dir = get_package_share_directory('interbotix_xsarm_descriptions')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_model',
                default_value='vx300',
                description='Interbotix arm model to visualize.',
            ),
            DeclareLaunchArgument(
                'robot_name',
                default_value='vx300',
                description='ROS namespace and TF prefix for the arm.',
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='true',
                description='Start RViz with the AuRo visualization config.',
            ),
            DeclareLaunchArgument(
                'use_joint_pub',
                default_value='false',
                description='Start joint_state_publisher without a GUI.',
            ),
            DeclareLaunchArgument(
                'use_joint_pub_gui',
                default_value='true',
                description='Start joint_state_publisher_gui for manual poses.',
            ),
            DeclareLaunchArgument(
                'rvizconfig',
                default_value=str(Path(auro_share_dir) / 'rviz' / 'vx300.rviz'),
                description='RViz config file to load.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulated time.',
            ),
            DeclareLaunchArgument(
                'use_world_frame',
                default_value='true',
                description='Publish a world frame at the robot base.',
            ),
            DeclareLaunchArgument(
                'use_gripper',
                default_value='true',
                description='Include the default gripper in the model.',
            ),
            DeclareLaunchArgument(
                'show_ar_tag',
                default_value='false',
                description='Include the Interbotix AR tag mount.',
            ),
            DeclareLaunchArgument(
                'show_gripper_bar',
                default_value='true',
                description='Include the gripper bar link.',
            ),
            DeclareLaunchArgument(
                'show_gripper_fingers',
                default_value='true',
                description='Include the gripper finger links.',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        Path(xsarm_description_dir)
                        / 'launch'
                        / 'xsarm_description.launch.py'
                    )
                ),
                launch_arguments={
                    'robot_model': LaunchConfiguration('robot_model'),
                    'robot_name': LaunchConfiguration('robot_name'),
                    'use_rviz': LaunchConfiguration('use_rviz'),
                    'use_joint_pub': LaunchConfiguration('use_joint_pub'),
                    'use_joint_pub_gui': LaunchConfiguration('use_joint_pub_gui'),
                    'rvizconfig': LaunchConfiguration('rvizconfig'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'use_world_frame': LaunchConfiguration('use_world_frame'),
                    'use_gripper': LaunchConfiguration('use_gripper'),
                    'show_ar_tag': LaunchConfiguration('show_ar_tag'),
                    'show_gripper_bar': LaunchConfiguration('show_gripper_bar'),
                    'show_gripper_fingers': LaunchConfiguration(
                        'show_gripper_fingers'
                    ),
                }.items(),
            ),
        ]
    )
