"""Launch webcam, QR detector, and apriltag_ros detector together."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    video_device = LaunchConfiguration('video_device').perform(context)
    publish_annotated = (
        LaunchConfiguration('publish_annotated').perform(context).lower()
        in {'1', 'true', 'yes'}
    )

    auro_share = get_package_share_directory('auro_robo_arm')
    apriltag_cfg = str(Path(auro_share) / 'config' / 'apriltag.yaml')

    return [
        # --- Webcam ---
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='nexigo_webcam',
            parameters=[{
                'video_device': video_device,
                'framerate': 30.0,
                'pixel_format': 'yuyv2rgb',
                'image_width': 640,
                'image_height': 480,
                'frame_id': 'webcam_optical_frame',
                'camera_name': 'nexigo_webcam',
            }],
            remappings=[
                ('image_raw', '/image_raw'),
                ('camera_info', '/camera_info'),
            ],
            output='log',
        ),

        # --- AprilTag detector (ros-jazzy-apriltag-ros) ---
        # Subscribes to /image_rect and /camera_info.
        # We remap /image_rect → /image_raw (no rectification; camera uncalibrated).
        # Publishes /detections (apriltag_msgs/AprilTagDetectionArray) and /tf.
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            remappings=[
                ('/image_rect', '/image_raw'),
            ],
            parameters=[apriltag_cfg],
            output='screen',
        ),

        # --- QR code detector (our node) ---
        # Publishes /tag_detections (std_msgs/String, JSON) and
        # /tag_detections/image (annotated feed).
        Node(
            package='auro_robo_arm',
            executable='tag_detector',
            name='tag_detector',
            parameters=[{
                'image_topic': '/image_raw',
                'publish_annotated': publish_annotated,
            }],
            output='screen',
        ),
    ]


def generate_launch_description():
    """Generate launch description for webcam + QR + AprilTag detection."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video4',
            description='Video device path for the webcam.',
        ),
        DeclareLaunchArgument(
            'publish_annotated',
            default_value='true',
            description='Publish QR-annotated image on /tag_detections/image.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
