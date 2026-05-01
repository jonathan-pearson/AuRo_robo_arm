"""Detect QR codes from a ROS 2 camera topic.

AprilTag detection is handled separately by the apriltag_ros node
(ros-jazzy-apriltag-ros). This node covers QR codes only.
"""

import json
import sys
from typing import List, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class TagDetector(Node):
    """Detect QR codes and publish results as JSON on /tag_detections."""

    def __init__(self) -> None:
        super().__init__('tag_detector')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('publish_annotated', True)

        image_topic = (
            self.get_parameter('image_topic').get_parameter_value().string_value
        )
        self._publish_annotated = (
            self.get_parameter('publish_annotated').get_parameter_value().bool_value
        )

        self._bridge = CvBridge()
        self._qr = cv2.QRCodeDetector()

        self._detection_pub = self.create_publisher(String, '/tag_detections', 10)
        self._image_pub = self.create_publisher(Image, '/tag_detections/image', 10)

        self._sub = self.create_subscription(
            Image, image_topic, self._on_image, 10
        )
        self.get_logger().info(
            f'tag_detector (QR) listening on {image_topic}'
        )

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'cv_bridge conversion failed: {exc}')
            return

        annotated = frame.copy() if self._publish_annotated else None
        detections = self._detect_qr(frame, annotated)

        if detections:
            payload = String()
            payload.data = json.dumps(detections)
            self._detection_pub.publish(payload)
            for det in detections:
                self.get_logger().info(f"QR detected: {det['data']}")

        if self._publish_annotated and annotated is not None:
            out_msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out_msg.header = msg.header
            self._image_pub.publish(out_msg)

    def _detect_qr(self, frame: np.ndarray, annotated) -> List[dict]:
        detections = []
        retval, decoded_list, points_list, _ = self._qr.detectAndDecodeMulti(frame)
        if not retval or not decoded_list:
            return detections
        for data, points in zip(decoded_list, points_list):
            if not data or points is None:
                continue
            corners = points.astype(int).tolist()
            detections.append({
                'type': 'qr',
                'data': data,
                'corners': corners,
                'center': _centroid(corners),
            })
            if annotated is not None:
                _draw_polygon(annotated, corners, (0, 255, 0), data)
        return detections


def _centroid(corners: List[List[int]]) -> List[int]:
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    return [sum(xs) // len(xs), sum(ys) // len(ys)]


def _draw_polygon(
    img: np.ndarray, corners: List[List[int]], color: Tuple, label: str
) -> None:
    pts = np.array(corners, dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(img, [pts], isClosed=True, color=color, thickness=2)
    if corners:
        cv2.putText(img, label[:40], (corners[0][0], corners[0][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


def main() -> int:
    """Run the tag_detector node."""
    rclpy.init()
    node = None
    try:
        node = TagDetector()
        rclpy.spin(node)
        return 0
    except Exception as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(str(exc), file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
