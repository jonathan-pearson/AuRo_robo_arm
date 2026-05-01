"""Move the arm directly above AprilTag ID 0.

Required startup order (three terminals):
  Terminal 1:  ros2 launch auro_robo_arm moveit.launch.py
  Terminal 2:  ros2 launch auro_robo_arm tag_detector.launch.py
  Terminal 3:  source install/setup.bash
               ros2 run auro_robo_arm arm_tag_follower

MoveItPy requires the arm driver and move_group to be running
(moveit.launch.py handles both).  If you run this node before
moveit.launch.py you will see: "Could not find parameter robot_description".

Camera-to-base parameters must match your physical camera mount.
Measure the camera lens position relative to vx300/base_link and pass
the cam_* parameters, e.g.:
  ros2 run auro_robo_arm arm_tag_follower --ros-args \\
    -p cam_x:=0.10 -p cam_z:=0.25 -p cam_pitch:=0.3
"""

import math
import queue
import sys
import threading
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
from rclpy.node import Node


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _euler_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX Euler angles → 3×3 rotation matrix (R = Rz · Ry · Rx)."""
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    Rz = np.array([[cy, -sy, 0.], [sy,  cy, 0.], [0., 0., 1.]])
    Ry = np.array([[cp,  0., sp], [0.,  1., 0.], [-sp, 0., cp]])
    Rx = np.array([[1.,  0., 0.], [0.,  cr, -sr], [0., sr,  cr]])
    return Rz @ Ry @ Rx


# AprilTag 3D corners in tag frame (x right, y up, z out of tag).
# Matches apriltag3 corner order: BL → BR → TR → TL.
def _tag_obj_pts(half: float) -> np.ndarray:
    return np.array([
        [-half, -half, 0.],
        [ half, -half, 0.],
        [ half,  half, 0.],
        [-half,  half, 0.],
    ], dtype=np.float64)


# ---------------------------------------------------------------------------
# Detection listener (spun in a background thread after MoveItPy is ready)
# ---------------------------------------------------------------------------

class _Listener(Node):
    """Subscribe to /detections and queue hover targets for the main thread."""

    def __init__(
        self,
        tag_id: int,
        tag_size: float,
        hover_height: float,
        max_reach: float,
        min_reach: float,
        K: np.ndarray,
        T_base_cam: np.ndarray,
    ) -> None:
        super().__init__('arm_tag_follower')

        self._tag_id    = tag_id
        self._tag_size  = tag_size
        self._hover_h   = hover_height
        self._max_reach = max_reach
        self._min_reach = min_reach
        self._K         = K
        self._T_base_cam = T_base_cam

        self.target_queue: 'queue.Queue[Tuple[float,float,float]]' = (
            queue.Queue(maxsize=1)
        )
        self.is_moving = False

        self.create_subscription(
            AprilTagDetectionArray, '/detections', self._cb, 10
        )
        self.get_logger().info(
            f'arm_tag_follower ready — watching for AprilTag ID {tag_id}'
        )

    # ------------------------------------------------------------------
    def _cb(self, msg: AprilTagDetectionArray) -> None:
        if self.is_moving:
            return

        for det in msg.detections:
            if det.id != self._tag_id:
                continue
            target = self._hover_in_base(det)
            if target is None:
                self.get_logger().warning('solvePnP failed on tag corners.')
                return
            x, y, z = target
            reach = math.sqrt(x*x + y*y + z*z)
            if reach > self._max_reach:
                self.get_logger().warning(
                    f'Hover point {reach:.3f} m away — outside max reach '
                    f'({self._max_reach} m). Move the robot closer.'
                )
                return
            if reach < self._min_reach:
                self.get_logger().warning(
                    f'Hover point {reach:.3f} m away — too close to base '
                    f'({self._min_reach} m).'
                )
                return
            try:
                self.target_queue.put_nowait((x, y, z))
            except queue.Full:
                pass
            break

    def _hover_in_base(
        self, det
    ) -> Optional[Tuple[float, float, float]]:
        img_pts = np.array(
            [[c.x, c.y] for c in det.corners], dtype=np.float64
        )
        obj_pts = _tag_obj_pts(self._tag_size / 2.0)
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts, img_pts, self._K,
            np.zeros((4, 1), dtype=np.float64),
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            return None
        R, _ = cv2.Rodrigues(rvec)
        T_cam_tag = np.eye(4)
        T_cam_tag[:3, :3] = R
        T_cam_tag[:3,  3] = tvec.flatten()
        pos = (self._T_base_cam @ T_cam_tag)[:3, 3]
        hover = pos + np.array([0., 0., self._hover_h])
        return float(hover[0]), float(hover[1]), float(hover[2])


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> int:
    rclpy.init()

    # ------------------------------------------------------------------ #
    # 1. MoveItPy — must come BEFORE creating any listener nodes so that  #
    #    if init fails (arm not running), the rcl context stays clean.     #
    # ------------------------------------------------------------------ #
    moveit = MoveItPy(node_name='arm_tag_follower_moveit')
    arm    = moveit.get_planning_component('interbotix_arm')

    # ------------------------------------------------------------------ #
    # 2. Parameters — declared on the listener node after MoveItPy is up. #
    # ------------------------------------------------------------------ #
    # Read via a tiny param node first, then destroy it.
    pn = rclpy.create_node('_arm_tag_follower_params')

    def dp(name, default):
        pn.declare_parameter(name, default)

    dp('robot_name',    'vx300')
    dp('tag_id',        0)
    dp('tag_size',      0.10)
    dp('hover_height',  0.15)
    dp('move_time',     3.0)
    dp('max_reach',     0.68)
    dp('min_reach',     0.10)
    # Camera intrinsics — rough defaults for 640×480; run cameracalibrator for accuracy.
    dp('cam_fx', 600.0)
    dp('cam_fy', 600.0)
    dp('cam_cx', 320.0)
    dp('cam_cy', 240.0)
    # Camera pose in vx300/base_link frame. Measure the lens origin.
    # Base frame: +x forward, +y left, +z up.
    # Camera optical frame: +x image-right, +y image-down, +z forward.
    # The default orientation assumes a forward-looking webcam:
    #   optical +z -> base +x, optical +x -> base -y, optical +y -> base -z.
    # cam_roll/pitch/yaw are small mount corrections applied in base-frame ZYX.
    dp('cam_x',     0.0)
    dp('cam_y',     0.0)
    dp('cam_z',     0.3)
    dp('cam_roll',  0.0)
    dp('cam_pitch', 0.0)
    dp('cam_yaw',   0.0)

    def gp(name, kind):
        return getattr(pn.get_parameter(name).get_parameter_value(),
                       f'{kind}_value')

    robot_name   = gp('robot_name',   'string')
    tag_id       = gp('tag_id',       'integer')
    tag_size     = gp('tag_size',     'double')
    hover_height = gp('hover_height', 'double')
    move_time    = gp('move_time',    'double')
    max_reach    = gp('max_reach',    'double')
    min_reach    = gp('min_reach',    'double')

    K = np.array([
        [gp('cam_fx', 'double'), 0.,                     gp('cam_cx', 'double')],
        [0.,                     gp('cam_fy', 'double'),  gp('cam_cy', 'double')],
        [0.,                     0.,                     1.],
    ], dtype=np.float64)

    R_base_cam_forward = np.array([
        [0.,  0.,  1.],
        [-1., 0.,  0.],
        [0., -1.,  0.],
    ])
    R_mount_adjust = _euler_to_rot(
        gp('cam_roll', 'double'),
        gp('cam_pitch', 'double'),
        gp('cam_yaw', 'double'),
    )
    T_base_cam = np.eye(4)
    T_base_cam[:3, :3] = R_mount_adjust @ R_base_cam_forward
    T_base_cam[:3, 3] = [gp('cam_x', 'double'),
                         gp('cam_y', 'double'),
                         gp('cam_z', 'double')]
    pn.destroy_node()

    base_frame = f'{robot_name}/base_link'
    ee_link    = f'{robot_name}/ee_gripper_link'

    # Quaternion: gripper pointing straight down.
    # At home pose, the ee local-X points along base +X.
    # Ry(+90°) rotates local-X to base -Z (downward).
    # q = (0, sin(45°), 0, cos(45°))
    _S = math.sqrt(2.0) / 2.0
    QX, QY, QZ, QW = 0.0, _S, 0.0, _S

    # ------------------------------------------------------------------ #
    # 3. Listener — created after MoveItPy succeeds.                      #
    # ------------------------------------------------------------------ #
    listener = _Listener(
        tag_id=tag_id,
        tag_size=tag_size,
        hover_height=hover_height,
        max_reach=max_reach,
        min_reach=min_reach,
        K=K,
        T_base_cam=T_base_cam,
    )
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(listener,), daemon=True
    )
    spin_thread.start()

    # ------------------------------------------------------------------ #
    # 4. Main loop                                                         #
    # ------------------------------------------------------------------ #
    exit_code = 0
    try:
        while rclpy.ok():
            try:
                x, y, z = listener.target_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            listener.get_logger().info(
                f'Tag {tag_id} found — planning hover at '
                f'x={x:.3f}  y={y:.3f}  z={z:.3f} m'
            )
            listener.is_moving = True
            try:
                pose = PoseStamped()
                pose.header.frame_id = base_frame
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = z
                pose.pose.orientation.x = QX
                pose.pose.orientation.y = QY
                pose.pose.orientation.z = QZ
                pose.pose.orientation.w = QW

                arm.set_start_state_to_current_state()
                arm.set_goal_state(
                    pose_stamped_msg=pose,
                    pose_link=ee_link,
                )
                plan_result = arm.plan()

                if plan_result:
                    listener.get_logger().info('Executing — arm is moving.')
                    moveit.execute(
                        plan_result.trajectory,
                        controllers=[],
                        blocking=True,
                    )
                    listener.get_logger().info('Arm is above the tag.')
                else:
                    listener.get_logger().warning(
                        'No plan found. Try: adjusting hover_height, '
                        'moving the robot, or checking the cam_* parameters.'
                    )
            finally:
                listener.is_moving = False

    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'Error: {exc}', file=sys.stderr)
        exit_code = 1
    finally:
        listener.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)

    return exit_code


if __name__ == '__main__':
    sys.exit(main())
