"""Sweep the waist through fixed poses until an AprilTag is detected."""

import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Optional

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from interbotix_xs_msgs.msg import JointGroupCommand
from interbotix_xs_msgs.msg import JointSingleCommand
from interbotix_xs_msgs.srv import RobotInfo, TorqueEnable
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class WaistTagScanner(Node):
    """Run a conservative waist-only search pattern and log matching tags."""

    def __init__(self) -> None:
        super().__init__('waist_tag_scanner')

        self.declare_parameter('robot_name', 'vx300')
        self.declare_parameter('joint_name', 'waist')
        self.declare_parameter('tag_id', 0)
        self.declare_parameter('scan_positions', '-0.45,-0.25,0.0,0.25,0.45')
        self.declare_parameter('preset_arm_positions', '')
        self.declare_parameter(
            'arm_joint_names',
            'waist,shoulder,elbow,wrist_angle,wrist_rotate',
        )
        self.declare_parameter('arm_group_name', 'arm')
        self.declare_parameter('preset_settle_sec', 2.0)
        self.declare_parameter('dwell_sec', 1.0)
        self.declare_parameter('settle_sec', 0.4)
        self.declare_parameter('max_cycles', 3)
        self.declare_parameter('command_repeats', 5)
        self.declare_parameter('wait_timeout_sec', 10.0)
        self.declare_parameter('max_abs_position_rad', 0.75)
        self.declare_parameter('torque_enable', True)
        self.declare_parameter('return_to_start_on_timeout', True)
        self.declare_parameter('return_to_start_on_found', False)
        self.declare_parameter('stop_on_first_detection', False)
        self.declare_parameter('min_log_interval_sec', 1.0)
        self.declare_parameter('track_on_detection', True)
        self.declare_parameter('image_width', 640.0)
        self.declare_parameter('image_height', 480.0)
        self.declare_parameter('track_interval_sec', 0.25)
        self.declare_parameter('track_lost_timeout_sec', 2.0)
        self.declare_parameter('track_deadband_px', 30.0)
        self.declare_parameter('waist_px_gain', -0.0008)
        self.declare_parameter('wrist_px_gain', 0.0008)
        self.declare_parameter('max_track_step_rad', 0.06)
        self.declare_parameter('track_min_waist_rad', -0.75)
        self.declare_parameter('track_max_waist_rad', 0.75)
        self.declare_parameter('track_min_wrist_angle_rad', 0.15)
        self.declare_parameter('track_max_wrist_angle_rad', 1.75)
        self.declare_parameter('log_path', '/tmp/auro_tag_scan.jsonl')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('command_topic', '/auro/command')
        self.declare_parameter('start_active', True)
        self.declare_parameter('stow_on_turn_off', True)
        self.declare_parameter('halt_hold_sec', 0.5)
        self.declare_parameter('home_positions', '0,0,0,0,0')
        self.declare_parameter('sleep_positions', '0,-1.85,1.55,0.8,0')
        self.declare_parameter('staged_home', True)
        self.declare_parameter('staged_home_step_wait_sec', 1.5)
        self.declare_parameter('home_wait_sec', 3.0)
        self.declare_parameter('sleep_wait_sec', 3.0)

        self.robot_name = (
            self.get_parameter('robot_name')
            .get_parameter_value()
            .string_value
            .strip('/')
        )
        if not self.robot_name:
            raise ValueError('robot_name must not be empty')

        self.joint_name = self.get_parameter('joint_name').value
        self.tag_id = int(self.get_parameter('tag_id').value)
        self.scan_positions = self._parse_positions(
            self.get_parameter('scan_positions').value
        )
        self.arm_joint_names = self._parse_names(
            self.get_parameter('arm_joint_names').value
        )
        self.preset_arm_positions = self._parse_optional_positions(
            self.get_parameter('preset_arm_positions').value
        )
        self.arm_group_name = self.get_parameter('arm_group_name').value
        self.log_path = Path(self.get_parameter('log_path').value).expanduser()
        self.detections_topic = self.get_parameter('detections_topic').value
        self.command_topic = self.get_parameter('command_topic').value

        self._latest_joint_state: Optional[JointState] = None
        self._matched_detection: Optional[dict[str, Any]] = None
        self._latest_detection: Optional[dict[str, Any]] = None
        self._last_log_time = 0.0
        self._active = bool(self.get_parameter('start_active').value)
        self._stow_requested = False

        self.create_subscription(
            JointState,
            f'/{self.robot_name}/joint_states',
            self._on_joint_state,
            10,
        )
        self.create_subscription(
            AprilTagDetectionArray,
            self.detections_topic,
            self._on_detections,
            10,
        )
        self.create_subscription(String, self.command_topic, self._on_command, 10)
        self._command_pub = self.create_publisher(
            JointSingleCommand,
            f'/{self.robot_name}/commands/joint_single',
            10,
        )
        self._group_command_pub = self.create_publisher(
            JointGroupCommand,
            f'/{self.robot_name}/commands/joint_group',
            10,
        )
        self._torque_client = self.create_client(
            TorqueEnable,
            f'/{self.robot_name}/torque_enable',
        )
        self._info_client = self.create_client(
            RobotInfo,
            f'/{self.robot_name}/get_robot_info',
        )

    def _on_command(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if command in {'turn_on', 'on', 'start', 'enable', 'run'}:
            self._active = True
            self._stow_requested = False
            self._matched_detection = None
            self.get_logger().info(f'Received {command}; scanner active.')
            return
        if command in {'turn_off', 'off', 'stop', 'disable', 'stow', 'shutdown'}:
            self._active = False
            self._stow_requested = True
            self.get_logger().info(f'Received {command}; halting scan and stowing.')
            return
        self.get_logger().warning(
            f"Unknown command '{msg.data}'. Use turn_on or turn_off."
        )

    def _parse_positions(self, raw: str) -> list[float]:
        positions = []
        for part in str(raw).replace('[', '').replace(']', '').split(','):
            value = part.strip()
            if value:
                positions.append(float(value))
        if not positions:
            raise ValueError('scan_positions must contain at least one angle.')

        max_abs = float(self.get_parameter('max_abs_position_rad').value)
        for position in positions:
            if not math.isfinite(position):
                raise ValueError('scan_positions must be finite values.')
            if abs(position) > max_abs:
                raise ValueError(
                    f'Refusing waist angle {position:.3f} rad; '
                    f'max_abs_position_rad is {max_abs:.3f}.'
                )
        return positions

    def _parse_optional_positions(self, raw: str) -> list[float]:
        if not str(raw).strip():
            return []
        positions = self._parse_float_list(raw)
        if len(positions) != len(self.arm_joint_names):
            raise ValueError(
                'preset_arm_positions must have the same number of values as '
                f'arm_joint_names ({len(self.arm_joint_names)}).'
            )
        return positions

    def _parse_float_list(self, raw: str) -> list[float]:
        values = []
        for part in str(raw).replace('[', '').replace(']', '').split(','):
            value = part.strip()
            if value:
                values.append(float(value))
        return values

    def _parse_names(self, raw: str) -> list[str]:
        names = [
            part.strip()
            for part in str(raw).replace('[', '').replace(']', '').split(',')
            if part.strip()
        ]
        if not names:
            raise ValueError('arm_joint_names must contain at least one joint.')
        return names

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        if not self._active:
            return
        for detection in msg.detections:
            if int(detection.id) == self.tag_id:
                record = self._detection_to_record(msg, detection)
                if self._should_log_detection(record):
                    self._write_log(record)
                    joint_text = (
                        f"{record['joint_position']:.3f}"
                        if record['joint_position'] is not None
                        else 'unknown'
                    )
                    self.get_logger().info(
                        f'Saw AprilTag ID {self.tag_id} at '
                        f'{joint_text} rad; logged to {self.log_path}.'
                    )
                if bool(self.get_parameter('stop_on_first_detection').value):
                    self._matched_detection = record
                self._latest_detection = record
                return

    def _should_log_detection(self, record: dict[str, Any]) -> bool:
        interval = float(self.get_parameter('min_log_interval_sec').value)
        if record['wall_time_sec'] - self._last_log_time < interval:
            return False
        self._last_log_time = record['wall_time_sec']
        return True

    def _detection_to_record(self, msg: AprilTagDetectionArray, detection) -> dict[str, Any]:
        return {
            'wall_time_sec': time.time(),
            'stamp': {
                'sec': int(msg.header.stamp.sec),
                'nanosec': int(msg.header.stamp.nanosec),
            },
            'frame_id': msg.header.frame_id,
            'family': detection.family,
            'id': int(detection.id),
            'hamming': int(detection.hamming),
            'decision_margin': float(detection.decision_margin),
            'centre': {
                'x': float(detection.centre.x),
                'y': float(detection.centre.y),
            },
            'corners': [
                {'x': float(corner.x), 'y': float(corner.y)}
                for corner in detection.corners
            ],
            'homography': [float(value) for value in detection.homography],
            'joint_position': self._current_joint_position(),
        }

    def run(self) -> bool:
        timeout = float(self.get_parameter('wait_timeout_sec').value)
        self._wait_for_command_subscriber(timeout)
        self._wait_for_group_command_subscriber(timeout)
        start_position = self._wait_for_joint_position(timeout)
        lower_limit, upper_limit = self._get_joint_limits(timeout)
        self._validate_positions(lower_limit, upper_limit)

        if bool(self.get_parameter('torque_enable').value):
            self._set_torque(True, timeout)

        if self.preset_arm_positions:
            self.get_logger().info(
                f'Commanding preset arm posture {self.preset_arm_positions} '
                f'for group {self.arm_group_name}.'
            )
            self._publish_group_positions(self.preset_arm_positions)
            self._spin_for(float(self.get_parameter('preset_settle_sec').value))

        self.get_logger().info(
            f'Waiting for commands on {self.command_topic}. Scanner '
            f"{'active' if self._active else 'idle'}."
        )

        found = self._run_command_loop()
        if found:
            self.get_logger().info(
                f'Found AprilTag ID {self.tag_id}; wrote log to {self.log_path}.'
            )
            if bool(self.get_parameter('return_to_start_on_found').value):
                self._publish_position(start_position)
                self._spin_for(float(self.get_parameter('settle_sec').value))
            return True

        self.get_logger().warning(
            f'AprilTag ID {self.tag_id} was not found before scan finished.'
        )
        if bool(self.get_parameter('return_to_start_on_timeout').value):
            self._publish_position(start_position)
            self._spin_for(float(self.get_parameter('settle_sec').value))
        return False

    def _run_command_loop(self) -> bool:
        while rclpy.ok():
            if self._stow_requested:
                self._halt_motion()
                if bool(self.get_parameter('stow_on_turn_off').value):
                    self._stow_home_then_sleep()
                self._stow_requested = False
                self.get_logger().info(
                    f'Robot is stowed. Waiting for turn_on on {self.command_topic}.'
                )
            if self._active:
                found = self._scan()
                if found:
                    return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def _scan(self) -> bool:
        max_cycles = int(self.get_parameter('max_cycles').value)
        dwell_sec = float(self.get_parameter('dwell_sec').value)
        settle_sec = float(self.get_parameter('settle_sec').value)

        cycle = 0
        while rclpy.ok() and (max_cycles <= 0 or cycle < max_cycles):
            if not self._active or self._stow_requested:
                return False
            cycle += 1
            for position in self.scan_positions:
                if not self._active or self._stow_requested:
                    return False
                self.get_logger().info(
                    f'Scan cycle {cycle}: commanding {self.joint_name} to '
                    f'{position:.3f} rad.'
                )
                self._publish_position(position)
                if self._spin_until_found(settle_sec + dwell_sec):
                    return True
                if self._should_track():
                    self._track_tag()
        return False

    def _spin_until_found(self, duration_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, duration_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._matched_detection is not None:
                return True
        return self._matched_detection is not None

    def _should_track(self) -> bool:
        if not bool(self.get_parameter('track_on_detection').value):
            return False
        detection = self._latest_detection
        if detection is None:
            return False
        lost_timeout = float(self.get_parameter('track_lost_timeout_sec').value)
        return time.time() - detection['wall_time_sec'] <= lost_timeout

    def _track_tag(self) -> None:
        self.get_logger().info(
            f'Tracking AprilTag ID {self.tag_id}; centering camera on tag.'
        )
        lost_timeout = float(self.get_parameter('track_lost_timeout_sec').value)
        interval = float(self.get_parameter('track_interval_sec').value)
        while rclpy.ok():
            if not self._active or self._stow_requested:
                return
            detection = self._latest_detection
            if detection is None or time.time() - detection['wall_time_sec'] > lost_timeout:
                self.get_logger().info(
                    f'Lost AprilTag ID {self.tag_id}; resuming waist sweep.'
                )
                return
            if self._matched_detection is not None:
                return
            self._command_tracking_correction(detection)
            deadline = time.monotonic() + max(0.05, interval)
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if not self._active or self._stow_requested:
                    return

    def _command_tracking_correction(self, detection: dict[str, Any]) -> None:
        image_width = float(self.get_parameter('image_width').value)
        image_height = float(self.get_parameter('image_height').value)
        deadband = float(self.get_parameter('track_deadband_px').value)
        max_step = float(self.get_parameter('max_track_step_rad').value)

        error_x = detection['centre']['x'] - image_width / 2.0
        error_y = detection['centre']['y'] - image_height / 2.0

        waist_step = 0.0
        if abs(error_x) > deadband:
            waist_step = self._clamp(
                float(self.get_parameter('waist_px_gain').value) * error_x,
                -max_step,
                max_step,
            )

        wrist_step = 0.0
        if abs(error_y) > deadband:
            wrist_step = self._clamp(
                float(self.get_parameter('wrist_px_gain').value) * error_y,
                -max_step,
                max_step,
            )

        if waist_step:
            current = self._current_joint_position_for('waist')
            if current is not None:
                goal = self._clamp(
                    current + waist_step,
                    float(self.get_parameter('track_min_waist_rad').value),
                    float(self.get_parameter('track_max_waist_rad').value),
                )
                self._publish_single_position('waist', goal)

        if wrist_step:
            current = self._current_joint_position_for('wrist_angle')
            if current is not None:
                goal = self._clamp(
                    current + wrist_step,
                    float(self.get_parameter('track_min_wrist_angle_rad').value),
                    float(self.get_parameter('track_max_wrist_angle_rad').value),
                )
                self._publish_single_position('wrist_angle', goal)

    def _clamp(self, value: float, low: float, high: float) -> float:
        return min(max(value, low), high)

    def _wait_for_command_subscriber(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self._command_pub.get_subscription_count() > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise TimeoutError(
            f'No subscriber found on /{self.robot_name}/commands/joint_single. '
            'Start the robot driver first.'
        )

    def _wait_for_group_command_subscriber(self, timeout: float) -> None:
        if not self.preset_arm_positions:
            return
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self._group_command_pub.get_subscription_count() > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise TimeoutError(
            f'No subscriber found on /{self.robot_name}/commands/joint_group. '
            'Start the robot driver first.'
        )

    def _wait_for_joint_position(self, timeout: float) -> float:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            position = self._current_joint_position()
            if position is not None:
                return position
            rclpy.spin_once(self, timeout_sec=0.1)
        raise TimeoutError(
            f'No /{self.robot_name}/joint_states sample contained joint '
            f"'{self.joint_name}'."
        )

    def _current_joint_position(self) -> Optional[float]:
        return self._current_joint_position_for(self.joint_name)

    def _current_joint_position_for(self, joint_name: str) -> Optional[float]:
        msg = self._latest_joint_state
        if msg is None or joint_name not in msg.name:
            return None
        index = msg.name.index(joint_name)
        if index >= len(msg.position):
            return None
        return float(msg.position[index])

    def _get_joint_limits(self, timeout: float) -> tuple[Optional[float], Optional[float]]:
        if not self._info_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warning(
                f'Robot info service /{self.robot_name}/get_robot_info is '
                'unavailable; skipping joint-limit check.'
            )
            return None, None

        request = RobotInfo.Request()
        request.cmd_type = 'single'
        request.name = self.joint_name
        future = self._info_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        if not future.done() or future.result() is None:
            self.get_logger().warning(
                'Robot info request timed out; skipping joint-limit check.'
            )
            return None, None

        response = future.result()
        if not response.joint_lower_limits or not response.joint_upper_limits:
            return None, None
        return (
            float(response.joint_lower_limits[0]),
            float(response.joint_upper_limits[0]),
        )

    def _validate_positions(
        self,
        lower_limit: Optional[float],
        upper_limit: Optional[float],
    ) -> None:
        for position in self.scan_positions:
            if lower_limit is not None and position < lower_limit:
                raise ValueError(
                    f'Scan angle {position:.3f} rad is below lower limit '
                    f'{lower_limit:.3f} rad.'
                )
            if upper_limit is not None and position > upper_limit:
                raise ValueError(
                    f'Scan angle {position:.3f} rad is above upper limit '
                    f'{upper_limit:.3f} rad.'
                )

    def _set_torque(self, enable: bool, timeout: float) -> None:
        if not self._torque_client.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(
                f'Torque service /{self.robot_name}/torque_enable is unavailable.'
            )

        request = TorqueEnable.Request()
        request.cmd_type = 'single'
        request.name = self.joint_name
        request.enable = enable
        future = self._torque_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        if not future.done() or future.result() is None:
            raise TimeoutError(f'Torque request for {self.joint_name} timed out.')

    def _publish_position(self, position: float) -> None:
        self._publish_single_position(self.joint_name, position)

    def _publish_single_position(self, joint_name: str, position: float) -> None:
        repeats = int(self.get_parameter('command_repeats').value)
        msg = JointSingleCommand()
        msg.name = joint_name
        msg.cmd = float(position)

        for _ in range(max(1, repeats)):
            self._command_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def _publish_group_positions(self, positions: list[float]) -> None:
        repeats = int(self.get_parameter('command_repeats').value)
        msg = JointGroupCommand()
        msg.name = self.arm_group_name
        msg.cmd = [float(position) for position in positions]

        for _ in range(max(1, repeats)):
            self._group_command_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def _halt_motion(self) -> None:
        current = self._current_group_positions()
        if current is not None:
            self.get_logger().info(f'Holding current arm posture: {current}')
            self._publish_group_positions(current)
        self._spin_for(float(self.get_parameter('halt_hold_sec').value))

    def _current_group_positions(self) -> Optional[list[float]]:
        msg = self._latest_joint_state
        if msg is None:
            return None
        positions = []
        for joint_name in self.arm_joint_names:
            if joint_name not in msg.name:
                return None
            index = msg.name.index(joint_name)
            if index >= len(msg.position):
                return None
            positions.append(float(msg.position[index]))
        return positions

    def _stow_home_then_sleep(self) -> None:
        home = self._parse_float_list(self.get_parameter('home_positions').value)
        sleep = self._parse_float_list(self.get_parameter('sleep_positions').value)
        if bool(self.get_parameter('staged_home').value):
            self._command_staged_home(home)
        else:
            self.get_logger().info(f'Commanding Home pose before shutdown: {home}')
            self._publish_group_positions(home)
        self._spin_for(float(self.get_parameter('home_wait_sec').value))
        self.get_logger().info(f'Commanding Sleep pose before shutdown: {sleep}')
        self._publish_group_positions(sleep)
        self._spin_for(float(self.get_parameter('sleep_wait_sec').value))

    def _command_staged_home(self, home: list[float]) -> None:
        current = self._current_group_positions()
        if current is None or len(current) != len(home):
            self.get_logger().warning(
                'Could not read current arm posture; falling back to group Home.'
            )
            self._publish_group_positions(home)
            return

        wait_sec = float(self.get_parameter('staged_home_step_wait_sec').value)
        staged = list(current)

        # VX300 order: waist, shoulder, elbow, wrist_angle, wrist_rotate.
        # Bring wrist/waist home first, shoulder second, elbow last.
        for index in (0, 3, 4):
            if index < len(staged):
                staged[index] = home[index]
        self.get_logger().info(f'Staged Home step 1, wrist/waist: {staged}')
        self._publish_group_positions(staged)
        self._spin_for(wait_sec)

        if len(staged) > 1:
            staged[1] = home[1]
        self.get_logger().info(f'Staged Home step 2, shoulder: {staged}')
        self._publish_group_positions(staged)
        self._spin_for(wait_sec)

        if len(staged) > 2:
            staged[2] = home[2]
        self.get_logger().info(f'Staged Home step 3, elbow last: {staged}')
        self._publish_group_positions(staged)

    def _spin_for(self, duration_sec: float) -> None:
        deadline = time.monotonic() + max(0.0, duration_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _write_log(self, record: Optional[dict[str, Any]]) -> None:
        if record is None:
            return
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        with self.log_path.open('a', encoding='utf-8') as file:
            file.write(json.dumps(record, sort_keys=True) + '\n')


def main() -> int:
    rclpy.init()
    node = None
    try:
        node = WaistTagScanner()
        return 0 if node.run() else 2
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(str(exc), file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
