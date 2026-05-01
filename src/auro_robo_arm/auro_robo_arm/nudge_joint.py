"""Move one Interbotix X-Series joint by a small, current-state-relative amount."""

import math
import sys
import time
from typing import Optional, Tuple

import rclpy
from interbotix_xs_msgs.msg import JointSingleCommand
from interbotix_xs_msgs.srv import RobotInfo, TorqueEnable
from rclpy.node import Node
from sensor_msgs.msg import JointState


class NudgeJoint(Node):
    """Command one Interbotix joint a small distance from its current position."""

    def __init__(self) -> None:
        super().__init__('nudge_joint')

        self.declare_parameter('robot_name', 'vx300')
        self.declare_parameter('joint_name', 'waist')
        self.declare_parameter('delta_rad', 0.10)
        self.declare_parameter('use_absolute_goal', False)
        self.declare_parameter('goal_rad', 0.0)
        self.declare_parameter('return_to_start', True)
        self.declare_parameter('torque_enable', True)
        self.declare_parameter('settle_sec', 2.0)
        self.declare_parameter('wait_timeout_sec', 10.0)
        self.declare_parameter('max_delta_rad', 0.35)
        self.declare_parameter('command_repeats', 5)

        robot_name = (
            self.get_parameter('robot_name')
            .get_parameter_value()
            .string_value
            .strip('/')
        )
        if not robot_name:
            raise ValueError('robot_name must not be empty')
        self.robot_name = robot_name
        self.joint_name = (
            self.get_parameter('joint_name').get_parameter_value().string_value
        )

        self._latest_joint_state: Optional[JointState] = None
        self._joint_sub = self.create_subscription(
            JointState,
            f'/{self.robot_name}/joint_states',
            self._on_joint_state,
            10,
        )
        self._command_pub = self.create_publisher(
            JointSingleCommand,
            f'/{self.robot_name}/commands/joint_single',
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

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def run(self) -> None:
        """Run the one-shot nudge command."""
        timeout = (
            self.get_parameter('wait_timeout_sec').get_parameter_value().double_value
        )
        settle_sec = (
            self.get_parameter('settle_sec').get_parameter_value().double_value
        )
        command_repeats = (
            self.get_parameter('command_repeats').get_parameter_value().integer_value
        )
        return_to_start = (
            self.get_parameter('return_to_start').get_parameter_value().bool_value
        )
        torque_enable = (
            self.get_parameter('torque_enable').get_parameter_value().bool_value
        )

        self._wait_for_command_subscriber(timeout)
        start_position = self._wait_for_joint_position(timeout)
        lower_limit, upper_limit = self._get_joint_limits(timeout)
        goal_position = self._resolve_goal(start_position, lower_limit, upper_limit)

        self.get_logger().info(
            f'Commanding /{self.robot_name}/{self.joint_name}: '
            f'{start_position:.4f} rad -> {goal_position:.4f} rad'
        )

        if torque_enable:
            self._set_torque(True, timeout)

        self._publish_position(goal_position, command_repeats)
        self._spin_for(settle_sec)

        if return_to_start:
            self.get_logger().info(
                f'Returning {self.joint_name} to {start_position:.4f} rad'
            )
            self._publish_position(start_position, command_repeats)
            self._spin_for(settle_sec)

    def _wait_for_command_subscriber(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self._command_pub.get_subscription_count() > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise TimeoutError(
            f'No subscriber found on /{self.robot_name}/commands/joint_single. '
            'Start usb_bringup.launch.py first.'
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
        msg = self._latest_joint_state
        if msg is None or self.joint_name not in msg.name:
            return None
        index = msg.name.index(self.joint_name)
        if index >= len(msg.position):
            return None
        return float(msg.position[index])

    def _get_joint_limits(
        self,
        timeout: float,
    ) -> Tuple[Optional[float], Optional[float]]:
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

    def _resolve_goal(
        self,
        start_position: float,
        lower_limit: Optional[float],
        upper_limit: Optional[float],
    ) -> float:
        use_absolute_goal = (
            self.get_parameter('use_absolute_goal').get_parameter_value().bool_value
        )
        if use_absolute_goal:
            goal = self.get_parameter('goal_rad').get_parameter_value().double_value
        else:
            delta = self.get_parameter('delta_rad').get_parameter_value().double_value
            max_delta = (
                self.get_parameter('max_delta_rad').get_parameter_value().double_value
            )
            if abs(delta) > max_delta:
                raise ValueError(
                    f'Refusing delta_rad={delta:.4f}; max_delta_rad is '
                    f'{max_delta:.4f}. Increase max_delta_rad deliberately '
                    'if you need a larger move.'
                )
            goal = start_position + delta

        if not math.isfinite(goal):
            raise ValueError('Goal position must be finite.')

        if lower_limit is not None and goal < lower_limit:
            raise ValueError(
                f'Goal {goal:.4f} rad is below {self.joint_name} lower limit '
                f'{lower_limit:.4f} rad.'
            )
        if upper_limit is not None and goal > upper_limit:
            raise ValueError(
                f'Goal {goal:.4f} rad is above {self.joint_name} upper limit '
                f'{upper_limit:.4f} rad.'
            )
        return goal

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

    def _publish_position(self, position: float, repeats: int) -> None:
        msg = JointSingleCommand()
        msg.name = self.joint_name
        msg.cmd = float(position)

        for _ in range(max(1, repeats)):
            self._command_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def _spin_for(self, duration_sec: float) -> None:
        deadline = time.monotonic() + max(0.0, duration_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)


def main() -> int:
    """Run the nudge_joint command-line entry point."""
    rclpy.init()
    node = None
    try:
        node = NudgeJoint()
        node.run()
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
