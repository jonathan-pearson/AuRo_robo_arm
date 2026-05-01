"""Pulse the Interbotix gripper in PWM mode, then stop it."""

import sys
import time

import rclpy
from interbotix_xs_msgs.msg import JointSingleCommand
from rclpy.node import Node


class PulseGripper(Node):
    """Send a timed gripper PWM command."""

    def __init__(self) -> None:
        super().__init__('pulse_gripper')
        self.declare_parameter('robot_name', 'vx300')
        self.declare_parameter('gripper_name', 'gripper')
        self.declare_parameter('effort', 180.0)
        self.declare_parameter('duration_sec', 0.5)
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
        self.gripper_name = (
            self.get_parameter('gripper_name').get_parameter_value().string_value
        )
        self.publisher = self.create_publisher(
            JointSingleCommand,
            f'/{self.robot_name}/commands/joint_single',
            10,
        )

    def run(self) -> None:
        """Pulse the gripper and then stop."""
        effort = self.get_parameter('effort').get_parameter_value().double_value
        duration_sec = (
            self.get_parameter('duration_sec').get_parameter_value().double_value
        )
        command_repeats = (
            self.get_parameter('command_repeats').get_parameter_value().integer_value
        )

        if abs(effort) > 350.0:
            raise ValueError('Refusing gripper effort above 350 PWM.')
        if duration_sec > 2.0:
            raise ValueError('Refusing gripper pulse longer than 2 seconds.')

        self._wait_for_subscriber(timeout_sec=5.0)

        direction = 'open' if effort > 0 else 'close' if effort < 0 else 'stop'
        self.get_logger().info(
            f'Pulsing gripper {direction}: effort={effort:.1f}, '
            f'duration={duration_sec:.2f}s'
        )
        self._publish(effort, command_repeats)
        self._spin_for(duration_sec)
        self._publish(0.0, command_repeats)
        self.get_logger().info('Sent gripper stop command.')

    def _wait_for_subscriber(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self.publisher.get_subscription_count() > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise TimeoutError(
            f'No subscriber found on /{self.robot_name}/commands/joint_single.'
        )

    def _publish(self, effort: float, repeats: int) -> None:
        msg = JointSingleCommand(name=self.gripper_name, cmd=float(effort))
        for _ in range(max(1, repeats)):
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def _spin_for(self, duration_sec: float) -> None:
        deadline = time.monotonic() + max(0.0, duration_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)


def main() -> int:
    """Run the pulse_gripper command-line entry point."""
    rclpy.init()
    node = None
    try:
        node = PulseGripper()
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
