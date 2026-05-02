"""Command the arm through Home and Sleep before shutdown."""

import sys
import time
from typing import Optional

import numpy as np
import rclpy
from interbotix_xs_msgs.msg import JointGroupCommand
from interbotix_xs_msgs.srv import TorqueEnable
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class SafeArmStow(Node):
    """Publish a conservative Home -> Sleep arm group sequence."""

    def __init__(self) -> None:
        super().__init__('safe_arm_stow')
        self.declare_parameter('robot_name', 'vx300')
        self.declare_parameter('group_name', 'arm')
        self.declare_parameter('moveit_group_name', 'interbotix_arm')
        self.declare_parameter('home_positions', '0,0,0,0,0')
        self.declare_parameter('sleep_positions', '0,-1.85,1.55,0.8,0')
        self.declare_parameter('use_moveit_for_stow', True)
        self.declare_parameter('home_wait_sec', 3.0)
        self.declare_parameter('sleep_wait_sec', 3.0)
        self.declare_parameter('command_repeats', 8)
        self.declare_parameter('wait_timeout_sec', 10.0)
        self.declare_parameter('torque_enable', True)
        self.declare_parameter('torque_off_after_sleep', False)
        self.declare_parameter(
            'arm_joint_names',
            'waist,shoulder,elbow,wrist_angle,wrist_rotate',
        )

        self.robot_name = (
            self.get_parameter('robot_name')
            .get_parameter_value()
            .string_value
            .strip('/')
        )
        self.group_name = self.get_parameter('group_name').value
        self.arm_joint_names = self._parse_names(
            self.get_parameter('arm_joint_names').value
        )
        self._latest_joint_state = None
        self._moveit: Optional[MoveItPy] = None
        self._moveit_arm = None
        self.create_subscription(
            JointState,
            f'/{self.robot_name}/joint_states',
            self._on_joint_state,
            10,
        )
        self._command_pub = self.create_publisher(
            JointGroupCommand,
            f'/{self.robot_name}/commands/joint_group',
            10,
        )
        self._torque_client = self.create_client(
            TorqueEnable,
            f'/{self.robot_name}/torque_enable',
        )

    def run(self) -> None:
        timeout = float(self.get_parameter('wait_timeout_sec').value)
        self._wait_for_command_subscriber(timeout)

        if bool(self.get_parameter('torque_enable').value):
            self._set_torque(True, timeout)

        home = self._parse_positions(self.get_parameter('home_positions').value)
        sleep = self._parse_positions(self.get_parameter('sleep_positions').value)

        if not self._plan_and_execute_group_pose(home, 'Home'):
            raise RuntimeError(
                'Stow aborted because MoveIt did not produce a valid Home trajectory.'
            )
        self._spin_for(float(self.get_parameter('home_wait_sec').value))

        if not self._plan_and_execute_group_pose(sleep, 'Sleep'):
            raise RuntimeError(
                'Sleep retraction was not executed because MoveIt did not '
                'produce a valid trajectory.'
            )
        self._spin_for(float(self.get_parameter('sleep_wait_sec').value))

        if bool(self.get_parameter('torque_off_after_sleep').value):
            self.get_logger().info('Torquing off arm after Sleep pose.')
            self._set_torque(False, timeout)

        self.get_logger().info('Safe stow sequence complete.')

    def _on_joint_state(self, msg) -> None:
        self._latest_joint_state = msg

    def _parse_positions(self, raw: str) -> list[float]:
        positions = [
            float(part.strip())
            for part in str(raw).replace('[', '').replace(']', '').split(',')
            if part.strip()
        ]
        if not positions:
            raise ValueError('Position list must not be empty.')
        return positions

    def _parse_names(self, raw: str) -> list[str]:
        names = [
            part.strip()
            for part in str(raw).replace('[', '').replace(']', '').split(',')
            if part.strip()
        ]
        if not names:
            raise ValueError('arm_joint_names must contain at least one joint.')
        return names

    def _wait_for_command_subscriber(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self._command_pub.get_subscription_count() > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise TimeoutError(
            f'No subscriber found on /{self.robot_name}/commands/joint_group. '
            'Start the robot driver first.'
        )

    def _set_torque(self, enable: bool, timeout: float) -> None:
        if not self._torque_client.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(
                f'Torque service /{self.robot_name}/torque_enable is unavailable.'
            )

        request = TorqueEnable.Request()
        request.cmd_type = 'group'
        request.name = self.group_name
        request.enable = enable
        future = self._torque_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        if not future.done() or future.result() is None:
            raise TimeoutError(f'Torque request for {self.group_name} timed out.')

    def _publish_group_positions(self, positions: list[float]) -> None:
        repeats = int(self.get_parameter('command_repeats').value)
        msg = JointGroupCommand()
        msg.name = self.group_name
        msg.cmd = [float(position) for position in positions]

        for _ in range(max(1, repeats)):
            self._command_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

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

    def _plan_and_execute_group_pose(self, positions: list[float], label: str) -> bool:
        if not bool(self.get_parameter('use_moveit_for_stow').value):
            self.get_logger().error(
                'use_moveit_for_stow is false; refusing direct stow actuation.'
            )
            return False
        if len(positions) != len(self.arm_joint_names):
            self.get_logger().error(
                f'{label} pose length does not match arm joints; refusing actuation.'
            )
            return False

        try:
            if self._moveit is None or self._moveit_arm is None:
                self.get_logger().info('Initializing MoveIt planner for stow poses.')
                self._moveit = MoveItPy(node_name='safe_stow_moveit')
                self._moveit_arm = self._moveit.get_planning_component(
                    self.get_parameter('moveit_group_name').value
                )

            goal_state = RobotState(self._moveit.get_robot_model())
            goal_state.set_to_default_values()
            goal_state.set_joint_group_positions(
                self.get_parameter('moveit_group_name').value,
                np.array(positions, dtype=np.float64),
            )
            goal_state.update()

            self.get_logger().info(f'Planning MoveIt {label} trajectory: {positions}')
            self._moveit_arm.set_start_state_to_current_state()
            self._moveit_arm.set_goal_state(robot_state=goal_state)
            plan_result = self._moveit_arm.plan()
            if not plan_result:
                self.get_logger().error(
                    f'MoveIt could not find a {label} trajectory; refusing actuation.'
                )
                return False

            self.get_logger().info(f'Executing MoveIt {label} trajectory.')
            self._moveit.execute(
                plan_result.trajectory,
                controllers=[],
                blocking=True,
            )
            self.get_logger().info(f'MoveIt {label} trajectory complete.')
            return True
        except Exception as exc:
            self.get_logger().error(
                f'MoveIt {label} planning failed ({exc}); refusing actuation.'
            )
            return False

    def _spin_for(self, duration_sec: float) -> None:
        deadline = time.monotonic() + max(0.0, duration_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)


def main() -> int:
    rclpy.init()
    node: Optional[SafeArmStow] = None
    try:
        node = SafeArmStow()
        node.run()
        return 0
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
