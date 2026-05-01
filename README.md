# AuRo Robot Arm

ROS 2 Jazzy workspace for controlling an Interbotix X-Series `vx300` robot arm over USB/U2D2. This repo is the application layer for the AuRo autonomous robot arm project: it brings up the Interbotix driver, provides small first-motion test nodes, and is the starting point for camera-guided pick and place.

## Current Hardware

- Robot arm: Interbotix `vx300`
- Arm DOF: 5-DOF
- ROS namespace: `vx300`
- USB interface: U2D2 DYNAMIXEL adapter, exposed as `/dev/ttyDXL`
- Servo IDs found on the working bus: `1-8`
- ROS distro: Jazzy

The `vx300` arm joints are:

```text
waist
shoulder
elbow
wrist_angle
wrist_rotate
```

The gripper is a separate actuator and is not counted as one of the 5 arm DOF.

## What This Package Provides

- `usb_bringup.launch.py`: starts the Interbotix `xs_sdk` driver using a selected USB port.
- `view_robot.launch.py`: opens a view-only RViz model with a joint-state GUI for checking the URDF without hardware.
- `rviz/vx300.rviz`: RViz layout for the AuRo `vx300` namespace.
- `config/vx300.yaml`: local motor map and joint-limit config for the 5-DOF `vx300`.
- `config/vx300_arm_only.yaml`: same 5-DOF arm map without the gripper actuator.
- `nudge_joint`: safely moves one joint by a small relative amount from its current position.
- `pulse_gripper`: briefly opens or closes the gripper in PWM mode, then sends a stop command.
- `scan_dynamixels.py`: scans the DYNAMIXEL bus for responding servo IDs.

## Workspace Setup

Run from the repository root:

```bash
cd /home/sanat/Arm_Robot/AuRo_robo_arm
source /opt/ros/jazzy/setup.bash
vcs import --recursive src < dependencies/interbotix_jazzy.repos
python3 scripts/prepare_interbotix_jazzy.py
```

Install the minimal system dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-dynamixel-sdk \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-moveit-planners-ompl \
  ros-jazzy-moveit-ros-visualization \
  ros-jazzy-moveit-simple-controller-manager \
  ros-jazzy-nav2-msgs \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2 \
  ros-jazzy-tf-transformations \
  python3-transforms3d
```

Install remaining ROS dependencies and build:

```bash
rosdep install --from-paths \
  src/auro_robo_arm \
  src/interbotix_ros_core/interbotix_ros_xseries \
  src/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_common_modules \
  src/interbotix_ros_toolboxes/interbotix_xs_toolbox/interbotix_xs_modules \
  src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_control \
  src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_descriptions \
  --ignore-src -r -y --skip-keys python-transforms3d-pip

colcon build --symlink-install --packages-up-to auro_robo_arm interbotix_xsarm_control
source install/setup.bash
```

## Visualize The Robot In RViz

For a hardware-free model check, launch RViz with a joint-state slider GUI:

```bash
cd /home/sanat/Arm_Robot/AuRo_robo_arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch auro_robo_arm view_robot.launch.py \
  robot_model:=vx300 \
  robot_name:=vx300
```

To visualize the physical arm while the driver is running, set `use_rviz:=true` in `usb_bringup.launch.py`.

The RViz config uses `world` as the fixed frame and expects the normal Interbotix `vx300/base_link` TF tree.

If the RViz model moves opposite the real arm for the Y-axis pitch joints
(`shoulder`, `elbow`, or `wrist_angle`), initialize the servo EEPROM once with
the configured `Drive_Mode` values:

```bash
ros2 launch auro_robo_arm usb_bringup.launch.py \
  robot_model:=vx300 \
  robot_name:=vx300 \
  port:=/dev/ttyDXL \
  load_configs:=true \
  use_rviz:=true
```

After that one-time run, go back to `load_configs:=false` for normal bringup.
Those pitch joints intentionally use reverse drive mode in the motor config; if
that EEPROM bit is stale, their published joint states look negated in RViz.

## Control The Robot In MoveIt RViz

MoveIt gives RViz the MotionPlanning panel for drag-to-pose planning and
trajectory execution. Build the MoveIt and ROS 2 control packages along with
this package:

```bash
cd /home/sanat/Arm_Robot/AuRo_robo_arm
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-up-to auro_robo_arm interbotix_xsarm_moveit \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

Start the physical arm with MoveIt RViz:

```bash
ros2 launch auro_robo_arm moveit.launch.py \
  robot_model:=vx300 \
  robot_name:=vx300 \
  port:=/dev/ttyDXL \
  load_configs:=false \
  use_moveit_rviz:=true
```

In RViz, use the `MotionPlanning` panel, select the `interbotix_arm` planning
group, then use `Plan` before `Execute`. Keep the first tests small and slow.

## VX300 Joint Limits

The default `usb_bringup.launch.py` now uses this package's `config/vx300.yaml` for `robot_model:=vx300`. It maps the 5-DOF `vx300` hardware correctly:

```text
waist:        ID 1
shoulder:     ID 2 + shadow ID 3
elbow:        ID 4 + shadow ID 5
wrist_angle:  ID 6
wrist_rotate: ID 7
gripper:      ID 8
```

This model has exactly five arm joints. If you need to launch without the gripper, pass:

```bash
motor_configs:=src/auro_robo_arm/config/vx300_arm_only.yaml
```

The arm joint limits are derived from the [Trossen VX300 specification](https://github.com/TrossenRobotics/interbotix_xsarms_docs/blob/main/docs/specifications/vx300.rst):

```text
waist:        -180 to 180 deg
shoulder:     -101 to 101 deg
elbow:        -101 to 92 deg
wrist_angle:  -107 to 130 deg
wrist_rotate: -180 to 180 deg
```

`load_configs:=false` remains the default to avoid unnecessary EEPROM writes during normal bringup. To write the configured limits to the servos, run the driver deliberately once with `load_configs:=true`, then return to `load_configs:=false` for normal sessions.

If you accidentally built from the wrong directory or stale packages are causing trouble:

```bash
cd /home/sanat/Arm_Robot/AuRo_robo_arm
rm -rf build install log
source /opt/ros/jazzy/setup.bash
vcs import --recursive src < dependencies/interbotix_jazzy.repos
python3 scripts/prepare_interbotix_jazzy.py
colcon build --symlink-install --packages-up-to auro_robo_arm interbotix_xsarm_control
source install/setup.bash
```

## USB Permissions

Plug in and power the arm. The U2D2 USB cable does not power the servos, so the arm power supply must also be on.

Install the udev rule once to create `/dev/ttyDXL` and grant access to users in
the `dialout` group:

```bash
cd /home/sanat/Arm_Robot/AuRo_robo_arm
sudo cp udev/99-auro-u2d2.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout "$USER"
```

Log out and log back in so the new group membership is applied, then unplug and
replug the U2D2. Check the device:

```bash
ls -l /dev/ttyDXL
```

The Interbotix stack defaults to `/dev/ttyDXL` because it is stable across
USB replugging. The raw kernel name, such as `/dev/ttyUSB0`, can change if
another USB serial device is plugged in first.

## Scan The Servo Bus

Before launching ROS control, verify the DYNAMIXEL bus:

```bash
cd /home/sanat/Arm_Robot/AuRo_robo_arm
source /opt/ros/jazzy/setup.bash
python3 scripts/scan_dynamixels.py
```

A working `vx300` should respond on Protocol 2.0 at `1000000` bps with IDs `1-8`.

## Start The Arm Driver

Terminal 1:

```bash
cd /home/sanat/Arm_Robot/AuRo_robo_arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch auro_robo_arm usb_bringup.launch.py \
  robot_model:=vx300 \
  robot_name:=vx300 \
  port:=/dev/ttyDXL \
  load_configs:=false \
  use_rviz:=false
```

Leave this running. The launch file defaults to this package's `vx300` motor config when `robot_model:=vx300` and generates a temporary mode config that sets:

- arm joints to position mode
- gripper to PWM mode
- a smooth time-based motion profile

## Check Driver Health

Terminal 2:

```bash
cd /home/sanat/Arm_Robot/AuRo_robo_arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic echo /vx300/joint_states --once
ros2 service list | grep vx300
```

If `/vx300/joint_states` prints joint positions, the driver is alive.

## First Joint Motion

Start with the waist. It is the safest first joint to test.

```bash
ros2 run auro_robo_arm nudge_joint --ros-args \
  -p robot_name:=vx300 \
  -p joint_name:=waist \
  -p delta_rad:=0.08 \
  -p return_to_start:=true
```

A larger, still modest waist test:

```bash
ros2 run auro_robo_arm nudge_joint --ros-args \
  -p robot_name:=vx300 \
  -p joint_name:=waist \
  -p delta_rad:=0.16 \
  -p return_to_start:=true
```

Useful `nudge_joint` parameters:

- `joint_name`: one of `waist`, `shoulder`, `elbow`, `wrist_angle`, `wrist_rotate`
- `delta_rad`: relative motion from the current position
- `return_to_start`: returns to the original position after the test
- `settle_sec`: wait time after each command, default `2.0`
- `max_delta_rad`: safety cap, default `0.35`

For quicker tests:

```bash
ros2 run auro_robo_arm nudge_joint --ros-args \
  -p robot_name:=vx300 \
  -p joint_name:=waist \
  -p delta_rad:=0.16 \
  -p settle_sec:=0.5 \
  -p return_to_start:=false
```

## Gripper Test

Open:

```bash
ros2 run auro_robo_arm pulse_gripper --ros-args \
  -p robot_name:=vx300 \
  -p effort:=180.0 \
  -p duration_sec:=0.5
```

Close:

```bash
ros2 run auro_robo_arm pulse_gripper --ros-args \
  -p robot_name:=vx300 \
  -p effort:=-180.0 \
  -p duration_sec:=0.5
```

The helper sends a stop command after the pulse.

## Why Motion Feels Delayed

The launch wrapper sets a smooth time-based DYNAMIXEL profile:

```yaml
profile_type: time
profile_velocity: 2000
profile_acceleration: 1000
```

This makes movements gentle and slow, which is useful for early testing. The `nudge_joint` helper also waits after each command. If `return_to_start:=true`, it performs a move out, waits, moves back, and waits again.

## Camera Guard Mode

The guard-mode launch brings up the arm driver, webcam, AprilTag detector, QR detector, and a waist-scan controller. It starts idle by default and waits for an external ROS actor to command it on or off.

```bash
cd /home/sanat/Arm_Robot/AuRo_robo_arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch auro_robo_arm waist_tag_scan.launch.py
```

The command topic is:

```text
/auro/command
```

Manual start command:

```bash
ros2 topic pub --once /auro/command std_msgs/msg/String "{data: turn_on}"
```

Manual stop/stow command:

```bash
ros2 topic pub --once /auro/command std_msgs/msg/String "{data: turn_off}"
```

Accepted on commands are `turn_on`, `on`, `start`, `enable`, and `run`.
Accepted off commands are `turn_off`, `off`, `stop`, `disable`, `stow`, and `shutdown`.

On startup, the scanner commands a default viewing posture before scanning:

```text
waist:        0
shoulder:     0
elbow:        0
wrist_angle:  1.047 rad  # 60 deg
wrist_rotate: 0
```

Then it sweeps the waist through:

```text
-0.45, -0.25, 0.0, 0.25, 0.45 rad
```

When AprilTag ID `0` is seen, the node logs detections and tracks the tag by making small `waist` and `wrist_angle` corrections to center it in the camera image. If the tag is lost, it resumes sweeping.

Detection logs are written as JSON lines:

```bash
tail -f /tmp/auro_tag_scan.jsonl
```

Useful launch overrides:

```bash
ros2 launch auro_robo_arm waist_tag_scan.launch.py \
  tag_id:=0 \
  scan_positions:="-0.6,-0.3,0.0,0.3,0.6" \
  dwell_sec:=1.5 \
  track_deadband_px:=25.0
```

For manual testing, start scanning immediately instead of waiting for `/auro/command`:

```bash
ros2 launch auro_robo_arm waist_tag_scan.launch.py start_active:=true
```

If an axis tracks away from the tag instead of toward it, flip the relevant gain:

```bash
ros2 launch auro_robo_arm waist_tag_scan.launch.py waist_px_gain:=0.0008
ros2 launch auro_robo_arm waist_tag_scan.launch.py wrist_px_gain:=-0.0008
```

On `turn_off`, the scanner stops scan/track commands, briefly holds the current arm posture, then stows:

```text
1. Home step 1: waist and wrist joints go home.
2. Home step 2: shoulder goes home.
3. Home step 3: elbow goes home last.
4. Sleep pose: [0, -1.85, 1.55, 0.8, 0].
5. Return to idle and wait for the next turn_on.
```

You can also run the stow sequence manually while the robot driver is still running:

```bash
ros2 launch auro_robo_arm safe_arm_stow.launch.py
```

## Troubleshooting

If scan finds no servos:

- Make sure the arm power supply is on.
- Make sure the U2D2 is connected to the DYNAMIXEL bus, not only USB.
- Check that the 3-pin DYNAMIXEL cables are fully seated.
- Make sure no other program, such as DYNAMIXEL Wizard, is using `/dev/ttyDXL`.
- If `/dev/ttyDXL` does not exist, reinstall the udev rule and replug the U2D2.

If `xs_sdk` reports `YAML::BadFile`:

- Make sure the latest `usb_bringup.launch.py` is installed or symlinked.
- Rebuild with `colcon build --symlink-install --packages-up-to auro_robo_arm interbotix_xsarm_control`.
- Source `install/setup.bash` again.

If joint topics do not appear:

```bash
ros2 topic list | grep vx300
ros2 service list | grep vx300
```

If the robot starts but does not move:

- Confirm the driver launch is still running.
- Confirm `/vx300/joint_states` is publishing.
- Try the waist first with a small `delta_rad`.
- Keep `load_configs:=false` unless intentionally writing EEPROM config values.

## Safety

Support the arm during early tests. Torque changes, driver shutdowns, or unexpected errors can let gravity move the shoulder and elbow. Start with waist and wrist joints before testing shoulder or elbow.

Use small deltas:

```text
0.05 rad = very small
0.08 rad = safe first test
0.16 rad = moderate waist/wrist test
```

## Roadmap To Camera Pick And Place

The robot arm now has basic actuation. The next system layers are:

1. Camera bringup
   - Choose RGB-D camera or regular RGB camera.
   - Publish image topics in ROS 2.
   - Verify frames in RViz.

2. Camera calibration
   - Calibrate camera intrinsics.
   - Find the transform between camera frame and robot base frame.
   - Publish that transform through `tf2`.

3. Object detection
   - Start with fiducials, colored objects, or simple segmentation.
   - Convert detected image location into a 3D target pose.

4. Motion planning
   - Use MoveIt for reaching target poses.
   - Define safe approach, grasp, lift, and place poses.

5. Grasp sequence
   - Move above object.
   - Open gripper.
   - Move down.
   - Close gripper.
   - Lift.
   - Move to place location.
   - Open gripper.

6. Integration node
   - Subscribe to detection output.
   - Query transforms.
   - Command arm and gripper actions in order.

Good first milestone: mount the camera, publish its ROS image stream, and detect one object on the table while the arm remains stationary.
