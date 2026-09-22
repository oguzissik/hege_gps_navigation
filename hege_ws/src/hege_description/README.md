# Hege vehicle description

Measured starting model for ROS 2 Humble, RViz and Gazebo Harmonic.

## Confirmed geometry

| Quantity | Value |
|---|---:|
| wheelbase (rear axle centre to front axle centre) | 1.90 m |
| track width (left wheel centre to right wheel centre) | 1.55 m |
| rear wheel radius | 0.40 m |
| front wheel radius | 0.28 m |
| base_link / vehicle CoM height | 1.10 m |
| Pixhawk IMU height | 1.20 m |
| GPS antenna phase-centre height | 1.30 m |
| maximum steering angle | 35 deg |
| approximate total mass | 1300 kg |

The minimum bicycle-model turning radius is `1.90 / tan(35 deg) = 2.71 m`.

TF geometry:

```text
base_footprint (ground z=0)
  -> base_link (z=1.10)
       -> imu_link (z=+0.10)
       -> gps_link (z=+0.20)
```

## Values still requiring measurement

Edit `urdf/hege_parameters.xacro` after measuring:

- chassis length, width, height and wheel width;
- individual wheel/knuckle masses or CAD inertias;
- maximum drive torque, no-load wheel speed and gear ratio;
- measured steering lock and steering lock-to-lock time;
- tyre friction/slip on the actual soil;
- IMU/GPS x-y lever arms if they are not exactly on the base_link vertical axis;
- measured command/actuator latency and drivetrain time constant.

The supplied physics values are safe starting estimates, not calibrated ground truth.
The `cmd_vel_relay.py` permits negative `linear.x`, so reverse is supported.

## Install in the ThinkPad devcontainer workspace

Keep backups outside `hege_ws/src`, otherwise colcon sees duplicate package names.
The final directory must be:

```text
/workspaces/hege_gps_navigation/hege_ws/src/hege_description
```

Build inside the container (leave `set +u` active while sourcing ROS):

```bash
cd /workspaces/hege_gps_navigation/hege_ws
set +u
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select hege_description
source install/setup.bash
export ROS_DOMAIN_ID=73
```

## RViz geometry check

```bash
ros2 launch hege_description display.launch.py
```

Check the transforms:

```bash
ros2 run tf2_ros tf2_echo base_footprint base_link
ros2 run tf2_ros tf2_echo base_link imu_link
ros2 run tf2_ros tf2_echo base_link gps_link
```

Expected z translations: `1.10`, `0.10`, `0.20` m.

For the real robot, launch static robot geometry with:

```bash
ros2 launch hege_description real_description.launch.py
```

In `hege_px4_sensors/config/sensors.yaml`, use:

```yaml
imu_frame: "imu_link"
gps_frame: "gps_link"
publish_tf: false
```

Keep `publish_tf: false` until the dynamic TF owner is designed. The future
localization owner should publish `odom -> base_footprint`; the URDF already
publishes `base_footprint -> base_link`.

## Gazebo test

Install the matching Gazebo Harmonic integration once:

```bash
sudo apt update
sudo apt install -y ros-humble-ros-gzharmonic ros-humble-gz-ros2-control \
  ros-humble-ackermann-steering-controller ros-humble-joint-state-broadcaster
```

Do not install `gazebo_ros`, `gazebo_ros2_control`, or Gazebo Classic 11 in this
container. They conflict with the already installed Gazebo Harmonic stack.

Launch:

```bash
ros2 launch hege_description gazebo.launch.py
```

In another sourced terminal, verify the integration:

```bash
ros2 control list_controllers
ros2 topic hz /imu/data
ros2 topic hz /gps/fix
```

Forward test:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.10}, angular: {z: 0.0}}"
```

Reverse test:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: -0.10}, angular: {z: 0.0}}"
```

Stop with Ctrl+C. Do not use these Gazebo topic commands on the real vehicle.

## Duplicate package warning

Only one `hege_description` package may exist under the workspace. Before
building, check:

```bash
find /workspaces/hege_gps_navigation/hege_ws/src -name package.xml -print \
  | xargs grep -l '<name>hege_description</name>'
```

If an older package exists, move it outside `src` before adding this one.
