# HEGE field scene through the existing PX4 architecture

This adds the friend's terrain, textures, four bale visuals and orange HEGE
model style to the main repository's command architecture:

`/cmd_vel/{test,teleop,nav}` -> existing `twist_mux` -> `/cmd_vel/selected`
-> existing `hege_px4_bridge` -> PX4 v1.16.1 Ackermann controller -> simulated
motor and steering actuators. Sensor feedback returns through PX4 and the
existing `hege_px4_sensors` adapter.

The field launch does not start Nav2 or a bale collection mission. It imports
the scene/design, not the friend's autonomy stack. No robot hardware is needed.
Existing real launch, bridge source, sensors, mux configuration, px4_msgs,
URDF and stock SITL launch are unchanged.

## One-time preparation, inside the existing Humble container

First open this branch in VS Code and select **Dev Containers: Rebuild Container**
(or **Reopen in Container** for a fresh clone). The Dockerfile installs
`ros-humble-twist-mux`, standalone `MicroXRCEAgent` **v2.4.3**, and
`libgz-sim8-dev` automatically. Pulling files alone does not update an existing
container image. Stop the running simulation before rebuilding the container.
An Internet connection is required for the initial image/dependency build.

The scene assets (heightmap, two textures, world file, about 2 MB) are
committed under `simulation/hege_field/assets/`; nothing has to be fetched from
another repository.

From the repository root (adjust only the PX4 checkout path if necessary):

```bash
source /opt/ros/humble/setup.bash
export HEGE_PX4_DIR=/workspaces/hege_gps_navigation/PX4-Autopilot
git -C "$HEGE_PX4_DIR" describe --tags --exact-match
# Must say v1.16.1. Do not change a different checkout automatically.
git -C "$HEGE_PX4_DIR" submodule update --init --recursive Tools/simulation/gz
make -C "$HEGE_PX4_DIR" px4_sitl

cmake -S simulation/hege_field -B .hege_sim/plugin_build
cmake --build .hege_sim/plugin_build -j2
python3 simulation/hege_field/prepare.py --px4 "$HEGE_PX4_DIR"

cd hege_ws
# Use the existing px4_msgs installation/overlay; do not copy the friend's.
# Source its install/setup.bash here if it is in a separate workspace.
colcon build --symlink-install --packages-up-to hege_bringup
source install/setup.bash
```

The preparation copies the vendored assets and generates the world and model
SDF from them. Generated files, build files, logs and PX4 parameters stay in the
ignored `.hege_sim` directory.
Preparation does not edit the PX4 checkout, install an airframe or change its
normal SITL parameter store. The ROS launcher checks dependencies before starting
processes and points to container rebuild / workspace sourcing if any are missing.

## Run: three container terminals

All commands below start at `/workspaces/hege_gps_navigation`.
Close an earlier instance of this field simulation before starting another.

Terminal 1 — field and HEGE model:

```bash
cd /workspaces/hege_gps_navigation
python3 simulation/hege_field/run.py world
```

Terminal 2 — PX4 controller (wait until the field has opened):

```bash
cd /workspaces/hege_gps_navigation
python3 simulation/hege_field/run.py px4
```

Terminal 3 — existing ROS command/feedback nodes plus simulation agent:

```bash
cd /workspaces/hege_gps_navigation
source /opt/ros/humble/setup.bash
source hege_ws/install/setup.bash
python3 simulation/hege_field/run.py ros
```

The scripts select ROS domain **74**, `ROS_LOCALHOST_ONLY=0`, XRCE UDP
port **8889** (passed to PX4 SITL as `PX4_UXRCE_DDS_PORT`, which is what its
`rcS` reads), and Gazebo partition **hege-field-sim**. The real Jetson domain 73
and agent port 8888 are separate. Do not set `ROS_LOCALHOST_ONLY=1`: the agent
ignores that variable and ROS nodes then cannot discover it. No process automatically arms or sends motion.
ROS nodes retain wall-clock time, as in the existing PX4 SITL bridge. Do not pause
or accelerate Gazebo during command/watchdog tests.

## Terminal 4 — verify and move the simulation

```bash
cd /workspaces/hege_gps_navigation
source /opt/ros/humble/setup.bash
source hege_ws/install/setup.bash
export ROS_DOMAIN_ID=74 ROS_LOCALHOST_ONLY=0
ros2 daemon stop
ros2 topic echo /hege/bridge/status --once
```

Wait for fresh PX4 state, i.e. `NOT_OFFBOARD | ... | offb=0 vel=0 armed=0`, not
`WAITING_PX4`. Then request the modes explicitly:

```bash
ros2 service call /hege/bridge/set_offboard std_srvs/srv/Trigger '{}'
ros2 service call /hege/bridge/arm std_srvs/srv/Trigger '{}'
ros2 topic echo /hege/bridge/status --once
```

Check both accepted responses and `CMD_TIMEOUT | nav_state=14 arming=2 | offb=1 vel=1 armed=1`.
If rejected, read the PX4 terminal's arming error; do not disable its checks.
The repository's scripted slow drive is
`ros2 run hege_px4_bridge step_test --ros-args -p confirm_motion_test:=true`
(0.3 m/s straight, then gentle left and right). For manual commands:

```bash
ros2 topic pub -r 20 --times 100 /cmd_vel/test geometry_msgs/msg/Twist \
  '{linear: {x: 1.0}, angular: {z: 0.0}}'
ros2 topic pub -r 20 --times 20 /cmd_vel/test geometry_msgs/msg/Twist '{}'

# A left arc; negative angular.z requests right.
ros2 topic pub -r 20 --times 100 /cmd_vel/test geometry_msgs/msg/Twist \
  '{linear: {x: 1.0}, angular: {z: 0.2}}'
ros2 topic pub -r 20 --times 20 /cmd_vel/test geometry_msgs/msg/Twist '{}'
ros2 service call /hege/bridge/disarm std_srvs/srv/Trigger '{}'
```

During motion, inspect `listener trajectory_setpoint 1`,
`listener rover_throttle_setpoint 1`, and `listener actuator_motors 1` in the
**PX4 terminal**. These distinguish ROS input from actual controller output.
No motion with nonzero PX4 outputs: inspect Gazebo for plugin load errors.
For Gazebo transport inspection use `export GZ_PARTITION=hege-field-sim` in the
inspection terminal, then `gz topic -l` and
`gz topic -e -t /model/hege_px4/command/motor_speed`.

## What is modelled, and what still needs validation

- Wheelbase 1.90 m, track 1.55 m, rear/front radii 0.40/0.275 m, mass 1300 kg
  follow the existing/friend model design. They are not new measurements.
- Virtual steering limit 0.5759 rad follows the last recorded real PX4 value.
  The steering-only Gazebo plugin converts the PX4 servo angle into unequal
  inner/outer wheel angles. It never consumes ROS cmd_vel.
- The ideal drive uses a 10:1 reduction. PX4's integer output minus 100 gives
  shaft rad/s; dividing by 10 and multiplying by 0.40 m gives wheel speed.
  Full scale is therefore 4 m/s, resolution 0.04 m/s. The command speed cap is
  separately 1 m/s. RO_MAX_THR_SPEED=4 is a **simulator-derived mapping**, not
  evidence that the real vehicle's unmeasured value is correct. PX4 retains
  its speed feedback controller; this does not map ROS speed directly to throttle.
- The wheel adapter zeros its velocity target after 0.5 simulated seconds
  without PX4 actuator messages. The original bridge watchdog is also active.
- Ideal equal rear wheel speeds, approximate inertias, no suspension and
  uncalibrated terrain friction mean this is not a validated digital twin.
- The imported bales/arrows are visuals, with no collision or collection logic.
  Navigation, obstacle detection and real-vehicle equivalence are not claimed.
- PX4's stock IMU, magnetometer, pressure and GPS sensor definitions are kept
  on base_link at their expected scoped topics. The coloured antenna/IMU pieces
  are visual markers, not separately calibrated sensor lever arms.
- Do not reuse this simulation parameter profile on the Pixhawk.

Offline checks: `python3 -m unittest discover -s simulation/hege_field/tests -v`.
Gazebo plugin compilation and runtime motion must be checked in the Humble /
Harmonic container; source checks alone cannot certify either.

## Source provenance

- Scene assets in `assets/` were copied from https://github.com/michalczaplinski5/hege-gps-navigation/tree/4170c7bea836b90559b304b609252dedfc303ffe/src/hege_description/worlds (files `hege_field.world`, `field_heightmap.pgm`, `textures/dirt_diffusespecular.png`, `textures/flat_normal.png`), unchanged.
- PX4 v1.16.1 sensor/model source submodule: https://github.com/PX4/PX4-gazebo-models/tree/e05f4312d3f28aa621157610584a4870406cb6d3
- Actuator topic and scaling contracts: PX4 v1.16.1 `GZMixingInterfaceWheel.cpp`
  and `GZMixingInterfaceServo.cpp`; attachment/startup: `px4-rc.gzsim`.
- Steering-only Double input and wheel_separation geometry: Gazebo Sim 8
  `src/systems/ackermann_steering/AckermannSteering.cc`.

The teammate's controllers, localization stack, hardware configuration and
px4_msgs are not imported.
