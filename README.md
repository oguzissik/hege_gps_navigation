# HEGE GPS Navigation and PX4 Rover Stack

ROS 2 Humble packages and a reproducible development environment for the **HEGE Ackermann field rover**. The project was built from scratch around a Jetson companion computer, a Pixhawk running PX4 Rover, a DroneCAN GNSS receiver, and a custom Gazebo Harmonic vehicle model.

The repository contains:

- a measured URDF/Xacro model of the rover;
- RViz visualization and TF geometry;
- a Gazebo Harmonic simulation using `gz_ros2_control`;
- an Ackermann steering controller and `/cmd_vel` relay;
- a ROS 2–PX4 Offboard bridge with watchdogs and command acknowledgement handling;
- PX4 GPS, IMU and odometry conversion to standard ROS 2 messages;
- separate real-vehicle and PX4 SITL configurations;
- a devcontainer pinned to ROS 2 Humble and the matching external dependencies.

> **Scope:** The custom rover simulation, ROS/PX4 communication, sensor conversion, command arbitration and real-hardware data path have been exercised. This repository does **not** present GPS waypoint navigation, Nav2 coverage planning, RTK Fixed operation, or autonomous real-vehicle motion as completed results. `nav2_ackermann_hints.yaml` is a geometry-aware starting point, not a complete Nav2 deployment.

## System architecture

```mermaid
flowchart LR
    RC[RC / QGroundControl] --> PX4[Pixhawk PX4 Rover]
    GNSS[HERE3 DroneCAN GNSS] --> PX4
    PX4 <-->|uXRCE-DDS / MAVLink| J[Jetson Orin Nano]
    J --> B[ROS 2 HEGE bridge]
    B --> M[twist_mux]
    M --> T[teleop / test / navigation commands]
```

The real-vehicle setup keeps the control computer and flight controller responsibilities separate:

- **Pixhawk / PX4:** vehicle state estimation, rover control and actuator interface;
- **Jetson:** ROS 2 bridge, command arbitration, standard sensor topics and higher-level autonomy software;
- **ThinkPad workstation:** development, RViz, Gazebo Harmonic, PX4 SITL and QGroundControl.

## Verified project status

| Component | Status |
|---|---|
| ROS 2 Humble devcontainer | Working |
| Measured HEGE URDF/Xacro and RViz display | Working |
| Static TF geometry | Working |
| Gazebo Harmonic custom rover spawn | Working |
| `gz_ros2_control` controller manager | Working |
| Joint-state broadcaster | Working |
| Ackermann steering controller | Working |
| Simulated odometry and joint states | Working |
| Jetson–Pixhawk ROS 2 topic connection | Working |
| PX4 Offboard heartbeat at 20 Hz | Working |
| PX4 attitude and GNSS data reception | Working |
| ROS 2 GPS, IMU and odometry conversion | Working |
| HERE3 correction path and RTK Float outdoors | Working |
| Nav2 Ackermann parameter hints | Included as reference only |

## Vehicle parameters

The model uses measurements taken from the real vehicle.

| Parameter | Value |
|---|---:|
| Steering geometry | Ackermann |
| Wheelbase | 1.90–1.91 m |
| Track width | 1.55 m |
| Rear wheel radius | 0.40 m |
| Front wheel radius | 0.28 m |
| Maximum steering angle | approximately 35° |
| Approximate total mass | 1300 kg |
| `base_link` height | 1.10 m |
| IMU height | 1.20 m |
| GNSS antenna height | 1.30 m |

Using the bicycle model, the geometric minimum turning radius is

```text
R_min = L / tan(delta_max)
      = 1.91 / tan(35 deg)
      ≈ 2.73 m
```

The operational turning radius may be larger because the first real tests use conservative velocity and yaw-rate limits.

## Repository structure

```text
.
├── .devcontainer/                 ROS 2 Humble development container
├── hege_ws/src/
│   ├── hege_description/          URDF, RViz, Gazebo world and controllers
│   ├── hege_px4_bridge/           Ackermann/PX4 command bridge and safety logic
│   ├── hege_px4_sensors/          PX4 to standard ROS 2 sensor conversion
│   └── hege_bringup/              Real and simulation launch files
├── scripts/                       Interface inventory and rosbag helpers
├── EXTERNAL_DEPS.txt              Pinned external dependency revisions
├── backups/                       Older Gazebo Classic description backup
└── hege_urdf_import/              Original description import
```

The active packages are under `hege_ws/src`. The `backups` and `hege_urdf_import` directories are retained for development history and must not be copied into an active colcon `src` directory.

## Main ROS 2 packages

### `hege_description`

- measured vehicle geometry;
- `base_footprint`, `base_link`, wheel, steering, IMU and GPS links;
- RViz display launch;
- Gazebo Harmonic world and spawn launch;
- `gz_ros2_control` Ackermann controller configuration;
- `/cmd_vel` relay for the standalone custom simulation.

### `hege_px4_bridge`

- subscribes to `/cmd_vel/selected`;
- converts ROS linear/angular velocity commands to the PX4 rover setpoint convention;
- publishes PX4 Offboard heartbeat and trajectory setpoints at 20 Hz;
- applies command timeout, speed, yaw-rate and steering constraints;
- supports software stop and neutral fallback;
- exposes supervised Arm, Disarm and Offboard services;
- waits for PX4 `VehicleCommandAck` before reporting command acceptance;
- has separate real and SITL parameter files.

### `hege_px4_sensors`

- converts PX4 NED/FRD data to ROS ENU/FLU conventions;
- publishes:
  - `/px4/odom`
  - `/px4/imu`
  - `/px4/gps/fix`
- preserves PX4 timestamps when the agent clock is synchronized;
- uses conservative covariance values when PX4 variance data is invalid.

### `hege_bringup`

- starts `twist_mux`;
- starts the PX4 bridge and sensor conversion node;
- optionally starts Micro XRCE-DDS Agent;
- provides separate `real.launch.py` and `sim.launch.py` files;
- includes Ackermann-aware Nav2 parameter hints.

## Prerequisites

- Ubuntu 22.04 inside the supplied devcontainer;
- ROS 2 Humble;
- Gazebo Harmonic;
- PX4-Autopilot v1.16.1;
- `px4_msgs` release/1.16;
- Micro XRCE-DDS Agent;
- `gz_ros2_control` for ROS 2 Humble;
- `ackermann_steering_controller`;
- `twist_mux`.

Pinned revisions are listed in [`EXTERNAL_DEPS.txt`](EXTERNAL_DEPS.txt).

## Development container

Clone the repository and open it with VS Code Dev Containers:

```bash
git clone https://github.com/oguzissik/hege_gps_navigation.git
cd hege_gps_navigation
code .
```

In VS Code, run **Dev Containers: Reopen in Container**.

After the container opens:

```bash
cd /workspaces/hege_gps_navigation/hege_ws
set +u
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`set +u` is used because some ROS setup scripts reference variables that may not yet exist when the shell has `nounset` enabled.

## RViz model check

```bash
cd /workspaces/hege_gps_navigation/hege_ws
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch hege_description display.launch.py
```

In another sourced terminal:

```bash
ros2 run tf2_ros tf2_echo base_footprint base_link
ros2 run tf2_ros tf2_echo base_link imu_link
ros2 run tf2_ros tf2_echo base_link gps_link
```

Expected vertical translations:

- `base_footprint -> base_link`: `1.10 m`
- `base_link -> imu_link`: `0.10 m`
- `base_link -> gps_link`: `0.20 m`

## Custom Gazebo Harmonic simulation

This launch runs the measured HEGE model with Gazebo Harmonic and `gz_ros2_control`.

Source the workspaces:

```bash
set +u
source /opt/ros/humble/setup.bash
source /workspaces/hege_gps_navigation/gz_ros2_control_ws/install/setup.bash
source /workspaces/hege_gps_navigation/hege_ws/install/setup.bash

export ROS_DOMAIN_ID=74
export ROS_LOCALHOST_ONLY=1

GZ_CONTROL_PREFIX="$(ros2 pkg prefix gz_ros2_control)"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$GZ_CONTROL_PREFIX/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
```

Launch the custom rover:

```bash
ros2 launch hege_description gazebo.launch.py
```

Verify the controllers:

```bash
ros2 control list_controllers
```

Expected active controllers:

```text
joint_state_broadcaster
ackermann_steering_controller
```

Inspect simulation outputs:

```bash
ros2 topic echo /ackermann_steering_controller/odometry --once
ros2 topic echo /joint_states --once
```

Send a slow standalone simulation command:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10}, angular: {z: 0.05}}"
```

Stop with `Ctrl+C`. Do not use this command on the real vehicle.

## PX4 SITL bridge

The PX4 bridge simulation uses PX4's Ackermann rover model. Start PX4 SITL separately:

```bash
cd /workspaces/hege_gps_navigation/PX4-Autopilot
make px4_sitl gz_rover_ackermann
```

In a second sourced terminal:

```bash
cd /workspaces/hege_gps_navigation/hege_ws
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch hege_bringup sim.launch.py
```

Optional scripted low-speed SITL command sequence:

```bash
ros2 launch hege_bringup sim.launch.py step_test:=true
```

The custom HEGE Gazebo model and the PX4 SITL rover are currently two separate validation paths. The repository does not claim that the custom HEGE model is already driven by PX4 SITL in one combined simulation.

## Real rover connection

The tested real-hardware topology is:

```text
ThinkPad / QGroundControl
        |
        | Wi-Fi / LAN
        v
Jetson Orin Nano (ROS 2 Humble)
        |
        | UART / Ethernet data links
        v
Pixhawk V6X (PX4 Rover v1.16.1)
        |
        | DroneCAN CAN1
        v
HERE3 GNSS
```

Connection roles used during development:

| Interface | Value |
|---|---|
| ROS domain | `73` |
| UART baud rate | `921600` |
| Micro XRCE-DDS UDP port | `8888` |
| GNSS bus | DroneCAN CAN1 |

SSH into the Jetson:

```bash
ssh <jetson-user>@<jetson-ip>
```

Source the Jetson workspaces:

```bash
set +u
source /opt/ros/humble/setup.bash
source "$HOME/ws_others/install/setup.bash"
source "$HOME/hege_ws/install/setup.bash"
export ROS_DOMAIN_ID=73
export ROS_LOCALHOST_ONLY=0
```

Start the real bridge when Micro XRCE-DDS Agent is already running:

```bash
ros2 launch hege_bringup real.launch.py start_agent:=false
```

Check the connection from a second sourced Jetson terminal:

```bash
ros2 node list
ros2 topic hz /fmu/out/vehicle_attitude
ros2 topic hz /fmu/in/offboard_control_mode
ros2 topic echo /fmu/out/vehicle_gps_position --once
ros2 topic echo /hege/bridge/status --once
```

Observed rates during the hardware test:

- PX4 attitude: approximately `100 Hz`
- Offboard heartbeat: approximately `20 Hz`
- GNSS: approximately `5 Hz`

## Standard ROS sensor topics

With `hege_px4_sensors` running:

```bash
ros2 topic echo /px4/odom --once
ros2 topic echo /px4/imu --once
ros2 topic echo /px4/gps/fix --once
```

`/px4/odom` is PX4 EKF output. It is not independent wheel odometry and should not be fused again as if it were an unrelated measurement.

## Command arbitration

`twist_mux` selects one command source:

| Topic | Priority | Timeout |
|---|---:|---:|
| `/cmd_vel/teleop` | 100 | 0.25 s |
| `/cmd_vel/test` | 50 | 0.25 s |
| `/cmd_vel/nav` | 10 | 0.50 s |

Selected output:

```bash
ros2 topic echo /cmd_vel/selected
```

The bridge publishes neutral commands when the selected input becomes stale.

## Bridge status and supervised services

```bash
ros2 topic echo /hege/bridge/status --once
ros2 service list | grep '/hege/bridge'
```

Services provided by the bridge:

```text
/hege/bridge/set_offboard
/hege/bridge/arm
/hege/bridge/disarm
/hege/bridge/clear_software_stop
```

Real-hardware configuration defaults to:

```yaml
allow_remote_vehicle_commands: false
```

This keeps Arm and mode changes under RC/QGroundControl control. Remote service calls are intended only for a supervised acceptance test with the vehicle secured and the emergency stop available.

## GNSS and RTK observations

The tested correction path was:

```text
NTRIP corrections -> Jetson -> MAVLink GPS_RTCM_DATA
-> Pixhawk -> DroneCAN -> HERE3
```

The PX4 console showed incoming RTCM data and an increasing DroneCAN RTCM forwarding counter. Outdoors, HERE3 reached **RTK Float** (`fix_type=5`) with approximately `0.22 m` horizontal and vertical error estimates during the recorded test.

The NTRIP client and credentials are intentionally not stored in this public repository.

Useful PX4 MAVLink Console commands:

```text
listener gps_inject_data 5
listener sensor_gps 20
uavcan status
listener vehicle_local_position 5
listener estimator_status_flags 5
listener failsafe_flags 5
```

GPS fix types reported by PX4:

| Value | Meaning |
|---:|---|
| 3 | 3D GPS |
| 4 | Differential solution |
| 5 | RTK Float |
| 6 | RTK Fixed |

## Safety notes

The real platform is approximately 1300 kg. Before any motion test:

1. Keep the rover disarmed until all checks pass.
2. Use lifted driven wheels or a clear controlled area.
3. Keep the physical emergency stop and RC takeover ready.
4. Confirm the bridge reports a neutral command.
5. Confirm PX4 local velocity and heading estimates are valid.
6. Start with the configured `0.3 m/s` speed limit.
7. Disarm before stopping bridge processes.

The vehicle has no lidar, radar or depth camera in this project configuration. It cannot detect or avoid unexpected people, animals, vehicles or objects using this software stack alone.

## Troubleshooting

### `px4_msgs` message type is invalid

Re-source the overlays and restart ROS discovery:

```bash
set +u
source /opt/ros/humble/setup.bash
source "$HOME/ws_others/install/setup.bash"
source "$HOME/hege_ws/install/setup.bash"
export ROS_DOMAIN_ID=73

ros2 daemon stop
ros2 daemon start

ros2 interface show px4_msgs/msg/SensorGps
```

### Duplicate package names

```bash
find hege_ws/src -name package.xml -print \
  | xargs grep -h '<name>' | sort | uniq -d
```

Only one copy of each ROS package may remain inside the active workspace.

### Controller manager is unavailable

Confirm that the Harmonic control overlay was sourced before the HEGE workspace:

```bash
source /opt/ros/humble/setup.bash
source /workspaces/hege_gps_navigation/gz_ros2_control_ws/install/setup.bash
source /workspaces/hege_gps_navigation/hege_ws/install/setup.bash
```

Then check:

```bash
ros2 node list | grep controller
ros2 control list_controllers
```

## Acknowledgement

Developed during the TU Berlin ENHANCE Field Robotics Bootcamp as a from-scratch integration of ROS 2, PX4, Gazebo Harmonic, Jetson and Pixhawk for an agricultural Ackermann rover.
