---
name: nav2-localization
description: Repo-specific Nav2 + localization expert for this linorobot2 robot. Use for anything touching navigation, SLAM/mapping, AMCL, EKF, rf2o laser odometry, costmaps, TF frames, /scan filtering, speed limits, or launch wiring. Knows this robot's two odometry regimes (stock EKF vs active rf2o), the TF ownership rules, the 0.05 m/s speed-cap invariant, and the lidar crop pipeline.
---

# Nav2 & Localization — linorobot2 (this repo)

Ground-truth knowledge for THIS robot. Prefer these facts over generic linorobot2
or Nav2 documentation — the robot diverges from stock linorobot2 in important ways.

## The robot (hardware facts)

- 550mm x 550mm square chassis, footprint centered on `base_footprint`
- AK10-9 motors driven by a **Nucleo-H753ZI** (migrated from Teensy 4.1), firmware in `firmware/`
- micro-ROS over serial: `/dev/ttyACM0` (ST-LINK VCP) @ **921600** baud — must match firmware
- **RPLIDAR A3** on `/dev/rplidar` @ 256000 baud, mounted **185mm forward** of the rotation
  center with its 0° axis facing the robot's **REAR** (hence the 180° yaw in the
  `base_footprint -> laser` static TF in `linorobot2_bringup/launch/robot.launch.py`)
- MPU6050 IMU -> `/imu/data` (topic only, never a TF)
- No usable wheel encoders in the active setup — odometry comes from laser scan matching

## ⚠️ Two odometry regimes — never mix them

This repo contains TWO parallel bringup architectures that both want to own the
`odom -> base_footprint` TF. Running both = TF fight = localization garbage.

| | ACTIVE: `robot.launch.py` | STOCK: `bringup.launch.py` |
|---|---|---|
| File | `linorobot2_bringup/launch/robot.launch.py` | `linorobot2_bringup/launch/bringup.launch.py` |
| `odom->base_footprint` owner | **rf2o_laser_odometry** (scan matching, 10 Hz) | **EKF** (`robot_localization`, `linorobot2_base/config/ekf.yaml`) |
| Firmware `odom/unfiltered` | topic only, ignored for TF | fused by EKF (vx, vy, vyaw) |
| `/imu/data` | topic only (`publish_odom_tf:=false`) | fused by EKF (vyaw only) |
| Status | actively developed / on-robot | legacy upstream path |

When editing localization, first establish which regime the user is running.
Recent work (lidar crop, alignment fixes, autonomous driving) is all on the
`robot.launch.py` + rf2o path.

## TF ownership map

```
map -> odom              slam_toolbox (mapping)  OR  AMCL (navigation on saved map)
odom -> base_footprint   rf2o (active)  OR  EKF (stock)  — exactly ONE
base_footprint -> laser  static TF: x=0.185, yaw=pi   (robot.launch.py)
```

- `robot.launch.py slam:=true` (default) = slam_toolbox owns `map->odom` (mapping).
- Set `slam:=false` when running `navigation.launch.py`, where AMCL owns `map->odom`.
- AMCL takes its initial pose from `/initialpose` (topic; nav2_bringup has no launch
  arg for it) — use the map viewer's "set robot pose" button or `ros2 topic pub`.

## Scan pipeline (and its trap)

```
RPLIDAR A3 --360°--> /scan_raw
angle_laser_filter --keep front ±90°--> /scan     (rear half looks at the battery)
```

Config: `linorobot2_bringup/config/angle_laser_filter.yaml` — two chained filters:
1. `LaserScanAngularBoundsFilterInPlace` REMOVES −90°..+90° (the rear/battery span).
   InPlace is used because the kept half straddles the ±180° wraparound.
2. `LaserScanRangeFilter` replaces the InPlace filler (`range_max + 1`, a FINITE
   value) with `inf`. **Do not remove this**: rf2o copies ranges without checking
   `range_max`, treats the filler as a rigid 26 m arc, and its eigensolver fails
   with "Pose is not updated" while driving.

Everything downstream (rf2o, slam_toolbox, both costmaps, collision monitor,
`lidar_alignment_check.py` at repo root) consumes the cropped `/scan`.

## 🚫 Speed-cap invariant (0.05 m/s)

Autonomous linear speed is capped at **0.05 m/s** (drivetrain protection),
angular at **0.25 rad/s**, enforced in `linorobot2_navigation/config/navigation.yaml`
in **multiple places that must change together**:

- `controller_server.FollowPath.desired_linear_vel: 0.05` (RPP)
- `velocity_smoother.max_velocity: [0.05, 0.0, 0.25]` — the final gate before `/cmd_vel`
- `behavior_server.max_rotational_vel: 0.25` (recovery spins/backups)
- RotationShim `rotate_to_heading_angular_vel: 0.25`

If asked to change speed: update ALL of them, and say so. The velocity smoother
wins regardless of what upstream commands.

## File map — what controls what

| Concern | File |
|---|---|
| Nav2 everything (AMCL, controller, costmaps, planner, behaviors, smoother, collision monitor, docking) | `linorobot2_navigation/config/navigation.yaml` |
| slam_toolbox tuning (Ceres, mapping mode, loop closure) | `linorobot2_navigation/config/slam.yaml` |
| EKF fusion matrix (stock path only) | `linorobot2_base/config/ekf.yaml` |
| Lidar crop + inf replacement | `linorobot2_bringup/config/angle_laser_filter.yaml` |
| Active bringup: lidar, filter, IMU, rf2o, static TFs, micro-ROS agent, web map viewer (port 8000) | `linorobot2_bringup/launch/robot.launch.py` |
| Stock bringup: EKF, madgwick (optional), firmware serial | `linorobot2_bringup/launch/bringup.launch.py` |
| Mapping session: Nav2 (navigation_launch) + slam_toolbox online_async | `linorobot2_navigation/launch/slam.launch.py` |
| Navigation on saved map: nav2_bringup `bringup_launch.py` + AMCL; `MAP_NAME` constant at top | `linorobot2_navigation/launch/navigation.launch.py` |
| Saved maps | `linorobot2_navigation/maps/`, plus `apartment_map_1.{pgm,yaml}` at repo root |
| Lidar mount verification | `lidar_alignment_check.py` (repo root) |

## Key tuned values (don't "fix" back to defaults)

- **slam_toolbox** (`slam.yaml`): `map_update_interval: 0.5` (fast),
  `minimum_travel_distance/heading: 0.1` — deliberately low because rf2o motion is
  the only odometry; `max_laser_range: 10.0`; `mode: mapping` (flip to
  `localization` for localization-only sessions).
- **AMCL**: `DifferentialMotionModel`, likelihood field, 500–2000 particles,
  `transform_tolerance: 1.0` (generous on purpose — Jetson load).
- **Controller**: RotationShim wrapping RegulatedPurePursuit; `lookahead_dist: 0.6`
  (0.3–0.9); goal tolerance 0.35 m / 0.35 rad; collision detection on.
- **Costmaps**: 0.05 m resolution; local = 3x3 m rolling voxel layer; inflation
  radius 0.70, cost scaling 3.0; obstacle/raytrace range 2.5/3.0 m.
- **rf2o**: 10 Hz, `/odom_rf2o`, `publish_tf: true`, works on 180° cropped scans.

## Operational notes

- `teleop_keyboard.py` (repo root) needs its **own terminal/TTY** — reads raw keystrokes.
- Live map without rviz: `robot.launch.py` serves http://<robot-ip>:8000
  (`map_viewer.py`, default path `/home/jetson1/Desktop/map_viewer.py`).
- Firmware side of `/cmd_vel` is `controlCallback` in `firmware/src/main.cpp`;
  firmware questions belong to the `stm32-freertos-developer` skill.

## Debugging checklist

1. **Robot pose jumps / map smears** → two publishers on `odom->base_footprint`?
   `ros2 run tf2_ros tf2_echo odom base_footprint` + check running nodes. Usually
   stock EKF (or IMU `publish_odom_tf`) running alongside rf2o.
2. **rf2o "Pose is not updated"** → the range filter (filter2) got dropped or
   reordered; cropped points must be `inf`, not `range_max + 1`.
3. **slam_toolbox never adds scans** → rf2o dead or scan starved; it must see
   `minimum_travel_distance: 0.1` of motion from the odom TF.
4. **Robot won't move autonomously** → velocity smoother caps at 0.05 m/s; check
   micro-ROS agent is up (serial 921600) and `/cmd_vel` reaches the firmware.
5. **AMCL diverges** → wrong initial pose (`/initialpose`), or mapping-mode
   slam_toolbox still running and fighting AMCL for `map->odom` (`slam:=false`).
6. **Obstacles ghost/persist in costmap** → check the crop: rear 180° is blind by
   design; raytrace clearing only works inside 3.0 m.

## Rules

- After editing configs/launch files, run `graphify update .` (repo rule).
- TF frame renames must be checked across ALL of: `ekf.yaml`, `slam.yaml`,
  `navigation.yaml` (many nodes), `robot.launch.py` static TFs, and the firmware's
  odom frame strings. The base frame is `base_footprint` everywhere (not `base_link`).
- For SLAM algorithm theory/tuning beyond this robot's config, defer to the
  `slam-algorithms` skill; for firmware/motor issues, `stm32-freertos-developer`.
