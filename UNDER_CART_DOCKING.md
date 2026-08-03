# Under-Cart Docking — Phase 0–2 Checklist

The AGV docks by driving underneath a cart. Detection is **tagless**: a new
node (`cart_leg_detector.py`) finds the cart's four legs in the raw lidar
scan and feeds `detected_dock_pose` to the Nav2 docking server, which is
already configured with `use_external_detection_pose: true`
(`linorobot2_navigation/config/navigation.yaml`, `docking_server`).

Phases: **0** geometry gate (this checklist) → **1** leg detector →
**2** EKF odometry swap → *3 docking server bringup → 4 NFC cart identity →
5 carrying-the-cart (later)*.

---

## Phase 0 — Geometry gate (do this BEFORE trusting any software)

### 0.1 Tape-measure checklist

| # | Measurement | Pass criterion | Feeds |
|---|---|---|---|
| 1 | Cart under-deck clearance | ≥ robot max height (incl. lidar mast + cabling) + 30 mm | go/no-go |
| 2 | Leg spacing, lateral (inner edge ↔ inner edge) | ≥ 550 mm robot footprint + 2 × 50 mm margin | go/no-go |
| 3 | Leg spacing, shorter, center ↔ center | record | `rect_width` |
| 4 | Leg spacing, longer, center ↔ center | record | `rect_length` |
| 5 | Leg cross-section (width the lidar sees) & shape | record; flag if < 25 mm or chromed/shiny | `leg_diameter` |
| 6 | Lidar scan-plane height vs. leg | scan plane must hit **bare leg**, below any cross-brace/shelf | go/no-go |
| 7 | Is the leg rectangle square (length ≈ width)? | if yes, note it — pose has 90° ambiguity, detector needs the expected-pose seed | detector config |

Enter 3–5 into `linorobot2_navigation/config/cart_leg_detector.yaml`.

### 0.2 Scan-quality capture (existing stack only, no new code)

```bash
# terminal 1 — the usual stack
ros2 launch linorobot2_bringup robot.launch.py

# terminal 2 — teleop
python3 teleop_keyboard.py

# terminal 3 — record
ros2 bag record /scan_raw /scan /tf /tf_static /odom_rf2o /odom /odom/unfiltered /imu/data
```

Drive: approach from ~2 m → slow entry under the cart → hold centered ~10 s
→ exit. Repeat 3×, at least one run entering at a deliberately bad angle.

Inspect in RViz (`/scan_raw`, fixed frame `odom`):

- [ ] At 0.5–2 m, all four legs show as distinct clusters of ≥ 3 points.
- [ ] While underneath, the legs around the robot are still visible in `/scan_raw`.

### 0.3 Gate decision

- **Pass** → measurements become the detector config; the bags become the
  Phase 1 offline test fixture.
- **Sparse/flickering leg clusters** → wrap legs in matte reflective tape and
  re-record. Still bad → fall back to the camera + belly-AprilTag design
  (only the detector node changes; Phases 2–3 carry over unchanged).
- **Clearance or leg-gap failure** → hardware change (lidar remount / cart
  selection) before writing any software.

---

## Phase 1 — Leg detector validation

Run standalone (works against live scans or a Phase 0 bag):

```bash
ros2 launch linorobot2_navigation cart_leg_detector.launch.py
# offline: ros2 bag play <phase0_bag> in another terminal
```

Watch `cart_leg_detector/markers` in RViz: orange spheres = leg candidates,
green rectangle = accepted fit, blue arrow = filtered `detected_dock_pose`.

For offline bag exploration set `gate_radius: 0.0` (accept anything); for
any live use keep gating on and seed `expected_dock_*` — gating is the guard
against look-alike leg patterns (racking, chairs, a second cart).

Acceptance:

- [ ] Detection on ≥ 95 % of scans within 2 m of the cart.
- [ ] Position jitter σ ≤ 1 cm on the hold-centered bag segment.
- [ ] No orientation flips during entry (the 2–3-visible-legs case).
- [ ] Zero detections on a cart-free bag segment (with gating configured).

## Phase 2 — Odometry swap validation (rf2o → EKF)

The EKF (`robot_localization`, config `linorobot2_base/config/ekf.yaml`) now
owns `odom->base_footprint`, fusing Teensy wheel odometry
(`odom/unfiltered`) with the IMU. rf2o still publishes `/odom_rf2o`
(topic only) for comparison; disable with `rf2o:=false`.

Preflight:

- [ ] `ros2 topic info /imu/data --verbose` → exactly **one** publisher (the
  Jetson `mpu6050_imu` node). The Teensy firmware also publishes `imu/data`
  if it finds an MPU6050 on its own bus — two publishers corrupts the EKF.
- [ ] Push the robot ~1 m by hand **with the micro-ROS agent running**
  (AK10-9s back-drive, encoders still count): `ros2 topic echo /odom/unfiltered`
  tracks direction and roughly the distance.

Regression:

- [ ] `ros2 run tf2_tools view_frames` → `odom->base_footprint` published
  only by `ekf_filter_node`; `map->odom` by slam_toolbox (or AMCL in nav mode).
- [ ] 2 m straight line: `/odom` x error < 2 % (tape measure).
- [ ] 2 × 360° spin in place: yaw error < 5°.
- [ ] Re-map a known area: map quality ≥ the existing baseline map.
- [ ] **The money test**: teleop under the cart while echoing `/odom` — pose
  stays smooth where rf2o (`/odom_rf2o`, still recorded) glitches.

Known caveat: with `micro_ros:=false` there is no wheel odometry, the EKF
has no input, and the odom TF goes stale — keep the agent running even for
hand-push sessions.
