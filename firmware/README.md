# linorobot2 Nucleo-H753ZI firmware — AK10-9 (MIT mode)

micro-ROS firmware that turns an **ST Nucleo-H753ZI** (STM32H753ZI) into a
linorobot2 base controller for two **CubeMars AK10-9** motors driven in
**MIT mode over FDCAN1**, with an optional **MPU6050** IMU.

> Migrated from the original Teensy 4.1 base (git history has the old
> version). Same control logic and topic contract; only the board, the CAN
> backend (FlexCAN_T4 → STM32_CAN/FDCAN), and the flash/serial plumbing
> changed.

| Topic | Direction | Type | Purpose |
|-------|-----------|------|---------|
| `cmd_vel` | subscribe | `geometry_msgs/Twist` | velocity commands (Nav2 / teleop) |
| `odom/unfiltered` | publish | `nav_msgs/Odometry` | wheel odometry from motor feedback (for SLAM) |
| `imu/data` | publish | `sensor_msgs/Imu` | raw IMU (only if an MPU6050 is wired) |
| `motor_current` | publish | `std_msgs/Float32MultiArray` | [left, right] motor current (A) |

These are exactly the topics linorobot2's `bringup.launch.py` expects, so once
flashed the Nucleo joins the ROS 2 graph automatically.

## Wiring

- **CAN (FDCAN1)**: `PD1` = CAN_TX → transceiver TXD, `PD0` = CAN_RX ←
  transceiver RXD. Use a 3.3 V CAN transceiver (SN65HVD230, TJA1051T/3, …) to
  the motors at **1 Mbps**, 120 Ω termination at each end of the bus, ground
  common with the motor controllers. Both pins are on the Zio/morpho headers.
  (PA11/PA12, the alternate FDCAN1 pins, are not used — they belong to USB.)
- **Motors**: left = controller ID `0x68` (cmd `0x868`), right = `0x69`
  (cmd `0x869`).
- **MPU6050** (optional): SDA = `D14` (PB9), SCL = `D15` (PB8), 3.3 V — the
  Arduino-header I2C, which is the default `Wire` on this board.
- **USB**: the single ST-LINK USB cable (CN1) carries **both** flashing and the
  micro-ROS serial link (ST-LINK Virtual COM Port = USART3 on PD8/PD9, wired
  on-board; nothing to connect).
- **Status LED**: LD1 (green) — on when connected to the agent.

## Configure before flashing — `include/config.h`

1. `WHEEL_DIAMETER` and `LR_WHEELS_DISTANCE` — **measure these on your robot.**
   They set the odometry scale; wrong values = drifting SLAM maps.
2. `LEFT_MOTOR_DIR` / `RIGHT_MOTOR_DIR` — flip a sign if a wheel spins the wrong
   way (mirrored drivetrains usually have one side negated; defaults: left `+1`,
   right `-1`).
3. `SPIN_KP / SPIN_KD / SPIN_TORQUE_FF` — if low-speed motion is jerky during
   SLAM, lower `SPIN_TORQUE_FF`.

## Host setup (one time)

This was set up on the Jetson (Ubuntu 24.04, ROS 2 Jazzy). Two host-side things
are needed before you can build/flash: **PlatformIO** and the **ST-LINK udev
rules**.

### 1. PlatformIO

Ubuntu 24.04's system Python is "externally managed" (PEP 668), so install
PlatformIO into its **own virtualenv** and always call `pio` by full path:

```bash
# python3.12-venv is not installed and needs sudo, so create the venv without
# pip and bootstrap pip into it manually:
python3 -m venv --without-pip ~/.platformio/penv
curl -fsSL https://bootstrap.pypa.io/get-pip.py | ~/.platformio/penv/bin/python
~/.platformio/penv/bin/python -m pip install platformio
```

From here on use **`~/.platformio/penv/bin/pio`** (a bare `pio` is not on PATH).
Installing into `~/.platformio/penv` matters: micro-ROS's build sources that
venv (`penv/bin/activate`) to install its own Python helpers.

### 2. ST-LINK udev rules (needed to flash without sudo)

PlatformIO's udev rules cover the Nucleo's onboard ST-LINK V3E (VID:PID
`0483:374e`):

```bash
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules \
  | sudo tee /etc/udev/rules.d/99-platformio-udev.rules > /dev/null
sudo udevadm control --reload-rules && sudo udevadm trigger
# replug the board; make sure your user is in the dialout group for /dev/ttyACM*
```

## Build & flash (directly to the STM32)

```bash
cd firmware
~/.platformio/penv/bin/pio run -e nucleo_h753zi -t upload
```

This flashes **directly over the onboard ST-LINK** (OpenOCD, bundled with
PlatformIO): no bootloader button, works even while the old firmware is
running. Stop the micro-ROS agent before flashing — it shares the same USB
cable (different USB interface, but a half-open session across the reset is
just noise).

Alternatives:
- **Drag-and-drop**: copy `.pio/build/nucleo_h753zi/firmware.bin` onto the
  `NOD_H753ZI` USB mass-storage drive the Nucleo exposes.
- **Debug**: `pio debug -e nucleo_h753zi` works out of the box
  (`debug_tool = stlink`).

The first build downloads micro-ROS for the `jazzy` distro
(`board_microros_distro` in `platformio.ini`) — **this must match the ROS 2
distro on your robot computer.**

Notes:
- **After changing any `build_flags`** run `~/.platformio/penv/bin/pio run -e
  nucleo_h753zi -t clean_microros` before rebuilding, or the micro-ROS CMake
  cache keeps the old flags.
- After flashing, the ST-LINK VCP appears as `/dev/ttyACM0` — the same device
  the agent uses. If other CDC-ACM devices (lidar, …) shuffle the numbering,
  add a udev symlink rule keyed on the ST-LINK serial number.

### Serial transport: real baud now

Unlike the Teensy (USB CDC, baud nominal), the ST-LINK VCP is a **real
921600-baud UART** — both the firmware (`Serial.begin(921600)`) and the agent
(`micro_ros_baudrate`, default 921600 in `robot.launch.py`) must agree.
Bandwidth check: odom + IMU + current at 50 Hz is ~60 KB/s vs ~92 KB/s
available at 921600 — fits. If you ever need more headroom, raise the baud on
both sides (ST-LINK V3E goes well beyond 921600), or switch the firmware to
the H753ZI's native USB port (CN13) by building with
`-DUSBCON -DUSBD_USE_CDC`.

## Run it with linorobot2

You also need the **micro-ROS agent** on the robot computer (the host side of the
serial bridge). Build it natively once (no Docker) in your linorobot2 workspace:

```bash
cd ~/linorobot2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run micro_ros_setup create_agent_ws.sh    # clones the agent sources
#  ^ the rosdep step may abort on unrelated workspace packages — that's fine,
#    the sources are already cloned. Build the agent directly:
colcon build --packages-up-to micro_ros_agent --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Then bring everything up:

```bash
cd ~/linorobot2_ws && source install/setup.bash

# Terminal 1 — micro-ROS agent (defaults to /dev/ttyACM0 @ 921600) + base
ros2 launch linorobot2_bringup bringup.launch.py
# wait for: "session established" and LD1 (green) turning ON

# Terminal 2 — drive it
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 3 — map
ros2 launch linorobot2_navigation slam.launch.py
```

> **Put the robot on blocks for the first drive** until `config.h` geometry and
> the `LEFT/RIGHT_MOTOR_DIR` signs are verified (see "Verify odometry" below).

LD1 (green) is **on** when connected to the agent, **off** otherwise. The
motors are actively held at zero whenever the agent is disconnected or `cmd_vel`
goes silent for `CMD_VEL_TIMEOUT_MS`.

## Verify odometry (do this before trusting SLAM)

```bash
ros2 topic echo /odom/unfiltered
```

- **No messages / all zeros while wheels turn** → the firmware isn't decoding
  motor feedback. First suspect on this board: the FDCAN RX filter dropping the
  motors' extended-ID status frames — see the note in `ak10_mit.h::ak10Begin()`.
  Then see "MIT feedback" below.
- **Rate check**: `ros2 topic hz /odom/unfiltered` should hold ≈50 Hz. If it
  stutters, the VCP link is saturating — raise the baud on both sides.
- **Scale check**: push (or drive) the robot exactly 1 m forward and confirm
  `pose.pose.position.x` ≈ 1.0. If it reads e.g. 0.5 or 2.0, fix
  `WHEEL_DIAMETER`. Spin in place 360° and confirm yaw returns to start; if not,
  fix `LR_WHEELS_DISTANCE`.

## MIT-mode notes & caveats

- **Velocity is at the output shaft (rad/s).** Unlike servo/ERPM mode, MIT mode
  needs no gear-ratio or pole-pair conversion — wheel angular velocity maps 1:1
  to the MIT `v_des` and feedback velocity. (Assumes the wheel sits directly on
  the output shaft.)
- **Feedback decoding** (`ak10_mit.h::handleFrame`) assumes the servo-mode
  status frame layout described in `config.h`. Some CubeMars firmware versions
  differ slightly. If `/odom/unfiltered` stays zero and the RX filter is ruled
  out, dump incoming frames while a wheel turns and adjust the byte layout in
  `handleFrame` to match.
- **Enter motor mode**: the firmware sends the MIT "enter motor mode" frame on
  boot (`ak10EnterMotorMode` in `setup()`), required for the motors to stream
  feedback.
- **FDCAN clock**: the bit timing is derived from the FDCAN kernel clock
  (reset default: HSE = 8 MHz on this Nucleo, which divides exactly to 1 Mbps).
  If the bus won't sync after a clock-config change, re-check the FDCAN kernel
  clock source before blaming the transceiver.
- This firmware does **not** depend on the linorobot2 ROS 2 packages at build
  time; it only shares their topic contract. It lives in this repo for
  convenience and version-tracking alongside the rest of your robot.
