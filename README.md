# 🤖 Maze Solver Robot — Full Project Documentation

> An autonomous maze-solving robot built on **ROS 2 Humble** using the **linorobot2** platform. The robot uses frontier-based exploration combined with SLAM and Nav2 to autonomously map and navigate through unknown environments without any pre-loaded map.

---

## 📑 Table of Contents

1. [Project Overview](#1-project-overview)
2. [System Architecture](#2-system-architecture)
3. [Project Structure](#3-project-structure)
4. [Workspace 1 — linorobot2_hardware (ESP32 Firmware)](#4-workspace-1--linorobot2_hardware-esp32-firmware)
5. [Workspace 2 — ldlidar_ros2_ws (LiDAR Driver)](#5-workspace-2--ldlidar_ros2_ws-lidar-driver)
6. [Workspace 3 — linorobot2_ws (Visualization)](#6-workspace-3--linorobot2_ws-visualization)
7. [Workspace 4 — maze_robot_ws (Core Maze Solver)](#7-workspace-4--maze_robot_ws-core-maze-solver)
8. [Frontier Exploration Algorithm — Deep Dive](#8-frontier-exploration-algorithm--deep-dive)
9. [SLAM Configuration](#9-slam-configuration)
10. [Nav2 Navigation Stack](#10-nav2-navigation-stack)
11. [ROS 2 Topics, Services & Actions](#11-ros-2-topics-services--actions)
12. [TF Tree (Coordinate Frames)](#12-tf-tree-coordinate-frames)
13. [How to Build & Run](#13-how-to-build--run)
14. [Troubleshooting](#14-troubleshooting)
15. [Technologies & Dependencies](#15-technologies--dependencies)

---

## 1. Project Overview

This project implements a **fully autonomous maze-solving robot** using a differential-drive mobile base. The robot requires zero prior knowledge of the environment. Starting from any position, it:

1. Scans its surroundings using a 2D LiDAR sensor
2. Builds a real-time map using **SLAM Toolbox**
3. Detects unexplored regions (**frontiers**) at the boundary between known and unknown space
4. Autonomously navigates to those frontiers using **Nav2**
5. Repeats until the entire environment (maze) is mapped and no more frontiers exist

The exploration terminates automatically after 30 consecutive seconds with no new frontiers, signaling complete coverage of the maze.

---

## 2. System Architecture

```
╔══════════════════════════════════════════════════════════════════════╗
║                         PHYSICAL HARDWARE                           ║
║                                                                      ║
║  ┌─────────────────┐    Serial/USB    ┌──────────────────────────┐  ║
║  │  ESP32 (micro-  │ ◄──────────────► │   Raspberry Pi / PC      │  ║
║  │  ROS firmware)  │                  │   (ROS 2 Humble Host)    │  ║
║  │                 │                  └──────────────────────────┘  ║
║  │ ┌─────────────┐ │                           ▲                    ║
║  │ │ Motor Driver│ │                           │ USB                ║
║  │ │ (Generic 2IN│ │                  ┌────────┴──────────┐        ║
║  │ │  H-Bridge)  │ │                  │  LD19 LiDAR       │        ║
║  │ └──────┬──────┘ │                  │  (230400 baud)    │        ║
║  │        │ PWM    │                  └───────────────────┘        ║
║  │  Left  │  Right │                                               ║
║  │ Motor  │ Motor  │                                               ║
║  │        │        │                                               ║
║  │ ┌──────┴──────┐ │                                               ║
║  │ │  Encoders   │ │                                               ║
║  │ │ (1400 CPR)  │ │                                               ║
║  │ └─────────────┘ │                                               ║
║  │                 │                                               ║
║  │ ┌─────────────┐ │                                               ║
║  │ │  MPU6050    │ │                                               ║
║  │ │  (I2C IMU)  │ │                                               ║
║  │ └─────────────┘ │                                               ║
║  └─────────────────┘                                               ║
╚══════════════════════════════════════════════════════════════════════╝

╔══════════════════════════════════════════════════════════════════════╗
║                         ROS 2 SOFTWARE STACK                        ║
║                                                                      ║
║  /scan (LaserScan)     /odom (Odometry)    /imu/data_raw            ║
║       │                     │                    │                  ║
║       ▼                     ▼                    │                  ║
║  ┌──────────────────────────────────────────────────────────────┐  ║
║  │                      SLAM TOOLBOX                            │  ║
║  │           (online_async mapping mode)                        │  ║
║  │  Ceres solver · Loop closure · 0.05m resolution             │  ║
║  └───────────────────────────┬──────────────────────────────────┘  ║
║                              │ /map (OccupancyGrid)                 ║
║                              ▼                                      ║
║  ┌──────────────────────────────────────────────────────────────┐  ║
║  │                   FRONTIER SERVER NODE                       │  ║
║  │      frontier_detection.py (BFS algorithm)                   │  ║
║  │  Reads /map → Detects frontiers → Publishes /frontiers       │  ║
║  └───────────────────────────┬──────────────────────────────────┘  ║
║                              │ /frontiers (MarkerArray)             ║
║                              ▼                                      ║
║  ┌──────────────────────────────────────────────────────────────┐  ║
║  │                  FRONTIER EXPLORER NODE                      │  ║
║  │   Selects best frontier → Sends NavigateToPose goal          │  ║
║  │   Cost = distance / frontier_size                            │  ║
║  └───────────────────────────┬──────────────────────────────────┘  ║
║                              │ /navigate_to_pose (Action)           ║
║                              ▼                                      ║
║  ┌──────────────────────────────────────────────────────────────┐  ║
║  │                       NAV2 STACK                             │  ║
║  │  NavFn Global Planner → DWB Local Planner → /cmd_vel        │  ║
║  │  Costmaps: Static + Obstacle + Inflation (0.55m radius)      │  ║
║  └──────────────────────────────────────────────────────────────┘  ║
╚══════════════════════════════════════════════════════════════════════╝
```

---

## 3. Project Structure

```
maze-solver-robot/
│
├── linorobot2_hardware/          # Workspace 1: ESP32 Firmware
│   └── linorobot2_hardware/
│       ├── config/
│       │   ├── config.h                  # Config selector (switches between boards)
│       │   ├── lino_base_config.h        # YOUR custom robot config ← main config
│       │   └── custom/
│       │       ├── esp32_config.h        # ESP32 template
│       │       ├── esp32s2_config.h      # ESP32-S2 template
│       │       ├── esp32s3_config.h      # ESP32-S3 template
│       │       ├── pico_config.h         # Raspberry Pi Pico template
│       │       └── pico2_config.h        # Pico 2 template
│       ├── firmware/
│       │   ├── platformio.ini            # PlatformIO project config
│       │   └── lib/
│       │       ├── encoder/              # Quadrature encoder library (ESP32)
│       │       ├── imu/                  # IMU drivers (MPU6050, MPU9250, QMI8658)
│       │       ├── kinematics/           # Differential drive kinematics
│       │       ├── motor/                # Motor driver abstraction (Generic2)
│       │       ├── odometry/             # Dead-reckoning odometry
│       │       └── pid/                  # PID controller
│       └── calibration/
│           └── platformio.ini            # Separate calibration firmware
│
├── ldlidar_ros2_ws/              # Workspace 2: LiDAR Driver
│   └── ldlidar_ros2_ws/
│       └── src/ldlidar_ros2/
│           ├── launch/
│           │   ├── ld19.launch.py        # LD19 LiDAR launch ← used in this project
│           │   ├── ld06.launch.py        # Other supported models
│           │   └── ld14.launch.py
│           ├── sdk/                      # C++ LiDAR SDK (serial comms + data processing)
│           │   ├── include/ldlidar_driver/
│           │   └── src/
│           └── src/demo.cpp              # ROS2 node wrapping the SDK
│
├── linorobot2_ws/                # Workspace 3: RViz2 Visualization
│   └── linorobot2_ws/
│       └── src/linorobot2_viz/
│           └── launch/
│               ├── robot_model.launch.py # Visualize URDF
│               ├── slam.launch.py        # RViz2 for SLAM view
│               └── navigation.launch.py  # RViz2 for Nav2 view
│
└── maze_robot_ws/                # Workspace 4: Core Maze Solving Logic
    └── maze_robot_ws/
        └── src/slam_robot/
            ├── slam_robot/
            │   ├── __init__.py
            │   ├── frontier_detection.py  # BFS frontier detection algorithm
            │   ├── frontier_explorer.py   # Autonomous navigation node
            │   ├── frontier_server.py     # Frontier publisher node
            │   └── frontier_utils.py      # Grid/coordinate utilities
            ├── config/
            │   ├── slam_toolbox_params.yaml
            │   └── nav2_params.yaml
            ├── launch/
            │   └── bringup.launch.py      # Full system launch
            ├── msg/                       # Custom ROS messages
            ├── srv/                       # Custom ROS services
            ├── setup.py
            └── package.xml
```

---

## 4. Workspace 1 — `linorobot2_hardware` (ESP32 Firmware)

This workspace contains the **embedded firmware** that runs directly on the ESP32 microcontroller. It handles all low-level hardware control and publishes/subscribes to ROS 2 topics via **micro-ROS over Serial**.

### 4.1 Hardware Configuration (`config/lino_base_config.h`)

This is the **primary configuration file** you customized for your specific robot build.

```c
// Robot type
#define LINO_BASE DIFFERENTIAL_DRIVE    // 2-wheel differential drive

// Motor driver type
#define USE_GENERIC_2_IN_MOTOR_DRIVER   // H-bridge with 2 direction pins + 1 PWM

// IMU
#define USE_MPU6050_IMU                 // MPU6050 on I2C
```

#### Physical Dimensions

| Parameter | Value | Notes |
|---|---|---|
| Wheel Diameter | 44 mm | Used in odometry and kinematics |
| Wheel Base (L-R distance) | 15 cm | Used in differential drive kinematics |
| Motor Max RPM | 330 | Spec sheet value |
| Max RPM Ratio | 0.85 | Safety factor: actual max = 330 × 0.85 = ~280 RPM |
| Operating Voltage | 12V | Motor rated voltage |
| Power Max Voltage | 12V | Voltage actually supplied |

The **effective max RPM** is calculated as:
```
max_rpm = (motor_power_max_voltage / motor_operating_voltage) × motor_max_rpm × max_rpm_ratio
         = (12/12) × 330 × 0.85
         = 280.5 RPM
```

#### Encoder Configuration

| Parameter | Value |
|---|---|
| Counts per Revolution (all wheels) | 1400 CPR |
| Left Motor Encoder Inverted | `true` |
| Right Motor Encoder Inverted | `false` |

The encoder inversion (`MOTOR1_ENCODER_INV = true`) was required to match the physical wiring direction of your robot so that forward motion produces positive encoder counts on both wheels.

#### PID Tuning Constants

| Constant | Value | Role |
|---|---|---|
| Kp (Proportional) | 0.6 | Reacts to current error |
| Ki (Integral) | 0.8 | Eliminates steady-state error |
| Kd (Derivative) | 0.5 | Dampens oscillation |

These were tuned for smooth motor velocity tracking. The PID runs per-motor to maintain target RPM.

### 4.2 ESP32 Pin Mapping

```
ESP32 DevKit
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Left Motor (Motor 1):
  GPIO 14  → PWM (speed)
  GPIO 23  → IN_A (direction)    ← was GPIO 12, changed to avoid boot conflict
  GPIO 13  → IN_B (direction)
  GPIO 26  → Encoder A (Green wire)
  GPIO 27  → Encoder B (Yellow wire)

Right Motor (Motor 2):
  GPIO 33  → PWM (speed)
  GPIO 32  → IN_A (direction)
  GPIO 25  → IN_B (direction)
  GPIO 19  → Encoder A (Green wire)
  GPIO 18  → Encoder B (Yellow wire)

IMU (MPU6050):
  GPIO 21  → SDA
  GPIO 22  → SCL
  0x68     → I2C Address

Debug:
  GPIO 2   → LED_PIN (onboard LED)
```

> **⚠️ Important Pin Note:** GPIO 12 on the ESP32 is a strapping pin that controls flash voltage at boot. Using it as an output can cause the ESP32 to fail to boot or enter download mode unexpectedly. `MOTOR1_IN_A` was moved from GPIO 12 → **GPIO 23** to fix this issue.

#### PWM Configuration

| Parameter | Value |
|---|---|
| PWM Resolution | 10-bit (0–1023) |
| PWM Frequency | 20,000 Hz (20 kHz, above human hearing) |
| PWM Max | 1023 |
| PWM Min | -1023 |

### 4.3 Firmware Software Libraries

#### PID Controller (`lib/pid/`)

A standard discrete PID controller runs in the main control loop for each motor:

```
error      = setpoint_rpm - measured_rpm
integral  += error
derivative = error - prev_error

output = Kp × error + Ki × integral + Kd × derivative
```

When the setpoint is zero and error is zero, the integral and derivative are reset to prevent windup. Output is clamped to `[PWM_MIN, PWM_MAX]`.

#### Kinematics Engine (`lib/kinematics/`)

Converts between **velocity commands** (`linear_x`, `angular_z`) and **individual motor RPMs**, and vice versa.

**Forward kinematics** (RPM → velocity — used to compute odometry):
```
linear_x  = ((rpm1 + rpm2) / 2) × wheel_circumference / 60
angular_z = ((rpm2 - rpm1) / 2) × wheel_circumference / (wheel_base/2) / 60
```

**Inverse kinematics** (velocity → RPM — used when receiving `/cmd_vel`):
```
tangential_vel = angular_z × (wheel_base / 2)
rpm_left       = (linear_x - tangential_vel) × 60 / wheel_circumference
rpm_right      = (linear_x + tangential_vel) × 60 / wheel_circumference
```

A **velocity scaler** is applied when required RPM exceeds `max_rpm_` to maintain the correct heading at a reduced speed rather than saturating one motor.

#### Odometry (`lib/odometry/`)

Dead-reckoning odometry integrates wheel velocities over time:

```
delta_heading = angular_vel_z × dt
delta_x       = (linear_vel_x × cos(heading) - linear_vel_y × sin(heading)) × dt
delta_y       = (linear_vel_x × sin(heading) + linear_vel_y × cos(heading)) × dt

x_pos    += delta_x
y_pos    += delta_y
heading  += delta_heading
```

Heading is converted to a quaternion and published in `nav_msgs/Odometry` on `/odom`.

**Covariance values:**
- Pose covariance: `[0.0001, 0.0001, 0, 0, 0, 0.0001]`
- Twist covariance: `[0.00001, 0.00001, 0, 0, 0, 0.00001]`

#### Motor Driver (`lib/motor/` — Generic2 Class)

Controls an H-bridge with 2 direction pins and 1 PWM pin:
- **Forward:** `IN_A = HIGH`, `IN_B = LOW`, PWM applied
- **Reverse:** `IN_A = LOW`, `IN_B = HIGH`, PWM applied

The `invert` parameter swaps direction logic for motors physically wired in reverse — used for `MOTOR1_INV = true`.

#### Encoder (`lib/encoder/`)

Uses the **ESP32Encoder** library in **half-quadrature mode** (`attachHalfQuad`) for interrupt-driven encoder counting. RPM is calculated from tick delta and time delta:

```
RPM = (delta_ticks / counts_per_rev) / (dt_microseconds / 60,000,000)
```

#### IMU (`lib/imu/` — MPU6050)

The MPU6050 is connected via I2C and provides 3-axis accelerometer and gyroscope data, published as `sensor_msgs/Imu` on `/imu/data_raw`. Supports multiple IMU models selectable via `#define`:

| Define | Driver |
|---|---|
| `USE_MPU6050_IMU` | MPU6050 (this project) |
| `USE_MPU9250_IMU` | MPU9250 |
| `USE_QMI8658_IMU` | QMI8658 |
| `USE_GY85_IMU` | GY-85 (ADXL345 + ITG3200) |

### 4.4 PlatformIO Configuration (`firmware/platformio.ini`)

```ini
[platformio]
default_envs = esp32         # Builds for ESP32 by default

[env:esp32]
platform = espressif32
board = esp32dev
framework = arduino

board_microros_transport = serial    # micro-ROS uses UART serial
board_microros_distro = humble       # ROS 2 Humble

monitor_speed = 115200
upload_protocol = esptool
board_build.f_flash = 80000000L      # 80 MHz flash speed
board_build.flash_mode = qio         # Quad I/O flash mode
board_build.partitions = min_spiffs.csv

build_flags =
    -I ../config                     # Include config/ directory
    -D __PGMSPACE_H_                 # Suppress AVR-specific pgmspace errors
```

**Key external libraries:**
- `micro_ros_platformio` — micro-ROS transport over Serial
- `jrowberg/I2Cdevlib-MPU6050` — IMU driver
- `madhephaestus/ESP32Encoder` — hardware encoder reading
- `madhephaestus/ESP32Servo` — PWM generation

---

## 5. Workspace 2 — `ldlidar_ros2_ws` (LiDAR Driver)

Provides the **ROS 2 driver** for the LDRobot LD19 2D LiDAR sensor.

### 5.1 LD19 Launch Configuration (`launch/ld19.launch.py`)

| Parameter | Value | Description |
|---|---|---|
| `product_name` | `LDLiDAR_LD19` | Sensor model |
| `laser_scan_topic_name` | `scan` | Published as `/scan` |
| `frame_id` | `base_laser` | TF frame for the LiDAR |
| `port_name` | `/dev/ttyUSB0` | Serial port |
| `serial_baudrate` | `230400` | High-speed baud rate |
| `laser_scan_dir` | `true` | Counterclockwise scan direction |
| `enable_angle_crop_func` | `false` | No angle masking |
| `range_min` | `0.02 m` | Minimum valid range |
| `range_max` | `12.0 m` | Maximum valid range |

### 5.2 TF Transform

The launch file also publishes a **static transform** from `base_link` to `base_laser`:

```
Translation: [0, 0, 0.18]   → LiDAR is mounted 18 cm above the base
Rotation:    [0, 0, 0]      → No angular offset
```

### 5.3 SDK Architecture

The LiDAR driver is built on a C++ SDK that handles:
- Serial port communication (`serial_interface_linux.cpp`)
- Protocol parsing (`ldlidar_protocol.cpp`)
- Data processing and filtering (`ldlidar_dataprocess.cpp`)
- Beacon/spot filtering (`slbf.cpp`, `tofbf.cpp`)

---

## 6. Workspace 3 — `linorobot2_ws` (Visualization)

Provides **RViz2** launch files for real-time monitoring.

### 6.1 Launch Files

| File | What it shows |
|---|---|
| `robot_model.launch.py` | URDF/XACRO robot model in 3D |
| `slam.launch.py` | Occupancy grid map being built + robot pose + LiDAR scan |
| `navigation.launch.py` | Map + costmaps + Nav2 planned path + robot footprint |

---

## 7. Workspace 4 — `maze_robot_ws` (Core Maze Solver)

The **heart of the project** — the custom `slam_robot` ROS 2 package implementing autonomous frontier-based exploration.

### 7.1 Package Information

| Field | Value |
|---|---|
| Package Name | `slam_robot` |
| Version | 0.0.0 |
| License | MIT |
| Build System | `ament_python` |
| ROS Distribution | Humble |

### 7.2 Node: `frontier_server` (`frontier_server.py`)

**Purpose:** Continuously monitors the occupancy grid and publishes detected frontiers.

**Subscriptions:**
- `/map` (`nav_msgs/OccupancyGrid`) — Latest map from SLAM Toolbox
- `/get_frontiers` (`std_msgs/Empty`) — On-demand trigger

**Publications:**
- `/frontiers` (`visualization_msgs/MarkerArray`) — One sphere marker per frontier

**Behavior:**
1. Stores the latest map from SLAM Toolbox
2. On a 1 Hz timer OR when triggered via `/get_frontiers`:
   - Looks up robot position via TF (`map → base_footprint`)
   - Calls `detect_frontiers()` with minimum size = 8 cells
   - Encodes each frontier as a `Marker` sphere (frontier size stored in `marker.scale.x`)
   - Publishes the `MarkerArray` to `/frontiers`

**Frontier encoding in MarkerArray:**
```python
marker.scale.x = float(frontier.size)   # Frontier size (number of cells)
marker.scale.y = 0.2                    # Visual diameter in RViz
marker.color   = (0.0, 0.0, 1.0, 1.0)  # Blue spheres in RViz
```

### 7.3 Node: `frontier_explorer` (`frontier_explorer.py`)

**Purpose:** Selects the best frontier and sends navigation goals to Nav2.

**Subscriptions:**
- `/frontiers` (`visualization_msgs/MarkerArray`) — Frontier list

**Publications:**
- `/get_frontiers` (`std_msgs/Empty`) — Triggers frontier_server
- `/frontier_markers` (`visualization_msgs/MarkerArray`) — Re-publishes for RViz

**Action Clients:**
- `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`) — Navigation goals to Nav2

**Exploration Loop (1 Hz timer):**

```
Every 1 second:
  IF not currently navigating:
    1. Publish Empty to /get_frontiers  (request detection)
    2. Read latest_frontiers received from /frontiers
    3. IF no frontiers:
         no_frontiers_count++
         IF count >= 30: LOG "Exploration complete" → return
    4. ELSE:
         no_frontiers_count = 0
         best = select_best_frontier(frontier_list)
         send_navigation_goal(best)
         is_navigating = True
```

**Frontier Selection Cost Function:**

```python
cost = distance_to_frontier / frontier.size

# Prefer: close frontiers (low distance) AND large frontiers (high size)
# This balances travel efficiency with information gain
```

**Navigation Status Handling:**

| Nav2 Status Code | Meaning | Action |
|---|---|---|
| 4 | SUCCEEDED | Log success, pick next frontier on next tick |
| 2 | CANCELED | Log warning, retry next tick |
| 3 | ABORTED | Log warning, retry next tick |

### 7.4 Module: Frontier Detection (`frontier_detection.py`)

The core algorithm. See [Section 8](#8-frontier-exploration-algorithm--deep-dive) for detailed walkthrough.

**Key constants:**
```python
MIN_FRONTIER_SIZE = 8       # Minimum cells to count as a valid frontier
WALKABLE_THRESHOLD = 50     # Cells with value < 50 are considered free
```

### 7.5 Module: Grid Utilities (`frontier_utils.py`)

| Function | Description |
|---|---|
| `grid_to_index(mapdata, p)` | Converts (x,y) grid cell to flat array index: `y × width + x` |
| `get_cell_value(mapdata, p)` | Returns occupancy value at a grid cell (-1, 0–100) |
| `grid_to_world(mapdata, p)` | Converts grid cell to world Point using map origin and resolution |
| `world_to_grid(mapdata, wp)` | Converts world Point to grid (x,y) cell |
| `is_cell_in_bounds(mapdata, p)` | Checks if cell is within map dimensions |
| `is_cell_walkable(mapdata, p)` | True if in bounds and value < 50 |
| `neighbors_of_4(mapdata, p)` | Returns up/down/left/right neighbors |
| `neighbors_of_8(mapdata, p)` | Returns all 8 neighbors including diagonals |

**Coordinate conversion formulas:**
```python
# Grid → World
world_x = (grid_x + 0.5) × resolution + origin_x
world_y = (grid_y + 0.5) × resolution + origin_y

# World → Grid
grid_x = int((world_x - origin_x) / resolution)
grid_y = int((world_y - origin_y) / resolution)
```

### 7.6 Launch File: `bringup.launch.py`

| Component | Package | Purpose |
|---|---|---|
| Gazebo + TurtleBot3 | `turtlebot3_gazebo` | Simulation environment (DQN Stage 4 map) |
| SLAM Toolbox | `slam_toolbox` | Online async SLAM mapping |
| Nav2 Navigation | `nav2_bringup` | Path planning + obstacle avoidance |
| Nav2 RViz | `nav2_bringup` | Visualization |
| Frontier Server | `slam_robot` | Frontier detection publisher |
| Frontier Explorer | `slam_robot` | Autonomous navigation controller |

> **Note:** `use_sim_time = true` means this is configured for **Gazebo simulation**. For the real robot, set `use_sim_time = false` and remove the Gazebo launch action.

---

## 8. Frontier Exploration Algorithm — Deep Dive

### 8.1 What is a Frontier?

In an occupancy grid map, each cell has one of three states:
- **Free** (`0–49`): Scanned and passable
- **Occupied** (`50–100`): Contains an obstacle
- **Unknown** (`-1`): Never observed by LiDAR

A **frontier** is a free cell adjacent to an unknown cell — it sits on the boundary of the explored world. Navigating to frontiers causes the LiDAR to scan new areas, expanding the map.

```
Map snapshot:
  [#][#][#][#][#][#][#][#]   # = Occupied (wall)
  [#][ ][ ][ ][?][?][?][#]   ' '= Free, '?' = Unknown
  [#][ ][R][ ][F][?][?][#]    F = Frontier cell
  [#][ ][ ][ ][?][?][?][#]
  [#][#][#][#][#][#][#][#]
```

### 8.2 `detect_frontiers()` — Step-by-Step

```
Input:  mapdata (OccupancyGrid), start_pos (robot in grid coords), min_size=8
Output: FrontierList

Algorithm:

  1. Initialize:
       queue       = [start_pos]
       visited     = {start_pos: True}
       is_frontier = {}
       frontiers   = []

  2. BFS over FREE cells:
       while queue not empty:
         current = queue.pop(0)

         for each 4-connected neighbor of current:
           neighbor_value = map[neighbor]

           CASE A: neighbor is FREE (0–49) and NOT visited:
             → Mark visited
             → Add to queue       (continue exploring free space)

           CASE B: neighbor is UNKNOWN (-1) and is_new_frontier_cell():
             → Mark as frontier
             → Build new frontier from this seed (8-connected BFS)
             → If frontier.size >= min_size: add to list

  3. Return FrontierList(frontiers)
```

### 8.3 `is_new_frontier_cell()` — Conditions

A cell qualifies as a new frontier cell if ALL of:

1. It is within map bounds
2. Its value is exactly `-1` (unknown)
3. It has NOT already been marked as a frontier
4. At least one of its **4-connected neighbors** is free (`0 ≤ value < 50`)

Condition 4 ensures the frontier is reachable — adjacent to explored free space.

### 8.4 `build_new_frontier()` — Grouping Connected Cells

Once a seed is found, all connected frontier cells are grouped via **8-connected BFS**:

```
Input:  initial_cell (seed frontier cell)
Output: Frontier(size, centroid_in_world_coords)

  size      = 1
  centroid  = initial_cell

  queue = [initial_cell]
  while queue not empty:
    current = queue.pop(0)
    for each 8-connected neighbor:
      if is_new_frontier_cell(neighbor):
        mark as frontier
        size     += 1
        centroid += neighbor
        queue.append(neighbor)

  centroid /= size                     # Average all frontier cell positions
  centroid = grid_to_world(centroid)   # Convert to meters (world frame)

  return Frontier(size=size, centroid=centroid)
```

The **8-connected** grouping (vs 4-connected for the main BFS) ensures diagonally adjacent frontier cells are correctly grouped into the same region.

### 8.5 Visual Example of the Full Algorithm

```
Step 1: Robot at R, known map so far
  [#][#][#][#][#][#][#][#]
  [#][ ][ ][ ][?][?][?][#]
  [#][ ][R][ ][?][?][?][#]
  [#][ ][ ][ ][?][?][?][#]
  [#][#][#][#][#][#][#][#]

Step 2: BFS spreads through free cells (B = visited)
  [#][#][#][#][#][#][#][#]
  [#][B][B][B][?][?][?][#]
  [#][B][R][B][?][?][?][#]
  [#][B][B][B][?][?][?][#]
  [#][#][#][#][#][#][#][#]

Step 3: Cells adjacent to unknown are detected as frontiers (F)
  [#][#][#][#][#][#][#][#]
  [#][B][B][F][?][?][?][#]
  [#][B][R][F][?][?][?][#]
  [#][B][B][F][?][?][?][#]
  [#][#][#][#][#][#][#][#]

Step 4: 8-connected BFS groups all F cells → Frontier #1
  size = 3, centroid = average of the 3 F cells → converted to world meters

Step 5: Explorer navigates robot to centroid
  Nav2 plans path → Robot moves → LiDAR scans unknown area → Map grows
  New frontiers appear → Process repeats until no frontiers remain
```

---

## 9. SLAM Configuration

**File:** `maze_robot_ws/src/slam_robot/config/slam_toolbox_params.yaml`

SLAM Toolbox runs in **`mapping` mode**, building the occupancy grid from scratch.

### 9.1 Core Settings

| Parameter | Value | Description |
|---|---|---|
| `mode` | `mapping` | Build new map (not localization) |
| `solver_plugin` | `CeresSolver` | Nonlinear graph optimizer |
| `ceres_linear_solver` | `SPARSE_NORMAL_CHOLESKY` | Efficient sparse matrix solver |
| `ceres_trust_strategy` | `LEVENBERG_MARQUARDT` | Robust optimization |
| `odom_frame` | `odom` | Odometry frame |
| `map_frame` | `map` | Global map frame |
| `base_frame` | `base_footprint` | Robot base frame |
| `scan_topic` | `/scan` | LiDAR input |

### 9.2 Map Quality Settings

| Parameter | Value | Description |
|---|---|---|
| `resolution` | `0.05 m` | Each cell = 5 cm × 5 cm |
| `max_laser_range` | `20.0 m` | Ignore returns beyond this |
| `min_laser_range` | `0.0 m` | Include all near returns |
| `map_update_interval` | `5.0 s` | How often the map is saved/updated |
| `transform_publish_period` | `0.02 s` | TF publish rate (50 Hz) |
| `throttle_scans` | `1` | Process every scan (no skipping) |

### 9.3 Motion Thresholds (Scan Integration)

| Parameter | Value | Description |
|---|---|---|
| `minimum_travel_distance` | `0.5 m` | Move 50 cm before integrating a new scan |
| `minimum_travel_heading` | `0.5 rad` | Or rotate ~28.6° before integrating |
| `minimum_time_interval` | `0.5 s` | Minimum time between integrations |

### 9.4 Loop Closure Settings

| Parameter | Value | Description |
|---|---|---|
| `do_loop_closing` | `true` | Enable loop closure detection |
| `loop_match_minimum_chain_size` | `10` | Min scans in a chain to attempt closure |
| `loop_search_maximum_distance` | `3.0 m` | Search radius for loop candidates |
| `loop_match_minimum_response_coarse` | `0.35` | Min match quality (coarse) |
| `loop_match_minimum_response_fine` | `0.45` | Min match quality (fine) |
| `loop_search_space_dimension` | `8.0 m` | Loop closure search space size |

Loop closure corrects **accumulated odometry drift** when the robot revisits a previously mapped area — critical for accurate maze mapping.

---

## 10. Nav2 Navigation Stack

**File:** `maze_robot_ws/src/slam_robot/config/nav2_params.yaml`

### 10.1 Localization: AMCL

AMCL (Adaptive Monte Carlo Localization) uses a particle filter to estimate robot pose.

| Parameter | Value |
|---|---|
| `robot_model_type` | `nav2_amcl::DifferentialMotionModel` |
| `min_particles` | `500` |
| `max_particles` | `2000` |
| `laser_model_type` | `likelihood_field` |
| `max_beams` | `60` |
| `update_min_d` | `0.25 m` |
| `update_min_a` | `0.2 rad` |
| `transform_tolerance` | `1.0 s` |

### 10.2 Path Planners

**Global Planner:** `NavfnPlanner` (Dijkstra/A* based)
```yaml
planner_plugins: ["GridBased"]
  plugin: "nav2_navfn_planner/NavfnPlanner"
expected_planner_frequency: 20.0 Hz
```

**Local Planner:** `DWBLocalPlanner` (Dynamic Window Approach)
```yaml
controller_plugins: ["FollowPath"]
  plugin: "dwb_core::DWBLocalPlanner"
controller_frequency: 20.0 Hz
```

### 10.3 Costmaps

**Local Costmap** (immediate obstacle avoidance):

| Layer | Plugin | Purpose |
|---|---|---|
| `voxel_layer` | `nav2_costmap_2d::VoxelLayer` | 3D obstacle detection from LiDAR |
| `inflation_layer` | `nav2_costmap_2d::InflationLayer` | Inflate obstacles by **0.55 m** |

**Global Costmap** (full path planning):

| Layer | Plugin | Purpose |
|---|---|---|
| `static_layer` | `nav2_costmap_2d::StaticLayer` | SLAM map as base |
| `obstacle_layer` | `nav2_costmap_2d::ObstacleLayer` | Dynamic obstacles from sensors |
| `inflation_layer` | `nav2_costmap_2d::InflationLayer` | Inflate obstacles by **0.55 m** |

The **0.55 m inflation radius** creates a safety buffer around walls. Since the wheel base is 15 cm, this keeps the robot well clear of maze walls during navigation.

### 10.4 Behavior Tree Navigator

The `bt_navigator` orchestrates the full navigation pipeline. Key BT nodes:
- `nav2_compute_path_to_pose_action_bt_node` — Calls the global planner
- `nav2_follow_path_action_bt_node` — Calls the local controller
- `nav2_spin_action_bt_node` — Recovery: spin in place
- `nav2_back_up_action_bt_node` — Recovery: back up
- `nav2_is_stuck_condition_bt_node` — Detects if robot is stuck

---

## 11. ROS 2 Topics, Services & Actions

### Topics

| Topic | Message Type | Publisher | Subscriber | Description |
|---|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR driver | SLAM Toolbox, Nav2 | 2D LiDAR scan |
| `/odom` | `nav_msgs/Odometry` | ESP32 (micro-ROS) | SLAM Toolbox, Nav2 | Wheel odometry |
| `/imu/data_raw` | `sensor_msgs/Imu` | ESP32 (micro-ROS) | SLAM Toolbox | Raw IMU data |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM Toolbox | Frontier Server, Nav2 | Occupancy map |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 controller | ESP32 (micro-ROS) | Velocity commands |
| `/get_frontiers` | `std_msgs/Empty` | Frontier Explorer | Frontier Server | Trigger detection |
| `/frontiers` | `visualization_msgs/MarkerArray` | Frontier Server | Frontier Explorer | Detected frontiers |
| `/frontier_markers` | `visualization_msgs/MarkerArray` | Frontier Explorer | RViz2 | Visualization |
| `/tf` | `tf2_msgs/TFMessage` | SLAM, Nav2, micro-ROS | All | Transform tree |

### Actions

| Action | Type | Client | Server |
|---|---|---|---|
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Frontier Explorer | Nav2 bt_navigator |

---

## 12. TF Tree (Coordinate Frames)

```
map
 └── odom                    (published by SLAM Toolbox — corrects drift)
      └── base_footprint     (published by micro-ROS from wheel odometry)
           └── base_link     (from URDF / robot state publisher)
                └── base_laser    (static: z=0.18m, published by ldlidar launch)
```

| Transform | Publisher | Description |
|---|---|---|
| `map → odom` | SLAM Toolbox | Corrects odometry drift with scan matching |
| `odom → base_footprint` | micro-ROS (ESP32) | Dead-reckoning from encoders |
| `base_footprint → base_link` | Robot State Publisher | Fixed, from URDF |
| `base_link → base_laser` | ldlidar launch (static TF) | LiDAR at 18 cm height |

---

## 13. How to Build & Run

### Prerequisites

```bash
# ROS 2 Humble
sudo apt install ros-humble-desktop

# Nav2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# SLAM Toolbox
sudo apt install ros-humble-slam-toolbox

# micro-ROS agent
sudo apt install ros-humble-micro-ros-agent

# PlatformIO
pip install platformio
```

### Build All Workspaces

```bash
cd ldlidar_ros2_ws && colcon build && source install/setup.bash && cd ..
cd linorobot2_ws   && colcon build && source install/setup.bash && cd ..
cd maze_robot_ws   && colcon build && source install/setup.bash && cd ..
```

### Step 1: Flash ESP32 Firmware

```bash
cd linorobot2_hardware/firmware
pio run -e esp32 --target upload --upload-port /dev/ttyUSB0

# Monitor serial output
pio device monitor --port /dev/ttyUSB0 --baud 115200
```

### Step 2: Start micro-ROS Agent

```bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyUSB0 \
  -b 115200
```

Wait for the ESP32 to connect — you will see "Connected" in the agent output.

### Step 3: Launch LiDAR Driver

```bash
cd ldlidar_ros2_ws && source install/setup.bash
ros2 launch ldlidar_ros2 ld19.launch.py

# Verify
ros2 topic echo /scan --once
```

### Step 4: Launch SLAM Toolbox

```bash
cd maze_robot_ws && source install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false \
  slam_params_file:=src/slam_robot/config/slam_toolbox_params.yaml
```

### Step 5: Launch Nav2

```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  params_file:=src/slam_robot/config/nav2_params.yaml
```

### Step 6: Launch Frontier Exploration

```bash
cd maze_robot_ws && source install/setup.bash
ros2 run slam_robot frontier_server &
ros2 run slam_robot frontier_explorer
```

### Step 7: Visualize (Optional)

```bash
cd linorobot2_ws && source install/setup.bash
ros2 launch linorobot2_viz slam.launch.py
```

### All-in-One (Simulation Only)

```bash
cd maze_robot_ws && source install/setup.bash
ros2 launch slam_robot bringup.launch.py
```

---

## 14. Troubleshooting

| Problem | Likely Cause | Fix |
|---|---|---|
| ESP32 won't boot after flashing | GPIO 12 strapping pin conflict | Confirm `MOTOR1_IN_A` is GPIO 23, not 12 |
| micro-ROS agent shows "disconnected" | Wrong baud rate or port | Verify `/dev/ttyUSB0` and baud 115200 |
| No `/scan` topic | LiDAR not detected | Check USB, run `ls /dev/ttyUSB*` |
| Map not building | SLAM not receiving `/odom` | Confirm micro-ROS agent is connected |
| Robot not moving to frontiers | Nav2 not ready | Wait for "Nav2 is ready" log |
| Robot oscillates / overshoots | PID not tuned | Adjust Kp, Ki, Kd in `lino_base_config.h` |
| Navigation fails near walls | Inflation radius too large | Reduce `inflation_radius` in `nav2_params.yaml` |
| Exploration completes too early | `min_size` too high | Lower `MIN_FRONTIER_SIZE` in `frontier_detection.py` |
| `/cmd_vel` published but robot not moving | Motor direction wrong | Adjust `MOTOR1_INV`/`MOTOR2_INV` in config |
| Robot drifts to one side | Encoder mismatch | Check `MOTOR1_ENCODER_INV`/`MOTOR2_ENCODER_INV` |

---

## 15. Technologies & Dependencies

| Category | Technology | Version/Notes |
|---|---|---|
| Robotics Framework | ROS 2 | Humble Hawksbill |
| Microcontroller | ESP32 DevKit | Espressif ESP32 |
| Firmware Framework | PlatformIO + Arduino | espressif32 platform |
| MCU–ROS Bridge | micro-ROS | Serial transport, Humble distro |
| SLAM | SLAM Toolbox | Online async, Ceres solver |
| Navigation | Nav2 | NavFn global + DWB local |
| LiDAR | LDRobot LD19 | 230400 baud, up to 12 m range |
| IMU | MPU6050 | I2C, address 0x68 |
| Motor Driver | Generic 2-IN H-Bridge | PWM + 2 direction pins |
| Encoder Library | ESP32Encoder | Half-quadrature interrupt mode |
| Visualization | RViz2 | Custom SLAM + Nav2 configs |
| Exploration Algorithm | Custom BFS Frontier Detection | Python 3 |
| Firmware Language | C++ | Arduino framework |
| ROS Nodes Language | Python 3 | ament_python build |
| Simulation | Gazebo | TurtleBot3 DQN Stage 4 map |

---

*Built with ROS 2 Humble · linorobot2 · SLAM Toolbox · Nav2 · micro-ROS · LDRobot LD19*
