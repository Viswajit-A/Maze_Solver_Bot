# 🤖 Maze Solver Robot

A maze-solving robot built on **ROS 2 (Humble)** using the **linorobot2** platform. The robot uses frontier-based exploration with SLAM and Nav2 to map and navigate through unknown environments.

---
## 👥 Team Members

| Name | Institute |
|---|---|
| Budharaju Pavani Siva Priya | Amrita Vishwa Vidyapeetham |
| Prajan S | Amrita Vishwa Vidyapeetham |
| Viswajit Arunkumar | Amrita Vishwa Vidyapeetham |
| Yashwanth Ram Mohan C | Amrita Vishwa Vidyapeetham |

**Mentor:** Jayasree P.R.

---

## 🎥 Preparation Video

[Click Here for the Maze Solver Robot Preparation](https://youtu.be/phrGzGO_d04)

## 📁 Project Structure

```
├── linorobot2_hardware/      # ESP32 firmware (micro-ROS, motors, encoders, IMU)
├── linorobot2_ws/            # ROS 2 visualization workspace (RViz2 launch files)
├── ldlidar_ros2_ws/          # LiDAR driver workspace (LD series LiDAR sensors)
└── maze_robot_ws/            # Core maze-solving logic (SLAM + Nav2 + Frontier Exploration)
```
## 🔩 Hardware Design

### A. Mechanical System

The chassis is fabricated from **laser-cut acrylic sheets** in a two-plate structure (base plate + top plate), connected by **four 30 mm spacers** for structural rigidity.

| Spec | Value |
|---|---|
| Drive Type | Differential Drive (2WD) |
| Wheel Diameter | 44 mm |
| Wheel Base | 15 cm |
| Robot Footprint | 15 × 15 cm |
| Robot Height | ~8.3 cm |
| Support | Ball caster wheel |

**Sensor Placement:**
- 2D LiDAR mounted at the **center** for symmetrical 360° scanning
- IMU positioned at the **center of the bottom plate** to minimize motion-induced measurement errors

### B. Electronics & Instrumentation

#### Control Architecture (Distributed)

| Layer | Component | Role |
|---|---|---|
| High-level | Raspberry Pi 4 Model B | ROS 2 nodes, SLAM, Nav2, path planning |
| Low-level | ESP32 | Motor PWM, encoder reading, micro-ROS client |
| Storage | 128 GB SD Card | OS, ROS packages, project data |
| Display | OLED Screen | Runtime status — boot, mapping, obstacle avoidance |

#### Sensing & Motion

| Component | Model | Purpose |
|---|---|---|
| LiDAR | LD Lidar STL19P (LD19) | 360° environmental scanning, obstacle detection |
| IMU | MPU6050 | Orientation, acceleration, angular velocity |
| Motors | N20 DC Geared + Encoder | Drive + odometry feedback |
| Motor Driver | TB6612FNG | Dual-channel H-bridge for motor power |

#### Power Source

| Component | Spec |
|---|---|
| Battery | 11.1V Lithium-Ion (3S, 3380mAh 60C) |
| Distribution | XT60 Power Distribution Board |
| Connectors | XT60 (battery), Type-C (Pi), Micro-USB (ESP32) |
| Safety | Rocket switch (master power cutoff) |
---

## 🧠 System Overview

```
┌─────────────────────────────────────────────────────────────┐
│                        Robot System                         │
│                                                             │
│   ESP32 Firmware (micro-ROS)                                │
│   ├── Motor Control (PWM + PID)                             │
│   ├── Encoder Feedback (Odometry)                           │
│   └── IMU (MPU6050) → Orientation                           │
│                        ↓                                    │
│   LiDAR (LD19) → /scan topic                                │
│                        ↓                                    │
│   SLAM Toolbox → /map (OccupancyGrid)                       │
│                        ↓                                    │
│   Frontier Server → Detects unknown regions                 │
│                        ↓                                    │
│   Frontier Explorer → Selects & navigates to frontiers      │
│                        ↓                                    │
│   Nav2 (navigate_to_pose) → Path planning & execution       │
└─────────────────────────────────────────────────────────────┘
```

---

## 📦 Workspace Details

### 1. `linorobot2_hardware` — ESP32 Firmware

The firmware runs on an **ESP32** microcontroller using **PlatformIO** and communicates with ROS 2 via **micro-ROS over Serial**.

#### Hardware Configuration (`config/lino_base_config.h`)

| Parameter | Value |
|---|---|
| Robot Type | Differential Drive (2WD) |
| Motor Driver | Generic 2-IN driver |
| IMU | MPU6050 (I2C) |
| Wheel Diameter | 44 mm |
| Wheel Base (LR distance) | 15 cm |
| Max Motor RPM | 330 |
| Encoder CPR | 1400 counts/rev |
| Transport | micro-ROS over Serial |
| ROS Distro | Humble |

#### ESP32 Pin Mapping

| Function | Pin |
|---|---|
| Left Motor PWM | 14 |
| Left Motor IN_A | 23 |
| Left Motor IN_B | 13 |
| Right Motor PWM | 33 |
| Right Motor IN_A | 32 |
| Right Motor IN_B | 25 |
| Left Encoder A/B | 26 / 27 |
| Right Encoder A/B | 19 / 18 |
| IMU SDA / SCL | 21 / 22 |
| IMU I2C Address | 0x68 |
| micro-ROS Agent Port | 8888 |

#### PID Constants

| Constant | Value |
|---|---|
| Kp | 0.6 |
| Ki | 0.8 |
| Kd | 0.5 |

#### Firmware Libraries

- `micro_ros_platformio` — ROS 2 communication over Serial
- `I2Cdevlib-MPU6050` — IMU sensor driver
- `ESP32Encoder` — Hardware encoder reading
- `ESP32Servo` — PWM servo/motor control

---

### 2. `ldlidar_ros2_ws` — LiDAR Driver Workspace

Provides ROS 2 drivers for the **LDRobot LD-series LiDAR** sensors. Publishes laser scan data to the `/scan` topic consumed by SLAM Toolbox.

#### Supported LiDAR Models

- LD06
- LD14 / LD14P
- **STL19P** (used in this project)

#### Key ROS Topics

| Topic | Type | Description |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Laser scan data output |

---

### 3. `linorobot2_ws` — Visualization Workspace

Provides **RViz2** launch files for visualizing the robot model, SLAM map, and navigation.

#### Launch Files

| Launch File | Purpose |
|---|---|
| `robot_model.launch.py` | Displays URDF robot model in RViz2 |
| `slam.launch.py` | Opens RViz2 configured for SLAM visualization |
| `navigation.launch.py` | Opens RViz2 configured for Nav2 navigation |

---

### 4. `maze_robot_ws` — Core Maze Solving Workspace

This is the heart of the project. The `slam_robot` ROS 2 package implements **autonomous frontier-based exploration** to map and solve mazes.

#### Package: `slam_robot`

```
slam_robot/
├── slam_robot/
│   ├── frontier_detection.py   # BFS-based frontier detection algorithm
│   ├── frontier_explorer.py    # Autonomous navigation to frontiers
│   ├── frontier_server.py      # Publishes detected frontiers as ROS topics
│   └── frontier_utils.py       # Grid/world coordinate utilities
├── config/
│   ├── slam_toolbox_params.yaml  # SLAM Toolbox configuration
│   └── nav2_params.yaml          # Nav2 stack configuration
└── launch/
    └── bringup.launch.py         # Full system launch file
```

---

## 🔍 Frontier Exploration Algorithm

### What is Frontier-Based Exploration?

A **frontier** is the boundary between explored free space and unexplored (unknown) space on the map. By navigating to these frontiers, the robot systematically uncovers the entire environment — effectively "solving" the maze by mapping it completely.

### How It Works — Step by Step

#### Step 1: Frontier Detection (`frontier_detection.py`)

The `detect_frontiers()` function runs a **BFS (Breadth-First Search)** starting from the robot's current grid position:

1. The BFS walks through all **free cells** (value `0–49` in the occupancy grid).
2. For each free cell, it checks its 4-connected neighbors.
3. If a neighbor is **unknown** (`-1`) and has at least one free neighbor, it qualifies as a **frontier cell**.
4. Connected frontier cells are grouped into **frontier regions** using an 8-connected BFS (`build_new_frontier()`).
5. Each frontier region is represented by its **size** (number of cells) and **centroid** (world coordinates).
6. Only frontiers with `size >= 8` cells are kept to filter noise.

```
Map Legend:
  0   = Free (explored)
  -1  = Unknown (unexplored)
  100 = Occupied (wall)
  F   = Frontier cell (boundary between free and unknown)

  [100][100][100][100][100]
  [  0][  0][  F][ -1][ -1]
  [  0][  0][  F][ -1][ -1]
  [  0][  0][  F][ -1][ -1]
  [100][100][100][100][100]
```

#### Step 2: Frontier Publishing (`frontier_server.py`)

The `FrontierServerNode`:
- Subscribes to `/map` (OccupancyGrid from SLAM Toolbox)
- Gets robot position via **TF** (`map → base_footprint`)
- Calls `detect_frontiers()` every 1 second or on demand
- Publishes results to `/frontiers` as a `MarkerArray` (each marker = one frontier)

#### Step 3: Frontier Navigation (`frontier_explorer.py`)

The `FrontierExplorerNode`:
1. **Triggers** frontier detection by publishing to `/get_frontiers` at 1 Hz.
2. **Receives** the latest frontiers from `/frontiers`.
3. **Selects** the best frontier using the cost function:

   ```
   cost = distance_to_frontier / frontier_size
   ```

   This balances proximity (prefer nearby frontiers) with informativeness (prefer larger unexplored regions).

4. **Sends** a `NavigateToPose` action goal to **Nav2**, which handles path planning and obstacle avoidance.
5. **Terminates** exploration after 30 consecutive seconds with no frontiers found — indicating the maze is fully mapped.

### Frontier Selection Strategy

| Factor | Effect |
|---|---|
| Smaller distance | Lower cost → preferred |
| Larger frontier size | Lower cost → preferred |
| Both combined | Efficient, greedy exploration |

---

## 🗺️ SLAM Configuration

SLAM Toolbox runs in **online async mapping mode**.

| Parameter | Value |
|---|---|
| Solver | Ceres (SPARSE_NORMAL_CHOLESKY) |
| Map Resolution | 0.05 m/cell |
| Max Laser Range | 20 m |
| Min Travel Distance | 0.5 m |
| Loop Closure | Enabled |
| Scan Topic | `/scan` |
| Base Frame | `base_footprint` |
| Map Frame | `map` |
| Odom Frame | `odom` |

---

## 🧭 Nav2 Configuration

Nav2 provides **path planning and execution** for navigating to selected frontiers.

| Component | Configuration |
|---|---|
| Localization | AMCL (Adaptive Monte Carlo) |
| Motion Model | Differential Drive |
| Global Planner | Default (Navfn) |
| Local Planner | Default (DWB) |
| Max Particles | 2000 |
| Min Particles | 500 |
| Robot Base Frame | `base_footprint` |
| Scan Topic | `/scan` |

---

## 🚀 How to Run the Project

### Prerequisites

- ROS 2 Humble
- PlatformIO (for firmware)
- micro-ROS agent

### Step 1: Flash the Firmware

```bash
cd linorobot2_hardware/firmware
pio run -e esp32 --target upload
```

### Step 2: Start the micro-ROS Agent

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

### Step 3: Launch the LiDAR Driver

```bash
cd ldlidar_ros2_ws
source install/setup.bash
ros2 launch ldlidar_ros2 ld19.launch.py
```

### Step 4: Launch the Maze Solver

```bash
cd maze_robot_ws
source install/setup.bash
ros2 launch slam_robot bringup.launch.py
```

### Step 5: Visualize (Optional)

```bash
cd linorobot2_ws
source install/setup.bash
ros2 launch linorobot2_viz slam.launch.py
```

---

## 📡 Key ROS 2 Topics

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR driver | SLAM Toolbox |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM Toolbox | Frontier Server |
| `/odom` | `nav_msgs/Odometry` | micro-ROS (ESP32) | Nav2, SLAM |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 | micro-ROS (ESP32) |
| `/get_frontiers` | `std_msgs/Empty` | Frontier Explorer | Frontier Server |
| `/frontiers` | `visualization_msgs/MarkerArray` | Frontier Server | Frontier Explorer |
| `/frontier_markers` | `visualization_msgs/MarkerArray` | Frontier Explorer | RViz2 |
| `/navigate_to_pose` | Action | Frontier Explorer | Nav2 |

---

## 🛠️ Technologies Summary

| Category | Technology |
|---|---|
| Robotics Framework | ROS 2 Humble |
| Microcontroller | ESP32 |
| Main Computer | Raspberry Pi 4 Model B |
| Firmware Framework | PlatformIO + Arduino (LinoRobot2) |
| MCU–ROS Bridge | micro-ROS (Serial / USB) |
| SLAM | SLAM Toolbox (online async) |
| Navigation | Nav2 |
| Exploration | Custom BFS + A* Frontier Detection |
| LiDAR | LDRobot LD19 (STL19P) |
| IMU | MPU6050 |
| Motor Driver | TB6612FNG Dual H-Bridge |
| Visualization | RViz2, Gazebo |
| CAD | AutoCAD |
| Chassis | Laser-cut acrylic (2-plate structure) |
| Language (firmware) | C++ |
| Language (ROS nodes) | Python 3 |

---

## 📝 Notes

- Motor 1 (Left) encoder direction is inverted (`MOTOR1_ENCODER_INV = true`) and motor direction is also inverted (`MOTOR1_INV = true`) — tuned for the physical wiring of the robot.
- The ESP32 pin `MOTOR1_IN_A` was moved from GPIO 12 to GPIO 23 to avoid boot-mode conflicts on the ESP32.
- The exploration loop terminates after **30 consecutive seconds** of no frontier detections, signaling complete maze coverage.
- The bringup launch file is configured for **simulation** (`use_sim_time: true`) and can be switched to real hardware by setting it to `false`.

---

*Built with ROS 2 Humble · linorobot2 · SLAM Toolbox · Nav2 · micro-ROS*
