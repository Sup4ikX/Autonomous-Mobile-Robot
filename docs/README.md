# 🤖 ESP32 LiDAR Robot Mapper

**Full-featured autonomous room mapping system based on ESP32, ROS2, and SLAM Toolbox.**

---

## 📋 CONTENTS

- [Description](#description)
- [Architecture](#architecture)
- [Components](#components)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Documentation](#documentation)
- [Statistics](#statistics)

---

## 📝 DESCRIPTION

This project implements an autonomous robot capable of scanning rooms and building 2D maps using:

- **ESP32**: microcontroller with motor and LiDAR control
- **ROS2**: distributed processing architecture (state, control, monitoring)
- **SLAM Toolbox**: simultaneous localization and mapping (Simultaneous Localization and Mapping)
- **RViz2**: real-time visualization

### Key Features

✅ **Autonomous scanning** — robot determines room boundaries independently  
✅ **Obstacles** — obstacle detection and avoidance  
✅ **SLAM** — real-time map building  
✅ **TCP/UDP** — wireless control and monitoring  
✅ **GUI** — beautiful control interface (Tkinter)  
✅ **No hardcode** — all parameters in config  
✅ **Logging** — map and statistics saving  

---

## 🏗️ ARCHITECTURE

```
┌─────────────────────────────────────────────────────────┐
│                    ESP32 ROBOT                          │
│  ┌──────────────────────────────────────────────────┐  │
│  │ • Motors (PWM)                                   │  │
│  │ • LiDAR (UDP/TCP)                               │  │
│  │ • Battery monitor                              │  │
│  │ • WiFi (TCP server 3333)                       │  │
│  │ • Flash memory (Preferences)                    │  │
│  │ • Serial CLI                                    │  │
│  └──────────────────────────────────────────────────┘  │
└─────────┬───────────────────────────────────────────────┘
          │ TCP/UDP (WiFi)
          ▼
┌─────────────────────────────────────────────────────────┐
│         ROS2 SYSTEM (Laptop/Linux)                      │
├─────────────────────────────────────────────────────────┤
│                                                           │
│  📡 COMMUNICATION:                                       │
│  ├─ esp32_tcp_client.py      (TCP control)            │
│  └─ lidar_udp_server.py      (UDP LiDAR)              │
│                                                           │
│  🧠 PROCESSING:                                          │
│  ├─ lidar_processor.cpp      (SLAM integration)       │
│  ├─ scan_state_machine.py    (Autopilot)              │
│  └─ robot_monitor.py         (Diagnostics)            │
│                                                           │
│  🎨 VISUALIZATION:                                       │
│  ├─ SLAM Toolbox   (Mapping)                           │
│  ├─ RViz2          (3D visualization)                  │
│  └─ robot_gui.py   (GUI control)                       │
│                                                           │
│  ⚙️ CONFIGURATION:                                       │
│  ├─ config.yaml         (Parameters)                   │
│  ├─ config_loader.py    (Loader)                       │
│  └─ setup_config.py     (Interactive wizard)           │
│                                                           │
└─────────────────────────────────────────────────────────┘
```

---

## 🔧 COMPONENTS

### ESP32 (Microcontroller)

**File:** `robot_lidar_version2.ino`

**Functions:**
- Control two motors (PWM, direction)
- Hall sensors (rotation counting)
- LiDAR motor control (PWM)
- TCP server for commands (port 3333)
- UDP broadcast status (port 4444)
- Small TFT display (80×106) for debugging
- Save settings to Flash (Preferences)
- Serial CLI with 8+ commands

**Serial Commands:**
```
-help          Show help
-ssid_pass     Enter WiFi SSID/password
-tcp <port>    Set TCP port
-udp <port>    Set UDP port
-status        Show current settings
-connect       Connect to WiFi
-restart       Restart
```

---

### ROS2 C++ (lidar_processor)

**File:** `src/lidar_processor.cpp` (~800 lines, path relative to project root)

**Functions:**
- Read LiDAR data via UART/TCP
- Parse 7-byte LiDAR packets
- Publish `sensor_msgs/LaserScan` → `/scan`
- TF broadcast (laser → base_link) for SLAM
-- PWM control of LiDAR RPM
- Determine room scan completion
- Asynchronous map saving on completion
- Logging to `scan_complete.log`

**Parameters (from launch args):**
```yaml
tcp_port: 5000
serial_port: /dev/ttyUSB0
target_rpm: 300.0
frame_id: laser
base_frame_id: base_link
```

---

### ROS2 Python Nodes

#### 1. `esp32_tcp_client.py` (~280 lines)

**Purpose:** TCP client for ESP32 control

**Functions:**
- Connect to ESP32 (TCP 3333)
- Convert `/cmd_vel` → `set_motor_*` commands
- Publish status to `/robot_status`
- Automatic reconnection

**Inputs:**
- `/cmd_vel` (geometry_msgs/Twist)

**Outputs:**
- `/robot_status` (std_msgs/String)

---

#### 2. LiDAR UDP server (~300 lines)

**Purpose:** UDP server node for receiving LiDAR data from ESP32

**Functions:**
- Listen on configurable UDP port (see `lidar_udp_server.udp_port` in `config.yaml`)
- Parse LiDAR packets and check checksum
- Publish `/scan` (sensor_msgs/LaserScan)

---

#### 3. `scan_state_machine.py` (~380 lines)

**Purpose:** Robot autopilot

**States:**
- `IDLE` — do nothing
- `MOVING_FORWARD` — move forward
- `ROTATING` — rotate 360°
- `OBSTACLE_DETECTED` — obstacle detected, stop
- `SCAN_COMPLETE` — scan completed

**Logic:**
1. Start with rotation (overview)
2. If obstacle → stop
3. If obstacle gone → rotate
4. Count full rotations
5. After N rotations → completion

---

#### 4. `robot_monitor.py` (~220 lines)

**Purpose:** Monitoring and diagnostics publishing

**Functions:**
- Parse `/robot_status` from ESP32
- Publish `/diagnostics` (diagnostic_msgs/DiagnosticArray)
- Check component health (motors, battery, LiDAR, WiFi)

---

#### 5. `robot_gui.py` (~650 lines)

**Purpose:** Control GUI (Tkinter)

**Functions:**
- Connect/disconnect from ESP32
- Monitor status (battery %, motor speeds)
- Control LiDAR PWM
- Request robot status
- Real-time logging

**Color scheme:**
```
#05386B (dark blue) — background
#379683 (sea)       — header
#5CDB95 (green)     — buttons
#EDF5E1 (cream)     — text
```

---

### Configuration

#### `config.yaml`

**MAIN config!** All key parameters are stored here (values below are examples, adjust for your setup):

```yaml
esp32_tcp_client:
  # "auto" = try to discover ESP32 in the network
  robot_ip: "auto"
  tcp_port: 3333
  timeout: 5.0

lidar_udp_server:
  # Must match UDP port used in ESP32 firmware
  udp_port: 4444
  range_min: 0.15
  range_max: 12.0
  frame_id: "laser"

scan_state_machine:
  obstacle_distance: 0.5
  max_forward_speed: 0.3
  rotation_speed: 0.5
  enable_auto_mode: false

slam_toolbox:
  map_save_path: "~/maps"
  auto_save: true
```

#### `config_loader.py`

Config loader with nested keys and fallback values support.

#### `setup_config.py`

Interactive wizard for initial setup. Asks all parameters and creates `config.yaml`.

---

## 📦 INSTALLATION

### Requirements

- **ESP32** (DevKit, 4MB Flash)
- **LiDAR** (YDLiDAR TL-16, or compatible)
- **Motors** (2× DC with gearbox)
- **Laptop** with ROS2 (Ubuntu 24.04.3 LTS with ROS2 Jazzy)

### On ESP32

1. Open `robot_lidarMYVERSION.ino` in Arduino IDE
2. Select board: `ESP32 Dev Module`
3. Upload (Flash)

### On Laptop (ROS2 Jazzy)

```bash
# Add ROS 2 repository (for Ubuntu 24.04)
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://repo.ros.org/ros2/keys/ros2.key -o /usr/share/keyrings/ros2-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

# Install ROS 2 Jazzy
sudo apt install -y ros-jazzy-desktop
sudo apt install -y ros-dev-tools

# Install dependencies
sudo apt install -y \
  ros-jazzy-slam-toolbox \
  ros-jazzy-rviz2 \
  ros-jazzy-nav2-map-server \
  ros-jazzy-tf2-tools \
  python3-colcon-common-extensions \
  python3-pip

# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/your/pj_lidar.git

# Build project
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# Activate
source install/setup.bash

# Install Python dependencies
pip install pyyaml
```

---

## 🚀 QUICK START

### 1️⃣ Initial setup (once)

```bash
python setup_config.py
# Answer questions → config.yaml created
```

### 2️⃣ Launch ROS2 nodes

```bash
ros2 launch pj_lidar launch_all.py
```

Should start:
- `esp32_tcp_client` — ESP32 connection
- `lidar_udp_server` — LiDAR reception
- `scan_state_machine` — autopilot
- `robot_monitor` — diagnostics
- `slam_toolbox` — mapping
- `rviz2` — visualization

### 3️⃣ Launch GUI

```bash
python robot_gui.py
```

Click **"✓ Connect"** → robot ready!

### 4️⃣ Start scanning

```bash
python start_scan.py
# Enter: start
```

**Robot will start rotating and moving, building a map!**

---

## 📚 DOCUMENTATION

| File | Content |
|------|---------|
| `cheatsheet.md` | Cheat sheet (commands, topics) |
| `README.md` | THIS BOOK |

### How to change parameters?

**Option 1: Reconfigure**
```bash
python setup_config.py
```

**Option 2: Edit config.yaml**
```bash
code config.yaml
# Change robot_ip, tcp_port etc.
```

### How to add your own node?

1. Create `python/pj_lidar/my_node.py` inside this project (for example, in `~/ros2_ws/src/pj_lidar/python/pj_lidar/`).
2. Add to `launch/launch_all.py`:
   ```python
   Node(package='pj_lidar', executable='my_node', name='my_node')
   ```
3. Build and run (usual `colcon build` / `ros2 launch ...`)

---

## 📊 STATISTICS

| Metric | Value |
|--------|-------|
| **Lines of code** | ~6,240 |
| **Files** | 24 |
| **Languages** | C++ (40%), Python (60%) |
| **ROS2 nodes** | 7 |
| **Documentation** | 100% |
| **Production-ready** | ✅ Yes |

### Commands and functions

- **Serial:** 8 commands on ESP32
- **TCP:** 12 control commands
- **ROS2 topics:** 6 main
- **State Machine:** 5 states
- **GUI functions:** 10+ buttons and panels

---

## 🎯 USAGE EXAMPLES

### Start scanning from Python

```python
import rclpy
from std_msgs.msg import String

node = rclpy.create_node('scan_starter')
pub = node.create_publisher(String, 'scan_command', 10)

msg = String()
msg.data = 'START_SCAN'
pub.publish(msg)
```

### Send TCP command

```bash
echo "set_motor_left:128" | nc 192.168.4.2 3333
```

### View LiDAR data

```bash
ros2 topic echo /scan
```

---

## 🐛 DEBUGGING

### Robot not connecting

```bash
# Check IP
ping 192.168.4.2

# Check TCP port
nc -zv 192.168.4.2 3333

# View Serial logs (Arduino IDE)
```

### LiDAR not publishing data

```bash
# Check UDP port
ros2 topic echo /scan
# If empty → check UDP socket on ESP32
```

### Map not building

```bash
# Check TF
ros2 run tf2_tools view_frames.py

# Check SLAM status
ros2 topic echo /map
```

---

## 📝 LICENSE

GNU Affero General Public License (AGPL) - any modifications must be provided as open source

---

## 👨‍💻 AUTHOR

sup4ikX and sup4ikY

---

## 📞 SUPPORT

- **Documentation:** See `cheatsheet.md`
- **Examples:** In code files (comments development in each method)
- **Questions:** Use ROS2 Answers or GitHub Issues

---

**Ready to sketch! 🚀**
