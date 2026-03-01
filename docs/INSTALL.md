# 📦 INSTALLATION AND SETUP

## System requirements

- **OS:** Ubuntu 24.04.3 LTS (recommended)
- **ROS2:** Jazzy (for Ubuntu 24.04)
- **Python:** 3.10+
- **Processor:** ARM or x86 (minimum 2 cores)
- **Dependencies:**
  ```bash
  sudo apt update
  sudo apt install -y python3-pip python3-colcon-common-extensions
  pip install pyyaml
  ```

## Step 1: Project preparation

### Option A: Project already on your PC

If you already have a copy of all project folders locally, just go to the project directory:

```bash
cd path/to/your/pj_lidar
# For example, if project is in ~/ros2_ws/src:
cd ~/ros2_ws/src/pj_lidar
```

### Option B: Download project from GitHub

If project is not yet copied to your PC:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/your/pj_lidar.git
cd pj_lidar
```

## Step 2: Install ROS 2 Jazzy (if not installed yet)

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
```

## Step 3: Install project dependencies

**Note:** This step is required for full ROS2 functionality.

```bash
# If your project is in ~/ros2_ws/src/pj_lidar:
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Or if project is in another directory:
cd /path/to/projectroot
rosdep install --from-paths . --ignore-src -r -y
```

**Main packages for Jazzy:**
```bash
sudo apt install -y \
  ros-jazzy-slam-toolbox \
  ros-jazzy-rviz2 \
  ros-jazzy-nav2-map-server \
  ros-jazzy-tf2-tools \
  ros-jazzy-serial \
  libserial-dev
```

## Step 4: Build project

**For ROS2 project:**

```bash
# If project is in ~/ros2_ws/src/pj_lidar:
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# Or if project is in another directory:
cd /path/to/projectroot
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Step 5: Initial setup

From the project root (for example `~/ros2_ws/src/pj_lidar`):

```bash
python3 python/pj_lidar/setup_config.py
```

Follow interactive wizard:
- ESP32 IP address
- TCP/UDP ports
- Navigation parameters
- Map save path

## Step 6: Flash ESP32

1. Open `firmware/robot_lidar_v2/robot_lidar_v2.ino` in Arduino IDE
2. Set board: Tools → Board → ESP32 Dev Module
3. Select COM port
4. Upload (Ctrl+U)

## Step 7: First launch

### Option 1: Full system with ROS2

```bash
# Terminal 1: Launch ROS2 nodes
ros2 launch pj_lidar launch_all.py

# Terminal 2: GUI control
python3 python/pj_lidar/robot_gui.py
```

### Option 2: Only GUI (without ROS2)

```bash
python3 python/pj_lidar/robot_gui.py
# Click "Connect" and start control
```

## Functionality check

```bash
# Check ESP32 connection
ping 192.168.4.2

# Check TCP port
nc -zv 192.168.4.2 3333

# Check ROS2 topics
ros2 topic list
ros2 topic echo /scan
ros2 topic echo /robot_status
```

## Troubleshooting

### Ubuntu 24.04 specific errors

**If colcon not found:**
```bash
sudo apt install -y python3-colcon-common-extensions
```

**If error with serial library build:**
```bash
sudo apt install -y libboost-dev libboost-system-dev libserial-dev
```

### Robot not connecting

```bash
# 1. Check WiFi (menu will open)
sudo nmtui

# 2. Check IP in network
arp-scan --localnet | grep -i esp

# 3. Check serial port (for ESP32 debugging)
minicom -b 115200 -o -D /dev/ttyUSB0

# 4. Update port access rules (if no rights)
sudo usermod -a -G dialout $USER
newgrp dialout
```

### CMake and build

```bash
# Full cleanup and rebuild
cd ~/ros2_ws
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Python dependencies

```bash
# Update pip
pip install --upgrade pip setuptools wheel

# Install dependencies manually
pip install pyyaml

# For development
sudo apt install -y python3-dev python3-venv
```

### ROS2 launch errors

```bash
# Make sure environment is sourced
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Check available nodes
ros2 node list

# Check topics
ros2 topic list

# Debug launch with logging
RCL_LOGGING_LEVELS=ros2.lidar_processor:=DEBUG ros2 launch pj_lidar launch_all.py
```

## Project structure

```
├── 📁 firmware/                    # ESP32 firmware
│   └── 📁 robot_lidar_v2/
│       └── robot_lidar_version2.ino
│
├── 📁 src/                         # C++ ROS2 nodes
│   └── lidar_processor.cpp
│
├── 📁 python/pj_lidar/             # Python ROS2 package (ROS2 nodes & tools)
│   ├── esp32_tcp_client.py
│   ├── lidar_udp_server_new.py
│   ├── scan_state_machine.py
│   ├── robot_monitor.py
│   ├── robot_controller.py
│   ├── start_scan.py
│   ├── slam_monitor.py
│   ├── robot_gui.py
│   ├── config_loader.py
│   ├── setup_config.py
│   └── config.yaml
│
├── 📁 launch/                      # ROS2 launch files
│   ├── launch_all.py
│   ├── launch_slam.py
│   └── launch_demo.py
│
├── 📁 docs/                        # Documentation
│   ├── README.md
│   ├── INSTALL.md
│   ├── QUICKSTART.md
│   ├── CHEATSHEET.md
│   ├── PROTOCOL.md
│   └── CHANGELOG.md
│
├── CMakeLists.txt                  # ROS2 build
├── package.xml                     # ROS2 manifest
├── setup.py                        # Python setup
└── README.md                       # Main documentation
```

## 📞 Support

If you have installation problems:

1. **Check system requirements** - Ubuntu 24.04.3 LTS, ROS2 Jazzy
2. **Follow step-by-step instructions** - each step is important
3. **Use "Troubleshooting" section** - typical errors and solutions
4. **Ask for help** - GitHub Issues, ROS2 Answers

## 🔄 Project update

```bash
# Update repository
cd ~/ros2_ws/src/pj_lidar
git pull

# Rebuild project
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

**✅ Installation complete! Now you can proceed to quick start.** 🚀
