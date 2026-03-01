# ⚡ QUICK START: POWER ON AND SCAN (5 minutes)

## If everything is already configured:

### Terminal 1: Launch ROS2
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pj_lidar launch_all.py robot_ip:=192.168.1.100
```

### Terminal 2: Open GUI
```bash
python3 ~/ros2_ws/src/pj_lidar/python/pj_lidar/robot_gui.py
```

**In GUI:** Click "✓ Connect"

### Terminal 3: Start scanning
```bash
cd ~/ros2_ws/src/pj_lidar
python3 python/pj_lidar/start_scan.py
# Enter: start
```

### Result:
- 🤖 Robot will start rotating
- 🗺️ RViz2 (in Terminal 1) will show real-time map
- ✅ After 1-3 min. scanning will complete
- 💾 Map saved to ~/maps/

---

## If FIRST TIME:

1. **Flash ESP32** → `firmware/robot_lidar_v2/robot_lidar_v2.ino` in Arduino IDE
2. **Install ROS2 Jazzy** → Add repository for Ubuntu 24.04:
   ```bash
   sudo apt update
   sudo apt install -y software-properties-common
   sudo add-apt-repository universe
   sudo curl -sSL https://repo.ros.org/ros2/keys/ros2.key -o /usr/share/keyrings/ros2-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update
   sudo apt install -y ros-jazzy-desktop
   ```
3. **Build project** → `colcon build` in ~/ros2_ws
4. **Create config** → from project root:
   ```bash
   cd ~/ros2_ws/src/pj_lidar
   python3 python/pj_lidar/setup_config.py
   ```
5. **Follow steps above** 👆

---

**READY! Scanning will start in 30 seconds.** 🚀
