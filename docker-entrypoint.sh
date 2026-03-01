#!/bin/bash

# ESP32 LiDAR Robot Mapper - Docker Entrypoint
# Sets up the environment and starts the container

set -e

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash
source /opt/ros_ws/install/setup.bash

# Set up environment variables
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

# Create maps directory if it doesn't exist
mkdir -p /opt/ros_ws/maps

# Function to show usage
show_usage() {
    echo "ESP32 LiDAR Robot Mapper - Docker Container"
    echo ""
    echo "Usage: docker run [OPTIONS] IMAGE [COMMAND]"
    echo ""
    echo "Commands:"
    echo "  bash              Start interactive shell (default)"
    echo "  launch            Launch all ROS2 nodes"
    echo "  setup             Run initial setup wizard"
    echo "  scan              Start scanning process"
    echo "  gui               Start GUI interface"
    echo "  monitor           Start robot monitoring"
    echo ""
    echo "Environment Variables:"
    echo "  ROS_DOMAIN_ID     ROS 2 domain ID (default: 0)"
    echo "  ROS_LOCALHOST_ONLY Only allow localhost communication (default: 0)"
    echo ""
    echo "Examples:"
    echo "  docker run -it pj_lidar:latest launch"
    echo "  docker run -it -e ROS_DOMAIN_ID=1 pj_lidar:latest scan"
    echo "  docker run -it -p 3333:3333 -p 8888:8888 pj_lidar:latest gui"
}

# Function to launch all nodes
launch_all() {
    echo "Starting ESP32 LiDAR Robot Mapper..."
    echo "ROS Domain ID: $ROS_DOMAIN_ID"
    echo "ROS Localhost Only: $ROS_LOCALHOST_ONLY"
    echo ""
    
    # Launch all ROS2 nodes
    ros2 launch pj_lidar launch_all.py
}

# Function to run setup
run_setup() {
    echo "Running initial setup wizard..."
    python3 /opt/ros_ws/src/pj_lidar/python/pj_lidar/setup_config.py
}

# Function to start scanning
start_scan() {
    echo "Starting scanning process..."
    python3 /opt/ros_ws/src/pj_lidar/python/pj_lidar/start_scan.py
}

# Function to start GUI
start_gui() {
    echo "Starting GUI interface..."
    echo "Note: GUI requires X11 forwarding for display"
    echo "Run with: docker run -it -e DISPLAY=\$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix pj_lidar:latest gui"
    python3 /opt/ros_ws/src/pj_lidar/python/pj_lidar/robot_gui.py
}

# Function to start monitoring
start_monitor() {
    echo "Starting robot monitoring..."
    python3 /opt/ros_ws/src/pj_lidar/python/pj_lidar/robot_monitor.py
}

# Main execution
case "${1:-bash}" in
    launch)
        launch_all
        ;;
    setup)
        run_setup
        ;;
    scan)
        start_scan
        ;;
    gui)
        start_gui
        ;;
    monitor)
        start_monitor
        ;;
    bash)
        exec /bin/bash
        ;;
    help|--help|-h)
        show_usage
        ;;
    *)
        echo "Unknown command: $1"
        show_usage
        exit 1
        ;;
esac