#!/bin/bash

# ESP32 LiDAR Robot Mapper - Deployment Script
# Usage: ./deploy.sh [build|install|clean]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_NAME="pj_lidar"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

check_ros2() {
    if ! command -v ros2 &> /dev/null; then
        log_error "ROS2 is not installed or not in PATH"
        log_info "Please install ROS2 Jazzy first:"
        log_info "https://docs.ros.org/en/jazzy/Installation.html"
        exit 1
    fi
    
    if [ "$ROS_DISTRO" != "jazzy" ]; then
        log_warning "ROS2 distribution is not Jazzy (current: $ROS_DISTRO)"
        log_info "This project is tested with ROS2 Jazzy on Ubuntu 24.04.3 LTS"
    fi
}

check_dependencies() {
    log_info "Checking dependencies..."
    
    # Check if colcon is available
    if ! command -v colcon &> /dev/null; then
        log_error "colcon is not installed"
        log_info "Install with: sudo apt install python3-colcon-common-extensions"
        exit 1
    fi
    
    # Check Python dependencies
    python3 -c "import pyyaml" 2>/dev/null || {
        log_warning "pyyaml not found, installing..."
        pip3 install pyyaml
    }
    
    log_success "Dependencies check completed"
}

setup_workspace() {
    log_info "Setting up ROS2 workspace..."
    
    # Create workspace if it doesn't exist
    if [ ! -d "$HOME/ros2_ws" ]; then
        mkdir -p "$HOME/ros2_ws/src"
        log_success "Created workspace directory: $HOME/ros2_ws/src"
    fi
    
    # Copy project to workspace
    if [ -d "$HOME/ros2_ws/src/$PROJECT_NAME" ]; then
        log_warning "Project already exists in workspace, removing..."
        rm -rf "$HOME/ros2_ws/src/$PROJECT_NAME"
    fi
    
    cp -r "$SCRIPT_DIR" "$HOME/ros2_ws/src/$PROJECT_NAME"
    log_success "Copied project to workspace"
}

build_project() {
    log_info "Building project with colcon..."
    
    cd "$HOME/ros2_ws"
    
    # Source ROS2 setup
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
    else
        log_warning "ROS2 setup.bash not found at /opt/ros/jazzy/setup.bash"
        log_info "Make sure ROS2 is properly installed"
    fi
    
    # Clean previous build
    if [ -d "build" ] || [ -d "install" ] || [ -d "log" ]; then
        log_info "Cleaning previous build..."
        rm -rf build install log
    fi
    
    # Build the project
    colcon build --symlink-install --event-handlers console_direct+
    
    if [ $? -eq 0 ]; then
        log_success "Build completed successfully"
    else
        log_error "Build failed"
        exit 1
    fi
}

install_project() {
    log_info "Installing project..."
    
    cd "$HOME/ros2_ws"
    
    # Source the workspace
    source install/setup.bash
    
    # Test that nodes are available
    if ros2 node list &> /dev/null; then
        log_success "Project installed and ready to use"
        log_info "Available commands:"
        log_info "  ros2 launch $PROJECT_NAME launch_all.py"
        log_info "  python3 start_scan.py"
        log_info "  python3 setup_config.py"
    else
        log_error "Installation verification failed"
        exit 1
    fi
}

clean_workspace() {
    log_info "Cleaning workspace..."
    
    cd "$HOME/ros2_ws"
    
    # Remove build artifacts
    rm -rf build install log
    
    log_success "Workspace cleaned"
}

show_help() {
    echo "ESP32 LiDAR Robot Mapper - Deployment Script"
    echo ""
    echo "Usage: $0 [COMMAND]"
    echo ""
    echo "Commands:"
    echo "  build     Build the project"
    echo "  install   Install and setup the project"
    echo "  clean     Clean build artifacts"
    echo "  help      Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 build"
    echo "  $0 install"
    echo "  $0 clean"
}

main() {
    case "${1:-help}" in
        build)
            check_ros2
            check_dependencies
            setup_workspace
            build_project
            ;;
        install)
            check_ros2
            check_dependencies
            setup_workspace
            build_project
            install_project
            ;;
        clean)
            clean_workspace
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            log_error "Unknown command: $1"
            show_help
            exit 1
            ;;
    esac
}

# Run main function with all arguments
main "$@"