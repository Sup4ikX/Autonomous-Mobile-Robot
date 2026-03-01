# ESP32 LiDAR Robot Mapper - Makefile
# Provides convenient commands for building, testing, and deploying

.PHONY: help build install clean deploy docker-build docker-run docker-clean all test lint format

# Default target
all: help

# Show help
help:
	@echo "ESP32 LiDAR Robot Mapper - Make Commands"
	@echo ""
	@echo "Build Commands:"
	@echo "  make build      - Build the ROS2 project"
	@echo "  make install    - Install and setup the project"
	@echo "  make clean      - Clean build artifacts"
	@echo ""
	@echo "Docker Commands:"
	@echo "  make docker-build    - Build Docker image"
	@echo "  make docker-run      - Run Docker container"
	@echo "  make docker-clean    - Clean Docker containers and images"
	@echo ""
	@echo "Development Commands:"
	@echo "  make test       - Run tests"
	@echo "  make lint       - Run linting"
	@echo "  make format     - Format code"
	@echo ""
	@echo "Utility Commands:"
	@echo "  make deploy     - Deploy using deploy.sh"
	@echo "  make setup      - Run initial setup"
	@echo "  make launch     - Launch all ROS2 nodes"
	@echo "  make scan       - Start scanning process"
	@echo "  make gui        - Start GUI interface"
	@echo ""

# Build the project
build:
	@echo "Building ESP32 LiDAR Robot Mapper..."
	./deploy.sh build

# Install the project
install:
	@echo "Installing ESP32 LiDAR Robot Mapper..."
	./deploy.sh install

# Clean build artifacts
clean:
	@echo "Cleaning build artifacts..."
	./deploy.sh clean

# Deploy using deploy.sh
deploy:
	@echo "Deploying project..."
	./deploy.sh install

# Setup initial configuration
setup:
	@echo "Running initial setup..."
	python3 python/pj_lidar/setup_config.py

# Launch all ROS2 nodes
launch:
	@echo "Launching all ROS2 nodes..."
	ros2 launch pj_lidar launch_all.py

# Start scanning process
scan:
	@echo "Starting scanning process..."
	python3 python/pj_lidar/start_scan.py

# Start GUI interface
gui:
	@echo "Starting GUI interface..."
	python3 python/pj_lidar/robot_gui.py

# Build Docker image
docker-build:
	@echo "Building Docker image..."
	docker build -t pj_lidar:latest .

# Run Docker container
docker-run:
	@echo "Running Docker container..."
	docker run -it \
		--name pj_lidar_container \
		-p 3333:3333 \
		-p 8888:8888 \
		-v $(PWD)/maps:/opt/ros_ws/maps \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-e DISPLAY=$(DISPLAY) \
		pj_lidar:latest

# Clean Docker containers and images
docker-clean:
	@echo "Cleaning Docker containers and images..."
	docker stop pj_lidar_container 2>/dev/null || true
	docker rm pj_lidar_container 2>/dev/null || true
	docker rmi pj_lidar:latest 2>/dev/null || true

# Run tests
test:
	@echo "Running tests..."
	cd python && python3 -m pytest tests/ -v

# Run linting
lint:
	@echo "Running linting..."
	@echo "Note: Install pylint/flake8 if not available"
	python3 -m pylint python/pj_lidar/ --disable=C0103,R0903 2>/dev/null || echo "pylint not available, install with: pip install pylint"

# Format code
format:
	@echo "Formatting code..."
	@echo "Note: Install black if not available"
	python3 -m black python/pj_lidar/ 2>/dev/null || echo "black not available, install with: pip install black"

# Create maps directory
maps:
	@mkdir -p maps

# Create backups directory
backups:
	@mkdir -p backups

# Create web interface directory
web:
	@mkdir -p web_interface

# Setup development environment
dev-setup: maps backups web
	@echo "Setting up development environment..."
	@echo "Installing development dependencies..."
	pip3 install pytest pylint black flake8

# Check prerequisites
check:
	@echo "Checking prerequisites..."
	@command -v ros2 >/dev/null 2>&1 || { echo "ROS2 not found. Please install ROS2 Jazzy."; exit 1; }
	@command -v colcon >/dev/null 2>&1 || { echo "colcon not found. Please install colcon."; exit 1; }
	@command -v docker >/dev/null 2>&1 || echo "Docker not found. Docker features will not be available."
	@echo "Prerequisites check completed."

# Show project status
status:
	@echo "Project Status:"
	@echo "Workspace: $(HOME)/ros2_ws"
	@echo "Project: pj_lidar"
	@echo ""
	@echo "Available commands:"
	@echo "  make build      - Build project"
	@echo "  make install    - Install project"
	@echo "  make launch     - Launch ROS2 nodes"
	@echo "  make scan       - Start scanning"
	@echo "  make gui        - Start GUI"
	@echo ""
	@echo "Docker commands:"
	@echo "  make docker-build - Build Docker image"
	@echo "  make docker-run   - Run Docker container"
	@echo ""
	@echo "Development commands:"
	@echo "  make test       - Run tests"
	@echo "  make lint       - Run linting"
	@echo "  make format     - Format code"
	@echo "  make dev-setup  - Setup development environment"