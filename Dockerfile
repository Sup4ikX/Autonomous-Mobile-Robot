# ESP32 LiDAR Robot Mapper - Docker Container
# Multi-stage build for development and runtime environments

# Build stage
FROM ubuntu:24.04 AS builder

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Install system dependencies
RUN apt-get update && apt-get install -y \
    wget \
    gnupg2 \
    lsb-release \
    software-properties-common \
    curl \
    build-essential \
    cmake \
    git \
    python3 \
    python3-pip \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 repository
RUN curl -sSL https://repo.ros.org/ros2/keys/ros2.key -o /usr/share/keyrings/ros2-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Install additional ROS 2 packages
RUN apt-get update && apt-get install -y \
    ros-jazzy-slam-toolbox \
    ros-jazzy-rviz2 \
    ros-jazzy-nav2-map-server \
    ros-jazzy-tf2-tools \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS 2 environment
ENV ROS_DISTRO=jazzy
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3

# Create workspace
RUN mkdir -p /opt/ros_ws/src

# Copy project files
COPY . /opt/ros_ws/src/pj_lidar/

# Set up Python environment
RUN python3 -m pip install --no-cache-dir pyyaml

# Build the project
WORKDIR /opt/ros_ws
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --event-handlers console_direct+

# Runtime stage
FROM ubuntu:24.04

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Install runtime dependencies
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 repository
RUN curl -sSL https://repo.ros.org/ros2/keys/ros2.key -o /usr/share/keyrings/ros2-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy runtime
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    ros-jazzy-slam-toolbox \
    ros-jazzy-rviz2 \
    ros-jazzy-nav2-map-server \
    ros-jazzy-tf2-tools \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS 2 environment
ENV ROS_DISTRO=jazzy
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3

# Copy built workspace from builder stage
COPY --from=builder /opt/ros_ws /opt/ros_ws

# Install Python dependencies
RUN python3 -m pip install pyyaml

# Set up entrypoint
COPY docker-entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

# Create user for non-root execution
RUN useradd -m -s /bin/bash rosuser && \
    echo "rosuser:rosuser" | chpasswd && \
    usermod -aG sudo rosuser

# Set up workspace permissions
RUN chown -R rosuser:rosuser /opt/ros_ws

# Switch to non-root user
USER rosuser

# Set working directory
WORKDIR /opt/ros_ws

# Source ROS 2 environment
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /opt/ros_ws/install/setup.bash" >> ~/.bashrc

# Expose ports for networking
EXPOSE 3333 8888 5000

# Default command
ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
CMD ["bash"]