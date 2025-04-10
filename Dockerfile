# Base image: ROS 2 Jazzy (Ubuntu 22.04)
FROM ros:jazzy-ros-base

# Install essential packages
RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-rclpy \
    ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install Python libraries
RUN pip install --break-system-packages modern_robotics transforms3d

# Set up workspace
WORKDIR /ros_ws/src
COPY ./src ./inverse_kinematics_pkg
WORKDIR /ros_ws

# Build your workspace
RUN . /opt/ros/jazzy/setup.sh && colcon build

# Always start bash when the container runs
ENTRYPOINT ["/bin/bash"]

