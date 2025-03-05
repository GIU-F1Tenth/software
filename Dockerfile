# Use the official ROS 2 Humble base image
FROM ros:humble

# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-desktop \
    git \
    build-essential \
    cmake \
    python3-pip \
    python3-numpy \
    python3-empy \
    python3-dev \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-rosdep \
    ros-humble-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep update

# Create a workspace
RUN mkdir -p /home/ros2_ws/src
WORKDIR /home/ros2_ws

# Copy the package source code to the workspace
COPY src src/

# Install dependencies
RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN . /opt/ros/humble/setup.sh && colcon build

# Set the entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /home/ros2_ws/install/setup.bash && bash"]