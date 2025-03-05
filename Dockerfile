# Use the official ROS 2 Humble base image
FROM ros:humble

# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep update

# Create a workspace
RUN mkdir -p /home/ros2_ws/src
WORKDIR /home/ros2_ws

RUN ls

# Copy the package source code to the workspace
COPY src src/

# Install dependencies
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN . /opt/ros/humble/setup.sh && colcon build

# Set the entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /home/ros2_ws/install/setup.bash && bash"]