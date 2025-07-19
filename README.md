# F1TENTH Autonomous Racing Stack - GIU Berlin

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS 2 Humble](https://img.shields.io/badge/ROS-2%20Humble-blue.svg)](https://docs.ros.org/en/humble/)

## Overview

This repository contains the complete autonomous racing software stack for F1TENTH vehicles developed by **GIU Berlin**. The stack provides all necessary components for autonomous racing including SLAM, path planning, control, perception, and safety systems.

The F1TENTH platform is a 1/10th scale autonomous racing car designed for research and education in autonomous systems. This software stack enables the vehicle to navigate autonomously in racing environments using LiDAR sensors, cameras, and other onboard sensors.

## Architecture

The software is organized into several key modules:

```
src/
├── control/          # Vehicle control algorithms
├── drivers/          # Hardware drivers and interfaces  
├── giu_f1t_interfaces/  # Custom message definitions
├── giu_f1t_system/   # Core system integration
├── giu_f1t_utils/    # Utility functions and tools
├── perception/       # Object detection and tracking
├── planning/         # Path planning and decision making
└── slam/            # Simultaneous Localization and Mapping
```

## Core Components

### 🚗 Control System

- **`safety_node`**: Automatic emergency braking system for collision avoidance
- **`mpc_controller`**: Model Predictive Controller for optimal vehicle control
- **`pid_controller`**: PID-based control for basic vehicle operations
- **`watchdog`**: System monitoring and failsafe mechanisms

### 🛰️ Planning & Navigation

- **`pure_pursuit`**: Pure pursuit path following controller with adaptive lookahead
- **`gap_follower`**: Gap-based reactive navigation for obstacle avoidance
- **`trajectory_planning`**: Advanced trajectory generation and optimization
- **`giu_f1t_behavior_tree`**: Behavior tree-based decision making system
- **`navigation2`**: Integration with ROS 2 Navigation stack

### 👁️ Perception

- **`obj_detection`**: Real-time object detection using computer vision
- **Clustering algorithms**: Point cloud processing for object identification

### 🗺️ SLAM & Mapping

- **`frontier_exploration`**: Autonomous exploration for unknown environments
- **`nav2_wfe`**: Wavefront exploration algorithms
- **Integration with Nav2 SLAM**: Real-time mapping and localization

### 🔧 Drivers & Hardware

- **`realsense-ros`**: Intel RealSense camera drivers
- **`urg_node`**: Hokuyo LiDAR drivers
- **`vesc`**: VESC motor controller interface
- **`f1tenth_stack`**: Core hardware integration and bringup

### 📚 Libraries

- **`frenet_optimal_trajectory_planner`**: Frenet-based trajectory optimization
- **`raceline-optimization`**: Racing line computation and optimization
- **`reference_line_generation`**: Reference path generation tools

## Prerequisites

### System Requirements

- **Operating System**: Ubuntu 22.04 LTS
- **ROS Distribution**: ROS 2 Humble
- **Python**: 3.10+
- **Hardware**: F1TENTH vehicle with LiDAR and camera sensors

### Dependencies

#### System Dependencies

Install ROS 2 Humble and required packages:

```bash
# Install ROS 2 Humble (if not already installed)
sudo apt update
sudo apt install ros-humble-desktop

# Install additional ROS 2 packages
sudo apt install -y \
    ros-humble-ackermann-msgs \
    ros-humble-urg-node \
    ros-humble-joy \
    ros-humble-joy-teleop \
    ros-humble-rosbridge-server \
    ros-humble-nav2-bringup \
    ros-humble-realsense2-camera \
    ros-humble-vision-msgs \
    python3-colcon-common-extensions \
    python3-pip
```

#### Python Dependencies

All Python dependencies are managed through a comprehensive `requirements.txt` file:

```bash
# Install all Python dependencies at once
pip3 install -r requirements.txt
```

The requirements.txt includes:
- **Core Libraries**: NumPy, SciPy, Matplotlib, Pandas
- **Computer Vision**: OpenCV, scikit-image  
- **Machine Learning**: scikit-learn, HDBSCAN
- **Deep Learning**: Ultralytics YOLO, PyTorch
- **Hardware Drivers**: PyRealSense2, GxIPy (industrial cameras)
- **Optimization**: CasADi (for MPC controller)
- **Data Processing**: PyYAML, Roboflow API

#### Optional: Conda Environment for Racing Line Optimization

For optimal performance with racing line optimization, create a dedicated conda environment:

```bash
# Create conda environment
conda create -n raceline python=3.10
conda activate raceline

# Install optimized packages via conda
conda install -c conda-forge casadi matplotlib numpy scipy

# Install remaining packages via pip
pip install -r requirements.txt
```

## Getting Started

### 1. Clone and Build

```bash
# Create workspace
mkdir -p ~/f1tenth_ws/src
cd ~/f1tenth_ws/src

# Clone the repository
git clone <repository-url> software
cd ~/f1tenth_ws

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

### 2. Vehicle Setup

On the F1TENTH vehicle, source the ROS 2 installation:

```bash
source ~/ros_humble/install/setup.bash
```

### 3. Launch the System

#### Basic Bringup
Start the core vehicle systems:

```bash
ros2 launch f1tenth_stack bringup_launch_raw.py
```

#### SLAM Mode
For mapping unknown environments:

```bash
ros2 launch f1tenth_stack bringup_launch_slam.py
ros2 launch nav2_bringup slam_launch.py params_file:=src/giu_f1t_system/f1tenth_stack/config/f1tenth_online_async_mapping.yaml
```

For saving the map:
```bash
ros2 run nav2_map_server map_saver_cli
```

#### Racing Mode
For autonomous racing with pre-built maps:

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

### 4. Teleoperation

Control the vehicle manually using a gamepad:

```bash
ros2 launch f1tenth_stack bringup_launch_raw.py

# Use gamepad or keyboard for manual control on your device 
# Example: 
ros2 run joy joy_node
```

## Advanced Usage

### SLAM and Mapping

1. **Start SLAM**:
   ```bash
   ros2 launch nav2_bringup slam_launch.py params_file:=src/giu_f1t_system/f1tenth_stack/config/f1tenth_online_async_mapping.yaml
   ```

2. **Save Map**:
   ```bash
   ros2 run nav2_map_server map_saver_cli
   ```

3. **Transfer Map** (from vehicle to host):
   ```bash
   scp ubuntu@[vehicle_ip]:/home/ubuntu/giu_f1tenth_ws/software/map.pgm ~/maps/
   ```

### Racing Line Optimization

1. **Setup Conda Environment**:
   ```bash
   source ~/miniconda3/etc/profile.d/conda.sh
   conda activate raceline
   ```

2. **Generate Optimal Racing Line**:
   ```bash
   cd ~/software/libs/raceline-optimization
   python3 main_globaltraj_f110.py --map_name track_name --map_path ~/maps/track.csv --export_path /tmp/optimal_raceline.csv
   ```

### Camera Configuration

For object detection and perception:

```bash
# Basic camera setup
ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:=640x480x30 enable_infra1:=false enable_infra2:=false enable_depth:=false enable_gyro:=false enable_accel:=false

# Optimized camera setup for racing
ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true \
    color_width:=320 \
    color_height:=240 \
    color_fps:=6 \
    enable_depth:=false \
    enable_infra1:=false \
    enable_infra2:=false \
    enable_accel:=false \
    enable_gyro:=false \
    enable_pose:=false \
    enable_sync:=false \
    unite_imu_method:="none"
```

## Building Specific Components

Build only the components you need:

```bash
# Core racing components
colcon build --packages-select f1tenth_stack pure_pursuit gap_follower trajectory_planning

# Full autonomous stack
colcon build --packages-select f1tenth_stack camera_obj_detection pure_pursuit gap_follower giu_f1t_behavior_tree watchdog trajectory_planning obj_detection opponent_tracker safety_node

# SLAM components
colcon build --packages-select f1tenth_stack slam_toolbox nav2_bringup
```

## Configuration

Configuration files are located in:
- `src/giu_f1t_system/f1tenth_stack/config/` - Main system configurations
- Individual package `config/` directories - Component-specific settings

Key configuration files:
- `f1tenth_online_async_mapping.yaml` - SLAM parameters
- Vehicle-specific parameters for controllers and sensors

## Component-Specific Dependencies

Different components have specific dependency requirements:

### 🎯 Object Detection & Perception
- **YOLO Detection**: `ultralytics`, `torch`, `torchvision`
- **Camera Processing**: `opencv-python`, `pyrealsense2`
- **Industrial Cameras**: `gxipy` (Daheng Imaging cameras)
- **Clustering**: `scikit-learn`, `hdbscan`
- **Annotation Platform**: `roboflow`

### 🧠 Control Systems
- **MPC Controller**: `casadi` (optimization), `numpy`, `pandas`
- **Safety Node**: Standard ROS 2 + `numpy`
- **Watchdog**: Standard ROS 2 libraries

### 🗺️ Planning & Navigation
- **Pure Pursuit**: `numpy`, `csv processing`
- **Trajectory Planning**: `scipy`, `matplotlib`
- **Racing Line Optimization**: `casadi`, `trajectory_planning_helpers`
- **Gap Follower**: `numpy`, Standard ROS 2

### 📊 SLAM & Mapping
- **Standard Nav2**: Included with ROS 2 installation
- **Custom Algorithms**: `numpy`, `scipy`

### 🔧 Hardware Drivers
- **LiDAR**: `urg_node` (ROS 2 package)
- **RealSense**: `pyrealsense2`, `realsense2_camera` (ROS 2)
- **VESC**: Custom ROS 2 messages and drivers

### ⚡ Performance Notes
- **GPU Acceleration**: YOLO models benefit from CUDA-enabled PyTorch
- **Real-time Requirements**: Some components require RT kernel for best performance
- **Memory Usage**: Object detection models require ~2-4GB GPU memory

## Monitoring and Debugging

### System Monitoring
```bash
# Monitor system performance (on Jetson)
sudo tegrastats

# View ROS 2 topics
ros2 topic list

# Monitor specific topics
ros2 topic echo /scan
ros2 topic echo /odom
```

### Visualization
Use RViz2 for real-time visualization:

```bash
ros2 run rviz2 rviz2
```

## Development and Contribution

### Project Structure

The codebase follows ROS 2 conventions with proper package organization. Each component is self-contained with its own:
- `package.xml` - Package metadata and dependencies
- `setup.py` - Python package configuration
- `launch/` - Launch files for easy deployment
- `config/` - Configuration parameters

### Adding New Components

1. Create new package in appropriate directory (`src/control/`, `src/planning/`, etc.)
2. Follow ROS 2 package structure
3. Add dependencies to `package.xml`
4. Create launch files for integration
5. Update this README with component description

### Testing

```bash
# Run tests for specific packages
colcon test --packages-select package_name

# Run all tests
colcon test
```

## Troubleshooting

### Common Issues

1. **Build Failures**: Ensure all dependencies are installed and ROS 2 is properly sourced
2. **Sensor Connection**: Check hardware connections and driver installations
3. **Performance Issues**: Monitor CPU/GPU usage with `tegrastats` on Jetson platforms
4. **Network Issues**: Verify ROS_DOMAIN_ID and network configuration for multi-machine setups

### Getting Help

- Check individual package documentation in their respective directories
- Review ROS 2 logs: `ros2 run tf2_tools view_frames.py`
- Monitor system resources and network connectivity

## Hardware Requirements

### F1TENTH Vehicle Components

- **Chassis**: 1/10 scale racing car platform
- **Compute**: NVIDIA Jetson (recommended) or equivalent
- **LiDAR**: Hokuyo UST-10LX or similar 2D LiDAR
- **Camera**: Intel RealSense D435 or similar RGB-D camera
- **Motor Controller**: VESC-based speed controller
- **Servo**: High-frequency steering servo
- **Battery**: LiPo battery pack suitable for the platform

### Sensor Specifications

- **LiDAR**: 270° field of view, 40m range, 40Hz update rate
- **Camera**: RGB camera with minimum 30fps, 640x480 resolution
- **IMU**: 6DOF or 9DOF inertial measurement unit
- **Odometry**: Wheel encoders or visual odometry system

## Performance Characteristics

The system is designed for:
- **Real-time operation**: < 100ms control loop latency
- **High-speed racing**: Up to 8 m/s (scaled to ~80 mph at full scale)
- **Robust navigation**: Reliable operation in dynamic environments
- **Safety-first design**: Multiple redundant safety systems

## Research and Education

This stack is designed for:
- **Research**: Algorithm development and testing in autonomous racing
- **Education**: Teaching autonomous systems concepts
- **Competition**: F1TENTH racing competitions
- **Industry**: Rapid prototyping of autonomous vehicle algorithms

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- F1TENTH Foundation for the platform specifications
- ROS 2 community for the underlying framework
- Navigation2 project for SLAM and navigation capabilities
- GIU Berlin Autonomous Systems Lab

## Contact and Support

For questions, issues, or contributions:
- Create an issue in this repository
- Contact the GIU Berlin Autonomous Systems team
- Refer to individual package maintainers listed in `package.xml` files

---

**Happy Racing! 🏎️**