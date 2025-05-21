# ROS2 Mapping with Depth Camera

A ROS2 package that implements 2D mapping using RGB-D camera data with RTAB-Map SLAM.

## Overview

This package provides launch files and configurations to create 2D occupancy grid maps using a depth camera on a robot platform. It utilizes RTAB-Map's SLAM capabilities to process RGB and depth data to generate accurate maps for navigation.

## Features

* 2D mapping using RGB-D camera data
* Integration with RTAB-Map for SLAM
* Compatible with optional LiDAR data
* Preconfigured RViz visualization
* Support for both mapping and localization modes

## Requirements

* ROS2 (tested on Humble)
* rtabmap_ros package
* RGB-D camera (e.g., Intel RealSense, Kinect, or similar)
* Robot with odometry (physical or simulated)

## Installation

1. Create a directory for the project and clone this repository into your ROS2 workspace's `src` directory:

```bash
cd ~/your_ros2_ws/src
mkdir -p 2d_mapping
cd 2d_mapping
git clone https://github.com/InterplanetarCodebase/ros2_mapping_with_depthcamera.git .
```

2. Install dependencies:

```bash
cd ~/your_ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the workspace:

```bash
colcon build --symlink-install
```

4. Source the workspace:

```bash
source ~/your_ros2_ws/install/setup.bash
```

## Usage

Launch the mapping system:

```bash
ros2 launch depth_camera_mapping robot_rtabmap.launch.py
```

Launch parameters:
* `use_sim_time`: Set to 'true' when using simulation (default: 'false')
* `qos`: QoS profile for sensor data (default: '1')
* `localization`: Use RTAB-Map in localization mode (default: 'false')

Example with parameters:

```bash
ros2 launch depth_camera_mapping robot_rtabmap.launch.py use_sim_time:=true localization:=true
```

## Topics

Subscribed Topics:
* `/camera/color/image_raw`: RGB image from camera
* `/camera/color/camera_info`: Camera calibration info
* `/camera/depth/image_raw`: Depth image from camera
* `/odometry/filtered`: Robot odometry
* `/scan`: LaserScan data (optional, for LiDAR integration)

Published Topics:
* `/map`: 2D occupancy grid map

## Configuration

The system is configured to create 2D maps with the following parameters:
* 5cm grid cell size
* 5m maximum range
* 0.1m maximum ground height
* 2.0m maximum obstacle height

These parameters can be adjusted in the launch file as needed.

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.

## Author

Abul Hasnat Abdullah

## Acknowledgments

* RTAB-Map developers
* ROS2 community
