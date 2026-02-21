# Rover Autonav

Autonomous navigation for a 6-wheel differential drive rover using a ZED 2i camera, Nav2, and Foxglove visualization.

## Prerequisites

- Jetson AGX Orin with JetPack 5
- ROS 2 Humble
- ZED SDK (from [Stereolabs](https://www.stereolabs.com/developers/release))

## Setup

```bash
source /opt/ros/humble/setup.bash
./setup.sh
```

## Usage

```bash
source install/setup.bash

# Full launch (with motors)
ros2 launch rover_autonav autonav.launch.py

# Without motor driver
ros2 launch rover_autonav autonav.launch.py enable_motors:=false
```

Connect Foxglove to `ws://<jetson-ip>:8765`. Click in the 3D panel to send navigation goals.

## Architecture

```
ZED 2i (VIO + PointCloud2) → Nav2 voxel costmaps (height-aware 2.5D) → /cmd_vel → Motor Driver
                                                                                  ↓
                                        Foxglove Bridge ← all topics ← 6x motor nodes
```

- **Odometry:** ZED visual-inertial odometry (no wheel encoders needed)
- **Planning:** Nav2 with NavFn planner + DWB differential drive controller
- **Costmaps:** Rolling window, fed by ZED point cloud through Nav2 voxel layers
- **Visualization:** Foxglove bridge with topic whitelist (~15-25 Mbps)
