# Rover Autonav System Design

**Date:** 2026-02-21
**Status:** Approved

## Overview

Full autonomous navigation stack for a 6-wheel differential drive URC-scale rover using a ZED 2i camera on a Jetson AGX Orin, with visualization over Foxglove.

## Rover Specifications

- 6-wheel differential drive (left: 0,2,4 / right: 1,3,5)
- ~3 inch wheel radius, ~27 inch track width (~0.686m)
- No wheel encoders, no GPS
- ZED 2i stereo camera (built-in IMU)
- Jetson AGX Orin, ROS 2 Humble
- ~40 Mbps Foxglove connection bandwidth
- Indoor + outdoor operation

## Architecture: ZED VIO + Nav2 + Foxglove Bridge

### Package Structure

```
ros_ws/
├── src/
│   ├── rover_autonav/              # Custom package
│   │   ├── config/
│   │   │   ├── nav2_params.yaml    # Nav2 costmaps, planners, controllers
│   │   │   ├── zed_config.yaml     # ZED camera overrides
│   │   │   └── foxglove.yaml       # Topic whitelist
│   │   ├── description/
│   │   │   └── rover.urdf.xacro    # 6-wheel diff-drive URDF
│   │   ├── launch/
│   │   │   ├── autonav.launch.py   # Master launch
│   │   │   ├── zed.launch.py       # ZED camera + VIO
│   │   │   ├── nav2.launch.py      # Nav2 stack
│   │   │   └── foxglove.launch.py  # Foxglove bridge
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   └── zed-ros2-wrapper/           # Cloned dependency
├── setup.sh
└── README.md
```

### Odometry & TF Tree

ZED 2i visual-inertial odometry (VIO) provides both odometry and SLAM:

- `pos_tracking_enabled: true`, `imu_fusion: true`, `area_memory: true`
- ZED publishes `map → odom` (with loop closure) and `odom → base_link`
- No separate SLAM node needed (ZED area memory handles drift correction)

```
map (ZED positional tracking)
 └── odom (ZED VIO)
      └── base_link
           ├── left_wheel_{front,mid,rear}_link
           ├── right_wheel_{front,mid,rear}_link
           └── zed2i_camera_link
```

### Nav2 Configuration

**Costmaps:**
- Global: Rolling window 50m x 50m, 0.05m resolution, obstacle + inflation layers
- Local: Rolling window 5m x 5m, voxel + inflation layers, 5 Hz update
- Robot footprint: Rectangular polygon ~0.9m x 0.7m

**Depth to LaserScan:**
- `depthimage_to_laserscan` converts ZED depth → virtual `/scan` for costmap obstacle layers
- ~120 degree FOV horizontal slice at rover height

**Planner:** NavFn (Dijkstra/A*)
**Controller:** DWB differential drive
- max_vel_x: 0.5 m/s, min_vel_x: 0.05 m/s
- max_vel_theta: 1.0 rad/s
- No lateral velocity (differential drive)

**Behavior tree:** Default `navigate_to_pose` with spin/backup/wait recovery

### Foxglove Visualization & Bandwidth

**Bandwidth strategy (~15-25 Mbps, within 40 Mbps budget):**
- ZED downscaled 2x, RGB at 15 fps, depth at 10 fps, point cloud at 5 fps
- Compressed image transport
- Topic whitelist on foxglove_bridge

**Whitelisted topics:**
- `/zed/zed_node/rgb/image_rect_color/compressed`
- `/zed/zed_node/depth/depth_registered/compressedDepth`
- `/scan`, `/local_costmap/costmap`, `/global_costmap/costmap`
- `/plan`, `/odom`, `/tf`, `/tf_static`, `/goal_pose`, `/robot_description`

**Foxglove panels:** 3D scene (URDF + costmaps + path + scan), RGB image, depth image, odometry plots

### Navigation Goals

Click-to-navigate in Foxglove publishes to `/goal_pose`, picked up by Nav2.

### Setup

`setup.sh` installs apt deps (nav2, foxglove-bridge, depthimage-to-laserscan, image-transport-plugins, robot-state-publisher, xacro), clones zed-ros2-wrapper, runs rosdep, builds with colcon.

**Prerequisites:** ZED SDK and ROS 2 Humble pre-installed on Jetson.

**Usage:**
```bash
./setup.sh                                    # One-time setup
ros2 launch rover_autonav autonav.launch.py   # Launch everything
# Connect Foxglove to ws://<jetson-ip>:8765
```
