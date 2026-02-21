# Rover Autonav Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Implement a full autonomous navigation stack with Foxglove visualization for a 6-wheel differential drive rover using ZED 2i camera on Jetson AGX Orin.

**Architecture:** ZED 2i provides visual-inertial odometry and depth. Nav2 handles path planning and obstacle avoidance using depth-to-laserscan conversion. Foxglove bridge streams compressed visualization data within a 40 Mbps bandwidth budget. No wheel encoders or GPS — all odometry comes from ZED VIO.

**Tech Stack:** ROS 2 Humble, Nav2, ZED ROS 2 Wrapper, Foxglove Bridge, depthimage_to_laserscan, xacro/URDF

**Design doc:** `docs/plans/2026-02-21-autonav-design.md`

---

### Task 1: Create ROS 2 Package Skeleton

**Files:**
- Create: `src/rover_autonav/package.xml`
- Create: `src/rover_autonav/CMakeLists.txt`

**Step 1: Create directory structure**

```bash
mkdir -p src/rover_autonav/{config,description,launch}
```

**Step 2: Write package.xml**

Create `src/rover_autonav/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>rover_autonav</name>
  <version>0.1.0</version>
  <description>Autonomous navigation for 6-wheel differential drive rover</description>
  <maintainer email="todo@todo.com">todo</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>navigation2</exec_depend>
  <exec_depend>foxglove_bridge</exec_depend>
  <exec_depend>depthimage_to_laserscan</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>image_transport</exec_depend>
  <exec_depend>compressed_image_transport</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**Step 3: Write CMakeLists.txt**

Create `src/rover_autonav/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.5)
project(rover_autonav)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  description
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

**Step 4: Commit**

```bash
git add src/rover_autonav/package.xml src/rover_autonav/CMakeLists.txt
git commit -m "feat: add rover_autonav package skeleton"
```

---

### Task 2: Create Rover URDF

**Files:**
- Create: `src/rover_autonav/description/rover.urdf.xacro`

**Context:** The rover is ~0.9m long x ~0.7m wide (URC-scale). Track width is 0.686m (27 inches). Wheel radius is 0.0762m (3 inches). The ZED 2i is mounted on top, facing forward. The ZED wrapper publishes its own internal camera frames, so we only need to provide the `base_link → zed2i_camera_link` static transform. The ZED wrapper's launch with `publish_urdf:=false` means we handle all frames ourselves.

**Important:** We set `base_link` at ground level, center of the rover. The ZED camera center link is called `zed2i_camera_center` — this is the frame the ZED wrapper expects when `camera_name:='zed'`. The actual link name convention from zed-ros2-wrapper is `{camera_name}_camera_center`.

**Step 1: Write the URDF xacro**

Create `src/rover_autonav/description/rover.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="rover">

  <!-- Properties -->
  <xacro:property name="chassis_length" value="0.9"/>
  <xacro:property name="chassis_width" value="0.7"/>
  <xacro:property name="chassis_height" value="0.2"/>
  <xacro:property name="chassis_z_offset" value="0.15"/>

  <xacro:property name="wheel_radius" value="0.0762"/>
  <xacro:property name="wheel_width" value="0.1"/>
  <xacro:property name="track_width" value="0.686"/>

  <xacro:property name="wheelbase_front" value="0.3"/>
  <xacro:property name="wheelbase_rear" value="-0.3"/>

  <xacro:property name="camera_x" value="0.35"/>
  <xacro:property name="camera_z" value="0.45"/>

  <!-- Base footprint (ground plane projection) -->
  <link name="base_footprint"/>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 ${chassis_z_offset}" rpy="0 0 0"/>
      <geometry>
        <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
      </geometry>
      <material name="dark_gray">
        <color rgba="0.3 0.3 0.3 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 ${chassis_z_offset}" rpy="0 0 0"/>
      <geometry>
        <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
      </geometry>
    </collision>
  </link>

  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 ${wheel_radius}" rpy="0 0 0"/>
  </joint>

  <!-- Wheel macro -->
  <xacro:macro name="wheel" params="name x y">
    <link name="${name}_link">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black">
          <color rgba="0.1 0.1 0.1 1.0"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
    </link>
    <joint name="${name}_joint" type="fixed">
      <parent link="base_link"/>
      <child link="${name}_link"/>
      <origin xyz="${x} ${y} 0" rpy="${-pi/2} 0 0"/>
    </joint>
  </xacro:macro>

  <!-- Left wheels (y = +track_width/2) -->
  <xacro:wheel name="left_wheel_front"  x="${wheelbase_front}" y="${track_width/2}"/>
  <xacro:wheel name="left_wheel_mid"    x="0"                  y="${track_width/2}"/>
  <xacro:wheel name="left_wheel_rear"   x="${wheelbase_rear}"   y="${track_width/2}"/>

  <!-- Right wheels (y = -track_width/2) -->
  <xacro:wheel name="right_wheel_front" x="${wheelbase_front}"  y="${-track_width/2}"/>
  <xacro:wheel name="right_wheel_mid"   x="0"                   y="${-track_width/2}"/>
  <xacro:wheel name="right_wheel_rear"  x="${wheelbase_rear}"    y="${-track_width/2}"/>

  <!-- ZED 2i camera mount point -->
  <!-- This is the frame the ZED wrapper will attach its internal frames to -->
  <link name="zed2i_camera_link">
    <visual>
      <geometry>
        <box size="0.175 0.03 0.033"/>
      </geometry>
      <material name="silver">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="zed2i_camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="zed2i_camera_link"/>
    <origin xyz="${camera_x} 0 ${camera_z}" rpy="0 0 0"/>
  </joint>

</robot>
```

**Step 2: Verify xacro parses**

Run: `xacro src/rover_autonav/description/rover.urdf.xacro`
Expected: Valid XML output with all links and joints expanded

**Step 3: Commit**

```bash
git add src/rover_autonav/description/rover.urdf.xacro
git commit -m "feat: add rover URDF with 6 wheels and ZED 2i mount"
```

---

### Task 3: Create ZED Camera Configuration

**Files:**
- Create: `src/rover_autonav/config/zed_config.yaml`

**Context:** The ZED ROS 2 wrapper reads override params from a YAML file. We downscale resolution by 2x and reduce frame rates to save bandwidth. Positional tracking is enabled with area memory for loop closure (acts as lightweight SLAM). The ZED wrapper publishes `map→odom` and `odom→base_link` TFs. We set `publish_tf: true` and `publish_map_tf: true` so Nav2 gets both frames. We set `base_frame: 'base_link'` so the ZED odom output references our rover's base_link, not its internal camera frame.

**Step 1: Write the config**

Create `src/rover_autonav/config/zed_config.yaml`:

```yaml
/**:
  ros__parameters:
    general:
      camera_name: 'zed'
      pub_resolution: 'CUSTOM'
      pub_downscale_factor: 2.0
      pub_frame_rate: 15.0
      grab_resolution: 'HD720'
      grab_frame_rate: 30

    video:
      auto_exposure_gain: true
      publish_rgb: true

    depth:
      depth_mode: 'NEURAL'
      depth_stabilization: 50
      point_cloud_freq: 5.0
      min_depth: 0.3
      max_depth: 20.0
      depth_confidence: 50
      depth_texture_conf: 100

    pos_tracking:
      pos_tracking_enabled: true
      pos_tracking_mode: 'GEN_2'
      imu_fusion: true
      publish_tf: true
      publish_map_tf: true
      map_frame: 'map'
      odometry_frame: 'odom'
      base_frame: 'base_link'
      area_memory: true
      floor_alignment: false
      set_gravity_as_origin: true
      path_pub_rate: 2.0

    mapping:
      mapping_enabled: false

    object_detection:
      od_enabled: false
```

**Step 2: Commit**

```bash
git add src/rover_autonav/config/zed_config.yaml
git commit -m "feat: add ZED 2i camera configuration with VIO and bandwidth limits"
```

---

### Task 4: Create Nav2 Parameters

**Files:**
- Create: `src/rover_autonav/config/nav2_params.yaml`

**Context:** Nav2 needs a complete parameter file for all its servers. Key decisions:
- Rolling window global costmap (no static map, since we have no pre-built map)
- Obstacle layers fed by `/scan` topic (from depthimage_to_laserscan)
- DWB controller for differential drive
- NavFn planner
- Rectangular footprint matching rover dimensions (~0.9m x ~0.7m)
- Recovery behaviors: spin, backup, wait
- For Humble, use `progress_checker_plugins` (not `progress_checker_plugin`)

**Step 1: Write nav2_params.yaml**

Create `src/rover_autonav/config/nav2_params.yaml`:

```yaml
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /zed/zed_node/odom
    bt_loop_duration: 10
    default_server_timeout: 20
    # Use default navigate_to_pose BT
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_is_path_valid_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
      - nav2_navigate_to_pose_action_bt_node
      - nav2_navigate_through_poses_action_bt_node
      - nav2_remove_passed_goals_action_bt_node

controller_server:
  ros__parameters:
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.30
      yaw_goal_tolerance: 0.25
      stateful: true

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.5
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 1.5
      acc_lim_y: 0.0
      acc_lim_theta: 2.0
      decel_lim_x: -1.5
      decel_lim_y: 0.0
      decel_lim_theta: -2.0
      vx_samples: 20
      vy_samples: 1
      vtheta_samples: 20
      sim_time: 1.5
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.5
      xy_goal_tolerance: 0.30
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: true
      stateful: true
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      GoalAlign.scale: 24.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

behavior_server:
  ros__parameters:
    local_costmap_topic: local_costmap/costmap_raw
    local_footprint_topic: local_costmap/published_footprint
    global_costmap_topic: global_costmap/costmap_raw
    global_footprint_topic: global_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUp"
    wait:
      plugin: "nav2_behaviors/Wait"
    local_frame: odom
    global_frame: map
    robot_base_frame: base_link
    transform_tolerance: 0.1
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 2.0

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      resolution: 0.05
      track_unknown_space: true
      rolling_window: true
      width: 50
      height: 50
      # Rectangular footprint: ~0.9m x ~0.7m
      footprint: "[[0.45, 0.35], [0.45, -0.35], [-0.45, -0.35], [-0.45, 0.35]]"
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 10.0
          raytrace_min_range: 0.0
          obstacle_max_range: 8.0
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.6
      always_send_full_costmap: true

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 5
      height: 5
      resolution: 0.05
      footprint: "[[0.45, 0.35], [0.45, -0.35], [-0.45, -0.35], [-0.45, 0.35]]"
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: true
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 5.0
          raytrace_min_range: 0.0
          obstacle_max_range: 4.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.6
      always_send_full_costmap: true
```

**Step 2: Commit**

```bash
git add src/rover_autonav/config/nav2_params.yaml
git commit -m "feat: add Nav2 params with rolling costmaps and DWB diff-drive controller"
```

---

### Task 5: Create ZED Camera Launch File

**Files:**
- Create: `src/rover_autonav/launch/zed.launch.py`

**Context:** This launch file starts:
1. `robot_state_publisher` with our rover URDF (publishes all static TFs for the rover body + camera mount)
2. ZED camera node via `IncludeLaunchDescription` of the zed_wrapper launch file, with `publish_urdf:=false` (we handle the URDF) and our config override
3. `depthimage_to_laserscan` node converting ZED depth to virtual laser scan

The ZED wrapper's internal frame naming convention when `camera_name:='zed'`:
- `zed_camera_center` — the center of the camera baseline
- `zed_left_camera_frame` — left camera optical frame
The depth topic will be `/zed/zed_node/depth/depth_registered`

**Step 1: Write the launch file**

Create `src/rover_autonav/launch/zed.launch.py`:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover_autonav')

    # Process URDF xacro
    xacro_file = os.path.join(pkg_dir, 'description', 'rover.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # Robot state publisher (publishes rover body + camera mount TFs)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # Joint state publisher (for fixed joints visualization)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
    )

    # ZED camera node
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    zed_config = os.path.join(pkg_dir, 'config', 'zed_config.yaml')

    zed_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'zed2i',
            'publish_urdf': 'false',
            'ros_params_override_path': zed_config,
        }.items(),
    )

    # Depth image to laser scan
    depthimage_to_laserscan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[
            ('depth', '/zed/zed_node/depth/depth_registered'),
            ('depth_camera_info', '/zed/zed_node/depth/camera_info'),
        ],
        parameters=[{
            'scan_height': 10,
            'scan_time': 0.033,
            'range_min': 0.3,
            'range_max': 20.0,
            'output_frame': 'base_link',
        }],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        zed_camera,
        depthimage_to_laserscan,
    ])
```

**Step 2: Commit**

```bash
git add src/rover_autonav/launch/zed.launch.py
git commit -m "feat: add ZED camera launch with URDF and depth-to-laserscan"
```

---

### Task 6: Create Nav2 Launch File

**Files:**
- Create: `src/rover_autonav/launch/nav2.launch.py`

**Context:** We include Nav2's bringup launch file, passing our custom params YAML. We set `use_sim_time:=false` (real hardware), `autostart:=true` so all lifecycle nodes start automatically, and `map:=''` since we're using rolling window costmaps with no static map.

**Step 1: Write the launch file**

Create `src/rover_autonav/launch/nav2.launch.py`:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover_autonav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        nav2_bringup,
    ])
```

**Step 2: Commit**

```bash
git add src/rover_autonav/launch/nav2.launch.py
git commit -m "feat: add Nav2 launch with custom params"
```

---

### Task 7: Create Foxglove Bridge Launch File

**Files:**
- Create: `src/rover_autonav/launch/foxglove.launch.py`

**Context:** Launch foxglove_bridge with a topic whitelist to limit bandwidth. We bind to `0.0.0.0` so it's accessible from a remote laptop on the same network. We use 2 threads since we're streaming compressed images alongside lightweight topics. The whitelist includes all topics needed for Foxglove's 3D panel, image panels, and map panel.

**Step 1: Write the launch file**

Create `src/rover_autonav/launch/foxglove.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    topic_whitelist = [
        '/zed/zed_node/rgb/image_rect_color/compressed',
        '/zed/zed_node/depth/depth_registered',
        '/zed/zed_node/depth/camera_info',
        '/zed/zed_node/odom',
        '/zed/zed_node/path_odom',
        '/zed/zed_node/pose',
        '/scan',
        '/local_costmap/costmap',
        '/local_costmap/published_footprint',
        '/global_costmap/costmap',
        '/global_costmap/published_footprint',
        '/plan',
        '/odom',
        '/tf',
        '/tf_static',
        '/goal_pose',
        '/robot_description',
        '/initialpose',
    ]

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'num_threads': 2,
            'topic_whitelist': topic_whitelist,
            'send_buffer_limit': 50000000,  # 50MB buffer limit
            'max_qos_depth': 5,
        }],
        output='screen',
    )

    return LaunchDescription([
        foxglove_bridge,
    ])
```

**Step 2: Commit**

```bash
git add src/rover_autonav/launch/foxglove.launch.py
git commit -m "feat: add Foxglove bridge launch with bandwidth-limited topic whitelist"
```

---

### Task 8: Create Master Launch File

**Files:**
- Create: `src/rover_autonav/launch/autonav.launch.py`

**Context:** This is the single entry point. It includes the three sub-launch files in order: ZED first (provides odom + TFs), then Nav2 (needs odom), then Foxglove (streams everything). We use `IncludeLaunchDescription` so all nodes share the same process lifecycle.

**Step 1: Write the master launch file**

Create `src/rover_autonav/launch/autonav.launch.py`:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover_autonav')
    launch_dir = os.path.join(pkg_dir, 'launch')

    # Start ZED camera + URDF + depth-to-laserscan
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'zed.launch.py')
        ),
    )

    # Start Nav2 after a short delay to let ZED TFs stabilize
    nav2_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'nav2.launch.py')
                ),
            ),
        ],
    )

    # Start Foxglove bridge
    foxglove_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'foxglove.launch.py')
        ),
    )

    return LaunchDescription([
        zed_launch,
        nav2_launch,
        foxglove_launch,
    ])
```

**Step 2: Commit**

```bash
git add src/rover_autonav/launch/autonav.launch.py
git commit -m "feat: add master launch file orchestrating ZED, Nav2, and Foxglove"
```

---

### Task 9: Create Setup Script

**Files:**
- Create: `setup.sh`

**Context:** One-shot script to run after cloning the repo on the Jetson. Installs all apt dependencies, clones zed-ros2-wrapper if missing, runs rosdep, and builds. Assumes ROS 2 Humble and ZED SDK are already installed.

**Step 1: Write setup.sh**

Create `setup.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}=== Rover Autonav Setup ===${NC}"

# Check ROS 2
if [ -z "${ROS_DISTRO:-}" ]; then
    echo "Error: ROS 2 is not sourced. Run: source /opt/ros/humble/setup.bash"
    exit 1
fi

if [ "$ROS_DISTRO" != "humble" ]; then
    echo -e "${YELLOW}Warning: Expected ROS 2 Humble, got $ROS_DISTRO${NC}"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Install apt dependencies
echo -e "${GREEN}[1/5] Installing apt dependencies...${NC}"
sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-foxglove-bridge \
    ros-humble-depthimage-to-laserscan \
    ros-humble-image-transport-plugins \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-compressed-image-transport \
    python3-colcon-common-extensions

# Clone ZED ROS 2 wrapper
echo -e "${GREEN}[2/5] Setting up ZED ROS 2 wrapper...${NC}"
if [ ! -d "src/zed-ros2-wrapper" ]; then
    cd src
    git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
    cd ..
else
    echo "zed-ros2-wrapper already exists, skipping clone"
fi

# Install rosdep dependencies
echo -e "${GREEN}[3/5] Installing rosdep dependencies...${NC}"
if ! command -v rosdep &> /dev/null; then
    sudo apt install -y python3-rosdep
    sudo rosdep init || true
fi
rosdep update --rosdistro=humble || true
rosdep install --from-paths src --ignore-src -r -y

# Build
echo -e "${GREEN}[4/5] Building workspace...${NC}"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source
echo -e "${GREEN}[5/5] Setup complete!${NC}"
echo ""
echo "To use:"
echo "  source install/setup.bash"
echo "  ros2 launch rover_autonav autonav.launch.py"
echo ""
echo "Then connect Foxglove to: ws://<jetson-ip>:8765"
```

**Step 2: Make executable and commit**

```bash
chmod +x setup.sh
git add setup.sh
git commit -m "feat: add one-shot setup script for Jetson deployment"
```

---

### Task 10: Build and Smoke Test

**Context:** Verify the package builds without errors. On a machine without ZED SDK, the zed-ros2-wrapper won't build, but our package should build fine. On the Jetson with ZED SDK installed, everything should build.

**Step 1: Build rover_autonav only (no ZED wrapper needed for structure validation)**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rover_autonav --symlink-install
```

Expected: `Summary: 1 package finished` with no errors

**Step 2: Verify install contents**

```bash
source install/setup.bash
ls install/rover_autonav/share/rover_autonav/
```

Expected: `config/`, `description/`, `launch/` directories present

**Step 3: Verify URDF parses**

```bash
xacro install/rover_autonav/share/rover_autonav/description/rover.urdf.xacro
```

Expected: Valid XML output

**Step 4: Verify launch file syntax**

```bash
ros2 launch rover_autonav autonav.launch.py --print
```

Expected: No Python syntax errors (will fail to actually launch without ZED, but structure is validated)

**Step 5: Commit any fixes if needed**

---

### Task 11: Final Integration Commit

**Step 1: Ensure all files are committed**

```bash
git status
git add -A
git commit -m "chore: ensure all autonav files tracked"
```

**Step 2: Verify final repo structure**

```bash
find src/rover_autonav -type f | sort
```

Expected output:
```
src/rover_autonav/CMakeLists.txt
src/rover_autonav/config/nav2_params.yaml
src/rover_autonav/config/zed_config.yaml
src/rover_autonav/description/rover.urdf.xacro
src/rover_autonav/launch/autonav.launch.py
src/rover_autonav/launch/foxglove.launch.py
src/rover_autonav/launch/nav2.launch.py
src/rover_autonav/launch/zed.launch.py
src/rover_autonav/package.xml
```
