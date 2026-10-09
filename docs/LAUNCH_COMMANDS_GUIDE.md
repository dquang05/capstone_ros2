# AMR System Launch & Operation Commands Guide

> **Project:** Autonomous Mobile Robot (AMR) for Food Delivery  
> **Framework:** ROS 2 Jazzy Jalisco (Ubuntu 24.04 LTS) | Simulation Engine: Gazebo Harmonic  
> **Package:** `capstone_ros2`  
> **Last Updated:** 2026-10-07  

---

## 1. Environment & Workspace Preparation

Before running any ROS 2 nodes or launch files, ensure the ROS 2 Jazzy underlay and the project overlay spaces are sourced:

```bash
# Source ROS 2 Jazzy underlay
source /opt/ros/jazzy/setup.bash

# Source workspace install overlay (built with symlink-install)
source ~/Projects/capstone_ros2/install/setup.bash
```

### Build Workspace (Optional - only required after modifying C++ / package structure)
Because the workspace is built with `--symlink-install`, Python launch scripts and YAML configuration edits take effect immediately without recompilation. When adding new packages or modifying C++/interfaces, run:
```bash
cd ~/Projects/capstone_ros2
colcon build --symlink-install
source install/setup.bash
```

---

## 2. All-in-One Full System Bringup (Recommended)

One single command orchestrates the entire autonomous delivery robot pipeline:
1. **Gazebo Harmonic**: Spawns the realistic restaurant blueprint world with hollow-legged tables/chairs and places the AMR at the Area 08 Main Hall dock (`x=8.85m, y=-3.80m, yaw=3.14159 rad`).
2. **ROS-Gz Bridge**: Bridges `/clock`, `/scan`, `/imu/data`, `/cmd_vel`, `/wheel/odom`, and `/joint_states`.
3. **EKF Sensor Fusion**: Fuses wheel odometry and 9-DOF IMU into `/odometry/filtered` (50Hz) and publishes `odom` $\to$ `base_footprint`.
4. **Nav2 Navigation Stack**: Starts AMCL localization, $A^*$ Global Planner, DWB Local Controller with anti-spill constraints, costmaps, and behavior servers.
5. **RViz2**: Displays costmaps, laser scans, planned paths, and AMCL particle cloud with pre-tuned layout.

### 2.1. Standard Launch (Main Realistic Restaurant)
```bash
ros2 launch amr_bringup amr_navigation.launch.py
```
*(By default, this automatically selects `maps/restaurant_main.yaml`)*

### 2.2. Explicit Custom Map Path
```bash
ros2 launch amr_bringup amr_navigation.launch.py map:=/home/quangtran/Projects/capstone_ros2/src/amr_navigation/maps/restaurant_main.yaml
```

### 2.3. Switch to Simple Rectangular Test World
```bash
ros2 launch amr_bringup amr_navigation.launch.py world_type:=test
```

### 2.4. Headless Mode (Fast Execution - Disable Gazebo 3D GUI)
Useful when focusing on RViz2 navigation and saving CPU/GPU resources:
```bash
ros2 launch amr_bringup amr_navigation.launch.py gui:=false
```

---

## 3. Standalone Gazebo Simulation (`amr_gazebo`)

Use these commands when developing or testing physics, sensor plugins, or robot spawn mechanics without launching navigation:

### 3.1. Main Restaurant Blueprint with AMR Spawned
Spawns the complete restaurant environment with hollow furniture and places AMR at the home dock:
```bash
ros2 launch amr_gazebo restaurant_main.launch.py
```

### 3.2. Inspect Main Restaurant World Only (No Robot)
Loads the 3D restaurant blueprint model without spawning the robot entity or the ROS-Gz bridge:
```bash
ros2 launch amr_gazebo restaurant_main.launch.py spawn_robot:=false
```

### 3.3. Test Rectangular World
Loads the lightweight 16m x 10m benchmark restaurant world:
```bash
ros2 launch amr_gazebo restaurant_test.launch.py
```

### 3.4. Custom Spawn Coordinates
Spawn AMR at custom world coordinates:
```bash
ros2 launch amr_gazebo restaurant_main.launch.py x_pose:=0.0 y_pose:=0.0 yaw_pose:=0.0
```

---

## 4. Standalone Visualization & Robot Inspection (`RViz2`)

### 4.1. Open RViz2 with Pre-configured Navigation Layout
```bash
rviz2 -d ~/Projects/capstone_ros2/src/amr_bringup/rviz/nav2.rviz --ros-args -p use_sim_time:=true
```

### 4.2. View Robot URDF Kinematics & TF Model
Loads the robot description, joint state publisher GUI slider, and RViz2 to verify physical dimensions, wheel joints, and sensor frames:
```bash
ros2 launch amr_description view_robot.launch.py
```

---

## 5. SLAM Mapping Workflow (`amr_bringup` + `slam_toolbox`)

To re-map the restaurant environment from scratch:

### Step 1: Launch Full SLAM Environment
Spawns Gazebo, AMR, EKF sensor fusion, SLAM Toolbox (online asynchronous), and RViz2:
```bash
ros2 launch amr_bringup amr_slam.launch.py
```
*(To map the test rectangular world instead: `ros2 launch amr_bringup amr_slam.launch.py world_type:=test`)*

### Step 2: Drive AMR via Keyboard Teleoperation (Separate Terminal)
```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

### Step 3: Save the Completed Occupancy Grid Map
Once all corridors and table clusters are scanned:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/Projects/capstone_ros2/src/amr_navigation/maps/restaurant_main
```
*(Generates `restaurant_main.yaml` and `restaurant_main.pgm`)*

---

## 6. Teleoperation & Runtime Diagnostics

### 6.1. Manual Driving via Keyboard Teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```
*Keyboard shortcuts:*
* `i`: Move forward | `,`: Move backward
* `j`: Turn left | `l`: Turn right
* `k`: Stop
* `q`/`z`: Increase / decrease linear max speed
* `w`/`x`: Increase / decrease angular max speed

### 6.2. Send Navigation Goal via CLI
In addition to clicking **Nav2 Goal** in RViz2, you can dispatch food delivery goals directly via CLI:
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 3.5, y: -2.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"
```

### 6.3. Monitor Critical System Topics
```bash
# Verify EKF filtered odometry (50 Hz)
ros2 topic hz /odometry/filtered
ros2 topic echo /odometry/filtered

# Verify 2D LiDAR scans (15 Hz)
ros2 topic hz /scan

# Verify 9-DOF IMU data (100 Hz)
ros2 topic hz /imu/data

# Inspect current velocity output from DWB controller
ros2 topic echo /cmd_vel
```

### 6.4. Verify TF Coordinate Transform Tree
```bash
# Check TF transformation frames (map -> odom -> base_footprint -> base_link -> ...)
ros2 run tf2_tools view_frames
# (Generates frames.pdf in current directory)
```

---

## 7. Quick Reference Cheat Sheet

| Task / Purpose | Launch / Run Command | Key Parameters / Options |
| :--- | :--- | :--- |
| **All-in-One Autonomous Nav** | `ros2 launch amr_bringup amr_navigation.launch.py` | `world_type:=main\|test`, `gui:=true\|false`, `map:=<path>` |
| **Gazebo Main World** | `ros2 launch amr_gazebo restaurant_main.launch.py` | `spawn_robot:=true\|false`, `x_pose:=<x>`, `y_pose:=<y>` |
| **Gazebo Test World** | `ros2 launch amr_gazebo restaurant_test.launch.py` | `spawn_robot:=true\|false` |
| **SLAM Mapping Bringup** | `ros2 launch amr_bringup amr_slam.launch.py` | `world_type:=main\|test`, `gui:=true\|false` |
| **Save SLAM Map** | `ros2 run nav2_map_server map_saver_cli -f <path>` | Saves `<path>.yaml` and `<path>.pgm` |
| **RViz2 Navigation View** | `rviz2 -d <path_to_rviz_config>` | `-p use_sim_time:=true` |
| **Robot URDF & TF Model** | `ros2 launch amr_description view_robot.launch.py` | Joint state publisher GUI |
| **Keyboard Teleoperation** | `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel` | Drives robot manually |
