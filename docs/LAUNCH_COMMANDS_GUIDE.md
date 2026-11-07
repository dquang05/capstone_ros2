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

## 3. Autonomous Food Delivery Mission Dispatcher (`dispatch_table`)

The `dispatch_table` mission dispatcher ([dispatch_table.py](file:///home/quangtran/Projects/capstone_ros2/src/amr_navigation/amr_navigation/dispatch_table.py)) enables end-to-end autonomous food delivery missions to predefined dining tables configured in [restaurant_tables.yaml](file:///home/quangtran/Projects/capstone_ros2/src/amr_navigation/config/restaurant_tables.yaml).

### 3.1. Key Capabilities & Engineering Features
* **Single-Command ID Dispatch:** Send AMR directly to any dining cluster or service station across the restaurant world using numeric IDs (`0` to `20`).
* **Cluster / Group Fallback Recovery Mechanism:** If the primary destination table is blocked by dynamic obstacles, customers, or narrow bottlenecks, the dispatcher automatically diverts to alternative fallback tables within the same dining cluster group instead of failing the mission.
* **Aisle Standoff Poses (Anti-Deadlock):** All target coordinates are positioned on open main corridors ($0.82\text{ m} - 1.35\text{ m}$ clearance) outside the costmap inflation layer ($0.40\text{ m}$). This guarantees the robot's $0.484\text{ m}$ diagonal turning radius has complete freedom for $360^\circ$ in-place rotation maneuvers without wedging between customer chairs.
* **Autonomous Meal Service Countdown:** Upon arrival, the AMR parks facing the customer table with its delivery tray, begins a 10-second countdown for meal retrieval, and signals mission completion.
* **One-Touch Home Return:** Dispatching ID 0 or using `--home` recalls the robot directly back to the Kitchen Docking Station.

### 3.2. Usage Syntax & Command Examples
```bash
# Ensure workspace environment is sourced in your mission terminal
source /opt/ros/jazzy/setup.bash
source ~/Projects/capstone_ros2/install/setup.bash

# 1. Display the complete interactive dining table catalog and current statuses:
ros2 run amr_navigation dispatch_table --list

# 2. Dispatch robot to a specific dining table (e.g. Group 02 NE Table 12):
ros2 run amr_navigation dispatch_table 12

# 3. Dispatch to other dining clusters (e.g. Center Greenery Island Table 4):
ros2 run amr_navigation dispatch_table 4

# 4. Recall robot back to Kitchen / Home Charging Dock:
ros2 run amr_navigation dispatch_table 0
# Or using the convenience flag:
ros2 run amr_navigation dispatch_table --home
```

### 3.3. Complete Restaurant Dining Table Catalog (IDs 0..20)
All target coordinates are referenced in the map frame and configured in `restaurant_tables.yaml`:

| ID | Table Code | Name & Description | Cluster Group | Target Pose $(X, Y, \text{Yaw})$ | Fallback IDs |
| :---: | :--- | :--- | :--- | :---: | :---: |
| **0** | `HOME_DOCK` | Trạm Đỗ Sạc & Nhận Món (Home Dock - Area 08) | Trạm Đỗ Sạc & Xuất Phát | `(0.00, 0.00, 0.0°)` | *None* |
| **1** | `CIRC_SE_NORTH` | Cụm Tròn ĐN - Bàn Bắc (North Set) | Cụm Bàn Tròn Đông Nam | `(8.96, -3.37, +30.5°)` | `[2, 3]` |
| **2** | `CIRC_SE_SW` | Cụm Tròn ĐN - Bàn Tây Nam (SW Set) | Cụm Bàn Tròn Đông Nam | `(11.75, +0.64, 180.0°)` | `[1, 3]` |
| **3** | `CIRC_SE_SE` | Cụm Tròn ĐN - Bàn Đông Nam (SE Set) | Cụm Bàn Tròn Đông Nam | `(8.75, +1.53, -160.1°)` | `[1, 2]` |
| **4** | `CENTER_NORTH` | Đảo Trung Tâm - Bàn Bắc (North Table) | Đảo Cây Xanh Trung Tâm | `(11.92, -6.28, +59.5°)` | `[5, 6, 7]` |
| **5** | `CENTER_SOUTH` | Đảo Trung Tâm - Bàn Nam (South Table) | Đảo Cây Xanh Trung Tâm | `(12.30, -1.06, -149.5°)` | `[4, 6, 7]` |
| **6** | `CENTER_EAST` | Đảo Trung Tâm - Bàn Đông (East Table) | Đảo Cây Xanh Trung Tâm | `(9.50, -3.43, -59.5°)` | `[4, 5, 7]` |
| **7** | `CENTER_WEST` | Đảo Trung Tâm - Bàn Tây (West Table) | Đảo Cây Xanh Trung Tâm | `(14.75, -3.66, 180.0°)` | `[4, 5, 6]` |
| **8** | `CIRC_N_SW` | Cụm Tròn Bắc - Bàn Tây Nam (SW Set) | Cụm Bàn Tròn Phía Bắc | `(15.46, -7.94, +119.7°)` | `[9, 10]` |
| **9** | `CIRC_N_SE` | Cụm Tròn Bắc - Bàn Đông Nam (SE Set) | Cụm Bàn Tròn Phía Bắc | `(10.49, -7.16, -69.8°)` | `[8, 10]` |
| **10** | `CIRC_N_SOUTH` | Cụm Tròn Bắc - Điểm Cận Nam (South Approach)| Cụm Bàn Tròn Phía Bắc | `(12.83, -6.86, -98.5°)` | `[8, 9]` |
| **11** | `G02_NE_NORTH` | Group 02 ĐB - Bàn Bắc (North Table) | Group 02 (Nam Cột ĐB) | `(7.35, -7.74, +8.5°)` | `[12, 13]` |
| **12** | `G02_NE_SW` | Group 02 ĐB - Bàn Tây Nam (SW Table) | Group 02 (Nam Cột ĐB) | `(9.35, -4.05, -135.0°)` | `[11, 13]` |
| **13** | `G02_NE_SE` | Group 02 ĐB - Bàn Đông Nam (SE Table) | Group 02 (Nam Cột ĐB) | `(6.44, -4.56, -69.8°)` | `[11, 12]` |
| **14** | `G02_SW_NORTH` | Group 02 TN - Bàn Bắc (North Table) | Group 02 (Bắc Cột TN) | `(16.47, -2.98, +30.5°)` | `[15, 16]` |
| **15** | `G02_SW_SW` | Group 02 TN - Bàn Tây Nam (SW Table) | Group 02 (Bắc Cột TN) | `(14.80, +0.40, -149.9°)` | `[14, 16]` |
| **16** | `G02_SW_SE` | Group 02 TN - Bàn Đông Nam (SE Table) | Group 02 (Bắc Cột TN) | `(14.99, +0.45, -69.8°)` | `[14, 15]` |
| **17** | `WEST_ROW_NORTH`| Dãy Cột Tây - Bàn Bắc (North Table) | Dãy Bàn Dọc Cột Tây Bắc | `(14.58, -6.13, -49.1°)` | `[18]` |
| **18** | `WEST_ROW_SOUTH`| Dãy Cột Tây - Bàn Nam (South Table) | Dãy Bàn Dọc Cột Tây Bắc | `(14.58, -4.75, -49.1°)` | `[17]` |
| **19** | `GROUP_01_SE` | Bàn Tiêu Chuẩn Group 01 (Cạnh Cột SE) | Bàn Tiêu Chuẩn Group 01 | `(8.66, +1.50, -159.9°)` | `[3]` |
| **20** | `AREA_08_BAR` | Quầy Bar 04 / Bàn Đứng Khu 08 | Khu 08 Quầy Bar & Bàn Đứng| `(-1.55, +0.50, 180.0°)` | *None* |

### 3.4. Mission Execution Flow & Architecture
1. **Nav2 Lifecycle Validation:** Checks `bt_navigator` action server readiness before issuing goals.
2. **Autonomous Trajectory Tracking:** Publishes goal to Nav2 and displays real-time telemetry (distance remaining, mission timer).
3. **Arrival & Table Service:** Automatically confirms arrival within tolerance (`xy_goal_tolerance <= 0.25m`), parks with delivery tray oriented toward customers, and executes the 10-second service wait countdown.
4. **Dynamic Cluster Fallback:** If Nav2 encounters an impassable bottleneck, the dispatcher seamlessly attempts alternate table targets in the same cluster group (`fallback_table_ids`).

---

## 4. Standalone Gazebo Simulation (`amr_gazebo`)

Use these commands when developing or testing physics, sensor plugins, or robot spawn mechanics without launching navigation:

### 4.1. Main Restaurant Blueprint with AMR Spawned
Spawns the complete restaurant environment with hollow furniture and places AMR at the home dock:
```bash
ros2 launch amr_gazebo restaurant_main.launch.py
```

### 4.2. Inspect Main Restaurant World Only (No Robot)
Loads the 3D restaurant blueprint model without spawning the robot entity or the ROS-Gz bridge:
```bash
ros2 launch amr_gazebo restaurant_main.launch.py spawn_robot:=false
```

### 4.3. Test Rectangular World
Loads the lightweight 16m x 10m benchmark restaurant world:
```bash
ros2 launch amr_gazebo restaurant_test.launch.py
```

### 4.4. Custom Spawn Coordinates
Spawn AMR at custom world coordinates:
```bash
ros2 launch amr_gazebo restaurant_main.launch.py x_pose:=0.0 y_pose:=0.0 yaw_pose:=0.0
```

---

## 5. Standalone Visualization & Robot Inspection (`RViz2`)

### 5.1. Open RViz2 with Pre-configured Navigation Layout
```bash
rviz2 -d ~/Projects/capstone_ros2/src/amr_bringup/rviz/nav2.rviz --ros-args -p use_sim_time:=true
```

### 5.2. View Robot URDF Kinematics & TF Model
Loads the robot description, joint state publisher GUI slider, and RViz2 to verify physical dimensions, wheel joints, and sensor frames:
```bash
ros2 launch amr_description view_robot.launch.py
```

---

## 6. SLAM Mapping Workflow (`amr_bringup` + `slam_toolbox`)

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

## 7. Teleoperation & Runtime Diagnostics

### 7.1. Manual Driving via Keyboard Teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```
*Keyboard shortcuts:*
* `i`: Move forward | `,`: Move backward
* `j`: Turn left | `l`: Turn right
* `k`: Stop
* `q`/`z`: Increase / decrease linear max speed
* `w`/`x`: Increase / decrease angular max speed

### 7.2. Send Navigation Goal via CLI
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

### 7.3. Monitor Critical System Topics
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

### 7.4. Verify TF Coordinate Transform Tree
```bash
# Check TF transformation frames (map -> odom -> base_footprint -> base_link -> ...)
ros2 run tf2_tools view_frames
# (Generates frames.pdf in current directory)
```

---

## 8. Quick Reference Cheat Sheet

| Task / Purpose | Launch / Run Command | Key Parameters / Options |
| :--- | :--- | :--- |
| **All-in-One Autonomous Nav** | `ros2 launch amr_bringup amr_navigation.launch.py` | `world_type:=main\|test`, `gui:=true\|false`, `map:=<path>` |
| **Autonomous Table Dispatcher** | `ros2 run amr_navigation dispatch_table <ID>` | `<ID>` (`0..20`), `--list`, `--home` |
| **Gazebo Main World** | `ros2 launch amr_gazebo restaurant_main.launch.py` | `spawn_robot:=true\|false`, `x_pose:=<x>`, `y_pose:=<y>` |
| **Gazebo Test World** | `ros2 launch amr_gazebo restaurant_test.launch.py` | `spawn_robot:=true\|false` |
| **SLAM Mapping Bringup** | `ros2 launch amr_bringup amr_slam.launch.py` | `world_type:=main\|test`, `gui:=true\|false` |
| **Save SLAM Map** | `ros2 run nav2_map_server map_saver_cli -f <path>` | Saves `<path>.yaml` and `<path>.pgm` |
| **RViz2 Navigation View** | `rviz2 -d <path_to_rviz_config>` | `-p use_sim_time:=true` |
| **Robot URDF & TF Model** | `ros2 launch amr_description view_robot.launch.py` | Joint state publisher GUI |
| **Keyboard Teleoperation** | `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel` | Drives robot manually |
