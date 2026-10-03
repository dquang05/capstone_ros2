# AMR Simulation Project Progress

> **Project:** Autonomous Mobile Robot (AMR) for Restaurant Food Delivery  
> **Framework:** ROS 2 Jazzy (Ubuntu 24.04 LTS) | Simulation Engine: Gazebo  
> **Last Updated:** 2026-10-01  

---

## 1. Current Status

- **Robot structure is complete:**
  - Base chassis plate (25 kg) and 3-tier food delivery trays (5 kg each) aligned with full collision geometry and calibrated inertial matrices.
  - 6-wheel mobility system (2 differential drive wheels, 4 passive caster wheels) leveled coplanar with the ground.
  - Front-mounted 2D LiDAR, center 9-DOF IMU, and wheel encoder feedback (joint states & diff drive controller) are integrated with Gazebo simulation plugins.
- **Restaurant Simulation Environment (Clean 16.0m x 10.0m Layout):**
  - Fully restored the clean, optimized rectangular layout (16.0m x 10.0m x 2.4m):
    - **Top Row (5 Dining Booths):** Tables 1 to 5 with partition walls ($H=1.6\text{m}$) and dining benches.
    - **Top Right (Kitchen & Food Dispatch / AMR Home Station):**
      - Room bounds: $X \in [1.7, 8.0], Y \in [1.6, 5.0]$ with wide $1.8\text{m}$ doorway.
      - **AMR Home Docking Station ($X = 2.6\text{m}, Y = 3.5\text{m}, \text{Yaw} = -90^\circ$):** Dedicated industrial floor docking mat with yellow safety stripes, automated charging tower with glowing cyan terminal and green status LED.
      - **Food Dispatch Counter:** Serving pass-through counter with meal trays and cloche covers.
    - **Central Aisle (2.4m wide):** 3 lounge cafe tables (C1, C2, C3) with comfortable armchairs.
    - **Bottom Row (4 Private VIP Dining Rooms):** Tables 6 to 9 with partition walls, individual wide doorways, and dining benches.
    - **Realistic 4-Legged Furniture (LiDAR Scanning Realism):**
      - All dining tables, coffee tables, benches, and lounge sofas replaced solid blocks with hollow bottoms and 4 individual rectangular prism legs.
      - Leg heights ($340\text{mm} - 700\text{mm}$) exceed LiDAR scan plane ($\sim 290\text{mm}$), allowing laser beams to pass under tabletops/seats and detect only the legs, accurately reflecting real-world SLAM obstacle avoidance conditions.
  - Built purely with lightweight analytical primitives (0 trimesh lag, 100% RTF, crisp LiDAR reflection).
  - Launch file `restaurant_world.launch.py` defaults spawn to the Kitchen Home Station facing South out into the main corridor.

---

## 2. Control & Navigation System Architecture (Completed)

- **Sensor Fusion (EKF - `amr_localization`):**
  - Integrated `robot_localization` 2D EKF at 50Hz, fusing `/wheel/odom` (linear $v_x$, angular $\omega_z$) and `/imu/data` (orientation yaw $\psi$, angular velocity $\omega_z$, linear acceleration $a_x$).
  - Broadcasts continuous TF `odom` $\to$ `base_footprint` and publishes `/odometry/filtered`.
- **SLAM Mapping (`amr_navigation`):**
  - Configured `slam_toolbox` (async) with 0.05m resolution, Ceres solver scan matching, and loop closure.
  - Broadcasts TF `map` $\to$ `odom` and publishes occupancy grid `/map`.
- **Autonomous Navigation (`amr_navigation` - Nav2):**
  - **Global Planner:** $A^*$ search (`nav2_navfn_planner::NavfnPlanner` with `use_astar: true`).
  - **Local Controller:** Strictly configured **DWB** (`dwb_core::DWBLocalPlanner` from `nav2_dwb_controller`).
  - **Anti-Spill Dynamic Limits:** Cruise speed $v_{\max} = 0.60\,\text{m/s}$, linear acceleration/deceleration $|a| \le 0.40\,\text{m/s}^2$, rotational acceleration $|\alpha| \le 0.60\,\text{rad/s}^2$, and centripetal acceleration $a_{\text{lat}} \le 0.40\,\text{m/s}^2$ to protect meal trays and liquids from spillage.
  - **DWB Critics:** `RotateToGoal`, `Oscillation`, `BaseObstacle`, `ObstacleFootprint`, `GoalAlign`, `PathAlign`, `PathDist`, `GoalDist`.
  - **Robot Footprint:** $760\,\text{mm} \times 600\,\text{mm}$ with 0.70m inflation radius to safely negotiate hollow 4-legged tables and chairs.
- **Top-Level Orchestration (`amr_bringup`):**
  - `amr_slam.launch.py`: One-command bringup for Gazebo + EKF + SLAM Toolbox + RViz2.
  - `amr_navigation.launch.py`: One-command bringup for Gazebo + EKF + Nav2 (DWB + $A^*$) + RViz2.

---

## 3. Launch Commands

- **Inspect Robot Model in RViz:**
  ```bash
  source ~/Projects/capstone_ros2/install/setup.bash
  ros2 launch amr_description view_robot.launch.py
  ```

- **Launch Full Simulation in Gazebo (Restaurant + AMR):**
  ```bash
  source ~/Projects/capstone_ros2/install/setup.bash
  ros2 launch amr_gazebo restaurant_world.launch.py
  ```

- **Step 4.1: Launch SLAM Mapping Session (Gazebo + EKF + SLAM Toolbox + RViz):**
  ```bash
  source ~/Projects/capstone_ros2/install/setup.bash
  ros2 launch amr_bringup amr_slam.launch.py
  ```

- **Teleoperate AMR during SLAM (in a separate terminal):**
  ```bash
  source /opt/ros/jazzy/setup.bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
  ```

- **Save Generated SLAM Map to `amr_navigation/maps`:**
  ```bash
  ros2 run nav2_map_server map_saver_cli -f ~/Projects/capstone_ros2/src/amr_navigation/maps/restaurant
  ```

- **Step 4.2: Launch Autonomous Navigation (DWB + A* + RViz):**
  ```bash
  source ~/Projects/capstone_ros2/install/setup.bash
  ros2 launch amr_bringup amr_navigation.launch.py
  ```
