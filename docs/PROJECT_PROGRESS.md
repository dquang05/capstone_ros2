# AMR Simulation Project Progress

> **Project:** Autonomous Mobile Robot (AMR) for Restaurant Food Delivery  
> **Framework:** ROS 2 Jazzy (Ubuntu 24.04 LTS) | Simulation Engine: Gazebo Harmonic  
> **Last Updated:** 2026-10-06  

---

## 1. Current Status

- **Robot Structure & Kinematics (Complete):**
  - Base chassis plate (25 kg) and 3-tier food delivery trays (5 kg each) aligned with collision geometry and calibrated inertial matrices.
  - 6-wheel mobility system (2 differential drive wheels, 4 passive caster wheels) leveled coplanar with the ground plane.
  - Front-mounted 2D LiDAR, center 9-DOF IMU, and wheel encoder feedback (joint states & diff-drive controller) integrated with Gazebo simulation plugins.
- **Simulation Environments:**
  - **Main Realistic Blueprint Restaurant (`restaurant_main.launch.py` / `restaurant_blueprint.world`):** Full architectural restaurant layout with realistic hollow-legged tables/chairs, dining clusters, bar counters, reception desk, and kitchen docking bay.
  - **Test Rectangular Restaurant (`restaurant_test.launch.py` / `restaurant.world`):** Clean 16.0m x 10.0m rectangular benchmark layout for quick testing and baseline verification.

---

## 2. Control & Navigation System Architecture (Complete)

- **Sensor Fusion (EKF - `amr_localization`):**
  - Integrated `robot_localization` 2D EKF at 50Hz, fusing `/wheel/odom` (linear $v_x$, angular $\omega_z$) and `/imu/data` (orientation yaw $\psi$, angular velocity $\omega_z$, linear acceleration $a_x$).
  - Broadcasts continuous TF `odom` $\to$ `base_footprint` and publishes `/odometry/filtered`.
- **SLAM Mapping (`amr_navigation`):**
  - Configured `slam_toolbox` (async) with 0.05m resolution, Ceres solver scan matching, and loop closure optimization.
  - Broadcasts TF `map` $\to$ `odom` and publishes occupancy grid `/map`.
- **Autonomous Navigation (`amr_navigation` - Nav2):**
  - **Global Planner:** $A^*$ search (`nav2_navfn_planner::NavfnPlanner` with `use_astar: true`).
  - **Local Controller:** DWB (`dwb_core::DWBLocalPlanner` from `nav2_dwb_controller`).
  - **Anti-Spill Dynamic Limits:** Cruise speed $v_{\max} = 0.60\,\text{m/s}$, linear acceleration/deceleration $|a| \le 0.40\,\text{m/s}^2$, rotational acceleration $|\alpha| \le 0.60\,\text{rad/s}^2$, and centripetal acceleration $a_{\text{lat}} \le 0.40\,\text{m/s}^2$ to protect meal trays from liquid spillage.
  - **DWB Critics:** `RotateToGoal`, `Oscillation`, `BaseObstacle`, `ObstacleFootprint`, `GoalAlign`, `PathAlign`, `PathDist`, `GoalDist`.
  - **Robot Footprint:** $760\,\text{mm} \times 600\,\text{mm}$ with inflation radius tuned to safely clear hollow 4-legged tables and dining chairs.
- **Top-Level Orchestration (`amr_bringup`):**
  - `amr_slam.launch.py`: One-command bringup for Gazebo + EKF + SLAM Toolbox + RViz2 (supports `world_type:=main` and `world_type:=test`).
  - `amr_navigation.launch.py`: One-command bringup for Gazebo + EKF + Nav2 (DWB + $A^*$) + RViz2.

---

## 3. Simulation & Operation Workflow

### 3.1. Standalone World Inspection & Simulation

- **Inspect Robot Model in RViz2:**
  ```bash
  source ~/Projects/capstone_ros2/install/setup.bash
  ros2 launch amr_description view_robot.launch.py
  ```

- **Launch Main (Realistic Blueprint) Restaurant Simulation with AMR:**
  ```bash
  source ~/Projects/capstone_ros2/install/setup.bash
  ros2 launch amr_gazebo restaurant_main.launch.py
  ```
  *(To inspect the main world without spawning the robot: `ros2 launch amr_gazebo restaurant_main.launch.py spawn_robot:=false`)*

- **Launch Test (Simple Rectangular) Restaurant Simulation with AMR:**
  ```bash
  source ~/Projects/capstone_ros2/install/setup.bash
  ros2 launch amr_gazebo restaurant_test.launch.py
  ```

---

### 3.2. End-to-End Workflow: Mapping to Autonomous Navigation

#### Step 1: Launch Full SLAM Mapping Session
Launch Gazebo Sim with the main restaurant, robot at the kitchen docking bay, EKF sensor fusion, SLAM Toolbox, and RViz2:
```bash
source ~/Projects/capstone_ros2/install/setup.bash
ros2 launch amr_bringup amr_slam.launch.py
```
*(Note: To map the test rectangular world instead, add argument: `world_type:=test`)*

#### Step 2: Teleoperate AMR to Scan the Environment (in a separate terminal)
Drive the robot smoothly through the restaurant corridors to scan all tables, chairs, and perimeter walls:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

#### Step 3: Save the Generated Occupancy Grid Map
Once the restaurant layout is completely mapped in RViz2 and loop closures are verified, save the map:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/Projects/capstone_ros2/src/amr_navigation/maps/restaurant_main
```
*(This produces `restaurant_main.yaml` and `restaurant_main.pgm` in `src/amr_navigation/maps/`)*

#### Step 4: Launch Nav2 Autonomous Navigation
Launch autonomous navigation with Nav2 ($A^*$ Global Planner + DWB Local Controller) in the mapped main restaurant:
```bash
source ~/Projects/capstone_ros2/install/setup.bash
ros2 launch amr_bringup amr_navigation.launch.py map:=/home/quangtran/Projects/capstone_ros2/src/amr_navigation/maps/restaurant_main.yaml
```
*(In RViz2, use the **2D Pose Estimate** tool to initialize the robot pose if needed, then use **Nav2 Goal** to send food delivery targets to any dining table).*

#### Step 5: Costmap Inflation & In-Place Rotation Optimization
- **Parameters Adjusted (`src/amr_navigation/config/nav2_params.yaml`):**
  - `inflation_radius`: Reduced to $0.31\text{m}$ (only $1\text{cm}$ safety margin beyond robot half-width $r_{\text{inscribed}} = 0.30\text{m}$).
  - `cost_scaling_factor`: Increased to $15.0$ (steep exponential decay away from obstacles).
  - DWB In-Place Rotation Penalties Proactively Relaxed:
    - `GoalAlign.scale`: Disabled (`0.0`, previously `24.0`) to stop the controller from trying to turn towards distant goal locations through narrow corridor walls.
    - `PathDist.scale`: Reduced from $32.0$ to $12.0$ (proactively eliminates severe penalties for zero-forward-velocity in-place pivots).
    - `GoalDist.scale`: Reduced from $24.0$ to $8.0$.
    - `ObstacleFootprint.scale` & `BaseObstacle.scale`: Lowered to $0.01$ (prevents rejecting valid in-place turning trajectories when corners sweep near tight boundaries).
    - `Oscillation.scale`: Reduced from $1.0$ to $0.2$ (prevents false oscillation abortion during directional pivots).
    - `vtheta_samples`: Increased to $35$ for finer rotational trajectory sampling.
  - `collision_monitor`: Reduced `time_before_collision` from $1.2\text{s}$ to $0.6\text{s}$ to prevent safety watchdog from premature velocity dampening in narrow spaces.
  - `behavior_server`: Reduced `simulate_ahead_time` to $1.0\text{s}$ for reliable recovery behaviors.
- **Clearance Validation:**
  - **Kitchen Docking Stall ($0.85\text{m}$ width):** With $2 \times 0.31\text{m} = 0.62\text{m} < 0.85\text{m}$, an unobstructed zero-cost corridor of $0.23\text{m}$ width is formed along the centerline, permitting the AMR ($0.60\text{m}$ width) to navigate smoothly without wall friction.
  - **Dining Tables ($0.60\text{m}$ leg gap):** With $2 \times 0.31\text{m} = 0.62\text{m} > 0.56\text{m}$ gap between leg faces, table leg inflation fields continuously overlap into lethal inscribed cost ($253$). The dining table perimeter remains an impenetrable obstacle.

---

## 4. Realistic Restaurant Architectural Simulation Model (Complete)

- **World Model File:** `src/amr_gazebo/worlds/restaurant_blueprint.world` (executed via `restaurant_main.launch.py`)
- **Key Architectural Features:**
  - **Perimeter & Structural Layout:** Enclosed perimeter walls with 4 main structural pillars ($1.56\text{m} \times 1.56\text{m}$), internal kitchen division walls with a dedicated AMR docking station in Area 08 Main Hall at $(X=8.85\text{m}, Y=-3.80\text{m})$, and private service rooms 06 & 07.
  - **Service Counters & Reception:**
    - Dual-layer slanted Bar 04 counters with warm wood and polished granite surfaces.
    - Dual-layer Bar 05 buffet counter with 6 cylindrical pedestal bar stools ($D=0.36\text{m}$).
    - Main entrance reception desk and cozy waiting bench aligned along the entrance facade.
  - **Dining Modules & Seating Clusters:**
    - Standard Group 01 benchmark rectangular dining tables ($0.67\text{m} \times 1.38\text{m}$) and single dining tables ($0.67\text{m} \times 0.69\text{m}$).
    - 3-fold circular dining clusters (Group 02 and north/south clusters) with rotational symmetry.
    - Central dining greenery island featuring a square planter box and foliage tree.
    - Wall dining rows along north walls and slanted west walls.
  - **Realistic Hollow Furniture Legs (LiDAR Realism):**
    - All dining tables and chairs feature hollow bases with individual rectangular prism legs.
    - 2D LiDAR scanning beams at height $Z \approx 0.20-0.29\text{m}$ penetrate cleanly underneath tabletops and seats, detecting only the legs to faithfully reproduce real-world SLAM obstacle avoidance behavior.
  - **Standardized Chair Backrests:**
    - All 84 square dining chairs have been geometrically realigned: backrests run strictly **parallel** to their corresponding table edges and are situated on the **outer perimeter** facing away from the tables.
  - **Corridor Clearances:** All robot transit corridors maintain generous clear passage widths ($1.20\text{m}$ to $2.20\text{m}$), safely exceeding the AMR circumscribed footprint ($0.93\text{m}$ diagonal).
