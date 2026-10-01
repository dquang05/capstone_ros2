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
    - **Decorative Greenery:** Potted indoor plants placed in key corners.
  - Built purely with lightweight analytical primitives (0 trimesh lag, 100% RTF, crisp LiDAR reflection).
  - Launch file `restaurant_world.launch.py` defaults spawn to the Kitchen Home Station facing South out into the main corridor.

---

## 2. Next Steps

- Launch simulation in Gazebo, verify robot spawning and sensor topic streams (`/scan`, `/imu/data`, `/joint_states`, `/wheel/odom`).
- Configure 2D SLAM (SLAM Toolbox) for mapping the restaurant.

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
