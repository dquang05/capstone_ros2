# AMR Simulation Project Progress

> **Project:** Autonomous Mobile Robot (AMR) for Restaurant Food Delivery  
> **Framework:** ROS 2 Jazzy (Ubuntu 24.04 LTS) | Simulation Engine: Gazebo  
> **Last Updated:** 2026-09-30  

---

## 1. Current Status

- **Robot structure is preliminarily complete:**
  - Base chassis plate (25 kg) and 3-tier food delivery trays (5 kg each) are aligned with full collision geometry and calibrated inertial matrices.
  - 6-wheel mobility system (2 differential drive wheels, 4 passive caster wheels) leveled coplanar with the ground.
  - Front-mounted 2D LiDAR, center 9-DOF IMU, and wheel encoder feedback (joint states & diff drive controller) are integrated with Gazebo simulation plugins.

---

## 2. Next Steps

- Integrate the restaurant simulation environment (Gazebo world) and test navigation.

---

## 3. Launch Command

```bash
source ~/Projects/capstone_ros2/install/setup.bash
ros2 launch amr_description view_robot.launch.py
```
