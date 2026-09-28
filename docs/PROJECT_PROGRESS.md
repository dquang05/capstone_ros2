# AMR Simulation Project Progress

> **Project:** Autonomous Mobile Robot (AMR) for Restaurant Food Delivery  
> **Framework:** ROS 2 Jazzy Jalisco (Ubuntu 24.04 LTS) | Simulation Engine: Gazebo Harmonic  
> **Last Updated:** 2026-09-28  

---

## 1. Current Status (Visual Mesh Alignment - Completed)

All visual mesh assets in `amr_description` have been calibrated, centered, and verified:

- **Chassis & Body Housing:** Main base chassis, lower housing, and top cover are aligned with standard 35 mm ground clearance.
- **Payload Trays:** 3-tier food trays aligned along the robot centerline at heights $Z = 0.30\text{ m}$, $0.52\text{ m}$, and $0.74\text{ m}$.
- **Side Assemblies:** Vertical skeleton pillars, spacers, and drive wheel covers are symmetrically aligned across both sides.
- **Head / Display Unit:** Mounted flush on top of the skeleton pillars at $Z = 0.904\text{ m}$.
- **Wheels & Mobility:** 
  - Differential drive wheels: Track width $0.50\text{ m}$, radius $0.075\text{ m}$.
  - Passive caster wheels: 4x omnidirectional casters leveled with drive wheels at the ground plane ($Z = 0$).
- **LiDAR Sensor:** 2D LiDAR placed on the designated top-cover mounting pedestal with an unobstructed $360^\circ$ field of view.

---

## 2. Next Session Plan (Physical Dynamics & Simulation)

The next phase focuses on physical and dynamical properties for Gazebo simulation:

1. **Analytical Collision Geometry:** Replace visual meshes with simple collision primitives (boxes, cylinders, spheres) to ensure efficient and stable physics.
2. **Mass Distribution & CoM:** Set mass budgets for all links (nominal $45\text{ kg}$ total: $30\text{ kg}$ curb weight + $15\text{ kg}$ payload) and compute multi-tier Center of Mass.
3. **Inertia Tensors:** Generate positive-definite rotational inertia tensors ($I_{xx}, I_{yy}, I_{zz}$) for each link using standard formulas.
4. **Surface & Contact Physics:** Configure Gazebo wheel friction ($\mu_1, \mu_2$) and surface contact stiffness/damping.
---

## 3. Launch Command

```bash
ros2 launch amr_description display.launch.py
```
