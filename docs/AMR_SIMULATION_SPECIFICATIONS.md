# ROS 2 Simulation Specifications and System Architecture for Food Delivery AMR
> **Extracted from Graduation Capstone Project:** *Analysis and Design of an Autonomous Mobile Robot (AMR) for Food Delivery in Restaurant Environments*  
> **Author:** Tran Danh Quang  
> **Framework:** ROS 2 (Jazzy Jalisco / Ubuntu 24.04 LTS)  
> **Simulation Engine:** Gazebo (Ignition / Gazebo Harmonic)

---

## 1. Geometric and Dynamic Modeling (URDF / SDF Specifications)

### 1.1. Drive Configuration & Wheel Architecture
- **Drive Type:** Differential Drive (2 independent active drive wheels) + 4 passive spring-loaded swivel caster wheels.
- **Track Width / Wheelbase ($b$):** $B = 500\text{ mm} = 0.5\text{ m}$.
- **Axle Layout & Distance:**
  - Longitudinal distance from drive axle to front/rear caster clusters: $L = 330\text{ mm} = 0.33\text{ m}$.
  - Lateral spacing between two casters on the same axle: $400\text{ mm} = 0.4\text{ m}$.
- **Active Drive Wheel Parameters:**
  - Wheel diameter: $D_w = 150\text{ mm} \implies$ Wheel radius: $r = 75\text{ mm} = 0.075\text{ m}$.
  - Material: Polyurethane (PU) tread with cast-iron hub.
  - Friction coefficients (Gazebo ODE/Bullet):
    - Kinetic friction: $\mu_k = 0.4$
    - Static friction: $\mu_s = 0.6$
- **Passive Swivel Caster Parameters:**
  - Caster wheel diameter: $50\text{ mm}$ ($r_c = 0.025\text{ m}$).
  - Assembly mounting height ($H$): $74\text{ mm}$.
  - Caster mechanism: $360^\circ$ continuous rotation swivel with independent spring-damper suspension.

### 1.2. Envelope Dimensions and Center of Mass (CoM)
- **Base Chassis Dimensions:**
  - Width: $600\text{ mm} = 0.6\text{ m}$ (Strict upper threshold: $< 900\text{ mm}$).
  - Length: $760\text{ mm} = 0.76\text{ m}$.
  - Overall height (including 3-tier food tray structure): $1100 - 1300\text{ mm}$.
  - Ground clearance ($H_g$): $35\text{ mm} = 0.035\text{ m}$.
- **Food Trays (3 Tiers):** $360\text{ mm} \times 720\text{ mm}$ per tray tier.
- **Mass and Inertia Distribution:**
  - Robot curb weight (unloaded mass): $30\text{ kg}$.
  - Maximum payload capacity: $15\text{ kg}$ ($3\text{ tiers} \times 5\text{ kg/tier}$).
  - Total gross nominal mass ($m$): $45\text{ kg}$.
  - Nominal Center of Mass (CoM):
    - Vertical elevation: $h = 0.5\text{ m}$ above ground (accounting for multi-tier food payloads).
    - Planar CoM: Aligned with the geometric center of the base chassis.

---

## 2. Kinematic Constraints and Dynamic Operating Limits

These parameters govern the low-level controller (`diff_drive_controller`), velocity profilers, and the Nav2 local trajectory planner (`DWB Local Planner`):

- **Maximum Linear Velocity ($v_{\max}$):** $1.0\text{ m/s}$.
- **Maximum Linear Acceleration ($a_{\max}$):** $0.5\text{ m/s}^2$.
- **Zero-Radius Turning (In-place rotation):** Supported ($v = 0, \omega \neq 0$).
- **Maximum Wheel Angular Velocity ($\omega_{\text{wheel}}$):**
  $$\omega_{\text{wheel}} = \frac{v_{\max}}{r} = \frac{1.0}{0.075} \approx 13.33\text{ rad/s}$$
- **Maximum Wheel Angular Acceleration ($\gamma_{\text{wheel}}$):**
  $$\gamma_{\text{wheel}} = \frac{a_{\max}}{r} = \frac{0.5}{0.075} \approx 6.67\text{ rad/s}^2$$
- **Chassis Angular Kinematics ($\omega, \alpha$):**
  - Maximum chassis angular velocity:
    $$\omega_{\max} = \frac{2 \cdot v_{\max}}{b} = \frac{2 \cdot 1.0}{0.5} = 4.0\text{ rad/s}$$
  - Maximum chassis angular acceleration:
    $$\alpha_{\max} = \frac{2 \cdot a_{\max}}{b} = \frac{2 \cdot 0.5}{0.5} = 2.0\text{ rad/s}^2$$

---

## 3. Sensors, Interfaces, and Gazebo Simulation Plugins

### 3.1. 2D LiDAR (Slamtec RPLIDAR A2M8)
- **Operating Range:** $0.2\text{ m} - 12.0\text{ m}$.
- **Field of View (FOV):** $360^\circ$ ($-\pi \text{ to } +\pi\text{ rad}$).
- **Angular Resolution:** $0.45^\circ$ (~800 range samples per rotation).
- **Scan Frequency:** $10\text{ Hz}$ (Configurable in the range $5 - 15\text{ Hz}$).
- **ROS 2 Output Topic:** `/scan` (`sensor_msgs/msg/LaserScan`).
- **Target Frame ID:** `lidar_link` / `laser_frame`.

### 3.2. 9-DOF IMU (Bosch BNO055)
- **Sampling Frequency:** $100\text{ Hz}$.
- **Telemetry Data:** 3-axis Linear Acceleration, 3-axis Angular Velocity, Orientation Quaternion / Roll-Pitch-Yaw.
- **ROS 2 Output Topic:** `/imu/data` (`sensor_msgs/msg/Imu`).
- **Target Frame ID:** `imu_link`.

### 3.3. Wheel Encoders (Integrated in K8XS50N2-E BLDC Motors)
- Direct measurement of left and right wheel velocities and angular displacements for dead reckoning.
- **Odometry Output Topic:** `/wheel/odom` (`nav_msgs/msg/Odometry`).
- **Joint State Topic:** `/joint_states` (`sensor_msgs/msg/JointState`).

### 3.4. Safety Contact Bumpers
- Tactile rubber bumpers mounted on the front and rear perimeters for physical emergency stop (E-Stop).
- Simulated via Gazebo contact sensor plugins to detect obstacle impact.

---

## 4. Coordinate Frame Transformations (TF Tree)

Standard ROS 2 navigation guidelines dictate the following TF tree hierarchy:

```text
map ──(SLAM Toolbox / AMCL)──> odom ──(robot_localization / EKF)──> base_footprint ──> base_link
                                                                                           ├── left_wheel_link
                                                                                           ├── right_wheel_link
                                                                                           ├── caster_fl_link
                                                                                           ├── caster_fr_link
                                                                                           ├── caster_rl_link
                                                                                           ├── caster_rr_link
                                                                                           ├── laser_frame
                                                                                           └── imu_link
```

- `base_footprint`: Planar projection of the robot footprint onto the ground plane ($z = 0$).
- `base_link`: Geometric center of the robot located on the active drive axle.

---

## 5. Sensor Fusion and State Estimation (EKF - `robot_localization`)

State estimation is handled by `ekf_node` from the `robot_localization` package:
- **Input Stream 1:** Wheel Odometry (`/wheel/odom`) $\implies$ Linear velocity $v_x$, Angular velocity $\omega_z$.
- **Input Stream 2:** Inertial Measurement Unit (`/imu/data`) $\implies$ Angular velocity $\omega_z$, Linear acceleration $a_x$.
- **Output:** Filtered odometry topic `/odometry/filtered` and continuous broadcast of the `odom -> base_footprint` TF transform.
- **Design Objective:** Eliminate odometry drift, wheel slippage errors, and rotational unbounded accumulation over long mission cycles.

---

## 6. Autonomous Navigation & Mapping Stack (Nav2 & SLAM)

### 6.1. SLAM (Mapping Phase)
- **Selected Package:** `slam_toolbox` (Online Asynchronous SLAM mode).
- **Sensor Inputs:** `/scan` and `/odometry/filtered`.
- **Occupancy Grid Resolution:** $\Delta x = \Delta y = 0.05\text{ m}$ ($5\text{ cm/cell}$).
- **Loop Closure:** Automatic loop closure detection upon returning to the kitchen dispatch area or main corridor.

### 6.2. Localization (Autonomous Running Phase)
- **Selected Package:** `Nav2 AMCL` (Adaptive Monte Carlo Localization).
- **Particle Cloud Range:** $50 - 500$ dynamic particles.
- **Goal Convergence Tolerance:** $<\pm 10\text{ mm}$ position error at table docking points.

### 6.3. Global Path Planning
- **Algorithm:** $A^*$ (Navfn Planner / Smac Planner 2D).
- **Global Costmap Configuration:**
  - Grid cell size: $0.05\text{ m}$.
  - Inflation Layer: Inscribed radius $r_{\text{inscribed}} \approx 0.47\text{ m}$ (Robot diagonal is $930\text{ mm} \implies r_{\text{circumscribed}} \approx 0.465\text{ m}$).
  - Exponential Cost Decay Function:
    $$Cost(d) = 253 \cdot e^{-\alpha(d - r_{\text{inscribed}})}$$
    Tuned to bias paths toward the centerline of restaurant corridors and avoid skimming dining chairs.

### 6.4. Local Trajectory Controller & Dynamic Obstacle Avoidance
- **Algorithm:** DWB (Dynamic Window Approach Based Controller).
- **Operational Envelope:**
  - Linear velocity: $v \in [0.0, 1.0]\text{ m/s}$.
  - Angular velocity: $\omega \in [-2.0, 2.0]\text{ rad/s}$.
  - Max linear acceleration: $0.5\text{ m/s}^2$; Max angular acceleration: $2.0\text{ rad/s}^2$.
  - Forward trajectory simulation time ($T_{\text{sim}}$): $1.5 - 2.0\text{ s}$.
- **Mandatory Trajectory Critics Configuration:**
  - `OscillationCritic`: Penalizes abrupt directional oscillation and rapid steering reversals to prevent liquid/soup spillage.
  - `BaseObstacleCritic`: Evaluates real polygonal footprint collision against dynamic obstacles, dining tables, and chair legs.
  - `PathAlignCritic`: Enforces robot heading alignment along the tangent of the global $A^*$ path.
  - `GoalAlignCritic` & `GoalDistCritic`: Guarantees precise terminal angle and standoff distance when docking at dining tables.

---

## 7. Simulation World Environment (Gazebo World Setup)

- **Target Facility Layout:** **Pizza 4P's Landmark 72 (Hanoi)**:
  - Total simulated indoor area: Approximately $330 - 400\text{ m}^2$.
  - Main corridor clearance: Minimum $1300\text{ mm}$ ($1.3\text{ m}$).
  - Secondary service aisles (between dining tables): Minimum $1000\text{ mm}$ ($1.0\text{ m}$).
  - Home / Docking Station: Open kitchen dispatch counter (serving station).
  - Target Navigation Goals: Indoor customer dining tables and outdoor veranda tables.
- **Floor Surface & Terrain Profile:**
  - Planar tiled flooring with discrete joint crevices and irregularities of $\pm 10\text{ mm}$.
  - The simulated suspension system must damp these high-frequency vibrations to ensure LiDAR scan stability and prevent point cloud distortion.


