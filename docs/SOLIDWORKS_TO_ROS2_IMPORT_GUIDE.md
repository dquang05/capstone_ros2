# Complete Guide: Porting SolidWorks CAD Models (Windows) to ROS 2 (Linux)
> **Target Project:** Food Delivery AMR (Restaurant Service Robot)  
> **Target Environment:** ROS 2 Jazzy Jalisco / Gazebo (Harmonic or Classic)  
> **Reference Specifications:** [`docs/AMR_SIMULATION_SPECIFICATIONS.md`](file:///home/quangtran/Projects/capstone_ros2/docs/AMR_SIMULATION_SPECIFICATIONS.md)  
> **Key Note:** The raw SolidWorks CAD parameters are known to be partially inaccurate. This guide focuses on extracting clean visual meshes from Windows and performing **manual mathematical calibration** in Linux for joint origins, collision geometries, inertias, and kinematics.

---

## 1. Core Engineering Concept: Why Raw CAD $\neq$ Simulation Model

In professional robotics engineering, importing raw CAD directly into Gazebo/ROS 2 usually fails or causes simulation instabilities (drifting, physics exploding with `NaN`, or extreme CPU lag).

| CAD Model (SolidWorks) | Simulation Model (ROS 2 / Gazebo URDF) |
| :--- | :--- |
| **Origin:** Arbitrary or assembly-relative. | **Origin:** `base_link` strictly located at the midpoint of the active drive axle on the floor plane. |
| **Visual Geometry:** Hundreds of thousands of faces (screws, fillets, internal electronics). | **Visual Geometry:** Simplified outer shell mesh (`.dae` or `.stl`), low polygon count ($< 50,000$ triangles). |
| **Collision Geometry:** Mesh-to-mesh complex polygon collision (causes massive physics slowdowns). | **Collision Geometry:** Simple analytic primitives (**Cylinder, Box, Sphere**). Fast, robust, and glitch-free. |
| **Inertia ($I_{xx}, I_{yy}, I_{zz}$):** Often unassigned or inaccurate materials in CAD. | **Inertia:** Rigorously calculated based on nominal mass distribution ($45\text{ kg}$) to guarantee positive-definite tensors. |
| **Coordinate System:** Usually Y-Up. | **Coordinate System:** Strictly ROS standard (REP-103): **X-Forward, Y-Left, Z-Up**. |

---

## 2. Phase 1: SolidWorks Preparation & Export (on Windows)

You have two workflows to export from SolidWorks on Windows. **Method B (Hybrid Approach)** is strongly recommended for your project because your CAD dimensions need manual calibration.

```
┌────────────────────────────────────────────────────────┐
│               SOLIDWORKS (WINDOWS)                     │
│  - Method A: SW2URDF Exporter Plugin (Automated)       │
│  - Method B: Individual Mesh Export (Recommended)      │
└──────────────────────────┬─────────────────────────────┘
                           │ Copy over network / USB / WSL
                           ▼
┌────────────────────────────────────────────────────────┐
│               ROS 2 WORKSPACE (LINUX)                  │
│  1. Organize meshes into amr_description/meshes/       │
│  2. Build modular Xacro (chassis, wheels, casters)     │
│  3. Manually override origins, collisions & inertias   │
│  4. Validate in RViz2 and Gazebo                       │
└────────────────────────────────────────────────────────┘
```

### Method A: Automated Export via `sw_urdf_exporter` (SW2URDF)
1. **Download & Install:** Download the official [SolidWorks to URDF Exporter](http://wiki.ros.org/sw_urdf_exporter) installer for Windows.
2. **Define Reference Geometry:**
   - In your assembly, create a Reference Coordinate System at the midpoint between the two active drive wheels. Ensure:
     - **X-axis points FORWARD** (direction of robot travel).
     - **Y-axis points LEFT** (towards left drive wheel).
     - **Z-axis points UP** (towards the ceiling).
3. **Configure Link Hierarchy in the Exporter Tree:**
   - `base_link` $\rightarrow$ Main chassis and trays.
   - `left_wheel_link` (Joint type: *continuous*, axis: `[0, 1, 0]`).
   - `right_wheel_link` (Joint type: *continuous*, axis: `[0, 1, 0]`).
   - `caster_fl_link`, `caster_fr_link`, `caster_rl_link`, `caster_rr_link`.
   - `lidar_link` (Joint type: *fixed*).
4. **Export Settings:**
   - Select output mesh format: **STL** or **DAE** (COLLADA).
   - Set units to **Meters** and **Kilograms**.
   - Click *Export Robot and Meshes*.

### Method B: Hybrid / Industry Best-Practice Export (Recommended)
Because you must recalibrate dimensions and joints manually, exporting the distinct visual components as individual `.STL` or `.STEP` files is much cleaner:
1. **Suppress Internal Detail:** Hide/suppress non-structural parts (bolts, washers, wires, internal motor coils).
2. **Export Individual Visual Parts as Binary STL:**
   - `base_chassis.stl`: The entire exterior chassis, frame, and 3 food trays.
   - `drive_wheel.stl`: Single wheel model (reusable for both left and right wheels).
   - `caster_swivel.stl` and `caster_wheel.stl`: Caster cluster components (or use a simplified representation).
   - `lidar.stl`: RPLIDAR A2 housing (optional, can use open-source mesh).
3. **Export Coordinate Warning:** When saving STL in SolidWorks (*Save As $\rightarrow$ STL $\rightarrow$ Options*):
   - Set Output Units to **Meters** (default is often millimeters, which makes parts $1000\times$ too large in ROS!).
   - Choose output coordinate system: Use the custom ROS coordinate frame (X-forward, Y-left, Z-up).

---

## 3. Phase 2: Transferring Files to Linux & Workspace Structure

### 3.1. Directory Setup in Linux
Transfer your exported folder to your ROS 2 workspace at `/home/quangtran/Projects/capstone_ros2/src/`. Create the dedicated robot description package:

```bash
cd /home/quangtran/Projects/capstone_ros2/src
ros2 pkg create amr_description --build-type ament_python
mkdir -p amr_description/urdf
mkdir -p amr_description/meshes/visual
mkdir -p amr_description/meshes/collision
mkdir -p amr_description/launch
mkdir -p amr_description/rviz
```

Place your exported mesh files (`.stl` or `.dae`) inside:
`amr_description/meshes/visual/`

### 3.2. Fix Line Endings (Windows CRLF to Linux LF)
Files exported from Windows often contain `\r\n` line endings. Run:
```bash
sudo apt-get install -y dos2unix
dos2unix /home/quangtran/Projects/capstone_ros2/src/amr_description/urdf/*
```

---

## 4. Phase 3: Constructing Modular Xacro (Manual Calibration)

Instead of maintaining a fragile 1000-line URDF file, split the robot into modular Xacro files inside `amr_description/urdf/`:

```text
amr_description/urdf/
├── amr.urdf.xacro          # Main entrypoint combining all modules
├── properties.xacro        # Calibrated geometric & dynamic variables
├── macros_inertia.xacro    # Analytical inertia tensor calculators
├── chassis.xacro           # base_footprint, base_link, and main body
├── drive_wheels.xacro      # Left and Right differential wheels
├── caster_wheels.xacro     # 4 spring-loaded or low-friction casters
├── sensors.xacro           # RPLIDAR A2, BNO055 IMU
└── gazebo_plugins.xacro    # diff_drive, joint_state, and sensor plugins
```

### 4.1. Inertia Macros (`macros_inertia.xacro`)
Gazebo requires strictly positive inertia tensors. If an inertia tensor is non-positive definite, the simulation will crash or throw `NaN` position errors. Use analytical box and cylinder macros:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Solid Box Inertia -->
  <xacro:macro name="inertial_box" params="mass x y z *origin">
    <inertial>
      <xacro:insert_block name="origin"/>
      <mass value="${mass}"/>
      <inertia ixx="${(1/12) * mass * (y*y + z*z)}" ixy="0.0" ixz="0.0"
               iyy="${(1/12) * mass * (x*x + z*z)}" iyz="0.0"
               izz="${(1/12) * mass * (x*x + y*y)}"/>
    </inertial>
  </xacro:macro>

  <!-- Solid Cylinder Inertia (along Z axis) -->
  <xacro:macro name="inertial_cylinder" params="mass length radius *origin">
    <inertial>
      <xacro:insert_block name="origin"/>
      <mass value="${mass}"/>
      <inertia ixx="${(1/12) * mass * (3*radius*radius + length*length)}" ixy="0.0" ixz="0.0"
               iyy="${(1/12) * mass * (3*radius*radius + length*length)}" iyz="0.0"
               izz="${(1/2) * mass * radius * radius}"/>
    </inertial>
  </xacro:macro>

  <!-- Solid Sphere Inertia -->
  <xacro:macro name="inertial_sphere" params="mass radius *origin">
    <inertial>
      <xacro:insert_block name="origin"/>
      <mass value="${mass}"/>
      <inertia ixx="${(2/5) * mass * radius*radius}" ixy="0.0" ixz="0.0"
               iyy="${(2/5) * mass * radius*radius}" iyz="0.0"
               izz="${(2/5) * mass * radius*radius}"/>
    </inertial>
  </xacro:macro>

</robot>
```

---

## 5. Phase 4: Step-by-Step Manual Calibration (Fixing CAD Inaccuracies)

Here are the exact mathematical corrections needed to match the requirements in [AMR_SIMULATION_SPECIFICATIONS.md](file:///home/quangtran/Projects/capstone_ros2/docs/AMR_SIMULATION_SPECIFICATIONS.md):

### 5.1. Base Frame & Origin Alignment
- **`base_footprint`:** Ground contact plane ($z = 0$).
- **`base_link`:** Midpoint of the active drive axle.
  - Since drive wheel radius $r = 75\text{ mm} = 0.075\text{ m}$, the joint connecting `base_footprint` to `base_link` must be elevated by $+0.075\text{ m}$ in $z$:
    ```xml
    <joint name="base_footprint_joint" type="fixed">
      <parent link="base_footprint"/>
      <child link="base_link"/>
      <origin xyz="0 0 0.075" rpy="0 0 0"/>
    </joint>
    ```

### 5.2. Active Drive Wheels Calibration
Regardless of CAD inaccuracies, override the wheel joints to the exact specifications:
- **Track Width:** $B = 0.5\text{ m} \implies$ Wheel offset $y = \pm B/2 = \pm 0.25\text{ m}$.
- **Wheel Radius:** $r = 0.075\text{ m}$. Wheel thickness: $\approx 0.04\text{ m}$.
- **Collision:** DO NOT use CAD mesh. Use a `<cylinder>` primitive.
- **Orientation:** In ROS URDF, a cylinder lies along the Z-axis by default. Rotate by $\pi/2$ around X or Y to align with the wheel rotational axis.

```xml
<!-- Left Drive Wheel -->
<joint name="left_wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="left_wheel_link"/>
  <origin xyz="0 0.25 0" rpy="-${pi/2} 0 0"/>
  <axis xyz="0 0 1"/>
</joint>

<link name="left_wheel_link">
  <visual>
    <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
    <geometry>
      <!-- Visual CAD Mesh -->
      <mesh filename="package://amr_description/meshes/visual/drive_wheel.stl" scale="1 1 1"/>
    </geometry>
    <material name="black"/>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Analytical Cylinder for Fast Collision -->
      <cylinder radius="0.075" length="0.04"/>
    </geometry>
  </collision>
  <xacro:inertial_cylinder mass="2.0" length="0.04" radius="0.075">
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:inertial_cylinder>
</link>
```
*(Right wheel joint is placed at `xyz="0 -0.25 0"` with `rpy="${pi/2} 0 0"`).*

### 5.3. Passive Caster Wheels Calibration (4 Casters)
According to the specification:
- Longitudinal offset from drive axle: $L = 0.33\text{ m}$ (Front casters: $+0.33\text{ m}$, Rear casters: $-0.33\text{ m}$).
- Lateral offset: $0.4\text{ m}$ spacing $\implies y = \pm 0.20\text{ m}$.
- Vertical elevation: Caster wheel radius $r_c = 0.025\text{ m}$. Relative to `base_link` ($z=0$ at $+0.075\text{ m}$ above ground), the caster wheel center must be at:
  $$z_{\text{caster}} = -(r - r_c) = -(0.075 - 0.025) = -0.050\text{ m}$$

#### Best Practice for Caster Collision in Gazebo:
Modeling full 3D swivel caster joints with spring dampers often causes numerical instability in Gazebo. The industry-standard simulation practice for differential drive robots is:
1. Model the visual caster swivel and wheel from CAD.
2. In `<collision>`, use a **frictionless sphere** (`<sphere radius="0.025"/>`) with:
   - Friction coefficient `mu1 = 0.0`, `mu2 = 0.0`.
   - Contact stiffness `kp = 1000000.0`, `kd = 100.0`.
This guarantees $100\%$ smooth motion, prevents caster jamming, and perfectly simulates omnidirectional ground support without altering dead reckoning odometry.

### 5.4. Collision Simplification for Chassis Body
Never use the complex SolidWorks chassis mesh for collision. Replace it with an oriented `<box>`:
```xml
<collision>
  <!-- Geometric center elevated to CoM height (h = 0.5m total, chassis center ~0.35m relative to base_link) -->
  <origin xyz="0 0 0.35" rpy="0 0 0"/>
  <geometry>
    <!-- Box: Length=0.76m, Width=0.60m, Height=0.90m -->
    <box size="0.76 0.60 0.90"/>
  </geometry>
</collision>
```

---

## 6. Phase 5: Verification & Launch Checklist in ROS 2

### 6.1. Build & Parse URDF/Xacro
Test that your Xacro generates clean URDF without syntax or XML errors:
```bash
cd /home/quangtran/Projects/capstone_ros2
colcon build --packages-select amr_description
source install/setup.bash

# Render Xacro to URDF text
xacro src/amr_description/urdf/amr.urdf.xacro > /tmp/amr_test.urdf

# Check for tree and link correctness
check_urdf /tmp/amr_test.urdf
```

### 6.2. Visual Inspection in RViz2
Launch RViz2 with `joint_state_publisher_gui` and `robot_state_publisher`:
- Verify all links are connected without broken branches.
- Toggle *Collision Enabled* in RViz to ensure cylinder collisions align exactly with visual wheel meshes.
- Use the slider to rotate the wheels and verify rotation axes.

### 6.3. Gazebo Physics Check
Spawn the robot in Gazebo:
1. Verify the robot stands stably on the floor without vibrating or jittering.
2. Check that the active wheels have firm friction contact with the floor.
3. Check `/joint_states` and send a test velocity command via `/cmd_vel` to confirm the differential drive plugin moves the robot smoothly.

---

## 7. Action Plan Summary

| Step | Action | Tools / Location |
| :---: | :--- | :--- |
| **1** | Export visual STL meshes from SolidWorks with unit = **Meters**. | SolidWorks (Windows) |
| **2** | Copy meshes into `src/amr_description/meshes/visual/`. | Linux / ROS 2 Workspace |
| **3** | Create `macros_inertia.xacro` for robust mass matrix calculation. | `amr_description/urdf/` |
| **4** | Manually set wheel origins ($y = \pm 0.25\text{ m}$) & primitives ($r=0.075\text{ m}$). | `drive_wheels.xacro` |
| **5** | Manually set caster positions ($x=\pm 0.33, y=\pm 0.20, z=-0.05$) with sphere collisions. | `caster_wheels.xacro` |
| **6** | Replace body collision with a simple $0.76 \times 0.60 \times 0.90\text{ m}$ box. | `chassis.xacro` |
| **7** | Integrate Gazebo plugins (`diff_drive`, `joint_state`, `sensor_plugins`). | `gazebo_plugins.xacro` |
