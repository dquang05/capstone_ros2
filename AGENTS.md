# Repository Guidelines

## Project Structure & Module Organization

This ROS 2 Jazzy / Gazebo Harmonic workspace simulates a food-delivery AMR. Five `ament_python` packages live under `src/`:

- `amr_description`: URDF/Xacro, meshes, and robot visualization.
- `amr_gazebo`: worlds, models, simulation launches, and wheel limiting.
- `amr_localization`: EKF configuration and localization launches.
- `amr_navigation`: Nav2/SLAM configuration, maps, and table dispatch.
- `amr_bringup`: integrated launches and RViz configurations.

Package tests live in `src/<package>/test/`. Validation tools and tests are under `scripts/{ekf,navigation,slam}_validation/`; engineering references are in `docs/`. Generated `build/`, `install/`, `log/`, `benchmark/`, and `report/` directories are ignored.

## Build, Test, and Development Commands

Run from the repository root; source the underlay before building and the overlay before launching:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch amr_bringup amr_navigation.launch.py
```

- Add `--packages-select amr_gazebo` to build only that package.
- `ros2 launch amr_bringup amr_slam.launch.py` starts mapping.
- `colcon test` runs package tests; `colcon test-result --verbose` reports failures.
- `python3 -m unittest discover -s scripts/ekf_validation -p 'test_*.py'` runs synthetic EKF analysis tests.

Register new assets and executables in the owning package's `setup.py`, declare dependencies in `package.xml`, and rebuild.

## Coding Style & Naming Conventions

Use four-space Python indentation, `snake_case` functions/modules, and `PascalCase` classes. Package checks use `ament_flake8` and `ament_pep257`. Keep package names prefixed with `amr_` and launch filenames ending in `.launch.py`. ROS nodes should inherit from `Node`, use ROS logging, and clean up on shutdown.

Follow SI units and the `map → odom → base_footprint → base_link` frame hierarchy. Use simple collision geometry and analytical inertia macros. See `docs/WORKSPACE_STANDARDS_AND_WORKFLOW.md` for modeling rules.

## Testing Guidelines

Name tests `test_*.py`; package tests use pytest, while validation suites also use unittest. Add regression checks for changed calculations or control behavior. No numerical coverage threshold is configured. For simulation changes, record the world, map, parameters, and observed results; distinguish synthetic checks from Gazebo validation.

## Commit & Pull Request Guidelines

Recent history favors Conventional Commits: `feat(navigation): ...`, `fix(localization): ...`, or `docs: ...`. Keep commits focused. PRs should explain behavior changes, link relevant issues, list validation commands/results, and include RViz/Gazebo screenshots when geometry or navigation behavior changes. Keep generated outputs out of commits and update affected documentation.
