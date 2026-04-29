# URDF Import Workflow

Use this workflow to bring a generated or hand-authored Th-RHex URDF into the ROS 2 Jazzy workspace.

## File Locations

- Main model: `src/th_rhex_description/urdf/th_rhex.urdf.xacro`
- Imported SolidWorks URDF copy: `src/th_rhex_description/urdf/3leg_Th_Rhex.urdf`
- Meshes: `src/th_rhex_description/meshes/`
- Joint-name config: `src/th_rhex_description/config/joint_names_3leg_Th_Rhex.yaml`
- Raw export archive: `src/th_rhex_description/original_exports/3leg_Th_Rhex/`
- RViz launch check: `src/th_rhex_description/launch/display.launch.py`
- Gazebo launch check: `src/th_rhex_description/launch/gazebo.launch.py`

## Import Rules

- Prefer Xacro for the maintained model, even if the first import is a plain `.urdf`.
- Keep all mesh files inside `src/th_rhex_description/meshes/`.
- Use `package://th_rhex_description/meshes/...` mesh paths instead of absolute local paths.
- Do not use the exported `3leg_Th_Rhex` package name in maintained URDF files; that export name is archived only.
- Every simulated link needs a valid `<inertial>` block.
- Every visual mesh that should collide in Gazebo should also have a matching `<collision>` geometry.
- Joint names, link names, and frame names should be stable and lowercase with underscores.

## Validate

Run these commands inside the Docker container:

```bash
cd /workspaces
colcon build --symlink-install
source install/setup.bash
ros2 run th_rhex_description validate_urdf.sh
```

To validate a one-off import before replacing the package model:

```bash
ros2 run th_rhex_description validate_urdf.sh /workspaces/path/to/imported_robot.urdf
```

## Visual Checks

RViz:

```bash
ros2 launch th_rhex_description display.launch.py
```

Gazebo:

```bash
ros2 launch th_rhex_description gazebo.launch.py
```

## Common Import Fixes

- If meshes do not appear, fix `filename` attributes to use `package://th_rhex_description/meshes/...`.
- If RViz shows disconnected parts, check each joint's `parent`, `child`, and `origin`.
- If Gazebo refuses the model, check for missing inertial data or unsupported mesh paths.
- If the robot falls through the floor, add or simplify collision geometry.
