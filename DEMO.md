# Th-RHex Holonomic Demo

Run all commands from the repository root unless noted.

1. Start the Docker container.

```bash
docker compose up -d ros-jazzy
docker compose exec ros-jazzy bash
```

2. Build and source the workspace inside the container.

```bash
cd /workspaces
colcon build --symlink-install
source /workspaces/install/setup.bash
```

3. Launch Gazebo with the 3-leg robot.

```bash
ros2 launch th_rhex_description_3leg gazebo.launch.py
```

4. In a second container terminal, launch the Jacobian controller.

```bash
cd /workspaces
source /workspaces/install/setup.bash
ros2 run th_rhex_control jacobian_controller
```

5. In a third container terminal, run the demo.

```bash
cd /workspaces
source /workspaces/install/setup.bash
./scripts/demo.sh
```

Reset options:

```bash
./scripts/reset_robot.sh
```

The demo menu also has:

```text
t  Tele-op velocity mode
r  Reset robot position
```

The reset helper intentionally avoids delete/respawn during the live demo. It
zeros `/cmd_vel`, resets the existing Gazebo model pose, and checks that the
controller topics are still present.

If reset fails or the robot stops listening to commands, restart Gazebo and the
Jacobian controller before continuing the demo.
