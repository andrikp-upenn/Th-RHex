# Th-RHex Quick Start Guide

This is a minimal setup guide for new developers to get Th-RHex running within a Dockerized ROS 2 Jazzy + Gazebo Harmonic environment.

---

## Prerequisites

- **Host OS:** Ubuntu 22.04 LTS  
- **Dependencies:** Docker Engine + NVIDIA drivers (if using GPU)  

---

## Quick Setup

```bash
# 1. Clone the repository
git clone git@github.com:andrikp-upenn/Th-RHex.git
cd Th-RHex

# 2. Generate environment variables
./scripts/gen-env.sh

# 3. Enable GUI access (for RViz/Gazebo)
xhost +local:root

# 4. Build the container
docker compose build ros-jazzy

# 5. Run in ephemeral mode (quick test)
docker compose run --rm ros-jazzy bash

# 6. OR persistent mode (recommended for daily use)
docker compose up -d ros-jazzy
docker compose exec ros-jazzy bash
```

Once inside the container:

```bash
cd /workspaces
colcon build --symlink-install
source install/setup.bash
gz sim -r shapes.sdf
```

URDF sanity check:

```bash
ros2 run th_rhex_description validate_urdf.sh
ros2 launch th_rhex_description display.launch.py
```

---

## Learn More

For detailed usage, development workflow, and explanations of Dockerfile and Compose setup, see the full documentation in [`README.md`](./README.md).
