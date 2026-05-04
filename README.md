# Th-RHex
Holonomic RHex being developed at The University of Pennsylvania, GRASP and ModLab

# Th-RHex ROS 2 Jazzy + Gazebo Harmonic Development Environment

This repository provides a Docker-based ROS 2 Jazzy workspace for developing, simulating, and testing the Th-RHex robot.  
It uses Gazebo Harmonic for URDF and control development and will later migrate to NVIDIA Isaac Sim for GPU-accelerated reinforcement learning and photorealistic testing.

---

## Overview

| Component | Version | Purpose |
|------------|----------|----------|
| Host OS | Ubuntu 22.04 LTS (Jammy) | Stable base with mature Docker and NVIDIA driver support |
| Container OS | Ubuntu 24.04 LTS (Noble) | Runs ROS 2 Jazzy |
| Simulator | Gazebo Harmonic | Physics-accurate testing for URDF and control |
| Future Simulator | Isaac Sim | Reinforcement learning and GPU physics simulation |

Running ROS 2 inside Docker ensures every developer uses identical dependencies and tools, independent of their host setup.

---

## Folder Structure

```
Th-RHex/
├── .devcontainer/               # VS Code Dev Container configuration
│   └── devcontainer.json
├── docker/                      # Docker build files
│   └── Dockerfile.jazzy         # ROS 2 Jazzy + Gazebo Harmonic base image
├── docker-compose.yml           # Runtime configuration (network, volumes, env)
├── scripts/
│   └── gen-env.sh               # Generates .env with UID/GID/USER info
├── .env                         # Auto-generated per-user (not committed)
└── src/                         # ROS 2 packages and URDF files for Th-RHex
```

Your repository is mounted inside the container at `/workspaces`.  
Edits made on your host appear immediately within the container.

---

## Setup Instructions (First-Time Use)

### 1. Install Docker

Follow [Docker Engine for Ubuntu 22.04](https://docs.docker.com/engine/install/ubuntu/), then add your user to the Docker group:

```bash
sudo usermod -aG docker $USER
newgrp docker
```

### 2. Clone the Repository

```bash
git clone git@github.com:andrikp-upenn/Th-RHex.git
cd Th-RHex
```

### 3. Generate Environment Variables

```bash
./scripts/gen-env.sh
```

This writes your UID, GID, and username to `.env`, ensuring files created inside Docker are owned by you on the host.

### 4. Enable GUI Access

```bash
xhost +local:root
```

This allows GUI applications (RViz, Gazebo) to run from inside the container.

### 5. Build the Docker Image

```bash
docker compose build ros-jazzy
```

---

## Running the Container

There are two ways to run the container depending on your workflow.

### Option A: Ephemeral Mode (`--rm`)

```bash
docker compose run --rm ros-jazzy bash
```

- Starts a temporary container.  
- Workspace files are persistent (mounted), but the container is deleted upon exit.  
- Ideal for clean testing or short-lived sessions.

### Option B: Persistent Mode (`up / exec`)

```bash
docker compose up -d ros-jazzy     # Start container (detached)
docker compose exec ros-jazzy bash # Enter running container
```

- Container remains active until manually stopped.  
- Preserves build caches and terminal history.  
- Recommended for daily development.

To stop all containers:

```bash
docker compose down
```

---

## Inside the Container

When the container starts, ROS 2 Jazzy is automatically sourced.  
The workspace root is `/workspaces`.

Example build and test commands:

```bash
cd /workspaces
colcon build --symlink-install
source install/setup.bash
```

Quick ROS 2 test:

```bash
ros2 run demo_nodes_cpp talker
# In another terminal
ros2 topic echo /chatter
```

Launch Gazebo demo:

```bash
gz sim -r shapes.sdf
```

---

## Th-RHex Holonomic Demo

The 3-leg holonomic demo runs in Gazebo through the Jacobian velocity
controller. Full step-by-step instructions are in [`DEMO.md`](DEMO.md).

Inside the container, build and source the workspace:

```bash
cd /workspaces
colcon build --symlink-install
source /workspaces/install/setup.bash
```

Start Gazebo:

```bash
ros2 launch th_rhex_description_3leg gazebo.launch.py
```

In a second container terminal, start the Jacobian controller:

```bash
cd /workspaces
source /workspaces/install/setup.bash
ros2 run th_rhex_control jacobian_controller
```

In a third container terminal, run the interactive demo:

```bash
cd /workspaces
./scripts/demo.sh
```

The demo includes forward motion, lateral holonomic motion, diagonal motion,
pure yaw, a full automatic sequence, tele-op velocity mode, and a reset helper.

---

## Typical Development Workflow

| Step | Description | Command |
|------|--------------|----------|
| 1 | Generate environment file (one-time) | `./scripts/gen-env.sh` |
| 2 | Enable GUI access | `xhost +local:root` |
| 3 | Start container | `docker compose up -d ros-jazzy` |
| 4 | Enter development shell | `docker compose exec ros-jazzy bash` |
| 5 | Build workspace | `colcon build --symlink-install` |
| 6 | Source setup | `source install/setup.bash` |
| 7 | Launch Gazebo or RViz | `gz sim -r shapes.sdf` or `rviz2` |
| 8 | Stop container | `docker compose down` |

For a quick disposable session, use:

```bash
docker compose run --rm ros-jazzy bash
```

---

## GUI Tools

| Tool | Purpose | Command |
|------|----------|----------|
| RViz 2 | Visualize TF frames, topics, and sensors | `rviz2` |
| Gazebo Harmonic | Physics simulation for URDF testing | `gz sim -r shapes.sdf` |
| rqt_graph | Visualize node and topic connections | `rqt_graph` |

---

## URDF and Gazebo

- **URDF (Unified Robot Description Format)** defines Th-RHex’s mechanical structure: links, joints, and geometry.  
- **Gazebo Harmonic** loads URDFs to simulate physics, sensors, and control systems.  
- This setup supports URDF and control development before transitioning to Isaac Sim for high-fidelity simulation and reinforcement learning.

### Importing the Th-RHex URDF

Robot description files live in the `th_rhex_description` ROS package:

```text
src/th_rhex_description/
├── urdf/      # URDF/Xacro files
├── meshes/    # STL/DAE/OBJ meshes referenced by the URDF
├── launch/    # RViz and Gazebo launch checks
├── rviz/      # RViz config
└── worlds/    # Gazebo worlds
```

Use `src/th_rhex_description/urdf/th_rhex.urdf.xacro` as the main robot model. Put mesh files under `src/th_rhex_description/meshes/` and reference them from URDF with package URLs, for example:

```xml
<mesh filename="package://th_rhex_description/meshes/base_link.STL"/>
```

The current imported SolidWorks export is archived at `src/th_rhex_description/original_exports/3leg_Th_Rhex/`. The maintained copy rewrites mesh paths to `package://th_rhex_description/...` so ROS 2 can resolve assets after install.

Validate and view the model from inside the container:

```bash
cd /workspaces
colcon build --symlink-install
source install/setup.bash
ros2 run th_rhex_description validate_urdf.sh
ros2 launch th_rhex_description display.launch.py
ros2 launch th_rhex_description gazebo.launch.py
```

---

## Dockerfile Summary (`docker/Dockerfile.jazzy`)

This Dockerfile builds the development image used by `docker-compose.yml`.

**Base Image:**  
- `osrf/ros:jazzy-desktop` (Ubuntu 24.04 LTS)

**Main Features:**  
- Installs Gazebo Harmonic  
- Adds developer tools (colcon, git, build-essential)  
- Sets up ROS 2 workspace at `/workspaces`  
- Automatically sources ROS 2 environment upon login  
- Supports GUI applications through X11 forwarding  
- Uses environment variables from `.env` for permissions and user identity

---

## GPU Acceleration (NVIDIA Users)

If your machine has an **NVIDIA GPU** and you want hardware acceleration for **Gazebo, RViz, or Isaac Sim**, you need to install the **NVIDIA Container Toolkit**.  
This toolkit allows Docker to use your GPU inside containers, enabling full OpenGL and CUDA support.

### Install the NVIDIA Container Toolkit (Ubuntu 22.04)

```bash
sudo apt-get update
sudo apt-get install -y curl gnupg
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey |   sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

distribution=$(. /etc/os-release; echo ${ID}${VERSION_ID})
curl -fsSL https://nvidia.github.io/libnvidia-container/${distribution}/libnvidia-container.list |   sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' |   sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### Verify GPU access works

Run this to confirm Docker can see your GPU:
```bash
docker run --rm --gpus all nvidia/cuda:12.4.0-base-ubuntu22.04 nvidia-smi
```

If it shows your GPU, you’re ready.

### Start the Th-RHex container with GPU access

```bash
xhost +local:root
docker compose up -d ros-jazzy
docker compose exec ros-jazzy bash
```

Inside the container, check GPU visibility:
```bash
nvidia-smi
glxinfo -B   # should show NVIDIA renderer
gz sim       # should run smoothly
```

> Once this is set up, all future runs (`docker compose up -d ros-jazzy`) will automatically use GPU acceleration.

---

## docker-compose.yml Summary

Defines how to run the `ros-jazzy` container.

**Key Features:**

- **Service:** `ros-jazzy`  
- **Build Context:** `docker/Dockerfile.jazzy`  
- **Volumes:**  
  - Mounts the entire repo to `/workspaces` (keeps code synced)  
  - Shares X11 socket for GUI display  
- **Environment:**  
  - Loads user-specific `.env` variables  
  - Enables GUI forwarding via `DISPLAY` and `XDG_RUNTIME_DIR`  
- **Network:** host or bridged depending on setup  
- **Restart Policy:** none (development use)  

You can modify this file to add additional services (e.g., database, visualization tools) or custom networks for multi-container ROS systems.

---

## Summary

- **Host:** Ubuntu 22.04 LTS  
- **Container:** Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic  
- **Purpose:** Develop and test Th-RHex’s URDF and control stack  
- **Future:** Transition to NVIDIA Isaac Sim for GPU-based reinforcement learning  
- **Workflow:** Clone → Generate `.env` → Enable GUI → Build → Develop → Simulate  

Use **persistent mode (`up / exec`)** for regular development and **ephemeral mode (`--rm`)** for clean, short sessions.  
All developers on Ubuntu 22.04 hosts will have a consistent, reproducible ROS 2 Jazzy environment ready for Th-RHex development.
