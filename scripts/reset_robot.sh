#!/bin/bash
# ─────────────────────────────────────────────────────────────
# Th-RHex Robot Reset Script
# Stops all motion, deletes the robot from Gazebo, and respawns
# it at the origin. Falls back to clear manual instructions if
# the gz service call is unavailable.
# Usage: ./scripts/reset_robot.sh
# ─────────────────────────────────────────────────────────────

source /workspaces/install/setup.bash 2>/dev/null || {
    echo "ERROR: Could not source ROS 2 environment."
    echo "Run this script from inside the Docker container."
    exit 1
}

# ── Constants ─────────────────────────────────────────────────
ROBOT_NAME="th_rhex"
WORLD_NAME="minimal"
SPAWN_Z="0.25"
TOPIC="/cmd_vel"
MSG="geometry_msgs/msg/Twist"
ZERO="{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Th-RHex Reset"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# ── Step 1: Stop all motion ───────────────────────────────────
echo "  [1/3] Sending zero velocity..."
ros2 topic pub --once "$TOPIC" "$MSG" "$ZERO" > /dev/null 2>&1 \
    && echo "        Done." \
    || echo "        Warning: could not publish zero cmd_vel (topic may be unavailable)."

# ── Step 2: Delete entity from Gazebo ────────────────────────
echo "  [2/3] Attempting to delete robot from Gazebo..."

DELETED=false

if command -v gz &>/dev/null; then
    gz service \
        -s "/world/${WORLD_NAME}/remove" \
        --reqtype gz.msgs.Entity \
        --reptype gz.msgs.Boolean \
        --timeout 2000 \
        --req "name: \"${ROBOT_NAME}\" type: MODEL" \
        > /dev/null 2>&1 \
    && DELETED=true && echo "        Deleted via gz service."
fi

if [ "$DELETED" = false ]; then
    echo ""
    echo "  ┌─────────────────────────────────────────────────────┐"
    echo "  │  Could not auto-delete the robot from Gazebo.       │"
    echo "  │                                                     │"
    echo "  │  Manual reset steps in the Gazebo GUI:             │"
    echo "  │    1. Right-click the robot body → Remove Entity   │"
    echo "  │    2. Re-run: ros2 launch th_rhex_description_3leg │"
    echo "  │               gazebo.launch.py                     │"
    echo "  │                                                     │"
    echo "  │  Or press Ctrl+C here and restart the launch file. │"
    echo "  └─────────────────────────────────────────────────────┘"
    echo ""
    exit 1
fi

# ── Step 3: Respawn at origin ─────────────────────────────────
echo "  [3/3] Respawning at origin (z=${SPAWN_Z} m)..."
sleep 1

ros2 run ros_gz_sim create \
    -topic robot_description \
    -name "$ROBOT_NAME" \
    -z "$SPAWN_Z" 2>/dev/null \
&& echo "        Respawned." \
|| {
    echo ""
    echo "  Respawn failed. Re-run the Gazebo launch file:"
    echo "    ros2 launch th_rhex_description_3leg gazebo.launch.py"
    exit 1
}

# Give robot_state_publisher time to catch up after re-spawn
sleep 2

echo ""
echo "  Robot reset complete. Origin: (0, 0, ${SPAWN_Z} m)"
echo ""
