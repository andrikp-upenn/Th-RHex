#!/usr/bin/env bash
# Th-RHex robot reset helper.
#
# Important: the primary reset path is non-destructive. Deleting and respawning
# the model can detach Gazebo ros2_control from the entity, leaving the robot
# visible but unresponsive to later velocity commands.

set -uo pipefail

echo ""
echo "------------------------------------------------------------"
echo "Th-RHex Reset"
echo "------------------------------------------------------------"
echo ""
echo "Checking ROS environment..."

if ! command -v ros2 >/dev/null 2>&1; then
    if [ -f /workspaces/install/setup.bash ]; then
        # shellcheck source=/dev/null
        source /workspaces/install/setup.bash 2>/dev/null || true
    fi
fi

if ! command -v ros2 >/dev/null 2>&1; then
    if [ -f /workspaces/install/setup.bash ]; then
        # shellcheck source=/dev/null
        source /workspaces/install/setup.bash 2>/dev/null || true
    fi
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: ros2 is not available."
    echo "Run this from inside the Docker container after:"
    echo "  source /workspaces/install/setup.bash"
    exit 1
fi

echo "      ROS environment ready."

ROBOT_NAME="${ROBOT_NAME:-th_rhex}"
WORLD_NAME="${WORLD_NAME:-minimal}"
SPAWN_Z="${SPAWN_Z:-0.25}"
TOPIC="/cmd_vel"
MSG="geometry_msgs/msg/Twist"
ZERO="{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

if [ -t 1 ] && command -v tput >/dev/null 2>&1 && [ "$(tput colors 2>/dev/null || echo 0)" -ge 8 ]; then
    C_GREEN="$(tput setaf 2)$(tput bold)"
    C_YELLOW="$(tput setaf 3)"
    C_RED="$(tput setaf 1)$(tput bold)"
    C_RESET="$(tput sgr0)"
else
    C_GREEN=""
    C_YELLOW=""
    C_RED=""
    C_RESET=""
fi

stop_robot() {
    echo "[1/3] Sending zero velocity..."
    timeout 4s ros2 topic pub --once "$TOPIC" "$MSG" "$ZERO" >/dev/null 2>&1 \
        && echo "      ${C_GREEN}cmd_vel zeroed.${C_RESET}" \
        || echo "      ${C_YELLOW}Warning: could not publish zero cmd_vel.${C_RESET}"
}

set_pose_reset() {
    command -v gz >/dev/null 2>&1 || return 1

    timeout 5s gz service \
        -s "/world/${WORLD_NAME}/set_pose" \
        --reqtype gz.msgs.Pose \
        --reptype gz.msgs.Boolean \
        --timeout 3000 \
        --req "name: \"${ROBOT_NAME}\" position {x: 0.0 y: 0.0 z: ${SPAWN_Z}} orientation {w: 1.0}" \
        >/dev/null 2>&1
}

world_control_reset() {
    command -v gz >/dev/null 2>&1 || return 1

    timeout 5s gz service \
        -s "/world/${WORLD_NAME}/control" \
        --reqtype gz.msgs.WorldControl \
        --reptype gz.msgs.Boolean \
        --timeout 3000 \
        --req "reset {model_only: true}" \
        >/dev/null 2>&1
}

delete_respawn_warning() {
    echo ""
    echo "${C_RED}Automatic non-destructive reset failed.${C_RESET}"
    echo ""
    echo "Do not use delete/respawn during a live demo unless you also restart:"
    echo "  1. Gazebo launch"
    echo "  2. ros2 run th_rhex_control jacobian_controller"
    echo "  3. ./scripts/demo.sh"
    echo ""
    echo "Reason: deleting the model can leave ros2_control connected to the old"
    echo "Gazebo entity, so the respawned robot may ignore later commands."
    echo ""
    echo "Manual GUI fallback:"
    echo "  - Use Gazebo reset-world/reset-model controls if available."
    echo "  - If you remove the entity, restart the full launch stack afterward."
    echo ""
}

check_after_reset() {
    echo "[3/3] Checking command topics..."
    sleep 1

    local missing=0
    for topic in "$TOPIC" "/joint_states" "/leg_velocity_controller/commands"; do
        if timeout 4s ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
            echo "      ${C_GREEN}OK${C_RESET} ${topic}"
        else
            echo "      ${C_RED}MISSING${C_RESET} ${topic}"
            missing=1
        fi
    done

    if [ "$missing" -ne 0 ]; then
        echo ""
        echo "${C_YELLOW}One or more topics are missing. Restart Gazebo and the Jacobian node.${C_RESET}"
        return 1
    fi
}

stop_robot

echo "[2/3] Resetting robot without deleting the model..."
if set_pose_reset; then
    echo "      ${C_GREEN}Pose reset through Gazebo set_pose.${C_RESET}"
elif world_control_reset; then
    echo "      ${C_GREEN}Model reset through Gazebo world control.${C_RESET}"
else
    delete_respawn_warning
    exit 1
fi

stop_robot >/dev/null 2>&1 || true
check_after_reset || exit 1

echo ""
echo "${C_GREEN}Robot reset complete. Controllers should still be attached.${C_RESET}"
echo ""
