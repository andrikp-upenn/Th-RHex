#!/bin/bash
# ─────────────────────────────────────────────────────────────
# Th-RHex Holonomic Robot — Interactive Demo Script
# Showcases holonomic motion capabilities via the Jacobian
# velocity controller.
#
# Prerequisites (must be running before this script):
#   ros2 launch th_rhex_description_3leg gazebo.launch.py
#   ros2 run th_rhex_control jacobian_controller
#
# Usage: ./scripts/demo.sh
# ─────────────────────────────────────────────────────────────

source /workspaces/install/setup.bash 2>/dev/null || {
    echo "ERROR: Could not source ROS 2 environment."
    echo "Run this script from inside the Docker container."
    exit 1
}

# ── Terminal colors ───────────────────────────────────────────
if [ -t 1 ] && command -v tput &>/dev/null && tput colors &>/dev/null; then
    C_GREEN=$(tput setaf 2; tput bold)
    C_YELLOW=$(tput setaf 3)
    C_RED=$(tput setaf 1; tput bold)
    C_BLUE=$(tput setaf 4; tput bold)
    C_CYAN=$(tput setaf 6)
    C_BOLD=$(tput bold)
    C_RESET=$(tput sgr0)
else
    C_GREEN="" C_YELLOW="" C_RED="" C_BLUE="" C_CYAN="" C_BOLD="" C_RESET=""
fi

# ── Constants ─────────────────────────────────────────────────
TOPIC="/cmd_vel"
MSG="geometry_msgs/msg/Twist"
JVEL_TOPIC="/leg_velocity_controller/commands"
RATE=10
DURATION=3
ZERO="{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

JVEL_FILE="/tmp/th_rhex_jvel_$$.txt"
JVEL_PID=""
PUB_PID=""

# ── Cleanup / safety trap ─────────────────────────────────────
cleanup() {
    printf "\r%-80s\n" ""   # clear any live countdown line
    echo "${C_RED}  Stopping all motion...${C_RESET}"
    ros2 topic pub --once "$TOPIC" "$MSG" "$ZERO" > /dev/null 2>&1
    [ -n "$JVEL_PID" ] && kill "$JVEL_PID" 2>/dev/null
    [ -n "$PUB_PID"  ] && kill "$PUB_PID"  2>/dev/null
    rm -f "$JVEL_FILE"
    echo "${C_RESET}"
}
trap cleanup EXIT INT TERM

# ── Sanity check ─────────────────────────────────────────────
check_ros() {
    printf "  Checking ROS 2 topics..."
    if ! ros2 topic list 2>/dev/null | grep -q "/cmd_vel"; then
        echo ""
        echo ""
        echo "${C_RED}  ERROR: /cmd_vel not found. Is the simulation running?${C_RESET}"
        echo ""
        echo "  Start the simulation first:"
        echo "  ${C_YELLOW}  ros2 launch th_rhex_description_3leg gazebo.launch.py${C_RESET}"
        echo "  ${C_YELLOW}  ros2 run th_rhex_control jacobian_controller${C_RESET}"
        echo ""
        exit 1
    fi
    echo "  ${C_GREEN}✓${C_RESET}"
}

# ── Joint velocity watcher ────────────────────────────────────
# Subscribes to /leg_velocity_controller/commands in background.
# Parses Float64MultiArray and writes a plain-text summary to
# JVEL_FILE so the countdown loop can display it inline.
start_jvel_watcher() {
    echo "waiting for data..." > "$JVEL_FILE"
    ros2 topic echo "$JVEL_TOPIC" 2>/dev/null | \
        python3 -u -c "
import sys
buf = []
jvel_file = '$JVEL_FILE'
for line in sys.stdin:
    line = line.strip()
    if line.startswith('- '):
        try:
            buf.append(float(line[2:]))
        except ValueError:
            pass
        if len(buf) == 3:
            text = 'joints:  fr:{:+6.3f}  ml:{:+6.3f}  rr:{:+6.3f}  rad/s'.format(*buf)
            try:
                with open(jvel_file, 'w') as f:
                    f.write(text)
            except Exception:
                pass
            buf = []
" &
    JVEL_PID=$!
}

stop_jvel_watcher() {
    [ -n "$JVEL_PID" ] && kill "$JVEL_PID" 2>/dev/null
    JVEL_PID=""
}

send_twist_once() {
    local lx=$1 ly=$2 lz=$3 ax=$4 ay=$5 az=$6
    ros2 topic pub --once "$TOPIC" "$MSG" \
        "{linear: {x: $lx, y: $ly, z: $lz}, angular: {x: $ax, y: $ay, z: $az}}" \
        > /dev/null 2>&1
}

prompt_number() {
    local prompt="$1"
    local default="$2"
    local value

    printf "  %s [%s]: " "$prompt" "$default" >&2
    read -r value
    if [ -z "$value" ]; then
        value="$default"
    fi

    case "$value" in
        ''|*[!0-9.-]*)
            echo "  ${C_YELLOW}Invalid value; using ${default}.${C_RESET}" >&2
            value="$default"
            ;;
    esac

    printf "%s" "$value"
}

teleop_mode() {
    local linear_speed angular_speed
    local lx="0.0" ly="0.0" az="0.0"
    local key rest old_stty

    echo ""
    echo "${C_BLUE}  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${C_RESET}"
    echo "  ${C_GREEN}TELE-OP VELOCITY MODE${C_RESET}"
    echo "  WASD commands body-frame translation; arrow left/right command yaw."
    echo "  Space or x stops. q returns to the main menu."
    echo "${C_BLUE}  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${C_RESET}"
    echo ""

    linear_speed=$(prompt_number "Linear speed in m/s" "0.10")
    echo ""
    angular_speed=$(prompt_number "Yaw rate in rad/s" "0.30")
    echo ""
    echo ""
    echo "  Controls:"
    echo "    w/s       forward/backward"
    echo "    a/d       left/right lateral motion"
    echo "    ←/→       yaw left/right"
    echo "    space/x   stop"
    echo "    q         quit tele-op"
    echo ""

    start_jvel_watcher
    old_stty=$(stty -g 2>/dev/null || true)
    stty -echo -icanon time 0 min 0 2>/dev/null || true
    trap '[ -n "$old_stty" ] && stty "$old_stty" 2>/dev/null; cleanup; exit 130' INT TERM

    while true; do
        key=""
        rest=""
        IFS= read -rsn1 key

        case "$key" in
            w|W)
                lx="$linear_speed"; ly="0.0"; az="0.0"
                ;;
            s|S)
                lx="-$linear_speed"; ly="0.0"; az="0.0"
                ;;
            a|A)
                lx="0.0"; ly="$linear_speed"; az="0.0"
                ;;
            d|D)
                lx="0.0"; ly="-$linear_speed"; az="0.0"
                ;;
            x|X|" ")
                lx="0.0"; ly="0.0"; az="0.0"
                ;;
            q|Q)
                send_twist_once 0.0 0.0 0.0 0.0 0.0 0.0
                break
                ;;
            $'\e')
                IFS= read -rsn2 rest
                case "$rest" in
                    "[D")
                        lx="0.0"; ly="0.0"; az="$angular_speed"
                        ;;
                    "[C")
                        lx="0.0"; ly="0.0"; az="-$angular_speed"
                        ;;
                    *)
                        continue
                        ;;
                esac
                ;;
            *)
                sleep 0.03
                continue
                ;;
        esac

        send_twist_once "$lx" "$ly" 0.0 0.0 0.0 "$az"
        local jvel
        jvel=$(cat "$JVEL_FILE" 2>/dev/null || echo "waiting for data...")
        printf "\r  ${C_YELLOW}cmd vx=%6s  vy=%6s  wz=%6s${C_RESET}  ${C_CYAN}%-42s${C_RESET}" \
            "$lx" "$ly" "$az" "$jvel"
    done

    [ -n "$old_stty" ] && stty "$old_stty" 2>/dev/null || true
    trap cleanup EXIT INT TERM
    stop_jvel_watcher
    send_twist_once 0.0 0.0 0.0 0.0 0.0 0.0
    printf "\r%-120s\r\n" ""
    echo "  ${C_RED}Tele-op stopped; cmd_vel zeroed.${C_RESET}"
}

# ── Core motion runner ────────────────────────────────────────
# Arguments:
#   $1  title       Display name for this motion
#   $2  callout     One-line description of why it is noteworthy
#   $3  lx          linear.x (m/s)
#   $4  ly          linear.y (m/s)
#   $5  lz          linear.z (m/s)
#   $6  ax          angular.x (rad/s)
#   $7  ay          angular.y (rad/s)
#   $8  az          angular.z (rad/s)
#   $9  duration    seconds (optional, default $DURATION)
run_motion() {
    local title="$1"
    local callout="$2"
    local lx=$3 ly=$4 lz=$5 ax=$6 ay=$7 az=$8
    local dur=${9:-$DURATION}

    echo ""
    echo "${C_BLUE}  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${C_RESET}"
    printf "  ${C_GREEN}%-48s${C_RESET}\n" "$title"
    [ -n "$callout" ] && printf "  ${C_BLUE}→ %s${C_RESET}\n" "$callout"
    printf "  ${C_YELLOW}vx=%-5s  vy=%-5s  vz=%-5s  wz=%-5s  (%ds)${C_RESET}\n" \
        "$lx" "$ly" "$lz" "$az" "$dur"
    echo "${C_BLUE}  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${C_RESET}"
    echo ""

    # Start background joint-velocity display
    start_jvel_watcher

    # Start motion publisher in background
    ros2 topic pub --rate $RATE "$TOPIC" "$MSG" \
        "{linear: {x: $lx, y: $ly, z: $lz}, angular: {x: $ax, y: $ay, z: $az}}" \
        > /dev/null 2>&1 &
    PUB_PID=$!

    # Live countdown line: joint velocities + remaining seconds
    local i
    for ((i=dur; i>0; i--)); do
        local jvel
        jvel=$(cat "$JVEL_FILE" 2>/dev/null || echo "waiting for data...")
        printf "\r  ${C_YELLOW}%-55s${C_RESET}  ${C_CYAN}[%d s remaining]${C_RESET}    " "$jvel" "$i"
        sleep 1
    done
    printf "\r%-80s\r\n" ""   # clear countdown line

    # Stop
    stop_jvel_watcher
    kill "$PUB_PID" 2>/dev/null
    wait "$PUB_PID" 2>/dev/null
    PUB_PID=""

    ros2 topic pub --once "$TOPIC" "$MSG" "$ZERO" > /dev/null 2>&1
    echo "  ${C_RED}◼ Stopped${C_RESET}"
    sleep 0.5
}

# ── UI helpers ────────────────────────────────────────────────
print_header() {
    clear
    echo ""
    echo "${C_BLUE}  ╔══════════════════════════════════════════════════════╗${C_RESET}"
    echo "${C_BLUE}  ║                                                      ║${C_RESET}"
    echo "${C_BLUE}  ║${C_RESET}   ${C_GREEN}████████╗██╗  ██╗    ██████╗ ██╗  ██╗███████╗██╗  ██╗${C_RESET}  ${C_BLUE}║${C_RESET}"
    echo "${C_BLUE}  ║${C_RESET}   ${C_GREEN}╚══██╔══╝██║  ██║   ██╔══██╗██║  ██║██╔════╝╚██╗██╔╝${C_RESET}  ${C_BLUE}║${C_RESET}"
    echo "${C_BLUE}  ║${C_RESET}   ${C_GREEN}   ██║   ███████║   ██████╔╝███████║█████╗   ╚███╔╝ ${C_RESET}  ${C_BLUE}║${C_RESET}"
    echo "${C_BLUE}  ║${C_RESET}   ${C_GREEN}   ██║   ██╔══██║   ██╔══██╗██╔══██║██╔══╝   ██╔██╗ ${C_RESET}  ${C_BLUE}║${C_RESET}"
    echo "${C_BLUE}  ║${C_RESET}   ${C_GREEN}   ██║   ██║  ██║   ██║  ██║██║  ██║███████╗██╔╝ ██╗${C_RESET}  ${C_BLUE}║${C_RESET}"
    echo "${C_BLUE}  ║${C_RESET}   ${C_GREEN}   ╚═╝   ╚═╝  ╚═╝   ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝╚═╝  ╚═╝${C_RESET}  ${C_BLUE}║${C_RESET}"
    echo "${C_BLUE}  ║                                                      ║${C_RESET}"
    echo "${C_BLUE}  ║${C_RESET}        ${C_BOLD}Holonomic Robot — Interactive Demo${C_RESET}           ${C_BLUE}║${C_RESET}"
    echo "${C_BLUE}  ║${C_RESET}        ${C_CYAN}University of Pennsylvania GRASP Lab${C_RESET}          ${C_BLUE}║${C_RESET}"
    echo "${C_BLUE}  ║                                                      ║${C_RESET}"
    echo "${C_BLUE}  ╚══════════════════════════════════════════════════════╝${C_RESET}"
    echo ""
}

print_menu() {
    echo "  ${C_BOLD}Select a demonstration:${C_RESET}"
    echo ""
    echo "    ${C_GREEN}1${C_RESET}  Forward motion               ${C_YELLOW}vx = +0.10 m/s${C_RESET}"
    echo "    ${C_BLUE}${C_BOLD}2  Lateral motion  ← HOLONOMIC${C_RESET}  ${C_YELLOW}vy = +0.10 m/s${C_RESET}"
    echo "    ${C_GREEN}3${C_RESET}  Diagonal motion              ${C_YELLOW}vx = +0.10  vy = +0.10 m/s${C_RESET}"
    echo "    ${C_GREEN}4${C_RESET}  Yaw in place                 ${C_YELLOW}wz = +0.30 rad/s${C_RESET}"
    echo "    ${C_GREEN}5${C_RESET}  ${C_BOLD}Full auto sequence${C_RESET}           all four motions, 1 s pause between"
    echo "    ${C_GREEN}t${C_RESET}  Tele-op velocity mode        WASD translation, arrows yaw"
    echo "    ${C_GREEN}r${C_RESET}  Reset robot position"
    echo "    ${C_GREEN}q${C_RESET}  Quit"
    echo ""
}

# ── Main ─────────────────────────────────────────────────────
print_header
check_ros
echo ""
print_menu

while true; do
    printf "  ${C_BOLD}Choice:${C_RESET} "
    read -r choice
    case "$choice" in
        1)
            run_motion \
                "FORWARD MOTION" \
                "Standard forward locomotion via Jacobian velocity controller." \
                0.1 0.0 0.0  0.0 0.0 0.0
            ;;
        2)
            run_motion \
                "LATERAL MOTION  [ HOLONOMIC DEMO ]" \
                "Pure sideways translation — impossible for differential-drive or Ackermann robots." \
                0.0 0.1 0.0  0.0 0.0 0.0
            ;;
        3)
            run_motion \
                "DIAGONAL MOTION" \
                "Simultaneous forward + lateral translation with no heading change required." \
                0.1 0.1 0.0  0.0 0.0 0.0
            ;;
        4)
            run_motion \
                "YAW IN PLACE" \
                "Full 360° rotation about the vertical axis — zero translational velocity." \
                0.0 0.0 0.0  0.0 0.0 0.3
            ;;
        5)
            echo ""
            echo "  ${C_BOLD}Starting full auto sequence (4 motions × ${DURATION}s each)...${C_RESET}"
            run_motion \
                "1 / 4  —  FORWARD" \
                "Standard forward locomotion." \
                0.1 0.0 0.0  0.0 0.0 0.0
            sleep 1

            run_motion \
                "2 / 4  —  LATERAL  [ HOLONOMIC DEMO ]" \
                "Pure sideways translation — impossible without holonomic drive." \
                0.0 0.1 0.0  0.0 0.0 0.0
            sleep 1

            run_motion \
                "3 / 4  —  DIAGONAL" \
                "Simultaneous forward + lateral with no heading change." \
                0.1 0.1 0.0  0.0 0.0 0.0
            sleep 1

            run_motion \
                "4 / 4  —  YAW IN PLACE" \
                "Body rotation with zero translational velocity." \
                0.0 0.0 0.0  0.0 0.0 0.3

            echo ""
            echo "  ${C_GREEN}${C_BOLD}✓ Full sequence complete.${C_RESET}"
            ;;
        t|T)
            teleop_mode
            ;;
        r|R)
            echo ""
            SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
            if [ -x "$SCRIPT_DIR/reset_robot.sh" ]; then
                echo "  ${C_RED}Reset requested. Running reset helper...${C_RESET}"
                if timeout 20s bash "$SCRIPT_DIR/reset_robot.sh"; then
                    echo "  ${C_GREEN}Reset helper finished.${C_RESET}"
                else
                    rc=$?
                    echo ""
                    echo "  ${C_RED}Reset helper did not complete cleanly (exit ${rc}).${C_RESET}"
                    echo "  Restart Gazebo and the Jacobian controller before continuing the demo."
                fi
            else
                echo "  ${C_RED}Stopping all motion...${C_RESET}"
                ros2 topic pub --once "$TOPIC" "$MSG" "$ZERO" > /dev/null 2>&1
                echo "  ${C_GREEN}✓ Velocity zeroed.${C_RESET}"
                echo "  (reset_robot.sh not found — position not reset in Gazebo)"
            fi
            ;;
        q|Q)
            echo ""
            echo "  Goodbye."
            exit 0
            ;;
        "")
            ;;
        *)
            echo "  ${C_RED}Invalid choice. Enter 1–5, t, r, or q.${C_RESET}"
            ;;
    esac
    echo ""
    print_menu
done
