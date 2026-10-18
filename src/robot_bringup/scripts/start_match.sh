#!/usr/bin/env bash
# One-command launcher for a whole match run on the Raspberry Pi.
#
#   start_match.sh                  field: sensors + Nav2 (synthetic map) + game_manager
#   start_match.sh --desk           desk:  sensors + fake_base + Nav2 desk_mode + game_manager
#   start_match.sh --map ~/field_map.yaml
#                                   field: sensors + Nav2 on a Cartographer map
#   start_match.sh --odom-tf        field with the chassis: start bringup with
#                                   publish_odom_tf:=true (/odom -> TF)
#   start_match.sh --desk --supervisor
#                                   hands-free desk rehearsal: also injects
#                                   START / RACKx / END
#   start_match.sh --foxglove       also start foxglove_bridge on port 8765,
#                                   for Foxglove Studio on another machine
#   start_match.sh --rviz           also start RViz on the Pi's display
#                                   (works from SSH and from the desktop)
#
# Everything runs in its own session and logs to ~/dase7503/logs/<name>.log.
# Ctrl-C stops all of it.
#
# Sensors already running?  The script detects robot_bringup and skips it,
# so it is safe to run this after starting the bringup by hand.

set -euo pipefail

DESK=0
SENSORS=1
SUPERVISOR=0
ODOM_TF=0
FOXGLOVE=0
RVIS=0
MAP=""

usage() {
    sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'
}

while [ $# -gt 0 ]; do
    case "$1" in
        --desk) DESK=1 ;;
        --no-sensors) SENSORS=0 ;;
        --supervisor) SUPERVISOR=1 ;;
        --odom-tf) ODOM_TF=1 ;;
        --foxglove) FOXGLOVE=1 ;;
        --rviz) RVIS=1 ;;
        --map)
            MAP="${2:?--map needs a path}"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
    shift
done

if [ "$SUPERVISOR" -eq 1 ] && [ "$DESK" -eq 0 ]; then
    echo "--supervisor only makes sense with --desk" >&2
    exit 2
fi
if [ -n "$MAP" ] && [ "$DESK" -eq 1 ]; then
    echo "--map and --desk are mutually exclusive" >&2
    exit 2
fi
if [ -n "$MAP" ] && [ ! -f "$MAP" ]; then
    echo "map file not found: $MAP" >&2
    exit 2
fi
if [ "$SUPERVISOR" -eq 1 ] && [ ! -f "$HOME/dase7503/mission_supervisor.py" ]; then
    echo "missing $HOME/dase7503/mission_supervisor.py" >&2
    exit 2
fi

set +u
source /opt/ros/jazzy/setup.bash
source /home/rsi-bd-11/camera_ws/install3/setup.bash
source /home/rsi-bd-11/lidar_ros_ws/install/setup.bash
set -u

if [ "$FOXGLOVE" -eq 1 ] && [ ! -x /opt/ros/jazzy/lib/foxglove_bridge/foxglove_bridge ]; then
    echo "foxglove_bridge is not installed:" >&2
    echo "  sudo apt install ros-jazzy-foxglove-bridge" >&2
    exit 2
fi
if [ "$RVIS" -eq 1 ]; then
    if [ -z "${WAYLAND_DISPLAY:-}${DISPLAY:-}" ]; then
        # Started over SSH: attach to the desktop session's Xwayland server.
        # RViz renders through Ogre/GLX, so Wayland alone is not enough.
        auth_file=$(ls /run/user/"$(id -u)"/.mutter-Xwaylandauth.* 2>/dev/null | head -1)
        if [ -n "$auth_file" ] && [ -S /tmp/.X11-unix/X0 ]; then
            export DISPLAY=:0
            export XAUTHORITY="$auth_file"
            export XDG_RUNTIME_DIR=/run/user/"$(id -u)"
        else
            echo "--rviz needs a display, and no desktop session was found" >&2
            echo "run this from the Pi's desktop terminal instead" >&2
            exit 2
        fi
    fi
fi

LOG_DIR="$HOME/dase7503/logs"
mkdir -p "$LOG_DIR"

PIDS=()
NAMES=()

cleanup() {
    trap - INT TERM EXIT
    echo
    echo "stopping..."
    for pid in "${PIDS[@]}"; do
        kill -INT -- "-$pid" 2>/dev/null || true
    done
    sleep 5
    for pid in "${PIDS[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || true
    done
    sleep 2
    for pid in "${PIDS[@]}"; do
        kill -KILL -- "-$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    echo "stopped."
}
trap cleanup INT TERM EXIT

start_bg() {  # name command...
    local name="$1"
    shift
    setsid "$@" > "$LOG_DIR/$name.log" 2>&1 < /dev/null &
    local pid=$!
    PIDS+=("$pid")
    NAMES+=("$name")
    printf '  %-12s pid %-6s log %s\n' "$name" "$pid" "$LOG_DIR/$name.log"
}

already_running() {  # pattern description
    if pgrep -f "$1" > /dev/null 2>&1; then
        echo "$2 is already running; stop it first" >&2
        exit 1
    fi
}

echo "mode: $([ "$DESK" -eq 1 ] && echo desk || echo field)$([ -n "$MAP" ] && echo " (map $MAP)")"
already_running 'navigation.launc[h].py' "Nav2"
already_running 'game_manage[r].launch.py' "game_manager"

if [ "$SENSORS" -eq 1 ]; then
    if pgrep -f 'robot_bringup.launc[h].py' > /dev/null 2>&1; then
        echo "sensors already running; skipping robot_bringup"
        if [ "$ODOM_TF" -eq 1 ]; then
            echo "warning: the running bringup did not get publish_odom_tf:=true" >&2
        fi
    else
        if [ "$ODOM_TF" -eq 1 ]; then
            start_bg bringup ros2 launch robot_bringup robot_bringup.launch.py publish_odom_tf:=true
        else
            start_bg bringup ros2 launch robot_bringup robot_bringup.launch.py
        fi
    fi
fi

if [ "$DESK" -eq 1 ]; then
    start_bg fake_base ros2 launch robot_navigation fake_base.launch.py
fi

NAV_ARGS=(ros2 launch robot_navigation navigation.launch.py)
if [ "$DESK" -eq 1 ]; then
    NAV_ARGS+=(desk_mode:=true)
elif [ -n "$MAP" ]; then
    NAV_ARGS+=(map:="$MAP")
fi
if [ "$RVIS" -eq 1 ]; then
    NAV_ARGS+=(rviz:=true)
fi
start_bg nav2 "${NAV_ARGS[@]}"

start_bg game_manager ros2 launch game_manager game_manager.launch.py

if [ "$FOXGLOVE" -eq 1 ]; then
    start_bg foxglove /opt/ros/jazzy/lib/foxglove_bridge/foxglove_bridge
fi

if [ "$SUPERVISOR" -eq 1 ]; then
    echo "waiting for Nav2 to become active..."
    for _ in $(seq 1 20); do
        state=$(timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null | head -1 || true)
        case "$state" in
            active*) echo "Nav2 is active"; break ;;
        esac
        sleep 3
    done
    start_bg supervisor python3 "$HOME/dase7503/mission_supervisor.py" --timeout 170
fi

echo
echo "stack is up.  watch:"
echo "  ros2 topic echo /game_state"
echo "  ros2 topic echo /game_score"
echo "  ros2 topic echo /qr_detected"
if [ "$FOXGLOVE" -eq 1 ]; then
    echo "  Foxglove Studio: ws://$(hostname -I | awk '{print $1}'):8765"
fi
echo "press Ctrl-C to stop everything."
# Exit when any component exits: in a --supervisor run that is the end of
# the rehearsal, otherwise it means a node died and the stack is unusable.
wait -n || true
