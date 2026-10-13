#!/usr/bin/env bash
set -euo pipefail

children=()
ros_pid=

cleanup() {
    trap - EXIT INT TERM
    # Keep X alive while Gazebo and RViz shut down.
    if [[ -n "$ros_pid" ]] && kill -0 "$ros_pid" 2>/dev/null; then
        kill -INT -- "-$ros_pid" 2>/dev/null || true
        for _ in {1..100}; do
            kill -0 "$ros_pid" 2>/dev/null || break
            sleep 0.1
        done
    fi
    for pid in "${children[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || true
    done
    wait || true
}
trap cleanup EXIT
trap 'exit 0' INT TERM

start() {
    setsid "$@" &
    children+=("$!")
}

start Xvfb "$DISPLAY" -screen 0 1600x900x24 -nolisten tcp -ac
for attempt in {1..100}; do
    if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
        break
    fi
    if [[ "$attempt" == 100 ]]; then
        echo 'Virtual display did not become ready.' >&2
        exit 1
    fi
    sleep 0.1
done

start openbox
start x11vnc -display "$DISPLAY" -localhost -rfbport 5900 -forever -shared -nopw
start websockify --web=/usr/share/novnc/ 6080 localhost:5900
start ros2 launch care_robot_bringup bringup.launch.py "$@"
ros_pid=${children[-1]}

echo 'Desktop: http://localhost:6080/vnc.html?autoconnect=true&resize=scale'
# A failed desktop component or an exited launch shuts down the whole container.
set +e
wait -n "${children[@]}"
status=$?
exit "$status"
