#!/usr/bin/env bash
# ci_launch_test.sh
#
# Runtime smoke test for a ROS2 launch file.
#
# Starts `ros2 launch <args>`, lets it run for STABLE_SECONDS. If the process
# exits before the timer elapses, the launch crashed -> we exit non-zero with
# the launch's exit code. If it survives the timer, we send SIGINT for a clean
# shutdown and succeed.
#
# Usage:
#   ci_launch_test.sh <label> <package> <launch_file> [key:=value ...]
#
# Example:
#   ci_launch_test.sh real-no-ovis roboguard_bringup real.launch.py \
#     use_mock_odrives:=true use_mock_ovis:=true with_ovis:=false with_rosbag:=false

set -u

LABEL="${1:-launch-test}"
PACKAGE="${2:?package name required}"
LAUNCH_FILE="${3:?launch file required}"
shift 3
LAUNCH_ARGS=("$@")

STABLE_SECONDS="${STABLE_SECONDS:-15}"
SHUTDOWN_GRACE_SECONDS="${SHUTDOWN_GRACE_SECONDS:-10}"

LOG_FILE="$(mktemp -t "ci-launch-${LABEL}-XXXXXX.log")"

echo "=============================================================="
echo "[${LABEL}] ros2 launch ${PACKAGE} ${LAUNCH_FILE} ${LAUNCH_ARGS[*]}"
echo "[${LABEL}] stable window: ${STABLE_SECONDS}s"
echo "[${LABEL}] log: ${LOG_FILE}"
echo "=============================================================="

# Start the launch in its own process group so we can signal the whole tree.
setsid ros2 launch "${PACKAGE}" "${LAUNCH_FILE}" "${LAUNCH_ARGS[@]}" \
    >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!
LAUNCH_PGID=$(ps -o pgid= "${LAUNCH_PID}" | tr -d ' ')

# Tail the log in the background so CI output is live.
tail -F "${LOG_FILE}" --pid=$$ &
TAIL_PID=$!

cleanup_tail() {
    kill "${TAIL_PID}" 2>/dev/null || true
    wait "${TAIL_PID}" 2>/dev/null || true
}

# Watch for early exit during the stable window.
crashed=0
for _ in $(seq 1 "${STABLE_SECONDS}"); do
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        crashed=1
        break
    fi
    sleep 1
done

if [[ "${crashed}" -eq 1 ]]; then
    wait "${LAUNCH_PID}"
    rc=$?
    cleanup_tail
    echo "=============================================================="
    echo "[${LABEL}] FAIL: launch exited early with code ${rc}"
    echo "=============================================================="
    exit "${rc}"
fi

echo "=============================================================="
echo "[${LABEL}] launch stable after ${STABLE_SECONDS}s, sending SIGINT"
echo "=============================================================="

# SIGINT the whole process group for a clean ros2 launch shutdown.
kill -INT -"${LAUNCH_PGID}" 2>/dev/null || true

# Wait up to SHUTDOWN_GRACE_SECONDS for clean shutdown.
for _ in $(seq 1 "${SHUTDOWN_GRACE_SECONDS}"); do
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        break
    fi
    sleep 1
done

# If still alive, escalate.
if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "[${LABEL}] launch did not exit after SIGINT, sending SIGTERM"
    kill -TERM -"${LAUNCH_PGID}" 2>/dev/null || true
    sleep 3
fi
if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "[${LABEL}] launch did not exit after SIGTERM, sending SIGKILL"
    kill -KILL -"${LAUNCH_PGID}" 2>/dev/null || true
fi

wait "${LAUNCH_PID}" 2>/dev/null
rc=$?
cleanup_tail

# After SIGINT, ros2 launch returns 0 on a graceful stop. Anything else is
# treated as a crash *unless* it's the signal exit codes from our escalation
# (130 = SIGINT, 143 = SIGTERM).
if [[ "${rc}" -eq 0 || "${rc}" -eq 130 || "${rc}" -eq 143 ]]; then
    echo "=============================================================="
    echo "[${LABEL}] PASS (exit code ${rc})"
    echo "=============================================================="
    exit 0
fi

echo "=============================================================="
echo "[${LABEL}] FAIL: launch exited with code ${rc} during shutdown"
echo "=============================================================="
exit "${rc}"