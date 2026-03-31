#!/usr/bin/env bash

is_distrobox() {
    # Method 1: Official env variable set by Distrobox
    if [[ -n "$DISTROBOX_ENTER_PATH" ]] || [[ "$DISTROBOX" == "1" ]]; then
        return 0
    fi

    # Method 2: Container marker file
    if [[ -f "/run/.containerenv" ]]; then
        # Check if it's specifically a distrobox container
        if grep -qi "distrobox" /run/.containerenv 2>/dev/null; then
            return 0
        fi
    fi

    # Method 3: Fallback - check parent process
    if ps -o comm= -p 1 | grep -qiE "bash|sh"; then
        if grep -qi "distrobox" /proc/1/environ 2>/dev/null; then
            return 0
        fi
    fi

    return 1
}

if is_distrobox; then
    echo "Cannot run this in a distrobox, run this script from outside the distrobox"
    exit 1
fi

if [[ "$EUID" -eq 0 ]]; then
    echo "Cannot run this as root, run this script as a user"
    exit 1
fi

# 1. Setup Logging Directories
sudo mkdir -p /var/log/ros2
sudo chmod 775 /var/log/ros2
sudo chown capra:capra /var/log/ros2

# 2. Define Paths
USER_SYSTEMD_DIR="$HOME/.config/systemd/user"
mkdir -p "$USER_SYSTEMD_DIR"
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"

# 3. Install and Enable User-level services
echo "Installing services to $USER_SYSTEMD_DIR..."

cp "$SCRIPT_DIR/roboguard_launch.service" "$USER_SYSTEMD_DIR/"

# Reload the user daemon
systemctl --user daemon-reload

# Enable and Start the services
systemctl --user enable roboguard_launch.service
systemctl --user restart roboguard_launch.service

echo "Done! Check status with: systemctl --user status roboguard_launch.service"