#!/usr/bin/bash

# 1. Setup Logging Directories
sudo mkdir -p /var/log/ros2
sudo chmod 775 /var/log/ros2
sudo chown capra:capra /var/log/ros2

# 2. Define Paths
USER_SYSTEMD_DIR="$HOME/.config/systemd/user"
mkdir -p "$USER_SYSTEMD_DIR"
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"

# 3. Clean up old System-level services (if they exist) to avoid confusion
sudo systemctl disable --now roboguard_launch.service 2>/dev/null
sudo systemctl disable --now roboguard_joy.service 2>/dev/null
sudo rm -f /etc/systemd/system/roboguard_launch.service
sudo rm -f /etc/systemd/system/roboguard_joy.service

# 4. Install and Enable User-level services
echo "Installing services to $USER_SYSTEMD_DIR..."

cp "$SCRIPT_DIR/roboguard_launch.service" "$USER_SYSTEMD_DIR/"
cp "$SCRIPT_DIR/roboguard_joy.service" "$USER_SYSTEMD_DIR/"

# Reload the user daemon
systemctl --user daemon-reload

# Enable and Start the services
systemctl --user enable roboguard_launch.service
systemctl --user restart roboguard_launch.service

# Joy is an optional service
systemctl --user stop roboguard_joy.service

echo "Done! Check status with: systemctl --user status roboguard_launch.service"
echo "For local joy control, use: systemctl --user start roboguard_joy.service"