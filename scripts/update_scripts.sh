#!/usr/bin/bash

sudo mkdir -p /var/log/ros2
sudo chmod 775 /var/log/ros2
sudo chown capra:capra /var/log/ros2

script_dir="$(dirname "$(readlink -f "$0")")"

systemctl disable --user roboguard_launch.service
sudo systemctl disable roboguard_launch.service

systemctl stop --user roboguard_launch.service
systemctl stop --user roboguard_joy.service
#sudo cp $script_dir/roboguard_launch.service /etc/systemd/user/roboguard_launch.service
#sudo cp $script_dir/roboguard_joy.service /etc/systemd/user/roboguard_joy.service
#systemctl enable --now --user roboguard_launch.service

sudo systemctl stop roboguard_launch.service
sudo systemctl stop roboguard_joy.service
sudo cp $script_dir/roboguard_launch.service /etc/systemd/system/roboguard_launch.service
sudo cp $script_dir/roboguard_joy.service /etc/systemd/system/roboguard_joy.service
sudo systemctl daemon-reload
sudo systemctl enable --now roboguard_launch.service

