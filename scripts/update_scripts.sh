#!/usr/bin/bash

script_dir="$(dirname "$(readlink -f "$0")")"

systemctl stop --user roboguard_launch.service
sudo cp $script_dir/roboguard_launch.service /etc/systemd/user/roboguard_launch.service
systemctl start --user roboguard_launch.service

