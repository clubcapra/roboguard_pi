#!/usr/bin/bash
# run_test
#
# Reproduces locally what the CI pipeline does:
#   1. Ensure submodules are synced/initialized on the host
#   2. Build the dev container image
#   3. colcon build + colcon test
#   4. Runtime smoke tests for real.launch.py (with/without ovis) and
#      teleop.launch.py, using ci_launch_test.sh
#
# Any step failing causes the whole script to fail.
#
# Note: we do NOT pass USERNAME as a build arg. The Dockerfile's default
# (USERNAME=roboguard) matches docker-compose.yml's mount at
# /workspace/roboguard. Overriding USERNAME here would make the image
# WORKDIR /workspace/<your-user> while the mount stays at
# /workspace/roboguard, leaving two disjoint trees in the container.
# Only UID/GID need to match the host for correct file permissions.

set -e

git submodule sync --recursive
git submodule update --init --recursive

docker compose build \
    --build-arg USER_UID=$(id -u) \
    --build-arg USER_GID=$(id -g)

docker compose run devcontainer bash -c "
    set -e
    colcon build --symlink-install
    colcon test --packages-ignore micro_ros_setup

    source /opt/ros/humble/setup.bash
    source install/setup.bash

    ./ci-launch-test.sh real-with-ovis roboguard_bringup real.launch.py \
        use_mock_odrives:=true \
        use_mock_ovis:=true \
        with_ovis:=true \
        with_rosbag:=false

    ./ci-launch-test.sh real-no-ovis roboguard_bringup real.launch.py \
        use_mock_odrives:=true \
        use_mock_ovis:=true \
        with_ovis:=false \
        with_rosbag:=false

    ./ci-launch-test.sh teleop roboguard_bringup teleop.launch.py
"
