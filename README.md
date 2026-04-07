# roboguard-pi
This repo contains the packages required to ron on roboguard-pi.

## Status
[![ROS2 roboguard build](https://github.com/clubcapra/roboguard_pi/actions/workflows/build.yaml/badge.svg)](https://github.com/clubcapra/roboguard_pi/actions/workflows/build.yaml)

## Installation Options

### Docker Container (Recommended)

Using a dev container ensures consistent environments across development and CI/CD pipelines.

#### Windows Setup
1. Install required software:
   - [Docker Desktop](https://www.docker.com/products/docker-desktop)
   - [Visual Studio Code](https://code.visualstudio.com/)
   - [VcXsrv](https://sourceforge.net/projects/vcxsrv/) (Xserver)
   - VSCode [Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension

2. Setup steps:
   - Clone and open the repository
   - Click the green button in VSCode's bottom left corner
   - Select "Remote-Containers: Reopen in Container"
   - Start Xserver with the **-nowgl** option

#### Linux Setup
Follow the same steps as Windows, excluding Xserver installation. Additionally:

1. Update DISPLAY environment variable in .env:
   ```bash
   echo DISPLAY=$DISPLAY
   ```

2. Configure controller node permissions:
   ```bash
   cat /dev/input/event0
   ```

**Note:** Consider [configuring Docker for non-root usage](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)

### Native/WSL Installation

Requires Ubuntu 22.04 LTS.

1. Install prerequisites:
   - [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) (use `ros-humble-desktop-full`)

2. Install dependencies (from `input_manager`):
   ```bash
   sudo apt install python3-evdev
   ```

3. Setup workspace:
   ```bash
   git clone --recurse-submodules https://github.com/clubcapra/roboguard_pi.git
   cd roboguard_pi
   sudo rosdep init
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash
   ```
### On a Raspberry Pi

#### Docker
Install docker:
```sh
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh ./get-docker.sh
```

## Running Roboguard

Note that if you are developing, you should use a different `ROS_DOMAIN_ID` than the robot's. If you do want to connect to the same domain ID, use:
```bash
ROS_DOMAIN_ID=96
```
This command needs to be run in every terminal you intend to communicate with the robot.

### Normal launch

You can launch the real-world robot using the following command:
```bash
ros2 launch roboguard_bringup real.launch.py
```

Here are the arguments that can be used:

|Argument |Default | Description |
|-|-|-|
|`use_mock_odrives`|`false`|Set this to true to use mock hardware interfaces. This allows testing out controlling the drives without having the hardware.|
|`with_ovis`|`true`|Choose whether to launch Ovis. When this is true, Ovis will appear in the URDF.|
|`use_mock_ovis`|`false`|Set this to true to use mock hardware interfaces. This allows testing out controlling Ovis without having the hardware.|
|`with_rosbag`|`true`|When this is true, rosbag will start logging and will compress the rosbag on a successful shutdown.|

Use args like so:
```bash
ros2 launch roboguard_bringup real.launch.py <arg>:=<value>
```

### Teleop

In order to control the robot, you need to run the teleop nodes:
```bash
ros2 launch roboguard_bringup teleop.launch.py
```

## Topic naming convention

Topics that start with `/rove` are topics that are handled.
Topics that start with `/rove/station` are topics that should not be listened to from the robot, they should stay on the station (control station).
topics that start with `/rove/robot` are topics that should stay on the robot.
Topics that start with `/rove/to_robot` are topics that should be sent from station to robot.
Topics that start with `/rove/to_station` are topics that should be sent from robot to station.

## Contributing

Before pushing your code, you should try to deploy it by using:
```bash
run_test
```
This will do essentially what the github action will do. It will start a docker, pull dependencies, install required packages and test the code.
This does not ensure the code is working, but it helps catch issues before making a pull request.
