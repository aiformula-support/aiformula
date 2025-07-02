# AI Formula
AI Formula is a technical challenge in which robot cars drive autonomously on a race course given a mission. Through competing for speed and intelligence in a real-world environment, AI Formula will provide an opportunity for rising engineers to acquire the skills and technology necessary for next-generation mobility research. This repository is the foundation of the AIFormula system.

![AIFormula_run](https://github.com/aiformula-support/aiformula/assets/113084733/87766cdd-de1e-4aef-83c6-0bfbcdcc25cb)

Functions to be provided in this package:
* common  (Util libraries for c++ and python)
* control  (Control motor)
* sample_launchers  (Manage launch and shell files for each package) 
* sensing  (Camera, Imu)
* sample_vehicle  (Include intrinsic and extrinsic params)
* sample_simulator
* docker
* perception
* planning

## Dependencies
* ROS2 Foxy (Ubuntu 20.04)
* ZED SDK
* [ZED ROS2 wrapper](https://github.com/stereolabs/zed-ros2-wrapper)
* [VectorNav](https://github.com/dawonn/vectornav)
* [ros2_socketcan](https://github.com/autowarefoundation/ros2_socketcan.git)

## Open Source Projects Related to AI-Formula
* [aiformula](https://github.com/aiformula-support/aiformula)
* [aiformula_common](https://github.com/aiformula-support/aiformula_common)
* [aiformula_pilot](https://github.com/aiformula-support/aiformula_pilot)

## Getting Started

### Installation

* **Local Environment:**\
Clone this repository and build:\
**Note:** This package contains submodules. If the build of a submodule fails, please refer to the original packages (linked above).
  ```bash
  mkdir -p ~/workspace/ros2_ws/src/ # create your workspace if it does not exist
  cd ~/workspace/ros2_ws/src/ #use your current ros2 workspace folder
  git clone --recursive https://github.com/aiformula-support/aiformula.git
  sed -i 's/tf2_geometry_msgs\.hpp/tf2_geometry_msgs.h/g' ~/workspace/ros2_ws/src/aiformula/sensing/vectornav/vectornav/src/vn_sensor_msgs.cc
  git clone https://github.com/autowarefoundation/ros2_socketcan.git
  cd ..
  colcon build --symlink-install  # build the workspace
  source ~/workspace/ros2_ws/install/local_setup.bash
  ```

* **Docker Environment:**\
To start docker:
  ```bash
  git clone https://github.com/aiformula-support/aiformula.git
  cd ./aiformula/docker
  ./docker_build_aiformula_foxy_amd.sh
  ./docker_run_aiformula_foxy_amd.sh
  ```

### Vehicle setup
To connect can, and device
```bash
cd ~/ros2_ws/src/aiformula/launchers/sample_launchers/shellscript/
./init_sensors.sh
```

### Running the Example
To start all nodes of aiformula:\
**Note:** This command launches the following nodes: camera data, imu data, can data, motor controller, tf, joy.
```bash
ros2 launch sample_launchers all_robot_nodes.launch.py
```
