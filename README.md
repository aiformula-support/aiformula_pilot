# aiformula_pilot
AI Formula is a technical challenge in which robot cars drive autonomously on a race course given a mission. Through competing for speed and intelligence in a real-world environment, AI Formula will provide an opportunity for rising engineers to acquire the skills and technology necessary for next-generation mobility research. This repository is intended for remotely controlling AI-formula.
![aiformula_pilot](https://github.com/user-attachments/assets/0f4289e8-e75e-402b-9ecd-cc28dc5335ba)

Functions to be provided in this package:
* common  (Util libraries for c++ and python)
* sample_launchers  (Manage launch and shell files for each package) 
* docker
* remote_control

## Dependencies
* ROS2 Foxy (Ubuntu 20.04)

## Open Source Projects Related to AI-Formula
* [aiformula](https://github.com/aiformula-support/aiformula)
* [aiformula_common](https://github.com/aiformula-support/aiformula_common)
* [aiformula_control](https://github.com/aiformula-support/aiformula_control)

## Getting Started

### Installation

* **Local Environment:**\
Clone this repository and build:\
**Note:** This package contains submodules. If the build of a submodule fails, please refer to the original packages (linked above).
  ```bash
  mkdir -p ~/workspace/ros2_ws/src/ # create your workspace if it does not exist
  cd ~/workspace/ros2_ws/src/ #use your current ros2 workspace folder
  git clone --recursive https://github.com/aiformula-pilot/aiformula.git
  sed -i 's/tf2_geometry_msgs\.hpp/tf2_geometry_msgs.h/g' ~/workspace/
  cd ..
  colcon build --symlink-install  # build the workspace
  source ~/workspace/ros2_ws/install/local_setup.bash
  ```

* **Docker Environment:**\
To start docker:
  ```bash
  git clone https://github.com/aiformula-support/aiformula_pilot.git
  cd ./aiformula_pilot/docker
  ./docker_build_aiformula_foxy_amd.sh
  ./docker_run_aiformula_foxy_amd.sh
  ```

### Running the Example
To start all nodes of aiformula:\
**Note:** This command launches the following nodes: camera data, imu data, can data, motor controller, tf, joy.
```bash
ros2 launch sample_launchers all_pilot_nodes.launch.py
