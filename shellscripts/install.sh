#!/bin/bash

sudo apt update
sudo apt install -y \
  lsof \
  ros-${ROS_DISTRO}-rosbridge-suite
