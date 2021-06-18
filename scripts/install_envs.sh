#!/bin/bash

# Copyright 2021 The LYDS Authors
#

if [[ `env | grep ROS_VERSION` != *"ROS_VERSION=2"* ]]; then
  echo "Make sure that you have your environment properly setup."
  echo "1. Install ROS 2 middleware before starting installation."
  echo "2. Source ROS2 environments."
  exit 1
fi

# Start as root
sudo uname

set -o errexit
set -o verbose

VERSION="conditional_build_of_builtin_types"
# test: a0585942a9e1a7255aa7c43452fd5fb2a8b869a4

userws=`pwd`
[ -d "/tmp/g2o" ] && rm -rf /tmp/g2o/

cd /tmp/

# Build and install g2o library
git clone -b ${VERSION} https://github.com/RainerKuemmerle/g2o.git
cd g2o

[ ! -d "/usr/include/eigen3" ] && ./script/install-deps-linux.sh

mkdir build && cd $_
cmake ..
make -j`nproc`
sudo make install
sudo ldconfig

cd ${userws}

# Install necessary libraries
sudo apt install -y \
        libpcap-dev \
        libyaml-cpp-dev

sudo apt install -y \
	ros-${ROS_DISTRO}-pcl-conversions \
	ros-${ROS_DISTRO}-diagnostic-updater \
	ros-${ROS_DISTRO}-angles \
	ros-${ROS_DISTRO}-slam-toolbox \
	ros-${ROS_DISTRO}-rviz2

